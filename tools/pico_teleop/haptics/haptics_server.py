# -*- coding: utf-8 -*-
"""PICO 振动页面服务端（可独立跑 serve.py，也可嵌进 pico_teleop 主程序）。

- GET /cmd?after=<id>：页面每 100 ms 拉一次增量命令。**服务端先按本机时钟丢弃发出
  超过 EXPIRY_MS 的旧命令**（F85：页面进 VR 前积压的命令不再被拉走；用服务端时钟
  避开 PICO 时钟偏差）；页面端不再按自己的时钟判过期（PICO 时钟可比 ubox 快 1 s 以上）。
- POST /log：页面回传能力探测与每次 pulse 结果，追加写 jsonl（out_dir 给定时）。
- 跨源：TeleRobot 的 VR 页（8012 端口，d1t_haptics.js）跨端口访问上面两个接口，所有响应带
  `Access-Control-Allow-Origin: *`，OPTIONS 预检直接放行。
- pulse(hand, pattern)：pattern ∈ PATTERNS——grasp 单次 300 ms；empty 两短
  （150 ms ×2，间隔 0.25 s，第二条走定时线程不阻塞调用方）。

只用标准库。证书生成（Wi-Fi 接法）用 openssl。
"""
from __future__ import annotations

import json
import ssl
import subprocess
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXPIRY_MS = 500                                   # 旧命令丢弃阈值（F85）
PATTERNS = {                                       # 模式 → [(延时 s, 强度, 毫秒)]
    "grasp": [(0.0, 1.0, 300)],
    "empty": [(0.0, 1.0, 150), (0.25, 1.0, 150)],
    "ack": [(0.0, 0.6, 100)],                        # A/B 一键回位：接受 / 到位
}


CERT_DIR = Path.home() / ".cache" / "d1t_haptics"   # 固定位置：证书复用，PICO 只需信任一次


def make_cert(ip: str, out_dir: Path = CERT_DIR):
    """含 IP SAN 的自签名证书（Wi-Fi 接法用）。已有且 SAN 是同一 IP、30 天内不过期就复用，
    否则用 openssl 重签（有效期 10 年）。复用 = PICO 浏览器信任过一次后不用每次重开 8000 页。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    cert, key = out_dir / f"haptics-{ip}-cert.pem", out_dir / f"haptics-{ip}-key.pem"
    if cert.exists() and key.exists():
        ok = subprocess.run(["openssl", "x509", "-in", str(cert), "-noout", "-checkend", str(30 * 86400)],
                            capture_output=True).returncode == 0
        san = subprocess.run(["openssl", "x509", "-in", str(cert), "-noout", "-ext", "subjectAltName"],
                             capture_output=True, text=True).stdout
        if ok and f"IP Address:{ip}" in san:
            return cert, key
    subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "3650",
                    "-keyout", str(key), "-out", str(cert), "-subj", f"/CN={ip}",
                    "-addext", f"subjectAltName=IP:{ip}"],
                   check=True, capture_output=True)
    return cert, key


class _Handler(SimpleHTTPRequestHandler):
    """静态页（index.html）+ /cmd + /log。haptics 由 partial 注入。"""

    def __init__(self, *a, haptics: "HapticsServer" = None, **kw):
        self.haptics = haptics
        super().__init__(*a, directory=str(HERE), **kw)

    def do_GET(self):
        from urllib.parse import parse_qs, urlparse
        url = urlparse(self.path)
        if url.path != "/cmd":
            return super().do_GET()
        after = int(parse_qs(url.query).get("after", ["0"])[0] or 0)
        with self.haptics._lock:
            now_ms = int(time.time() * 1000)
            out = [c for c in self.haptics._cmds
                   if c["id"] > after and now_ms - c["sent_ms"] <= EXPIRY_MS]
            expired = [c["id"] for c in self.haptics._cmds
                       if c["id"] > after and now_ms - c["sent_ms"] > EXPIRY_MS]
        if expired:
            self.haptics.log_line({"kind": "server_drop", "ids": expired})
        body = json.dumps(out).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path != "/log":
            self.send_error(404)
            return
        msg = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        msg["server_rx_ms"] = int(time.time() * 1000)
        self.haptics.log_line(msg)
        kind = msg.get("kind")
        if kind == "caps":
            print(f"手柄能力 {msg['hand']}: {json.dumps(msg['caps'], ensure_ascii=False)}", flush=True)
        elif kind == "pulse":
            res = (f"{msg.get('method')} → {msg.get('result')}" if msg.get("method")
                   else f"失败：{msg.get('error')}")
            print(f"  页面执行 {msg.get('via')} #{msg.get('id', '-')} {msg.get('hand')}: {res}",
                  flush=True)
        elif kind == "drop":
            print(f"  页面丢弃 #{msg.get('id', '-')} {msg.get('hand')} "
                  f"（{msg.get('age_ms')} ms > 500）", flush=True)
        elif kind in ("start", "end"):
            print(f"页面 {kind}", flush=True)
        self.send_response(204)
        self.end_headers()

    def do_OPTIONS(self):
        # CORS 预检：跨端口 POST /log 带 application/json 时浏览器先发 OPTIONS，一律放行。
        self.send_response(204)
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def end_headers(self):
        # TeleRobot 的 VR 页跨端口拉 /cmd、回传 /log（接入 TeleRobot 的振动脚本）：所有响应放行跨源。
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


class HapticsServer:
    """服务 index.html、下发振动命令、记录页面回传。线程安全；嵌入 teleop 与独立跑共用。"""

    def __init__(self, port: int = 8000, https_ip: str | None = None,
                 out_dir: Path | None = None):
        self.port = int(port)
        self.https_ip = https_ip
        self._cmds: list[dict] = []
        self._lock = threading.Lock()
        self._log_path = None
        self._timers: list[threading.Timer] = []
        self._stopped = False
        if out_dir is not None:
            out_dir.mkdir(parents=True, exist_ok=True)
            self._log_path = out_dir / time.strftime("pico-haptics-%H%M%S.jsonl")
        host = "0.0.0.0" if https_ip else "127.0.0.1"
        self._server = ThreadingHTTPServer(
            (host, self.port), lambda *a, **kw: _Handler(*a, haptics=self, **kw))
        if https_ip:
            cert, key = make_cert(https_ip)
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(cert, key)
            self._server.socket = ctx.wrap_socket(self._server.socket, server_side=True)
            self.url = f"https://{https_ip}:{self.port}"
        else:
            self.url = f"http://127.0.0.1:{self.port}"

    # ---- 命令 ----
    def push(self, hand: str, intensity: float = 1.0, ms: int = 300) -> int:
        """发一条振动命令，返回命令 id。hand ∈ right/left/both。"""
        with self._lock:
            cmd = {"id": len(self._cmds) + 1, "hand": hand, "intensity": float(intensity),
                   "ms": int(ms), "sent_ms": int(time.time() * 1000)}
            self._cmds.append(cmd)
        self.log_line(dict(kind="cmd", **cmd))
        print(f"→ 振动命令 #{cmd['id']} {hand} 强度 {intensity:g} {ms} ms", flush=True)
        return cmd["id"]

    def pulse(self, hand: str, pattern: str) -> list[int]:
        """按模式发（grasp 单次 / empty 两短）。第二拍起用定时线程，不阻塞控制循环。"""
        ids = []
        for delay_s, intensity, ms in PATTERNS[pattern]:
            if delay_s <= 0:
                ids.append(self.push(hand, intensity, ms))
            else:
                t = threading.Timer(delay_s, self.push, (hand, intensity, ms))
                t.daemon = True
                self._timers.append(t)
                t.start()
        return ids

    # ---- 生命周期 ----
    def start(self) -> str:
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        if self._log_path is not None:
            print(f"振动页面 {self.url}（记录 → {self._log_path}）", flush=True)
        else:
            print(f"振动页面 {self.url}", flush=True)
        return self.url

    def log_line(self, obj: dict) -> None:
        if self._log_path is None:
            return
        with self._lock, self._log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(obj, ensure_ascii=False) + "\n")

    def stop(self) -> None:
        if self._stopped:
            return
        self._stopped = True
        for t in self._timers:
            t.cancel()
        self._server.shutdown()
