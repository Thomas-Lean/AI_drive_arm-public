#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PICO 手柄振动页面（独立测试用；遥操作时由 pico_teleop 内嵌起同一个服务）。

页面每 100 ms 拉一次 /cmd；服务端与页面都丢弃超过 0.5 s 的旧命令（F85）。
命令：--auto 3 = 每 3 s 依次振 右 / 左 / 双手（不用敲键盘）；运行中在终端输入
  r / l / b [强度 0–1] [毫秒]  例：r 0.6 400      q 退出

两种接法（任选）：
  A. USB：python3 serve.py；本机 adb reverse tcp:8000 tcp:8000；PICO 浏览器开 http://localhost:8000
  B. Wi-Fi：python3 serve.py --https-ip <本机局域网 IP>；PICO 开 https://<IP>:8000 并信任自签名证书
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from haptics_server import HapticsServer  # noqa: E402

REPO = Path(__file__).resolve().parents[3]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--https-ip", help="Wi-Fi 接法：本机局域网 IP（生成证书并监听 0.0.0.0）")
    ap.add_argument("--auto", type=float, default=0.0,
                    help="每 N 秒自动发一条（右/左/双手轮换），0 = 只手动")
    ap.add_argument("--out", type=Path,
                    default=REPO / "tests/runs" / (time.strftime("%Y-%m-%d") + "-Pico振动"))
    args = ap.parse_args()

    srv = HapticsServer(port=args.port, https_ip=args.https_ip, out_dir=args.out)
    srv.start()
    print(f"记录写入 {srv._log_path}", flush=True)

    if args.auto > 0:
        import threading

        def auto():
            hands = ["right", "left", "both"]
            k = 0
            while True:
                time.sleep(args.auto)
                srv.push(hands[k % 3])
                k += 1
        threading.Thread(target=auto, daemon=True).start()
    try:
        for line in sys.stdin:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "q":
                break
            hand = {"r": "right", "l": "left", "b": "both"}.get(parts[0])
            if hand is None:
                print("输入 r / l / b [强度] [毫秒]，q 退出", flush=True)
                continue
            srv.push(hand, float(parts[1]) if len(parts) > 1 else 1.0,
                     int(parts[2]) if len(parts) > 2 else 300)
        else:
            while True:            # stdin 关闭（后台运行）时靠 --auto 继续
                time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()


if __name__ == "__main__":
    main()
