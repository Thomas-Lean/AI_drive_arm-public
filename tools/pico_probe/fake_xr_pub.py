#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_xr_pub — 假 PICO 遥测发布器（没有 PICO 时离线验证 pico_probe 用）。

按 TeleRobot 查看器的发布格式（ZMQ PUB，topic "xr"，JSON schema telerobot.xr.v1，
与 telerobot/teleop.py TelemetryPublisher.message() 同构）发一只手柄：位置画圆，握把/扳机
按时间表变化。默认与查看器同一端口，二者不能同时运行。

默认时间表（12 s 一轮，循环）：
  0–2 s    握把 0      脱开
  2–8 s    握把 0.95   接合；手柄在 XR 水平面上画半径 10 cm 的圆（目标 TCP 跟着画圆）
  8–10 s   握把 0.5    脱开（圆继续画，目标应保持不动）
  10–12 s  握把 0      脱开
  扳机：0→1→0 三角波，周期 6 s（覆盖全行程两端）

用法：
    python3 fake_xr_pub.py                                 # 默认 tcp://127.0.0.1:55556，60 Hz
    python3 fake_xr_pub.py --restart-every 20              # 每 20 s 换 epoch（模拟查看器重启）
    python3 fake_xr_pub.py --stall-every 15 --stall-s 1.5  # 每 15 s 停发 1.5 s（模拟断流）
"""
from __future__ import annotations

import argparse
import json
import math
import time

import zmq

TOPIC = b"xr"
SCHEMA = "telerobot.xr.v1"

CIRCLE_CENTER = (0.20, 1.10, -0.35)  # XR 系（+X 右 +Y 上 +Z 后，米）：右前方腰高
CIRCLE_RADIUS = 0.10
CIRCLE_PERIOD_S = 6.0


def pose_matrix(position, rotation=None):
    """位置 + 3×3 旋转 → 4×4 嵌套列表（行优先存放的普通矩阵，平移在第 4 列）。

    查看器把浏览器的 16 值列主序数组 reshape(4,4,order="F") 后 tolist() 发出，即本格式。"""
    r = rotation if rotation is not None else [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return [[float(r[i][0]), float(r[i][1]), float(r[i][2]), float(position[i])] for i in range(3)] \
        + [[0.0, 0.0, 0.0, 1.0]]


def _empty_sample():
    return {"pose": None, "position_m": None, "buttons": {}, "sequence": 0, "fresh": False}


def make_message(epoch_ns, side, position, grip, trigger, sequence, fresh=True, rotation=None):
    """一帧遥测消息：只有 side 这只手柄有数据，其余 head/另一只手为未收到（fresh=False）。"""
    samples = {name: _empty_sample() for name in ("head", "left", "right")}
    samples[side] = {
        "pose": pose_matrix(position, rotation),
        "position_m": [float(v) for v in position],
        "buttons": {"trigger": trigger > 0.5, "squeeze": grip > 0.5,
                    "triggerValue": float(trigger), "squeezeValue": float(grip),
                    "aButton": False, "bButton": False},
        "sequence": int(sequence),
        "fresh": bool(fresh),
    }
    return {"schema": SCHEMA, "epoch_ns": int(epoch_ns), "published_unix_ns": time.time_ns(),
            "mode": "controller", "samples": samples}


def scripted(t, period_s=12.0):
    """默认时间表：t（秒）→ (位置, 握把, 扳机)。"""
    phase = t % period_s
    if phase < 2.0:
        grip = 0.0
    elif phase < 8.0:
        grip = 0.95
    elif phase < 10.0:
        grip = 0.5
    else:
        grip = 0.0
    a = 2.0 * math.pi * t / CIRCLE_PERIOD_S
    cx, cy, cz = CIRCLE_CENTER
    position = (cx + CIRCLE_RADIUS * math.cos(a), cy, cz + CIRCLE_RADIUS * math.sin(a))
    tri = (t % 6.0) / 3.0
    trigger = tri if tri <= 1.0 else 2.0 - tri
    return position, grip, trigger


class FakeXrPublisher:
    """绑定 PUB 端口发遥测帧。endpoint 可用 tcp://127.0.0.1:* 取随机端口（见 .endpoint）。"""

    def __init__(self, endpoint="tcp://127.0.0.1:55556", side="right"):
        self.side = side
        self.epoch_ns = time.time_ns()
        self.sequence = 0
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 4)  # 同查看器：慢消费者不积压
        self._socket.bind(endpoint)
        self.endpoint = self._socket.getsockopt_string(zmq.LAST_ENDPOINT)

    def new_epoch(self):
        """换 epoch_ns（等同查看器重启），sequence 从头计。"""
        self.epoch_ns = max(time.time_ns(), self.epoch_ns + 1)
        self.sequence = 0

    def send(self, position, grip, trigger, fresh=True, rotation=None):
        if fresh:
            self.sequence += 1
        msg = make_message(self.epoch_ns, self.side, position, grip, trigger, self.sequence,
                           fresh=fresh, rotation=rotation)
        self._socket.send_multipart([TOPIC, json.dumps(msg, allow_nan=False).encode()])

    def close(self):
        self._socket.close(linger=0)
        self._context.term()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:55556")
    parser.add_argument("--side", default="right", choices=("left", "right"))
    parser.add_argument("--rate", type=float, default=60.0, help="发布频率 Hz")
    parser.add_argument("--duration", type=float, default=0.0, help="秒，0 = 一直发")
    parser.add_argument("--restart-every", type=float, default=0.0, help="每 N 秒换 epoch，0 = 不换")
    parser.add_argument("--stall-every", type=float, default=0.0, help="每 N 秒停发一次，0 = 不停")
    parser.add_argument("--stall-s", type=float, default=1.5, help="每次停发时长（秒）")
    args = parser.parse_args(argv)

    pub = FakeXrPublisher(args.endpoint, args.side)
    print(f"fake_xr_pub：{pub.endpoint}  {args.side} 手柄  {args.rate:g} Hz  "
          f"epoch {pub.epoch_ns}。Ctrl+C 退出。", flush=True)
    t0 = time.monotonic()
    next_restart = args.restart_every or math.inf
    next_stall = args.stall_every or math.inf
    period = 1.0 / args.rate
    try:
        while True:
            t = time.monotonic() - t0
            if args.duration and t >= args.duration:
                break
            if t >= next_restart:
                pub.new_epoch()
                next_restart += args.restart_every
                print(f"[{t:6.1f} s] 换 epoch → {pub.epoch_ns}", flush=True)
            if t >= next_stall:
                print(f"[{t:6.1f} s] 停发 {args.stall_s:g} s", flush=True)
                time.sleep(args.stall_s)
                next_stall += args.stall_every
                continue
            position, grip, trigger = scripted(t)
            pub.send(position, grip, trigger)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        pub.close()


if __name__ == "__main__":
    main()
