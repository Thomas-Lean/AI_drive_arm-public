#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""replay.py — P1.3 开环重复抓取（TC-012）：示教三个点，循环回放「待机 → 预抓 → 抓取 → 合拢 → 张开 → 预抓 → 待机」。

用法：
  python3 replay.py teach standby     # 读当前 7 轴角度存为待机点（写 points.json）
  python3 replay.py teach pre         # 预抓点：夹爪张开、在瓶子正后方几厘米
  python3 replay.py teach grasp       # 抓取点：夹爪张开、正好套住瓶子
  python3 replay.py run --cycles 10   # 回放；每圈回到待机点后 HOLD，按回车继续下一圈

要点：
- 每收到一帧 ArmState 发一条 STREAM（同 send.py --sync-state，F73）。
- 段内各轴线性插值，时长由最慢轴决定：J0–J5 按 --speed（默认 30°/s，URDF 上限一半），J6 按 20°/s（F60）。
- 示教点里的 J6 不用：移动时 J6 保持张开 64.5°，合拢段走到 --close（默认 29.0°）。
- 圈间等回车时控制器处于 HOLD（不在 STREAMING，看门狗不触发）。
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]  # 仓库根
for _p in (str(_REPO / "sdk"), str(_REPO / "tools" / "arm_client")):
    if _p not in sys.path:
        sys.path.insert(0, _p)  # 协议常量从 d1t_sdk 导入；Sender 在 tools/arm_client/send.py

from send import Sender
from d1t_sdk._common import MODE_HOLD, MODE_STREAM

OPEN, J6_SPEED = 64.5, 20.0
HZ = 20.0


def parse_args():
    p = argparse.ArgumentParser(description="P1.3 开环重复抓取")
    p.add_argument("cmd", choices=["teach", "run"])
    p.add_argument("name", nargs="?", choices=["standby", "pre", "grasp"])
    p.add_argument("--points", default="points.json")
    p.add_argument("--cycles", type=int, default=10)
    p.add_argument("--speed", type=float, default=30.0, help="J0–J5 段内最大角速度 °/s（默认 30）")
    p.add_argument("--close", type=float, default=29.0, help="合拢目标角（默认 29.0）")
    p.add_argument("--dwell", type=float, default=1.0, help="合拢后停留 s（默认 1）")
    p.add_argument("--grip-power", type=int, default=0)
    p.add_argument("--no-wait", action="store_true", help="圈间不等回车")
    return p.parse_args()


def teach(args):
    s = Sender()
    st = s.wait_first_state(5)
    if st is None:
        print("NG：没收到 ArmState")
        return 1
    path = Path(args.points)
    pts = json.loads(path.read_text()) if path.exists() else {}
    pts[args.name] = [round(float(x), 1) for x in st.q]
    path.write_text(json.dumps(pts, indent=1))
    print(f"{args.name} = {pts[args.name]} → {path}")
    return 0


class Player:
    def __init__(self, s, args):
        self.s, self.args, self.last_seq, self.q = s, args, None, None

    def tick(self, q):
        """等下一帧新 ArmState，然后发一条 STREAM。"""
        while True:
            st = self.s.latest_state()
            if st is not None and st.seq != self.last_seq:
                self.last_seq = st.seq
                break
            time.sleep(0.001)
        self.s.send_cmd(MODE_STREAM, q=q, dt_ms=int(1000 / HZ), grip_power=self.args.grip_power)
        self.q = list(q)

    def move(self, target):
        a, b = self.q, target
        t = max([abs(b[i] - a[i]) / self.args.speed for i in range(6)] + [abs(b[6] - a[6]) / J6_SPEED])
        n = max(1, math.ceil(t * HZ))
        for k in range(1, n + 1):
            self.tick([a[i] + (b[i] - a[i]) * k / n for i in range(7)])

    def hold_still(self, sec):
        for _ in range(max(1, int(sec * HZ))):
            self.tick(self.q)


def run(args):
    path = Path(args.points)
    pts = json.loads(path.read_text()) if path.exists() else {}
    miss = [k for k in ("standby", "pre", "grasp") if k not in pts]
    if miss:
        print(f"NG：{path} 缺示教点 {miss}，先 teach")
        return 1
    with_j6 = lambda q, j6: list(q[:6]) + [j6]
    standby, pre, grasp = (with_j6(pts[k], OPEN) for k in ("standby", "pre", "grasp"))
    closed = with_j6(pts["grasp"], args.close)

    s = Sender()
    st = s.wait_first_state(5)
    if st is None:
        print("NG：没收到 ArmState")
        return 1
    p = Player(s, args)
    for c in range(1, args.cycles + 1):
        if s.send_until(MODE_HOLD, {3}, 5, q=list(st.q)) is None:
            print("NG：HOLD 未生效")
            return 1
        if not args.no_wait:
            input(f"第 {c}/{args.cycles} 圈：看好瓶子，按回车开始（Ctrl-C 退出）")
        st = s.wait_first_state(5)
        p.q, p.last_seq = list(st.q), None
        p.move(standby)
        p.move(pre)
        p.move(grasp)
        p.move(closed)
        p.hold_still(args.dwell)
        p.move(grasp)
        p.move(pre)
        p.move(standby)
        print(f"第 {c} 圈完成")
        st = s.latest_state() or st
    s.send_until(MODE_HOLD, {3}, 5, q=list(st.q))
    print("全部完成，已 HOLD")
    return 0


def main():
    args = parse_args()
    try:
        if args.cmd == "teach":
            if not args.name:
                print("teach 需要点名：standby / pre / grasp")
                return 1
            return teach(args)
        return run(args)
    except KeyboardInterrupt:
        print("\n中断（控制器看门狗约 0.3 s 后停臂）")
        return 130


if __name__ == "__main__":
    sys.exit(main())
