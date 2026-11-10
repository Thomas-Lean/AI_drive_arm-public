#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""总线缺包实时监视（只读）：每秒打印一行「本秒缺包帧数 + 哪些轴没回」，用来边摇线边找接触不良的位置。

valid_mask 第 i 位 = Ji 本帧回包有效。J0–J2 与 J3–J6 是两条舵机线：缺 J3–J6 整段 = 第二条线这一帧整段没回；只缺 J6 = 该线末端夹爪舵机没回。
只订阅 arm_state，不发任何命令。
用法（上位机，CYCLONEDDS_URI 的 ParticipantIndex 用 5）：python3 tests/scripts/bus_loss_watch.py   Ctrl-C 退出
"""
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sdk"))

from cyclonedds.sub import DataReader  # noqa: E402

from d1t_sdk._common import dds_qos, make_participant, make_state_topic  # noqa: E402


def missing_axes(mask):
    return "".join(str(i) for i in range(7) if not mask & (1 << i))


def main():
    dp = make_participant()
    rd = DataReader(dp, make_state_topic(dp), dds_qos())
    frames, miss, total_frames, total_miss = 0, Counter(), 0, 0
    t0 = t_last = time.monotonic()
    while True:
        for s in rd.take():
            frames += 1
            if s.valid_mask != 0x7F:
                miss["J" + missing_axes(s.valid_mask)] += 1
        now = time.monotonic()
        if now - t_last >= 1.0:
            n = sum(miss.values())
            total_frames += frames
            total_miss += n
            bar = "#" * n
            detail = " ".join(f"缺{k}×{v}" for k, v in miss.items())
            print(f"{now - t0:6.0f}s  帧 {frames:2d}  缺包 {n:2d} {bar:<10} {detail}"
                  f"   累计 {total_miss}/{total_frames}", flush=True)
            frames, miss, t_last = 0, Counter(), now
        time.sleep(0.02)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
