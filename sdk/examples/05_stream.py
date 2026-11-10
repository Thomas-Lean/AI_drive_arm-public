#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""05 关节流式（遥操作/低层控制入口）：stream_start → stream_joints → stream_stop。

要点：SDK 后台线程每收到一帧 ArmState 发一条命令，20 Hz 跟控制器节拍，
发送节拍不用自己管；但 stream_joints 只更新目标（下一帧生效），
目标要跟臂的节拍更新：每次 stream_joints 后 wait_frame() 等下一帧，每帧恰好一个目标。
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d1t_sdk import D1Arm, D1ArmError

Q0 = [0, -40, 60, 0, 30, 0]  # 起始姿态（先确保臂在此附近且安全）


def main():
    arm = D1Arm()
    try:
        # 先走到流式起始姿态
        arm.move_joints(Q0, vel_dps=20)
        arm.wait_idle()

        print("stream_start（每帧发命令，看门狗持续喂住）...")
        arm.stream_start()
        n = 200  # 200 帧 ≈ 10 s @20 Hz
        for i in range(n):
            t = i / 20.0
            q = list(Q0)
            q[0] = Q0[0] + 5.0 * math.sin(2 * math.pi * 0.25 * t)  # J0 ±5°、0.25 Hz
            arm.stream_joints(q)
            arm.wait_frame()  # 跟臂的节拍：每帧一个目标（不用 sleep，防本机时钟漂移漏帧/跳步）
            if i % 15 == 0:
                print(f"  帧 {i}: J0 目标 {q[0]:.2f}°")
        arm.stream_stop()
        print("stream_stop，已 HOLD")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()  # 流式中 close 也会触发看门狗停臂（约 0.3 s）
    return 0


if __name__ == "__main__":
    sys.exit(main())
