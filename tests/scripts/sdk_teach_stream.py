#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""示教两点 + 关节流式往返（stream_run 按帧回调）：J0–J5 按余弦曲线在 A、B 间来回，看各轴流式平不平滑。

流程：失力摆 A → 锁 → 失力摆 B → 锁 → 关节运动回 A → 每轮先输入「一个来回几秒」，
      stream_run 每帧取一个目标走 A→B→A（q = A + (B−A)·(1−cos 2πt/T)/2）。
周期太短会让某轴超过控制器限速，脚本自动拉长到不超限的最短周期并提示。
⚠ 失力 = 臂会掉：回车前手先托住臂。示教点避开手腕水平伸出（J4≈90°）。
用法（上位机，已设 CYCLONEDDS_URI）：python3 tests/scripts/sdk_teach_stream.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sdk"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from d1t_sdk import D1Arm, D1ArmError  # noqa: E402
from d1t_sdk._common import URDF_VMAX_DPS  # noqa: E402
from sdk_teach_line import r1, teach_point  # noqa: E402

DT = 0.05          # 20 Hz，与控制器节拍一致
SPEED_MARGIN = 0.9  # 峰值速度不超过限速的 90%


def min_period(a, b):
    """余弦往返峰值速度 = π·|Δ|/T，按各轴限速求最短周期 T。"""
    return max(math.pi * abs(qb - qa) / (SPEED_MARGIN * v)
               for qa, qb, v in zip(a, b, URDF_VMAX_DPS))


def ask_period(last, t_min):
    while True:
        s = input(f"\n一个来回几秒（≥ {t_min:.1f}，回车 = {last:.1f}，q = 结束）：").strip().lower()
        if s == "q":
            return None
        if s == "":
            return max(last, t_min)
        try:
            t = float(s)
        except ValueError:
            print("   不是数字，重输")
            continue
        if t < t_min:
            print(f"   {t} 秒会让某轴超限速，按 {t_min:.1f} 秒走")
            t = t_min
        return t


def stream_round(arm, a, b, period):
    n = int(round(period / DT))
    peak = [round(math.pi * abs(qb - qa) / period, 1) for qa, qb in zip(a, b)]
    print(f"   各轴峰值速度 {peak} °/s，共 {n} 帧")

    def target(k, st):  # 第 k 帧的目标；节奏由 SDK 跟臂走（stream_run）
        if k > n:
            return None
        s = (1 - math.cos(2 * math.pi * k / n)) / 2
        return [qa + (qb - qa) * s for qa, qb in zip(a, b)]

    r = arm.stream_run(target)
    if r["late_frames"]:
        print(f"   ⚠ {r['late_frames']} 帧回调超过 50 ms")
    got = arm.state()["q_deg"]
    print(f"   回到 A，误差 {[round(g - x, 1) for g, x in zip(got, a)]} °")


def main():
    arm = D1Arm()
    try:
        a = teach_point(arm, "A 点")["q_deg"]
        b = teach_point(arm, "B 点")["q_deg"]
        print(f"\n各轴 A→B 转角 {r1([qb - qa for qa, qb in zip(a, b)])} °")
        t_min = min_period(a, b)
        print("回 A（关节运动 20°/s）...")
        arm.move_joints(a, vel_dps=20)
        period, n = max(6.0, t_min), 0
        while True:
            period = ask_period(period, t_min)
            if period is None:
                break
            n += 1
            print(f"第 {n} 轮，一个来回 {period:.1f} 秒：")
            stream_round(arm, a, b, period)
        print("\n完成（HOLD 在 A）")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
