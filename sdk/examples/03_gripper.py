#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""03 夹爪开合：gripper(mm) 用标定表（arm_model/d1t.json）换算 J6 角，20°/s 走。

mm = 金属指间距（含两侧垫约 2 mm；垫面间距 = 金属 − 4）。
可达域 4–70 mm；夹瓶常用 45 mm 附近（空夹停住约 29.2°↔45 mm）。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d1t_sdk import D1Arm, D1ArmError


def main():
    arm = D1Arm()
    try:
        for mm in (70.0, 45.0, 20.0, 70.0):  # 全开 → 半合 → 近闭 → 全开
            print(f"gripper({mm} mm) ...")
            r = arm.gripper(mm)
            got = arm.state()["gripper_mm"]
            print(f"  到位：实测 {got:.1f} mm，用时 {r['duration_s']:.1f}s")
        print("完成")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
