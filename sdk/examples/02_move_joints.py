#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""02 关节运动：move_joints → wait_idle → home（回全零位）。

⚠ 安全：执行前确认臂周围无人无障碍、摆位离限位远；急停 Ctrl-C 后看门狗约 0.3 s 停臂。
home 走到 URDF 全零（舵机全零，臂前伸）——与原厂 HOME 姿势不同，确认不碰桌面/自身。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d1t_sdk import D1Arm, D1ArmError

TARGET = [0, -40, 60, 0, 30, 0]  # URDF 角（度），同事 README 同款示例


def main():
    arm = D1Arm()
    try:
        print("当前：", [round(q, 1) for q in arm.state()["q_deg"]], "°")
        print(f"move_joints({TARGET})，vel 30°/s ...")
        arm.move_joints(TARGET, vel_dps=30)
        arm.wait_idle()
        print("到位：", [round(q, 2) for q in arm.state()["q_deg"]], "°")
        print("home(4.0) ...")
        arm.home(4.0, async_=False)
        print("到位：", [round(q, 2) for q in arm.state()["q_deg"]], "°")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
