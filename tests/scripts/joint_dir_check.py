#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节方向核对：逐个把 J0–J5 正转 +15°、停 2 s、再回原位，对照真臂和 MuJoCo 显示是否同向。

每个轴动之前等回车（s = 跳过该轴，q = 结束）。速度 15°/s。
用法（上位机，CYCLONEDDS_URI 用 Index 3）：python3 tests/scripts/joint_dir_check.py [轴号...]
  例：python3 tests/scripts/joint_dir_check.py 0 3     # 只核 J0、J3
同时另开终端跑 arm_viz（真臂加 --iface enp5s0 --peer 192.168.123.100）看模型。
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sdk"))

from d1t_sdk import D1Arm, D1ArmError  # noqa: E402

STEP_DEG = 15.0
VEL_DPS = 15.0


def main(axes):
    arm = D1Arm()
    try:
        q0 = list(arm.state()["q_deg"])
        print("起始 q =", [round(x, 1) for x in q0])
        for i in axes:
            s = input(f"\nJ{i} 将正转 +{STEP_DEG:.0f}° 再回来。回车开始（s 跳过，q 结束）：").strip().lower()
            if s == "q":
                break
            if s == "s":
                continue
            q = list(q0)
            q[i] += STEP_DEG
            arm.move_joints(q, vel_dps=VEL_DPS)
            print(f"   J{i} 已到 +{STEP_DEG:.0f}°：看真臂和模型往哪边转")
            time.sleep(2.0)
            arm.move_joints(q0, vel_dps=VEL_DPS)
            print(f"   J{i} 已回原位")
        print("\n完成（HOLD 在起始姿态）")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main([int(a) for a in sys.argv[1:]] or list(range(6))))
