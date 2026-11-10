#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""大幅直线验收：J4=45°、其余 0 的姿态（避开 J4=0 腕部奇异）出发，
以 base_link 为参考沿 X ±10 cm、Y ±15 cm 各走一个来回，只用 move_l。

起点 TCP ≈ (309, 0, 316) mm；离线核过：全程 J4 45–57°，Y ±15 cm 时 J0 ±37°，均远离限位。
J4=90° 时实机 J4 保持抖动触发 HOLD_FAIL，故改 45°。
用法（上位机，已设 CYCLONEDDS_URI）：python3 tests/scripts/sdk_xy_line.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sdk"))

from d1t_sdk import D1Arm, D1ArmError  # noqa: E402

START_Q = [0, 0, 0, 0, 45, 0]
VEL = 0.06  # m/s 平均末端速度
LEGS = [("+X 10cm", 0.10, 0.0), ("-X 10cm", -0.10, 0.0),
        ("+Y 15cm", 0.0, 0.15), ("-Y 15cm", 0.0, -0.15)]


def mm(p):
    return [round(v * 1000) for v in p]


def main():
    arm = D1Arm()
    try:
        print("走到起始姿态", START_Q, "...")
        arm.move_joints(START_Q, vel_dps=20)
        tcp = arm.state()["tcp"]
        p0, rpy = list(tcp["pos"]), tcp["rpy"]
        print(f"起点 TCP {mm(p0)} mm\n")
        print(f"{'段':<10}{'目标 mm':<22}{'实到 mm':<22}误差 mm (x,y,z)")
        for name, dx, dy in LEGS:
            input(f"回车执行 {name} 并返回 ...")
            for label, tgt in ((name, [p0[0] + dx, p0[1] + dy, p0[2]]), ("  回起点", p0)):
                arm.move_l(tgt, rpy=rpy, vel=VEL)
                got = arm.state()["tcp"]["pos"]
                err = [round((g - t) * 1000) for g, t in zip(got, tgt)]
                print(f"{label:<10}{str(mm(tgt)):<22}{str(mm(got)):<22}{err}")
        print("\n完成（停在起点，HOLD）")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
