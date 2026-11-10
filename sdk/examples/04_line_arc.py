#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""04 直线 + 圆弧：move_l（末端走直线）→ move_c（三点定圆弧）。

⚠ 先把臂摆到工作空间中部的安全姿态（例：先用 02 移到 [0,-40,60,0,30,0]）。
目标位姿 = 位置(m) + rpy(弧度)；本例只改位置、姿态保持 rpy。
弧半径 5 cm；同类构型下 π 半圆弧对侧可能够不着（SDK 会报「路径 IK 失败」，不是 bug）。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d1t_sdk import D1Arm, D1ArmError


def main():
    arm = D1Arm()
    try:
        tcp = arm.state()["tcp"]
        pos, rpy = tcp["pos"], tcp["rpy"]
        print(f"起点 pos={[round(v, 3) for v in pos]} m")

        # 直线：往 +x 8 cm → 回来
        target = [pos[0] + 0.08, pos[1], pos[2]]
        print(f"move_l → {[round(v, 3) for v in target]}，vel 0.08 m/s ...")
        arm.move_l(target, rpy=rpy, vel=0.08)
        arm.wait_idle()
        print("  到位：", [round(v, 3) for v in arm.state()["tcp"]["pos"]], "m")
        arm.move_l(pos, rpy=rpy, vel=0.08)
        arm.wait_idle()

        # 圆弧：当前 → 经上 4 cm 点 → 回到起点旁（三点定圆，半径 5 cm）
        via = [pos[0] - 0.025, pos[1], pos[2] + 0.04]
        end = [pos[0] - 0.05, pos[1], pos[2]]
        print(f"move_c 经 {[round(v, 3) for v in via]} → {[round(v, 3) for v in end]} ...")
        arm.move_c(end, via, rpy=rpy, via_rpy=rpy, vel=0.06)
        arm.wait_idle()
        print("  到位：", [round(v, 3) for v in arm.state()["tcp"]["pos"]], "m")
        print("完成")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
