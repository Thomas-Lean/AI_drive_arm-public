#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""01 读状态 + 在线检查：state()/ping() 各打印一次，再连续看 5 秒。"""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sdk/

from d1t_sdk import D1Arm, D1ArmError


def main():
    arm = D1Arm()
    try:
        st = arm.state()
        print(json.dumps(st, ensure_ascii=False, indent=1))
        print("online(7 舵机):", arm.ping())
        print("--- 5 秒状态（1 Hz）---")
        for _ in range(5):
            time.sleep(1.0)
            st = arm.state()
            print(f"mode={st['mode']:<9} q_deg={[round(q, 1) for q in st['q_deg']]} "
                  f"gripper={st['gripper_mm']:.1f}mm J6温度={st['temp_c'][6]:.1f}℃ "
                  f"fault=0x{st['fault']:x}")
    except D1ArmError as e:
        print(f"NG：{e}")
        print("检查：臂上控制器起了吗？CYCLONEDDS_URI 设了吗？（见 sdk/实机验收清单.md）")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
