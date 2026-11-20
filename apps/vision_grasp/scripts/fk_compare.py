#!/usr/bin/env python3
"""FK 与 d1arm_sdk 金标逐项比对（离线可重跑，不需要控制器/相机）。

对照值取自原工程 d1arm_sdk（分支 grasp_back、提交 f95f6c9）tests/test_main.cpp
的 test_fk_golden()：其数值由 numpy 独立从 d1_description URDF 算出。本脚本用
本仓库 d1t_sdk 的运动学（适配层 call('fk') → D1Arm.fk → 同一 Kinematics）复算
并逐项比对，验证 TCP 定义（Link6 法兰 × 工具偏移 xyz(-5.62,0,70.6) mm、
rpy(0,-90°,0)，即 URDF 两夹爪指 origin 中点）、单位（米/弧度）与 rpy 约定
（ZYX）两边一致——不一致就得在适配层换算，这里是"不用换算"的证据。

用法：python3 apps/vision_grasp/scripts/fk_compare.py   # 退出码 0 = 全过
"""
from pathlib import Path
import sys

import numpy as np

_SDK_DIR = Path(__file__).resolve().parents[3] / 'sdk'
if _SDK_DIR.is_dir() and str(_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(_SDK_DIR))
from d1t_sdk import Kinematics  # noqa: E402

# d1arm_sdk tests/test_main.cpp::test_fk_golden —— 容差沿用其原始断言
GOLDEN = [
    ('q=0', [0.0] * 6,
     dict(pos=[0.350980, -0.001278, 0.425542], rpy=None), 1e-6),
    ('q=10,20,-30,40,-50,60', [10.0, 20.0, -30.0, 40.0, -50.0, 60.0],
     dict(pos=[0.366329, -0.010289, 0.547792],
          rpy=[0.016232, -0.760837, -0.573056]), 1e-5),
    ('q=90,45,90,-60,-30,30', [90.0, 45.0, 90.0, -60.0, -30.0, 30.0],
     dict(pos=[-0.069288, 0.014250, 0.011524], rpy=None), 1e-6),
]


def main():
    kin = Kinematics()
    failures = 0
    for label, q_deg, expect, tol in GOLDEN:
        t = kin.fk(np.deg2rad(np.asarray(q_deg, float)))
        pos_err = float(np.max(np.abs(t[:3, 3] - np.asarray(expect['pos']))))
        line = (f'{label:24s} tcp=[{t[0, 3]:+.6f},{t[1, 3]:+.6f},{t[2, 3]:+.6f}] '
                f'pos_err={pos_err:.2e}')
        ok = pos_err <= tol
        if expect['rpy'] is not None:
            # Kinematics 内部 to_rpy；此处直接复算同式（ZYX 欧拉）
            from d1t_sdk._kinematics import to_rpy
            rpy = np.asarray(to_rpy(t))
            rpy_err = float(np.max(np.abs(rpy - np.asarray(expect['rpy']))))
            line += f' rpy_err={rpy_err:.2e}'
            ok = ok and rpy_err <= 1e-5
        print(f'{"PASS" if ok else "FAIL"}  {line}')
        failures += 0 if ok else 1
    # quat 与 rpy 描述同一旋转（上层 pose_matrix 只用 quat，交叉验证一次）
    t = kin.fk(np.deg2rad(np.array([10.0, 20.0, -30.0, 40.0, -50.0, 60.0])))
    from d1t_sdk._kinematics import to_rpy, from_pose_dict
    back = from_pose_dict(t[:3, 3], rpy=to_rpy(t))
    rot_err = float(np.max(np.abs(back[:3, :3] - t[:3, :3])))
    ok = rot_err <= 1e-9
    print(f'{"PASS" if ok else "FAIL"}  quat<->rpy 往返          rot_err={rot_err:.2e}')
    failures += 0 if ok else 1
    print(f'== {len(GOLDEN) + 1} 项检查，失败 {failures} ==')
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
