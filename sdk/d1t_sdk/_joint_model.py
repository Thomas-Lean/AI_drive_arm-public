# -*- coding: utf-8 -*-
"""d1t_sdk 关节模型：URDF 角 ↔ 舵机角换算、限位预检、夹爪标定表。

角度约定（任务书 §「必须处理的差异」#1/#2/#4）：
- 对外（用户）一律用同事 d1arm_sdk 的关节角约定（URDF 方向）：
    joint_deg = sign * (servo_deg - offset_deg)      # servo→joint
    servo_deg = sign * joint_deg + offset_deg        # joint→servo
  calib.json 下标 0/3 sign=-1，即我们的 J0/J3；offset 全 0（「零偏待米尺复核」）。
- 限位以控制器 F7 为准（舵机角空间）；URDF 空间限位由 sign 换算，仅用于 SDK 规划
  （IK/预检），控制器收到舵机角后会再查一遍。
- 夹爪 mm 用实测标定表（金属指间距，arm_model/d1t.json），不用 d1arm_sdk 的 0°↔0mm/64.5°↔65mm 线性。
"""
import math

import numpy as np

from ._common import D1ArmError, GRIPPER_TABLE, JOINT_CALIB, SERVO_LIMITS


def servo_to_joint(joint_id: int, servo_deg: float) -> float:
    sign, offset = JOINT_CALIB[joint_id]
    return sign * (servo_deg - offset)


def joint_to_servo(joint_id: int, joint_deg: float) -> float:
    sign, offset = JOINT_CALIB[joint_id]
    return sign * joint_deg + offset


def urdf_limits() -> tuple:
    """六关节 URDF 角空间限位（由 F7 舵机限位经 sign 换算，区间翻转）。"""
    out = []
    for i in range(6):
        smin, smax = SERVO_LIMITS[i]
        sign, _ = JOINT_CALIB[i]
        lo, hi = sign * smin, sign * smax
        out.append((min(lo, hi), max(lo, hi)))
    return tuple(out)


def check_q_urdf(q_deg) -> None:
    """规划前预检（学同事的报错格式，带关节号；我们 0 起编号）。越界抛 D1ArmError。"""
    lims = urdf_limits()
    for i, q in enumerate(q_deg):
        lo, hi = lims[i]
        if not (lo <= q <= hi):
            raise D1ArmError(
                f"joint {i} (J{i}) = {q:.1f}° 超出 [{lo:.1f}, {hi:.1f}]")


def gripper_mm_to_deg(mm: float) -> float:
    """金属指间距 mm → J6 舵机角（标定表分段线性插值，出界截断）。"""
    mm = min(max(float(mm), GRIPPER_TABLE[-1][1]), GRIPPER_TABLE[0][1])
    pts = sorted(GRIPPER_TABLE, key=lambda p: p[1])  # np.interp 要求 x 升序
    return float(np.interp(mm, [p[1] for p in pts], [p[0] for p in pts]))


def gripper_deg_to_mm(deg: float) -> float:
    """J6 舵机角 → 金属指间距 mm（标定表分段线性插值，出界截断到表内）。"""
    deg = min(max(float(deg), GRIPPER_TABLE[-1][0]), GRIPPER_TABLE[0][0])
    pts = sorted(GRIPPER_TABLE, key=lambda p: p[0])  # np.interp 要求 x 升序
    return float(np.interp(deg, [p[0] for p in pts], [p[1] for p in pts]))


def gripper_formula_mm(deg: float) -> float:
    """旧公式 间距 = 0.85×角度 + 19.8（仅 0–50° 段有效；供单测对照，≤1 mm）。"""
    return 0.85 * deg + 19.8


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi
