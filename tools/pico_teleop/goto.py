# -*- coding: utf-8 -*-
"""A/B 键一键回位：A = 回准备位，B = 回折叠位（到位后保持，不失力）。

- ButtonHold：按键按住 ≥ hold_s 才触发一次（防拇指误碰），松开后才能再次触发；
- GotoMotion：关节空间点到点，逐帧 stream_joints 下发（跟主循环 wait_frame 节拍），
  任何一帧都能停（主循环不再调 step、改发 stream_stop 即停在当前命令位置）；
  J6 传 None = 保持当前命令角（手里夹着东西回准备位不松手）。

起点取控制器的**命令角**（SDK `_start_q_urdf`），不取实测角（负载轴保持时实测比命令低，
从实测起步第一帧会掉一下，F87）。
"""
from __future__ import annotations

import sys
from pathlib import Path

_SDK_DIR = Path(__file__).resolve().parents[2] / "sdk"
if str(_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(_SDK_DIR))

from d1t_sdk._joint_model import servo_to_joint  # noqa: E402
from d1t_sdk._trajectory import plan_joint_move  # noqa: E402
from d1t_sdk._common import DT_MS, URDF_VMAX_DPS  # noqa: E402
from d1t_sdk.d1arm import PARK_GRIP_OPEN_MIN_MM, PARK_SERVO_DEG  # noqa: E402

READY_Q_DEG = (0.0, -40.0, 60.0, 0.0, 30.0, 0.0)      # 准备位 = 遥操作起步位（URDF 角）
FOLD_Q_DEG = tuple(servo_to_joint(i, PARK_SERVO_DEG[i]) for i in range(6))  # 折叠位（同 park）
FOLD_GRIP_OPEN_MIN_MM = PARK_GRIP_OPEN_MIN_MM          # 折叠前夹爪须张开（同 park）
FOLD_NEAR_DEG = 10.0                                   # 各关节都在折叠位 ±此值内 = 视为折叠
GOTO_VEL_DPS = 65.0
GOTO_ACC_DPS2 = 60.0
HOLD_S = 0.5


class ButtonHold:
    """update(now, a, b) → None | "ready" | "fold"。同一次按住只触发一次。"""

    def __init__(self, hold_s: float = HOLD_S):
        self.hold_s = float(hold_s)
        self._since = {"a": None, "b": None}
        self._fired = {"a": False, "b": False}

    def update(self, now: float, a: bool, b: bool):
        fire = None
        for key, pressed, name in (("a", a, "ready"), ("b", b, "fold")):
            if not pressed:
                self._since[key] = None
                self._fired[key] = False
                continue
            if self._since[key] is None:
                self._since[key] = now
            if not self._fired[key] and now - self._since[key] >= self.hold_s:
                self._fired[key] = True
                fire = fire or name
        return fire


def near_fold(q_deg, tol_deg: float = FOLD_NEAR_DEG) -> bool:
    """J0–J5（URDF 角）都在折叠位 ±tol 内。折叠位 TCP 仍在工作空间盒内，接合前须单独判。"""
    return max(abs(float(q_deg[i]) - FOLD_Q_DEG[i]) for i in range(6)) <= tol_deg


def plan_segments(q_start, segments, vel_dps=GOTO_VEL_DPS, acc_dps2=GOTO_ACC_DPS2):
    """把若干段目标（URDF 角）规划成一串逐帧样本。返回 (samples, 总时长 s)。"""
    samples, total, q0 = [], 0.0, list(q_start)
    for q1 in segments:
        seg, t_s = plan_joint_move(q0, list(q1), vel_dps, acc_dps2, DT_MS, URDF_VMAX_DPS)
        samples.extend(seg[1:] if samples else seg)
        total += t_s
        q0 = list(q1)
    return samples, total


def fold_segments(q_cmd):
    """折叠两段（同 park 顺序）：① 腕部 J3–J5 回折叠值、J0–J2 不动；② J0–J2 收回。"""
    wrist = list(q_cmd[0:3]) + list(FOLD_Q_DEG[3:6])
    return [wrist, list(FOLD_Q_DEG)]


class GotoMotion:
    """逐帧下发的一键回位。调用方先 stream_start；step() 每帧调一次，返回是否已走完。"""

    def __init__(self, arm, kind: str, vel_dps: float = GOTO_VEL_DPS,
                 acc_dps2: float = GOTO_ACC_DPS2):
        self.arm = arm
        self.kind = kind
        q_cmd = arm._start_q_urdf(arm._wait_state())
        segments = [list(READY_Q_DEG)] if kind == "ready" else fold_segments(q_cmd)
        self.samples, self.duration_s = plan_segments(q_cmd, segments, vel_dps, acc_dps2)
        self.i = 0

    @property
    def done(self) -> bool:
        return self.i >= len(self.samples)

    def step(self) -> bool:
        if not self.done:
            self.arm.stream_joints([float(v) for v in self.samples[self.i]], j6_deg=None)
            self.i += 1
        return self.done
