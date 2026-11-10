# -*- coding: utf-8 -*-
"""d1t_sdk 运动学：FK / 几何雅可比 / DLS 数值 IK（移植自 d1arm_sdk 开源项目 src/kinematics.cpp）。

连杆参数与 TCP 从臂模型加载（sdk/arm_model/d1t.json，缺省；可传别的路径的模型 dict），
数值与 arm_model/d1_description 的 URDF 逐项一致，TCP 取两夹爪指 origin 的中点。
注意：q 为 URDF 关节角弧度；舵机角 ↔ URDF 角的换算在 _joint_model，本模块不感知舵机。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ._arm_model import load_arm_model

PI = math.pi


def _xyzrpy(x, y, z, roll, pitch, yaw) -> np.ndarray:
    """URDF rpy：R = Rz(yaw) * Ry(pitch) * Rx(roll)。"""
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t[:3, :3] = rz @ ry @ rx
    return t


def _axis_rot(axis, q) -> np.ndarray:
    """绕单位轴 axis 转 q（Rodrigues）。"""
    x, y, z = axis
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _rot_to_quat(r: np.ndarray):
    """旋转矩阵 → (x, y, z, w)，w ≥ 0（与 Eigen AngleAxis 的角度 ∈ [0,π] 口径一致）。"""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array((x, y, z, w))
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q


def _quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_slerp(qa, qb, s):
    qa = np.asarray(qa, dtype=float)
    qb = np.asarray(qb, dtype=float)
    dot = float(np.dot(qa, qb))
    if dot < 0.0:
        qb, dot = -qb, -dot
    if dot > 0.9995:
        q = qa + s * (qb - qa)
        return q / np.linalg.norm(q)
    theta_0 = math.acos(min(1.0, dot))
    theta = theta_0 * s
    s0 = math.sin(theta_0 - theta) / math.sin(theta_0)
    s1 = math.sin(theta) / math.sin(theta_0)
    return s0 * qa + s1 * qb


def rot_to_axis_angle(r: np.ndarray):
    """旋转矩阵 → (axis*angle) 旋转向量（角度 ∈ [0, π]）。"""
    q = _rot_to_quat(r)
    angle = 2.0 * math.acos(min(1.0, max(-1.0, q[3])))
    s = math.sqrt(max(0.0, 1.0 - q[3] * q[3]))
    if s < 1e-10:
        return np.zeros(3)
    return np.array([q[0], q[1], q[2]]) / s * angle


@dataclass
class IkParams:
    max_iter: int = 150
    pos_tol: float = 5e-4     # m
    rot_tol: float = 5e-3     # rad
    damping: float = 0.08     # DLS λ
    max_step_rad: float = 0.15  # 单次迭代最大关节步长（≈8.6°）
    nullspace_gain: float = 0.02  # 向关节中位收敛的零空间增益
    random_restarts: bool = True
    restarts: int = 6
    rng_seed: int = 42


@dataclass
class IkResult:
    success: bool = False
    q_rad: np.ndarray = field(default_factory=lambda: np.zeros(6))
    iters: int = 0
    pos_err: float = 0.0
    rot_err: float = 0.0


class Kinematics:
    """六关节链（移植自 d1arm_sdk 的 Kinematics，默认 TCP）。

    model：臂模型 dict（load_arm_model 的返回值）；None = 加载 sdk/arm_model/d1t.json。
    """

    N = 6

    def __init__(self, model=None):
        if model is None:
            model = load_arm_model()
        self.links = [
            (_xyzrpy(*j["xyz"], *j["rpy"]), np.array(j["axis"]))
            for j in model["joints"]
        ]
        self.tool = _xyzrpy(*model["tcp"]["xyz"], *model["tcp"]["rpy"])

    def set_tool(self, tool: np.ndarray) -> None:
        self.tool = np.asarray(tool, dtype=float)

    def fk_chain(self, q_rad) -> list:
        """base..link6 共 7 个 frame。"""
        t = np.eye(4)
        frames = [t.copy()]
        for (origin, axis), qi in zip(self.links, q_rad):
            rot = np.eye(4)
            rot[:3, :3] = _axis_rot(axis, qi)
            t = t @ origin @ rot
            frames.append(t.copy())
        return frames

    def fk(self, q_rad) -> np.ndarray:
        return self.fk_chain(q_rad)[-1] @ self.tool

    def jacobian(self, q_rad) -> np.ndarray:
        """几何雅可比（线速度/角速度，TCP，基坐标系），6×6。"""
        frames = self.fk_chain(q_rad)
        t_tcp = frames[-1] @ self.tool
        p_tcp = t_tcp[:3, 3]
        jac = np.zeros((6, 6))
        for i in range(6):
            origin, axis = self.links[i]
            z = frames[i + 1][:3, :3] @ axis
            p = frames[i + 1][:3, 3]
            jac[:3, i] = np.cross(z, p_tcp - p)
            jac[3:, i] = z
        return jac

    def ik(self, target: np.ndarray, seed_rad, qmin_rad, qmax_rad,
           p: IkParams | None = None) -> IkResult:
        """DLS + 关节限位 + 零空间中位偏置 + 随机重启（移植同事算法，含自适应阻尼）。"""
        prm = p if p is not None else IkParams()
        q_mid = 0.5 * (np.asarray(qmin_rad) + np.asarray(qmax_rad))
        rng = np.random.default_rng(prm.rng_seed)
        q = np.clip(np.asarray(seed_rad, dtype=float).copy(),
                    np.asarray(qmin_rad), np.asarray(qmax_rad))
        best = IkResult()
        best.pos_err, best.rot_err = 1e9, 1e9

        for attempt in range(1 + (prm.restarts if prm.random_restarts else 0)):
            if attempt > 0:
                q = q_mid + rng.uniform(-1.0, 1.0, 6) * 0.5 * (np.asarray(qmax_rad) - np.asarray(qmin_rad))
            for iters in range(prm.max_iter):
                cur = self.fk(q)
                e = pose_error(target, cur)
                pe = float(np.linalg.norm(e[:3]))
                re = float(np.linalg.norm(e[3:]))
                if pe + re < best.pos_err + best.rot_err:
                    best.pos_err, best.rot_err, best.q_rad, best.iters = pe, re, q.copy(), iters
                if pe < prm.pos_tol and re < prm.rot_tol:
                    return IkResult(True, q.copy(), iters, pe, re)
                jac = self.jacobian(q)
                lam = prm.damping * min(1.0, max(0.12, (pe + re) / 0.05))
                bias_scale = min(1.0, (pe + re) / 0.02)
                bias = bias_scale * prm.nullspace_gain * (q_mid - q)
                jjt = jac @ jac.T
                jjt[np.diag_indices(6)] += lam * lam
                jpinv = jac.T @ np.linalg.inv(jjt)
                dq = jpinv @ e + (np.eye(6) - jpinv @ jac) @ bias
                if not np.all(np.isfinite(dq)):
                    break
                maxabs = float(np.max(np.abs(dq)))
                if maxabs > prm.max_step_rad:
                    dq *= prm.max_step_rad / maxabs
                q = np.clip(q + dq, np.asarray(qmin_rad), np.asarray(qmax_rad))
        return best


def pose_error(target: np.ndarray, cur: np.ndarray) -> np.ndarray:
    """6 维误差：位置 3 + 旋转向量 3（移植 Kinematics::poseError）。"""
    e = np.zeros(6)
    e[:3] = target[:3, 3] - cur[:3, 3]
    r_err = target[:3, :3] @ cur[:3, :3].T
    e[3:] = rot_to_axis_angle(r_err)
    return e


def to_rpy(t: np.ndarray):
    """Isometry → (roll, pitch, yaw)，ZYX 顺序（移植 Kinematics::toRPY）。"""
    r = t[:3, :3]
    pitch = -math.asin(min(1.0, max(-1.0, r[2, 0])))
    if abs(math.cos(pitch)) < 1e-8:  # 万向锁
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    else:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def to_pose_dict(t: np.ndarray) -> dict:
    """与同事 fillState/setPose 的 pose JSON 同构：pos [m]、quat [w,x,y,z]、rpy [rad]。"""
    q = _rot_to_quat(t[:3, :3])
    roll, pitch, yaw = to_rpy(t)
    return {"pos": [float(v) for v in t[:3, 3]],
            "quat": [float(q[3]), float(q[0]), float(q[1]), float(q[2])],
            "rpy": [roll, pitch, yaw]}


def from_pose_dict(pos, rpy=None, quat=None) -> np.ndarray:
    """{"pos", "rpy"(rad) 或 "quat"(w,x,y,z)} → 4×4 Isometry（同事 parsePose 口径：quat 优先）。"""
    t = np.eye(4)
    t[:3, 3] = np.asarray(pos, dtype=float)[:3]
    if quat is not None:
        w, x, y, z = (float(v) for v in quat[:4])
        t[:3, :3] = _quat_to_rot(np.array([x, y, z, w]))
    elif rpy is not None:
        roll, pitch, yaw = (float(v) for v in rpy[:3])
        t[:3, :3] = _xyzrpy(0, 0, 0, roll, pitch, yaw)[:3, :3]
    return t


def interp_cartesian(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    """笛卡尔插值（直线）：位置 lerp + 姿态 slerp（移植 interpCartesian）。"""
    s = min(max(s, 0.0), 1.0)
    out = np.eye(4)
    out[:3, 3] = (1 - s) * a[:3, 3] + s * b[:3, 3]
    qa = _rot_to_quat(a[:3, :3])
    qb = _rot_to_quat(b[:3, :3])
    out[:3, :3] = _quat_to_rot(_quat_slerp(qa, qb, s))
    return out
