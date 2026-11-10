# -*- coding: utf-8 -*-
"""d1t_sdk 轨迹生成（移植自同事 d1arm_sdk src/trajectory.cpp，按我们的 20 Hz/dt 50 ms 与
控制器逐轴限速（臂模型 axis_speed_dps）调整）。

时间参数化：五次多项式 s(τ)=10τ³-15τ⁴+6τ⁵（端点速度/加速度为 0）
  峰值速度 = 1.875·d/T，峰值加速度 = 5.7735·d/T²，由限速/限加速度反解最小周期 T。
输出：20 Hz 关节角采样序列（度），由 d1t_sdk 的发送线程按帧同步节拍（每收到一帧 ArmState
发一条）下发给控制器。
"""
import math

import numpy as np

from ._kinematics import IkParams, interp_cartesian, pose_error

K_DEG = 180.0 / math.pi


def _quintic(tau: float) -> float:
    tau = min(max(tau, 0.0), 1.0)
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def _min_duration(d: float, v: float, a: float) -> float:
    if d < 1e-9:
        return 0.0
    t_v = 1.875 * d / v              # 峰值速度约束
    t_a = math.sqrt(5.7735 * d / a)  # 峰值加速度约束
    return max(t_v, t_a)


def plan_joint_move(q0_deg, q1_deg, vel_dps, acc_dps2, dt_ms: float = 50.0,
                    vel_max_dps=None) -> tuple:
    """多关节同步点到点（关节空间，度）。q0/q1 等长（6 关节或单轴 1 维均可）。

    vel_dps/acc_dps2：标量或与 q 等长序列；vel_max_dps：逐轴上限（°/s，与 q 同空间，
    缺省标量 99.1）——规划前逐轴钳到上限，否则控制器截速会让各轴不同步、路径变形
    （任务书 §「必须处理的差异」#3）。
    返回 (samples 列表 [N][len(q0)], duration_s)。
    """
    q0 = np.asarray(q0_deg, dtype=float)
    q1 = np.asarray(q1_deg, dtype=float)
    n_ax = len(q0)
    vel = np.full(n_ax, float(vel_dps)) if np.isscalar(vel_dps) else np.asarray(vel_dps, dtype=float)
    acc = np.full(n_ax, float(acc_dps2)) if np.isscalar(acc_dps2) else np.asarray(acc_dps2, dtype=float)
    vmax = (np.full(n_ax, 99.1) if vel_max_dps is None
            else (np.full(n_ax, float(vel_max_dps)) if np.isscalar(vel_max_dps)
                  else np.asarray(vel_max_dps, dtype=float)))
    vel = np.minimum(vel, vmax)

    t_s = 0.0
    for i in range(n_ax):
        t_s = max(t_s, _min_duration(abs(q1[i] - q0[i]), vel[i], acc[i]))
    n = int(math.ceil(t_s * 1000.0 / dt_ms))
    samples = []
    for k in range(n + 1):
        t = min(k * dt_ms, t_s * 1000.0)
        s = _quintic(t / (t_s * 1000.0)) if t_s > 0 else 1.0
        samples.append(q0 + s * (q1 - q0))
    return samples, t_s


def plan_line(kin, pose_a, pose_b, seed_rad, lims_rad, vel_mps: float = 0.15,
              acc_mps2: float = 2.0, dt_ms: float = 50.0,
              vmax_dps=None) -> tuple:
    """直线（笛卡尔空间）：逐点 IK + 五次多项式时间最优化。

    返回 (samples_deg, duration_s)；IK 失败抛 RuntimeError（由调用方转 D1ArmError）。
    lims_rad: (qmin[6], qmax[6]) 弧度；vmax_dps: 逐轴可行性约束用的限速（°/s）。
    """
    return _cartesian_to_joint_path(kin, [pose_a, pose_b], seed_rad, lims_rad,
                                    vel_mps, acc_mps2, dt_ms, vmax_dps)


def plan_arc(kin, pose_a, pose_via, pose_b, seed_rad, lims_rad,
             vel_mps: float = 0.15, acc_mps2: float = 2.0, dt_ms: float = 50.0,
             vmax_dps=None) -> tuple:
    """圆弧：经过 via 的三点定圆（移植 planArc，含共线退化成直线）。"""
    p0 = pose_a[:3, 3]
    p1 = pose_via[:3, 3]
    p2 = pose_b[:3, 3]
    d01 = p1 - p0
    d02 = p2 - p0
    if float(np.linalg.norm(np.cross(d01, d02))) < 1e-6:
        return _cartesian_to_joint_path(kin, [pose_a, pose_b], seed_rad, lims_rad,
                                        vel_mps, acc_mps2, dt_ms, vmax_dps)
    e1 = d01 / np.linalg.norm(d01)
    n = np.cross(d01, d02)
    n = n / np.linalg.norm(n)
    e2 = np.cross(n, e1)
    x1 = float(np.dot(d01, e1))
    x2 = float(np.dot(d02, e1))
    y2 = float(np.dot(d02, e2))
    cx = 0.5 * x1
    # 过 (0,0),(x1,0),(x2,y2) 的圆心：cy = (x2²+y2²−x1·x2)/(2·y2)
    cy = 0.5 * (x2 * x2 + y2 * y2 - x1 * x2) / y2
    center = p0 + cx * e1 + cy * e2

    def ang_of(p):
        rel = p - center
        return math.atan2(float(np.dot(rel, e2)), float(np.dot(rel, e1)))

    def unwrap(frm, to):
        while to < frm:
            to += 2 * math.pi
        while to - frm >= 2 * math.pi:
            to -= 2 * math.pi
        return to

    a0, a1, a2 = ang_of(p0), ang_of(p1), ang_of(p2)
    a1 = unwrap(a0, a1)
    a2 = unwrap(a1, a2)
    sweep = a2 - a0

    nseg = max(8, int(abs(sweep) / (5.0 * math.pi / 180.0)))
    poses = []
    radius = float(np.linalg.norm(p0 - center))
    for k in range(nseg + 1):
        s = k / nseg
        ang = a0 + s * sweep
        pose = interp_cartesian(pose_a, pose_b, s)  # 姿态 slerp
        pos = center + radius * (math.cos(ang) * e1 + math.sin(ang) * e2)
        if k == nseg:
            pos = p2  # 半径比例修正（p0/p2 半径可能略有差异）
        pose = pose.copy()
        pose[:3, 3] = pos
        poses.append(pose)
    return _cartesian_to_joint_path(kin, poses, seed_rad, lims_rad,
                                    vel_mps, acc_mps2, dt_ms, vmax_dps)


def _cartesian_to_joint_path(kin, poses, seed_rad, lims_rad, vel_mps, acc_mps2,
                             dt_ms, vmax_dps) -> tuple:
    """笛卡尔路径 → 关节轨迹（逐点 IK，失败即中止；两遍法：粗采样求关节总行程拉长 T）。"""
    if len(poses) < 2:
        raise RuntimeError("笛卡尔路径至少 2 个位姿")
    qmin, qmax = (np.asarray(lims_rad[0]), np.asarray(lims_rad[1]))
    length = 0.0
    for i in range(1, len(poses)):
        length += float(np.linalg.norm(poses[i][:3, 3] - poses[i - 1][:3, 3]))
    t_s = 0.0
    if length > 1e-9:
        # vel_mps 为期望平均速度：T = L/vel（峰值 = 1.875×vel）
        t_s = max(length / vel_mps, math.sqrt(5.7735 * length / acc_mps2))

    vmax = (np.full(6, 99.1) if vmax_dps is None else np.asarray(vmax_dps, dtype=float))
    # ---- 关节可行性约束（第一遍）：粗采样 IK 求各关节总行程，保证五次多项式
    #      峰值关节速度/加速度不超限；否则拉长 T（移植 cartesianToJointPath）。
    coarse = 10
    q_prev = np.asarray(seed_rad, dtype=float).copy()
    dq_total = np.zeros(6)
    feas_ok = True
    ikp_coarse = IkParams(random_restarts=False)
    for k in range(coarse + 1):
        sk = k / coarse
        idx = min(int(sk * (len(poses) - 1)), len(poses) - 2)
        local = sk * (len(poses) - 1) - idx
        target = interp_cartesian(poses[idx], poses[idx + 1], local)
        res = kin.ik(target, q_prev, qmin, qmax, ikp_coarse)
        if not res.success:
            feas_ok = False
            break
        dq_total += np.abs(res.q_rad - q_prev)
        q_prev = res.q_rad.copy()
    if feas_ok:
        for i in range(6):
            dqi = dq_total[i] * K_DEG
            t_s = max(t_s, _min_duration(dqi, vmax[i], 500.0))  # acc 上限沿用同事 500°/s²

    n = max(len(poses) - 1, 1)
    n_samples = max(int(math.ceil(t_s * 1000.0 / dt_ms)), 4)

    q = np.asarray(seed_rad, dtype=float).copy()
    ikp = IkParams(random_restarts=False)  # 路径 IK 不重启，保持连续性
    samples = []
    for k in range(n_samples + 1):
        u = k / n_samples                    # 均匀参数
        s = _quintic(u) if t_s > 0 else 1.0  # 时间最优化
        f = s * n
        idx = min(int(f), n - 1)
        local = f - idx
        target = interp_cartesian(poses[idx], poses[idx + 1], local)
        res = kin.ik(target, q, qmin, qmax, ikp)
        if not res.success:
            raise RuntimeError(
                f"路径 IK 失败（样本 {k}/{n_samples}，位置误差 {res.pos_err * 1000:.1f} mm）")
        q = res.q_rad.copy()
        samples.append(q * K_DEG)
    return samples, t_s


def check_pose_reachable(kin, pose, seed_rad, lims_rad) -> None:
    """单点可达性检查（move_j 前）：IK 不收敛或越界即抛 RuntimeError，带详情。"""
    qmin, qmax = (np.asarray(lims_rad[0]), np.asarray(lims_rad[1]))
    res = kin.ik(np.asarray(pose), np.asarray(seed_rad), qmin, qmax)
    if not res.success:
        raise RuntimeError(
            f"IK 无解（位置误差 {res.pos_err * 1000:.1f} mm，姿态误差 "
            f"{res.rot_err * 180 / math.pi:.2f}°）")
