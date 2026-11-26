# -*- coding: utf-8 -*-
"""TeleopLoop — 遥操作一帧一步（分层中的中间层）。

入：PicoProbe 的 status（state/target/trigger，见 tools/pico_probe/pico_probe.py）与
arm.state() 字典；出：至多一次 arm.stream_pose（返回 dict 记录是否发送、下发目标、
j6、事件、限幅与逆解失败计数，供主循环打日志）。

- 接合（idle→active）：stream_start，首帧目标 = mapper 捕获的实测 TCP（不跳，不限幅）；
- 脱开（active→idle）：持续发**命令**位姿保持（不用实测角，防负载轴下沉/夹爪棘轮），
  j6 继续随扳机走 GripLock；
- 断流（stale）：stream_stop（臂由控制器 HOLD 保持），GripLock 冻结；恢复后须先松握把
  才能再接合（mapper 的 require_release），再次接合重新 stream_start；
- heading：目标相对接合参考的水平位移左乘 Rz(yaw)，姿态增量换系 Rz·D·Rzᵀ（接合首帧不跳）；
- limiter：单帧位置步长截断 + 工作空间盒钳制 + 单帧姿态转角截断；逆解失败保持上一帧命令并计数，不抛出；
- 控制器 ESTOP/FAULT：抛 D1ArmError，由主循环停机。

循环节拍由主循环 arm.wait_frame() 保证（20 Hz），本类不做 sleep。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

_SDK_DIR = Path(__file__).resolve().parents[2] / "sdk"
for _p in (str(_SDK_DIR), str(Path(__file__).resolve().parent),
           str(Path(__file__).resolve().parents[1] / "pico_probe")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from d1t_sdk import D1ArmError                      # noqa: E402
from d1t_sdk._common import CTRL_ESTOP, CTRL_FAULT, CTRL_STATE_NAMES  # noqa: E402
from d1t_sdk._joint_model import gripper_mm_to_deg  # noqa: E402

from griplock import GripLock                        # noqa: E402
from pico_probe import trigger_to_j6                 # noqa: E402

DT_S = 0.05                     # 控制周期 20 Hz（与控制器/SDK 一致）
DEFAULT_MAX_VEL_MPS = 0.25      # TCP 步长上限 → 12.5 mm/帧
DEFAULT_MAX_ROT_VEL_DPS = 60.0  # TCP 姿态转速上限 → 3°/帧（实机 30 太紧，60 手感可用）
WS_BOX_M = ((0.05, 0.50), (-0.35, 0.35), (0.00, 0.55))   # 基座系 x前 y左 z上（m）


def state_str(state) -> str:
    """mapper 状态枚举 → 字符串（测试可直接喂字符串）。"""
    return state.value if hasattr(state, "value") else str(state)


def j6_measured_deg(arm_state: dict) -> float:
    """J6 舵机实测角（度）。state() 不直接暴露它，用夹爪标定表从 gripper_mm 反算：
    表内精确互逆；表外（[−30, −29.6] 与 [64.2, 64.5]°）截到表端，误差 ≤ 0.4°，
    对 GripLock 各阈值（−28 / ±2–3° / 0.3°）无影响。"""
    return float(gripper_mm_to_deg(arm_state["gripper_mm"]))


def quat_wxyz(rot) -> list:
    """3×3 旋转矩阵 → [w, x, y, z]（stream_pose 的 quat 参数格式）。"""
    x, y, z, w = Rotation.from_matrix(np.asarray(rot, dtype=float)).as_quat()  # scipy 默认 xyzw
    return [float(w), float(x), float(y), float(z)]


def limit_step(pos: np.ndarray, prev: np.ndarray, step_max_m: float):
    """单帧步长截断：|pos − prev| 超上限则沿方向截到上限。返回 (新 pos, 是否截断)。"""
    d = pos - prev
    n = float(np.linalg.norm(d))
    if n <= step_max_m:
        return pos, False
    return prev + d * (step_max_m / n), True


def limit_rot(quat_wxyz_new, quat_wxyz_prev, step_max_rad: float):
    """单帧姿态步长截断：从上一帧命令姿态转向新姿态，转角超上限则沿同一转轴截到上限。
    返回 (新 quat wxyz, 是否截断)。"""
    w, x, y, z = quat_wxyz_new
    r_new = Rotation.from_quat([x, y, z, w])
    w, x, y, z = quat_wxyz_prev
    r_prev = Rotation.from_quat([x, y, z, w])
    rv = (r_new * r_prev.inv()).as_rotvec()
    ang = float(np.linalg.norm(rv))
    if ang <= step_max_rad:
        return list(quat_wxyz_new), False
    r_out = Rotation.from_rotvec(rv * (step_max_rad / ang)) * r_prev
    x, y, z, w = r_out.as_quat()
    return [float(w), float(x), float(y), float(z)], True


def clamp_box(pos: np.ndarray, ws_box):
    """工作空间盒逐轴钳制。返回 (新 pos, 是否钳制)。"""
    out = np.array([min(max(float(pos[i]), lo), hi) for i, (lo, hi) in enumerate(ws_box)])
    return out, bool(np.any(np.abs(out - pos) > 1e-12))


def yaw_from_push(p_engage, p_release, min_push_m: float = 0.10) -> float:
    """标定手势的纯计算：接合首帧目标 → 松开前目标 的水平推程 → 修正 yaw（度）。

    推程方向相对基座 +X（前）的偏角 θ = atan2(Δy, Δx)，修正量取 −θ（使「向前推」映射到
    +X）。推程水平长度不足 min_push_m 抛 ValueError。"""
    dx = float(p_release[0]) - float(p_engage[0])
    dy = float(p_release[1]) - float(p_engage[1])
    if math.hypot(dx, dy) < min_push_m:
        raise ValueError(f"推程水平分量不足 {min_push_m:.2f} m（{math.hypot(dx, dy):.3f} m），重推一次")
    return -math.degrees(math.atan2(dy, dx))


class TeleopLoop:
    """一帧一步的遥操作状态机（无 sleep、无线程；由主循环逐帧驱动）。"""

    def __init__(self, arm, yaw_deg: float = 0.0, max_vel_mps: float = DEFAULT_MAX_VEL_MPS,
                 max_rot_vel_dps: float = DEFAULT_MAX_ROT_VEL_DPS,
                 ws_box=WS_BOX_M, griplock: GripLock | None = None,
                 fake_grasp_at: float | None = None, dt_s: float = DT_S):
        self.arm = arm
        self.set_yaw_deg(yaw_deg)
        self.step_max_m = float(max_vel_mps) * float(dt_s)
        self.rot_step_max_rad = math.radians(float(max_rot_vel_dps)) * float(dt_s)
        self.ws_box = tuple((float(lo), float(hi)) for lo, hi in ws_box)
        self.griplock = griplock if griplock is not None else GripLock(dt_s=dt_s)
        self.fake_grasp_at = None if fake_grasp_at is None else float(fake_grasp_at)
        self.streaming = False
        self.held = None            # (np.ndarray pos, quat wxyz)：最近一次**下发**的位姿命令
        self.ref = None             # 本次接合的参考（mapper 捕获帧的目标）
        self.ik_fail = 0            # 累计逆解失败帧数
        self.step_clamped_n = 0     # 累计步长截断帧数
        self.box_clamped_n = 0      # 累计出盒钳制帧数
        self._last_state = None

    def set_yaw_deg(self, deg: float) -> None:
        self.yaw_deg = float(deg)
        r = math.radians(self.yaw_deg)
        c, s = math.cos(r), math.sin(r)
        self._rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def detach(self) -> None:
        """交出流式（A/B 一键回位前后用）：在流就 stream_stop（臂 HOLD），清参考与保持位姿；
        下次接合重新 stream_start、以新位置为参考。"""
        if self.streaming:
            self.arm.stream_stop()
        self.streaming = False
        self.held = None
        self.ref = None

    # ------------------------------------------------------------------
    def step(self, status: dict, arm_state: dict) -> dict:
        st = state_str(status.get("state"))
        if arm_state["ctrl_state"] in (CTRL_ESTOP, CTRL_FAULT):
            name = CTRL_STATE_NAMES.get(arm_state["ctrl_state"], arm_state["ctrl_state"])
            raise D1ArmError(f"控制器进入 {name}，遥操作停止（软急停恢复要先 reset()）")

        j6_meas_raw = j6_measured_deg(arm_state)
        j6_meas = (max(j6_meas_raw, self.fake_grasp_at) if self.fake_grasp_at is not None
                   else j6_meas_raw)   # --fake-grasp-at：该角度以下的实测钳在该角度（调试用）
        j6_cmd = trigger_to_j6(float(status.get("trigger", 0.0)))
        engaged = st == "active" and status.get("target") is not None
        holding = self.streaming and self.held is not None     # 脱开但仍在发保持命令
        event = None
        if st == "stale" or not (engaged or holding):
            # 断流，或首帧接合前（从未发送）：冻结输出——避免 _out 在未发送帧里
            # 自行推进、接合瞬间相对实测角跳变
            j6_out = self.griplock.freeze()
            if j6_out is None:
                j6_out = j6_meas
        else:
            j6_out, event = self.griplock.update(j6_cmd, j6_meas)

        out = {"probe_state": st, "sent": False, "event": event, "streaming": self.streaming,
               "j6_cmd": j6_cmd, "j6_meas": j6_meas, "j6_meas_raw": j6_meas_raw,
               "j6_out": j6_out, "locked": self.griplock.locked,
               "lock_deg": self.griplock.lock_deg,
               "pos": None, "quat": None,
               "step_clamped": False, "box_clamped": False, "rot_clamped": False,
               "ik_fail": self.ik_fail,
               "step_clamped_n": self.step_clamped_n, "box_clamped_n": self.box_clamped_n}

        # 断流：停流（一次），臂由控制器保持；参考/保持位姿作废
        if st == "stale":
            if self.streaming:
                self.arm.stream_stop()
                self.streaming = False
                self.held = None
                self.ref = None
            out["streaming"] = False
            self._last_state = st
            return out

        target = status.get("target")
        engaged = st == "active" and target is not None
        if engaged:
            if not self.streaming:
                self.arm.stream_start()
                self.streaming = True
            if self._last_state != "active":          # 新接合：重捕参考；首帧不限幅（不跳）
                self.ref = None
                self.held = None
                out["streaming"] = self.streaming
            if self.ref is None:
                self.ref = (np.asarray(target.position, dtype=float).copy(),
                            np.asarray(target.rotation, dtype=float).copy())
            delta = np.asarray(target.position, dtype=float) - self.ref[0]
            pos = self.ref[0] + self._rz @ delta
            # 姿态只转「增量」：mapper 给的是 D·R_ref（D = 手柄在 XR 世界系的转动增量），
            # 把 D 换到基座系 Rz·D·Rzᵀ 再乘回 R_ref；接合首帧 D = I → 姿态不跳（任意 yaw）
            d_rot = np.asarray(target.rotation, dtype=float) @ self.ref[1].T
            quat = quat_wxyz(self._rz @ d_rot @ self._rz.T @ self.ref[1])
            if self.held is not None:
                pos, s_cl = limit_step(pos, self.held[0], self.step_max_m)
                pos, b_cl = clamp_box(pos, self.ws_box)
                quat, r_cl = limit_rot(quat, self.held[1], self.rot_step_max_rad)
                out["step_clamped"], out["box_clamped"], out["rot_clamped"] = s_cl, b_cl, r_cl
        elif self.streaming and self.held is not None:
            pos, quat = self.held[0], self.held[1]    # 脱开：保持命令位姿，j6 随扳机
        else:
            self._last_state = st                     # 尚未接合过：什么都不发
            return out

        try:
            self.arm.stream_pose([float(v) for v in pos], quat=quat, j6_deg=j6_out)
        except D1ArmError as e:
            if "IK 无解" in str(e) or "超出" in str(e):
                self.ik_fail += 1                     # 保持上一帧命令，不抛出
                out["ik_fail"] = self.ik_fail
                out["step_clamped"] = out["box_clamped"] = False
                self._last_state = st
                return out
            raise
        if out["step_clamped"]:
            self.step_clamped_n += 1
        if out["box_clamped"]:
            self.box_clamped_n += 1
        out["step_clamped_n"], out["box_clamped_n"] = self.step_clamped_n, self.box_clamped_n
        self.held = (np.asarray(pos, dtype=float).copy(), quat)
        out.update(sent=True, pos=[float(v) for v in pos], quat=quat,
                   streaming=self.streaming)
        self._last_state = st
        return out
