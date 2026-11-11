# -*- coding: utf-8 -*-
"""d1t_sdk — AI_drive_arm 臂载控制器（d1t_arm_ctrl）上位机 SDK。

接口与 D1 臂社区开源项目 d1arm_sdk（clients/python/d1arm.py）同名同参同单位，底层换成 DDS：
用户脚本 ── d1t_sdk.D1Arm（上位机或任一联网电脑，Python 库，不起服务）
              ├─ FK / IK、关节五次多项式、直线、圆弧（移植自 d1arm_sdk）
              └─ 规划结果切成 20 Hz 关节角，每收到一帧 ArmState 发一条 ArmCmd
                      ↓ DDS 命令话题 ArmCmd      ↑ 状态话题 ArmState
臂上 d1t_arm_ctrl（不改）：执行关节角 + 20 Hz 七轴反馈 + 看门狗/限位/限速/急停/J6 保护

与同事 SDK 的关系与差异见 README §「与 d1arm_sdk 的关系」。

发送线程（核心机制）：后台线程每收到一帧 ArmState 发恰好一条命令（按自身时钟定时发
会与控制器节拍漂移、出现顿挫，帧同步发送消除此问题）：
- 轨迹任务：按帧序号 seq 推进样本下标（丢帧按 seq 差补样本序号，同 send.py
  --sync-state），首样本重发直到控制器进入 STREAMING；
- 单发命令（HOLD/ESTOP/RESET/RELEASE）：每帧重发（新 seq/stamp）直到 ctrl_state 确认；
- 关节流式：每帧发最新 stream_joints 目标。
不发送时什么都不发；流式中停止发送 → 控制器看门狗约 0.3 s 停臂。
"""
import math
import threading
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader

from ._common import (CTRL_ESTOP, CTRL_FAULT, CTRL_HOLD, CTRL_IDLE, CTRL_RELEASED,
                      CTRL_STREAMING, D1ArmError, DT_MS, J6_SPEED_DPS, MODE_ESTOP,
                      MODE_HOLD, MODE_RELEASE, MODE_RESET, MODE_STREAM,
                      SERVO_LIMITS, URDF_VMAX_DPS, ArmCmd_, CTRL_STATE_NAMES, mode_name,
                      dds_qos, make_cmd_topic, make_state_topic)
from ._joint_model import (GRIPPER_TABLE, check_q_urdf, deg2rad, gripper_deg_to_mm,
                           gripper_mm_to_deg, joint_to_servo, rad2deg,
                           servo_to_joint, urdf_limits)
from ._kinematics import IkParams, Kinematics, from_pose_dict, to_pose_dict
from ._trajectory import plan_arc, plan_joint_move, plan_line
from . import _validate as _v

STATE_TIMEOUT_S = 5.0  # 等第一帧 ArmState 的超时（同 send.py 默认）

# 折叠位目标（下电收臂用，park()）：原厂 HOME 姿势（F7 舵机角 [0,-90,89,0,0,0,0]），
# 实际折叠位 J1/J2 比软限位各多约 0.5°（实机验收清单 §0、F44），取限位内留余量：
# J1=-90 ≥ -90.3、J2=88.5 ≤ 89
PARK_SERVO_DEG = (0.0, -90.0, 88.5, 0.0, 0.0, 0.0)
PARK_TOLERANCE_DEG = 3.0  # park 每段到位容差（沿用旧项目 F16 归位容差）
# park 起步夹爪判定：空夹全程约 14 mA（F60），带物最低约 58–70 mA（F61）。
# 判空 = J6 电流 < PARK_GRIP_EMPTY_MA 且 |命令−反馈| ≤ PARK_GRIP_GAP_MM（夹爪标定表换算 mm）
PARK_GRIP_EMPTY_MA = 40
PARK_GRIP_GAP_MM = 3.0
PARK_GRIP_OPEN_MM = 70.0
PARK_SETTLE_S = 0.5  # 最后一段到位后停多久再 free_all
JOINT_VEL_MAX_DPS = min(URDF_VMAX_DPS)  # 用户 vel_dps 上限（各轴 100 °/s），超出报错不钳


class _RealTransport:
    """真 DDS 传输（cyclonedds-python）。CYCLONEDDS_URI 由使用者环境提供
    （跨机必须固定 ParticipantIndex，见 sdk/实机验收清单.md）。"""

    def __init__(self):
        self.dp = DomainParticipant(0)
        self.rd = DataReader(self.dp, make_state_topic(self.dp), dds_qos())
        self.wr = DataWriter(self.dp, make_cmd_topic(self.dp), dds_qos())

    def poll_frames(self):
        try:
            return list(self.rd.take())
        except Exception:
            return []

    def send_cmd(self, cmd):
        self.wr.write(cmd)

    def close(self):
        pass  # cyclonedds-python 无 Participant.close()，进程退出即释放


class D1Arm:
    """D1-550 七轴臂（6 关节 + 夹爪）上位机客户端。

    参数单位（与同事 d1arm.py 一致）：关节角度、夹爪 mm（金属指间距）、位置 m、
    rpy 弧度、四元数 [w,x,y,z]、vel_dps °/s、acc_dps2 °/s²、vel m/s、acc m/s²。

    transport 仅供离线测试注入假控制器；生产用默认 DDS。
    """

    def __init__(self, timeout=30.0, transport=None):
        self._timeout = _v.number("timeout", timeout, 0, lo_open=True, unit="s")
        self._lock = threading.RLock()
        self._state = None          # 最新 ArmState_
        self._state_evt = threading.Event()
        self._frame_cv = threading.Condition()  # 每处理完一帧 ArmState（含本帧命令已发）通知一次
        self._frame_no = 0
        self._job = None            # 当前任务 dict（oneshot/traj），None=空闲
        self._stream_q = None       # 流式目标（舵机角 7 向量）；None=未在流式
        self._cmd7 = None           # 最近下发的 STREAM/HOLD 目标（舵机角 7）；失力/急停/复位后作废
        self._seq = 0
        self._last_stamp = 0
        self._closed = False
        self._kin = Kinematics()
        self._transport = transport if transport is not None else _RealTransport()
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

    # ---------- 底层发送线程 ----------

    def _send_loop(self):
        while not self._closed:
            frames = self._transport.poll_frames()
            for st in frames:
                self._on_frame(st)
            if not frames:
                time.sleep(0.002)

    def _on_frame(self, st):
        try:
            with self._lock:
                self._state = st
                self._state_evt.set()
                job = self._job
                if job is not None:
                    self._step_job(job, st)
                    return
                stream_q = self._stream_q
            if stream_q is not None:
                self._send_cmd(MODE_STREAM, q=stream_q)
        finally:
            with self._frame_cv:
                self._frame_no += 1
                self._frame_cv.notify_all()

    def wait_frame(self, timeout=1.0):
        """等到下一帧 ArmState 处理完（流式中即本帧命令已发出）。

        流式循环用它代替 time.sleep(0.05)：每帧恰好更新一次目标，
        不会因本机时钟与控制器节拍漂移而漏帧（停一下）或一帧跳两步（蹭一下）。"""
        timeout = _v.number("timeout", timeout, 0, lo_open=True, unit="s")
        with self._frame_cv:
            n = self._frame_no
            if not self._frame_cv.wait_for(lambda: self._frame_no > n, timeout):
                raise D1ArmError(f"{timeout:.1f}s 内没收到下一帧 ArmState")

    def _step_job(self, job, st):
        """发送线程内逐帧推进任务（持锁调用）。"""
        if job["kind"] == "oneshot":
            if st.ctrl_state in job["targets"]:
                job["ok"] = True
                job["done"].set()
                self._job = None
                return
            if time.monotonic() > job["deadline"]:
                job["reason"] = (f"{job['desc']} 后 {job['timeout_s']:.0f}s 内 ctrl_state "
                                 f"没进入 {job['target_names']}（当前 "
                                 f"{CTRL_STATE_NAMES.get(st.ctrl_state, st.ctrl_state)}）")
                job["done"].set()
                self._job = None
                return
            self._send_cmd(job["mode"], q=job["q"], axis_mask=job["mask"])
            return

        # ---- traj：首样本重发直到 STREAMING；之后按 seq 差推进样本（丢帧补偿）----
        samples = job["samples"]
        if not job["started"]:
            job["started"] = True
            self._send_cmd(MODE_STREAM, q=samples[0])
            return
        if job["t0_seq"] is None:
            if st.ctrl_state == CTRL_STREAMING:
                job["t0_seq"] = int(st.seq)
            else:
                job["warmups"] += 1
                if job["warmups"] > 60:  # ~3 s 还没进 STREAMING
                    job["reason"] = (f"首样本发出后控制器未进入 STREAMING（当前 "
                                     f"{CTRL_STATE_NAMES.get(st.ctrl_state, st.ctrl_state)}）")
                    job["done"].set()
                    self._job = None
                    return
                self._send_cmd(MODE_STREAM, q=samples[0])
                return
        if st.seq < job["t0_seq"]:  # uint32 回绕（会话极长才会出现）：重锚
            job["t0_seq"] = int(st.seq) - 1
        idx = int(st.seq) - job["t0_seq"] + 1  # 帧差 = 样本差 → 丢帧按 seq 差补偿
        if idx >= len(samples):
            self._send_cmd(MODE_HOLD, q=samples[-1])  # 收尾 HOLD（同 replay.py）
            job["ok"] = True
            job["done"].set()
            self._job = None
            return
        self._send_cmd(MODE_STREAM, q=samples[idx])

    def _send_cmd(self, mode, q, axis_mask=0, dt_ms=DT_MS, grip_power_mw=0):
        with self._lock:
            self._seq += 1
            stamp = max(self._last_stamp + 1, time.time_ns())  # 严格递增（任务书 §1.3）
            self._last_stamp = stamp
            cmd = ArmCmd_(seq=self._seq, stamp_ns=stamp, mode=mode, axis_mask=axis_mask,
                          q=[float(x) for x in q], dt_ms=dt_ms, grip_power_mw=grip_power_mw)
            self._cmd7 = [float(x) for x in q] if mode in (MODE_STREAM, MODE_HOLD) else None
        self._transport.send_cmd(cmd)

    # ---------- 任务提交 ----------

    def _wait_state(self, timeout_s=STATE_TIMEOUT_S):
        if not self._state_evt.wait(timeout_s):
            raise D1ArmError(f"{timeout_s:.0f}s 内没收到 ArmState（控制器没起？"
                             f"CYCLONEDDS_URI 没配？跨机 ParticipantIndex 固定了吗？）")
        with self._lock:
            return self._state

    def _check_ctrl_for_motion(self, st, what="运动"):
        if st.ctrl_state in (CTRL_ESTOP, CTRL_FAULT):
            raise D1ArmError(f"控制器处于 {CTRL_STATE_NAMES[st.ctrl_state]}，不能{what}"
                             f"（软急停恢复要先 reset()）")
        if st.ctrl_state == 0:
            raise D1ArmError(f"控制器 INIT（启动中），不能{what}")

    def _submit_oneshot(self, mode, targets, q_servo, axis_mask=0, timeout_s=5.0,
                        desc="", force=False):
        with self._lock:
            if self._job is not None and not force:
                raise D1ArmError("AlreadyMoving：已有命令/轨迹在执行（先 wait_idle）")
            job = {"kind": "oneshot", "mode": mode, "targets": set(targets),
                   "target_names": "/".join(CTRL_STATE_NAMES[t] for t in targets),
                   "q": list(q_servo), "mask": axis_mask,
                   "deadline": time.monotonic() + timeout_s, "timeout_s": timeout_s,
                   "desc": desc or f"命令 mode={mode}",
                   "done": threading.Event(), "ok": False, "reason": ""}
            self._job = job
        return job

    def _submit_traj(self, samples_servo, duration_s, desc="轨迹"):
        with self._lock:
            if self._job is not None:
                raise D1ArmError("AlreadyMoving：已有命令/轨迹在执行（先 wait_idle）")
            if self._stream_q is not None:
                raise D1ArmError("正在关节流式，先 stream_stop() 再发起规划运动")
            job = {"kind": "traj", "samples": samples_servo, "started": False,
                   "t0_seq": None, "warmups": 0, "desc": desc,
                   "done": threading.Event(), "ok": False, "reason": ""}
            self._job = job
        return job

    def _wait_job(self, job, duration_s=0.0, desc="轨迹"):
        total = duration_s + 10.0 + self._timeout
        if not job["done"].wait(total):
            raise D1ArmError(f"{desc} 超时（>{total:.0f}s）未完成")
        if not job["ok"]:
            raise D1ArmError(f"{desc} 失败：{job['reason']}")

    def _base7(self, st):
        """不动的轴用的保持目标（舵机角 7），优先级：
        ① 本进程上次下发的命令角 _cmd7（控制器在 HOLD/STREAMING 时有效）；
        ② 否则逐轴取控制器上报的命令角 st.q_target[i]（有限值时；本进程尚未下令也能拿到）；
        ③ q_target[i] 为 NaN（该轴失力/阻尼/从未下令）才退回实测角 st.q[i]。
        用实测角当保持目标会把负载轴的棘轮下沉、夹住物体的 J6 松开锁进新目标。"""
        with self._lock:
            c = self._cmd7
        if c is not None and st.ctrl_state in (CTRL_HOLD, CTRL_STREAMING):
            return list(c)
        out = []
        for i in range(7):
            qt = float(st.q_target[i])
            out.append(qt if math.isfinite(qt) else float(st.q[i]))
        return out

    def _to_servo7(self, q_urdf_deg, st=None):
        """6 关节 URDF 角 + J6 保持目标（舵机角，见 _base7）→ 7 轴舵机角向量。"""
        if st is None:
            st = self._wait_state()
        j6 = self._base7(st)[6]
        return [joint_to_servo(i, float(q_urdf_deg[i])) for i in range(6)] + [j6]

    # ---------- 状态 ----------

    def state(self, timeout_s=STATE_TIMEOUT_S) -> dict:
        """最新 ArmState 转成 dict（键同同事 fillState + 我们独有字段，见 README）。"""
        timeout_s = _v.number("timeout_s", timeout_s, 0, lo_open=True, unit="s")
        st = self._wait_state(timeout_s)
        with self._lock:
            job_active = self._job is not None
        q_servo = [float(x) for x in st.q]
        q_urdf = [servo_to_joint(i, q_servo[i]) for i in range(6)]
        mode = mode_name(st.ctrl_state, job_active)
        out = {
            "q_deg": q_urdf,                       # URDF 角（度），6 关节
            "servo_raw_deg": q_servo[:6],          # 舵机原始角（度），6 关节
            "gripper_mm": gripper_deg_to_mm(q_servo[6]),  # 金属指间距 mm（夹爪标定表）
            "calibrated": True,                    # 标定内置于 arm_model/d1t.json，接口兼容字段
            "tcp": to_pose_dict(self._kin.fk([deg2rad(q) for q in q_urdf])),
            "online": [bool(st.valid_mask >> i & 1) for i in range(7)],
            "voltage_mv": [int(v) for v in st.voltage_mv],
            "mode": mode,
            "stamp_ms": int(st.stamp_ns // 1_000_000),
            "seq": int(st.seq),
            "ctrl_state": int(st.ctrl_state),
            "fault": int(st.fault),
            "warn": int(st.warn),
            "temp_c": [float(t) for t in st.temp_c],
            "current_ma": [int(c) for c in st.current_ma],
        }
        return out

    def ping(self) -> list:
        """7 个舵机（id 0–6，0 起编号）在线状态 → list[bool]（按 valid_mask）。"""
        st = self._wait_state()
        return [bool(st.valid_mask >> i & 1) for i in range(7)]

    def wait_idle(self, timeout=30) -> bool:
        """等运动/命令完成且控制器不在运动（IDLE/HOLD/RELEASED/ESTOP/FAULT）。超时返回 False。"""
        timeout = _v.number("timeout", timeout, 0, lo_open=True, unit="s")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self._lock:
                busy = self._job is not None
                st = self._state
            if not busy and st is not None and st.ctrl_state in (
                    CTRL_IDLE, CTRL_HOLD, CTRL_RELEASED, CTRL_ESTOP, CTRL_FAULT):
                return True
            time.sleep(0.05)
        return False

    # ---------- v1 关节 ----------

    def move_joints(self, q_deg, vel_dps=80, acc_dps2=500, async_=False):
        """关节空间点到点（URDF 角，度）。vel_dps ∈ (0, 100] °/s（控制器各轴限速），超出报错。"""
        q1 = _v.vector("q_deg", q_deg, 6, "°")
        vel_dps = _v.number("vel_dps", vel_dps, 0, JOINT_VEL_MAX_DPS, lo_open=True, unit="°/s")
        acc_dps2 = _v.number("acc_dps2", acc_dps2, 0, lo_open=True, unit="°/s²")
        _v.flag("async_", async_)
        check_q_urdf(q1)
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "move_joints")
        q0 = [servo_to_joint(i, float(st.q[i])) for i in range(6)]
        samples, t_s = plan_joint_move(q0, q1, vel_dps, acc_dps2, DT_MS, URDF_VMAX_DPS)
        servo_samples = [self._to_servo7(s, st) for s in samples]
        job = self._submit_traj(servo_samples, t_s, desc="move_joints")
        if not async_:
            self._wait_job(job, t_s, "move_joints")
        return {"ok": True, "duration_s": float(t_s), "samples": len(samples)}

    def home(self, duration_s=6.0, async_=False):
        """回「全零位」（URDF 六关节全零，同事语义；五次多项式 duration_s 走完）。

        默认等走完再返回（与其它运动方法一致）。async_=True 时立即返回，调用方必须在
        close() 前 wait_idle()：轨迹由本进程逐帧下发，进程一退看门狗约 0.3 s 停臂。

        注意：这与原厂固件 HOME=[0,-90,89,0,0,0,0]（F7，舵机角）不是同一个姿势；
        URDF 全零 = 舵机全零 = 臂前伸（TCP ≈ [0.35, 0, 0.43] m）。执行前确认不碰桌面/自身。
        """
        t_s = _v.number("duration_s", duration_s, 0.5, unit="s")
        _v.flag("async_", async_)
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "home")
        q0 = [servo_to_joint(i, float(st.q[i])) for i in range(6)]
        q1 = [0.0] * 6
        maxd = max(abs(q1[i] - q0[i]) for i in range(6))
        vel = 1.875 * maxd / t_s
        acc = 5.7735 * maxd / (t_s * t_s)
        samples, _ = plan_joint_move(q0, q1, vel, acc, DT_MS, URDF_VMAX_DPS)
        servo_samples = [self._to_servo7(s, st) for s in samples]
        job = self._submit_traj(servo_samples, t_s, desc="home")
        if not async_:
            self._wait_job(job, t_s, "home")
        return {"ok": True, "duration_s": float(t_s), "samples": len(samples)}

    def park(self, vel_dps=20.0, async_=False):
        """慢速回折叠位（原厂 HOME 姿势，F7）后全轴失力——下电收臂用。

        目标舵机角 PARK_SERVO_DEG（J1/J2 留限位余量，见常量注释）。顺序：
        ① 夹爪判定：J6 电流 < PARK_GRIP_EMPTY_MA 且 |q_target[6]−q[6]|（换算 mm）
           ≤ PARK_GRIP_GAP_MM 判为空 → gripper(70) 张开（等走完）；否则判为夹着东西，
           抛 D1ArmError，一条运动命令都不发。
        ② 腕部回零：J3/J4/J5 → 0（都已在容差内则跳过）；J0/J1/J2 保持命令角。
        ③ 大小臂收回：J0/J1/J2 同时 → 折叠位目标，一段关节插值同步到达。
        ④ 停 PARK_SETTLE_S（0.5 s）→ free_all()（全轴含 J6 失力，臂靠在折叠位机械限位上）。
        不动的轴目标取控制器上报的命令角 q_target（NaN = 该轴失力/从未下令 → 退回实测角），
        不取实测 q（防负载轴棘轮下沉）。每段走完读回实测，各轴与段目标差 ≤
        PARK_TOLERANCE_DEG；超差或控制器 fault → 停下抛 D1ArmError，不继续、不失力。
        已在折叠位（J0–J5 实测都在容差内）：跳过 ②③，仍做 ① 和 ④。
        async_ 不支持 True：分段到位校验必须逐段在本进程确认。
        """
        vel_dps = _v.number("vel_dps", vel_dps, 0, JOINT_VEL_MAX_DPS, lo_open=True, unit="°/s")
        _v.only("async_", async_, False, "park 分段到位校验须逐段在本进程确认")
        target = [servo_to_joint(i, PARK_SERVO_DEG[i]) for i in range(6)]
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "park")

        def cmd_servo(s, i):  # 命令角（舵机角）；NaN（失力/从未下令）退回实测
            qt = float(s.q_target[i])
            return qt if math.isfinite(qt) else float(s.q[i])

        q_meas = [servo_to_joint(i, float(st.q[i])) for i in range(6)]
        q_cmd = [servo_to_joint(i, cmd_servo(st, i)) for i in range(6)]

        # ① 夹爪判定（不通过则一轴都不动）
        cur_ma = abs(int(st.current_ma[6]))
        gap_mm = abs(gripper_deg_to_mm(cmd_servo(st, 6)) - gripper_deg_to_mm(float(st.q[6])))
        if cur_ma >= PARK_GRIP_EMPTY_MA or gap_mm > PARK_GRIP_GAP_MM:
            raise D1ArmError(f"park 拒绝：夹爪可能夹着东西（J6 电流 {cur_ma} mA、"
                             f"命令与反馈差 {gap_mm:.1f} mm），请先松开/取下物体再 park")
        self.gripper(PARK_GRIP_OPEN_MM)
        grip = {"current_ma": cur_ma, "gap_mm": round(gap_mm, 2)}

        already = max(abs(q_meas[i] - target[i]) for i in range(6)) <= PARK_TOLERANCE_DEG
        segments = []
        if not already:
            wrist_ok = max(abs(q_meas[i] - target[i]) for i in (3, 4, 5)) <= PARK_TOLERANCE_DEG
            wrist = q_cmd[3:6] if wrist_ok else target[3:6]
            if not wrist_ok:
                segments.append(("1-腕部回零", q_cmd[0:3] + target[3:6]))
            segments.append(("2-大小臂收回", target[0:3] + wrist))
        done = []
        for name, seg in segments:
            started = time.monotonic()
            self.move_joints(seg, vel_dps=vel_dps)
            check = self._wait_state()
            err = max(abs(servo_to_joint(i, float(check.q[i])) - seg[i]) for i in range(6))
            if check.ctrl_state == CTRL_FAULT or check.fault:
                raise D1ArmError(f"park 第 {name} 段后控制器 fault=0x{int(check.fault):02x}，"
                                 "停止后续段（查 fault 位后 reset()）")
            if err > PARK_TOLERANCE_DEG:
                raise D1ArmError(f"park 第 {name} 段到位超差 {err:.1f}°"
                                 f"（容差 {PARK_TOLERANCE_DEG:.0f}°），停止后续段")
            done.append((name, round(err, 2), round(time.monotonic() - started, 2)))

        # ④ 停 0.5 s 再全轴失力
        time.sleep(PARK_SETTLE_S)
        self.free_all()
        return {"ok": True, "already_parked": already, "segments": done,
                "gripper": grip, "freed": True}

    def gripper(self, mm, vel_dps=J6_SPEED_DPS, grip_power_mw=0, async_=False):
        """夹爪开合到 mm（金属指间距，按 arm_model/d1t.json 标定表换算 J6 角；J6 以 ≤20°/s 走）。

        可达域 mm ∈ [4, 70]（标定表两端）、vel_dps ∈ (0, 20]，超出抛 D1ArmError、不动；
        grip_power_mw 只支持 0 = 舵机自保护兜底
        （夹持力暂用 P=0 舵机自保护 + 控制器温度守卫；P≤6000 在 20°/s 下接触冲击会触发功率位）。
        J0–J5 及 J6 起点取 _base7：命令角优先（本进程上次命令 → 控制器 q_target），
        仅失力轴退回实测，不把下沉锁进目标。
        """
        mm = _v.number("mm", mm, GRIPPER_TABLE[-1][1], GRIPPER_TABLE[0][1], unit="mm")
        vel = _v.number("vel_dps", vel_dps, 0, J6_SPEED_DPS, lo_open=True, unit="°/s")
        _v.only("grip_power_mw", grip_power_mw, 0, "夹持力暂由舵机自保护兜底")
        _v.flag("async_", async_)
        target_deg = gripper_mm_to_deg(mm)
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "gripper")
        j6_now = self._base7(st)[6]
        samples, t_s = plan_joint_move([j6_now], [target_deg], vel, 500.0, DT_MS,
                                       vel_max_dps=J6_SPEED_DPS)
        servo_samples = []
        base = self._base7(st)[:6]  # J0–J5 保持命令角（见 _base7），不取实测（防负载轴逐次下沉）
        for s in samples:
            servo_samples.append(base + [float(s[0])])
        job = self._submit_traj(servo_samples, t_s, desc=f"gripper({mm} mm)")
        if not async_:
            self._wait_job(job, t_s, "gripper")
        return {"ok": True, "duration_s": float(t_s), "samples": len(samples),
                "j6_deg": target_deg}

    # ---------- v1 保护/急停 ----------

    def free_all(self):
        """全轴 RELEASE（失力）。⚠ 臂会掉，先托住/撤走人员/物品！"""
        st = self._wait_state()
        job = self._submit_oneshot(MODE_RELEASE, (CTRL_RELEASED,), [float(x) for x in st.q],
                                   axis_mask=0x7F, desc="free_all(RELEASE)")
        self._wait_job(job, 0, "free_all")
        return {"ok": True}

    def lock_all(self):
        """全轴 HOLD（锁在当前姿态）。"""
        st = self._wait_state()
        job = self._submit_oneshot(MODE_HOLD, (CTRL_HOLD,), [float(x) for x in st.q],
                                   desc="lock_all(HOLD)")
        self._wait_job(job, 0, "lock_all")
        return {"ok": True}

    def estop(self):
        """软急停（ESTOP，恢复需 reset()）。中止进行中的轨迹/流式。实测位 111 ms、
        各轴停住 129 ms。"""
        with self._lock:
            old = self._job
            self._job = None
            self._stream_q = None
        if old is not None:
            old["reason"] = "estop 中止"
            old["done"].set()
        st = self._wait_state()
        job = self._submit_oneshot(MODE_ESTOP, (CTRL_ESTOP,), [float(x) for x in st.q],
                                   desc="estop", force=True)
        self._wait_job(job, 0, "estop")
        return {"ok": True}

    def reset(self):
        """软急停/故障恢复（RESET → IDLE/HOLD）。同事客户端没有此命令，
        为 ESTOP 恢复必需而加。"""
        st = self._wait_state()
        job = self._submit_oneshot(MODE_RESET, (CTRL_IDLE, CTRL_HOLD),
                                   [float(x) for x in st.q], desc="reset", force=True)
        self._wait_job(job, 0, "reset")
        return {"ok": True}

    # ---------- v1 流式 ----------

    def stream_start(self):
        """进入关节流式：先 HOLD 到当前姿态（清旧状态，同 send.py 流程），
        之后每帧发送 stream_joints 的最新目标。"""
        with self._lock:
            if self._job is not None:
                raise D1ArmError("AlreadyMoving：已有命令/轨迹在执行")
            if self._stream_q is not None:
                return {"ok": True}  # 已在流式，幂等
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "stream_start")
        q_now = self._base7(st)
        job = self._submit_oneshot(MODE_HOLD, (CTRL_HOLD,), q_now, desc="stream_start 前置 HOLD")
        self._wait_job(job, 0, "stream_start")
        with self._lock:
            self._stream_q = q_now
        return {"ok": True}

    def stream_joints(self, q_deg, j6_deg=None):
        """更新流式目标（URDF 角，度，6 关节）。下一帧起生效。

        j6_deg：J6 目标（舵机角，度，限位见 arm_model）。None = 保持上次命令角——不取实测角：
        夹住物体时实测角被顶开，拿它当目标会逐帧松开。流式中 J6 每帧步长受控制器限速
        （各轴 100°/s）钳制；gripper() 的 ≤20°/s 不作用于流式。"""
        q_deg = _v.vector("q_deg", q_deg, 6, "°")
        check_q_urdf(q_deg)
        if j6_deg is not None:
            j6_deg = _v.number("j6_deg（J6 舵机角）", j6_deg, *SERVO_LIMITS[6], unit="°")
        st = self._wait_state()
        with self._lock:
            if self._stream_q is None:
                raise D1ArmError("未在流式（先 stream_start()）")
            q7 = self._to_servo7(q_deg, st)
            if j6_deg is not None:
                q7[6] = j6_deg
            self._stream_q = q7
        return {"ok": True}

    def stream_stop(self):
        """退出流式并 HOLD（停流式目标；若什么都不发，看门狗约 0.3 s 也会停臂）。"""
        with self._lock:
            sq = self._stream_q
            self._stream_q = None
        q = sq if sq is not None else [float(x) for x in self._wait_state().q]
        job = self._submit_oneshot(MODE_HOLD, (CTRL_HOLD,), q, desc="stream_stop")
        self._wait_job(job, 0, "stream_stop")
        return {"ok": True}

    def stream_run(self, fn, max_frames=None):
        """按帧回调的流式（按顺序播放用：回放、ACT 动作块、脚本轨迹）。

        SDK 每收到一帧 ArmState 调一次 fn(k, state)：k = 第几帧（0 起），state = state() 字典；
        fn 返回 6 个 URDF 角（度，J6 保持）作为这一帧的目标，返回 None 结束。
        节奏由 SDK 跟臂走，调用方不写循环、不写 sleep。fn 须在 50 ms 内返回，
        超时的帧计入 late_frames（那一帧控制器沿用上一目标）。
        结束（含 fn 抛异常）一律 stream_stop → HOLD。遥操作「取最新值」仍用 stream_joints。"""
        if not callable(fn):
            raise D1ArmError(f"参数 fn 须为可调用对象 fn(k, state)，收到 {fn!r}")
        if max_frames is not None and (isinstance(max_frames, bool) or not isinstance(
                max_frames, int) or max_frames < 1):
            raise D1ArmError(f"参数 max_frames 须为 None 或 ≥1 的整数，收到 {max_frames!r}")
        self.stream_start()
        k = late = 0
        try:
            while max_frames is None or k < max_frames:
                with self._frame_cv:
                    n0 = self._frame_no
                q = fn(k, self.state())
                if q is None:
                    break
                with self._frame_cv:
                    if self._frame_no > n0 + 1:
                        late += 1
                self.stream_joints(q)
                self.wait_frame()
                k += 1
        finally:
            self.stream_stop()
        return {"ok": True, "frames": k, "late_frames": late}

    # ---------- v2 运动学 ----------

    def fk(self, q_deg) -> dict:
        """正运动学：URDF 角（度，6）→ {"pos"(m), "quat"[w,x,y,z], "rpy"(rad)}。"""
        q_deg = _v.vector("q_deg", q_deg, 6, "°")
        t = self._kin.fk([deg2rad(x) for x in q_deg])
        return to_pose_dict(t)

    def ik(self, pos, rpy=None, quat=None, seed=None) -> list:
        """逆运动学：pos(m) + rpy(rad)/quat([w,x,y,z]) → URDF 角（度，6）。

        seed：种子关节角（度），缺省用当前实测角；解在 F7 换算的 URDF 限位内。
        """
        pos, rpy, quat = _v.pose(pos, rpy, quat)
        if seed is not None:
            seed = _v.vector("seed", seed, 6, "°")
        target = from_pose_dict(pos, rpy=rpy, quat=quat)
        if seed is None:
            st = self._wait_state()
            seed_deg = [servo_to_joint(i, float(st.q[i])) for i in range(6)]
        else:
            seed_deg = seed
        lims = urdf_limits()
        qmin = [deg2rad(lo) for lo, _ in lims]
        qmax = [deg2rad(hi) for _, hi in lims]
        res = self._kin.ik(target, [deg2rad(q) for q in seed_deg], qmin, qmax, IkParams())
        if not res.success:
            raise D1ArmError(f"IK 无解（位置误差 {res.pos_err * 1000:.1f} mm，"
                             f"姿态误差 {rad2deg(res.rot_err):.2f}°）")
        return [rad2deg(q) for q in res.q_rad]

    def move_j(self, pos, rpy=None, quat=None, vel_dps=80, acc_dps2=500, async_=False):
        """笛卡尔点到点：IK 一次后走关节空间轨迹（同事 moveJointTo 语义）。"""
        pos, rpy, quat = _v.pose(pos, rpy, quat)
        vel_dps = _v.number("vel_dps", vel_dps, 0, JOINT_VEL_MAX_DPS, lo_open=True, unit="°/s")
        acc_dps2 = _v.number("acc_dps2", acc_dps2, 0, lo_open=True, unit="°/s²")
        _v.flag("async_", async_)
        q_target = self.ik(pos, rpy=rpy, quat=quat)
        check_q_urdf(q_target)
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "move_j")
        q0 = [servo_to_joint(i, float(st.q[i])) for i in range(6)]
        samples, t_s = plan_joint_move(q0, q_target, vel_dps, acc_dps2, DT_MS, URDF_VMAX_DPS)
        servo_samples = [self._to_servo7(s, st) for s in samples]
        job = self._submit_traj(servo_samples, t_s, desc="move_j")
        if not async_:
            self._wait_job(job, t_s, "move_j")
        return {"ok": True, "duration_s": float(t_s), "samples": len(samples),
                "q_deg": [round(q, 3) for q in q_target]}

    def move_l(self, pos, rpy=None, quat=None, vel=0.15, acc=2.0, async_=False):
        """直线：当前位姿 → 目标位姿，末端走直线（笛卡尔逐点 IK + 五次多项式）。
        vel m/s（期望平均末端速度，峰值 ≈1.875×vel）、acc m/s²。"""
        pos, rpy, quat = _v.pose(pos, rpy, quat)
        vel = _v.number("vel", vel, 0, lo_open=True, unit="m/s")
        acc = _v.number("acc", acc, 0, lo_open=True, unit="m/s²")
        _v.flag("async_", async_)
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "move_l")
        q0 = [servo_to_joint(i, float(st.q[i])) for i in range(6)]
        pose_a = self._kin.fk([deg2rad(q) for q in q0])
        pose_b = from_pose_dict(pos, rpy=rpy, quat=quat)
        lims = urdf_limits()
        lims_rad = ([deg2rad(lo) for lo, _ in lims], [deg2rad(hi) for _, hi in lims])
        samples, t_s = plan_line(self._kin, pose_a, pose_b, [deg2rad(q) for q in q0],
                                 lims_rad, vel, acc, DT_MS, URDF_VMAX_DPS)
        for s in samples:
            check_q_urdf(s)
        servo_samples = [self._to_servo7(s, st) for s in samples]
        job = self._submit_traj(servo_samples, t_s, desc="move_l")
        if not async_:
            self._wait_job(job, t_s, "move_l")
        return {"ok": True, "duration_s": float(t_s), "samples": len(samples)}

    def move_c(self, pos, via, rpy=None, quat=None, via_rpy=None, via_quat=None,
               vel=0.15, acc=2.0, async_=False):
        """圆弧：当前位姿 → 经 via → 目标位姿（三点定圆；同事客户端缺此命令，按
        其服务端 planArc 补上）。参数含义同 move_l。"""
        pos, rpy, quat = _v.pose(pos, rpy, quat)
        via, via_rpy, via_quat = _v.pose(via, via_rpy, via_quat, ("via", "via_rpy", "via_quat"))
        vel = _v.number("vel", vel, 0, lo_open=True, unit="m/s")
        acc = _v.number("acc", acc, 0, lo_open=True, unit="m/s²")
        _v.flag("async_", async_)
        st = self._wait_state()
        self._check_ctrl_for_motion(st, "move_c")
        q0 = [servo_to_joint(i, float(st.q[i])) for i in range(6)]
        pose_a = self._kin.fk([deg2rad(q) for q in q0])
        pose_via = from_pose_dict(via, rpy=via_rpy, quat=via_quat)
        pose_b = from_pose_dict(pos, rpy=rpy, quat=quat)
        lims = urdf_limits()
        lims_rad = ([deg2rad(lo) for lo, _ in lims], [deg2rad(hi) for _, hi in lims])
        samples, t_s = plan_arc(self._kin, pose_a, pose_via, pose_b,
                                [deg2rad(q) for q in q0], lims_rad, vel, acc, DT_MS,
                                URDF_VMAX_DPS)
        for s in samples:
            check_q_urdf(s)
        servo_samples = [self._to_servo7(s, st) for s in samples]
        job = self._submit_traj(servo_samples, t_s, desc="move_c")
        if not async_:
            self._wait_job(job, t_s, "move_c")
        return {"ok": True, "duration_s": float(t_s), "samples": len(samples)}

    def stream_pose(self, pos, rpy=None, quat=None, j6_deg=None):
        """流式目标改成位姿（IK 后走 stream_joints；遥操作用）。j6_deg 同 stream_joints。"""
        pos, rpy, quat = _v.pose(pos, rpy, quat)
        if j6_deg is not None:
            j6_deg = _v.number("j6_deg（J6 舵机角）", j6_deg, *SERVO_LIMITS[6], unit="°")
        q_deg = self.ik(pos, rpy=rpy, quat=quat)
        return self.stream_joints(q_deg, j6_deg=j6_deg)

    # ---------- 生命周期 ----------

    def close(self):
        """停止发送线程。若在流式中：停止发送后控制器看门狗约 0.3 s 停臂。"""
        with self._lock:
            old = self._job
            self._closed = True
            self._stream_q = None
        if old is not None:
            old["reason"] = old["reason"] or "close() 中止"
            old["done"].set()
        self._thread.join(timeout=2.0)
        self._transport.close()
        return {"ok": True}


CLI_USAGE = ("用法: python3 -m d1t_sdk state | ping | home | park | free | lock | estop | reset"
             " | joint q0,q1,q2,q3,q4,q5 | line x,y,z[,roll,pitch,yaw] | gripper mm")
_CLI_NOARG = ("state", "ping", "home", "park", "free", "lock", "estop", "reset")
_CLI_COUNTS = {"joint": (6,), "line": (3, 6), "gripper": (1,)}  # 逗号分隔的数值个数


def _cli_parse(args):
    """命令行参数 → (op, 数值列表)；个数/格式不符抛 D1ArmError（此时还没连臂）。"""
    if not args:
        raise D1ArmError("缺少指令")
    op = args[0]
    if op in _CLI_NOARG:
        if len(args) != 1:
            raise D1ArmError(f"{op} 不带参数，收到 {len(args) - 1} 个：{' '.join(args[1:])}")
        return op, []
    if op not in _CLI_COUNTS:
        raise D1ArmError(f"未知指令 {op!r}")
    if len(args) != 2:
        raise D1ArmError(f"{op} 需要 1 个参数（逗号分隔、无空格），收到 {len(args) - 1} 个")
    counts = _CLI_COUNTS[op]
    parts = args[1].split(",")
    if len(parts) not in counts:
        want = " 或 ".join(str(c) for c in counts)
        raise D1ArmError(f"{op} 参数须为 {want} 个逗号分隔的数，收到 {len(parts)} 个：{args[1]!r}")
    vals = []
    for k, text in enumerate(parts):
        try:
            x = float(text)
        except ValueError:
            x = math.nan
        if not math.isfinite(x):
            raise D1ArmError(f"{op} 参数第 {k + 1} 个值 {text!r} 不是有限数字")
        vals.append(x)
    return op, vals


def _cli():
    """命令行入口（同同事 _cli，IP 换成 DDS 环境变量 CYCLONEDDS_URI）。

    用法: python3 -m d1t_sdk <op> [args]
      state | ping | home | park | free | lock | estop | reset | gripper <mm>
      joint <q0,...,q5>   line <x,y,z[,roll,pitch,yaw]>
    参数个数/格式错 → stderr 打「失败：原因」+ 用法行，退出码 2，不连臂。
    """
    import json
    import sys

    try:
        op, vals = _cli_parse(sys.argv[1:])
    except D1ArmError as exc:
        print(f"失败：{exc}", file=sys.stderr)
        print(CLI_USAGE, file=sys.stderr)
        return 2
    arm = None
    try:
        arm = D1Arm()
        if op == "state":
            print(json.dumps(arm.state(), ensure_ascii=False, indent=1))
        elif op == "ping":
            print("online:", arm.ping())
        elif op == "home":
            # 轨迹由本进程逐帧下发：必须等走完再退出，否则发送端一关看门狗就停臂
            arm.home(4.0)
            print("done:", [round(x, 2) for x in arm.state()["q_deg"]])
        elif op == "park":
            # 下电收臂：夹爪判定→腕部回零→大小臂收回→停 0.5 s 失力，断电由人做
            result = arm.park()
            if result["already_parked"]:
                print("已在折叠位，未动")
            for name, err, dur in result["segments"]:
                print(f"park {name}: 到位偏差 {err}°，用时 {dur}s")
            print("已到折叠位并已失力，可以下电")
        elif op == "free":
            arm.free_all()
            print("freed（臂已失力）")
        elif op == "lock":
            arm.lock_all()
            print("locked")
        elif op == "estop":
            arm.estop()
            print("ESTOP（恢复：python3 -m d1t_sdk reset）")
        elif op == "reset":
            arm.reset()
            print("reset")
        elif op == "joint":
            arm.move_joints(vals)
            arm.wait_idle()
            print("done:", [round(x, 2) for x in arm.state()["q_deg"]])
        elif op == "line":
            arm.move_l(vals[:3], rpy=vals[3:6] if len(vals) > 3 else None)
            arm.wait_idle()
            print("tcp:", [round(v, 4) for v in arm.state()["tcp"]["pos"]])
        elif op == "gripper":
            arm.gripper(vals[0])
            print("gripper ->", f"{vals[0]:g}", "mm")
    except (RuntimeError, ValueError, IndexError) as exc:  # D1ArmError 是 RuntimeError 子类
        # 规划失败 / 参数越界：打一行原因，非零退出码；臂不会因此而动
        print(f"失败：{exc}", file=sys.stderr)
        return 2
    finally:
        if arm is not None:
            arm.close()
    return 0
