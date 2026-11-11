# -*- coding: utf-8 -*-
"""假控制器回放测试（离线，无 DDS）：FakeCtrl 模拟 d1t_arm_ctrl 的信箱/状态机，
验证帧同步发送节拍（每帧一条）、丢帧按 seq 差补偿样本序号、单发命令重发直至确认、
夹爪只动 J6、限速钳制、estop/reset 语义。"""
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d1t_sdk import D1Arm, D1ArmError
from d1t_sdk._common import AXIS_SPEED_DPS, CTRL_IDLE, URDF_VMAX_DPS, ArmState_
from d1t_sdk._joint_model import gripper_mm_to_deg, joint_to_servo, servo_to_joint

HOME_Q = [0.0, -90.0, 89.0, 0.0, 0.0, 0.0, 29.0]  # 舵机角（F7 HOME + J6 29°）


class FakeCtrl:
    """模拟 d1t_arm_ctrl：记录 ArmCmd，按命令翻转 ctrl_state；可丢帧/不认命令。"""

    def __init__(self):
        self.cmds = []
        self.ctrl_state = CTRL_IDLE
        self._queue = []
        self._lk = threading.Lock()
        self.q = list(HOME_Q)
        self.drop_next_cmds = 0       # >0 时吞掉接下来 N 条命令（不翻状态）
        self.delay_estop_frames = 0   # estop 确认前额外收到的帧数

    def emit(self, seq):
        with self._lk:
            self._queue.append((seq, self.ctrl_state, list(self.q)))

    def poll_frames(self):
        with self._lk:
            out, self._queue = self._queue, []
        frames = []
        for seq, cs, q in out:
            frames.append(ArmState_(
                seq=seq, stamp_ns=time.time_ns(), sample_span_us=15300,
                ctrl_state=cs, cmd_seq_ack=0, cmd_stamp_ns=0, cmd_rx_ns=0,
                valid_mask=0x7F, q=[float(x) for x in q], q_target=[float(x) for x in q],  # 假定在原位保持
                current_ma=[10] * 7, voltage_mv=[12000] * 7, temp_c=[27.0] * 7,
                status=[0] * 7, turns=[0] * 7, fault=0, warn=0, overruns=0))
        return frames

    def send_cmd(self, cmd):
        with self._lk:
            if self.drop_next_cmds > 0:
                self.drop_next_cmds -= 1
                return  # 静默丢命令（BEST_EFFORT）
            self.cmds.append(cmd)
            m = cmd.mode
            if m == 0:
                self.ctrl_state = 2   # STREAM
            elif m == 1:
                self.ctrl_state = 3   # HOLD
            elif m == 2:
                self.ctrl_state = 4   # RELEASE
            elif m == 3:
                self.ctrl_state = 4   # DAMP → RELEASED
            elif m == 4:
                if self.delay_estop_frames > 0:
                    self.delay_estop_frames -= 1
                    return
                self.ctrl_state = 5   # ESTOP
            elif m == 5:
                self.ctrl_state = 1   # RESET → IDLE
            self.q = [float(x) for x in cmd.q]

    def stream_cmds(self):
        return [c for c in self.cmds if c.mode == 0]

    def holds(self):
        return [c for c in self.cmds if c.mode == 1]

    def close(self):
        pass  # 与 _RealTransport.close() 同接口


class Emitter:
    """按 seq 连续发帧（间隔可调）；可指定丢帧的 seq 集合。"""

    def __init__(self, fc, start_seq=1, interval_s=0.004, drop=()):
        self.fc = fc
        self.seq = start_seq - 1
        self.interval_s = interval_s
        self.drop = set(drop)
        self.stop_evt = threading.Event()
        self.emitted = 0
        self.th = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self.stop_evt.is_set():
            self.seq += 1
            if self.seq not in self.drop:
                self.fc.emit(self.seq)
                self.emitted += 1
            self.stop_evt.wait(self.interval_s)

    def start(self):
        self.th.start()
        return self

    def stop(self):
        self.stop_evt.set()
        self.th.join(timeout=2.0)


def make_arm(fc):
    arm = D1Arm(transport=fc)
    e = Emitter(fc).start()
    # 等第一帧进缓存
    t0 = time.monotonic()
    while arm._state is None and time.monotonic() - t0 < 2.0:
        time.sleep(0.005)
    assert arm._state is not None, "假控制器没发出帧"
    return arm, e


def em_wait_frames(arm, n):
    """等 SDK 再收到 n 帧（让假控制器改过的 q 进缓存）。"""
    seq0 = arm._state.seq
    t0 = time.monotonic()
    while arm._state.seq < seq0 + n and time.monotonic() - t0 < 2.0:
        time.sleep(0.005)


def sync_ctrl_state(fc, arm, emitter, cs):
    """停发流、设 ctrl_state、发一帧并等 SDK 缓存更新（消除测试竞态）。"""
    emitter.stop()
    fc.ctrl_state = cs
    fc.emit(100000)
    t0 = time.monotonic()
    while (arm._state is None or arm._state.ctrl_state != cs) and time.monotonic() - t0 < 1.0:
        time.sleep(0.005)
    assert arm._state is not None and arm._state.ctrl_state == cs, "缓存状态没跟上"


class TestStreamExecutor(unittest.TestCase):
    def test_move_joints_frame_sync_and_drop_compensation(self):
        """每帧恰一条命令；seq 丢 3 帧 → 样本下标跳 4（波形时间保持）；收尾 HOLD。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            em.stop()
            em2 = Emitter(fc, start_seq=em.seq + 1, interval_s=0.004,
                          drop={8, 9, 10}).start()  # 中途丢 3 帧
            r = arm.move_joints([0, -40, 60, 0, 30, 0], vel_dps=30, acc_dps2=100)
            em2.stop()
            streams = fc.stream_cmds()
            holds = fc.holds()
            # 1. 帧同步：除「样本 0 热身重发」外每样本至多一次；
            #    总条数 = 样本数 − 丢的 3 个 + 热身重发（0–3 次，取决于帧排队相位）
            self.assertGreaterEqual(len(streams), r["samples"] - 3)
            self.assertLessEqual(len(streams), r["samples"] - 3 + 3)
            # 2. 收尾恰好 1 条 HOLD，q = 最后样本
            self.assertEqual(len(holds), 1)
            self.assertAlmostEqual(holds[0].q[1], -40.0, places=6)
            # 3. stamp 严格递增、seq 递增、dt_ms=50
            stamps = [c.stamp_ns for c in fc.cmds]
            seqs = [c.seq for c in fc.cmds]
            self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])))
            self.assertTrue(all(b > a for a, b in zip(seqs, seqs[1:])))
            self.assertEqual({c.dt_ms for c in fc.cmds}, {50})
            # 4. 丢帧补偿：把每条 STREAM 命令对回规划样本的下标，
            #    期望 = [0,1,2,3] + [7,8,...,N-1]（seq 8,9,10 丢帧 → 下标 4,5,6 跳过）
            from d1t_sdk._trajectory import plan_joint_move
            q0 = [joint_to_servo(0, 0.0), HOME_Q[1], HOME_Q[2], joint_to_servo(3, 0.0),
                  HOME_Q[4], HOME_Q[5]]  # = servo 空间起点 [0,-90,89,0,0,0]
            q1 = [joint_to_servo(i, q) for i, q in enumerate([0, -40, 60, 0, 30, 0])]
            samples, _ = plan_joint_move(q0, q1, 30, 100, 50.0, URDF_VMAX_DPS)
            n = len(samples)
            idxs = []
            for c in streams:
                cq = [float(x) for x in c.q]
                match = [k for k, s in enumerate(samples)
                         if all(abs(cq[i] - float(s[i])) < 1e-4 for i in range(6))]
                self.assertEqual(len(match), 1, f"命令 q 对不上任何样本：{cq}")
                idxs.append(match[0])
            # 下标序列：首条必是样本 0；先若干个 0（热身重发），之后严格递增、
            # 只跳过连续 3 个样本（丢帧）；末条 = 最后样本
            self.assertEqual(idxs[0], 0)
            warmups = 0
            while warmups < len(idxs) and idxs[warmups] == 0:
                warmups += 1
            body = idxs[warmups:]
            self.assertTrue(all(b > a for a, b in zip(body, body[1:])), f"下标不严格递增：{body}")
            skipped = [i for i in range(n) if i not in idxs]  # 全程从未发出的样本
            print(f"  丢帧补偿: STREAM={len(streams)}（样本 {n}，热身重发 {warmups - 1}），"
                  f"跳过样本 {skipped}")
            self.assertEqual(len(skipped), 3, f"应恰好跳过 3 个样本，实际 {skipped}")
            self.assertEqual(skipped, list(range(skipped[0], skipped[0] + 3)), "跳过应连续")
            self.assertEqual(body[-1], n - 1, "最后样本必须发到")
            # 5. 终点：舵机角精确（URDF→servo 换算后）
            for i in range(6):
                self.assertAlmostEqual(fc.q[i], q1[i], places=6)
            self.assertAlmostEqual(fc.q[6], HOME_Q[6], places=6)  # J6 不动
            # 6. 完成后 wait_idle
            for s in range(em2.seq + 1, em2.seq + 4):
                fc.emit(s)
                time.sleep(0.01)
            self.assertTrue(arm.wait_idle(2))
        finally:
            arm.close()

    def test_oneshot_resend_until_confirmed(self):
        """ESTOP 前几条被吞 → 每帧重发（新 seq/stamp），确认后停止发送。"""
        fc = FakeCtrl()
        fc.delay_estop_frames = 3  # 前 3 条 ESTOP 被吞
        arm, em = make_arm(fc)
        try:
            arm.estop()
            estops = [c for c in fc.cmds if c.mode == 4]
            seqs = [c.seq for c in estops]
            stamps = [c.stamp_ns for c in estops]
            print(f"  estop 重发: {len(estops)} 条 ESTOP（吞 3 后第 4 条确认）")
            self.assertGreaterEqual(len(estops), 4)
            self.assertTrue(all(b > a for a, b in zip(seqs, seqs[1:])))
            self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])))
            self.assertEqual(fc.ctrl_state, 5)
            n_after = len(fc.cmds)
            time.sleep(0.05)
            em.stop()
            self.assertEqual(len(fc.cmds), n_after, "确认后不应再发命令")
        finally:
            arm.close()

    def test_gripper_moves_only_j6_at_20dps(self):
        """gripper(mm)：q[0:6] 保持不动，q[6] → 标定表角；速度 ≤20°/s。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            r = arm.gripper(45.0)
            streams = fc.stream_cmds()
            self.assertAlmostEqual(r["j6_deg"], gripper_mm_to_deg(45.0), places=6)
            for c in streams[:-1]:
                for i in range(6):
                    self.assertAlmostEqual(c.q[i], HOME_Q[i], places=6)
            self.assertAlmostEqual(streams[-1].q[6], gripper_mm_to_deg(45.0), places=6)
            span = abs(gripper_mm_to_deg(45.0) - HOME_Q[6])
            t_min = 1.875 * span / 20.0
            self.assertGreaterEqual(r["duration_s"], t_min - 0.06,
                                    "J6 速度不得超过 20°/s")
            print(f"  gripper: 45mm → {r['j6_deg']:.2f}°, T={r['duration_s']:.2f}s "
                  f"(≥{t_min:.2f}s), STREAM={len(streams)}")
        finally:
            arm.close()

    def test_move_joints_rejected_in_estop(self):
        """ESTOP 状态下发运动 → 立即报错提示 reset，不发命令。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            sync_ctrl_state(fc, arm, em, 5)
            with self.assertRaises(D1ArmError) as ctx:
                arm.move_joints([0, -40, 60, 0, 30, 0])
            self.assertIn("reset", str(ctx.exception))
            self.assertEqual(len(fc.cmds), 0)
        finally:
            arm.close()

    def test_reset_recovers(self):
        """estop → reset → ctrl_state 回 IDLE，可再运动。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            arm.estop()
            self.assertEqual(fc.ctrl_state, 5)
            arm.reset()
            self.assertEqual(fc.ctrl_state, 1)
            r = arm.move_joints([0, -80, 70, 0, 10, 0], vel_dps=30, acc_dps2=100)
            self.assertTrue(r["ok"])
            want = [joint_to_servo(i, q) for i, q in enumerate([0, -80, 70, 0, 10, 0])]
            for i in range(6):
                self.assertAlmostEqual(fc.q[i], want[i], places=6)
        finally:
            arm.close()

    def test_stream_mode(self):
        """stream_start → stream_joints 每帧发最新目标 → stream_stop HOLD。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            arm.stream_start()
            self.assertEqual(fc.ctrl_state, 3)  # 前置 HOLD 生效
            n0 = len(fc.cmds)
            time.sleep(0.03)
            n1 = len(fc.cmds)
            self.assertGreater(n1, n0, "流式中每帧都发")
            arm.stream_joints([10, -70, 70, 5, 20, 5])
            time.sleep(0.03)
            latest = fc.cmds[-1]
            want = [joint_to_servo(i, q) for i, q in enumerate([10, -70, 70, 5, 20, 5])]
            for i in range(6):
                self.assertAlmostEqual(latest.q[i], want[i], places=6)
            arm.stream_stop()
            self.assertEqual(len(fc.holds()) >= 1, True)
            self.assertEqual(fc.ctrl_state, 3)
            n2 = len(fc.cmds)
            time.sleep(0.05)
            em.stop()
            self.assertEqual(len(fc.cmds), n2, "stop 后不再发命令")
        finally:
            arm.close()

    def test_speed_at_controller_limit(self):
        """vel_dps 取上限（各轴限速），请求 J0 走 90° → 实际下发相邻命令差 ≤ 臂模型限速×0.05s+余量。
        （超过上限的 vel_dps 现在报错，见 test_param_validation.py）"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            vel = min(AXIS_SPEED_DPS[:6])  # 用户可给的最大值
            arm.move_joints([90, -90, 89, 0, 0, 0], vel_dps=vel, acc_dps2=500)
            streams = fc.stream_cmds()
            # 逐轴检查相邻命令角速度（首尾段五次多项式慢、中段最快）
            for i in range(6):
                lim = AXIS_SPEED_DPS[i]
                peak = max(abs(b.q[i] - a.q[i]) for a, b in zip(streams, streams[1:]))
                self.assertLessEqual(peak, lim * 0.05 + 0.15,
                                     f"J{i} 峰值步进 {peak:.3f}°/50ms 超 {lim}°/s")
        finally:
            arm.close()

    def test_limit_precheck(self):
        """目标越 F7 限位 → 报错带关节号，不发命令。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            with self.assertRaises(D1ArmError) as ctx:
                arm.move_joints([170, 0, 0, 0, 0, 0])  # URDF J0 上限 163
            self.assertIn("J0", str(ctx.exception))
            self.assertEqual(len(fc.cmds), 0)
        finally:
            arm.close()

    def test_idle_axes_keep_commanded_not_measured(self):
        """防棘轮：轨迹后负载轴实测下沉，gripper 的 J0–J5 仍发上次命令角；
        动臂时 J6 仍发上次命令角；失力后退回实测。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            target = [0, -40, 60, 0, 30, 0]
            arm.move_joints(target, vel_dps=60)
            cmd = [joint_to_servo(i, q) for i, q in enumerate(target)]
            with fc._lk:
                fc.q[2] += 0.9    # 模拟 J2 重力下沉
                fc.q[6] += 3.0    # 模拟 J6 被物体顶开
            em_wait_frames(arm, 3)
            n0 = len(fc.cmds)
            arm.gripper(45.0)
            g = [c for c in fc.cmds[n0:] if c.mode == 0]
            self.assertTrue(g)
            for c in g:
                for i in range(6):
                    self.assertAlmostEqual(c.q[i], cmd[i], places=4)
            j6_cmd = g[-1].q[6]
            with fc._lk:
                fc.q[6] = j6_cmd + 3.0
            em_wait_frames(arm, 3)
            n1 = len(fc.cmds)
            arm.move_joints([0, -45, 60, 0, 30, 0], vel_dps=60)
            for c in [c for c in fc.cmds[n1:] if c.mode == 0]:
                self.assertAlmostEqual(c.q[6], j6_cmd, places=4)
            arm.free_all()
            with fc._lk:
                fc.q[2] = 5.0
            em_wait_frames(arm, 3)
            arm.lock_all()
            n2 = len(fc.cmds)
            arm.gripper(70.0)
            self.assertAlmostEqual([c for c in fc.cmds[n2:] if c.mode == 0][0].q[2], 5.0, places=4)
        finally:
            arm.close()

    def test_stream_joints_with_j6(self):
        """stream_joints/stream_pose 带 j6_deg：J6 随给定值；j6_deg=None 保持上次命令角
        （实测角被顶开也不跟）；J0–J5 不受 j6_deg 影响；越界报错。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            arm.stream_start()
            arm.stream_joints([0, -40, 60, 0, 30, 0], j6_deg=-10.0)
            em_wait_frames(arm, 3)
            want = [joint_to_servo(i, q) for i, q in enumerate([0, -40, 60, 0, 30, 0])]
            last = fc.cmds[-1]
            for i in range(6):
                self.assertAlmostEqual(last.q[i], want[i], places=6)  # J6 不影响 J0–J5
            self.assertAlmostEqual(last.q[6], -10.0, places=6)
            # 实测角被物体顶开（差 5°）：j6_deg=None 仍发上次命令角，不取实测
            with fc._lk:
                fc.q[6] = -5.0
            em_wait_frames(arm, 3)
            arm.stream_joints([10, -40, 60, 0, 30, 0])
            em_wait_frames(arm, 3)
            last = fc.cmds[-1]
            self.assertAlmostEqual(last.q[6], -10.0, places=6,
                                   msg="j6_deg=None 应保持上次命令角，不得用实测角")
            self.assertAlmostEqual(last.q[0], joint_to_servo(0, 10.0), places=6)
            # stream_pose 带 j6：IK 解进 J0–J5（FK 回到目标位姿），J6 用给定值
            pose = arm.fk([0, -40, 60, 0, 30, 0])
            arm.stream_pose(pose["pos"], rpy=pose["rpy"], j6_deg=30.0)
            em_wait_frames(arm, 3)
            last = fc.cmds[-1]
            self.assertAlmostEqual(last.q[6], 30.0, places=6)
            got = arm.fk([servo_to_joint(i, last.q[i]) for i in range(6)])
            for a, b in zip(got["pos"], pose["pos"]):
                self.assertAlmostEqual(a, b, delta=1e-3)  # IK 容差内（≤1 mm）
            # 越界（J6 限位约 −30…64.5°）报错且不改当前目标
            with self.assertRaises(D1ArmError) as ctx:
                arm.stream_joints([0, -40, 60, 0, 30, 0], j6_deg=70.0)
            self.assertIn("J6", str(ctx.exception))
            em_wait_frames(arm, 3)
            self.assertAlmostEqual(fc.cmds[-1].q[6], 30.0, places=6)
            arm.stream_stop()
        finally:
            arm.close()

    def test_wait_frame_one_target_per_frame(self):
        """stream_joints + wait_frame：每个目标恰好发出一次（不漏、不重）。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            arm.stream_start()
            n0 = len(fc.cmds)
            for i in range(20):
                arm.stream_joints([float(i), -40, 60, 0, 30, 0])
                arm.wait_frame()
            arm.stream_stop()
            j0 = [round(servo_to_joint(0, c.q[0]), 6) for c in fc.cmds[n0:] if c.mode == 0]
            self.assertEqual(j0, [float(i) for i in range(20)])
        finally:
            arm.close()

    def test_stream_run_one_target_per_frame(self):
        """stream_run：第 k 帧发第 k 个目标，不漏不重；None 结束后 HOLD；fn 异常也收尾 HOLD。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            n0 = len(fc.cmds)
            r = arm.stream_run(lambda k, st: None if k >= 15 else [float(k), -40, 60, 0, 30, 0])
            self.assertEqual(r["frames"], 15)
            j0 = [round(servo_to_joint(0, c.q[0]), 6) for c in fc.cmds[n0:] if c.mode == 0]
            self.assertEqual(j0[-15:], [float(i) for i in range(15)])
            self.assertEqual(fc.ctrl_state, 3)

            def boom(k, st):
                if k == 3:
                    raise ValueError("fn 出错")
                return [0.0, -40, 60, 0, 30, 0]
            with self.assertRaises(ValueError):
                arm.stream_run(boom)
            self.assertEqual(fc.ctrl_state, 3)
        finally:
            arm.close()

    def test_mode_name_unmerged(self):
        """mode 不合并：HOLD/ESTOP/RELEASED 各自原名；轨迹中为 MOVING。"""
        from d1t_sdk._common import mode_name
        self.assertEqual([mode_name(i) for i in range(7)],
                         ["INIT", "IDLE", "STREAMING", "HOLD", "RELEASED", "ESTOP", "FAULT"])
        self.assertEqual(mode_name(3, job_active=True), "MOVING")

    def test_state_dict_keys(self):
        """state() 键与 fillState 对齐 + 我们独有字段。"""
        fc = FakeCtrl()
        arm, em = make_arm(fc)
        try:
            st = arm.state()
            for k in ("q_deg", "gripper_mm", "tcp", "mode", "voltage_mv", "online",
                      "stamp_ms", "seq", "fault", "warn", "temp_c", "current_ma",
                      "servo_raw_deg", "calibrated", "ctrl_state"):
                self.assertIn(k, st)
            self.assertEqual(len(st["q_deg"]), 6)
            self.assertEqual(len(st["online"]), 7)
            self.assertEqual(st["mode"], "IDLE")
            self.assertEqual(st["online"], [True] * 7)
            self.assertAlmostEqual(st["gripper_mm"], gripper_deg_to_mm_check(29.0),
                                   places=6)
        finally:
            arm.close()


def gripper_deg_to_mm_check(deg):
    from d1t_sdk._joint_model import gripper_deg_to_mm
    return gripper_deg_to_mm(deg)


if __name__ == "__main__":
    unittest.main()
