#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TeleopLoop 离线测试（FakeCtrl 假臂，复用 sdk/tests 的装置；无 DDS 网络、无 TeleRobot）。

覆盖：接合不跳、脱开保持、STALE 停流、步长截断、逆解失败保持；
另测工作空间盒钳制、--fake-grasp-at 锁定通路、朝向修正（heading）、标定计算、ESTOP 停机。

备注：FakeCtrl 的 HOME 是折叠位（J2 顶在 89° 限位），朝 +X 的 IK 无解——测试先用
move_joints 摆到中部姿态 [0,-40,60,0,30,0] 再接合（全链路/实机同理：从可用位姿起步）；
逆解失败保持行为正由此姿态实测过。

运行（需 sdk 的依赖；在仓库根或任意目录均可）：
    python3 tools/pico_teleop/test_teleop_loop.py -v
"""
import sys
import unittest
from collections import namedtuple
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent          # tools/pico_teleop（目录：parents[0]=tools，parents[1]=仓库根）
for _p in (str(HERE), str(HERE.parents[1] / "sdk"), str(HERE.parents[1] / "sdk" / "tests")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import test_stream_executor as tse            # noqa: E402  sdk/tests 的 FakeCtrl 装置
from d1t_sdk import D1ArmError                # noqa: E402
from d1t_sdk._joint_model import servo_to_joint  # noqa: E402

from griplock import CLOSE_SPEED_DPS  # noqa: E402
from teleop_loop import DT_S, TeleopLoop, yaw_from_push  # noqa: E402

STEP = CLOSE_SPEED_DPS * DT_S   # 默认闭合每帧步长（°）

PoseTarget = namedtuple("PoseTarget", "position rotation")
START_Q_URDF = [0.0, -40.0, 60.0, 0.0, 30.0, 0.0]   # 中部起步姿态（同 pico_probe 假姿态）


def make_status(state, target=None, trigger=0.0):
    return {"state": state, "target": target, "trigger": trigger}


def tcp_pose(arm):
    st = arm.state()
    w, x, y, z = st["tcp"]["quat"]
    return np.array(st["tcp"]["pos"], dtype=float), \
        Rotation.from_quat([x, y, z, w]).as_matrix()


class LoopRig:
    """FakeCtrl 假臂 + TeleopLoop；先摆到中部姿态，target 用臂当前 TCP 合成
    （mapper 捕获参考的替身）。每次 step 后等一帧，模拟主循环 wait_frame 节拍
    （否则实测角不刷新，静止误判）。"""

    def __init__(self, **loop_kw):
        self.fc = tse.FakeCtrl()
        self.arm, self.em = tse.make_arm(self.fc)
        self.arm.move_joints(START_Q_URDF, vel_dps=100)
        self.arm.wait_idle(2)
        self.loop = TeleopLoop(self.arm, **loop_kw)
        self.pos0, self.rot0 = tcp_pose(self.arm)
        self.q0 = [servo_to_joint(i, self.fc.q[i]) for i in range(6)]

    def step(self, state="active", dpos=(0.0, 0.0, 0.0), trigger=0.0, rot=None):
        rot = self.rot0 if rot is None else rot
        target = PoseTarget(self.pos0 + np.array(dpos), rot)
        out = self.loop.step(make_status(state, target, trigger), self.arm.state())
        tse.em_wait_frames(self.arm, 1)
        return out

    def engage(self, trigger=0.0):
        return self.step(trigger=trigger)

    def close(self):
        self.arm.close()


class TestEngageDisengage(unittest.TestCase):
    def setUp(self):
        self.rig = LoopRig()

    def tearDown(self):
        self.rig.close()

    def test_engage_no_jump_and_first_cmd_equals_current_q(self):
        """接合帧：stream_start + 首条命令 J0–J5 = 当前角（Δ<0.05°）；J6 从实测起步
        按闭合限速走（扳机全压 → 实测 − 一帧步长）。接合前（idle）不发任何命令。"""
        n0 = len(self.rig.fc.cmds)                 # 装置 __init__ 里 move_joints 已发过命令
        out = self.rig.loop.step(make_status("idle"), self.rig.arm.state())   # 接合前：不发
        self.assertFalse(out["sent"])
        self.assertEqual(len(self.rig.fc.cmds), n0, "未接合不得发命令")
        out = self.rig.engage(trigger=1.0)
        self.assertTrue(out["sent"] and out["streaming"])
        np.testing.assert_allclose(out["pos"], self.rig.pos0, atol=1e-9)
        tse.em_wait_frames(self.rig.arm, 2)
        last = self.rig.fc.cmds[-1]
        self.assertEqual(last.mode, 0)                                          # STREAM
        for i in range(6):
            self.assertLess(abs(servo_to_joint(i, last.q[i]) - self.rig.q0[i]), 0.05,
                            f"J{i} 接合首帧跳变")
        self.assertAlmostEqual(last.q[6], tse.HOME_Q[6] - STEP, places=6)      # 从实测闭合一帧步长

    def test_disengage_holds_pose_j6_follows_trigger(self):
        """脱开：J0–J5 保持命令角；j6 继续随扳机按默认闭合限速闭合。"""
        self.rig.engage(trigger=0.0)                       # j6 张到 64.5
        tse.em_wait_frames(self.rig.arm, 2)
        q_hold = [self.rig.fc.cmds[-1].q[i] for i in range(6)]
        outs = [self.rig.step(state="idle", trigger=1.0) for _ in range(5)]
        self.assertTrue(all(o["sent"] for o in outs))
        last = [c for c in self.rig.fc.cmds if c.mode == 0][-1]
        for i in range(6):
            self.assertAlmostEqual(last.q[i], q_hold[i], places=4, msg=f"J{i} 脱开应保持")
        self.assertAlmostEqual(last.q[6], 64.5 - STEP * 5, places=6)          # 限速 × 5 帧

    def test_stale_stops_stream_then_reengage_no_jump(self):
        """STALE：stream_stop（控制器 HOLD）且不再发；恢复后 idle 仍不发；再接合不跳。"""
        self.rig.engage()
        out = self.rig.step(state="stale")
        self.assertFalse(out["sent"])
        self.assertFalse(out["streaming"])
        tse.em_wait_frames(self.rig.arm, 3)
        self.assertEqual(self.rig.fc.ctrl_state, 3)                            # HOLD
        n_cmd = len(self.rig.fc.cmds)
        for _ in range(3):
            self.rig.step(state="stale")
        self.assertEqual(len(self.rig.fc.cmds), n_cmd, "停流后不得再发命令")
        for _ in range(3):                                                     # 恢复但未松握把
            out = self.rig.step(state="idle", trigger=1.0)
            self.assertFalse(out["sent"])
        self.assertEqual(len(self.rig.fc.cmds), n_cmd)
        out = self.rig.engage()                                                # 再接合
        self.assertTrue(out["sent"] and out["streaming"])
        tse.em_wait_frames(self.rig.arm, 2)
        last = self.rig.fc.cmds[-1]
        for i in range(6):
            self.assertLess(abs(servo_to_joint(i, last.q[i]) - self.rig.q0[i]), 0.05,
                            f"J{i} 再接合跳变")


class TestLimiter(unittest.TestCase):
    def setUp(self):
        self.rig = LoopRig()

    def tearDown(self):
        self.rig.close()

    def test_step_limit_truncates(self):
        """目标一帧跳 0.25 m：单帧只走 12.5 mm；持续给目标则逐帧逼近。"""
        self.rig.engage()
        prev = np.array(self.rig.pos0)
        first = self.rig.step(dpos=(0.25, 0.0, 0.0))
        self.assertTrue(first["step_clamped"])
        self.assertEqual(self.rig.loop.ik_fail, 0)
        d = np.linalg.norm(np.array(first["pos"]) - prev)
        self.assertAlmostEqual(d, 0.0125, places=9)
        prev = np.array(first["pos"])
        clamped_any_untruncated = False
        for _ in range(25):
            out = self.rig.step(dpos=(0.25, 0.0, 0.0))
            self.assertLessEqual(np.linalg.norm(np.array(out["pos"]) - prev), 0.0125 + 1e-9)
            prev = np.array(out["pos"])
            clamped_any_untruncated |= not out["step_clamped"]
        self.assertTrue(clamped_any_untruncated, "够近之后应不再截断")
        self.assertEqual(self.rig.loop.ik_fail, 0)

    def test_box_clamp(self):
        """目标 y=0.6 m 出盒（本测试用 y∈[−0.35,0.10] 的盒）：y 被钳在 0.10。"""
        rig = LoopRig(ws_box=((0.05, 0.50), (-0.35, 0.10), (0.00, 0.55)))
        try:
            rig.engage()
            box_clamped_seen = False
            out = None
            for _ in range(45):
                out = rig.step(dpos=(0.0, 0.6, 0.0))
                box_clamped_seen |= out["box_clamped"]
            self.assertTrue(box_clamped_seen)
            self.assertAlmostEqual(out["pos"][1], 0.10, places=9)
            self.assertEqual(rig.loop.ik_fail, 0)
        finally:
            rig.close()

    def test_ik_fail_holds_previous_frame(self):
        """逆解失败：保持上一帧命令并计数，不抛出；目标恢复后继续发。
        （确定性注入：patch stream_pose 抛 IK 无解；折叠位朝限位方向的真实无解
        也实测过——HOME 姿态 +X 12.5 mm 即无解。）"""
        self.rig.engage()
        tse.em_wait_frames(self.rig.arm, 2)
        held_cmd = [c for c in self.rig.fc.cmds if c.mode == 0][-1]
        arm = self.rig.arm
        orig = arm.stream_pose

        def boom(*a, **k):
            raise D1ArmError("IK 无解（位置误差 500.0 mm，姿态误差 0.00°）")
        arm.stream_pose = boom                                  # 注入逆解失败（确定性）
        try:
            out = self.rig.step(dpos=(0.05, 0.0, 0.0))
            self.assertFalse(out["sent"])
            self.assertEqual(out["ik_fail"], 1)
            out = self.rig.step(dpos=(0.05, 0.0, 0.0))
            self.assertEqual(out["ik_fail"], 2)
        finally:
            arm.stream_pose = orig
        tse.em_wait_frames(self.rig.arm, 2)
        still = [c for c in self.rig.fc.cmds if c.mode == 0][-1]
        self.assertEqual([float(q) for q in still.q], [float(q) for q in held_cmd.q],
                         "逆解失败期间必须保持上一帧命令")
        out = self.rig.step(dpos=(0.01, 0.0, 0.0))
        self.assertTrue(out["sent"])
        self.assertEqual(out["ik_fail"], 2)


class TestGraspAndHeading(unittest.TestCase):
    def setUp(self):
        self.rig = LoopRig(fake_grasp_at=20.0)

    def tearDown(self):
        self.rig.close()

    def test_fake_grasp_locks_and_releases(self):
        """--fake-grasp-at 20：实测钳在 20° → 锁定在 20°（grasp）；松扳机解锁（release）。"""
        self.rig.engage(trigger=1.0)                 # j6 从实测起持续闭合
        raws, events, outs = [], [], []
        for _ in range(35):
            out = self.rig.step(trigger=1.0)
            raws.append(out["j6_meas_raw"])
            events.append(out["event"])
            outs.append(out)
        self.assertEqual(events.count("grasp"), 1)
        self.assertTrue(self.rig.loop.griplock.locked)
        self.assertAlmostEqual(self.rig.loop.griplock.lock_deg, 20.0, places=6)
        self.assertLess(min(raws), 20.0)             # 原始实测确实越过 20°（闭合过冲）
        self.assertAlmostEqual(outs[-1]["j6_meas"], 20.0, places=6)   # 被钳住
        self.assertAlmostEqual(outs[-1]["j6_out"], 20.0, places=6)    # 继续压扳机：不再变
        out = self.rig.step(trigger=0.0)             # 松扳机 → 解锁张开
        self.assertEqual(out["event"], "release")
        self.assertAlmostEqual(out["j6_out"], 64.5)  # 开方向不限速

    def test_yaw_rotates_horizontal_delta(self):
        """yaw 90°：目标水平位移 +X 10 mm → 下发 +Y 10 mm；手不转时姿态保持接合时不变。"""
        rig = LoopRig(yaw_deg=90.0)
        try:
            rig.engage()
            out = rig.step(dpos=(0.01, 0.0, 0.0))
            np.testing.assert_allclose(out["pos"], rig.pos0 + [0.0, 0.01, 0.0], atol=1e-9)
            want = Rotation.from_matrix(rig.rot0).as_quat()                  # xyzw
            got = out["quat"]                                                # wxyz
            np.testing.assert_allclose([got[1], got[2], got[3], got[0]], want, atol=1e-9)
        finally:
            rig.close()

    def test_yaw_180_engage_no_orientation_jump(self):
        """面朝臂标定（yaw 180°）：接合首帧姿态 = 当前 TCP（旧实现转 180° → 逆解失败）。"""
        rig = LoopRig(yaw_deg=180.0)
        try:
            out = rig.engage()
            self.assertTrue(out["sent"])
            got = out["quat"]
            want = Rotation.from_matrix(rig.rot0).as_quat()
            np.testing.assert_allclose(np.abs(np.dot([got[1], got[2], got[3], got[0]], want)), 1.0, atol=1e-9)
            self.assertEqual(rig.loop.ik_fail, 0)
        finally:
            rig.close()

    def test_yaw_rotation_delta_changes_frame(self):
        """yaw 90°：手绕 XR 世界 X 轴转 10° → 下发姿态 = 绕基座 Y 轴转 10° 后的 R_ref。"""
        rig = LoopRig(yaw_deg=90.0, max_rot_vel_dps=1000.0)   # 不让限速介入，只测换系
        try:
            rig.engage()
            d = Rotation.from_euler("x", 10, degrees=True).as_matrix()
            out = rig.step(rot=d @ rig.rot0)
            want = Rotation.from_matrix(
                Rotation.from_euler("y", 10, degrees=True).as_matrix() @ rig.rot0).as_quat()
            got = out["quat"]
            np.testing.assert_allclose(np.abs(np.dot([got[1], got[2], got[3], got[0]], want)), 1.0, atol=1e-9)
        finally:
            rig.close()

    def test_rot_limit_truncates_and_converges(self):
        """姿态限速 30°/s（显式给）：手一帧转 30° → 单帧只转 1.5°、rot_clamped；持续给目标逐帧逼近；1° 不截。"""
        rig = LoopRig(max_rot_vel_dps=30.0)
        try:
            rig.engage()
            d = Rotation.from_euler("z", 30, degrees=True).as_matrix()
            tgt = Rotation.from_matrix(d @ rig.rot0)

            def ang_to(out, r):
                w, x, y, z = out["quat"]
                return np.degrees((Rotation.from_quat([x, y, z, w]) * r.inv()).magnitude())

            out = rig.step(rot=d @ rig.rot0)
            self.assertTrue(out["rot_clamped"])
            self.assertAlmostEqual(ang_to(out, Rotation.from_matrix(rig.rot0)), 1.5, places=6)
            for _ in range(25):
                out = rig.step(rot=d @ rig.rot0)
            self.assertLess(ang_to(out, tgt), 1e-6, "约 20 帧后到达目标姿态")
            self.assertFalse(out["rot_clamped"])
            small = Rotation.from_euler("z", 31, degrees=True).as_matrix()
            out = rig.step(rot=small @ rig.rot0)
            self.assertFalse(out["rot_clamped"], "1° 小于单帧上限不截")
        finally:
            rig.close()

    def test_yaw_from_push(self):
        """标定计算：+X 推程 → 0°；+Y → −90°；对角 → −45°；推程不足报错。"""
        p = (0.30, 0.20, 0.17)
        self.assertAlmostEqual(yaw_from_push(p, (0.50, 0.20, 0.17)), 0.0)
        self.assertAlmostEqual(yaw_from_push(p, (0.30, 0.40, 0.17)), -90.0)
        self.assertAlmostEqual(yaw_from_push(p, (0.45, 0.35, 0.17)), -45.0)
        with self.assertRaises(ValueError):
            yaw_from_push(p, (0.35, 0.20, 0.17))


class TestControllerFault(unittest.TestCase):
    def test_estop_raises(self):
        """控制器 ESTOP：step 抛 D1ArmError，主循环停机。"""
        rig = LoopRig()
        try:
            rig.engage()
            tse.sync_ctrl_state(rig.fc, rig.arm, rig.em, 5)
            with self.assertRaises(D1ArmError) as ctx:
                rig.loop.step(make_status("idle"), rig.arm.state())
            self.assertIn("ESTOP", str(ctx.exception))
        finally:
            rig.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
