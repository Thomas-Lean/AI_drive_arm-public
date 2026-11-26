#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A/B 一键回位离线测试（FakeCtrl 假臂，复用 sdk/tests 装置；无 DDS 网络、无 TeleRobot）。

覆盖：按住 0.5 s 才触发且一次按住只触发一次；回准备位走到 READY 且夹爪命令角不变；
回折叠位两段（腕部先回）到 FOLD；中途停流即停在中间；TeleopLoop.detach 交出流式。

运行：python3 tools/pico_teleop/test_goto.py -v
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
for _p in (str(HERE), str(HERE.parents[1] / "sdk"), str(HERE.parents[1] / "sdk" / "tests")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import test_stream_executor as tse               # noqa: E402
from d1t_sdk._joint_model import servo_to_joint  # noqa: E402

from goto import FOLD_Q_DEG, READY_Q_DEG, ButtonHold, GotoMotion, near_fold  # noqa: E402
from teleop_loop import TeleopLoop               # noqa: E402

START_Q = [20.0, -30.0, 50.0, 10.0, 20.0, 5.0]   # 偏离准备位的任意中部姿态


def q_urdf(fc):
    return [servo_to_joint(i, fc.q[i]) for i in range(6)]


class TestButtonHold(unittest.TestCase):
    def test_hold_threshold_and_single_fire(self):
        bh = ButtonHold(hold_s=0.5)
        self.assertIsNone(bh.update(0.0, True, False))
        self.assertIsNone(bh.update(0.4, True, False))
        self.assertEqual(bh.update(0.5, True, False), "ready")
        self.assertIsNone(bh.update(2.0, True, False), "一次按住只触发一次")
        self.assertIsNone(bh.update(2.1, False, False))
        self.assertIsNone(bh.update(3.0, False, True))
        self.assertEqual(bh.update(3.6, False, True), "fold")

    def test_short_tap_ignored(self):
        bh = ButtonHold(hold_s=0.5)
        bh.update(0.0, True, False)
        bh.update(0.3, False, False)
        self.assertIsNone(bh.update(0.6, True, False), "松开后计时重新开始")


class TestNearFold(unittest.TestCase):
    def test_near_fold(self):
        self.assertTrue(near_fold(FOLD_Q_DEG))
        self.assertTrue(near_fold([v + 9.0 for v in FOLD_Q_DEG]))
        self.assertFalse(near_fold(READY_Q_DEG))


class GotoRig:
    def __init__(self, start=START_Q, grip_deg=None):
        self.fc = tse.FakeCtrl()
        self.arm, self.em = tse.make_arm(self.fc)
        self.arm.move_joints(start, vel_dps=100)
        self.arm.wait_idle(3)
        if grip_deg is not None:
            self.arm.stream_start()
            self.arm.stream_joints(start, j6_deg=grip_deg)
            tse.em_wait_frames(self.arm, 30)
            self.arm.stream_stop()

    def run(self, kind, max_frames=None):
        self.arm.stream_start()
        g = GotoMotion(self.arm, kind, vel_dps=60)
        n = 0
        while not g.step():
            tse.em_wait_frames(self.arm, 1)
            n += 1
            if max_frames is not None and n >= max_frames:
                break
        tse.em_wait_frames(self.arm, 3)
        self.arm.stream_stop()
        return g

    def close(self):
        self.arm.close()


class TestGoto(unittest.TestCase):
    def test_ready_reaches_pose_keeps_gripper(self):
        rig = GotoRig(grip_deg=20.0)
        try:
            j6_before = rig.fc.q[6]
            rig.run("ready")
            for got, want in zip(q_urdf(rig.fc), READY_Q_DEG):
                self.assertAlmostEqual(got, want, delta=0.2)
            self.assertAlmostEqual(rig.fc.q[6], j6_before, delta=0.2, msg="回准备位不动夹爪")
        finally:
            rig.close()

    def test_fold_wrist_first_then_arm(self):
        rig = GotoRig()
        try:
            rig.arm.stream_start()
            g = GotoMotion(rig.arm, "fold", vel_dps=60)
            # 第一段中途：J0–J2 不动，腕部在动
            for _ in range(5):
                g.step()
                tse.em_wait_frames(rig.arm, 1)
            q = q_urdf(rig.fc)
            for i in range(3):
                self.assertAlmostEqual(q[i], START_Q[i], delta=0.2, msg=f"腕部段 J{i} 不该动")
            while not g.step():
                tse.em_wait_frames(rig.arm, 1)
            tse.em_wait_frames(rig.arm, 3)
            rig.arm.stream_stop()
            for got, want in zip(q_urdf(rig.fc), FOLD_Q_DEG):
                self.assertAlmostEqual(got, want, delta=0.2)
        finally:
            rig.close()

    def test_abort_midway_stops_between(self):
        rig = GotoRig()
        try:
            g = rig.run("ready", max_frames=10)
            self.assertFalse(g.done)
            q = q_urdf(rig.fc)
            self.assertGreater(abs(q[0] - READY_Q_DEG[0]), 1.0, "中止后不该到终点")
            self.assertLess(abs(q[0]), abs(START_Q[0]), "中止前已朝终点走了一段")
            q_after = q_urdf(rig.fc)
            tse.em_wait_frames(rig.arm, 10)
            for a, b in zip(q_after, q_urdf(rig.fc)):
                self.assertAlmostEqual(a, b, delta=0.05, msg="停流后保持不动")
        finally:
            rig.close()


class TestDetach(unittest.TestCase):
    def test_detach_releases_stream(self):
        rig = GotoRig()
        try:
            loop = TeleopLoop(rig.arm)
            rig.arm.stream_start()
            loop.streaming, loop.held, loop.ref = True, ("p", "q"), ("r", "R")
            loop.detach()
            self.assertFalse(loop.streaming)
            self.assertIsNone(loop.held)
            self.assertIsNone(loop.ref)
            rig.arm.stream_start()          # 交出后可重新进流式（不报 AlreadyMoving）
            rig.arm.stream_stop()
        finally:
            rig.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
