# -*- coding: utf-8 -*-
"""park（回折叠位准备下电）离线测试：目标换算、夹爪检查（未张开拒绝且零运动）、
两段顺序与目标（不动轴=命令角）、已在折叠位只检查夹爪+失力、0.5 s 后 free_all、
到位超差抛错。复用 test_stream_executor 的 FakeCtrl（假控制器，无 DDS）。"""
import math
import sys
import time
import unittest
from unittest import mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sdk/
sys.path.insert(0, str(Path(__file__).resolve().parent))      # tests/（复用 FakeCtrl）

from d1t_sdk import D1Arm, D1ArmError
from d1t_sdk import d1arm as d1arm_mod
from d1t_sdk.d1arm import PARK_SERVO_DEG, PARK_SETTLE_S, PARK_TOLERANCE_DEG
from d1t_sdk._joint_model import (gripper_mm_to_deg, joint_to_servo, servo_to_joint,
                                  urdf_limits)
from test_stream_executor import Emitter, FakeCtrl, make_arm

# 起点用的舵机角（URDF 角经 sign 换算；J0/J3 sign=-1）
OPEN_J6 = gripper_mm_to_deg(70.0)                          # 夹爪全开 J6 角（park 前须张开）
EXTENDED_Q_SERVO = [0.0] * 6 + [OPEN_J6]                   # 前伸（URDF 全零）+ 夹爪全开
POSED_Q_SERVO = [0.0, -40.0, 60.0, 0.0, 30.0, 0.0, OPEN_J6]  # 常用示例姿态（舵机角）

MODE_STREAM, MODE_HOLD, MODE_RELEASE = 0, 1, 2


class TestParkTargetConversion(unittest.TestCase):
    def test_target_conversion_and_limits(self):
        """park 目标 = 原厂 HOME 留余量版；舵机↔URDF 往返精确；在软限位内。"""
        target_urdf = [servo_to_joint(i, PARK_SERVO_DEG[i]) for i in range(6)]
        # 往返：URDF 目标换回舵机角应与常量逐位一致（offset=0、sign=±1）
        for i in range(6):
            self.assertAlmostEqual(joint_to_servo(i, target_urdf[i]),
                                   PARK_SERVO_DEG[i], places=9)
        # J1/J2 留限位余量：J1=-90 ≥ -90.3（0.3°）、J2=88.5 ≤ 89（0.5°）
        self.assertGreaterEqual(PARK_SERVO_DEG[1] + 90.3, 0.29)
        self.assertLessEqual(PARK_SERVO_DEG[2], 89.0 - 0.49)
        # 各轴目标都在 URDF 限位内（move_joints 预检会再查一遍）
        lims = urdf_limits()
        for i in range(6):
            lo, hi = lims[i]
            self.assertTrue(lo <= target_urdf[i] <= hi,
                            f"J{i} 目标 {target_urdf[i]:.1f} 超出 [{lo}, {hi}]")
        # 与 URDF 全零 home 不是同一姿势（J1/J2 不同）
        self.assertNotEqual(target_urdf[1], 0.0)
        self.assertNotEqual(target_urdf[2], 0.0)


class ParkCtrl(FakeCtrl):
    """FakeCtrl + 上报 q_target（最近一次 STREAM/HOLD 目标，失力后 NaN）与 J6 电流。"""

    def __init__(self, q, q_target=None, j6_current_ma=14):
        super().__init__()
        self.q = [float(x) for x in q]
        self.q_target = [float(x) for x in (q_target if q_target is not None else q)]
        self.j6_current_ma = j6_current_ma

    def send_cmd(self, cmd):
        super().send_cmd(cmd)
        with self._lk:
            if cmd.mode in (MODE_STREAM, MODE_HOLD):
                self.q_target = [float(x) for x in cmd.q]
            elif cmd.mode == MODE_RELEASE:
                self.q_target = [math.nan] * 7

    def poll_frames(self):
        frames = super().poll_frames()
        with self._lk:
            qt = list(self.q_target)
            cur = [10] * 6 + [int(self.j6_current_ma)]
        for f in frames:
            f.q_target = qt
            f.current_ma = cur
        return frames


class TestPark(unittest.TestCase):
    def _arm(self, fc):
        arm, emitter = make_arm(fc)
        self.addCleanup(emitter.stop)
        self.addCleanup(arm.close)
        return arm

    @staticmethod
    def _motion_groups(fc):
        """把 STREAM 帧按 HOLD 边界分组 → [组内全部样本(舵机角 7)]，每组 = 一次轨迹。"""
        groups, current = [], []
        for c in fc.cmds:
            if c.mode == MODE_STREAM:
                current.append(list(c.q))
            elif c.mode == MODE_HOLD and current:
                groups.append(current)
                current = []
        return groups

    def _run_park_spy_sleep(self, arm, fc):
        """跑 park，记录每次 sleep(PARK_SETTLE_S) 时已发命令条数。"""
        real_sleep = time.sleep
        marks = []

        def spy(sec):
            if sec == PARK_SETTLE_S:
                with fc._lk:
                    marks.append(len(fc.cmds))
            real_sleep(sec)

        with mock.patch.object(d1arm_mod.time, "sleep", side_effect=spy):
            result = arm.park()
        return result, marks

    # ---------- 夹爪检查 ----------

    def test_gripper_closed_raises_no_motion(self):
        """夹爪未张开（J6 29° ≈ 45 mm）→ 抛错，零命令。"""
        q = list(POSED_Q_SERVO)
        q[6] = 29.0
        fc = ParkCtrl(q, j6_current_ma=14)
        arm = self._arm(fc)
        with self.assertRaisesRegex(D1ArmError, "未张开"):
            arm.park()
        self.assertEqual(fc.cmds, [])

    def test_holding_with_target_at_contact_raises(self):
        """实机复现：命令 47.2 mm 夹瓶，电流低、命令≈反馈 → 仍须拒绝（旧判定在此张开掉瓶）。"""
        q = list(POSED_Q_SERVO)
        q[6] = gripper_mm_to_deg(47.5)
        qt = list(POSED_Q_SERVO)
        qt[6] = gripper_mm_to_deg(47.2)
        fc = ParkCtrl(q, q_target=qt, j6_current_ma=16)
        arm = self._arm(fc)
        with self.assertRaisesRegex(D1ArmError, "未张开"):
            arm.park()
        self.assertEqual(fc.cmds, [])

    def test_gripper_open_no_gripper_motion(self):
        """夹爪已张开（70 mm）→ 不发夹爪轨迹，J6 全程保持张开。"""
        fc = ParkCtrl(EXTENDED_Q_SERVO, j6_current_ma=14)
        arm = self._arm(fc)
        result, _ = self._run_park_spy_sleep(arm, fc)
        self.assertTrue(result["freed"])
        for g in self._motion_groups(fc):
            for sample in g:
                self.assertAlmostEqual(sample[6], OPEN_J6, places=4)

    # ---------- 顺序与目标 ----------

    def test_two_segments_order_targets_held_axes_use_q_target(self):
        """夹爪已开：段1 腕部回零（J0–J2=命令角，非实测）→ 段2 J0–J2 同收 →
        0.5 s 后 free_all（最后一条命令）。"""
        qt = list(POSED_Q_SERVO)
        q = list(POSED_Q_SERVO)
        q[1] -= 1.5   # J1 实测比命令角下沉 1.5°（棘轮）
        q[2] += 1.0
        fc = ParkCtrl(q, q_target=qt, j6_current_ma=14)
        arm = self._arm(fc)
        result, marks = self._run_park_spy_sleep(arm, fc)

        self.assertFalse(result["already_parked"])
        self.assertEqual([s[0] for s in result["segments"]], ["1-腕部回零", "2-大小臂收回"])
        groups = self._motion_groups(fc)
        self.assertEqual(len(groups), 2)   # 两段
        seg1, seg2 = (g[-1] for g in groups)
        # 段1：J3/J4/J5 → 0；J0/J1/J2 = 命令角 q_target（不是下沉后的实测角）
        for i in (0, 1, 2):
            self.assertAlmostEqual(seg1[i], qt[i], places=4)
        self.assertNotAlmostEqual(seg1[1], q[1], places=2)
        for i in (3, 4, 5):
            self.assertAlmostEqual(seg1[i], PARK_SERVO_DEG[i], places=4)
        # 段2：J0/J1/J2 同时到折叠位，腕部保持 0
        for i in range(6):
            self.assertAlmostEqual(seg2[i], PARK_SERVO_DEG[i], places=4)
        # 段2 是同步插值：J1、J2 在同一条轨迹里一起动（中点处都已离开起点）
        mid = groups[1][len(groups[1]) // 2]
        self.assertGreater(abs(mid[1] - seg1[1]), 1.0)
        self.assertGreater(abs(mid[2] - seg1[2]), 1.0)
        # 0.5 s 停顿在最后一帧之后、free_all（RELEASE，最后一条命令）之前
        self.assertEqual(fc.cmds[-1].mode, MODE_RELEASE)
        self.assertEqual(fc.cmds[-1].axis_mask, 0x7F)
        self.assertEqual(len(marks), 1)
        self.assertTrue(all(c.mode != MODE_RELEASE for c in fc.cmds[:marks[0]]))
        self.assertTrue(all(c.mode == MODE_RELEASE for c in fc.cmds[marks[0]:]))
        self.assertEqual(fc.ctrl_state, 4)  # RELEASED

    def test_wrist_already_zero_skips_segment1(self):
        """腕部已在 0 的 3° 内：只走段2。"""
        fc = ParkCtrl([0.0, -40.0, 60.0, 1.0, -2.0, 0.5, OPEN_J6], j6_current_ma=14)
        arm = self._arm(fc)
        result, _ = self._run_park_spy_sleep(arm, fc)
        self.assertEqual([s[0] for s in result["segments"]], ["2-大小臂收回"])
        self.assertEqual(len(self._motion_groups(fc)), 1)  # 只有段2
        self.assertEqual(fc.cmds[-1].mode, MODE_RELEASE)

    def test_already_parked_only_check_and_free(self):
        """已在折叠位且夹爪已开：不发任何运动，0.5 s + free_all。"""
        fc = ParkCtrl(list(PARK_SERVO_DEG) + [OPEN_J6], j6_current_ma=14)
        arm = self._arm(fc)
        result, marks = self._run_park_spy_sleep(arm, fc)
        self.assertTrue(result["already_parked"])
        self.assertEqual(result["segments"], [])
        self.assertEqual(self._motion_groups(fc), [])
        self.assertEqual(len(marks), 1)
        self.assertEqual(fc.cmds[-1].mode, MODE_RELEASE)

    def test_already_parked_but_gripper_closed_raises(self):
        """已在折叠位也要过夹爪检查：未张开 → 抛错、不失力。"""
        fc = ParkCtrl(list(PARK_SERVO_DEG) + [29.0], j6_current_ma=14)
        arm = self._arm(fc)
        with self.assertRaisesRegex(D1ArmError, "未张开"):
            arm.park()
        self.assertEqual(fc.cmds, [])

    def test_over_tolerance_stops_no_free(self):
        """段1 到位超差 → 抛 D1ArmError，不发段2、不 free_all。"""

        class StuckCtrl(ParkCtrl):
            """J0–J5 不跟随命令（模拟堵转）。"""

            def send_cmd(self, cmd):
                with self._lk:
                    keep = list(self.q[:6])
                super().send_cmd(cmd)
                with self._lk:
                    self.q[:6] = keep

        fc = StuckCtrl(POSED_Q_SERVO, j6_current_ma=14)
        arm = self._arm(fc)
        with self.assertRaisesRegex(D1ArmError, r"超差"):
            arm.park()
        self.assertEqual(len(self._motion_groups(fc)), 1)  # 只有段1
        self.assertFalse(any(c.mode == MODE_RELEASE for c in fc.cmds))

    def test_async_rejected(self):
        fc = ParkCtrl(EXTENDED_Q_SERVO)
        arm = self._arm(fc)
        with self.assertRaisesRegex(D1ArmError, "async_"):
            arm.park(async_=True)
        self.assertEqual(fc.cmds, [])


if __name__ == "__main__":
    unittest.main()
