# -*- coding: utf-8 -*-
"""关节模型测试：URDF↔舵机角换算往返、限位换算、夹爪标定表（表点精确、
0–50° 段与旧公式差 ≤1 mm、非法输入截断）。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d1t_sdk._common import GRIPPER_TABLE, SERVO_LIMITS, JOINT_CALIB
from d1t_sdk._joint_model import (check_q_urdf, gripper_deg_to_mm, gripper_formula_mm,
                                  gripper_mm_to_deg, joint_to_servo, servo_to_joint,
                                  urdf_limits)
from d1t_sdk._common import D1ArmError


class TestAngleConversion(unittest.TestCase):
    def test_calib_matches_colleague(self):
        """标定与同事 calib.json 一致：下标 0/3 sign=-1，offset 全 0。"""
        signs = [s for s, _ in JOINT_CALIB]
        offsets = [o for _, o in JOINT_CALIB]
        self.assertEqual(signs, [-1, 1, 1, -1, 1, 1])
        self.assertEqual(offsets, [0.0] * 6)

    def test_roundtrip(self):
        """servo→URDF→servo 往返恒等（含负角、极限附近）。"""
        cases = [0.0, 1.25, -90.25, 36.25, 162.7, -162.7, 89.9, 1e-3, -64.5]
        for i in range(6):
            for v in cases:
                self.assertAlmostEqual(joint_to_servo(i, servo_to_joint(i, v)), v, places=9)

    def test_sign_flip(self):
        """J0/J3 反号，其余直传。"""
        self.assertEqual(servo_to_joint(0, 10.0), -10.0)
        self.assertEqual(joint_to_servo(0, -10.0), 10.0)
        self.assertEqual(servo_to_joint(3, 10.0), -10.0)
        self.assertEqual(servo_to_joint(1, 10.0), 10.0)

    def test_urdf_limits_conversion(self):
        """F7 舵机限位经 sign 换算（区间翻转）：J3 [-166,163] → [-163,166]。"""
        lims = urdf_limits()
        expect = [(-163.0, 163.0), (-90.3, 97.0), (-97.0, 89.0),
                  (-163.0, 166.0), (-96.0, 113.0), (-166.0, 162.0)]
        for got, want in zip(lims, expect):
            self.assertEqual(got, want)

    def test_check_q_urdf_message(self):
        """越界报错带关节号（学同事格式），正文含数值与限位。"""
        q = [0.0] * 6
        q[3] = 170.0  # URDF J3 上限 166
        with self.assertRaises(D1ArmError) as ctx:
            check_q_urdf(q)
        msg = str(ctx.exception)
        self.assertIn("joint 3 (J3)", msg)
        self.assertIn("170.0", msg)
        self.assertIn("166.0", msg)

    def test_check_q_urdf_ok(self):
        check_q_urdf([-163.0, -90.3, -97.0, -163.0, -96.0, -166.0])
        check_q_urdf([163.0, 97.0, 89.0, 166.0, 113.0, 162.0])


class TestGripperTable(unittest.TestCase):
    def test_table_points_exact(self):
        """标定表 8 点：mm→deg→mm 往返精确回表点值（≤0.05 mm）。"""
        for deg, mm in GRIPPER_TABLE:
            got = gripper_deg_to_mm(gripper_mm_to_deg(mm))
            self.assertAlmostEqual(got, mm, delta=0.05, msg=f"表点 {mm} mm")
        for deg, mm in GRIPPER_TABLE:
            self.assertAlmostEqual(gripper_mm_to_deg(gripper_deg_to_mm(deg)), deg,
                                   delta=0.05, msg=f"表点 {deg}°")

    def test_matches_formula_0_50(self):
        """0–50° 段与旧公式 间距=0.85×角度+19.8 偏差 ≤1 mm。"""
        degs = [0.3, 5, 10.2, 20, 20.2, 30, 40, 40.3, 49, 50.0]
        for d in degs:
            mm = gripper_deg_to_mm(d)
            self.assertLess(abs(mm - gripper_formula_mm(d)), 1.0, msg=f"{d}°")

    def test_out_of_domain_clamps(self):
        """mm 出表域截断：>70 → 64.2°，<4 → -29.6°。"""
        self.assertAlmostEqual(gripper_mm_to_deg(80.0), 64.2, places=6)
        self.assertAlmostEqual(gripper_mm_to_deg(0.0), -29.6, places=6)
        self.assertAlmostEqual(gripper_deg_to_mm(70.0), 70.0, places=6)
        self.assertAlmostEqual(gripper_deg_to_mm(-40.0), 4.0, places=6)

    def test_monotonic(self):
        """角度越大开度越大（单调递增）。"""
        degs = [-29.6, 0.3, 10.2, 20.2, 29.2, 40.3, 50.3, 64.2]
        mms = [gripper_deg_to_mm(d) for d in degs]
        self.assertEqual(mms, sorted(mms))


if __name__ == "__main__":
    unittest.main()
