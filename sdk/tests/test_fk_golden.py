# -*- coding: utf-8 -*-
"""FK 金标测试：数值逐项对照同事 d1arm_sdk tests/test_main.cpp::test_fk_golden
（numpy 独立从 URDF 计算）。附带雅可比 vs 数值微分、Pose 往返。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from d1t_sdk import Kinematics
from d1t_sdk._kinematics import rot_to_axis_angle, to_pose_dict, to_rpy, from_pose_dict

D = math.pi / 180.0


class TestFkGolden(unittest.TestCase):
    def setUp(self):
        self.k = Kinematics()

    def test_fk_zero_chain(self):
        """q=0 时各 frame 平移（同事金标，1e-9）。"""
        t = self.k.fk_chain([0.0] * 6)
        checks = [
            (t[1], 0.0, None, 0.0533),
            (t[2], None, -0.028, 0.1096),
            (t[3], None, None, 0.3789),
            (t[4], 0.0577, -0.0014, 0.4209),
            (t[5], 0.19788, None, None),
            (t[6], 0.28038, None, None),
        ]
        for T, x, y, z in checks:
            if x is not None:
                self.assertAlmostEqual(T[0, 3], x, delta=1e-9)
            if y is not None:
                self.assertAlmostEqual(T[1, 3], y, delta=1e-9)
            if z is not None:
                self.assertAlmostEqual(T[2, 3], z, delta=1e-9)

    def test_fk_zero_tcp(self):
        tcp = self.k.fk([0.0] * 6)
        self.assertAlmostEqual(tcp[0, 3], 0.35098, delta=1e-9)
        self.assertAlmostEqual(tcp[1, 3], -0.001278, delta=1e-6)
        self.assertAlmostEqual(tcp[2, 3], 0.4255418, delta=1e-9)

    def test_fk_golden_q1(self):
        """q=10,20,-30,40,-50,60 度（同事金标，1e-6 / rpy 1e-5）。"""
        t = self.k.fk([10 * D, 20 * D, -30 * D, 40 * D, -50 * D, 60 * D])
        self.assertAlmostEqual(t[0, 3], 0.366329, delta=1e-6)
        self.assertAlmostEqual(t[1, 3], -0.010289, delta=1e-6)
        self.assertAlmostEqual(t[2, 3], 0.547792, delta=1e-6)
        r = to_rpy(t)
        self.assertAlmostEqual(r[0], 0.016232, delta=1e-5)
        self.assertAlmostEqual(r[1], -0.760837, delta=1e-5)
        self.assertAlmostEqual(r[2], -0.573056, delta=1e-5)

    def test_fk_golden_q2(self):
        """q=90,45,90,-60,-30,30 度（同事金标，1e-6）。"""
        t = self.k.fk([90 * D, 45 * D, 90 * D, -60 * D, -30 * D, 30 * D])
        self.assertAlmostEqual(t[0, 3], -0.069288, delta=1e-6)
        self.assertAlmostEqual(t[1, 3], 0.014250, delta=1e-6)
        self.assertAlmostEqual(t[2, 3], 0.011524, delta=1e-6)

    def test_pose_roundtrip(self):
        """to_pose_dict ↔ from_pose_dict 往返（同事金标 1e-9）。"""
        t = self.k.fk([10 * D, 20 * D, -30 * D, 40 * D, -50 * D, 60 * D])
        p = to_pose_dict(t)
        back = from_pose_dict(p["pos"], quat=p["quat"])
        self.assertAlmostEqual(float(np.linalg.norm(back[:3, 3] - t[:3, 3])), 0.0, delta=1e-9)
        self.assertAlmostEqual(float(np.linalg.norm(back[:3, :3] - t[:3, :3])), 0.0, delta=1e-9)
        # quat 顺序 [w,x,y,z]（同事 setPose 口径）
        self.assertAlmostEqual(p["quat"][0] ** 2 + sum(v * v for v in p["quat"][1:]), 1.0, places=9)

    def test_jacobian_vs_numeric(self):
        """几何雅可比 vs 中心差分（同事 test_jacobian，1e-5）。"""
        q = np.array([0.3, -0.5, 0.8, -0.2, 0.6, 1.1])
        jac = self.k.jacobian(q)
        h = 1e-6
        for j in range(6):
            qp, qm = q.copy(), q.copy()
            qp[j] += h
            qm[j] -= h
            tp, tm = self.k.fk(qp), self.k.fk(qm)
            dp = (tp[:3, 3] - tm[:3, 3]) / (2 * h)
            dw = rot_to_axis_angle(tp[:3, :3] @ tm[:3, :3].T) / (2 * h)
            for r in range(3):
                self.assertAlmostEqual(jac[r, j], dp[r], delta=1e-5)
                self.assertAlmostEqual(jac[r + 3, j], dw[r], delta=1e-5)


if __name__ == "__main__":
    unittest.main()
