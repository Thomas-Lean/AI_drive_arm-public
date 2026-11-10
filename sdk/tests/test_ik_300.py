# -*- coding: utf-8 -*-
"""IK 测试：300 随机位姿收敛（口径同同事 test_main.cpp::test_ik：q∈[-0.6,0.6] rad、
种子=q*0.5、要求 ≥97% 收敛、误差 ≤2 mm/0.02 rad、解在限位内）。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from d1t_sdk import IkParams, Kinematics
from d1t_sdk._joint_model import rad2deg, urdf_limits


class TestIk300(unittest.TestCase):
    def test_ik_300_random_poses(self):
        k = Kinematics()
        lims = urdf_limits()
        qmin = np.array([math.radians(lo) for lo, _ in lims])
        qmax = np.array([math.radians(hi) for _, hi in lims])
        rng = np.random.default_rng(7)
        solved = 0
        total = 300
        for _ in range(total):
            q = rng.uniform(-0.6, 0.6, 6)
            target = k.fk(q)
            res = k.ik(target, q * 0.5, qmin, qmax, IkParams())
            if res.success:
                solved += 1
                t2 = k.fk(res.q_rad)
                self.assertLess(float(np.linalg.norm(t2[:3, 3] - target[:3, 3])), 2e-3)
                from d1t_sdk._kinematics import rot_to_axis_angle
                self.assertLess(float(np.linalg.norm(
                    rot_to_axis_angle(t2[:3, :3] @ target[:3, :3].T))), 2e-2)
                for i in range(6):
                    self.assertGreaterEqual(res.q_rad[i], qmin[i] - 1e-6)
                    self.assertLessEqual(res.q_rad[i], qmax[i] + 1e-6)
        print(f"  IK: {solved}/{total} solved（97%+ 达标线 {total * 97 // 100}）")
        self.assertGreaterEqual(solved, total * 97 // 100)

    def test_ik_seed_from_current(self):
        """ik() 从当前角种子解（SDK ik 接口路径）；限位内随机 20 点。"""
        from d1t_sdk._joint_model import deg2rad
        k = Kinematics()
        lims = urdf_limits()
        qmin = np.array([deg2rad(lo) for lo, _ in lims])
        qmax = np.array([deg2rad(hi) for _, hi in lims])
        rng = np.random.default_rng(11)
        n_ok = 0
        for _ in range(20):
            q = rng.uniform(-0.5, 0.5, 6)
            res = k.ik(k.fk(q), q * 0.7, qmin, qmax, IkParams())
            if res.success:
                n_ok += 1
                degs = [rad2deg(x) for x in res.q_rad]
                for i, d in enumerate(degs):
                    lo, hi = lims[i]
                    self.assertGreaterEqual(d, lo - 1e-6)
                    self.assertLessEqual(d, hi + 1e-6)
        self.assertGreaterEqual(n_ok, 19)


if __name__ == "__main__":
    unittest.main()
