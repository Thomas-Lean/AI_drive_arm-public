# -*- coding: utf-8 -*-
"""轨迹测试：关节五次多项式（端点/单调时刻/峰值速度≤钳制后限速）、直线偏差、
圆弧偏差（口径：FK 位置对直线段/圆的最大偏差 ≤2 mm，端点 ≤IK 容差 0.5 mm）。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from d1t_sdk import Kinematics
from d1t_sdk._joint_model import deg2rad, urdf_limits
from d1t_sdk._trajectory import plan_arc, plan_joint_move, plan_line

LIMS = urdf_limits()
LIMS_RAD = ([deg2rad(lo) for lo, _ in LIMS], [deg2rad(hi) for _, hi in LIMS])
VMAX = (60.2, 60.2, 60.2, 99.1, 99.1, 99.1)  # 纯函数测试用的固定限速输入（验钳制逻辑；与控制器现值无关）


class TestJointMove(unittest.TestCase):
    def test_endpoints_and_shape(self):
        q0 = [10, -40, 60, 5, -30, 0]
        q1 = [40, -20, 30, -30, 40, 25]
        samples, t_s = plan_joint_move(q0, q1, 80, 500, 50.0, VMAX)
        # 逐轴钳制 min(vel_dps=80, VMAX)：J3–J5 用 80 → 最长轴 J4（70°/80 = 1.6406 s）
        expect = max(max(1.875 * abs(q1[i] - q0[i]) / min(80.0, VMAX[i]),
                         math.sqrt(5.7735 * abs(q1[i] - q0[i]) / 500.0))
                     for i in range(6))
        self.assertAlmostEqual(t_s, expect, places=6)
        self.assertAlmostEqual(float(samples[0][0]), q0[0], places=9)
        self.assertAlmostEqual(float(samples[-1][3]), q1[3], places=9)
        # 采样间隔 50 ms
        self.assertAlmostEqual(t_s * 1000 / 50, len(samples) - 1, delta=1.0)

    def test_peak_vel_within_clamped_limit(self):
        """默认 vel_dps=80 超过 J0–J2 上限 60.2 → 钳制后峰值速度仍 ≤60.2。"""
        q0 = [0] * 6
        q1 = [90, -60, 80, 100, -90, 120]
        samples, t_s = plan_joint_move(q0, q1, 80, 500, 50.0, VMAX)
        dt = 0.05
        for i in range(6):
            d = np.abs(np.diff([s[i] for s in samples]))
            peak = float(d.max()) / dt
            self.assertLessEqual(peak, VMAX[i] * 1.02 + 0.2)  # 采样离散化余量

    def test_scalar_api(self):
        """同事接口 vel_dps=80 标量调用不炸；时长由钳制后限速决定。"""
        samples, t_s = plan_joint_move([0] * 6, [60] * 6, 80, 500, 50.0, VMAX)
        self.assertAlmostEqual(t_s, 1.875 * 60 / 60.2, places=6)
        self.assertGreater(len(samples), 4)


class TestLine(unittest.TestCase):
    def setUp(self):
        self.k = Kinematics()

    def test_line_straightness(self):
        """直线：起点→直线终点，FK 轨迹点全部贴直线段（≤2 mm），端点误差 ≤0.5 mm。"""
        q_seed = np.array([10, -40, 60, 0, 30, 0]) * math.pi / 180
        a = self.k.fk(q_seed)
        b = a.copy()
        b[:3, 3] = a[:3, 3] + np.array([0.06, 0.03, -0.05])  # 往下往侧走 9 cm
        samples, t_s = plan_line(self.k, a, b, q_seed, LIMS_RAD, 0.10, 1.0, 50.0, VMAX)
        p0, p1 = a[:3, 3], b[:3, 3]
        seg = p1 - p0
        ll = float(np.linalg.norm(seg))
        worst_end = float(np.linalg.norm(self.k.fk(np.radians(samples[0]))[:3, 3] - p0))
        self.assertLess(worst_end, 5e-4)
        worst_end = float(np.linalg.norm(self.k.fk(np.radians(samples[-1]))[:3, 3] - p1))
        self.assertLess(worst_end, 5e-4)
        worst_dev = 0.0
        for s in samples:
            p = self.k.fk(np.radians(s))[:3, 3]
            dev = np.linalg.norm(np.cross(p - p0, seg)) / ll  # 点到直线距离
            worst_dev = max(worst_dev, float(dev))
        print(f"  line: {len(samples)} 样本 T={t_s:.2f}s 最大直线偏差 {worst_dev * 1000:.2f} mm")
        self.assertLess(worst_dev, 2e-3)
        self.assertGreater(t_s, 0)

    def test_line_rejects_unreachable(self):
        """够不着的目标 → 报错（IK 无解），而不是给出坏轨迹。"""
        q_seed = np.array([10, -40, 60, 0, 30, 0]) * math.pi / 180
        a = self.k.fk(q_seed)
        b = a.copy()
        b[:3, 3] = a[:3, 3] + np.array([0.0, 0.0, -0.60])  # 往下 60 cm，必然不可达
        with self.assertRaises(RuntimeError):
            plan_line(self.k, a, b, q_seed, LIMS_RAD, 0.10, 1.0, 50.0, VMAX)


class TestArc(unittest.TestCase):
    def setUp(self):
        self.k = Kinematics()

    def test_arc_on_circle(self):
        """圆弧：三点定圆（1/4 圆），FK 轨迹点全部贴圆（径向偏差 ≤2 mm）。

        注：同构型下 π 半圆弧的对侧超出该工作空间（IK 收敛不了，路径 IK 会报错），
        这是臂的工作空间特性；测试用 1/4 圆。"""
        q_seed = np.array([10, -40, 60, 0, 30, 0]) * math.pi / 180
        a = self.k.fk(q_seed)
        p0 = a[:3, 3]
        # 构造一个半径 5 cm 的水平圆上的三点（1/4 圆）
        center = p0 + np.array([-0.05, 0.0, 0.0])
        r = 0.05
        def pt(ang):
            return center + np.array([r * math.cos(ang), r * math.sin(ang), 0.0])
        ang0 = math.atan2(p0[1] - center[1], p0[0] - center[0])
        p_via = pt(ang0 + math.pi / 4)
        p_end = pt(ang0 + math.pi / 2)
        pose_via = a.copy(); pose_via[:3, 3] = p_via
        pose_b = a.copy(); pose_b[:3, 3] = p_end
        samples, t_s = plan_arc(self.k, a, pose_via, pose_b, q_seed, LIMS_RAD,
                                0.08, 1.0, 50.0, VMAX)
        worst = 0.0
        for s in samples:
            p = self.k.fk(np.radians(s))[:3, 3]
            worst = max(worst, abs(float(np.linalg.norm(p - center)) - r))
        print(f"  arc: {len(samples)} 样本 T={t_s:.2f}s 最大径向偏差 {worst * 1000:.2f} mm")
        self.assertLess(worst, 2e-3)
        end_err = float(np.linalg.norm(self.k.fk(np.radians(samples[-1]))[:3, 3] - p_end))
        self.assertLess(end_err, 5e-4)

    def test_arc_collinear_degrades_to_line(self):
        """三点共线 → 退化为直线，不报错。"""
        q_seed = np.array([10, -40, 60, 0, 30, 0]) * math.pi / 180
        a = self.k.fk(q_seed)
        p0 = a[:3, 3]
        d = np.array([0.03, 0.0, 0.0])
        pose_via = a.copy(); pose_via[:3, 3] = p0 + 0.5 * d
        pose_b = a.copy(); pose_b[:3, 3] = p0 + d
        samples, _ = plan_arc(self.k, a, pose_via, pose_b, q_seed, LIMS_RAD,
                              0.10, 1.0, 50.0, VMAX)
        seg = pose_b[:3, 3] - p0
        ll = float(np.linalg.norm(seg))
        worst = max(float(np.linalg.norm(np.cross(
            self.k.fk(np.radians(s))[:3, 3] - p0, seg)) / ll) for s in samples)
        self.assertLess(worst, 2e-3)


if __name__ == "__main__":
    unittest.main()
