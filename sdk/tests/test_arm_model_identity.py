# -*- coding: utf-8 -*-
"""臂模型一致性测试：
① 重构金标——FK/IK/轨迹/换算结果与参数外置改造前逐位相同（防 d1t.json 化引入数值漂移）；
② d1t.json 限位/限速与臂载控制器 src/config.hpp 常量一致。"""
import math
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from d1t_sdk import IkParams, Kinematics
from d1t_sdk._arm_model import load_arm_model
from d1t_sdk._kinematics import to_rpy
from d1t_sdk._joint_model import (deg2rad, gripper_deg_to_mm, gripper_mm_to_deg,
                                  joint_to_servo, servo_to_joint, urdf_limits)
from d1t_sdk._trajectory import plan_joint_move, plan_line

REPO = Path(__file__).resolve().parents[2]

# 改造前（常量硬编码在 _kinematics.py/_common.py 时）的输出，repr 字符串保真（double 逐位）。
# 比对用相对误差 1e-12：同一台机器上逐位相同；跨 CPU/numpy（Mac arm64 ↔ 上位机 x86）末位会差 1–2 ulp。
# 改臂型/重标定后这些值会变，属预期：需重新生成金标（见 sdk/arm_model/README.md）。
GOLD = {
    "fk_zero_tcp": ["0.35098", "-0.0012779999999999192", "0.4255418"],
    "fk_q1_tcp": ["0.366328520199641", "-0.010289040165613704", "0.547792407703207"],
    "rpy_q1": ["0.016232418987300056", "-0.7608368866762805", "-0.5730557524932114"],
    "ik_q_deg": ["10.000885111801573", "19.914570080314824", "-29.80238772988371",
                 "39.910483599027884", "-50.07036553547423", "59.873803318199954"],
    "ik_err": ["0.0002980646285538455", "0.0005445682733585926"],
    "jm_T": "0.9343853820598006",
    "jm_n": 20,
    "jm_mid": ["0.0", "-23.031483089694387", "43.03148308969439", "0.0",
               "30.0", "14.14043075858801"],
    "line_T": "2.316326746010939",
    "line_n": 48,
    "line_last": ["22.6228873255027", "-12.417367984644232", "66.48407939049068",
                  "-112.5462835960431", "8.747479603918233", "-122.51934665261585"],
    "mm2deg_45": "29.2",
    "deg2mm_29p2": "45.0",
    "servo_to_joint_0_10": "-10.0",
}


class TestBitIdentical(unittest.TestCase):
    def _close(self, got, gold_repr):
        want = float(gold_repr)
        self.assertTrue(math.isclose(float(got), want, rel_tol=1e-12, abs_tol=1e-15),
                        f"{float(got)!r} != {gold_repr}")

    """参数外置（d1t.json）前后结果逐位相同。"""

    @classmethod
    def setUpClass(cls):
        cls.k = Kinematics()
        D = math.pi / 180
        cls.q1 = [10 * D, 20 * D, -30 * D, 40 * D, -50 * D, 60 * D]
        lims = urdf_limits()
        cls.lims_rad = ([deg2rad(lo) for lo, _ in lims], [deg2rad(hi) for _, hi in lims])

    def test_fk(self):
        tz = self.k.fk([0.0] * 6)
        t1 = self.k.fk(self.q1)
        r1 = to_rpy(t1)
        for i in range(3):
            self._close(tz[i, 3], GOLD["fk_zero_tcp"][i])
            self._close(t1[i, 3], GOLD["fk_q1_tcp"][i])
            self._close(r1[i], GOLD["rpy_q1"][i])

    def test_ik(self):
        res = self.k.ik(self.k.fk(self.q1), [x * 0.5 for x in self.q1],
                        self.lims_rad[0], self.lims_rad[1], IkParams(rng_seed=42))
        self.assertTrue(res.success)
        for i in range(6):
            self._close(res.q_rad[i] * 180 / math.pi, GOLD["ik_q_deg"][i])
        self._close(res.pos_err, GOLD["ik_err"][0])
        self._close(res.rot_err, GOLD["ik_err"][1])

    def test_joint_move(self):
        samples, t_s = plan_joint_move([0, -40, 60, 0, 30, 0], [0, -10, 30, 0, 30, 25],
                                       80, 500, 50.0, (60.2,) * 3 + (99.1,) * 3)
        self.assertEqual(len(samples), GOLD["jm_n"])
        self._close(t_s, GOLD["jm_T"])
        mid = samples[len(samples) // 2]
        for i in range(6):
            self._close(mid[i], GOLD["jm_mid"][i])

    def test_line(self):
        seed = np.radians([10, -40, 60, 0, 30, 0])
        a = self.k.fk(seed)
        b = a.copy()
        b[:3, 3] = a[:3, 3] + np.array([0.06, 0.03, -0.05])
        samples, t_s = plan_line(self.k, a, b, seed, self.lims_rad,
                                 0.10, 1.0, 50.0, (60.2,) * 3 + (99.1,) * 3)
        self.assertEqual(len(samples), GOLD["line_n"])
        self._close(t_s, GOLD["line_T"])
        for i in range(6):
            self._close(samples[-1][i], GOLD["line_last"][i])

    def test_conversions(self):
        self._close(gripper_mm_to_deg(45.0), GOLD["mm2deg_45"])
        self._close(gripper_deg_to_mm(29.2), GOLD["deg2mm_29p2"])
        self._close(servo_to_joint(0, 10.0), GOLD["servo_to_joint_0_10"])


def _parse_config_hpp():
    """从控制器 config.hpp 解析 kJointLimit（7 对 float）与 kAxisSpeedDps（7 个 float）。"""
    src = (REPO / "arm_onboard" / "controller" / "src" / "config.hpp").read_text(
        encoding="utf-8")
    m = re.search(r"kJointLimit\s*=\s*\{\{(.*?)\}\};", src, re.S)
    if not m:
        raise AssertionError("config.hpp 里找不到 kJointLimit")
    pairs = re.findall(r"\{\s*(-?\d+(?:\.\d+)?)f?\s*,\s*(-?\d+(?:\.\d+)?)f?\s*\}", m.group(1))
    m2 = re.search(r"kAxisSpeedDps\s*=\s*\{(.*?)\};", src, re.S)
    if not m2:
        raise AssertionError("config.hpp 里找不到 kAxisSpeedDps")
    speeds = re.findall(r"(-?\d+(?:\.\d+)?)f", m2.group(1))
    return [[float(lo), float(hi)] for lo, hi in pairs], [float(v) for v in speeds]


class TestLimitsMatchController(unittest.TestCase):
    """d1t.json 限位/限速 ↔ 控制器 config.hpp 常量一致（两侧是同一套闸，必须同值）。"""

    def test_servo_limits_and_speeds(self):
        ctrl_limits, ctrl_speeds = _parse_config_hpp()
        model = load_arm_model()
        self.assertEqual(len(ctrl_limits), 7, "控制器限位应 7 轴")
        self.assertEqual(len(ctrl_speeds), 7, "控制器限速应 7 轴")
        self.assertEqual(model["servo_limits_deg"], ctrl_limits,
                         "d1t.json servo_limits_deg 与控制器 kJointLimit 不一致")
        self.assertEqual(model["axis_speed_dps"], ctrl_speeds,
                         "d1t.json axis_speed_dps 与控制器 kAxisSpeedDps 不一致")

    def test_urdf_limits_derivation(self):
        """_joint_model 的 URDF 限位 = F 舵机限位经 sign 换算（区间翻转）。"""
        model = load_arm_model()
        signs = model["calib"]["sign"]
        lims = urdf_limits()
        for i in range(6):
            slo, shi = model["servo_limits_deg"][i]
            lo, hi = sorted((signs[i] * slo, signs[i] * shi))
            self.assertEqual(lims[i], (lo, hi))


if __name__ == "__main__":
    unittest.main()
