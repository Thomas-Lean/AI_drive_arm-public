# -*- coding: utf-8 -*-
"""_base7 不动轴保持目标离线测试：本进程未下过令时，gripper() 的 J0–J5 取控制器
q_target（有限值），不取下沉后的实测 q；q_target 为 NaN 时退回实测 q。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sdk/
sys.path.insert(0, str(Path(__file__).resolve().parent))      # tests/

from test_park import MODE_STREAM, ParkCtrl
from test_stream_executor import make_arm

QT = [0.0, -40.0, 60.0, 0.0, 30.0, 0.0, 29.0]


class TestBase7Hold(unittest.TestCase):
    def _gripper_stream(self, q, qt):
        fc = ParkCtrl(q, q_target=qt, j6_current_ma=14)
        arm, emitter = make_arm(fc)
        self.addCleanup(emitter.stop)
        self.addCleanup(arm.close)
        arm.gripper(70)
        return [list(c.q) for c in fc.cmds if c.mode == MODE_STREAM]

    def test_holds_q_target_not_sagged_q(self):
        q = list(QT)
        q[1] -= 2.0   # J1 下沉 2°
        q[2] += 2.0
        frames = self._gripper_stream(q, QT)
        self.assertTrue(frames)
        for f in frames:
            for i in range(6):
                self.assertAlmostEqual(f[i], QT[i], places=4)

    def test_nan_q_target_falls_back_to_q(self):
        q = list(QT)
        q[1] -= 2.0
        frames = self._gripper_stream(q, [math.nan] * 7)
        self.assertTrue(frames)
        for f in frames:
            for i in range(6):
                self.assertAlmostEqual(f[i], q[i], places=4)


if __name__ == "__main__":
    unittest.main()
