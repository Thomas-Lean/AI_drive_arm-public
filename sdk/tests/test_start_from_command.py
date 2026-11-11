# -*- coding: utf-8 -*-
"""轨迹起点取命令角（F87）：负载轴实测比命令角低 1° 时，move_joints 第一帧 J1/J2
仍是命令角（不先掉到实测角）；差超过 3° 的轴退回实测角，避免起步跳变。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # sdk/
sys.path.insert(0, str(Path(__file__).resolve().parent))      # tests/

from d1t_sdk._joint_model import servo_to_joint
from test_park import MODE_STREAM, ParkCtrl
from test_stream_executor import make_arm

QT = [0.0, -40.0, 60.0, 0.0, 30.0, 0.0, 70.0]   # 命令角（舵机角 7）


class TestStartFromCommand(unittest.TestCase):
    def _first_frame(self, q):
        fc = ParkCtrl(q, q_target=QT, j6_current_ma=14)
        arm, emitter = make_arm(fc)
        self.addCleanup(emitter.stop)
        self.addCleanup(arm.close)
        target = [servo_to_joint(i, QT[i]) for i in range(6)]
        target[0] += 15.0   # 只转 J0
        arm.move_joints(target, vel_dps=20)
        frames = [list(c.q) for c in fc.cmds if c.mode == MODE_STREAM]
        self.assertTrue(frames)
        return frames[0]

    def test_sagged_axes_start_at_command(self):
        q = list(QT)
        q[1] -= 1.0   # J1 实测比命令角低 1°
        q[2] += 1.0
        f0 = self._first_frame(q)
        self.assertAlmostEqual(f0[1], QT[1], places=3)
        self.assertAlmostEqual(f0[2], QT[2], places=3)

    def test_far_axis_falls_back_to_measured(self):
        q = list(QT)
        q[1] -= 5.0   # 差 5° > 3°：退回实测
        f0 = self._first_frame(q)
        self.assertAlmostEqual(f0[1], q[1], places=3)
        self.assertAlmostEqual(f0[2], QT[2], places=3)


if __name__ == "__main__":
    unittest.main()
