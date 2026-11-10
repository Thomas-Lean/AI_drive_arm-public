import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

import joycon_teleop as tele
from d1t_sdk import D1ArmError
from d1t_sdk._joint_model import gripper_deg_to_mm, gripper_mm_to_deg


class Arm:
    """假 D1Arm：记录调用，模拟最小状态机；夹爪开度可按脚本响应 J6 目标。"""

    def __init__(self):
        self.commands = []
        self.q = np.array([12., -35., 25., 10., 40., 8.])
        self.gripper_mm = 55.
        self.mode = 'IDLE'
        self.fault = 0
        self.seq = 0
        self.ik_q = self.q.copy()
        self.gripper_script = None   # 每次「带 j6 目标的 stream_joints」弹出一个实测开度
        self.freeze_seq = False      # True 时 seq 不再前进（测反馈停更分支）

    def state(self, timeout_s=None):
        self.commands.append(('state',))
        if not self.freeze_seq:
            self.seq += 1
        return dict(mode=self.mode, online=[True] * 7, seq=self.seq, fault=self.fault,
                    q_deg=self.q.tolist(), gripper_mm=self.gripper_mm)

    def fk(self, q_deg):
        self.commands.append(('fk', tuple(q_deg)))
        return dict(pos=[0.3, 0., 0.2], quat=tele.from_rpy([0, 0, np.pi / 2]))

    def ik(self, pos, rpy=None, quat=None, seed=None):
        self.commands.append(('ik', tuple(pos), tuple(quat)))
        return self.ik_q.tolist()

    def stream_start(self):
        self.commands.append(('stream_start',))
        self.mode = 'STREAMING'
        return {'ok': True}

    def stream_joints(self, q_deg, j6_deg=None):
        self.commands.append(('stream_joints', tuple(q_deg), j6_deg))
        if j6_deg is not None:
            if self.gripper_script:
                self.gripper_mm = self.gripper_script.pop(0)
            else:
                self.gripper_mm = gripper_deg_to_mm(j6_deg)
        return {'ok': True}

    def stream_stop(self):
        self.commands.append(('stream_stop',))
        self.mode = 'HOLD'
        return {'ok': True}

    def wait_frame(self, timeout=1.0):
        self.commands.append(('wait_frame',))
        return None

    def close(self):
        self.commands.append(('close',))
        return {'ok': True}

    def names(self):
        return [c[0] for c in self.commands]

    def j6_targets(self):
        return [c[2] for c in self.commands if c[0] == 'stream_joints']


class FakeClock:
    """可控单调时钟：夹爪守卫的停滞 / 超时判定要跨多帧，不能真等 0.6 s。"""

    def __init__(self):
        self.t = 0.

    def __call__(self):
        return self.t


class TeleopTests(unittest.TestCase):
    def setUp(self):
        self.arm = Arm()
        self.args = SimpleNamespace(dry_run=False, j5_margin_deg=5, angular_dps=20,
                                    speed_mps=.04, joint_dps=25, open_mm=55, close_mm=10,
                                    axes_deg=[0, 0, 0], period_s=.05)
        self.sample = tele.Input(tele.from_rpy([.1, .2, -.3]), True, 0, 0, 0, True, False)
        self.session = tele.Session(self.arm, self.args)

    def test_limits_take_min_of_source_and_model(self):
        # 原脚本对称限位与 d1t.json 实测限位逐轴取较小：仅 J2 高侧 89° 比原 90° 紧。
        np.testing.assert_allclose(self.session.limits, [135., 90., 89., 135., 90., 135.])

    def test_acquire_tool_translation_and_reclutch(self):
        self.session.engage(self.sample)
        # 接管首帧发实测角（engage 里最后一条 stream_joints；其后还跟一条流式确认 state）。
        first_stream = [c for c in self.arm.commands if c[0] == 'stream_joints'][-1]
        np.testing.assert_allclose(first_stream[1], self.arm.q)
        pos, rot, _ = self.session.propose(replace(self.sample, forward=1), .05)
        np.testing.assert_allclose(pos, [.3, .002, .2], atol=1e-9)
        np.testing.assert_allclose(rot, self.session.start_rot)
        self.session.release()
        changed = replace(self.sample, wrist=tele.from_rpy([.9, -.3, 1.]))
        self.session.engage(changed)
        np.testing.assert_allclose(self.session.propose(changed, .05)[1], self.session.start_rot)

    def test_orient_relative_to_tcp_with_angle_rate_limit(self):
        self.session.engage(self.sample)
        changed = replace(self.sample, wrist=tele.product(self.sample.wrist, tele.from_rpy([1., 0, 0])))
        _, rot, _ = self.session.propose(changed, .05)
        expected = tele.product(self.session.start_rot, tele.from_rpy([np.deg2rad(1), 0, 0]))
        np.testing.assert_allclose(rot, expected, atol=1e-8)

    def test_joint_limits_jumps_and_unreachable_never_advance_target(self):
        self.arm.q[4] = 84.9
        self.session.engage(self.sample)
        moving = replace(self.sample, lift=1)
        for joint, value in [(4, 85.1), (0, 100)]:
            self.arm.ik_q = self.arm.q.copy()
            self.arm.ik_q[joint] = value
            np.testing.assert_equal(self.session.propose(moving, .05)[0], self.session.pos)
        with patch.object(self.arm, 'ik',
                          side_effect=D1ArmError('IK 无解（位置误差 12.3 mm，姿态误差 4.56°）')):
            np.testing.assert_equal(self.session.propose(moving, .05)[2], self.arm.q)

    def test_release_during_ik_and_gripper_edge(self):
        self.session.engage(self.sample)
        toggled = replace(self.sample, grip_open=False)
        for _ in range(2):
            self.session.tick(toggled, .05, lambda: toggled)
        # engage 首帧（None）→ 边沿帧本帧仍 None → 下一帧起发合拢目标角
        # （d1armd 版的 gripper RPC 换成 j6_deg 随流式下发）。
        self.assertEqual(self.arm.j6_targets(), [None, None, gripper_mm_to_deg(10)])
        self.arm.commands.clear()
        self.session.tick(replace(toggled, forward=1), .05,
                          lambda: replace(toggled, enabled=False))
        self.assertEqual(self.arm.names(), ['state', 'ik', 'stream_stop'])

    def test_grip_guard_arrives_then_holds_command(self):
        self.session.engage(self.sample)
        self.arm.gripper_script = [30., 10.]   # 先在途，后到位
        toggled = replace(self.sample, grip_open=False)
        for _ in range(4):
            self.session.tick(toggled, .05, lambda: toggled)
        # engage 首帧（None）→ 边沿帧 None → 目标角两帧（在途 30 mm 未到位）→ 到位后 None 保持目标角。
        self.assertEqual(self.arm.j6_targets(), [None, None, gripper_mm_to_deg(10),
                                                 gripper_mm_to_deg(10), None])
        self.assertIsNone(self.session.grip_target_deg)

    def test_grip_guard_stall_freezes_j6_at_measured(self):
        clock = FakeClock()
        with patch('joycon_teleop.time.monotonic', clock):
            self.session.engage(self.sample)
            self.arm.gripper_script = [30., 30., 30., 30.]   # 合拢到 30 mm 后被「物体」顶住
            toggled = replace(self.sample, grip_open=False)
            for t in (0., .05, .10, .20, .80, .85):
                clock.t = t
                self.session.tick(toggled, .05, lambda: toggled)
        # engage 首帧（None）→ 边沿帧 None → 三帧目标角 → 冻结角（实测 30 mm）→ 之后保持。
        self.assertEqual(self.arm.j6_targets(), [None, None, gripper_mm_to_deg(10),
                                                 gripper_mm_to_deg(10),
                                                 gripper_mm_to_deg(10),
                                                 gripper_mm_to_deg(30), None])
        self.assertIsNone(self.session.grip_target_deg)

    def test_state_rejects_fault_mode_and_stale_feedback(self):
        self.arm.mode = 'ESTOP'
        with self.assertRaisesRegex(tele.RobotFault, 'Unexpected feedback'):
            self.session.state('IDLE')
        self.arm.mode = 'IDLE'
        self.arm.fault = 8
        with self.assertRaisesRegex(tele.RobotFault, 'fault=8'):
            self.session.state('IDLE')
        self.arm.fault = 0
        self.session.state('IDLE')
        self.arm.freeze_seq = True
        self.session.updated -= 1.0   # seq 不再前进且超时 → 停更
        with self.assertRaisesRegex(tele.RobotFault, 'stalled'):
            self.session.state('IDLE')

    def test_engage_accepts_hold_rejects_released(self):
        self.arm.mode = 'HOLD'   # 上一次 stream_stop 之后控制器的常态
        self.session.engage(self.sample)
        self.assertTrue(self.session.active)
        self.arm.mode = 'RELEASED'
        with self.assertRaisesRegex(tele.RobotFault, 'Unexpected feedback'):
            self.session.engage(self.sample)

    def test_preview_read_only_and_loss_releases_stream(self):
        self.args.dry_run = True
        self.session.engage(self.sample)
        moving = replace(self.sample, lateral=1, grip_open=False)
        self.session.tick(moving, .05, lambda: moving)
        self.session.release()
        self.assertEqual(set(self.arm.names()), {'state', 'fk', 'ik'})
        self.args.dry_run = False
        handset = Mock()
        handset.read.side_effect = [replace(self.sample, enabled=False), self.sample,
                                    RuntimeError('Joy-Con HID reports timed out')]
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            tele.run(handset, self.arm, self.args)
        self.assertEqual(self.arm.commands[-1][0], 'stream_stop')
        # 主循环改由 wait_frame 跟帧（20 Hz），不再 sleep。
        self.assertIn('wait_frame', self.arm.names())


if __name__ == '__main__':
    unittest.main()
