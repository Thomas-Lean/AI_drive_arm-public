import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

import joycon_teleop as tele


class Arm:
    def __init__(self):
        self.commands = []
        self.q = np.array([12., -35., 25., 10., 40., 8.])
        self.mode = 'IDLE'
        self.ik_q = self.q.copy()

    def call(self, cmd, **args):
        self.commands.append((cmd, args))
        if cmd == 'state':
            return dict(state=dict(mode=self.mode, online=[True]*7, stamp_ms=1234,
                                   q_deg=self.q.tolist(), gripper_mm=55))
        if cmd == 'fk':
            return dict(pose=dict(pos=[0.3, 0., 0.2], quat=tele.from_rpy([0,0,np.pi/2])))
        if cmd == 'ik':
            return dict(q_deg=self.ik_q.tolist())
        if cmd == 'stream_start':
            self.mode = 'STREAMING'
        if cmd == 'stream_stop':
            self.mode = 'IDLE'
        return {'ok': True}


class TeleopTests(unittest.TestCase):
    def setUp(self):
        self.arm = Arm()
        self.args = SimpleNamespace(dry_run=False, j5_margin_deg=5, hz=20, angular_dps=20,
                                    speed_mps=.04, joint_dps=25, open_mm=55, close_mm=10,
                                    axes_deg=[0,0,0])
        self.sample = tele.Input(tele.from_rpy([.1,.2,-.3]), True, 0, 0, 0, True, False)
        self.session = tele.Session(self.arm, self.args)

    def test_acquire_tool_translation_and_reclutch(self):
        self.session.engage(self.sample)
        np.testing.assert_allclose(self.arm.commands[-1][1]['q'], self.arm.q)
        pos, rot, _ = self.session.propose(replace(self.sample, forward=1), .05)
        np.testing.assert_allclose(pos, [.3,.002,.2], atol=1e-9)
        np.testing.assert_allclose(rot, self.session.start_rot)
        self.session.release()
        changed = replace(self.sample, wrist=tele.from_rpy([.9,-.3,1.]))
        self.session.engage(changed)
        np.testing.assert_allclose(self.session.propose(changed,.05)[1], self.session.start_rot)

    def test_orient_relative_to_tcp_with_angle_rate_limit(self):
        self.session.engage(self.sample)
        changed = replace(self.sample, wrist=tele.product(self.sample.wrist, tele.from_rpy([1.,0,0])))
        _, rot, _ = self.session.propose(changed, .05)
        expected = tele.product(self.session.start_rot, tele.from_rpy([np.deg2rad(1),0,0]))
        np.testing.assert_allclose(rot, expected, atol=1e-8)

    def test_joint_limits_jumps_and_unreachable_never_advance_target(self):
        self.arm.q[4] = 84.9
        self.session.engage(self.sample)
        moving = replace(self.sample, lift=1)
        for joint, value in [(4,85.1),(0,100)]:
            self.arm.ik_q = self.arm.q.copy()
            self.arm.ik_q[joint] = value
            np.testing.assert_equal(self.session.propose(moving,.05)[0], self.session.pos)
        with patch.object(self.arm, 'call', side_effect=tele.RobotFault('ik no solution')):
            np.testing.assert_equal(self.session.propose(moving,.05)[2], self.arm.q)

    def test_release_during_ik_and_gripper_edge(self):
        self.session.engage(self.sample)
        toggled = replace(self.sample, grip_open=False)
        for _ in range(2):
            self.session.tick(toggled,.05,lambda:toggled)
        grips = [args for cmd,args in self.arm.commands if cmd == 'gripper']
        self.assertEqual(grips,[dict(mm=10)])
        self.arm.commands.clear()
        self.session.tick(replace(toggled,forward=1), .05, lambda:replace(toggled,enabled=False))
        self.assertEqual([cmd for cmd,_ in self.arm.commands], ['state','ik','stream_stop'])

    def test_preview_read_only_and_loss_releases_stream(self):
        self.args.dry_run = True
        self.session.engage(self.sample)
        moving = replace(self.sample,lateral=1,grip_open=False)
        self.session.tick(moving,.05,lambda:moving)
        self.session.release()
        self.assertEqual(set(cmd for cmd,_ in self.arm.commands), {'state','fk','ik'})
        self.args.dry_run = False
        handset = Mock()
        handset.read.side_effect = [replace(self.sample,enabled=False), self.sample,
                                    RuntimeError('Joy-Con HID reports timed out')]
        with self.assertRaisesRegex(RuntimeError,'timed out'):
            tele.run(handset, self.arm, self.args)
        self.assertEqual(self.arm.commands[-1][0],'stream_stop')


if __name__ == '__main__':
    unittest.main()
