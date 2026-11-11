# -*- coding: utf-8 -*-
"""全部公开方法与命令行的参数校验：类型 / 格式 / 个数 / 非有限值 / 越界 → D1ArmError，
且在读状态、下发命令之前拒绝（不建连接，读状态/下发一律 AssertionError）。"""
import contextlib
import io
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d1t_sdk import d1arm  # noqa: E402
from d1t_sdk.d1arm import D1Arm, D1ArmError  # noqa: E402

NAN, INF = math.nan, math.inf
Q_OK = [0, -40, 60, 0, 30, 0]
POS_OK = [0.30, 0, 0.38]


def _boom(*a, **k):
    raise AssertionError("参数错误不应读状态或下发")


def bare_arm():
    arm = D1Arm.__new__(D1Arm)  # 不建连接
    for name in ("_wait_state", "_submit_traj", "_submit_oneshot", "_send_cmd",
                 "stream_start", "stream_stop", "state"):
        setattr(arm, name, _boom)
    arm.close = lambda: {"ok": True}
    return arm


# 每个方法：(调用, 说明) —— 每项都必须抛 D1ArmError 且不读状态 / 不下发
BAD_CALLS = {
    "D1Arm()": [
        lambda a: D1Arm(timeout="30"), lambda a: D1Arm(timeout=0), lambda a: D1Arm(timeout=NAN),
    ],
    "state": [
        lambda a: D1Arm.state(a, "5"), lambda a: D1Arm.state(a, -1), lambda a: D1Arm.state(a, INF),
    ],
    "wait_idle": [
        lambda a: a.wait_idle("30"), lambda a: a.wait_idle(0), lambda a: a.wait_idle(NAN),
    ],
    "wait_frame": [
        lambda a: a.wait_frame(None), lambda a: a.wait_frame(-1.0), lambda a: a.wait_frame(INF),
    ],
    "move_joints": [
        lambda a: a.move_joints("0,-40,60,0,30,0"),            # 格式
        lambda a: a.move_joints([0, -40, 60, 0, 30]),          # 个数
        lambda a: a.move_joints([0, -40, 60, 0, 30, 0, 0]),    # 个数
        lambda a: a.move_joints([0, -40, "60", 0, 30, 0]),     # 类型
        lambda a: a.move_joints([0, -40, NAN, 0, 30, 0]),      # 非有限
        lambda a: a.move_joints([170, -40, 60, 0, 30, 0]),     # 越限位
        lambda a: a.move_joints(Q_OK, vel_dps=200),            # 超速
        lambda a: a.move_joints(Q_OK, vel_dps=0),
        lambda a: a.move_joints(Q_OK, vel_dps="30"),
        lambda a: a.move_joints(Q_OK, acc_dps2=-1),
        lambda a: a.move_joints(Q_OK, async_=1),
    ],
    "home": [
        lambda a: a.home("4"), lambda a: a.home(0.1), lambda a: a.home(NAN),
        lambda a: a.home(4.0, async_="yes"),
    ],
    "park": [
        lambda a: a.park(vel_dps="20"), lambda a: a.park(vel_dps=150), lambda a: a.park(vel_dps=-5),
        lambda a: a.park(async_=True), lambda a: a.park(async_=0),
    ],
    "gripper": [
        lambda a: a.gripper("45"), lambda a: a.gripper(233), lambda a: a.gripper(3.9),
        lambda a: a.gripper(NAN), lambda a: a.gripper([45]),
        lambda a: a.gripper(45, vel_dps=30), lambda a: a.gripper(45, vel_dps=0),
        lambda a: a.gripper(45, vel_dps="10"),
        lambda a: a.gripper(45, grip_power_mw=500), lambda a: a.gripper(45, grip_power_mw=False),
        lambda a: a.gripper(45, grip_power_mw="0"),
        lambda a: a.gripper(45, async_=None),
    ],
    "stream_joints": [
        lambda a: a.stream_joints(None), lambda a: a.stream_joints([0] * 5),
        lambda a: a.stream_joints([0, -40, 60, 0, 30, INF]),
        lambda a: a.stream_joints([170, -40, 60, 0, 30, 0]),
        lambda a: a.stream_joints(Q_OK, j6_deg="40"), lambda a: a.stream_joints(Q_OK, j6_deg=70),
        lambda a: a.stream_joints(Q_OK, j6_deg=NAN),
    ],
    "stream_run": [
        lambda a: a.stream_run(None), lambda a: a.stream_run(lambda k, s: None, max_frames=0),
        lambda a: a.stream_run(lambda k, s: None, max_frames=2.5),
        lambda a: a.stream_run(lambda k, s: None, max_frames=True),
    ],
    "fk": [
        lambda a: a.fk("0,0,0,0,0,0"), lambda a: a.fk([0] * 7), lambda a: a.fk([0, 0, 0, 0, 0, NAN]),
    ],
    "ik": [
        lambda a: a.ik("0.3,0,0.38"), lambda a: a.ik([0.3, 0]), lambda a: a.ik([0.3, 0, NAN]),
        lambda a: a.ik(POS_OK, rpy=[0, 0]), lambda a: a.ik(POS_OK, quat=[1, 0, 0]),
        lambda a: a.ik(POS_OK, quat=[0, 0, 0, 0]),
        lambda a: a.ik(POS_OK, rpy=[0, 0, 0], quat=[1, 0, 0, 0]),
        lambda a: a.ik(POS_OK, seed=[0] * 7), lambda a: a.ik(POS_OK, seed=[0, 0, 0, 0, 0, "x"]),
    ],
    "move_j": [
        lambda a: a.move_j([0.3, 0]), lambda a: a.move_j(POS_OK, rpy=[0, 0, INF]),
        lambda a: a.move_j(POS_OK, vel_dps=101), lambda a: a.move_j(POS_OK, acc_dps2="500"),
        lambda a: a.move_j(POS_OK, async_="no"),
    ],
    "move_l": [
        lambda a: a.move_l([0.3, 0, 0.38, 0]), lambda a: a.move_l(POS_OK, rpy="0,0,0"),
        lambda a: a.move_l(POS_OK, vel=0), lambda a: a.move_l(POS_OK, vel=NAN),
        lambda a: a.move_l(POS_OK, acc=-2), lambda a: a.move_l(POS_OK, async_=1),
        lambda a: a.move_l(POS_OK, rpy=[0, 0, 0], quat=[1, 0, 0, 0]),
    ],
    "move_c": [
        lambda a: a.move_c(POS_OK, via=[0.3, 0]), lambda a: a.move_c(POS_OK, via=None),
        lambda a: a.move_c(POS_OK, via=POS_OK, via_quat=[0, 0, 0, 0]),
        lambda a: a.move_c(POS_OK, via=POS_OK, vel="0.1"), lambda a: a.move_c(POS_OK, via=POS_OK, acc=0),
        lambda a: a.move_c(POS_OK, via=POS_OK, async_=None),
    ],
    "stream_pose": [
        lambda a: a.stream_pose([0.3]), lambda a: a.stream_pose(POS_OK, quat=[1, 0, 0, NAN]),
        lambda a: a.stream_pose(POS_OK, j6_deg=-31), lambda a: a.stream_pose(POS_OK, j6_deg="0"),
    ],
}


class MethodValidationTest(unittest.TestCase):
    def test_bad_params_rejected_before_state(self):
        for method, calls in BAD_CALLS.items():
            for k, call in enumerate(calls):
                with self.subTest(method=method, case=k):
                    arm = bare_arm()
                    with mock.patch.object(d1arm, "_RealTransport", side_effect=_boom):
                        with self.assertRaises(D1ArmError):
                            call(arm)

    def test_message_names_param_value_range(self):
        arm = bare_arm()
        with self.assertRaises(D1ArmError) as cm:
            arm.move_joints(Q_OK, vel_dps=200)
        msg = str(cm.exception)
        for part in ("vel_dps", "200", "(0, 100]"):
            self.assertIn(part, msg)
        with self.assertRaisesRegex(D1ArmError, "grip_power_mw 只支持 0"):
            arm.gripper(45, grip_power_mw=100)
        with self.assertRaisesRegex(D1ArmError, r"\[4, 70\] mm"):
            arm.gripper(233)

    def test_numpy_inputs_accepted(self):
        """numpy 数组 / 标量是合法输入（遥操作等调用方在用）。"""
        import numpy as np
        from d1t_sdk import _validate as v
        self.assertEqual(v.vector("q", np.zeros(6), 6), [0.0] * 6)
        self.assertEqual(v.number("x", np.float32(1.5), 0, 2), 1.5)
        self.assertEqual(v.pose(np.array(POS_OK), quat=(1, 0, 0, 0))[2], [1.0, 0.0, 0.0, 0.0])


class CliValidationTest(unittest.TestCase):
    def _run(self, argv, arm_factory=None):
        err, out = io.StringIO(), io.StringIO()
        factory = arm_factory or mock.Mock(side_effect=AssertionError("参数错误不应连臂"))
        with mock.patch.object(sys, "argv", ["d1t_sdk"] + argv), \
                mock.patch.object(d1arm, "D1Arm", factory), \
                contextlib.redirect_stderr(err), contextlib.redirect_stdout(out):
            code = d1arm._cli()
        return code, err.getvalue().splitlines()

    def test_parse_errors_exit2_with_usage_no_arm(self):
        cases = [
            [],                                   # 缺指令
            ["frobnicate"],                       # 未知指令
            ["state", "x"], ["park", "1"],        # 多余参数
            ["joint"], ["joint", "1,2,3,4,5,6", "7"],  # 参数个数
            ["joint", "1,2,x,4,5,6"],             # 不是数字
            ["joint", "1,2,3,4,5"],               # 逗号个数
            ["joint", "1,2,3,4,5,nan"],           # 非有限
            ["line", "0.3,0"], ["line", "0.3,0,0.38,0"], ["line", "0.3,0,inf"],
            ["gripper"], ["gripper", "45,5"], ["gripper", "abc"], ["gripper", ""],
        ]
        for argv in cases:
            with self.subTest(argv=argv):
                code, lines = self._run(argv)
                self.assertEqual(code, 2)
                self.assertEqual(len(lines), 2, lines)
                self.assertTrue(lines[0].startswith("失败："), lines)
                self.assertEqual(lines[1], d1arm.CLI_USAGE)

    def test_range_errors_exit2_one_line_no_motion(self):
        """数值能解析但越界：由方法校验拒绝，一行「失败：」，不读状态不下发。"""
        for argv in (["gripper", "233"], ["gripper", "3"], ["joint", "170,0,0,0,0,0"]):
            with self.subTest(argv=argv):
                code, lines = self._run(argv, arm_factory=lambda: bare_arm())
                self.assertEqual(code, 2)
                self.assertEqual(len(lines), 1, lines)
                self.assertTrue(lines[0].startswith("失败："), lines)
                self.assertNotIn("Traceback", "\n".join(lines))


if __name__ == "__main__":
    unittest.main()
