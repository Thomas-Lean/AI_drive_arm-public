"""gripper(mm) 超出 [4, 70] mm 报错且不发运动帧（原先静默截断，gripper 233 也会执行）。"""
import math
import unittest

from d1t_sdk.d1arm import D1Arm, D1ArmError


class GripperRangeTest(unittest.TestCase):
    def test_out_of_range_rejected_before_state(self):
        arm = D1Arm.__new__(D1Arm)  # 不建连接：越界须在读状态/下发之前拒绝

        def boom(*a, **k):
            raise AssertionError("越界不应读状态或下发")
        arm._wait_state = boom
        arm._submit_traj = boom
        for mm in (233, 70.5, 3.9, -1, math.nan):
            with self.assertRaises(D1ArmError):
                arm.gripper(mm)


if __name__ == "__main__":
    unittest.main()
