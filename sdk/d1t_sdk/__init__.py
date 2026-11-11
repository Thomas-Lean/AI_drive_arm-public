# -*- coding: utf-8 -*-
"""d1t_sdk — AI_drive_arm 上位机 SDK（接口同同事 d1arm_sdk，DDS 底层）。

用法：
    from d1t_sdk import D1Arm
    arm = D1Arm()                       # 需先设 CYCLONEDDS_URI（见实机验收清单）
    arm.home(4.0); arm.wait_idle()
    print(arm.state()["tcp"]["pos"])
    arm.move_joints([0, -40, 60, 0, 30, 0], vel_dps=30)
    arm.gripper(45)
    arm.close()
"""
from ._common import D1ArmError
from ._kinematics import IkParams, Kinematics
from ._trajectory import plan_arc, plan_joint_move, plan_line
from .d1arm import D1Arm

__version__ = "0.1.3"

__all__ = ["D1Arm", "D1ArmError", "Kinematics", "IkParams",
           "plan_joint_move", "plan_line", "plan_arc", "__version__"]
