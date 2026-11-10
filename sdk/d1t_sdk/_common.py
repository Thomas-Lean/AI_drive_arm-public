# -*- coding: utf-8 -*-
"""d1t_sdk 内部共用件：话题 / QoS / 状态名 / 臂参数（唯一来源 sdk/arm_model/d1t.json）。

- 话题与 QoS：TOPIC_CMD / TOPIC_STATE 两条话题，BEST_EFFORT + KEEP_LAST(1) + VOLATILE，
  与臂载控制器一致（话题名字面量只在本文件定义一次）。
- 臂参数（限位、限速、方向/零位标定、夹爪标定表）：全部从 `sdk/arm_model/d1t.json` 加载，
  数值与臂载控制器 `arm_onboard/controller/src/config.hpp` 一致（tests 有自动比对）。
- 发送节拍：控制频率 20 Hz（dt 50 ms）；每收到一帧 ArmState 发一条命令（遥测节拍同步）。
"""
import sys
from pathlib import Path

_PKG_DIR = Path(__file__).resolve().parent
if str(_PKG_DIR) not in sys.path:
    # vendored 消息包（idlc 0.10.2 生成，与臂上部署版本同源；见仓库 arm_onboard/controller/idl/）
    sys.path.insert(0, str(_PKG_DIR))

from cyclonedds.domain import DomainParticipant  # noqa: E402
from cyclonedds.qos import Qos, Policy  # noqa: E402
from cyclonedds.topic import Topic  # noqa: E402

from d1t_interfaces.msg.dds_ import ArmCmd_, ArmState_  # noqa: E402

from ._arm_model import DEFAULT_MODEL_PATH, load_arm_model  # noqa: E402

__all__ = ["ArmCmd_", "ArmState_", "D1ArmError", "dds_qos", "make_participant",
           "make_cmd_topic", "make_state_topic", "TOPIC_CMD", "TOPIC_STATE", "DOMAIN_ID",
           "MODE_STREAM", "MODE_HOLD", "MODE_RELEASE", "MODE_DAMP", "MODE_ESTOP",
           "MODE_RESET", "CTRL_INIT", "CTRL_IDLE", "CTRL_STREAMING", "CTRL_HOLD",
           "CTRL_RELEASED", "CTRL_ESTOP", "CTRL_FAULT", "CTRL_STATE_NAMES", "MODE_NAMES",
           "mode_name", "ARM_MODEL", "SERVO_LIMITS", "AXIS_SPEED_DPS",
           "URDF_VMAX_DPS", "J6_SPEED_DPS", "CONTROL_HZ", "DT_MS", "JOINT_CALIB",
           "GRIPPER_TABLE", "FAULT_WATCHDOG", "FAULT_ESTOP", "FAULT_MTURN",
           "FAULT_SERVO_PROT", "FAULT_COMM_LOSS", "FAULT_J6_THERMAL"]


class D1ArmError(RuntimeError):
    """d1t_sdk 异常（错误以异常抛出，详情见消息文本）。"""


TOPIC_CMD = "rt/d1t/arm_cmd"
TOPIC_STATE = "rt/d1t/arm_state"
DOMAIN_ID = 0

# ArmCmd_.mode
MODE_STREAM, MODE_HOLD, MODE_RELEASE, MODE_DAMP, MODE_ESTOP, MODE_RESET = 0, 1, 2, 3, 4, 5
MODE_NAMES = {MODE_STREAM: "STREAM", MODE_HOLD: "HOLD", MODE_RELEASE: "RELEASE",
              MODE_DAMP: "DAMP", MODE_ESTOP: "ESTOP", MODE_RESET: "RESET"}
# ArmState_.ctrl_state
CTRL_INIT, CTRL_IDLE, CTRL_STREAMING, CTRL_HOLD, CTRL_RELEASED, CTRL_ESTOP, CTRL_FAULT = range(7)
CTRL_STATE_NAMES = {0: "INIT", 1: "IDLE", 2: "STREAMING", 3: "HOLD",
                    4: "RELEASED", 5: "ESTOP", 6: "FAULT"}


def mode_name(ctrl_state, job_active=False):
    """state()["mode"]：SDK 轨迹执行中为 MOVING，否则即控制器状态名（同 CTRL_STATE_NAMES）。"""
    if job_active:
        return "MOVING"
    return CTRL_STATE_NAMES.get(ctrl_state, "FAULT")

# fault / warn 位（任务口径见控制器 README「fault / warn 位」）
FAULT_WATCHDOG = 1 << 0
FAULT_ESTOP = 1 << 1
FAULT_MTURN = 1 << 2
FAULT_SERVO_PROT = 1 << 3
FAULT_COMM_LOSS = 1 << 4
FAULT_J6_THERMAL = 1 << 5

# ---- 臂参数（sdk/arm_model/d1t.json 是唯一来源；换臂改 JSON，不改代码）----
ARM_MODEL = load_arm_model()
# F7 帧的臂限位等数值统一从 d1t.json 读：
SERVO_LIMITS = tuple((lo, hi) for lo, hi in ARM_MODEL["servo_limits_deg"])  # 舵机角空间，含 J6
AXIS_SPEED_DPS = tuple(ARM_MODEL["axis_speed_dps"])                          # 控制器按轴限速
URDF_VMAX_DPS = AXIS_SPEED_DPS[:6]      # 规划用六关节限速（URDF 幅值同舵机角）
J6_SPEED_DPS = ARM_MODEL["j6_speed_dps"]
JOINT_CALIB = tuple((int(s), float(o)) for s, o in
                    zip(ARM_MODEL["calib"]["sign"], ARM_MODEL["calib"]["offset_deg"]))
GRIPPER_TABLE = tuple((float(deg), float(mm))
                      for deg, mm in ARM_MODEL["gripper_table_deg_mm"])

CONTROL_HZ = 20.0  # 控制频率
DT_MS = 50         # 命令插补周期


def dds_qos() -> Qos:
    """两条话题统一 QoS（与控制器一致）。"""
    return Qos(Policy.Reliability.BestEffort,
               Policy.History.KeepLast(1),
               Policy.Durability.Volatile)


def make_participant() -> DomainParticipant:
    return DomainParticipant(DOMAIN_ID)


def make_cmd_topic(dp) -> Topic:
    return Topic(dp, TOPIC_CMD, ArmCmd_)


def make_state_topic(dp) -> Topic:
    return Topic(dp, TOPIC_STATE, ArmState_)
