# -*- coding: utf-8 -*-
"""d1t_sdk 臂模型加载：sdk/arm_model/d1t.json 是唯一的臂参数来源。

加载器不做 URDF 解析：d1t.json 里的数值人工核对自 arm_model/d1_description 的 URDF
（逐项一致），换臂时改 JSON 即可（见 sdk/arm_model/README.md）。
"""
import json
from pathlib import Path

DEFAULT_MODEL_PATH = Path(__file__).resolve().parent.parent / "arm_model" / "d1t.json"

_REQUIRED = ("joints", "tcp", "calib", "servo_limits_deg", "axis_speed_dps",
             "j6_speed_dps", "gripper_table_deg_mm")


def load_arm_model(path=None) -> dict:
    """读臂模型 JSON，数值全部规整成 float。path 缺省用 sdk/arm_model/d1t.json。

    返回 dict（键同 JSON 文件）；结构不对抛 D1ArmError。
    """
    from ._common import D1ArmError
    p = Path(path) if path is not None else DEFAULT_MODEL_PATH
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except OSError as e:
        raise D1ArmError(f"臂模型文件读不到：{p}（{e}）")
    except json.JSONDecodeError as e:
        raise D1ArmError(f"臂模型文件不是合法 JSON：{p}（{e}）")
    missing = [k for k in _REQUIRED if k not in data]
    if missing:
        raise D1ArmError(f"臂模型缺字段 {missing}：{p}")

    def f(v):
        return [float(x) for x in v]

    data["joints"] = [{"xyz": f(j["xyz"]), "rpy": f(j["rpy"]), "axis": f(j["axis"])}
                      for j in data["joints"]]
    data["tcp"] = {"xyz": f(data["tcp"]["xyz"]), "rpy": f(data["tcp"]["rpy"])}
    data["calib"] = {"sign": [int(s) for s in data["calib"]["sign"]],
                     "offset_deg": f(data["calib"]["offset_deg"])}
    data["servo_limits_deg"] = [f(pair) for pair in data["servo_limits_deg"]]
    data["axis_speed_dps"] = f(data["axis_speed_dps"])
    data["j6_speed_dps"] = float(data["j6_speed_dps"])
    data["gripper_table_deg_mm"] = [f(pair) for pair in data["gripper_table_deg_mm"]]
    if len(data["joints"]) != 6 or len(data["calib"]["sign"]) != 6:
        raise D1ArmError(f"臂模型 joints/calib 必须 6 关节：{p}")
    if len(data["servo_limits_deg"]) != 7 or len(data["axis_speed_dps"]) != 7:
        raise D1ArmError(f"臂模型限位/限速必须 7 轴（含 J6）：{p}")
    return data
