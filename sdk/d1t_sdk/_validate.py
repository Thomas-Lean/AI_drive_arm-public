# -*- coding: utf-8 -*-
"""用户参数校验：类型 / 个数 / 有限值 / 范围不符一律抛 D1ArmError，不截断、不纠正。

D1Arm 公开方法在读状态、下发命令之前调用这些函数；返回转换好的 float / list[float]。
"""
import math
import numbers
from collections.abc import Sequence

import numpy as np

from ._common import D1ArmError


def _is_num(v) -> bool:
    return isinstance(v, numbers.Real) and not isinstance(v, bool)


def _rng(lo, hi, lo_open, unit) -> str:
    left = "-∞" if lo is None else f"{lo:g}"
    right = "+∞" if hi is None else f"{hi:g}"
    text = f"{'(' if lo_open or lo is None else '['}{left}, {right}{')' if hi is None else ']'}"
    return f"{text} {unit}".rstrip()


def number(name, v, lo=None, hi=None, lo_open=False, unit=""):
    """有限实数，且在 [lo, hi]（lo_open=True 时为 (lo, hi]）内。"""
    rng = _rng(lo, hi, lo_open, unit)
    if not _is_num(v):
        raise D1ArmError(f"参数 {name} 须为数字，收到 {v!r}（可接受 {rng}）")
    x = float(v)
    if (not math.isfinite(x) or (lo is not None and (x <= lo if lo_open else x < lo))
            or (hi is not None and x > hi)):
        raise D1ArmError(f"参数 {name} = {v!r} 超出范围，可接受 {rng}")
    return x


def vector(name, v, n, unit=""):
    """恰好 n 个有限实数（list / tuple / 一维 ndarray）。"""
    what = f"{n} 个数字的列表" + (f"（{unit}）" if unit else "")
    ok_type = (isinstance(v, np.ndarray) and v.ndim == 1) or (
        isinstance(v, Sequence) and not isinstance(v, (str, bytes)))
    if not ok_type:
        raise D1ArmError(f"参数 {name} 须为 {what}，收到 {v!r}")
    if len(v) != n:
        raise D1ArmError(f"参数 {name} 须为 {what}，收到 {len(v)} 个：{list(v)!r}")
    out = []
    for i, x in enumerate(v):
        if not _is_num(x) or not math.isfinite(float(x)):
            raise D1ArmError(f"参数 {name} 第 {i + 1} 个值 {x!r} 不是有限数字（须为 {what}）")
        out.append(float(x))
    return out


def flag(name, v):
    if not isinstance(v, bool):
        raise D1ArmError(f"参数 {name} 须为 True 或 False，收到 {v!r}")
    return v


def only(name, v, value, why=""):
    """只支持一个固定值（bool 与数字不混用：async_=0、grip_power_mw=False 都拒）。"""
    if isinstance(v, bool) != isinstance(value, bool) or not (
            isinstance(value, bool) or _is_num(v)) or v != value:
        raise D1ArmError(f"参数 {name} 只支持 {value!r}，收到 {v!r}" + (f"（{why}）" if why else ""))
    return v


def pose(pos, rpy=None, quat=None, names=("pos", "rpy", "quat")):
    """位姿：pos 3 个数（m）；rpy 3 个数（rad）或 quat 4 个数 [w,x,y,z]（模长 > 0），
    二者至多给一个，都不给 = 姿态 rpy [0, 0, 0]。"""
    n_pos, n_rpy, n_quat = names
    pos = vector(n_pos, pos, 3, "m")
    if rpy is not None and quat is not None:
        raise D1ArmError(f"参数 {n_rpy} 与 {n_quat} 只能给一个（收到 {n_rpy}={rpy!r}、{n_quat}={quat!r}）")
    if rpy is not None:
        rpy = vector(n_rpy, rpy, 3, "rad")
    if quat is not None:
        quat = vector(n_quat, quat, 4, "[w,x,y,z]")
        if math.sqrt(sum(x * x for x in quat)) < 1e-9:
            raise D1ArmError(f"参数 {n_quat} = {quat!r} 模长为 0，不是有效四元数")
    return pos, rpy, quat
