#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_util — 维修工具小函数（事件日志、轴号解析）。协议常量与消息类型统一从 d1t_sdk 导入
（sys.path 指向仓库 sdk/，见各工具文件头），本目录不再自带协议副本。
"""
import json
import sys
from pathlib import Path

_SDK = Path(__file__).resolve().parents[2] / "sdk"  # 仓库根/sdk
if str(_SDK) not in sys.path:
    sys.path.insert(0, str(_SDK))

def append_event(path: Path, obj: dict) -> None:
    """事件日志（jsonl 追加）：send.py 的发送时刻与 record.py 的会话起止共用此格式。

    t_ns 一律 time.time_ns()（CLOCK_REALTIME），与本机收到 ArmState 的时刻同钟，
    analyze.py 才能把「发送命令」和「收到状态帧」对齐。
    """
    if path is None:
        return  # 不带 --event-log 时静默跳过
    path = Path(path)
    obj = {"t_ns": obj.get("t_ns"), **{k: v for k, v in obj.items() if k != "t_ns"}}
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(obj, ensure_ascii=False) + "\n")


def parse_axes(s: str):
    """'0,2,6' → [0,2,6]；校验范围 0–6 且去重升序。非法时抛 ValueError。"""
    axes = sorted({int(x) for x in s.split(",") if x.strip() != ""})
    if not axes:
        raise ValueError("轴列表为空（axis_mask=0 会被控制器拒收）")
    for a in axes:
        if not 0 <= a <= 6:
            raise ValueError(f"轴号 {a} 超出 0–6")
    return axes


def axes_to_mask(axes) -> int:
    m = 0
    for a in axes:
        m |= 1 << a
    return m
