#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record.py — 录 rt/d1t/arm_state 到 jsonl + csv（P1.1-T4）。

用法：
  python3 record.py --duration 75 --out sim_runs/stream_001

要点（控制器 README §6 实测）：BEST_EFFORT + KEEP_LAST(1) 下轮询读会丢帧，
必须用 listener 回调在回调里立刻 take()。每帧另记本机收到时刻 rx_local_ns
（time.time_ns()，与 send.py 的 stamp_ns 同钟，E3 带内估算依赖）。

输出：
  <out>/arm_state.jsonl   每行一个 dict（rx_local_ns + ArmState 全字段，数组为 list）
  <out>/arm_state.csv     展平成 67 列
  <out>/run_meta.json     起止时刻、参数、样本数（R1：分析时写明样本范围用）
"""
import argparse
import csv
import json
import queue
import sys
import threading
import time
from pathlib import Path

_SDK = Path(__file__).resolve().parents[2] / "sdk"  # 仓库根/sdk
if str(_SDK) not in sys.path:
    sys.path.insert(0, str(_SDK))  # 协议常量与消息类型统一从 d1t_sdk 导入

from cyclonedds.core import Listener
from cyclonedds.sub import DataReader

from _util import append_event
from d1t_sdk._common import dds_qos, make_participant, make_state_topic

CSV_PREFIX_COLS = ["rx_local_ns", "seq", "stamp_ns", "sample_span_us", "ctrl_state",
                   "cmd_seq_ack", "cmd_stamp_ns", "cmd_rx_ns", "valid_mask",
                   "fault", "warn", "overruns"]
CSV_ARRAY_COLS = ["q", "q_target", "current_ma", "voltage_mv", "temp_c", "status", "turns"]


def parse_args():
    p = argparse.ArgumentParser(description="录 ArmState（jsonl+csv）")
    p.add_argument("--duration", type=float, default=60.0, help="录制时长 s（默认 60）")
    p.add_argument("--out", required=True, help="输出目录（自动创建）")
    p.add_argument("--event-log", default=None, help="会话起止事件写入的 jsonl（与 send.py 同格式）")
    return p.parse_args()


class StateListener(Listener):
    """on_data_available 里立刻 take()——KEEP_LAST(1) 下唯一不丢帧的取法。"""

    def __init__(self, q):
        super().__init__()
        self.q = q

    def on_data_available(self, reader):
        for s in reader.take():
            self.q.put((time.time_ns(), s))


def sample_to_row(rx_ns, s) -> dict:
    row = {
        "rx_local_ns": rx_ns,
        "seq": s.seq, "stamp_ns": s.stamp_ns, "sample_span_us": s.sample_span_us,
        "ctrl_state": s.ctrl_state, "cmd_seq_ack": s.cmd_seq_ack,
        "cmd_stamp_ns": s.cmd_stamp_ns, "cmd_rx_ns": s.cmd_rx_ns,
        "valid_mask": s.valid_mask,
        "q": list(s.q), "q_target": list(s.q_target),
        "current_ma": list(s.current_ma), "voltage_mv": list(s.voltage_mv),
        "temp_c": list(s.temp_c), "status": list(s.status), "turns": list(s.turns),
        "fault": s.fault, "warn": s.warn, "overruns": s.overruns,
    }
    return row


def write_csv(path: Path, rows):
    if not rows:
        return
    cols = CSV_PREFIX_COLS + [f"{name}{i}" for name in CSV_ARRAY_COLS for i in range(7)]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in rows:
            flat = [r[c] for c in CSV_PREFIX_COLS]
            for name in CSV_ARRAY_COLS:
                flat += list(r[name])
            w.writerow(flat)


def main():
    args = parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    q = queue.Queue()
    dp = make_participant()
    listener = StateListener(q)
    rd = DataReader(dp, make_state_topic(dp), dds_qos(), listener=listener)

    t0_ns = time.time_ns()
    if args.event_log:
        append_event(args.event_log, {"event": "record_start", "path": str(out)})
    print(f"录制开始：{args.duration}s → {out}/（listener 回调取数，KEEP_LAST(1) 不丢帧）")

    rows = []
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        try:
            rx_ns, s = q.get(timeout=0.2)
            rows.append(sample_to_row(rx_ns, s))
        except queue.Empty:
            continue

    t1_ns = time.time_ns()
    if args.event_log:
        append_event(args.event_log, {"event": "record_end", "n": len(rows)})

    jsonl_path = out / "arm_state.jsonl"
    with jsonl_path.open("w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    write_csv(out / "arm_state.csv", rows)

    span = (t1_ns - t0_ns) / 1e9
    meta = {
        "t_start_ns": t0_ns, "t_end_ns": t1_ns, "duration_s": args.duration,
        "wall_s": span, "n_frames": len(rows),
        "rate_hz": (len(rows) / span) if span > 0 else 0.0,
        "topic": "rt/d1t/arm_state", "qos": "BEST_EFFORT+KEEP_LAST(1)+VOLATILE",
        "rx_clock": "time.time_ns() CLOCK_REALTIME",
    }
    (out / "run_meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=1),
                                       encoding="utf-8")
    print(f"录制完成：{len(rows)} 帧 / {span:.2f}s = {meta['rate_hz']:.2f} Hz"
          f" → {jsonl_path}")
    return 0  # cyclonedds-python 无 Participant.close()，进程退出即释放


if __name__ == "__main__":
    sys.exit(main())
