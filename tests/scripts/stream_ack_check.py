#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""流式命令连续性复算：读 record.py 的 arm_state.jsonl，统计流式期间
cmd_seq_ack 每帧增量（1 = 正常；0 = 沿用旧命令「停一下」；2 = 跳一条「蹭一下」），
并列出每个异常帧与控制器 overruns 增长是否同帧。

用法：python3 tests/scripts/stream_ack_check.py <arm_state.jsonl>
"""
import json
import sys
from collections import Counter


def main(path):
    rows = [json.loads(line) for line in open(path, encoding="utf-8")]
    ack = [r["cmd_seq_ack"] for r in rows]
    cs = [r["ctrl_state"] for r in rows]
    ov = [r["overruns"] for r in rows]
    seq = [r["seq"] for r in rows]
    t0 = rows[0]["rx_local_ns"]
    stream = [i for i in range(1, len(rows)) if cs[i] == 2 and cs[i - 1] == 2]
    dist = Counter(ack[i] - ack[i - 1] for i in stream)
    print(f"帧 {len(rows)}，seq 缺号 {sum(1 for a, b in zip(seq, seq[1:]) if b - a != 1)}，"
          f"overruns {ov[0]}→{ov[-1]}（+{ov[-1] - ov[0]}）")
    print(f"流式帧 {len(stream)}，cmd_seq_ack 增量分布 {dict(dist)}")
    bad = [i for i in stream if ack[i] - ack[i - 1] != 1]
    for i in bad:
        near_ov = any(ov[j] != ov[j - 1] for j in range(max(1, i - 1), min(len(rows), i + 1)))
        dt = (rows[i]["stamp_ns"] - rows[i - 1]["stamp_ns"]) / 1e6
        print(f"  帧 {i} t={(rows[i]['rx_local_ns'] - t0) / 1e9:6.2f}s ack+{ack[i] - ack[i - 1]} "
              f"帧间隔 {dt:5.1f} ms  同帧/前一帧 overrun：{'是' if near_ov else '否'}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1])
