#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""q3_ripple_check.py —— Q3 关卡复核：流式下发的速度起伏（审查方 2026-10-30）

只读离线分析 TC-007 的 tc007_samples_J*.csv，不连舵机。

做法（不做平滑，避开 30 Hz 采样下 ±3 样本差分会抹平 100 ms 锯齿的问题）：
  1. 相邻两样本求速度 |Δ角度|/Δt（Δt>80 ms 的断档丢弃）
  2. 剔除行程两端各 12% 的样本对（换向区）
  3. 近零占比 = 速度 < 0.2×标称速度 的比例（「每条从零重起」应有周期性近零）
  4. 指令频率起伏 M(T) = 速度在周期 T 上的基波幅值 / 平均速度
     采样时刻不规则，不需与指令时刻对齐；用 T=137 ms（与指令无关）作噪声对照

用法：python3 q3_ripple_check.py tests/runs/2026-10-30-TC-007
"""

import cmath
import csv
import glob
import math
import os
import statistics
import sys

AXIS_SPEEDS = {0: 60.2, 1: 60.2, 2: 60.2, 3: 99.1, 4: 99.1, 5: 99.1, 6: 99.1}
END_MARGIN = 0.12
MAX_DT = 0.08


def modulation(ts, vs, period):
    z = sum(v * cmath.exp(-2j * math.pi * t / period) for t, v in zip(ts, vs))
    return 2 * abs(z) / len(vs) / statistics.mean(vs)


def speeds(rows):
    ang = [p for _, p in rows]
    lo, hi = min(ang), max(ang)
    m = END_MARGIN * (hi - lo)
    ts, vs = [], []
    for (t1, p1), (t2, p2) in zip(rows, rows[1:]):
        dt = t2 - t1
        if dt <= 0 or dt > MAX_DT or min(p1, p2) < lo + m or max(p1, p2) > hi - m:
            continue
        ts.append((t1 + t2) / 2)
        vs.append(abs(p2 - p1) / dt)
    return ts, vs


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else '.'
    print('轴  段      n   均速  p5   p95  近零%  M(100ms) M(50ms) M(137ms对照)')
    for f in sorted(glob.glob(os.path.join(d, 'tc007_samples_J*.csv'))):
        sid = int(os.path.basename(f).split('_J')[1][0])
        data = list(csv.DictReader(open(f, encoding='utf-8')))
        for ph in ('P0_go', 'P100', 'P50'):
            rows = [(float(r['t_mono_s']), float(r['angle_deg'])) for r in data if r['phase'] == ph]
            if len(rows) < 10:
                continue
            ts, vs = speeds(rows)
            if len(vs) < 8:
                continue
            s = sorted(vs)
            n = len(s)
            near0 = 100.0 * sum(v < 0.2 * AXIS_SPEEDS[sid] for v in vs) / n
            print('J%d  %-6s %4d %5.0f %4.0f %5.0f %5.0f   %6.2f   %6.2f   %6.2f'
                  % (sid, ph, n, statistics.mean(vs), s[int(n * .05)], s[int(n * .95) - 1],
                     near0, modulation(ts, vs, 0.1), modulation(ts, vs, 0.05),
                     modulation(ts, vs, 0.137)))


if __name__ == '__main__':
    main()
