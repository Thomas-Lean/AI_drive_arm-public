#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""grip_probe.py —— J6 空夹 / 夹瓶 电流对照探针（TC-008 现场临时，Thomas 2026-11-02 提出；只出数）

**已由 tc009_grip_char.py（TC-009）取代，未执行**（审查方 2026-11-02 分析后改为正式用例采集，
见 tests/cases/TC-009-夹爪夹持特性采集.md 与 §00.6 第 22 条）；本文件按 Thomas 要求保留存档。

背景：TC-008 两轮夹瓶判窗均未过（F56 窗 [33,34]）：第 1 轮停 32.5° / 557 mA，第 2 轮停 31.6° /
215 mA（0x16 与 0x0A 一致、极差 0）。Thomas 判断两次都夹到了，是绝对角度窗口不覆盖实测；
并提出用 0x16 电流直接区分「夹到 / 未夹到」。本探针量「张开静止 → 空关到 29 → 夹瓶关到 29」
三态的 J6 角度/电流/功率，各读 1 s，关闭后再等 3 s 复读一次看电流是否随时间衰减。
只动 J6；J0–J5 不发任何帧。原始样本进 jsonl（ev=grip_probe）。
用法（臂上，服务已停、Thomas 在场）：python3 grip_probe.py 2>&1 | tee grip_probe_输出_<ts>.txt
**是否执行、如何据此改 TC-008 判法，等 审查方 分析后由 Thomas 定**（未经 审查方 审前不作为验收依据）。
"""
import time

from fsbus import Bus, norm_single_turn

OPEN, CLOSE, PW = 64.5, 29.0, 0   # F56 张开/关闭目标；0x08 power=0（按 J6 保护阈值）


def read_win(b, secs):
    out, t_end = [], time.monotonic() + secs
    while time.monotonic() < t_end:
        f = b.read_monitor(6, timeout=0.05)
        if f and f['verdict'] == 'ok' and f['valid_measure']:
            out.append((round(norm_single_turn(f['angle']), 1), f['current_mA'], f['power_mW'], f['voltage_mV']))
    return out


def show(label, r):
    if not r:
        print('[%s] 无有效样本' % label)
        return
    a = sorted(x[0] for x in r)
    c = sorted(x[1] for x in r)
    p = sorted(x[2] for x in r)
    n = len(r)
    print('[%s] n=%d 角度 min/中位/max=%.1f/%.1f/%.1f  电流mA min/中位/max=%d/%d/%d  功率mW 中位=%d'
          % (label, n, a[0], a[n // 2], a[-1], c[0], c[n // 2], c[-1], p[n // 2]))
    print('[%s] 电流全列：%s' % (label, [x[1] for x in r]))


def cycle(b, label, with_bottle):
    b.set_angle(6, OPEN, 1800, PW)
    time.sleep(2.3)
    r0 = read_win(b, 1.0)
    show(label + '·张开静止', r0)
    b.log('grip_probe', label=label + '_open', samples=r0)
    input('[%s] J6 已张开。%s，按回车关闭到 %.1f：'
          % (label, '放入瓶子（h=108）并扶住' if with_bottle else '两指间不放任何东西', CLOSE))
    b.set_angle(6, CLOSE, 1775, PW)
    time.sleep(1.775 + 0.5)
    r1 = read_win(b, 1.0)
    show(label + '·关闭停稳', r1)
    b.log('grip_probe', label=label + '_closed', samples=r1)
    time.sleep(2.0)
    r2 = read_win(b, 1.0)
    show(label + '·停稳+3s', r2)
    b.log('grip_probe', label=label + '_closed_3s', samples=r2)


def main():
    ts = time.strftime('%Y%m%d_%H%M%S')
    b = Bus('/dev/ttyS4', log_path='grip_probe_%s.jsonl' % ts)
    print('日志: grip_probe_%s.jsonl' % ts)
    try:
        cycle(b, '空夹', False)
        cycle(b, '夹瓶1', True)
        if input('取出瓶子后再夹一次（换个放法）？(y/n)：').strip().lower() == 'y':
            cycle(b, '夹瓶2', True)
    finally:
        input('按回车张开 J6 取瓶（请托住瓶子）：')
        b.set_angle(6, OPEN, 1800, PW)
        time.sleep(2.3)
        f = b.read_angle(6)
        print('J6 =', f['angle'] if f and f['verdict'] == 'ok' else f)
        b.close()


if __name__ == '__main__':
    main()
