#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tc009_grip_char.py —— TC-009 夹爪夹持特性采集 v2（P0.5-T4b 定稿任务单，2026-11-03；只出数）

v2 按 审查方 定稿任务单整份重写（取代 T4 版；T4 版首轮因 J6 多圈 388.6° 事故中止，见
tests/runs/2026-11-03-TC-009/）。

【驱控三层（用例「目的」）】①接触检测＝0x16 电流突升+角速度骤降（主判据，目标 ~0.1 s）；
②夹持力＝0x08 power 上限；③兜底＝旧 grip_detect 位置分类（|停住−目标|≤1° empty / ≥+2°
grasped / 其余 uncertain）+ not_moved + 超时。旧阈值来自百岁山最终方案
（GRASP-GRIP-SETTING-20261027）；TC-008 两轮停住值 32.5/31.6 按旧阈值均判 grasped；
33–34 窄窗作废（F57）。TC-009 v2 为三层提供数据并验证旧阈值。

【流程】
  第 0 段·开度标定：J6 空载依次 64.5/50/40/29/20/10/0/−30（≤20°/s，每点 S3 校验到位，
    不到位即释放中止），停稳后提示量**金属指间距**（mm）输入，记 calib 表（读回角度+mm）。
  第 1 段·示教夹取姿态：teach_pose（grip_check=False，等候期间 J6 守卫）。
  第 2 段·试次：空夹×10、夹瓶×10（放法 1中/2贴左垫/3贴右垫/4靠指根/5靠指尖，定义见用例 §3）、P=6000/3000 各
    空夹×3+夹瓶×3、拔瓶×3。每试记录 start（关闭前 0x16 角度与电流），关闭（20°/s 固定
    1775 ms）后 0x16 高频连读 4 s（拔瓶 6 s）。
  第 3 段·收尾：J6 张开验证到位 → 锁 7 轮 → 打印恢复服务命令。

【安全 S1–S7（v2 任务单）】S1 圈数门（read_raw_multiturn 原始多圈；J0–J5 超限拒动；J6 超限
→ clear_turns 内部先 0x18 释放再 0x11，读回 ±180 内才继续；保护位/F7 检查）；S2 J6 首动作
方向试探（朝目标 5°/≤10°/s，位移 ≥3° 且方向正确，否则释放中止）；S3 每条 J6 0x08 在
T+0.5 s 内读回（空载 |读数−目标|≤2°；夹物=停稳且未超时，停稳判定窗 [close_ms+0.2, close_ms+0.5]，
窗起点避开关闭收尾微动——T4b4，试次 11 复算 31.6–31.7 已停稳而旧窗含 32.3→32.0 收尾被判未过；
not_moved=读数未动→释放中止）；
S4 非夹持保持期间（张开/标定点/示教与放瓶等候、空夹试次）电流 >400 mA 持续 2 s →释放
（400 为临时值）；夹瓶/拔笔试次与 tc008 夹持保持同理不套用，改温度 ≥55 ℃ → 正常中止；
S5 中止：J6 不确定→释放；已验证张开→可保持；已验证夹持（tc008）→保持 0x08→29 并打印温度；
除此之外中止路径不给 J6 发 0x08；S6 退出打印 J6 终态（「已验证静止、未堵转」或「已释放」）；
S7 每试前温度门 ≥55 ℃ 暂停。

【--analyze（只出数，不定阈值）】a) t_contact_I=电流相对关闭前基线首次持续升高时刻
（定义随表打印，附候选门限 Δ=100/200/300 mA 对照）；b) 与位置口径 t_stop 差值；c) 空夹试次
合拢途中电流峰值分布；d) 各 P 档稳态电流；e) 旧阈值逐试分类（标注「旧阈值套用」）；
f) calib 表与拟合直线、残差。
"""

import argparse
import csv
import json
import os
import sys
import time
import types

try:
    import serial  # noqa: F401
except ImportError:
    _stub = types.ModuleType('serial')
    _stub.EIGHTBITS, _stub.PARITY_NONE, _stub.STOPBITS_ONE, _stub.Serial = 8, 'N', 1, None
    sys.modules['serial'] = _stub

from fsbus import (Bus, preflight, STOP_HOLD, norm_single_turn,  # noqa: E402
                   read_mturn, mturn_gate, mturn_assert, MturnRefused)
from tc007_stream import read_ang, read_all, DEFAULT_HOLD_POWER  # noqa: E402
from tc008_sync_load import (teach_pose, guard_monitor, median, fmt, pad,  # noqa: E402
                             j6_stall_track, input_j6guard, j6_open_verified,
                             pose_f7_check, j6_abort_action,
                             GRIP_OPEN, GRIP_CLOSE, GRIP_SPEED, MOVE_IDS, ALL_IDS, GRIP_ID,
                             J6_OPEN_TOL, J6_STALL_MA, J6_STALL_S, T_FLOOR_MS,
                             OLD_EMPTY_ABS, OLD_GRASP_MIN)   # T6：与 tc008 夹瓶判定同源同值

CLOSE_MS = int(abs(GRIP_OPEN - GRIP_CLOSE) / GRIP_SPEED * 1000.0)   # 1775 ms = 20°/s
DUR_S = 4.0
DUR_PULL_S = 6.0
PULL_PROMPT_S = 2.5
TEMP_PAUSE_C = 55.0
LAST_WIN_S = 1.0
SPEED_WIN_S = 0.1
SPEED_STOP = 5.0
CALIB_POINTS = (64.5, 50.0, 40.0, 29.0, 20.0, 10.0, 0.0, -30.0)
# 旧 grip_detect 阈值 = OLD_EMPTY_ABS/OLD_GRASP_MIN，自 tc008_sync_load import（T6：同源同值；
# 百岁山最终方案 GRASP-GRIP-SETTING-20261027；「旧阈值套用」输出用，不是新判定）
CONTACT_DELTAS = (100, 200, 300)   # a) 候选电流门限 mA
CONTACT_HOLD_S = 0.2     # a) 「持续升高」判据：其后 0.2 s 内全部 ≥ 门限
# 放法标签（Thomas 同意；参考视角与逐条定义见用例 TC-009 §3 表）
LABELS = {'1': '中', '2': '贴左垫', '3': '贴右垫', '4': '靠指根', '5': '靠指尖'}
CSV_HEADER = ['trial', 'kind', 'label', 'P', 't_s', 'ang', 'mA', 'mW', 'mV', 'st', 'tC', 'rtt_ms']


class Run(object):
    """运行状态：样本缓存 + CSV 即时落盘 + J6 状态（字段名与 tc008 守卫函数共用）"""

    def __init__(self, csv_path, vbat_low, j6_stall_ma=J6_STALL_MA):
        self.rows = []
        self.vbat_low = vbat_low
        self.grip_active = False
        self.grip_verified = False
        self.j6_verified_open = False
        self.j6_probed = False
        self.j6_release_now = False
        self.j6_stall_since = None
        self.j6_stall_ma = j6_stall_ma
        self.j6_state = '未知'
        self.fh = open(csv_path, 'w', newline='') if csv_path else None
        self.w = csv.writer(self.fh) if self.fh else None
        if self.w:
            self.w.writerow(CSV_HEADER)

    @property
    def j6_closed(self):
        return self.grip_active

    @j6_closed.setter
    def j6_closed(self, v):
        self.grip_active = v

    def add(self, row):
        self.rows.append(row)
        if self.w is not None:
            self.w.writerow([row['trial'], row['kind'], row['label'], row['P'], '%.4f' % row['t_s'],
                             '%.2f' % row['ang'], row['mA'], row['mW'], row['mV'], row['st'],
                             '' if row['tC'] is None else '%.1f' % row['tC'], '%.1f' % row['rtt_ms']])

    def flush(self):
        if self.fh:
            self.fh.flush()

    def close(self):
        if self.fh:
            self.fh.close()
            self.fh = None


# ---------- 第 0 段·开度标定 ----------

def ask_mm(bus, run, prompt):
    """标定 mm 输入（非数字重问）。等待走 input_j6guard：量尺期间每 0.5 s 读 J6 做非夹持
    堵转检查（S4）——尤其 −30 点两垫接近贴合，卡住 >400 mA 持续 2 s 即释放中止。"""
    while True:
        a = input_j6guard(bus, run, prompt).strip()
        try:
            return float(a)
        except ValueError:
            print('   请输入数字（毫米）')


def run_calib(bus, run, args):
    """开度标定：每点 0x08（≤20°/s）→ S3 读回校验（≤2°）→ 量金属指间距输入。
    calib 记录 {pt, read_ang, mm}；任一点不到位 → 释放并中止。"""
    calib = []
    for i, pt in enumerate(CALIB_POINTS, 1):
        m = read_mturn(bus, GRIP_ID)
        cur = m['angle'] if m else None
        if cur is None:
            raise RuntimeError('j6_read_fail@calib%d' % i)
        if not (pt == GRIP_OPEN and abs(cur - pt) <= J6_OPEN_TOL):
            mturn_assert(bus, ALL_IDS, 'calib%d' % i)          # k：运动前复查
            ms = int(max(T_FLOOR_MS, abs(pt - cur) / GRIP_SPEED * 1000.0))
            bus.set_angle(GRIP_ID, pt, ms, args.grip_power)
            time.sleep(ms / 1000.0 + 0.5)
            m2 = read_mturn(bus, GRIP_ID)
            ok = m2 is not None and abs(m2['angle'] - pt) <= J6_OPEN_TOL
            print('   [标定 %d/%d] 目标 %+.1f° → 读回 %s（判过 ≤%.1f°）→ %s'
                  % (i, len(CALIB_POINTS), pt, fmt(m2['angle'] if m2 else None, '%.1f', '读不到'),
                     J6_OPEN_TOL, '通过' if ok else '未过'))
            if not ok:
                bus.stop(GRIP_ID, 0x10, 0)
                run.j6_state = '已释放(0x18 0x10，标定点未到位)'
                raise RuntimeError('j6_calib_not_verified@%d(target %.1f)' % (i, pt))
            # M1（T4b3）：到位后立即 0x18 保持——按当前位置保持、无远处目标，量距期间无持续
            # 驱动力（200–400 mA 半堵转也消除）；下一点的 0x08 从保持恢复（F48 顺序性）
            bus.stop(GRIP_ID, STOP_HOLD, 0)
            run.grip_active = False      # 0x18 保持态：无 0x08 目标在身
            cmd_ang = m2['angle']
        else:
            bus.stop(GRIP_ID, STOP_HOLD, 0)   # M1：首点（已在张开位）同样转保持
            cmd_ang = cur
        run.j6_verified_open = False   # 已离开验证张开态：此后中止处置按「不确定→释放」
        mm = ask_mm(bus, run, '   [标定 %d/%d @ %+.1f°] 请用尺子量两根金属指之间的距离（不算垫子），'
                    '输入毫米数后回车：' % (i, len(CALIB_POINTS), pt))
        m3 = read_mturn(bus, GRIP_ID)             # 量距时刻重读（保持态下读数）
        read_ang = m3['angle'] if m3 else cmd_ang
        rec = {'pt': pt, 'read_ang': read_ang, 'read_ang_cmd': cmd_ang, 'mm': mm}
        calib.append(rec)
        bus.log('calib_point', **rec)
        print('   [标定 %d/%d] 记录：量距时读回 %.1f°（到位校验 %.1f°）↔ %.1f mm'
              % (i, len(CALIB_POINTS), read_ang, cmd_ang, mm))
    return calib


# ---------- 第 2 段·试次 ----------

def temp_gate(bus, run, trial_no):
    """S7：试次开始前 J6 温度 ≥55℃ → 暂停等冷却（回车重测 / q 中止）"""
    while True:
        m = read_mturn(bus, GRIP_ID)
        if m is None or m['tC'] is None or m['tC'] < TEMP_PAUSE_C:
            return (m or {}).get('tC')
        print('   !! [温度] 试次 %d 前 J6 温度 %.1f℃ ≥ %.0f℃，暂停等冷却' % (trial_no, m['tC'], TEMP_PAUSE_C))
        bus.log('temp_pause', trial=trial_no, temp_c=m['tC'])
        a = input('   [温度] 冷却后按回车重测（q=中止）：').strip().lower()
        if a == 'q':
            raise RuntimeError('temp_pause_abort(J6 %.1f℃)' % m['tC'])


def run_trial(bus, run, trials, no, kind, label, power, args):
    """一个试次（S3/S4 全套）：提示（守卫 input）→ 记 start（角度+电流）→ 关闭 → close 验证 →
    高频采样（守卫：空夹=400mA/2s；夹瓶/拔瓶=温度≥55℃）→ 张开验证。"""
    dur = DUR_PULL_S if kind == 'pull' else DUR_S

    def ask(pr):
        return input_j6guard(bus, run, pr)

    if kind == 'empty':
        ask('   [试次 %d · 空夹 P=%d] 两指间不放任何东西，手离开两指之间，按回车关闭：' % (no, power))
    elif kind == 'bottle':
        while True:
            a = ask('   [试次 %d · 夹瓶 P=%d] 放入瓶子（h=108），手离开两指之间，输入放法标签后回车'
                    '（1中/2贴左垫/3贴右垫/4靠指根/5靠指尖，定义见用例 §3）：' % (no, power)).strip()
            if a in LABELS:
                label = LABELS[a]
                break
            print('   请输入 1–5')
    else:
        ask('   [试次 %d · 拔瓶 P=%d] 放入瓶子（h=108），手离开两指之间，按回车关闭；关闭 %.1fs 后会提示抽瓶：'
            % (no, power, PULL_PROMPT_S))
    mturn_assert(bus, ALL_IDS, 'trial%d' % no)          # k：运动前原始多圈复查
    m0 = read_mturn(bus, GRIP_ID)
    start_ang = m0['angle'] if m0 else None
    start_mA = m0['mA'] if m0 else None                 # a) t_contact_I 的关闭前基线
    t_cmd = time.monotonic()
    bus.set_angle(GRIP_ID, GRIP_CLOSE, CLOSE_MS, power)
    run.j6_closed = True
    rec = {'trial': no, 'kind': kind, 'label': label, 'P': power,
           't_cmd': round(t_cmd - bus.t0, 4), 'close_ms': CLOSE_MS, 'dur_s': dur,
           'start_ang': start_ang, 'start_mA': start_mA, 'n': 0}
    trials.append(rec)
    bus.log('trial_close', **rec)
    prompted = False
    t_end = t_cmd + dur
    while time.monotonic() < t_end:
        if kind == 'pull' and not prompted and time.monotonic() >= t_cmd + PULL_PROMPT_S:
            print('   >>> 现在只抓瓶尾，慢慢把瓶子往外抽（手不进指间）……')
            bus.log('pull_prompt', trial=no, t=round(time.monotonic() - bus.t0, 4))
            prompted = True
        f = bus.read_monitor(GRIP_ID, timeout=0.05)
        if f is None or f['verdict'] != 'ok':
            continue
        why = guard_monitor(f, run.vbat_low)
        if why:
            raise RuntimeError(why)
        if not f['valid_measure']:
            continue
        if kind == 'empty':                              # S4：空夹套 400mA/2s（堵转=卡指）
            j6_stall_track(run, f['current_mA'], 'trial%d' % no)
        elif f['temp_c'] is not None and f['temp_c'] >= TEMP_PAUSE_C:
            raise RuntimeError('j6_overtemp@trial%d(%.1f℃≥55，走正常中止)' % (no, f['temp_c']))
        run.add({'trial': no, 'kind': kind, 'label': label, 'P': power,
                 't_s': round(f['t_rx'] - t_cmd, 4), 'ang': round(norm_single_turn(f['angle']), 2),
                 'mA': f['current_mA'], 'mW': f['power_mW'], 'mV': f['voltage_mV'], 'st': f['status'],
                 'tC': f['temp_c'], 'rtt_ms': round(f['rtt'] * 1000.0, 1)})
        rec['n'] += 1
        # S3：关闭验证（判定期 close_ms+0.5 s，判据窗 [close_ms+0.2, close_ms+0.5]——T4b4：窗起点
        # +0.2 s 避开关闭收尾微动，试次 11 旧窗含 32.3→32.0 收尾极差 0.7 误判未过）：空夹=到位±2°；
        # 夹物=停稳；not_moved=未动
        t_rel = f['t_rx'] - t_cmd
        if not rec.get('close_checked') and t_rel >= CLOSE_MS / 1000.0 + 0.5:
            win0 = [r for r in run.rows if r['trial'] == no
                    and CLOSE_MS / 1000.0 + 0.2 <= r['t_s'] <= t_rel]
            angs = [r['ang'] for r in win0]
            if kind == 'empty':
                ok_c = bool(angs) and abs(median(angs) - GRIP_CLOSE) <= J6_OPEN_TOL
                crit = '|读数−29.0|≤2.0°'
            else:
                ok_c = bool(angs) and (max(angs) - min(angs)) <= 0.5
                crit = '0.3s 窗极差≤0.5°（停稳）'
            if start_ang is not None and angs and abs(median(angs) - norm_single_turn(start_ang)) < 0.5:
                ok_c = False                             # not_moved
                crit += '；not_moved'
            rec['close_checked'] = True
            rec['close_verified'] = bool(ok_c)
            rec['close_median'] = round(median(angs), 2) if angs else None
            print('   [关闭验证@试次 %d] %s（%.2f°，判过 %s）'
                  % (no, '通过' if ok_c else '未过→释放中止',
                     median(angs) if angs else float('nan'), crit))
            if not ok_c:
                raise RuntimeError('j6_close_not_verified@trial%d(kind=%s med=%s start=%s)'
                                   % (no, kind, rec['close_median'], start_ang))
    rec['t_end'] = round(time.monotonic() - bus.t0, 4)
    run.flush()
    a_open = j6_open_verified(bus, run, args, 'trial%d' % no)   # S3：每试张开同样验证
    rec['open_read'] = a_open
    st = analyze_trial(rec, run.rows)
    print('   [试次 %d · %s P=%d%s] n=%d 停住角=%s 稳态电流=%s mA 峰值=%s mA@%ss 接触/到位(位置)=%ss 张开后=%s'
          % (no, kind, power, ('·' + label) if label else '', rec['n'], fmt(st['ang_hold'], '%.1f'),
             fmt(st['mA_hold'], '%d'), fmt(st['mA_peak'], '%d'), fmt(st['t_peak'], '%.2f'),
             fmt(st['t_stop'], '%.2f', '未检出'), fmt(a_open, '%.1f')))
    return rec


# ---------- 统计（--analyze 可从 CSV+JSON 复算，R6；只出数不定阈值） ----------

def analyze_trial(rec, rows):
    pts = sorted((r for r in rows if r['trial'] == rec['trial']), key=lambda r: r['t_s'])
    dur = rec['dur_s']
    out = {'trial': rec['trial'], 'kind': rec['kind'], 'label': rec['label'], 'P': rec['P'], 'n': len(pts)}
    last = [r for r in pts if r['t_s'] >= dur - LAST_WIN_S]
    out['n_last'] = len(last)
    out['ang_hold'] = median([r['ang'] for r in last]) if last else None
    out['mA_hold'] = median([r['mA'] for r in last]) if last else None
    out['mA_hold_range'] = (max(r['mA'] for r in last) - min(r['mA'] for r in last)) if last else None
    out['mW_hold'] = median([r['mW'] for r in last]) if last else None
    if pts:
        pk = max(pts, key=lambda r: r['mA'])
        out['mA_peak'], out['t_peak'] = pk['mA'], pk['t_s']
        out['tC_start'], out['tC_end'] = pts[0]['tC'], pts[-1]['tC']
    else:
        out['mA_peak'] = out['t_peak'] = out['tC_start'] = out['tC_end'] = None
    t_stop, onset = None, None
    j = 0
    for i in range(len(pts)):
        while j < len(pts) and pts[j]['t_s'] - pts[i]['t_s'] < SPEED_WIN_S:
            j += 1
        if j >= len(pts):
            break
        v = abs(pts[j]['ang'] - pts[i]['ang']) / (pts[j]['t_s'] - pts[i]['t_s'])
        if onset is None:
            if v > SPEED_STOP:
                onset = pts[i]['t_s']
        elif v <= SPEED_STOP:
            t_stop = pts[i]['t_s']
            break
    out['t_onset'], out['t_stop'] = onset, t_stop
    # e) 旧阈值套用（|停住−目标|≤1 empty；停住−目标≥+2 grasped；其间 uncertain）
    if out['ang_hold'] is not None:
        err = out['ang_hold'] - GRIP_CLOSE
        out['old_err'] = round(err, 2)
        out['old_class'] = ('empty' if abs(err) <= OLD_EMPTY_ABS
                            else ('grasped' if err >= OLD_GRASP_MIN else 'uncertain'))
    else:
        out['old_err'] = out['old_class'] = None
    # a) t_contact_I：mA ≥ 基线+Δ 且其后 CONTACT_HOLD_S 内全部 ≥ 基线+Δ
    base = rec.get('start_mA')
    out['t_contact_I'] = {}
    if base is not None:
        for delta in CONTACT_DELTAS:
            thr = base + delta
            tc = None
            for i in range(len(pts)):
                if pts[i]['mA'] < thr:
                    continue
                if all(p['mA'] >= thr for p in pts[i:]
                       if p['t_s'] - pts[i]['t_s'] <= CONTACT_HOLD_S):
                    tc = pts[i]['t_s']
                    break
            out['t_contact_I'][delta] = tc
    return out


def calib_fit(calib):
    """f) 最小二乘 mm = a×ang + b（全点拟合；口径注明全开段 >50° 线性最差，残差随表）"""
    if not calib:
        return None
    xs = [c['read_ang'] for c in calib]
    ys = [c['mm'] for c in calib]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    a = (sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx) if sxx else 0.0
    b = my - a * mx
    res = [round(y - (a * x + b), 2) for x, y in zip(xs, ys)]
    return {'a': round(a, 4), 'b': round(b, 2), 'residuals_mm': res}


def group_key(st):
    return (st['kind'], st['P'])


def report(doc, rows, trials):
    print('')
    print('===== TC-009 v2 数据表（只出数不分类不定阈值）=====')
    print('定义：停住角/稳态电流(中位,极差)/稳态功率=该试最后 %.0f s 内样本；峰值电流=全试 max 及时刻；'
          '位置口径接触/到位时刻=运动起始(首个 %.0f ms 前向窗平均速度>%.0f°/s)之后首个窗平均速度≤%.0f°/s '
          '的窗起点（分辨率≈%.1f s）；电流口径 t_contact_I(Δ)=首个 mA≥关闭前基线+Δ 且其后 %.1f s 内'
          '全部≥基线+Δ 的样本时刻（基线=关闭前一条 0x16 电流，候选 Δ=%s mA）；'
          '旧阈值分类=|停住−29.0|≤%.0f° empty / 停住−29.0≥+%.0f° grasped / 其间 uncertain'
          '（旧阈值套用，非新判定）；t 相对关闭下发；关闭 %.1f→%.1f° 固定 %d ms'
          % (LAST_WIN_S, SPEED_WIN_S * 1000, SPEED_STOP, SPEED_STOP, SPEED_WIN_S, CONTACT_HOLD_S,
             list(CONTACT_DELTAS), OLD_EMPTY_ABS, OLD_GRASP_MIN, GRIP_OPEN, GRIP_CLOSE, CLOSE_MS))
    calib = doc.get('calib') or []
    if calib:
        fit = calib_fit(calib)
        print('[f) 开度标定] 读回角度 ↔ 金属指间距（mm）：')
        for c in calib:
            print('   %+7.1f° ↔ %6.1f mm' % (c['read_ang'], c['mm']))
        if fit:
            print('   拟合 mm = %.4f×ang + %.2f；残差(mm) %s（口径：>50° 全开段线性最差，以实测为准）'
                  % (fit['a'], fit['b'], fit['residuals_mm']))
    stats = [analyze_trial(t, rows) for t in trials]
    if stats:
        print('[逐试]（旧类=旧阈值套用，非新判定）')
        print('  试次 类型     放法   P      n   停住角  旧误差→旧类   稳态mA(极差)  峰值mA@t   '
              't_stop   t_contact_I(Δ100/200/300)   温度首→末')
        for st in stats:
            tci = '/'.join(fmt(st['t_contact_I'].get(d), '%.2f', '—') for d in CONTACT_DELTAS)
            print('  %3d  %-7s %s %-6d %4d  %7s  %+5s→%-9s %6s(%s)  %6s@%-5s  %-7s  %-24s  %s→%s'
                  % (st['trial'], st['kind'], pad(st['label'] or '—', 6), st['P'], st['n'],
                     fmt(st['ang_hold'], '%.1f'), fmt(st['old_err'], '%+.1f'), fmt(st['old_class'], '%s'),
                     fmt(st['mA_hold'], '%d'), fmt(st['mA_hold_range'], '%d'),
                     fmt(st['mA_peak'], '%d'), fmt(st['t_peak'], '%.2f'),
                     fmt(st['t_stop'], '%.2f', '未检出'), tci,
                     fmt(st['tC_start'], '%.1f'), fmt(st['tC_end'], '%.1f')))
        print('[b) t_stop − t_contact_I(s)，正=电流口径更早]')
        for st in stats:
            for d in CONTACT_DELTAS:
                tc = st['t_contact_I'].get(d)
                if tc is not None and st['t_stop'] is not None:
                    print('   试次 %2d Δ%3d：t_stop−t_contact_I = %+.2f s'
                          % (st['trial'], d, st['t_stop'] - tc))
        empties = [t for t in trials if t['kind'] == 'empty']
        peaks = []
        for t in empties:
            pk = [r['mA'] for r in rows if r['trial'] == t['trial'] and 0 <= r['t_s'] <= t['close_ms'] / 1000.0]
            if pk:
                peaks.append(max(pk))
        if peaks:
            print('[c) 空夹合拢途中电流峰值 mA（t∈[0,%dms]）] n=%d min/中位/max = %d / %d / %d'
                  % (CLOSE_MS, len(peaks), min(peaks), int(median(peaks)), max(peaks)))
        print('[d) 分组 kind×P] n=试次数 | 停住角 | 稳态mA | 峰值mA | 稳态mW | 旧类分布')
        keys = sorted(set(group_key(s) for s in stats), key=lambda k: (k[0], k[1]))
        for k in keys:
            grp = [s for s in stats if group_key(s) == k]

            def mmm(field, f='%.1f'):
                vals = [s[field] for s in grp if s[field] is not None]
                return '—' if not vals else '%s / %s / %s' % (f % min(vals), f % median(vals), f % max(vals))
            cls = {}
            for s in grp:
                cls[s['old_class']] = cls.get(s['old_class'], 0) + 1
            print('  %s P=%-5d n=%d | 停住角 %s | 稳态mA %s | 峰值mA %s | 稳态mW %s | 旧类 %s'
                  % (pad(k[0], 7), k[1], len(grp), mmm('ang_hold'), mmm('mA_hold', '%d'),
                     mmm('mA_peak', '%d'), mmm('mW_hold', '%d'), cls))
    print('样本范围：样本总数 %d；试次 %d；标定点 %d；剔除=无（0x16 哨兵/坏帧不入样本，计入 jsonl）'
          % (len(rows), len(trials), len(calib)))
    return stats


def load_csv(path):
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            rows.append({'trial': int(r['trial']), 'kind': r['kind'], 'label': r['label'], 'P': int(r['P']),
                         't_s': float(r['t_s']), 'ang': float(r['ang']), 'mA': int(r['mA']), 'mW': int(r['mW']),
                         'mV': int(r['mV']), 'st': int(r['st']),
                         'tC': (float(r['tC']) if r['tC'] != '' else None), 'rtt_ms': float(r['rtt_ms'])})
    return rows


def analyze_files(csv_path, json_path):
    with open(json_path, encoding='utf-8') as fh:
        doc = json.load(fh)
    rows = load_csv(csv_path)
    print('--analyze：%s（%d 样本）+ %s（%d 试次 / %d 标定点）'
          % (csv_path, len(rows), json_path, len(doc.get('trials', [])), len(doc.get('calib', []))))
    report(doc, rows, doc.get('trials', []))


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(description='TC-009 v2 夹爪夹持特性采集（只出数）')
    ap.add_argument('--port', default='/dev/ttyS4')
    ap.add_argument('--stop-power', type=int, default=DEFAULT_HOLD_POWER, help='0x18 保持锁力 power_mW（默认 0）')
    ap.add_argument('--grip-power', type=int, default=0, help='J6 0x08 power_mW（0=按 J6 保护阈值）')
    ap.add_argument('--vbat-low', type=int, default=11000)
    ap.add_argument('--n-empty', type=int, default=10)
    ap.add_argument('--n-bottle', type=int, default=10)
    ap.add_argument('--powers', default='6000,3000')
    ap.add_argument('--n-power', type=int, default=3)
    ap.add_argument('--n-pull', type=int, default=3)
    ap.add_argument('--j6-stall-ma', type=int, default=J6_STALL_MA,
                    help='S4：非夹持期间 J6 堵转阈 mA（默认 %d【临时值待审定】），持续 %.0f s 释放中止'
                         % (J6_STALL_MA, J6_STALL_S))
    ap.add_argument('--analyze', nargs=2, metavar=('CSV', 'JSON'))
    args = ap.parse_args()
    if args.analyze:
        analyze_files(args.analyze[0], args.analyze[1])
        return
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    pw = args.stop_power
    powers = [int(x) for x in args.powers.split(',') if x.strip()]
    ts = time.strftime('%Y%m%d_%H%M%S')
    here = os.path.dirname(os.path.abspath(__file__))
    log = os.path.join(here, 'tc009_log_%s.jsonl' % ts)
    csvf = os.path.join(here, 'tc009_samples_%s.csv' % ts)
    jsonf = os.path.join(here, 'tc009_trials_%s.json' % ts)
    plan = ([('empty', 0)] * args.n_empty + [('bottle', 0)] * args.n_bottle
            + [(k, p) for p in powers for k in ['empty'] * args.n_power + ['bottle'] * args.n_power]
            + [('pull', 0)] * args.n_pull)
    print('TC-009 v2 夹爪夹持特性采集（板载时钟 %s，仅参考）' % ts)
    print('!! 第0段 开度标定 %d 点（64.5→−30，每点到位校验+量金属指间距）→ 第1段 示教 → 第2段 %d 试：'
          '空夹×%d → 夹瓶×%d → P∈%s 各空×%d+瓶×%d → 拔瓶×%d → 第3段 收尾'
          % (len(CALIB_POINTS), len(plan), args.n_empty, args.n_bottle, powers,
             args.n_power, args.n_power, args.n_pull))
    print('!! S1 圈数门（J6 超限→释放→0x11 清→确认；J0–J5 超限→拒动）；S2 首动作方向试探（位移≥3°）；'
          'S3 每条 0x08 读回验证；S4 非夹持期间 >%dmA/2s 或夹持期间温度≥55℃ → 释放/中止；'
          'S7 每试前温度门 ≥%.0f℃' % (args.j6_stall_ma, TEMP_PAUSE_C))
    print('!! 前置检查：4 服务 inactive + %s 无占用' % args.port)
    ok_pf, msgs = preflight(args.port)
    for m in msgs:
        print('   preflight: %s' % m)
    if not ok_pf:
        print('   !! preflight 未过，退出')
        return
    input('   [上电姿态] 机械臂是否处于可上电姿态（未压在折叠位/机械限位上）？确认后回车'
          '（T4b4：仅提示，不自动判断——折叠位 J2 会超 F7，且中止路径 0x18 保持会带抖动）：')
    print('日志: %s\n样本CSV: %s（即时落盘）\n试次JSON: %s' % (log, csvf, jsonf))
    bus = Bus(args.port, log_path=log)
    run = Run(csvf, args.vbat_low, args.j6_stall_ma)
    doc = {'ts': ts, 'ver': 'v2', 'stop_power': pw, 'vbat_low': args.vbat_low,
           'grip_power': args.grip_power, 'close_ms': CLOSE_MS, 'dur_s': DUR_S,
           'dur_pull_s': DUR_PULL_S, 'temp_pause_c': TEMP_PAUSE_C,
           'j6_stall_ma': args.j6_stall_ma, 'plan': plan, 'calib': [], 'trials': []}
    trials = doc['trials']

    def save_doc():
        with open(jsonf, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, ensure_ascii=False, indent=1)

    try:
        bus.drain()
        # S1 圈数门：J6 超限 → clear_turns（内部释放→0x11）→ 确认；J0–J5 超限/保护位 → 拒动
        ok_mt, rep_mt = mturn_gate(bus, ALL_IDS, clear_ids=(GRIP_ID,), stop_power=pw)
        print('   [圈数门] %s：%s' % ('通过' if ok_mt else '未过（拒绝运动，不发任何运动帧）',
              {('J%d' % k2): (('%.1f°/t%d st0x%02x' % (v['angle'], v['turns'], v['st'])) if v else '读不到')
               for k2, v in rep_mt.items()}))
        if not ok_mt:
            raise MturnRefused('mturn_gate_fail')
        pose_pre = read_all(bus)                       # S1：全轴 F7 只读检查
        if pose_pre is None:
            raise RuntimeError('preflight_read_fail')
        pose_f7_check(pose_pre, 'preflight')
        print('   [前置] 7 轴 0x0A 读数均在 F7 限位内：%s' % json.dumps(pose_pre, ensure_ascii=False))
        # 第 0 段：先张开（S2 试探+S3 验证）再标定
        print('   [准备] J6 张开到 %.1f（S2 方向试探 + S3 读回验证）' % GRIP_OPEN)
        j6_open_verified(bus, run, args, '准备')
        doc['calib'] = run_calib(bus, run, args)
        save_doc()
        # 第 1 段：示教夹取姿态（H1 → 释放 J0–J5 → 摆位 → 锁 → 读+F7 → S1 下垂；等候期间 J6 守卫）
        j6_open_verified(bus, run, args, '标定后')
        pose, sag = teach_pose(bus, run, '夹取', pw, release_first=True, grip_check=False,
                               wait=lambda pr: input_j6guard(bus, run, pr))
        doc['pose'], doc['sag'] = pose, sag
        save_doc()
        # 第 2 段：试次
        for i, (kind, power) in enumerate(plan, 1):
            temp_gate(bus, run, i)
            bus.log('trial_begin', trial=i, kind=kind, P=power)
            run_trial(bus, run, trials, i, kind, '', power, args)
            save_doc()
        # 第 3 段：收尾（末试已验证张开）
        print('   [收尾] 锁 7 轮')
        sent = bus.lock_all(ALL_IDS, pw, mode=STOP_HOLD)
        final = read_all(bus)
        print('   [收尾] lock_all 发出 %d/7；最终姿态：%s' % (sent, json.dumps(final, ensure_ascii=False)))
        doc['pose_final'] = final
        doc['stats'] = report(doc, run.rows, trials)
        save_doc()
        print('文件：%s\n      %s\n      %s' % (csvf, jsonf, log))
        print('   [退出状态] J6：%s（S6）；J0–J5：lock_all 7 轮保持锁力' % run.j6_state)
        print('恢复服务（Thomas 臂上执行，TC-004 步骤 6）：sudo systemctl start marm_controller marm_communication marm_control marm_subscripber')
    except (KeyboardInterrupt, Exception) as e:
        if isinstance(e, MturnRefused) and not getattr(e, 'runtime', False):
            # M2①：圈数门阶段拒动 → 不发任何帧直接退出（舵机上电本为释放态）
            print('\n!! 多圈超限，拒绝运动（%s）：圈数门阶段，不发送任何帧，直接退出——' % e)
            print('   脚本开机会自动清 J6 圈数；J0–J5 超限请重启臂（开机服务会清除）后再跑')
        else:
            # S5：J0–J5 恒 safe_stop_all；J6 不发 0x08——不确定→0x18 释放；已验证张开→保持
            act = j6_abort_action(run)
            r = bus.safe_stop_all(MOVE_IDS, pw, mode=STOP_HOLD)
            bad = [sid for sid, st in r['still'].items() if st is not True]
            print('\n!! 异常/中断（%s）：J0–J5 safe_stop_all 收尾（J6 处置=%s，不给 J6 发 0x08）——'
                  % (repr(e), act))
            print('   停止帧已尝试发送 %d；各轴停住判定：%s'
                  % (r['sent'], {('J%d' % sid): v for sid, v in r['still'].items()}))
            if bad:
                print('   !! 未能确认已锁住的轴：%s——请 Thomas 扶住臂目视确认，必要时急停（断电，F27）'
                      % ['J%d' % sid for sid in bad])
            if act == 'release':
                try:
                    bus.stop(GRIP_ID, 0x10, 0)
                    time.sleep(0.2)
                    a6 = read_ang(bus, GRIP_ID)
                    run.j6_state = '已释放(0x18 0x10)'
                    print('   J6：已 0x18 释放（0x10，power=0，手勿入指间）；读数 %s' % fmt(a6, '%.1f', '读不到'))
                    bus.log('abort_j6_release', j6=a6)
                except Exception as e3:
                    print('   !! J6 释放失败（%s）——请 Thomas 目视确认夹爪，手勿入指间，必要时断电' % e3)
            elif act == 'keep_open':
                print('   J6：已验证张开 → 保持，不发帧')
            else:
                print('   J6：保持（本用例无已验证夹持场景）')
        doc['abort'] = repr(e)
        doc['abort_j6'] = ('mturn_refused' if isinstance(e, MturnRefused) else j6_abort_action(run))
        try:
            run.flush()
            if trials:
                doc['stats'] = report(doc, run.rows, trials)
            save_doc()
            print('   已落盘：%s（%d 样本）/ %s' % (csvf, len(run.rows), jsonf))
        except Exception as e2:
            print('   (落盘失败: %s)' % e2)
        # S6：退出前打印 J6 终态（只允许 已验证静止未堵转 / 已释放）
        print('   [退出状态] J6：%s；J0–J5：%s' % (
            run.j6_state,
            '未发帧（圈数门拒绝路径）' if isinstance(e, MturnRefused) and not e.runtime
            else 'safe_stop_all 保持锁力'))
    finally:
        run.close()
        bus.close()


if __name__ == '__main__':
    main()
