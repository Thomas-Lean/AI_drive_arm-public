#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tc007_stream.py —— TC-007 0x08 流式平滑性实测（P0.4-T12b 合并式，关闭 §11 Q3 · 关卡）

v6（2026-10-30，T12b 按 审查方 关键代码复审 H1–H4/S1/S2 修订）：
  - **H2 合并式**：示教与测试合并为每轴一个流程（原两段式摆位环节在全锁状态下
    做不到）——摆姿态→扶住回车→全锁→读姿态→（S1 下垂检查）→扶住确认后只释放
    被测轴→示教上下限→锁回→（H3 慢速归位+验收）→参考段→流式→口述→（S2 非被测
    轴核查）→下一轴。示教 JSON 边跑边存（每轴完成即落盘，中断不丢已完成轴）。
  - **H1**：每一次释放（全轴/单轴）之前都有 input「已扶住后按回车解锁」，
    等待期间不发送任何帧（首轴开跑前全失力、无释放，不问）。
  - **H3**：每轴测试前慢速（≤20°/s）0x08 归位到测试下限，读角验收 ≤1°，
    不过即中止（safe_stop_all）。
  - **H4**：0x18 保持锁力功率默认 **0**（F38 厂家：power=0 按功率保护阈值执行
    且有锁力）；F49「0 与 6000 等效」只在空载 J6 验证过，J1/J2 扛整臂静态保持
    不能沿用 6000（12V 下仅约 0.5A）。--stop-power 保留用于覆盖。
  - **S1**：全锁后下垂检查——提示 Thomas 慢慢松手，等 1s 重读 7 轴，任一轴
    偏差 >0.5° 报警，由 Thomas 决定继续/中止（尤其 J1/J2）。
  - **S2**：每轴测试结束锁回后重读 7 轴，非被测轴偏差 ≤0.5° 核查写入 SUMMARY。

v5 口径（Thomas 2026-10-30 决定）不变：7 轴逐轴（J6→J5→J4→J3→J0→J2→J1）全速
（默认 114.59°/s=源码 maxomega，三来源见表 A，Thomas 可覆盖）、不录像
（主依据=速度曲线数据，审查方 复核裁定；辅助=每轴口述 1/2/3）、采样按 F45/F47
（0x16 窗 15ms、0x0A 补；P100≈70Hz 主判据、P50≈60Hz 辅助）、判据写死
（展宽比 <0.35 连续 / ≥0.55 每条重起 / 之间不明确）。
中止条件：Ctrl-C、任何异常、0x16 保护位（bit2–7）、电压 < 阈值（默认 11000mV）
→ 7 轴全部保持锁力（safe_stop_all，power=0）并如实报告。
"""

import argparse
import csv
import json
import os
import time

from fsbus import (Bus, preflight, STOP_HOLD, STOP_RELEASE,
                   KNOWN_STATUS_MASK, JOINT_LIMITS)

# 分轴全速（Thomas 2026-10-30 选表 A② URDF；T12c）：
#   J0–J2 = 60.2°/s（URDF Joint1–3 = 1.05 rad/s）
#   J3–J5 = 99.1°/s（URDF Joint4–6 = 1.73 rad/s）
#   J6 = 99.1°/s（URDF 无旋转对应值——Joint_L/R 为 prismatic 手指；按与 J3–J5
#   同型号 HX8 推定，标「同型号推定」）
# 映射依据 F22：控制器 a0–a5 ↔ URDF Joint1–6（URDF 的 Joint6 不是夹爪）。
AXIS_SPEEDS = {0: 60.2, 1: 60.2, 2: 60.2, 3: 99.1, 4: 99.1, 5: 99.1, 6: 99.1}
DEFAULT_HOLD_POWER = 0          # H4：保持锁力 power=0（按保护阈值，F38/F49）
SPREAD_CONT = 0.35
SPREAD_RESTART = 0.55
SAMPLE_TIMEOUT = 0.015          # 0x16 等待窗（rtt≈11ms + 余量，F47）
SAMPLE_PERIOD = 0.012           # 采样对表（实测节奏，F45）
HOME_SPEED = 20.0               # H3 归位速度上限 °/s
SAG_LIMIT = 0.5                 # S1/S2 偏差阈 °
MARGIN_DEG = 3.0                # 示教范围两端收缩量
MIN_RANGE_DEG = 10.0
SMOOTH_WORDS = {'1': '连续', '2': '轻微顿挫', '3': '明显顿挫'}


def seg_ms(speed, a, b):
    return int(max(500.0, abs(a - b) / speed * 1000.0))


def tri(t, lo, hi, seg_s):
    ph = (t % (2.0 * seg_s)) / seg_s
    return lo + (hi - lo) * ph if ph <= 1.0 else hi - (hi - lo) * (ph - 1.0)


def read_ang(bus, sid, tries=2):
    for _ in range(tries):
        f = bus.read_angle(sid, timeout=0.05)
        if f and f['verdict'] == 'ok':
            return f['angle']
    return None


def read_all(bus):
    out = {}
    for sid in range(7):
        a = read_ang(bus, sid)
        if a is None:
            print('   !! J%d 读角失败（姿态不可记录）' % sid)
            return None
        out[sid] = round(a, 1)
    return out


def sample_once(bus, sid):
    f = bus.read_monitor(sid, timeout=SAMPLE_TIMEOUT)
    if f is not None and f['verdict'] == 'ok' and f.get('valid_measure', True):
        return f['angle'], 'mon', f['voltage_mV'], f['current_mA'], f['power_mW'], f['status']
    f2 = bus.read_angle(sid, timeout=0.03)
    if f2 is not None and f2['verdict'] == 'ok':
        return f2['angle'], '0a', None, None, None, None
    return None


def guard(s):
    volt, status = s[2], s[5]
    if volt is not None and volt < GUARD_VBAT:
        return 'vbat_low(%dmV<J%d)' % (volt, GUARD_VBAT)
    if status is not None and (status & KNOWN_STATUS_MASK):
        return 'status_protect(0x%02x)' % status
    return None


def dense_sample(bus, sid, dur_s, samples, meta, phase):
    t0 = time.monotonic()
    k = n_bad = 0
    while True:
        if time.monotonic() >= t0 + dur_s:
            return n_bad
        s = sample_once(bus, sid)
        now = time.monotonic()
        if s is not None:
            why = guard(s)
            if why:
                raise RuntimeError(why)
            samples.append((phase, now, s[0]))
            meta.append((phase, now, s[0], s[1], s[2], s[3], s[4], s[5]))
        else:
            n_bad += 1
        k += 1
        d = t0 + k * SAMPLE_PERIOD - time.monotonic()
        if d > 0:
            time.sleep(min(d, 0.004))


def sag_check(bus, pose, tag):
    """S1 下垂检查：与锁定时姿态读数比对，任一轴 >SAG_LIMIT 报警由 Thomas 定。"""
    time.sleep(1.0)
    cur = read_all(bus)
    if cur is None:
        raise RuntimeError('sag_read_fail@%s' % tag)
    bad = [(s2, pose[s2], cur[s2]) for s2 in range(7)
           if abs(cur[s2] - pose[s2]) > SAG_LIMIT]
    if bad:
        print('   [下垂检查] !! 锁定后偏差超 %.1f°：%s' % (
            SAG_LIMIT, [('J%d %.1f→%.1f' % (s2, p, c)) for s2, p, c in bad]))
        ans = input('   [下垂检查] 是否继续？（y=继续 / n=中止并全轴锁）：').strip().lower()
        if ans != 'y':
            raise RuntimeError('sag_over_limit@%s' % tag)
        print('   [下垂检查] Thomas 选择继续（登记为偏离）')
    else:
        print('   [下垂检查] 7 轴偏差全部 ≤%.1f°：通过' % SAG_LIMIT)
    bus.log('sag_check', tag=tag, bad=[[s2, p, c] for s2, p, c in bad] if bad else [])
    return bad


def home_to(bus, sid, lo, label):
    """H3 慢速归位（≤HOME_SPEED）+ 读角验收 ≤1°。"""
    cur = read_ang(bus, sid)
    if cur is None:
        raise RuntimeError('home_read_fail@J%d' % sid)
    ms = int(max(500.0, abs(lo - cur) / HOME_SPEED * 1000.0))
    print('   [%s归位] %+.1f° → %+.1f° / %dms (≤%.0f°/s)' % (label, cur, lo, ms, HOME_SPEED))
    bus.set_angle(sid, lo, ms, 0)
    time.sleep(ms / 1000.0 + 0.3)
    last = None
    for _ in range(4):
        last = read_ang(bus, sid)
        if last is not None:
            break
        time.sleep(0.1)
    ok = last is not None and abs(last - lo) <= 1.0
    print('   [%s归位] 验收: %s（目标 %+.1f°，判过 ≤1.0°）'
          % (label, ('%+.1f° 通过' % last) if ok else (('%.1f° 未过' % last) if last is not None else '读不到'),
             lo))
    if not ok:
        raise RuntimeError('home_not_verified@J%d' % sid)
    return last


def phase_ref(bus, sid, lo, hi, speed, samples, meta):
    ms = seg_ms(speed, lo, hi)
    print('   [P0 参考] 单条 0x08: %+.1f°→%+.1f° / %dms (全速 %.1f°/s)'
          % (lo, hi, ms, abs(hi - lo) * 1000.0 / ms))
    t_cmd = time.monotonic()
    bus.set_angle(sid, hi, ms, 0)
    m = {'P0_go': {'t_start': t_cmd, 't_end': t_cmd + ms / 1000.0,
                   'reversals': [t_cmd + ms / 1000.0]}}
    time.sleep(0.05)
    n_bad = dense_sample(bus, sid, ms / 1000.0 - 0.05 + 0.8, samples, meta, 'P0_go')
    print('   [P0 参考] 采样完成（无效 %d 次）' % n_bad)
    print('   [P0 回程] 单条 0x08 全速返回（不采样）')
    bus.set_angle(sid, lo, seg_ms(speed, hi, lo), 0)
    time.sleep(seg_ms(speed, hi, lo) / 1000.0 + 0.4)
    return m


def phase_stream(bus, sid, lo, hi, speed, period_ms, round_trips, samples, meta, tag):
    seg_s = (hi - lo) / speed
    legs = round_trips * 2
    total_steps = int(round(legs * seg_s / (period_ms / 1000.0)))
    print('   [%s 流式] %dms × %d 条（%d 往返 %+.1f°↔%+.1f°，全速 %.1f°/s，步长 ≈%.1f°/条）'
          % (tag, period_ms, total_steps, round_trips, lo, hi, speed, speed * period_ms / 1000.0))
    t_start = time.monotonic() + 0.2
    k = 0
    next_cmd = t_start
    next_poll = t_start
    n_bad = 0
    while k < total_steps:
        now = time.monotonic()
        if now >= next_cmd:
            target = tri(k * period_ms / 1000.0, lo, hi, seg_s)
            bus.set_angle(sid, target, period_ms, 0)
            bus.log('stream_cmd', phase=tag, k=k, target=round(target, 2),
                    interval_ms=period_ms)
            k += 1
            next_cmd = t_start + k * period_ms / 1000.0
        elif now >= next_poll:
            s = sample_once(bus, sid)
            if s is not None:
                why = guard(s)
                if why:
                    raise RuntimeError(why)
                samples.append((tag, time.monotonic(), s[0]))
                meta.append((tag, time.monotonic(), s[0], s[1], s[2], s[3], s[4], s[5]))
            else:
                n_bad += 1
            next_poll = time.monotonic() + SAMPLE_PERIOD
        else:
            time.sleep(0.0005)
    t_last = t_start + total_steps * period_ms / 1000.0
    reversals = [t_start + i * seg_s for i in range(1, legs)]
    print('   [%s 流式] 下发完成（%d 条，无效采样 %d），停后采样 0.8s' % (tag, total_steps, n_bad))
    dense_sample(bus, sid, 0.8, samples, meta, tag + '_stop')
    return {tag: {'t_start': t_start, 't_end': t_last + period_ms / 1000.0,
                  'reversals': reversals}}


def speed_stats(samples, phase, cmd_speed, win=None, excl=None, w=3):
    pts = [(t, a) for (p, t, a, *rest) in samples if p == phase]
    pts.sort()
    if win:
        pts = [(t, a) for (t, a) in pts if win[0] <= t <= win[1]]
    if excl:
        pts = [(t, a) for (t, a) in pts
               if not any(a0 <= t <= a1 for (a0, a1) in excl)]
    if len(pts) < 2 * w + 2:
        return None
    vs = []
    for i in range(w, len(pts) - w):
        t_lo, t_hi = pts[i - w][0], pts[i + w][0]
        if t_hi <= t_lo:
            continue
        if excl and any(t_lo <= a0 <= t_hi or t_lo <= a1 <= t_hi or (a0 <= t_lo and t_hi <= a1)
                        for (a0, a1) in excl):
            continue
        vs.append(abs(pts[i + w][1] - pts[i - w][1]) / (t_hi - t_lo))
    if not vs:
        return None
    vs.sort()
    n = len(vs)
    span = pts[-1][0] - pts[0][0]
    mean = sum(vs) / n
    p5, p95 = vs[int(n * 0.05)], vs[min(n - 1, int(n * 0.95))]
    return (len(pts), mean, p5, p95,
            sum(1 for v in vs if v < 0.2 * cmd_speed) / n,
            (len(pts) / span) if span > 0 else 0.0,
            ((p95 - p5) / mean) if mean > 0 else float('inf'))


def verdict_of(spread):
    if spread is None:
        return '样本不足'
    if spread < SPREAD_CONT:
        return '倾向连续'
    if spread >= SPREAD_RESTART:
        return '倾向每条重起'
    return '不明确（交关卡）'


def ask_smooth(sid):
    while True:
        a = input('   [口述] J%d 流式顺滑度（1=%s / 2=%s / 3=%s）：'
                  % (sid, SMOOTH_WORDS['1'], SMOOTH_WORDS['2'], SMOOTH_WORDS['3'])).strip()
        if a in SMOOTH_WORDS:
            return a, SMOOTH_WORDS[a]
        print('   请输入 1/2/3')


def confirm_yes(prompt):
    while True:
        a = input(prompt).strip().lower()
        if a in ('y', 'n'):
            return a == 'y'
        print('   请输入 y 或 n')


def save_poses(posefile, doc):
    with open(posefile, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)


def main():
    global GUARD_VBAT
    ap = argparse.ArgumentParser(description='TC-007 流式平滑性（合并式：示教+测试，7 轴逐轴全速）')
    ap.add_argument('--port', default='/dev/ttyS4')
    ap.add_argument('--speed', type=float, default=None,
                    help='全轴统一覆盖 °/s（默认不传=按轴表 AXIS_SPEEDS：J0–J2 60.2 /'
                         ' J3–J6 99.1，Thomas 2026-10-30 选 URDF 表 A②，J6 同型号推定；'
                         '来源③源码限幅 114.59 作参照）')
    ap.add_argument('--periods', default='100,50')
    ap.add_argument('--order', default='6,5,4,3,0,2,1')
    ap.add_argument('--round-trips', type=int, default=3)
    ap.add_argument('--stop-power', type=int, default=DEFAULT_HOLD_POWER,
                    help='0x18 保持锁力 power_mW（默认 0=按功率保护阈值执行，F38/F49；'
                         'F49 等效性只在空载 J6 验证过，J1/J2 扛臂保持不能沿用 6000）')
    ap.add_argument('--vbat-low', type=int, default=11000,
                    help='电压中止阈值 mV（默认 11000：静置 12094–12375 留约 1V，F15）')
    args = ap.parse_args()
    GUARD_VBAT = args.vbat_low
    pw = args.stop_power
    order = [int(x) for x in args.order.split(',')]
    periods = [int(x) for x in args.periods.split(',')]

    def axis_speed(sid):
        return args.speed if args.speed is not None else AXIS_SPEEDS[sid]

    ts = time.strftime('%Y%m%d_%H%M%S')
    here = os.path.dirname(os.path.abspath(__file__))
    log = os.path.join(here, 'tc007_log_%s.jsonl' % ts)
    posefile = os.path.join(here, 'poses_%s.json' % ts)
    print('TC-007 流式平滑性 · 合并式（示教+测试逐轴，板载时钟 %s，仅参考）' % ts)
    print('!! 分轴全速（Thomas 2026-10-30 选 URDF 表 A②）：J0–J2 %.1f°/s / J3–J6 %.1f°/s'
          '（J6 同型号推定）%s，开始前确认：'
          % (AXIS_SPEEDS[0], AXIS_SPEEDS[3],
             ('；--speed=%.1f 全轴覆盖' % args.speed) if args.speed is not None else ''))
    print('!!   工作区无人无杂物（全速行程按示教范围）、急停在手边（断电，F27）')
    print('!!   残余风险（Thomas 已知悉，用例 v6 §安全注意）：全速=限幅首测（F4）、'
          '高于 URDF 标注、无备件、换向冲击、欠压失力（F15）、电源 240W（F21）')
    print('!!   保持锁力 power=%dmW（%s）' % (pw, '按功率保护阈值执行' if pw == 0 else '指定值'))
    print('!! 中止条件：Ctrl-C/异常/0x16 保护位/电压<%dmV → 7 轴全部保持锁力（safe_stop_all）' % GUARD_VBAT)
    print('!! 前置检查：4 服务 inactive + %s 无占用' % args.port)
    ok_pf, msgs = preflight(args.port)
    for m in msgs:
        print('   preflight: %s' % m)
    if not ok_pf:
        print('   !! preflight 未过，退出')
        return
    print('日志: %s\n示教: %s（边跑边存）' % (log, posefile))
    bus = Bus(args.port, log_path=log)
    results, smooth, joints = {}, {}, {}
    test_range = {}
    cur_axis = None
    samples, meta = [], []
    try:
        bus.drain()
        locked = False   # 脚本跟踪：开跑前全失力（停服+上电锁力关）
        for n, sid in enumerate(order):
            cur_axis = sid
            speed = axis_speed(sid)
            print('')
            print('========== 第 %d/%d 轴：J%d（本轴全速 %.1f°/s）=========='
                  % (n + 1, len(order), sid, speed))
            # A/H2 摆姿态（H1：先确认扶住才释放）
            if locked:
                input('   [H1] 请扶住整臂，按回车解锁全部关节（0x18 释放）：')
                bus.lock_all(range(7), pw, mode=STOP_RELEASE)
                locked = False
                time.sleep(0.3)
            input('   [姿态] 请摆好 J%d 的测试姿态（整个臂的构型），扶住后按回车（回车后先锁 7 轴再读）：' % sid)
            sent = bus.lock_all(range(7), pw, mode=STOP_HOLD)
            locked = True
            if sent != 7:
                print('   !! lock_all 仅发出 %d/7，重试……' % sent)
                if bus.lock_all(range(7), pw, mode=STOP_HOLD) + sent < 7:
                    raise RuntimeError('lock_all_failed@J%d' % sid)
            time.sleep(0.2)
            pose = read_all(bus)
            if pose is None:
                raise RuntimeError('pose_read_fail@J%d' % sid)
            print('   [姿态] 已锁定并记录：%s' % json.dumps(pose, ensure_ascii=False))
            # S1 下垂检查（松手→等 1s→重读比对）
            input('   [S1] 请慢慢松手（臂由舵机保持），按回车开始下垂检查：')
            sag = sag_check(bus, pose, 'J%d' % sid)
            # D/H1 示教范围：扶住确认后只释放被测轴
            input('   [H1] 请扶住 J%d，按回车只解锁 J%d（其余 6 轴保持锁定）：' % (sid, sid))
            bus.stop(sid, STOP_RELEASE, pw)
            locked = False
            input('   [示教] 手推 J%d 到摆动下限（留出行程余量），按回车读角：' % sid)
            lo = read_ang(bus, sid)
            if lo is None:
                raise RuntimeError('lo_read_fail@J%d' % sid)
            print('   [示教] J%d 下限 = %+.1f°' % (sid, lo))
            input('   [示教] 手推 J%d 到摆动上限，按回车读角：' % sid)
            hi = read_ang(bus, sid)
            if hi is None:
                raise RuntimeError('hi_read_fail@J%d' % sid)
            print('   [示教] J%d 上限 = %+.1f°' % (sid, hi))
            if hi < lo:
                lo, hi = hi, lo
            bus.stop(sid, STOP_HOLD, pw)
            locked = True
            print('   [示教] J%d 已重新锁住（示教 [%+.1f, %+.1f]）' % (sid, lo, hi))
            time.sleep(0.3)
            # 测试范围 = 示教收 3° ∩ F7
            f7lo, f7hi = JOINT_LIMITS[sid]
            tlo = max(lo + MARGIN_DEG, f7lo)
            thi = min(hi - MARGIN_DEG, f7hi)
            if thi - tlo < MIN_RANGE_DEG:
                print('   !! 测试行程 %.1f° < 最小 %.0f°（收 3°∩F7 后 [%+.1f,%+.1f]）'
                      % (thi - tlo, MIN_RANGE_DEG, tlo, thi))
                if confirm_yes('      接受现状继续吗？(y/n)：'):
                    print('      已接受（登记为偏离）')
                else:
                    raise RuntimeError('range_too_narrow@J%d' % sid)
            print('   [范围] 测试范围 [%+.1f, %+.1f]（示教收 %.0f° ∩ F7，行程 %.1f°）'
                  % (tlo, thi, MARGIN_DEG, thi - tlo))
            joints[str(sid)] = {'pose': pose, 'lo': round(lo, 1), 'hi': round(hi, 1)}
            test_range[str(sid)] = [round(tlo, 1), round(thi, 1)]
            save_poses(posefile, {'ts': ts, 'order': order, 'stop_power': pw,
                                  'margin_deg': MARGIN_DEG, 'joints': joints,
                                  'test_range': test_range})
            # H3 慢速归位到 lo + 验收
            home_to(bus, sid, tlo, 'J%d ' % sid)
            # 测试序列（被测轴 0x08 驱动；其余 6 轴锁定）
            samples, meta = [], []
            pm = {}
            pm.update(phase_ref(bus, sid, tlo, thi, speed, samples, meta))
            time.sleep(0.4)
            for pms in periods:
                pm.update(phase_stream(bus, sid, tlo, thi, speed, pms,
                                       args.round_trips, samples, meta, 'P%d' % pms))
                time.sleep(0.3)
            dense_sample(bus, sid, 1.0, samples, meta, 'idle')
            ans, word = ask_smooth(sid)
            smooth['J%d' % sid] = word
            bus.log('smooth_oral', sid=sid, ans=ans, word=word)
            bus.stop(sid, STOP_HOLD, pw)
            # S2 非被测轴核查：整段测试中未被带动
            time.sleep(0.5)
            cur2 = read_all(bus)
            s2bad = []
            if cur2 is not None:
                s2bad = [(s3, pose[s3], cur2[s3]) for s3 in range(7)
                         if s3 != sid and abs(cur2[s3] - pose[s3]) > SAG_LIMIT]
                if s2bad:
                    print('   [S2] !! 非被测轴被带动（>%.1f°）：%s' % (
                        SAG_LIMIT, [('J%d %.1f→%.1f' % (s3, p, c)) for s3, p, c in s2bad]))
                else:
                    print('   [S2] 非被测轴偏差全部 ≤%.1f°：通过（J%d 已锁回）' % (SAG_LIMIT, sid))
            bus.log('axis_end_check', sid=sid, bad=[[s3, p, c] for s3, p, c in s2bad] if s2bad else [])
            # CSV 落盘（每轴一份）
            csvf = os.path.join(here, 'tc007_samples_J%d_%s.csv' % (sid, ts))
            with open(csvf, 'w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['phase', 't_mono_s', 'angle_deg', 'src', 'volt_mV',
                            'curr_mA', 'pow_mW', 'status'])
                for (p, t, a, src, v, c, pw2, st) in meta:
                    w.writerow([p, '%.4f' % t, '%.2f' % a, src,
                                v if v is not None else '', c if c is not None else '',
                                pw2 if pw2 is not None else '', st if st is not None else ''])
            ax_res = {}
            for tag in ['P0_go'] + ['P%d' % p2 for p2 in periods]:
                m2 = pm.get(tag)
                if not m2:
                    continue
                excl = [(r - 0.2, r + 0.2) for r in m2['reversals']]
                st = speed_stats(samples, tag, speed, win=(m2['t_start'], m2['t_end']), excl=excl)
                if st:
                    v = verdict_of(st[6])
                    ax_res[tag] = (st, v)
                    print('SUMMARY J%d %s: n=%d (%.0fHz) |v| mean=%.1f°/s p5=%.1f p95=%.1f '
                          '展宽比=%.2f → %s'
                          % (sid, tag, st[0], st[5], st[1], st[2], st[3], st[6], v))
                else:
                    ax_res[tag] = (None, '样本不足')
                    print('SUMMARY J%d %s: 样本不足' % (sid, tag))
            results['J%d' % sid] = ('done', ax_res, s2bad)
            input('   [轮间] J%d 完成。确认继续下一轴，按回车（Ctrl-C=中止并全轴锁）：' % sid)
        print('')
        print('===== SUMMARY（判定=Q3 关卡：执行方 出数、审查方 复核裁定；阈值 <%.2f 连续 / ≥%.2f 重起）====='
              % (SPREAD_CONT, SPREAD_RESTART))
        for k in ['J%d' % s for s in order]:
            r = results.get(k)
            if not r:
                continue
            tags = ', '.join('%s 展宽比=%s（%s）'
                             % (t, ('%.2f' % v[0][6]) if v[0] else 'n/a', v[1])
                             for t, v in r[1].items())
            extra = ' | S2非被测轴异常:%s' % r[2] if r[2] else ''
            print('SUMMARY %s: %s | 口述顺滑度=%s%s' % (k, tags, smooth.get(k, 'n/a'), extra))
        print('SUMMARY 判据：展宽比<%.2f→倾向连续；≥%.2f→倾向每条重起；之间不明确（交关卡）'
              % (SPREAD_CONT, SPREAD_RESTART))
        print('SUMMARY 示教文件：%s（含 7 轴姿态/范围/测试范围）' % posefile)
    except (KeyboardInterrupt, Exception) as e:
        print('\n!! 异常/中断（%s）：safe_stop_all 收尾（7 轴全部保持锁力）——' % repr(e))
        r = bus.safe_stop_all(range(7), pw, mode=STOP_HOLD)
        bad = [sid for sid, s in r['still'].items() if s is not True]
        print('   停止帧已尝试发送 %d；各轴停住判定：%s'
              % (r['sent'], {('J%d' % s): v for s, v in r['still'].items()}))
        if bad:
            print('   !! 未能确认已锁住的轴：%s——请 Thomas 扶住臂目视确认，必要时急停（断电，F27）'
                  % ['J%d' % s for s in bad])
        if cur_axis is not None and meta:
            csvf = os.path.join(here, 'tc007_samples_J%d_%s_中止.csv' % (cur_axis, ts))
            try:
                with open(csvf, 'w', newline='') as fh:
                    w = csv.writer(fh)
                    w.writerow(['phase', 't_mono_s', 'angle_deg', 'src', 'volt_mV',
                                'curr_mA', 'pow_mW', 'status'])
                    for (p, t, a, src, v, c, pw2, st) in meta:
                        w.writerow([p, '%.4f' % t, '%.2f' % a, src,
                                    v if v is not None else '', c if c is not None else '',
                                    pw2 if pw2 is not None else '', st if st is not None else ''])
                print('   已完成部分采样落盘：%s' % csvf)
            except Exception as e2:
                print('   (落盘失败: %s)' % e2)
        for k, r in results.items():
            print('   SUMMARY(部分) %s: %s' % (k, r[0]))
    finally:
        bus.close()


if __name__ == '__main__':
    main()
