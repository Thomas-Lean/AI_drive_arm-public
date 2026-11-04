#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tc004_recon.py —— TC-004 总线侦察与 0x16 只读摸底（P0.4，V330 重测轮）

只发只读指令：0x01 ping / 0x0A 读角度 / 0x16 数据监控。不发任何运动类指令。
帧间隔由 fsbus.Bus 的 min_gap_ms=10 统一保证（≥10ms，F14 官方建议；复审 M4）。

V330 重测（2026-10-30，F35）：
  - 2026-10-28 首轮在出厂固件 V311 上 0x16 全部零响应（7 舵机 × 40 次请求，
    ping/0x0A 正常）；厂家确认 V311 不支持 0x16/0x18/0x19（F34）——
    本轮在 V330 上重测 0x16，判定标准沿用用例 P2/P3。
  - 新增 F40 对照（只记录不判定）：0x16 回读的电流/功率与上位机显示值
    （V330 升级后 J6/J2 静止约 14mA / 172mW，原因【未验证】）并列打印。
    0x16 与上位机读数口径是否一致，本身就是本轮要记录的事实，不设阈值。
  - 0x16 响应格式按 V311 文档口径校验（params 16 字节）；V330 若有格式
    变化会以 bad_len 如实暴露，原始帧在日志 rx 事件中可查。

用法（臂载计算机，脚本所在目录）：
    python3 tc004_recon.py
    python3 tc004_recon.py --port /dev/ttyS4 --rounds 3

输出：
    - 终端逐行结果 + 末尾 SUMMARY 块（回贴给 执行方 判定）
    - tc004_log_<板载时间戳>.jsonl（事件级原始数据；板载时钟不准，F18，仅作文件名）
"""

import argparse
import os
import time

from fsbus import Bus, fmt_monitor, norm_single_turn


def wait_until(t):
    while True:
        d = t - time.monotonic()
        if d <= 0:
            return
        time.sleep(min(d, 0.005))


def phase_ping(bus, rounds):
    print('===== 阶段1：ping 扫描（0x01，%d 轮 × ID 0–6，轮内间隔 60ms、轮间 ≥150ms）=====' % rounds)
    ok = {i: 0 for i in range(7)}
    rtts = {i: [] for i in range(7)}
    for r in range(rounds):
        t0 = time.monotonic() + 0.15  # 轮间保护 ≥150ms（复审 M4）
        for sid in range(7):
            wait_until(t0 + sid * 0.060)
            f = bus.ping(sid)
            if f and f['verdict'] == 'ok':
                ok[sid] += 1
                rtts[sid].append(f['rtt'] * 1000)
                print('round%d ping J%d: OK   rtt=%.1fms' % (r + 1, sid, f['rtt'] * 1000))
            else:
                v = f['verdict'] if f else 'timeout'
                print('round%d ping J%d: FAIL (%s)' % (r + 1, sid, v))
    online = [i for i in range(7) if ok[i] == rounds]
    print('-- ping 小结：%d/%d 轮全过：%s' % (rounds, rounds, online if online else '无'))
    for i in range(7):
        if rtts[i]:
            rtts[i].sort()
            print('   J%d: %d/%d OK, rtt 中位 %.1fms 最大 %.1fms'
                  % (i, ok[i], rounds, rtts[i][len(rtts[i]) // 2], rtts[i][-1]))
        else:
            print('   J%d: 0/%d OK' % (i, rounds))
    return online, ok, rtts


def phase_monitor(bus, counts):
    """counts: {sid: 次数}，采样间隔 150ms（远大于官方 ≥10ms 建议，F14）"""
    print('===== 阶段2：0x16 数据监控（间隔 150ms）=====')
    stats = {}
    for sid, n in counts.items():
        vals = []
        n_sent = n_sent_bad = 0
        for k in range(n):
            f = bus.read_monitor(sid)
            if f is not None:
                n_sent += 1
                if f['verdict'] == 'ok':
                    vals.append(f)
                    print('   ' + fmt_monitor(sid, f))
                else:
                    n_sent_bad += 1
                    print('   J%d 0x16: %s  #%d' % (sid, f['verdict'], k + 1))
            else:
                n_sent_bad += 1
                print('   J%d 0x16: TIMEOUT  #%d' % (sid, k + 1))
            wait_until(time.monotonic() + 0.150)
        if vals:
            i_sorted = sorted(v['current_mA'] for v in vals)
            p_sorted = sorted(v['power_mW'] for v in vals)
            stats[sid] = {
                'n_ok': len(vals), 'n': n, 'n_noresp': n_sent_bad,
                'v_min': min(v['voltage_mV'] for v in vals),
                'v_max': max(v['voltage_mV'] for v in vals),
                'i_max': max(v['current_mA'] for v in vals),
                'i_med': i_sorted[len(i_sorted) // 2],
                'p_med': p_sorted[len(p_sorted) // 2],
                't_max': max((v['temp_c'] or 0) for v in vals),
                'st_nonzero': sum(1 for v in vals if v['status'] != 0),
                'n_sentinel': sum(1 for v in vals if v['sentinel']),
                'ang_last': vals[-1]['angle'],
            }
            s = stats[sid]
            print('   -- J%d: %d/%d 有效（无响应/坏帧 %d），电压 %d–%dmV, 电流峰值 %dmA '
                  '中位 %dmA, 功率中位 %dmW, 温度峰值 %.1fC, 状态非零 %d 次, 无效哨兵 %d 次'
                  % (sid, s['n_ok'], s['n'], s['n_noresp'], s['v_min'], s['v_max'],
                     s['i_max'], s['i_med'], s['p_med'], s['t_max'],
                     s['st_nonzero'], s['n_sentinel']))
        else:
            stats[sid] = {'n_ok': 0, 'n': n, 'n_noresp': n_sent_bad}
            print('   -- J%d: 0/%d 有效（无响应/坏帧 %d）' % (sid, n, n_sent_bad))
    return stats


def phase_cross(bus, sids, n):
    """0x0A（单圈口径）vs 0x16（多圈口径 u32+圈数）→ 归一化后比较（复审 M7）：
    归一化换算本身【待实测核对】，不一致按「口径问题」记录，不判硬件失败"""
    print('===== 阶段3：0x0A 读角度交叉核对（0x0A 与 0x16 之间由 10ms 帧间隔隔开）=====')
    out = {}
    for sid in sids:
        angs, diffs = [], []
        for k in range(n):
            fa = bus.read_angle(sid)
            fm = bus.read_monitor(sid)
            if fa and fm and fa['verdict'] == 'ok' and fm['verdict'] == 'ok' and fm['valid_measure']:
                a0a = fa['angle']
                a16 = norm_single_turn(fm['angle'])
                diffs.append(abs(a0a - a16))
                print('J%d #%d: 0x0A=%+.1f°  0x16=%+.1f°(归一) Δ=%.2f°'
                      % (sid, k + 1, a0a, a16, abs(a0a - a16)))
            else:
                va = fa['verdict'] if fa else 'timeout'
                vm = fm['verdict'] if fm else 'timeout'
                print('J%d #%d: 0x0A=%s 0x16=%s（本对作废）' % (sid, k + 1, va, vm))
            if fa and fa['verdict'] == 'ok':
                angs.append(fa['angle'])
            wait_until(time.monotonic() + 0.150)
        if angs:
            md = sum(diffs) / len(diffs) if diffs else None
            out[sid] = (len(angs), md, len(diffs))
            print('   -- J%d: 0x0A %d/%d OK; 归一化后 |Δ| 均值 %s（有效对 %d）'
                  % (sid, len(angs), n, ('%.2f°' % md) if md is not None else 'n/a', len(diffs)))
        else:
            out[sid] = (0, None, 0)
            print('   -- J%d: 0x0A 0/%d OK' % (sid, n))
    return out


def main():
    ap = argparse.ArgumentParser(description='TC-004 总线侦察（只读）')
    ap.add_argument('--port', default='/dev/ttyS4')
    ap.add_argument('--rounds', type=int, default=3, help='ping 扫描轮数')
    args = ap.parse_args()

    ts = time.strftime('%Y%m%d_%H%M%S')
    log = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'tc004_log_%s.jsonl' % ts)
    print('TC-004 总线侦察与 0x16 只读摸底 —— V330 重测轮（固件 F35；上轮 V311 0x16 '
          '零响应原因见 F34；板载时钟 %s，仅参考）' % ts)
    print('!! 本脚本只发只读指令（0x01/0x0A/0x16），不发任何运动指令')
    print('!! 前置检查：4 服务 inactive + %s 无占用（复审 H1）' % args.port)
    from fsbus import preflight
    ok_pf, msgs = preflight(args.port)
    for m in msgs:
        print('   preflight: %s' % m)
    if not ok_pf:
        print('   !! preflight 未过，退出。先完成 TC-002 停止流程再运行本脚本')
        return
    print('日志: %s' % log)
    bus = Bus(args.port, log_path=log)
    try:
        bus.drain()
        online, ok, rtts = phase_ping(bus, args.rounds)
        # 主要对象 J5/J6 各 10 次（§00.6 第 7 条：先 J6 或 J5），其余 2 次基线
        mon = phase_monitor(bus, {5: 10, 6: 10, 0: 2, 1: 2, 2: 2, 3: 2, 4: 2})
        cross = phase_cross(bus, [5, 6], 5)
        print('===== SUMMARY =====')
        print('SUMMARY ping_online_ids = %s (全部轮次全过者)' % online)
        print('SUMMARY ping_ok_counts  = %s' % {i: '%d/%d' % (ok[i], args.rounds) for i in range(7)})
        for sid in sorted(mon):
            s = mon[sid]
            if s['n_ok']:
                print('SUMMARY J%d 0x16 %d/%d 有效（无响应/坏帧 %d），电压 %d–%dmV, '
                      '电流中位 %dmA 功率中位 %dmW, 状态非零 %d 次(bit0/1 含义未知), 无效哨兵 %d 次'
                      % (sid, s['n_ok'], s['n'], s['n_noresp'], s['v_min'], s['v_max'],
                         s['i_med'], s['p_med'], s['st_nonzero'], s['n_sentinel']))
            else:
                print('SUMMARY J%d 0x16 0/%d 有效' % (sid, s['n']))
        print('SUMMARY F40 对照（只记录不判定）：上位机在 V330 升级后读 J6/J2 静止'
              '约 14mA / 172mW（F40，原因【未验证】）；上表 0x16 电流/功率中位为'
              '本脚本读数——两口径是否一致交离线分析记录，不设阈值不判定')
        for sid in sorted(cross):
            n, d, pairs = cross[sid]
            print('SUMMARY J%d 0x0A %d/5 OK, 归一化 |Δ0x0A−0x16| 均值 %s（有效对 %d）'
                  % (sid, n, ('%.2f°' % d) if d is not None else 'n/a', pairs))
        print('TC-004 脚本执行完毕；判定由 执行方 对照用例给出')
    finally:
        bus.close()


if __name__ == '__main__':
    main()
