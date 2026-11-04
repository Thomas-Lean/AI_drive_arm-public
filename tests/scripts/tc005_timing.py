#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tc005_timing.py —— TC-005 指令间隔语义与同步监控耗时（P0.4，关闭 §11 Q4）

只发只读指令（0x01 ping / 0x16 监控 / 0x19+0x16 同步监控），不发任何运动指令。

三个阶段（对应 Q4 两问）：
  A 单舵机定频查询：J6 以不同周期连发 0x16 —— 标定本机丢包门限 L
  B 双舵机交替查询：J5/J6 交替、命令间隔 c（同舵机间隔 ≈2c）——
      与 A 对比：门限随交替减半 → 间隔按单舵机计；不变 → 按总线计；
      两组都全零 miss → 门限未观察到（记下界，不硬给结论）
  C 同步监控 0x19+0x16 d=1：n=1→2→7 递进（每级须见到成功才升级），
      7 轴 ×1000 周期 —— 收齐 7 个回包耗时分布

复审要点（本版落实）：
  - M2：verdict 词表 ok/timeout/corrupt/bad_len/id_mismatch；超时后开静默窗，
    迟到帧记 late 不进统计；协议无序号，同命令同 ID 迟到/当次不可辨识——
    离线分析按 rtt 与等待窗口关系标「疑似迟到」，不作超范围归因
  - M5：阶段 C 绝对周期基准调度，记录实际请求起点间隔与超期；打印中的
    「软件间隔」= 上次调用结束→本次发出（软件口径，不作总线物理空闲证据）
  - H5：n=1 全超时只记「本配置未收到有效响应」（重试 3 次更长超时后仍无才停），
    不判固件不支持；结论分支见用例文档（含「不可辨识/未观察到门限」）

V330 修订（T9，2026-10-30，审查方 B3 裁定·二，依据 F41 rtt jsonl 全量复核）：
  - 阶段 A、B 等回包窗**均固定 20ms**（≈全量实测最大 14.4ms 的 1.4 倍），
    >20ms 才计 timeout，单独登记为异常。初版裁定曾保留 A 阶段 8ms 窗
    （据 TC-004 终端打印 J6 5.7–6.5ms）——后经 jsonl 按 tx 配对全量复核
    （J5/J6 各 15 样本）：J6 在前序间隔 ≈10ms 时 rtt 4.7–12.1ms（3/5 超 8ms），
    「8ms 窗够用」不成立，故 A/B 统一 20ms
  - 仍 J5/J6 交替不换关节（偏慢/双峰本身是要追的现象，不把慢响应藏进 timeout）
  - 每档按舵机打印 ok 响应 rtt 分布（中位/p95/max）+ 前序发出间隔中位；
    阶段末按前序发出间隔分箱（≤12ms / >12ms）汇总每舵机 rtt
  - 观察目标（只登记不判失败）：rtt 与前序发出间隔的关系——TC-004 数据
    显示 J6 前序间隔越小 rtt 越大（密集 10.4–12.1 vs 宽松 5.7–6.5ms）；
    J5 宽松下亦双峰至 14.4ms（与前序间隔无关的舵机自身形态）

本脚本 Bus 用 min_gap_ms=0（A/B 就是间隔实验本身）；阶段 C 的帧间隔由
25ms 周期调度保证请求帧间总线空闲 ≥10ms（事务约 13–15ms + 空闲 ≈10ms，F14）。
"""

import argparse
import os
import time

from fsbus import Bus


def med(v):
    v = sorted(v)
    return v[len(v) // 2] if v else float('nan')


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * p))] if v else float('nan')


def wait_until(t):
    while True:
        d = t - time.monotonic()
        if d <= 0:
            return
        time.sleep(min(d, 0.002))


def run_single_phase(bus, phase_name, sids_cycle, cycles, wait_cap_s=None):
    """定频发 0x16。sids_cycle: 每命令目标 id 序列（A=同 id 重复，B=交替）。
    wait_cap_s=None → 等回包上限 = min(8ms, 周期)（阶段 A 原逻辑，B3 裁定保持不变）；
    wait_cap_s=固定秒数 → 全档位统一该值（阶段 B=0.020：J5 实测 0x16 rtt 最大
    14.4ms 的约 1.4 倍，超 20ms 才计 timeout——审查方 B3 裁定 2026-10-30，依据 F41）。
    返回逐条记录 [(cyc_ms, sid, verdict, s2s_ms, sw_gap_ms, rtt_ms)]"""
    recs = []
    prev_send = prev_done = None
    for cyc_ms in cycles:
        n = {'ok': 0, 'timeout': 0, 'corrupt': 0, 'bad_len': 0, 'id_mismatch': 0}
        late_total = 0
        wait_cap = (min(0.008, cyc_ms / 1000.0) if wait_cap_s is None
                    else wait_cap_s)  # 单次等回包上限；超时按未收到记（B3）
        t_base = time.monotonic() + 0.05
        k = 0
        while k < len(sids_cycle):
            sid = sids_cycle[k]
            t_send = t_base + k * cyc_ms / 1000.0
            now = time.monotonic()
            if t_send - now < -0.05:  # 落后过多（如上一条超时），平移基准重新对表
                t_base = now - k * cyc_ms / 1000.0
                t_send = now
            wait_until(t_send)
            t_call = time.monotonic()
            f = bus.read_monitor(sid, timeout=wait_cap)
            t_done = time.monotonic()
            # s2s = 相邻两条「发出」的间隔（实测口径）
            s2s_ms = (t_call - prev_send) * 1000 if prev_send is not None else float('nan')
            # 软件间隔 = 上一条调用结束→本条发出（软件口径，非总线物理空闲，M5）
            sw_gap_ms = (t_call - prev_done) * 1000 if prev_done is not None else float('nan')
            verdict = f['verdict'] if f is not None else 'timeout'
            late_total += bus.txn_late  # 超时静默窗的迟到帧（fsbus.transact 统一处理，M-a）
            recs.append((cyc_ms, sid, verdict, s2s_ms, sw_gap_ms,
                         (f['rtt'] * 1000) if f is not None else float('nan')))
            n[verdict] = n.get(verdict, 0) + 1
            prev_send, prev_done = t_call, t_done
            k += 1
        oks = [r for r in recs if r[0] == cyc_ms and r[2] == 'ok']
        print('%s 周期=%2dms n=%3d ok=%3d timeout=%3d corrupt=%3d bad_len=%3d id_mismatch=%3d late=%d | '
              '实测发出间隔中位 %5.1fms 软件间隔中位 %5.1fms rtt中位 %4.1fms' % (
                  phase_name, cyc_ms, len(sids_cycle), n['ok'], n['timeout'], n['corrupt'],
                  n['bad_len'], n['id_mismatch'], late_total,
                  med([r[3] for r in oks]), med([r[4] for r in oks]), med([r[5] for r in oks])))
        # B3 裁定 3：每档按舵机输出 ok 响应的 rtt 分布（J5 双峰等形态交判读登记）
        for sid_u in sorted(set(sids_cycle)):
            rs = [(r[5], r[3]) for r in recs
                  if r[0] == cyc_ms and r[2] == 'ok' and r[1] == sid_u]
            if rs:
                print('   %s 周期=%2dms J%d rtt 分布: n=%d 中位 %.1fms p95 %.1fms max %.1fms'
                      ' | 前序发出间隔中位 %.1fms'
                      % (phase_name, cyc_ms, sid_u, len(rs),
                         med([a for a, _ in rs]), pct([a for a, _ in rs], 0.95),
                         max(a for a, _ in rs), med([s for _, s in rs if s == s])))
        bus.log('phase_rate', phase=phase_name, cycle_ms=cyc_ms, late=late_total, **n)
        time.sleep(0.3)
        bus.drain()
    # B3 裁定 4（观察目标）：rtt 与前序发出间隔的关系——按实测 s2s 分箱
    # （≤12ms = 紧贴总线最小间隔 10ms 的密集档；>12ms = 有喘息）。只登记不判失败
    for sid_u in sorted(set(sids_cycle)):
        for tag, sel in (('s2s≤12ms', True), ('s2s>12ms', False)):
            rs = [r[5] for r in recs if r[2] == 'ok' and r[1] == sid_u
                  and r[3] == r[3] and ((r[3] <= 12.0) == sel)]
            if rs:
                print('%s J%d rtt×前序发出间隔 [%s]: n=%d 中位 %.1fms p95 %.1fms max %.1fms'
                      % (phase_name, sid_u, tag, len(rs), med(rs), pct(rs, 0.95), max(rs)))
    return recs


def phase_a(bus, sid, cycles, n_cmd):
    print('===== 阶段A：单舵机(J%d)定频 0x16 =====' % sid)
    return run_single_phase(bus, 'A', [sid] * n_cmd, cycles, wait_cap_s=0.020)


def phase_b(bus, sid1, sid2, cycles, n_cmd):
    print('===== 阶段B：J%d/J%d 交替 0x16（同舵机间隔 ≈ 2×命令间隔）=====' % (sid1, sid2))
    seq = [sid1 if i % 2 == 0 else sid2 for i in range(n_cmd)]
    return run_single_phase(bus, 'B', seq, cycles, wait_cap_s=0.020)


def phase_c(bus, n7, period_ms):
    print('===== 阶段C：0x19+0x16 d=1 同步监控（只读，每级须见到成功才升级）=====')
    for ids, cnt in (([6], 20), ([5, 6], 20), (list(range(7)), n7)):
        if ids == list(range(7)):
            print('7 轴请求帧(首帧) = %s（预期 12 4c 19 0a 16 01 07 00 01 02 03 04 05 06 cs）'
                  % Bus.build(0x19, bytes([0x16, 1, 7]) + bytes(range(7))).hex())
        tot, req_t, overruns, all_ok, extra_b, bad_b = [], [], 0, 0, 0, 0
        tmo = 0.1
        # M-b：全组共用一个绝对周期基准，请求严格按 t_base+k*period 发出，
        # 事务后不再额外等待（此前每轮多等一个周期 → 标称 25ms 实测 ~45ms）
        t_base = time.monotonic() + 0.05
        for k in range(cnt):
            wait_until(t_base + k * period_ms / 1000.0)
            t_req = time.monotonic()
            req_t.append(t_req)
            r = bus.sync_monitor(ids, timeout=tmo)
            tot.append(r['total_s'] * 1000)
            extra_b += r['extra']
            bad_b += r['bad']
            if r['ok']:
                all_ok += 1
            if len(ids) == 7 and (k + 1) % 200 == 0:
                print('   …%d/%d 周期' % (k + 1, cnt))
        # 实际请求起点间隔与超期（M5）
        ivs = [(req_t[i] - req_t[i - 1]) * 1000 for i in range(1, len(req_t))]
        overruns = sum(1 for v in ivs if v > period_ms * 1.5)
        print('n=%d ×%d 周期 %dms: 收齐率 %d/%d, 总耗时中位 %.1fms p90 %.1fms p99 %.1fms max %.1fms | '
              '请求起点间隔中位 %.1fms max %.1fms 超期(>1.5×周期) %d 次 | extra=%d bad=%d'
              % (len(ids), cnt, period_ms, all_ok, cnt, med(tot), pct(tot, 0.90), pct(tot, 0.99),
                 max(tot), med(ivs), max(ivs), overruns, extra_b, bad_b))
        bus.log('sync_phase', n=len(ids), cnt=cnt, all_ok=all_ok,
                total_ms_med=round(med(tot), 3), total_ms_p99=round(pct(tot, 0.99), 3),
                total_ms_max=round(max(tot), 3), req_iv_med=round(med(ivs), 3),
                req_iv_max=round(max(ivs), 3), overruns=overruns,
                extra=extra_b, bad=bad_b)
        if all_ok == 0:
            if len(ids) == 1:
                print('   -- n=1 全部未收到有效响应：换 0.25s 超时重试 3 次 …')
                got_any = False
                for tr in range(3):
                    r = bus.sync_monitor(ids, timeout=0.25)
                    print('      重试%d: ok=%s total=%.1fms' % (tr + 1, r['ok'], r['total_s'] * 1000))
                    got_any = got_any or r['ok']
                    time.sleep(0.1)
                if not got_any:
                    print('   !! 仍未收到有效响应——按用例 H5 分支处理：只记录「本配置未收到有效响应」，'
                          '不判固件不支持，停止升级（n=2/7 不做），交离线分析')
                    return False
            else:
                print('   !! 本级收齐 0 次（上一级曾成功）——记录事实，停止升级，交分析')
                return False
        time.sleep(0.3)
        bus.drain()
    return True


def main():
    ap = argparse.ArgumentParser(description='TC-005 间隔语义与同步监控耗时（只读）')
    ap.add_argument('--port', default='/dev/ttyS4')
    ap.add_argument('--phase', choices=['A', 'B', 'C', 'all'], default='all')
    ap.add_argument('--id-a', type=int, default=6, help='阶段A 目标（J6）')
    ap.add_argument('--id-b', type=int, default=5, help='阶段B 交替对象（J5）')
    ap.add_argument('--cycles', default='20,15,12,10,8,6', help='阶段A 命令周期表 ms')
    ap.add_argument('--cycles-b', default='12,10,8,6,5,4,3', help='阶段B 命令间隔表 ms')
    ap.add_argument('--n-cmd', type=int, default=150, help='A/B 每档命令数')
    ap.add_argument('--n7', type=int, default=1000, help='阶段C 7轴周期数（Q4 要求 ≥1000）')
    ap.add_argument('--period-c', type=int, default=25,
                    help='阶段C 周期 ms（默认 25：事务约 13–15ms + 空闲 ≥10ms，F14）')
    args = ap.parse_args()

    ts = time.strftime('%Y%m%d_%H%M%S')
    log = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tc005_log_%s.jsonl' % ts)
    print('TC-005 指令间隔语义与同步监控耗时（板载时钟 %s，仅参考）' % ts)
    print('!! 本脚本只发只读指令（0x01/0x16/0x19+0x16），不发任何运动指令')
    print('!! 前置检查：4 服务 inactive + %s 无占用（复审 H1）' % args.port)
    from fsbus import preflight
    ok_pf, msgs = preflight(args.port)
    for m in msgs:
        print('   preflight: %s' % m)
    if not ok_pf:
        print('   !! preflight 未过，退出。先完成 TC-002 停止流程再运行本脚本')
        return
    print('日志: %s' % log)
    print('!! 等回包窗（B3 裁定·二 2026-10-30）：A/B 阶段均固定 20ms，>20ms 才计 timeout；'
          '依据 F41 jsonl 全量复核（最大 14.4ms 的 1.4 倍），密集档实际发出间隔逐条如实记录')
    # min_gap_ms=0：A/B 是间隔实验；C 的间隔由 25ms 周期调度保证（见文件头）
    bus = Bus(args.port, log_path=log, min_gap_ms=0)
    try:
        bus.drain()
        print('-- 预检：ping J%d / J%d 各 2 次（间隔 50ms）' % (args.id_a, args.id_b))
        for sid in (args.id_a, args.id_b, args.id_a, args.id_b):
            f = bus.ping(sid)
            print('   ping J%d: %s' % (sid, f['verdict'] if f else 'timeout'))
            if f is None or f['verdict'] != 'ok':
                print('   !! 预检失败，终止（防在未知状态下密集查询）')
                return
            time.sleep(0.05)
        cyc_a = [int(x) for x in args.cycles.split(',')]
        cyc_b = [int(x) for x in args.cycles_b.split(',')]
        if args.phase in ('A', 'all'):
            phase_a(bus, args.id_a, cyc_a, args.n_cmd)
        if args.phase in ('B', 'all'):
            phase_b(bus, args.id_a, args.id_b, cyc_b, args.n_cmd)
        if args.phase in ('C', 'all'):
            phase_c(bus, args.n7, args.period_c)
        print('===== SUMMARY =====')
        print('SUMMARY 逐条数据与各档速率见日志 %s（判定与 Q4/Q6 决议由 执行方 离线分析给出，'
              '结论分支含「不可辨识/未观察到门限」，不作超范围归因）' % log)
        print('-- 收尾健康检查：ping J%d/J%d（间隔 50ms）' % (args.id_a, args.id_b))
        for sid in (args.id_a, args.id_b):
            f = bus.ping(sid)
            print('   ping J%d: %s' % (sid, f['verdict'] if f else 'timeout'))
            time.sleep(0.05)
    finally:
        bus.close()


if __name__ == '__main__':
    main()
