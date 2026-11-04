#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tc006_stop_sync.py —— TC-006 0x18 停止指令与 0x19 同步运动实测（P0.4，Q5 运动部分）

含小幅运动指令，只应针对 J6（夹爪，空载）运行；由 Thomas 在臂载计算机上执行。

复审要点（v4，第二轮清单落实）：
  - H-a：--stop-power 默认 6000 = 协议页示例值，本机语义待实测【未验证】，
    审查方 临时批准仅限 J6（2026-10-28）。轮次 = 0x11×2（power=0 与 6000 对比）
    + 0x12 + 0x10（均 6000）；实测出的 0x18 power 语义写入 FACTS；软急停取值留 P1.1
  - H-b（Thomas 已批准手拨）：每轮停止后、恢复前——①采一次 0x16 打印电流/功率；
    ②提示并 input() 等待「y=推得动 / n=推不动」，等待期间不发送任何帧；
    ③再采一次 0x16；④结果写入日志与 SUMMARY。判定：0x11 要求 n（推不动）、
    0x10 要求 y（推得动）、0x12 只记录不判；0x11 不得只凭漂移量判通过
  - H-c：恢复失败或同步去程未到位 → 立即中止并 safe_stop，不进入下一轮，
    不自行补发回位指令（H2/H3 的验收门保持）
  - M3：任何异常（含 Ctrl-C）→ safe_stop（重发 0x18×3 + 读角核实，如实报告）
  - H1：开串口前 preflight（4 服务 inactive + ttyS4 无占用）

序列：preflight → 只读基线 → 慢速归位 35°（验收）→
  [0x11@0mW, 0x11@6000mW, 0x12@6000mW, 0x10@6000mW] 每轮：
  35°→55° 运动中(1.2s) 0x18 → 停止判定 → 手拨（4 步）→ 恢复验收 →
  0x19 同步 35°↔45°（各段验收）→ 只读收尾。
"""

import argparse
import os
import time

from fsbus import Bus, fmt_monitor, preflight, STOP_HOLD, STOP_RELEASE, STOP_DAMPING


def read_ok(bus, sid):
    f = bus.read_angle(sid)
    return f['angle'] if f and f['verdict'] == 'ok' else None


def settle_read(bus, sid, n=8, interval=0.15, expect=None, label=''):
    """连续读角度；返回 (末角, 末3次波动, 是否到位)。expect=None 只读不判。"""
    angs = []
    for k in range(n):
        f = bus.read_angle(sid)
        if f and f['verdict'] == 'ok':
            angs.append(f['angle'])
        print('   %s读角 #%d: %s' % (label, k + 1,
              ('%+.1f°' % f['angle']) if f and f['verdict'] == 'ok'
              else (f['verdict'] if f else 'timeout')))
        time.sleep(interval)
    if not angs:
        return None, None, False
    last = angs[-1]
    spread = max(angs[-3:]) - min(angs[-3:]) if len(angs) >= 3 else float('nan')
    reached = (expect is None) or (abs(last - expect) <= 1.0 and spread <= 0.5)
    return last, spread, reached


def commanded_move(bus, sid, target, dps, label):
    """受控运动：读当前角 → 发 0x08 → 等待计算时长 → 到位验收（H2）。
    False = 读不到当前角或未到位——调用方必须中止（H-c），不得带未确认起点继续。"""
    cur = read_ok(bus, sid)
    if cur is None:
        print('   %s!! 读不到当前角度，中止本段' % label)
        return False
    itv = int(max(500.0, abs(target - cur) / dps * 1000.0))
    print('   %s0x08: %+.1f° → %+.1f°, %d ms (≈%.0f°/s)，等待运动完成 …'
          % (label, cur, target, itv, abs(target - cur) * 1000.0 / itv))
    bus.set_angle(sid, target, itv, 0)
    time.sleep(itv / 1000.0 + 0.3)
    last, spread, reached = settle_read(bus, sid, expect=target, label='[%s验收]' % label.strip('[]'))
    print('   %sOBS 到位: 末角 %s, 末3次波动 %s, 到位(±1.0°且波动≤0.5°)=%s'
          % (label, ('%+.1f°' % last) if last is not None else 'n/a',
             ('%.2f°' % spread) if spread == spread else 'n/a', reached))
    return reached


def push_phase(bus, sid):
    """手拨阶段（H-b，Thomas 已批准）：4 步协议。等待期间不发送任何帧。
    返回 movable ∈ {'y','n',其他}（原始记录，判定由调用方按模式套用）。"""
    f1 = bus.read_monitor(sid)
    print('   [手拨①] 拨前 0x16: %s' % (fmt_monitor(sid, f1) if f1 is not None else 'TIMEOUT'))
    ans = input('   [手拨②] 请轻拨夹爪，拨完输入 y=推得动 / n=推不动，回车：').strip().lower()
    f2 = bus.read_monitor(sid)
    print('   [手拨③] 拨后 0x16: %s' % (fmt_monitor(sid, f2) if f2 is not None else 'TIMEOUT'))
    bus.log('push_test', movable=ans,
            before=(f1['current_mA'], f1['power_mW']) if f1 is not None and f1['verdict'] == 'ok' else None,
            after=(f2['current_mA'], f2['power_mW']) if f2 is not None and f2['verdict'] == 'ok' else None)
    print('   [手拨④] 记录：拨动结果=%s（y=推得动/n=推不动），已写入日志' % ans)
    return ans


def stop_mode_test(bus, sid, mode, name, home, hi, stop_power):
    """一轮「运动中停止 + 手拨 + 恢复」。返回 (verdict, reason)；
    verdict ∈ pass / fail / undetermined。恢复失败时 raise（H-c：不得继续下一轮）。"""
    print('===== 0x18 %s @ %d mW：运动中停止测试 =====' % (name, stop_power))
    time.sleep(1.0)
    start = read_ok(bus, sid)
    if start is None:
        print('   OBS: 起点读数失败 → 不可判（本轮不判通过）')
        return 'undetermined', 'start_read_fail'
    bus.set_angle(sid, hi, 4000, 0)   # 35→55°，5°/s
    print('   已发 0x08: →%+.1f° / 4000ms (5°/s)，等 1.2s …' % hi)
    time.sleep(1.2)
    cur = read_ok(bus, sid)
    if cur is None:
        print('   OBS: 停止前读数失败 → 不可判；仍发 0x18（保守）')
        moving = False
    else:
        moving = abs(cur - home) >= 1.5
        print('   停止前读角: %+.1f°（离起点 %+.1f°，%s）'
              % (cur, cur - home, '已证实运动中' if moving else '未证实运动中(≥1.5°才可判)'))
    bus.stop(sid, mode, stop_power)
    print('   已发 0x18 mode=%s power=%dmW — 停止后读角 ×8（间隔 150ms，带时间戳）：'
          % ('0x%02x' % mode, stop_power))
    pts = []
    t0 = time.monotonic()
    for k in range(8):
        a = read_ok(bus, sid)
        if a is not None:
            pts.append((time.monotonic() - t0, a))
            print('      #%d: t=%.2fs %+.1f°' % (k + 1, pts[-1][0], a))
        else:
            print('      #%d: 读取失败' % (k + 1))
        time.sleep(0.15)
    stopped = None
    if len(pts) >= 4:
        (ta, aa), (tb, ab) = pts[0], pts[-1]
        slope = (ab - aa) / (tb - ta) if tb > ta else 0.0
        spread_after = max(a for _, a in pts[1:]) - min(a for _, a in pts[1:])
        hold_limit = 10.0 if mode == STOP_RELEASE else 0.5
        stopped = (abs(slope) <= 1.0) and (spread_after <= hold_limit)
        print('   OBS 停止: 斜率 %+.2f°/s（阈 ±1.0）, 停后极差 %.2f°（阈 %.1f°）→ %s'
              % (slope, spread_after, hold_limit, '停住' if stopped else '未停住'))
    else:
        print('   OBS 停止: 有效读数 %d 个（<4）→ 不可判' % len(pts))
    movable = push_phase(bus, sid)   # H-b：停止后、恢复前
    # 恢复：回到 home，要求存在可恢复的位移（H-c：失败即 raise 中止）
    time.sleep(0.5)
    pre = read_ok(bus, sid)
    displaced = pre is not None and abs(pre - home) >= 1.5
    print('   恢复前读角 %s → %s' % (('%+.1f°' % pre) if pre is not None else '读取失败',
                                     ('存在位移，执行恢复' if displaced else '无 ≥1.5° 位移，恢复不可判')))
    rec_ok = False
    if displaced:
        rec_ok = commanded_move(bus, sid, home, dps=5.0, label='[恢复]')
        if not rec_ok:
            print('   !! 恢复未到位 → 按 H-c 中止全部测试（safe_stop），不进入下一轮')
            raise RuntimeError('recovery_failed@%s@%dmW' % ('%02x' % mode, stop_power))
    # 判定（H3/H-b）：运动证实 + 停住 + 恢复 + 手拨模式符合性
    if not moving:
        verdict, reason = 'undetermined', 'motion_not_proven'
    elif stopped is None:
        verdict, reason = 'undetermined', 'insufficient_post_samples'
    elif not stopped:
        verdict, reason = 'fail', 'not_stopped'
    elif not displaced:
        verdict, reason = 'undetermined', 'no_displacement_to_recover'
    elif mode == STOP_HOLD and movable != 'n':
        if movable == 'y':
            verdict, reason = 'fail', 'push=y(保持锁力应推不动)'
        else:
            verdict, reason = 'undetermined', 'push=%s(未明确回答)' % movable
    elif mode == STOP_RELEASE and movable != 'y':
        if movable == 'n':
            verdict, reason = 'fail', 'push=n(释放应推得动)'
        else:
            verdict, reason = 'undetermined', 'push=%s(未明确回答)' % movable
    else:
        verdict, reason = 'pass', ('push=%s' % movable if mode != STOP_DAMPING else
                                   'push=%s(阻尼只记录不判)' % movable)
    print('   OBS 本轮结论: %s（%s）' % (verdict, reason))
    return verdict, reason


def main():
    ap = argparse.ArgumentParser(description='TC-006 0x18 停止 + 0x19 同步运动（含小幅运动）')
    ap.add_argument('--port', default='/dev/ttyS4')
    ap.add_argument('--id', type=int, default=6, help='目标关节（默认 J6 夹爪；空载）')
    ap.add_argument('--home', type=float, default=35.0, help='基准位（度）')
    ap.add_argument('--hi', type=float, default=55.0, help='停止测试远端位（度）')
    ap.add_argument('--mid', type=float, default=45.0, help='同步测试中位（度）')
    ap.add_argument('--stop-power', type=int, default=6000,
                    help='0x18 power_mW 临时取值（协议页示例值 6000，本机语义待实测【未验证】；'
                         '审查方 临时批准仅限 J6，2026-10-28）。0x11 另跑一轮 power=0 对比（H-a）；'
                         '官方协议页 §16 定义 power=0=按保护阈值执行')
    args = ap.parse_args()
    sid, home, hi, mid = args.id, args.home, args.hi, args.mid
    p = args.stop_power
    rounds = [(STOP_HOLD, '0x11 保持锁力', 0),      # H-a：保持模式两轮对比
              (STOP_HOLD, '0x11 保持锁力', p),
              (STOP_DAMPING, '0x12 阻尼', p),
              (STOP_RELEASE, '0x10 释放锁力', p)]

    ts = time.strftime('%Y%m%d_%H%M%S')
    log = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tc006_log_%s.jsonl' % ts)
    print('TC-006 0x18 停止 / 0x19 同步运动实测（板载时钟 %s，仅参考）' % ts)
    print('!! 本脚本会移动 J%d（%+.1f°↔%+.1f° 区间，速度 ≤10°/s），开始前确认：' % (sid, home, hi))
    print('!!   工作区无人无杂物、急停在手边、手机已开始录像（§6.1 运动类加录像）')
    print('!!   轮次：0x11@0mW、0x11@%dmW、0x12@%dmW、0x10@%dmW（H-a），每轮含手拨阶段（H-b）' % (p, p, p))
    print('!! Ctrl-C/异常中止：脚本走 safe_stop（重发 0x18×3 + 读角核实，结果如实打印）')
    print('!! 前置检查：4 服务 inactive + %s 无占用（复审 H1）' % args.port)
    ok_pf, msgs = preflight(args.port)
    for m in msgs:
        print('   preflight: %s' % m)
    if not ok_pf:
        print('   !! preflight 未过，退出。先完成 TC-002 停止流程再运行本脚本')
        return
    print('日志: %s' % log)
    bus = Bus(args.port, log_path=log)
    results = {}
    try:
        bus.drain()

        print('===== 步骤1/5：只读基线（0x16 ×5，间隔 200ms）=====')
        for k in range(5):
            f = bus.read_monitor(sid)
            print('   ' + fmt_monitor(sid, f))
            time.sleep(0.2)

        print('===== 步骤2/5：慢速归位 %+.1f°（等时长 + 到位验收，不过即中止）=====' % home)
        if not commanded_move(bus, sid, home, dps=10.0, label='[归位]'):
            print('   !! 归位未过验收，中止（safe_stop 收尾）')
            raise RuntimeError('home_move_not_verified')

        for mode, name, pw in rounds:
            v, r = stop_mode_test(bus, sid, mode, name, home, hi, pw)
            results['0x%02x@%dmW' % (mode, pw)] = (v, r)
            if v == 'fail' and r == 'not_stopped':
                # 复审（有条件通过·中）：判 not_stopped = 0x18 这一兜底中止手段
                # 本身不可信 → 立即中止剩余轮次与同步段；safe_stop 同样依赖
                # 0x18，其可信度存疑，如实说明并交 Thomas 目视处置
                print('   !! 停止指令未生效——0x18 兜底手段不可信，中止剩余轮次与同步段'
                      '（safe_stop 收尾；safe_stop 同样基于 0x18，可信度存疑）')
                raise RuntimeError('stop_not_effective@%s@%dmW' % ('%02x' % mode, pw))
            time.sleep(1.0)

        print('===== 步骤4/5：0x19 同步 0x08（n=1，仅 J%d，等时长 + 到位验收）=====' % sid)
        time.sleep(1.0)
        print('   同步帧: →%+.1f° / 1000ms，等待 1.3s …' % mid)
        bus.sync_set_angle([(sid, mid, 1000, 0)])
        time.sleep(1.3)
        last3, spread3, reached3 = settle_read(bus, sid, n=8, expect=mid, label='[同步去程]')
        print('   OBS 同步去程: 末角 %s, 到位(±1.0°)=%s' % (
              ('%+.1f°' % last3) if last3 is not None else 'n/a', reached3))
        rec_go = reached3 and last3 is not None
        rec_back = False
        if not rec_go:
            # H-c：去程未到位 → 中止，不补发回位、不进收尾运动段
            print('   !! 同步去程未到位 → 按 H-c 中止（safe_stop），不补发回位指令')
            raise RuntimeError('sync_go_not_verified')
        print('   同步帧: →%+.1f° / 1000ms（回程也走 0x19），等待 1.3s …' % home)
        bus.sync_set_angle([(sid, home, 1000, 0)])
        time.sleep(1.3)
        last4, spread4, reached4 = settle_read(bus, sid, n=8, expect=home, label='[同步回程]')
        print('   OBS 同步回程: 末角 %s, 到位=%s' % (
              ('%+.1f°' % last4) if last4 is not None else 'n/a', reached4))
        rec_back = reached4 and last4 is not None
        if not rec_back:
            print('   !! 同步回程未到位 → 按 H-c 中止（safe_stop）')
            raise RuntimeError('sync_back_not_verified')
        results['sync'] = ('pass' if (rec_go and rec_back) else 'fail', '')

        print('===== 步骤5/5：只读收尾（0x16 ×3）=====')
        for k in range(3):
            f = bus.read_monitor(sid)
            print('   ' + fmt_monitor(sid, f))
            time.sleep(0.2)

        print('===== SUMMARY =====')
        for k in list(results):
            v, r = results[k]
            print('SUMMARY %s: %s%s' % (k, v, ('（%s）' % r) if r else ''))
        print('SUMMARY 说明：pass=运动中下发且停住+手拨符合模式+恢复到位；'
              'undetermined=运动未证实/样本不足/无可恢复位移/手拨未答；fail=判据未达')
        print('SUMMARY 手拨判定（H-b）：0x11 应推不动(n)、0x10 应推得动(y)、0x12 只记录；'
              '各轮 y/n 与拨前/拨后电流功率均在日志 push_test 事件')
        print('SUMMARY 判定由 执行方 对照用例 TC-006 给出；录像路径登记进 tests/runs/')
    except (KeyboardInterrupt, Exception) as e:
        print('\n!! 异常/中断（%s）：safe_stop 收尾 ——' % repr(e))
        r = bus.safe_stop(sid, power_mw=p, mode=STOP_HOLD)
        print('   停止帧已尝试发送 %d 次；停后读角 %s → %s'
              % (r['sent'], r['angles'],
                 {True: '静止（末差 ≤0.5°）', False: '仍在变动', None: '读数不足无法判'}[r['still']]))
        if r['still'] is not True:
            print('   !! 未能确认已停住——请 Thomas 目视确认臂/夹爪状态，必要时急停（断电，F27）')
        print('   （中止后夹爪可能停在中间位置；恢复服务后是否归位由 Thomas 决定）')
        print('   已完成的轮次结论：')
        for k in list(results):
            v, r2 = results[k]
            print('   SUMMARY(部分) %s: %s%s' % (k, v, ('（%s）' % r2) if r2 else ''))
    finally:
        bus.close()


if __name__ == '__main__':
    main()
