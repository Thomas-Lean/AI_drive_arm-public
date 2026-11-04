#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tc008_sync_load.py —— TC-008 多轴同步到位测试（带载，P0.5-T2）

T4b（2026-11-03，审查方 复审 a–k 条，根因=J6 多圈漂移致 0x08 反向堵转过温，
见 tests/runs/2026-11-03-TC-009/）：k/k2 开跑圈数门（J6 超限→释放→0x11 清→确认；J0–J5 超限→
拒绝运动）；f/b 全轴 F7 限位只读检查；a/d/e/g J6 张开走验证（首动作 +5° 试探、读回 ≤2° 才算开，
失败即释放）；h 中止不再对 J6 发 0x08（堵转/未验证→0x18 释放；已验证停稳→保持夹持；已验证张开
→保持）；i 退出前打印 J6 终态；j 堵转保护（电流>阈值持续 2 s→释放，阈值 400 mA 为临时值待审定）；
每运动段前 mturn_assert 原始多圈复查；示教等候期间每 0.5 s 读 J6 堵转检查。

要回答的问题（Thomas 2026-11-02 派单）：遥操作时 6 个关节同时动、夹着 330 ml 瓶（F55，
378 g）时各轴能不能同时到位、最终误差多大，J1/J2/J4 带载后是否明显落后。
本脚本只出数据表（每段 × 每轴），不写结论（§2.1 执行方 口径）。

流程（只做带载，不做空载）：
  ①放瓶姿态：Thomas 扶臂摆好 → 回车 → 锁 7 轴 → 读姿态 → S1 下垂检查
  ②夹瓶：J6 张开到 64.5 → 放瓶（瓶身 h=108 mm 处，F55）→ 回车 → J6 关闭到 29.0（F56）
    → 停稳后 0x16 连读 1 s + 0x0A ×5 全部记录 → 稳定值（0x16 中位）须落在 [33.0, 34.0]
    且极差 ≤0.5°，否则中止。F56 首次上实机，原始读数全记（jsonl + segments JSON）。
  ③示教 A、B（与 TC-007 同款：H1 扶住确认 → 只释放 J0–J5 → 摆位 → 回车 → 先锁 J0–J5
    再读 7 轴 → S1 下垂检查 → 夹持核查 J6 仍在窗内）。J6 全程不发 0x18（保持 0x08 目标 29
    的夹持力；示教锁只锁 J0–J5）。
  ④时长两档（Thomas 2026-11-02 定）：慢档 T_slow = 最大行程 ÷ 30°/s；中档 T_mid = 各轴
    都不超过 URDF 速度（J0–J2 60.2°/s、J3–J5 99.1°/s，tc007 AXIS_SPEEDS）的最短时长；
    下限 500 ms。运行时打印请 Thomas 确认后才动。
  ⑤H3 慢速归位到 A（0x19 同步、≤20°/s），0x0A 读角验收 ≤1°，不过即中止。
  ⑥每档 A→B→A 往返 3 次：0x19 同步 0x08 同时下发 J0–J5（同一 T），全程 0x19+0x16
    同步监控 7 轴（F46）至 T+1.0 s；J6 不下发运动。
  ⑦收尾：Thomas 扶住瓶子 → 回车 → J6 张开 → 锁 7 轴 → 打印恢复服务提示。
中止条件（任一触发 → J0–J5 safe_stop_all 保持锁力并如实报告）：J6 连续 --j6-out-frames 帧
（默认 3，Thomas 2026-11-02 定）离开窗口 [33, 34]（掉瓶/打滑）、J6 连续 5 帧不可读（守卫
失明）、电压 < --vbat-low（默认 11000 mV，沿用 TC-007）、0x16 保护位（bit2–7）、Ctrl-C、
任何异常。**已夹瓶时 J6 不发 0x18**（F48：0x18 按当前位置保持 → 夹持位置误差归零 → 夹持力
可能消失，M1 复审）：改为重发一次 0x08 目标 29.0 保持夹持并读角如实上报；未夹瓶/已张开则
J6 并入 safe_stop_all（7 轴）。

数据输出：
  tc008_frames_<ts>.csv    每帧一行：t_tx_s、段号(gear/trip/leg)、frame_ok、n_got、
                           7 轴 × (ang, mA, mV, mW, st, tC, dt_ms)
  tc008_segments_<ts>.json 姿态 A/B、夹瓶原始读数、两档时长、每段 (t_cmd, T, targets)、统计
  tc008_log_<ts>.jsonl     fsbus 总线事件日志（tx/rx/sync_mon/lock_all/…）
统计口径（写死，同一处给定义，R3）：
  到位时刻 = 该轴首次进入目标 ±1.0° 且此后至段末（T+1.0 s）一直保持的采样时刻（相对下发时刻）；
            分辨率 = 帧间隔（随表打印）；未到位/未保持 → 「未到位」并给出最后出窗时刻
  6 轴到位极差 = 6 轴到位时刻 max − min（任一轴未到位 → 不可判）
  最终误差 = 段末 0.5 s（T+0.5 s ~ T+1.0 s）内 0x16 角度均值 − 目标
  电流峰值 / 电压最低 = [0, T]（指令时长内）各轴 0x16 读数的 max / min
复算（R6）：python3 tc008_sync_load.py --analyze tc008_frames_<ts>.csv tc008_segments_<ts>.json
"""

import argparse
import csv
import json
import os
import sys
import time
import types

# --analyze 离线复算不需要 pyserial：缺失时注入桩（与 fsbus_selftest 同法）
try:
    import serial  # noqa: F401
except ImportError:
    _stub = types.ModuleType('serial')
    _stub.EIGHTBITS, _stub.PARITY_NONE, _stub.STOPBITS_ONE, _stub.Serial = 8, 'N', 1, None
    sys.modules['serial'] = _stub

from fsbus import (Bus, preflight, STOP_HOLD, STOP_RELEASE,  # noqa: E402
                   KNOWN_STATUS_MASK, norm_single_turn, read_mturn,
                   mturn_gate, mturn_assert, MturnRefused, JOINT_LIMITS)
from tc007_stream import (read_ang, read_all, sag_check, confirm_yes,  # noqa: E402
                          AXIS_SPEEDS, HOME_SPEED, DEFAULT_HOLD_POWER)

ALL_IDS = tuple(range(7))
MOVE_IDS = (0, 1, 2, 3, 4, 5)
GRIP_ID = 6

GRIP_OPEN = 64.5            # F56 张开 angle6_raw
GRIP_CLOSE = 29.0           # F56 关闭目标
# 旧 grip_detect 阈值（百岁山最终方案 GRASP-GRIP-SETTING-20261027；F57 作废 F56 窗 [33,34]；
# T6 起夹瓶判定与运动/静止守卫基准同用本组：tc009 同名常量自本版起改为从本文件 import，保证同值）
OLD_EMPTY_ABS = 1.0        # |停住−29.0| ≤1.0° → empty（未夹到）
OLD_GRASP_MIN = 2.0        # 停住−29.0 ≥+2.0° 且停稳 → grasped（夹到）；其间 uncertain
# T6b：keep_grip 中止退出后 J6 仍 0x08→29 持续出力（约 240 mA 顶瓶）且脚本不再监控——给
# Thomas 的可直接复制运行释放命令（0x18 0x10 按当前位置失力，power=0，与位置/多圈无关安全）。
# 发帧后 read_ang 读回一次确认。shell 外层双引号、内层单引号，可整行粘贴。
J6_RELEASE_CMD = ("python3 -c \"from fsbus import Bus, STOP_RELEASE; from tc007_stream import read_ang; "
                  "b=Bus('/dev/ttyS4'); b.stop(6, STOP_RELEASE, 0); "
                  "print('J6 释放帧已发(0x18 0x10)'); print('J6 =', read_ang(b, 6))\"")
GRIP_SPEED = 20.0           # J6 张开/关闭速度上限 °/s（与 H3 归位同档，慢）
GRIP_SETTLE_S = 0.5         # 关闭指令时长结束后再等多久才开始读稳定值
GRIP_READ_S = 1.0           # 稳定值读取窗（0x16 连读）
GRIP_STABLE_SPREAD = 0.5    # 稳定判据：读取窗内 0x16 角度极差 ≤ 此值
GRIP_STATIC_N = 5           # 每次示教锁定后夹持核查的 0x16 读数条数

SLOW_SPEED = 30.0           # 慢档：最大行程 ÷ 30°/s
T_FLOOR_MS = 500            # 时长下限
TOL_ARRIVE = 1.0            # 到位判据 ±1.0°
HOME_TOL = 3.0              # H3 归位验收 ≤3.0°（Thomas 2026-11-03：沿用旧项目 arm_driver.tolerance_deg=3°，F16；带载重力静差 J2/J3 达 1.1–1.6°）
SETTLE_S = 1.0              # 每段指令时长结束后继续采样的时间
FINAL_WIN_S = 0.5           # 最终误差 = 段末 0.5 s 均值 − 目标
MON_TIMEOUT = 0.1           # 0x19+0x16 7 轴收齐等待上限（F46：静止 max 33.9 ms；运动中 rtt 变慢 F52）
J6_MISS_LIMIT = 5           # J6 连续不可读帧数 → 守卫失明 → 中止
J6_OUT_FRAMES_DEFAULT = 3   # 运动中 J6 连续出窗多少帧即中止（Thomas 2026-11-02 定）
J6_OPEN_TOL = 2.0           # a/g：J6 张开验证 |读数−64.5| ≤ 此值
CLOSE_MS_S = 1.775           # 与 tc009 CLOSE_MS 同源（64.5→29 @20°/s）；张开首检窗用
J6_STALL_MA = 400           # j：J6 堵转电流阈 mA【临时值，待 审查方 审定——2026-11-02 实测健康夹持
                            # 停稳电流 541–557 mA 高于此值，按此默认夹瓶段会 2 s 内触发；--j6-stall-ma 可覆盖】
J6_STALL_S = 2.0            # j：持续超阈时长
GEAR_WORD = {'home': '归位', 'slow': '慢档', 'mid': '中档'}
LEG_WORD = {'home': '归位', 'go': '去程', 'back': '回程'}

AX_COLS = ('ang', 'mA', 'mV', 'mW', 'st', 'tC', 'dt_ms')
CSV_HEADER = (['t_tx_s', 'gear', 'trip', 'leg', 'frame_ok', 'n_got']
              + ['J%d_%s' % (s, c) for s in ALL_IDS for c in AX_COLS])


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if not n:
        return None
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def fmt(v, f='%.2f', none='—'):
    return none if v is None else (f % v)


def pad(s, n):
    """按显示宽度补空格（CJK 计 2），矩阵行名对齐用"""
    w = sum(2 if ord(c) > 0x2E7F else 1 for c in s)
    return s + ' ' * max(0, n - w)


class Ctx(object):
    """运行上下文：帧缓存 + CSV 即时落盘 + 守卫状态"""

    def __init__(self, csv_path, vbat_low, grip_dev, j6_out_frames):
        self.frames = []
        self.vbat_low = vbat_low
        self.grip_dev = grip_dev   # T6：运动/静止守卫窗 = grip_ref ± grip_dev
        self.grip_ref = None       # T6：夹瓶判定通过（grasped）后 = 停稳中位；未设前不启用出窗守卫
        self.j6_out_frames = j6_out_frames
        self.j6_miss = 0
        self.j6_out = 0
        self.grip_active = False   # 关闭指令已发且尚未收尾张开
        self.grip_verified = False  # 夹持已验证停稳（h：中止保留夹持的前提）
        self.j6_verified_open = False  # 已验证张开（h：中止允许保持的前提）
        self.j6_probed = False     # d：本会话 J6 小步试探已做过
        self.j6_release_now = False  # j 堵转触发：中止时必须释放
        self.j6_stall_since = None
        self.j6_state = '未知'     # i：退出时打印的 J6 状态
        self.fh = open(csv_path, 'w', newline='') if csv_path else None
        self.w = csv.writer(self.fh) if self.fh else None
        if self.w:
            self.w.writerow(CSV_HEADER)

    def add_frame(self, fr):
        self.frames.append(fr)
        if not self.w:
            return
        row = ['%.4f' % fr['t_tx'], fr['gear'], fr['trip'], fr['leg'],
               1 if fr['ok'] else 0, len(fr['got'])]
        for s in ALL_IDS:
            d = fr['got'].get(s)
            if d is None:
                row += [''] * len(AX_COLS)
                continue
            row += ['' if d['ang'] is None else '%.2f' % d['ang'], d['mA'], d['mV'], d['mW'],
                    d['st'], '' if d['tC'] is None else '%.1f' % d['tC'], '%.1f' % d['dt_ms']]
        self.w.writerow(row)

    def flush(self):
        if self.fh:
            self.fh.flush()

    def close(self):
        if self.fh:
            self.fh.close()
            self.fh = None


# ---------- 守卫（关键代码·交 审查方 审） ----------

def j6_hold_guard(ctx, m, tag):
    """S4（v2 口径）：J6 读数守卫——夹持保持期间（grip_active，tc008）用温度 ≥55℃ 触发
    正常中止；非夹持期间用 400mA/2s 堵转规则（临时值）。m 为 read_mturn 结果。"""
    if m is None:
        return
    if ctx.grip_active:
        if m['tC'] is not None and m['tC'] >= 55.0:
            raise RuntimeError('j6_overtemp@%s(%.1f℃≥55，走正常中止)' % (tag, m['tC']))
    else:
        j6_stall_track(ctx, m['mA'], tag)


def j6_stall_track(ctx, mA, tag):
    """j：J6 电流持续 > J6_STALL_MA 超 J6_STALL_S → 置 j6_release_now 并 raise。
    阈值为临时值（见常量注释），--j6-stall-ma 可覆盖。"""
    now = time.monotonic()
    if mA is not None and mA > ctx.j6_stall_ma:
        if ctx.j6_stall_since is None:
            ctx.j6_stall_since = now
        elif now - ctx.j6_stall_since >= J6_STALL_S:
            ctx.j6_release_now = True
            raise RuntimeError('j6_stall_release@%s(J6 %dmA>%d 持续%.1fs)'
                               % (tag, mA, ctx.j6_stall_ma, now - ctx.j6_stall_since))
    else:
        ctx.j6_stall_since = None


def input_j6guard(bus, ctx, prompt):
    """j：等待回车期间每 0.5 s 只读一次 J6 0x16 并做堵转检查（select 轮询 stdin，
    主线程不并发用总线）。触发即 raise（由 main 收尾释放）。"""
    import select
    sys.stdout.write(prompt)
    sys.stdout.flush()
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.5)
        if r:
            line = sys.stdin.readline()
            if line == '':
                raise EOFError('stdin EOF')
            return line
        m = read_mturn(bus, GRIP_ID)
        j6_hold_guard(ctx, m, 'input_wait')


def j6_open_verified(bus, ctx, args, tag):
    """a/d/e/g：J6 张开到 64.5 并验证。d：本会话首次动作先小步试探（往张开方向 +5°、
    500 ms ≤10°/s），读回确认动了才继续；g：整段张开后在（时长+0.5 s）内读回，
    |读数−64.5|≤J6_OPEN_TOL 算到位。任一不满足 → 立即 0x18 释放 J6 并 raise。"""
    m = read_mturn(bus, GRIP_ID)
    cur = m['angle'] if m else None
    if cur is None:
        raise RuntimeError('j6_read_fail@open@%s' % tag)
    print('   [J6张开@%s] 当前 %.1f°' % (tag, cur))
    if abs(cur - GRIP_OPEN) > J6_OPEN_TOL:
        if not ctx.j6_probed:
            step = 5.0 if GRIP_OPEN > cur else -5.0
            tgt = max(-30.0, min(64.5, cur + step))
            need = 3.0 if abs(tgt - cur) >= 4.0 else 0.6 * abs(tgt - cur)   # S1：步长被限位截短(<4°)时按 0.6×步长判（T6d 修正 T4b3 的 max() 写法——053150 现场 2.6° 步长被判失败）
            bus.set_angle(GRIP_ID, tgt, 500, args.grip_power)   # 5°/500ms = 10°/s
            time.sleep(0.7)
            m2 = read_mturn(bus, GRIP_ID)
            moved = (m2['angle'] - cur) if m2 else None
            ok_probe = (moved is not None and abs(moved) >= need
                        and (moved > 0) == (tgt > cur))
            print('   [J6试探@%s] %.1f → %s（目标 %+.1f，判过 位移≥%.1f° 且方向正确）' % (
                tag, cur, fmt(moved, '%+.1f', '读不到'), tgt, need))
            if not ok_probe:
                bus.stop(GRIP_ID, STOP_RELEASE, 0)
                ctx.j6_state = '已释放(0x18 0x10，试探失败)'
                raise RuntimeError('j6_probe_fail@%s(读数 %s，已释放)' % (tag, fmt(moved, '%+.1f', '读不到')))
            ctx.j6_probed = True
            cur = m2['angle']
        ms = int(max(T_FLOOR_MS, abs(GRIP_OPEN - cur) / GRIP_SPEED * 1000.0))
        t_send = time.monotonic()
        bus.set_angle(GRIP_ID, GRIP_OPEN, ms, args.grip_power)
        # S3+时限（T4b v2）：CLOSE_MS+0.3 s 首检——未到位且相对出发位移 <3°（没在动）→ 立即释放
        #（事故场景下自发出至释放 ≤2.3 s）；在动 → 等满 ms+0.5 s 终检
        time.sleep(max(0.0, t_send + CLOSE_MS_S + 0.3 - time.monotonic()))
        m3 = read_mturn(bus, GRIP_ID)
        ok = m3 is not None and abs(m3['angle'] - GRIP_OPEN) <= J6_OPEN_TOL
        if not ok and (m3 is None or abs(m3['angle'] - cur) < 3.0):
            bus.stop(GRIP_ID, STOP_RELEASE, 0)
            ctx.j6_state = '已释放(0x18 0x10，张开未动/未验证)'
            raise RuntimeError('j6_open_stalled@%s(读数 %s 未动，已释放)'
                               % (tag, fmt(m3['angle'] if m3 else None, '%.1f', '读不到')))
        if not ok:
            time.sleep(max(0.0, t_send + ms / 1000.0 + 0.5 - time.monotonic()))
            m3 = read_mturn(bus, GRIP_ID)
            ok = m3 is not None and abs(m3['angle'] - GRIP_OPEN) <= J6_OPEN_TOL
        print('   [J6张开@%s] 读回 %s / 目标 %.1f（判过 ≤%.1f°）→ %s'
              % (tag, fmt(m3['angle'] if m3 else None, '%.1f', '读不到'), GRIP_OPEN,
                 J6_OPEN_TOL, '通过' if ok else '未过'))
        if not ok:
            bus.stop(GRIP_ID, STOP_RELEASE, 0)
            ctx.j6_state = '已释放(0x18 0x10，张开未验证)'
            raise RuntimeError('j6_open_not_verified@%s(已释放)' % tag)
    ctx.j6_verified_open = True
    ctx.grip_active = False
    ctx.j6_state = '已验证静止、未堵转（张开 %.1f°）' % GRIP_OPEN
    return read_ang(bus, GRIP_ID)


def pose_f7_check(pose, tag):
    """b/f：任一轴 0x0A 读数超出 F7 限位 → 报警中止（今晨 J6=-42.4 案的补丁）。"""
    bad = [(sid, pose[sid], JOINT_LIMITS[sid]) for sid in ALL_IDS
           if not (JOINT_LIMITS[sid][0] - 0.5 <= pose[sid] <= JOINT_LIMITS[sid][1] + 0.5)]
    if bad:
        raise RuntimeError('pose_out_of_f7@%s(%s)' % (
            tag, ['J%d %.1f∉[%s,%s]' % (b[0], b[1], b[2][0], b[2][1]) for b in bad]))
    return bad


def j6_abort_action(ctx):
    """h：中止时 J6 处置——j6_release_now(堵转/未验证)→释放；已验证夹持(停稳)→保持 0x08 夹持；
    已验证张开→保持；其余→释放。返回动作字符串。"""
    if ctx.j6_release_now:
        return 'release'
    if ctx.grip_active:
        return 'keep_grip' if ctx.grip_verified else 'release'
    return 'keep_open' if ctx.j6_verified_open else 'release'

def guard_monitor(d, vbat_low):
    """单条 0x16 解码结果的电压/保护位守卫：返回中止原因或 None（沿用 TC-007 口径）"""
    if d['voltage_mV'] < vbat_low:
        return 'vbat_low(J%d %dmV<%d)' % (d['servo_id'], d['voltage_mV'], vbat_low)
    if d['status'] & KNOWN_STATUS_MASK:
        return 'status_protect(J%d 0x%02x)' % (d['servo_id'], d['status'])
    return None


def guard_frame(fr, ctx):
    """每帧守卫（运动段内每一帧都过）：
    ① 任一轴电压 < vbat_low / 出保护位 → raise
    ② J6 不在帧内或为哨兵 → 连续计数，≥J6_MISS_LIMIT → raise（守卫失明不允许继续）
    ③ J6 角度（单圈归一）离开 [grip_ref−dev, grip_ref+dev] → 连续计数，≥j6_out_frames →
       raise（掉瓶/打滑）；grip_ref 未设（夹瓶判定前）不启用本条（T6）"""
    for sid, d in fr['got'].items():
        if d['mV'] < ctx.vbat_low:
            raise RuntimeError('vbat_low(J%d %dmV<%d)' % (sid, d['mV'], ctx.vbat_low))
        if d['st'] & KNOWN_STATUS_MASK:
            raise RuntimeError('status_protect(J%d 0x%02x)' % (sid, d['st']))
    d6 = fr['got'].get(GRIP_ID)
    if d6 is None or d6['ang'] is None:
        ctx.j6_miss += 1
        if ctx.j6_miss >= J6_MISS_LIMIT:
            raise RuntimeError('j6_unreadable(连续%d帧无 J6 有效读数)' % ctx.j6_miss)
        return
    ctx.j6_miss = 0
    if ctx.grip_verified:
        if d6['tC'] is not None and d6['tC'] >= 55.0:   # S4：夹持保持 → 温度规则
            raise RuntimeError('j6_overtemp@motion(J6 %.1f℃≥55，走正常中止)' % d6['tC'])
    else:
        j6_stall_track(ctx, d6['mA'], 'motion_frame')
    a = norm_single_turn(d6['ang'])
    if ctx.grip_ref is not None:   # T6：夹瓶判定通过前无基准，不启用出窗守卫
        lo, hi = ctx.grip_ref - ctx.grip_dev, ctx.grip_ref + ctx.grip_dev
        if a < lo or a > hi:
            ctx.j6_out += 1
            if ctx.j6_out >= ctx.j6_out_frames:
                raise RuntimeError('j6_out_of_window(J6=%.2f°∉[%.1f,%.1f] 连续%d帧)'
                                   % (a, lo, hi, ctx.j6_out))
        else:
            ctx.j6_out = 0


# ---------- 夹瓶（关键代码·交 审查方 审） ----------

def grip_clamp(bus, ctx, args, doc):
    """②夹瓶：张开 → 放瓶回车 → 关闭 → 稳定读（0x16 连读 + 0x0A ×5 全记）→ 旧阈值判定（T6）。
    判定用 0x16 中位（与运动中守卫同源，单一口径）：med−29.0 ≥ +OLD_GRASP_MIN 且停稳
    （极差 ≤ GRIP_STABLE_SPREAD）→ grasped 通过，记 ctx.grip_ref = med；|med−29.0| ≤
    OLD_EMPTY_ABS → raise grip_empty；其余 → raise grip_uncertain。旧阈值 = 百岁山最终方案
    （GRASP-GRIP-SETTING-20261027；F57 作废 F56 窗 [33,34]）。0x0A 为 F56 原口径（旧控制器
    反馈通路同款指令）仅记录对照。失败 → raise（main 走 safe_stop_all）。"""
    a_open = j6_open_verified(bus, ctx, args, '夹瓶')
    input('   [夹瓶] 请将瓶身 h=108 mm 处放入两指之间（瓶轴与开合方向垂直，F55），'
          '扶住瓶子，按回车后 J6 关闭到 %.1f：' % GRIP_CLOSE)
    ms2 = int(max(T_FLOOR_MS, abs(GRIP_OPEN - GRIP_CLOSE) / GRIP_SPEED * 1000.0))
    t_close = time.monotonic()
    bus.set_angle(GRIP_ID, GRIP_CLOSE, ms2, args.grip_power)
    ctx.grip_active = True  # M1：自此至收尾张开，中止路径对 J6 保持 0x08、不发 0x18
    bus.log('grip_close', target=GRIP_CLOSE, ms=ms2, power=args.grip_power)
    print('   [夹瓶] 关闭指令已发（%dms），等 %.1fs 后连读 %.1fs……' % (ms2, GRIP_SETTLE_S, GRIP_READ_S))
    time.sleep(ms2 / 1000.0 + GRIP_SETTLE_S)
    r16 = []
    t_end = time.monotonic() + GRIP_READ_S
    while time.monotonic() < t_end:
        f = bus.read_monitor(GRIP_ID, timeout=0.05)
        if f is not None and f['verdict'] == 'ok':
            why = guard_monitor(f, ctx.vbat_low)
            if why:
                raise RuntimeError(why)
            if f['valid_measure']:
                j6_hold_guard(ctx, {'tC': f['temp_c'], 'mA': f['current_mA']}, 'clamp_read')
                r16.append({'t': round(time.monotonic() - bus.t0, 4),
                            'ang': round(norm_single_turn(f['angle']), 2),
                            'mA': f['current_mA'], 'mV': f['voltage_mV'], 'st': f['status']})
    r0a = [read_ang(bus, GRIP_ID) for _ in range(5)]
    angs = [r['ang'] for r in r16]
    print('   [夹瓶] 0x16 读数 ×%d（角度）：%s' % (len(angs), angs))
    print('   [夹瓶] 0x16 电流 mA：%s' % [r['mA'] for r in r16])
    print('   [夹瓶] 0x0A 读数 ×5（F56 原口径对照）：%s' % r0a)
    rec = {'t_close': round(t_close - bus.t0, 4), 'close_ms': ms2, 'open_read_0a': a_open,
           'r16': r16, 'r0a': r0a, 'stable_spread_max': GRIP_STABLE_SPREAD,
           'old_empty_abs': OLD_EMPTY_ABS, 'old_grasp_min': OLD_GRASP_MIN}
    if len(angs) < 3:
        rec['verdict'] = 'read_fail'
        doc['grip'] = rec
        bus.log('grip_check', **rec)
        raise RuntimeError('grip_read_fail@close(0x16 有效 %d 条<3)' % len(angs))
    med, spread = median(angs), max(angs) - min(angs)
    old_err = med - GRIP_CLOSE
    stable = spread <= GRIP_STABLE_SPREAD
    verdict = ('pass' if (old_err >= OLD_GRASP_MIN and stable)
               else 'empty' if abs(old_err) <= OLD_EMPTY_ABS else 'uncertain')
    rec.update({'median': round(med, 2), 'spread': round(spread, 2), 'old_err': round(old_err, 2),
                'stable': stable, 'verdict': verdict})
    doc['grip'] = rec
    bus.log('grip_check', **rec)
    print('   [夹瓶] 稳定值(0x16 中位)=%.2f° 极差=%.2f°（n=%d，停稳判据 极差≤%.1f）'
          '旧误差 med−%.1f=%+.2f → 判定：%s（旧阈值：百岁山最终方案 GRASP-GRIP-SETTING-20261027）'
          % (med, spread, len(angs), GRIP_STABLE_SPREAD, GRIP_CLOSE, old_err,
             {'pass': '通过（grasped）', 'empty': '未过（empty）', 'uncertain': '未过（uncertain）'}[verdict]))
    if verdict == 'empty':
        raise RuntimeError('grip_empty(med=%.2f |med−%.1f|=%.2f≤%.1f 未夹到瓶)'
                           % (med, GRIP_CLOSE, abs(old_err), OLD_EMPTY_ABS))
    if verdict == 'uncertain':
        raise RuntimeError('grip_uncertain(med=%.2f err=%+.2f 在 (%.1f,%.1f) 间或未停稳 spread=%.2f)'
                           % (med, old_err, OLD_EMPTY_ABS, OLD_GRASP_MIN, spread))
    ctx.grip_ref = med                # T6：运动/静止守卫基准 = 停稳中位
    doc['grip_ref'] = round(med, 2)
    doc['grip_dev'] = args.grip_dev
    print('   [夹瓶] grip_ref=%.2f，守卫窗 [%.1f,%.1f]（±dev=%.1f，--grip-dev 可改；dev 暂定待 TC-009 v2）'
          % (med, med - args.grip_dev, med + args.grip_dev, args.grip_dev))
    ctx.grip_verified = True          # h：已验证停稳 → 中止时可保留 M1 夹持
    ctx.j6_verified_open = False
    ctx.j6_state = '已验证静止、未堵转（夹持停稳 %.2f°）' % med
    return rec


def grip_static_check(bus, ctx, tag):
    """示教锁定后的夹持核查：J6 0x16 ×GRIP_STATIC_N 全部须在窗内（Thomas 手摆期间瓶子
    若打滑在此拦下，不带着松瓶进入运动段）"""
    vals = []
    for _ in range(GRIP_STATIC_N):
        f = bus.read_monitor(GRIP_ID, timeout=0.05)
        if f is not None and f['verdict'] == 'ok':
            why = guard_monitor(f, ctx.vbat_low)
            if why:
                raise RuntimeError(why)
            if f['valid_measure']:
                j6_hold_guard(ctx, {'tC': f['temp_c'], 'mA': f['current_mA']}, 'static_check')
                vals.append(round(norm_single_turn(f['angle']), 2))
        time.sleep(0.02)
    if ctx.grip_ref is None:            # T6：夹瓶判定通过前无基准，不启用窗口核查（只记录）
        out = []
        win_txt = '无基准（grip_ref 未设，不核查窗口）'
    else:
        lo, hi = ctx.grip_ref - ctx.grip_dev, ctx.grip_ref + ctx.grip_dev
        out = [v for v in vals if v < lo or v > hi]
        win_txt = '窗 [%.1f,%.1f]' % (lo, hi)
    bus.log('grip_static', tag=tag, vals=vals, out=out, grip_ref=ctx.grip_ref)
    if len(vals) < 3:
        raise RuntimeError('grip_read_fail@%s(0x16 有效 %d 条<3)' % (tag, len(vals)))
    print('   [夹持核查@%s] J6 0x16 ×%d：%s %s → %s' % (
        tag, len(vals), vals, win_txt, '全部在窗内' if not out else ('出窗 %s' % out)))
    if out:
        raise RuntimeError('j6_out_of_window@%s(%s)' % (tag, out))
    return vals


# ---------- 示教 ----------

def teach_pose(bus, ctx, label, pw, release_first, grip_check=True, wait=None):
    """示教一个姿态：H1 扶住确认 → 只释放 J0–J5（J6 不动）→ 摆位回车 → 先锁 J0–J5 再读 7 轴
    → S1 下垂检查 → 夹持核查。返回 7 轴姿态 dict。
    grip_check=False（TC-009 复用）：J6 张开无瓶，跳过夹持核查、提示语不提瓶；ctx 可为 None。"""
    note = '（瓶子已夹住）' if grip_check else '（J6 已张开、无瓶）'
    hold = '请夹着瓶、扶住臂' if grip_check else '请扶住臂'
    s1 = '，瓶子仍请托着' if grip_check else ''
    ask = wait if wait is not None else input
    if release_first:
        ask('   [H1] 请扶住整臂%s，按回车释放 J0–J5（J6 不释放）：' % note)
        bus.lock_all(MOVE_IDS, pw, mode=STOP_RELEASE)
        time.sleep(0.3)
    ask('   [姿态%s] %s用手摆到姿态 %s，摆好后扶住按回车（先锁 J0–J5 再读）：'
        % (label, hold, label))
    sent = bus.lock_all(MOVE_IDS, pw, mode=STOP_HOLD)
    if sent != len(MOVE_IDS):
        print('   !! lock_all 仅发出 %d/%d，重试……' % (sent, len(MOVE_IDS)))
        if bus.lock_all(MOVE_IDS, pw, mode=STOP_HOLD) + sent < len(MOVE_IDS):
            raise RuntimeError('lock_all_failed@%s' % label)
    time.sleep(0.2)
    pose = read_all(bus)
    if pose is None:
        raise RuntimeError('pose_read_fail@%s' % label)
    pose_f7_check(pose, label)      # b：任一轴越 F7 限位 → 中止
    print('   [姿态%s] 已锁定并记录（F7 限位内）：%s' % (label, json.dumps(pose, ensure_ascii=False)))
    ask('   [S1] 请慢慢松手（臂由舵机保持%s），按回车开始下垂检查：' % s1)
    sag = sag_check(bus, pose, label)
    if grip_check:
        grip_static_check(bus, ctx, label)
    return pose, sag


# ---------- 时长 ----------

def compute_T(A, B):
    travel = {s: abs(B[s] - A[s]) for s in MOVE_IDS}
    mx_s = max(MOVE_IDS, key=lambda s: travel[s])
    mx = travel[mx_s]
    t_slow = int(max(T_FLOOR_MS, mx / SLOW_SPEED * 1000.0))
    t_mid = int(max(T_FLOOR_MS, max(travel[s] / AXIS_SPEEDS[s] for s in MOVE_IDS) * 1000.0))
    t_home = int(max(T_FLOOR_MS, mx / HOME_SPEED * 1000.0))
    return travel, mx_s, t_slow, t_mid, t_home


def print_T(travel, mx_s, t_slow, t_mid, t_home):
    print('   [时长] 行程(°)：%s；最大 %.1f°（J%d）'
          % (' / '.join('J%d %.1f' % (s, travel[s]) for s in MOVE_IDS), travel[mx_s], mx_s))
    print('   [时长] 慢档 T_slow = %d ms（= 最大行程 ÷ %.0f°/s，下限 %d ms）→ 各轴速度 °/s：%s'
          % (t_slow, SLOW_SPEED, T_FLOOR_MS,
             ' / '.join('J%d %.1f' % (s, travel[s] * 1000.0 / t_slow) for s in MOVE_IDS)))
    print('   [时长] 中档 T_mid = %d ms（= max_i 行程_i ÷ URDF_i，J0–J2 %.1f / J3–J5 %.1f °/s，下限 %d ms）'
          '→ 各轴速度 °/s：%s'
          % (t_mid, AXIS_SPEEDS[0], AXIS_SPEEDS[3], T_FLOOR_MS,
             ' / '.join('J%d %.1f(≤%.1f)' % (s, travel[s] * 1000.0 / t_mid, AXIS_SPEEDS[s])
                        for s in MOVE_IDS)))
    print('   [时长] 归位 T_home = %d ms（= 最大行程 ÷ %.0f°/s，H3）' % (t_home, HOME_SPEED))


# ---------- 运动段（关键代码·交 审查方 审：0x19 同步下发 + 逐帧守卫） ----------

def run_leg(bus, ctx, segs, targets, T_ms, gear, trip, leg):
    """一段运动：0x19 同步 0x08 一帧下发 J0–J5 目标（同一 T，power=0），随后 0x19+0x16
    同步监控 7 轴直到 T+SETTLE_S；每帧记入 ctx（CSV 即时落盘）并过 guard_frame。
    J6 不在下发列表内（保持夹紧）。段描述在下发后立即登记进 segs（中止时该段已采帧也进统计）。"""
    mturn_assert(bus, ALL_IDS, '%s/%d/%s' % (gear, trip, leg))   # k：运动段前原始多圈复查
    items = [(s, targets[s], T_ms, 0) for s in MOVE_IDS]
    t_cmd = time.monotonic()
    bus.sync_set_angle(items)
    seg = {'gear': gear, 'trip': trip, 'leg': leg, 't_cmd': round(t_cmd - bus.t0, 6),
           'T_ms': T_ms, 'targets': {str(s): round(targets[s], 2) for s in MOVE_IDS}, 'n_frames': 0}
    segs.append(seg)
    bus.log('leg_cmd', **seg)
    t_end = t_cmd + T_ms / 1000.0 + SETTLE_S
    while time.monotonic() < t_end:
        r = bus.sync_monitor(list(ALL_IDS), timeout=MON_TIMEOUT)
        t_ret = time.monotonic()
        t_tx = t_ret - r['total_s']
        fr = {'t_tx': round(t_tx - bus.t0, 4), 'gear': gear, 'trip': trip, 'leg': leg,
              'ok': r['ok'], 'got': {}}
        # 时间/角度在入帧时就按 CSV 精度取整（t 0.1ms、角 0.01°、dt 0.1ms），
        # 使现场打印的表与 --analyze 从 CSV 复算的表逐字一致（R6）
        for sid, d in r['got'].items():
            fr['got'][sid] = {'ang': (round(d['angle'], 2) if d['valid_measure'] else None),
                              'mA': d['current_mA'], 'mV': d['voltage_mV'], 'mW': d['power_mW'],
                              'st': d['status'], 'tC': d['temp_c'],
                              'dt_ms': round(d['t_offset'] * 1000.0, 1)}
        ctx.add_frame(fr)
        seg['n_frames'] += 1
        guard_frame(fr, ctx)
    ctx.flush()
    seg['complete'] = True
    return seg


def home_to_A(bus, ctx, segs, A, t_home):
    """H3：0x19 同步慢速归位到 A（≤HOME_SPEED），全程监控+守卫，然后 0x0A 读角验收 ≤HOME_TOL"""
    print('   [归位] 0x19 同步 J0–J5 → A，T_home=%dms（≤%.0f°/s）' % (t_home, HOME_SPEED))
    seg = run_leg(bus, ctx, segs, A, t_home, 'home', 0, 'home')
    cur = read_all(bus)
    if cur is None:
        raise RuntimeError('home_read_fail')
    errs = {s: round(cur[s] - A[s], 2) for s in MOVE_IDS}
    bad = [s for s in MOVE_IDS if abs(errs[s]) > HOME_TOL]
    print('   [归位] 验收（0x0A，判过 ≤%.1f°）误差：%s → %s'
          % (HOME_TOL, ' / '.join('J%d %+.2f' % (s, errs[s]) for s in MOVE_IDS),
             '通过' if not bad else ('未过 %s' % ['J%d' % s for s in bad])))
    bus.log('home_verify', errs={str(s): errs[s] for s in MOVE_IDS}, bad=bad)
    seg['home_errs'] = {str(s): errs[s] for s in MOVE_IDS}
    if bad:
        raise RuntimeError('home_not_verified(%s)' % ['J%d' % s for s in bad])
    return seg


# ---------- 统计（只出数；--analyze 可从 CSV+JSON 复算，R6） ----------

def seg_frames(frames, seg):
    return [f for f in frames
            if f['gear'] == seg['gear'] and int(f['trip']) == int(seg['trip']) and f['leg'] == seg['leg']]


def analyze_segment(seg, frames):
    fr = seg_frames(frames, seg)
    t0 = seg['t_cmd']
    T = seg['T_ms'] / 1000.0
    win_end = T + SETTLE_S
    ivs = [fr[i]['t_tx'] - fr[i - 1]['t_tx'] for i in range(1, len(fr))]
    res = {'n': len(fr), 'n_ok': sum(1 for f in fr if f['ok']),
           'iv_med_ms': (median(ivs) * 1000.0) if ivs else None,
           'iv_max_ms': (max(ivs) * 1000.0) if ivs else None,
           'window_s': win_end, 'axes': {}}
    targets = {int(k): v for k, v in seg['targets'].items()}
    for sid in ALL_IDS:
        pts = []
        for f in fr:
            d = f['got'].get(sid)
            if d is None:
                continue
            pts.append((f['t_tx'] + d['dt_ms'] / 1000.0 - t0, d['ang'], d['mA'], d['mV']))
        ax = {'n': len(pts)}
        mv = [p[2] for p in pts if 0.0 <= p[0] <= T and p[2] is not None]
        vv = [p[3] for p in pts if 0.0 <= p[0] <= T and p[3] is not None]
        ax['mA_peak'] = max(mv) if mv else None
        ax['mV_min'] = min(vv) if vv else None
        angs = [(p[0], norm_single_turn(p[1])) for p in pts if p[1] is not None]
        ax['n_ang'] = len(angs)
        if angs:
            ax['ang_min'] = min(a for _, a in angs)
            ax['ang_max'] = max(a for _, a in angs)
        if sid in targets:
            tgt = targets[sid]
            ax['target'] = tgt
            last_bad = -1
            for i, (t, a) in enumerate(angs):
                if abs(a - tgt) > TOL_ARRIVE:
                    last_bad = i
            if not angs or last_bad == len(angs) - 1:
                ax['arrive_s'] = None
            else:
                ax['arrive_s'] = angs[last_bad + 1][0]
            ax['last_out_s'] = angs[last_bad][0] if last_bad >= 0 else None
            fw = [a for t, a in angs if (win_end - FINAL_WIN_S) <= t <= win_end]
            ax['final_n'] = len(fw)
            ax['final_err'] = (sum(fw) / len(fw) - tgt) if fw else None
        res['axes'][sid] = ax
    arr = [res['axes'][s].get('arrive_s') for s in MOVE_IDS]
    if all(a is not None for a in arr):
        res['arrive_min_s'], res['arrive_max_s'] = min(arr), max(arr)
        res['arrive_range_s'] = max(arr) - min(arr)
    else:
        res['arrive_min_s'] = res['arrive_max_s'] = res['arrive_range_s'] = None
    return res


def seg_name(seg):
    if seg['gear'] == 'home':
        return '归位'
    return '%s/往返%d/%s' % (GEAR_WORD.get(seg['gear'], seg['gear']), int(seg['trip']),
                            LEG_WORD.get(seg['leg'], seg['leg']))


def print_segment(seg, st):
    print('')
    print('[段 %s] T=%dms 帧 n=%d（7 轴收齐 %d）帧间隔中位 %s ms / max %s ms（到位时刻分辨率=帧间隔）'
          % (seg_name(seg), seg['T_ms'], st['n'], st['n_ok'],
             fmt(st['iv_med_ms'], '%.1f'), fmt(st['iv_max_ms'], '%.1f')))
    print('  轴  目标(°)   到位时刻(s)  最终误差(°)  电流峰值(mA)  电压最低(mV)  角度范围(°)       n角')
    for s in ALL_IDS:
        ax = st['axes'][s]
        if 'target' in ax:
            arr = fmt(ax['arrive_s'], '%.3f', '未到位')
            if ax['arrive_s'] is None and ax.get('last_out_s') is not None:
                arr += '(末出窗%.2fs)' % ax['last_out_s']
            print('  J%d  %+8.2f  %-12s %-12s %-13s %-13s %s~%s  %d'
                  % (s, ax['target'], arr, fmt(ax['final_err'], '%+.2f'),
                     fmt(ax['mA_peak'], '%d'), fmt(ax['mV_min'], '%d'),
                     fmt(ax.get('ang_min'), '%.1f'), fmt(ax.get('ang_max'), '%.1f'), ax['n_ang']))
        else:
            print('  J%d  夹紧      —            —            %-13s %-13s %s~%s  %d'
                  % (s, fmt(ax['mA_peak'], '%d'), fmt(ax['mV_min'], '%d'),
                     fmt(ax.get('ang_min'), '%.2f'), fmt(ax.get('ang_max'), '%.2f'), ax['n_ang']))
    print('  6 轴到位极差 = %s s（最早 %s / 最晚 %s）'
          % (fmt(st['arrive_range_s'], '%.3f', '不可判(有轴未到位)'),
             fmt(st['arrive_min_s'], '%.3f'), fmt(st['arrive_max_s'], '%.3f')))


def report(doc, frames, segs):
    print('')
    print('===== TC-008 数据表（只出数不写结论；口径见文件头/用例）=====')
    print('定义：到位时刻=首次进入目标±%.1f°且至段末(T+%.1fs)一直保持（分辨率=帧间隔）；'
          '最终误差=段末%.1fs内0x16均值−目标（若某轴到位时刻>T+%.1fs，该轴最终误差含运动中样本，'
          '表中可对照判读）；电流峰值/电压最低=[0,T]内；各数字样本量=表内n角/帧n，精确窗口n见段JSON'
          % (TOL_ARRIVE, SETTLE_S, FINAL_WIN_S, SETTLE_S - FINAL_WIN_S))
    if doc.get('grip'):
        g = doc['grip']
        print('夹瓶：0x16 n=%d 中位=%s 极差=%s 窗=%s → %s；0x0A×5=%s'
              % (len(g.get('r16', [])), fmt(g.get('median')), fmt(g.get('spread')),
                 g.get('window'), g.get('verdict'), g.get('r0a')))
    if doc.get('T'):
        print('时长：T_slow=%(t_slow)d ms  T_mid=%(t_mid)d ms  T_home=%(t_home)d ms' % doc['T'])
    stats = []
    for seg in segs:
        st = analyze_segment(seg, frames)
        stats.append(st)
        print_segment(seg, st)
    print('')
    print('[矩阵] 到位时刻(s)：行=段，列=J0..J5，极差')
    for seg, st in zip(segs, stats):
        print('  %s %s  极差 %s' % (pad(seg_name(seg), 16),
                                     ' '.join('%8s' % fmt(st['axes'][s]['arrive_s'], '%.3f', '未到位')
                                              for s in MOVE_IDS),
                                     fmt(st['arrive_range_s'], '%.3f', '不可判')))
    print('[矩阵] 最终误差(°)：')
    for seg, st in zip(segs, stats):
        print('  %s %s' % (pad(seg_name(seg), 16),
                              ' '.join('%8s' % fmt(st['axes'][s]['final_err'], '%+.2f')
                                       for s in MOVE_IDS)))
    print('[矩阵] 电流峰值(mA)：')
    for seg, st in zip(segs, stats):
        print('  %s %s' % (pad(seg_name(seg), 16),
                              ' '.join('%8s' % fmt(st['axes'][s]['mA_peak'], '%d')
                                       for s in MOVE_IDS)))
    tot = len(frames)
    print('样本范围：帧总数 %d（7 轴收齐 %d）；段数 %d；剔除=无（哨兵角度不计入角度统计，计入帧数）'
          % (tot, sum(1 for f in frames if f['ok']), len(segs)))
    return stats


def load_csv(path):
    frames = []
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            fr = {'t_tx': float(row['t_tx_s']), 'gear': row['gear'], 'trip': int(row['trip']),
                  'leg': row['leg'], 'ok': row['frame_ok'] == '1', 'got': {}}
            for s in ALL_IDS:
                if row['J%d_mV' % s] == '':
                    continue
                fr['got'][s] = {'ang': (float(row['J%d_ang' % s]) if row['J%d_ang' % s] != '' else None),
                                'mA': int(row['J%d_mA' % s]), 'mV': int(row['J%d_mV' % s]),
                                'mW': int(row['J%d_mW' % s]), 'st': int(row['J%d_st' % s]),
                                'tC': (float(row['J%d_tC' % s]) if row['J%d_tC' % s] != '' else None),
                                'dt_ms': float(row['J%d_dt_ms' % s])}
            frames.append(fr)
    return frames


def analyze_files(csv_path, json_path):
    with open(json_path, encoding='utf-8') as fh:
        doc = json.load(fh)
    frames = load_csv(csv_path)
    print('--analyze：%s（%d 帧）+ %s（%d 段）' % (csv_path, len(frames), json_path, len(doc.get('segments', []))))
    report(doc, frames, doc.get('segments', []))


# ---------- main ----------

def main():
    ap = argparse.ArgumentParser(description='TC-008 多轴同步到位（带载）')
    ap.add_argument('--port', default='/dev/ttyS4')
    ap.add_argument('--stop-power', type=int, default=DEFAULT_HOLD_POWER,
                    help='0x18 保持锁力 power_mW（默认 0=按功率保护阈值，H4/F38/F49）')
    ap.add_argument('--vbat-low', type=int, default=11000,
                    help='电压中止阈值 mV（默认 11000，沿用 TC-007）')
    ap.add_argument('--grip-power', type=int, default=0,
                    help='J6 0x08 power_mW（默认 0=按 J6 功率保护阈值 12000mW 执行，F36；F56 未记录功率）')
    ap.add_argument('--grip-dev', type=float, default=1.0,
                    help='运动/静止守卫窗 = grip_ref ± dev（°），dev 默认 1.0【暂定，待 TC-009 v2 数据修订，T6】')
    ap.add_argument('--j6-out-frames', type=int, default=J6_OUT_FRAMES_DEFAULT,
                    help='运动中 J6 连续离开守卫窗多少帧即中止（默认 %d，Thomas 2026-11-02 定）'
                         % J6_OUT_FRAMES_DEFAULT)
    ap.add_argument('--round-trips', type=int, default=3)
    ap.add_argument('--j6-stall-ma', type=int, default=J6_STALL_MA,
                    help='j：J6 堵转电流阈 mA（默认 %d【临时值待 审查方 审定：2026-11-02 实测健康'
                         '夹持停稳 541–557 mA 高于此值】），持续 %.0f s 超阈即释放并中止'
                         % (J6_STALL_MA, J6_STALL_S))
    ap.add_argument('--analyze', nargs=2, metavar=('CSV', 'JSON'),
                    help='离线复算：从 tc008_frames CSV + segments JSON 重打全部数据表')
    args = ap.parse_args()
    if args.analyze:
        analyze_files(args.analyze[0], args.analyze[1])
        return
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    pw = args.stop_power
    ts = time.strftime('%Y%m%d_%H%M%S')
    here = os.path.dirname(os.path.abspath(__file__))
    log = os.path.join(here, 'tc008_log_%s.jsonl' % ts)
    csvf = os.path.join(here, 'tc008_frames_%s.csv' % ts)
    jsonf = os.path.join(here, 'tc008_segments_%s.json' % ts)
    print('TC-008 多轴同步到位·带载（板载时钟 %s，仅参考）' % ts)
    print('!! 负载 = 330 ml 百岁山 378 g（F55）；夹爪 张开 %.1f / 关闭 %.1f；夹瓶判定=旧阈值'
          '（med−29.0 ≥+%.1f 且停稳 → grasped；|med−29.0| ≤%.1f → empty；其余 uncertain；'
          '百岁山最终方案 GRASP-GRIP-SETTING-20261027，F57 作废 F56 窗 [33,34]）'
          % (GRIP_OPEN, GRIP_CLOSE, OLD_GRASP_MIN, OLD_EMPTY_ABS))
    print('!! 两档时长运行时计算并请确认：慢档=最大行程÷%.0f°/s；中档=各轴≤URDF（J0–J2 %.1f / J3–J5 %.1f °/s）'
          % (SLOW_SPEED, AXIS_SPEEDS[0], AXIS_SPEEDS[3]))
    print('!! 中止条件：J6 离 grip_ref±%.1f(连续%d帧，夹瓶判定前不启用)/J6 连续%d帧不可读/电压<%dmV/保护位/Ctrl-C/异常 → J0–J5 safe_stop_all 保持锁力(power=%d)；'
          % (args.grip_dev, args.j6_out_frames, J6_MISS_LIMIT, args.vbat_low, pw))
    print('!!   已夹瓶时 J6 不发 0x18（防夹持力消失，M1），改重发 0x08 目标 %.1f 保持夹持；未夹瓶/已张开则 7 轴 safe_stop_all' % GRIP_CLOSE)
    print('!! 工作区无人无杂物、急停在手边（断电，F27）；J6 全程不发 0x18（保持夹持力），示教只锁 J0–J5')
    print('!! j 堵转保护：J6 电流>%dmA 持续%.0fs → 释放并中止（--j6-stall-ma 可覆盖；400 为临时值，'
          '低于 11-02 实测健康夹持 541–557mA，待 审查方 审定）' % (args.j6_stall_ma, J6_STALL_S))
    print('!! 前置检查：4 服务 inactive + %s 无占用' % args.port)
    ok_pf, msgs = preflight(args.port)
    for m in msgs:
        print('   preflight: %s' % m)
    if not ok_pf:
        print('   !! preflight 未过，退出')
        return
    input('   [上电姿态] 机械臂是否处于可上电姿态（未压在折叠位/机械限位上）？确认后回车'
          '（T4b4：仅提示，不自动判断——折叠位 J2 会超 F7，且中止路径 0x18 保持会带抖动）：')
    print('日志: %s\n帧CSV: %s（即时落盘）\n段JSON: %s' % (log, csvf, jsonf))
    bus = Bus(args.port, log_path=log)
    ctx = Ctx(csvf, args.vbat_low, args.grip_dev, args.j6_out_frames)
    ctx.j6_stall_ma = args.j6_stall_ma
    doc = {'ts': ts, 'stop_power': pw, 'grip_power': args.grip_power, 'grip_dev': args.grip_dev,
           'j6_out_frames': args.j6_out_frames, 'vbat_low': args.vbat_low,
           'round_trips': args.round_trips, 'tol_arrive': TOL_ARRIVE, 'settle_s': SETTLE_S,
           'final_win_s': FINAL_WIN_S, 'slow_speed': SLOW_SPEED, 'axis_speeds': AXIS_SPEEDS,
           'home_speed': HOME_SPEED, 'segments': []}

    def save_doc():
        with open(jsonf, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, ensure_ascii=False, indent=1)

    segs = doc['segments']
    try:
        bus.drain()
        # k/k2 圈数门（P0.5-T4b）：J6 超限→释放→0x11 清→确认；J0–J5 超限→拒绝运动
        ok_mt, rep_mt = mturn_gate(bus, ALL_IDS, clear_ids=(GRIP_ID,), stop_power=pw)
        print('   [圈数门] %s：%s' % ('通过' if ok_mt else '未过（拒绝运动，不发任何运动帧）',
              {('J%d' % k2): (('%.1f°/t%d' % (v['angle'], v['turns'])) if v else '读不到')
               for k2, v in rep_mt.items()}))
        if not ok_mt:
            raise MturnRefused('mturn_gate_fail')
        # f 只读 F7 检查（0x0A 全轴；±0.5° 容差）
        pose0_pre = read_all(bus)
        if pose0_pre is None:
            raise RuntimeError('preflight_read_fail')
        pose_f7_check(pose0_pre, 'preflight')
        print('   [前置] 7 轴 0x0A 读数均在 F7 限位内：%s' % json.dumps(pose0_pre, ensure_ascii=False))
        # ① 放瓶姿态：开跑前全失力（停服+上电锁力关），无释放、不问 H1
        input('   [姿态·放瓶] 请扶住整臂摆到便于放瓶的姿态（夹爪开口朝向便于放瓶），扶住后按回车（先锁 7 轴再读）：')
        sent = bus.lock_all(ALL_IDS, pw, mode=STOP_HOLD)
        if sent != 7 and bus.lock_all(ALL_IDS, pw, mode=STOP_HOLD) + sent < 7:
            raise RuntimeError('lock_all_failed@放瓶')
        time.sleep(0.2)
        pose0 = read_all(bus)
        if pose0 is None:
            raise RuntimeError('pose_read_fail@放瓶')
        pose_f7_check(pose0, '放瓶')
        print('   [姿态·放瓶] 已锁定并记录：%s' % json.dumps(pose0, ensure_ascii=False))
        input('   [S1] 请慢慢松手（臂由舵机保持），按回车开始下垂检查：')
        doc['pose_place'] = pose0
        doc['sag_place'] = sag_check(bus, pose0, '放瓶')
        save_doc()
        # ② 夹瓶
        grip_clamp(bus, ctx, args, doc)
        save_doc()
        # ③ 示教 A、B（wait=input_j6guard：等候期间每 0.5 s 读 J6 堵转检查，j）
        A, sagA = teach_pose(bus, ctx, 'A', pw, release_first=True,
                             wait=lambda pr: input_j6guard(bus, ctx, pr))
        doc['pose_A'], doc['sag_A'] = A, sagA
        save_doc()
        B, sagB = teach_pose(bus, ctx, 'B', pw, release_first=True,
                             wait=lambda pr: input_j6guard(bus, ctx, pr))
        doc['pose_B'], doc['sag_B'] = B, sagB
        save_doc()
        # ④ 时长
        travel, mx_s, t_slow, t_mid, t_home = compute_T(A, B)
        doc['T'] = {'t_slow': t_slow, 't_mid': t_mid, 't_home': t_home,
                    'travel': {str(s): round(travel[s], 2) for s in MOVE_IDS}, 'max_axis': mx_s}
        save_doc()
        print_T(travel, mx_s, t_slow, t_mid, t_home)
        print('   [时长] 下一步：H3 归位到 A（臂现在锁在 B）→ 慢档往返 %d → 中档往返 %d。请离开臂的活动范围。'
              % (args.round_trips, args.round_trips))
        if not confirm_yes('   [时长] 确认两档时长并开始运动？(y=开始 / n=中止并全轴锁)：'):
            raise RuntimeError('T_rejected_by_Thomas')
        # ⑤ H3 归位
        home_to_A(bus, ctx, segs, A, t_home)
        save_doc()
        grip_static_check(bus, ctx, '归位后')
        # ⑥ 两档往返
        for gi, (gear, T) in enumerate((('slow', t_slow), ('mid', t_mid))):
            if gi > 0:
                input('   [档间] %s 完成。请检查瓶子与臂，按回车进入 %s（T=%dms，Ctrl-C=中止并全轴锁）：'
                      % (GEAR_WORD[segs[-1]['gear']], GEAR_WORD[gear], T))
            for trip in range(1, args.round_trips + 1):
                for leg, tgt in (('go', B), ('back', A)):
                    seg = run_leg(bus, ctx, segs, tgt, T, gear, trip, leg)
                    st = analyze_segment(seg, ctx.frames)
                    print('   [%s] 帧 %d 到位极差 %s s 最终误差 %s'
                          % (seg_name(seg), st['n'], fmt(st['arrive_range_s'], '%.3f', '不可判'),
                             ' '.join('J%d %s' % (s, fmt(st['axes'][s]['final_err'], '%+.2f'))
                                      for s in MOVE_IDS)))
                    save_doc()
        # ⑦ 收尾（e/g：张开同样走验证版）
        input('   [收尾] 全部运动完成。请扶住瓶子，按回车后 J6 张开到 %.1f（≤%.0f°/s）：' % (GRIP_OPEN, GRIP_SPEED))
        j6_open_verified(bus, ctx, args, '收尾')
        print('   [收尾] 取下瓶子后锁 7 轴')
        sent = bus.lock_all(ALL_IDS, pw, mode=STOP_HOLD)
        final = read_all(bus)
        print('   [收尾] lock_all 发出 %d/7；最终姿态：%s' % (sent, json.dumps(final, ensure_ascii=False)))
        doc['pose_final'] = final
        doc['stats'] = report(doc, ctx.frames, segs)
        save_doc()
        print('文件：%s\n      %s\n      %s' % (csvf, jsonf, log))
        print('   [退出状态] J6：%s（i）；J0–J5：lock_all 7 轮保持锁力' % ctx.j6_state)
        print('恢复服务（Thomas 臂上执行，TC-004 步骤 6）：sudo systemctl start marm_controller marm_communication marm_control marm_subscripber')
    except (KeyboardInterrupt, Exception) as e:
        if isinstance(e, MturnRefused) and not getattr(e, 'runtime', False):
            # M2①：开机圈数门阶段拒动 → 不发任何帧直接退出（舵机上电本为释放态）
            print('\n!! 多圈超限，拒绝运动（%s）：圈数门阶段，不发送任何帧，直接退出——' % e)
            print('   脚本开机会自动清 J6 圈数；J0–J5 超限请重启臂（开机服务会清除）后再跑')
        else:
            # h/M2②：J0–J5 恒 safe_stop_all（0x18 保持，不依赖目标角度，多圈超限时安全）；
            # J6 按 j6_abort_action 处置（堵转/未验证→释放；已验证夹持→保持不发帧防掉瓶；
            # 已验证张开→保持）；无论何种异常，本路径都不对 J6 发 0x08（0x19 本就不涉及 J6）
            act = j6_abort_action(ctx)
            r = bus.safe_stop_all(MOVE_IDS, pw, mode=STOP_HOLD)
            bad = [sid for sid, st in r['still'].items() if st is not True]
            print('\n!! 异常/中断（%s）：J0–J5 safe_stop_all 收尾（J6 处置=%s）——' % (repr(e), act))
            print('   停止帧已尝试发送 %d（J0–J5）；各轴停住判定：%s'
                  % (r['sent'], {('J%d' % sid): v for sid, v in r['still'].items()}))
            if bad:
                print('   !! 未能确认已锁住的轴：%s——请 Thomas 扶住臂目视确认，必要时急停（断电，F27）'
                      % ['J%d' % sid for sid in bad])
            if act == 'release':
                try:
                    bus.stop(GRIP_ID, STOP_RELEASE, 0)
                    time.sleep(0.2)
                    a6 = read_ang(bus, GRIP_ID)
                    ctx.j6_state = '已释放(0x18 0x10)'
                    print('   J6：已 0x18 释放（0x10，power=0）；读数 %s' % fmt(a6, '%.1f', '读不到'))
                    bus.log('abort_j6_release', j6=a6)
                except Exception as e3:
                    print('   !! J6 释放失败（%s）——请 Thomas 目视确认夹爪，必要时断电' % e3)
            elif act == 'keep_grip':
                mt = read_mturn(bus, GRIP_ID)
                print('   J6：夹持已验证停稳 → 保持 0x08 目标 %.1f，不发帧；温度 %s；'
                      'J6 仍在出力（持续电流顶瓶），脚本退出后无温度监控'
                      % (GRIP_CLOSE, ('%.1f℃' % mt['tC']) if (mt and mt['tC'] is not None) else '读不到'))
                print('   释放三步：① 托住瓶子 ② 在臂上 ~/p04_probes 下运行下面命令（服务仍停着）③ 取下瓶子：')
                print('      ' + J6_RELEASE_CMD)
            else:
                print('   J6：已验证张开 → 保持，不发帧')
        doc['abort'] = repr(e)
        doc['abort_j6'] = ('mturn_refused' if isinstance(e, MturnRefused) else j6_abort_action(ctx))
        try:
            ctx.flush()
            if segs:
                doc['stats'] = report(doc, ctx.frames, segs)
            save_doc()
            print('   已落盘：%s（%d 帧）/ %s' % (csvf, len(ctx.frames), jsonf))
        except Exception as e2:
            print('   (落盘失败: %s)' % e2)
        # i：退出前打印 J6 终态（已验证静止未堵转 / 已释放 之一；未知=中断过早）
        j6_note = '（仍在出力，需按上面命令释放）' if doc['abort_j6'] == 'keep_grip' else ''
        print('   [退出状态] J6：%s%s；J0–J5：%s' % (
            ctx.j6_state, j6_note,
            '未发帧（圈数门拒绝路径）' if isinstance(e, MturnRefused) and not e.runtime
            else 'safe_stop_all 保持锁力'))
    finally:
        ctx.close()
        bus.close()


if __name__ == '__main__':
    main()
