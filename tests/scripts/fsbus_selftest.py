#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fsbus_selftest.py —— 探针库与分析器离线自检（P0.4，复审 B1/M1/M2/H5/H6 要求）

不需要 pyserial 与实机：串口用假件注入。任何一段失败即非零退出。
在 Mac 与臂载计算机上都可运行：python3 fsbus_selftest.py

覆盖：
  S1 请求帧构造 == PROTOCOL_NOTES 官方示例（6 条，校验和逐字节）
  S2 read_frame：逐字节到达 / 分批到达 / 粘包 / 前导垃圾 / 超时（B1——收到的
     字节一律进解析缓冲，不得丢弃）
  S3 响应长度异常 → bad_len；0x16 哨兵 → valid_measure=False（M1）
  S4 sync_monitor：非请求 ID / 重复帧不充数，请求 ID 集齐才算 ok（M1）
  S5 min_gap_ms：连续发送的实测发出间隔 ≥ 设定值（M4）
  S6 单圈归一化 norm_single_turn（M7）
  S7 speed_stats 区分度：理想匀速 vs 100ms 每条重起，两项指标可分开（H6）
  S10 lock_all / safe_stop_all 单元（T12 关键代码）
  S11 TC-008 统计口径（到位时刻/最终误差/极差/电流峰/电压低）合成数据复算（P0.5-T2）
  S12 TC-008 guard_frame（电压/保护位/J6 出窗/J6 不可读/连续帧计数）（P0.5-T2 关键代码）
  S13 TC-009 统计口径（停住角/稳态/峰值/接触到位 100ms 窗 5°/s）合成数据复算（P0.5-T4）
  S9  仿真总线整跑 tc004–tc009 main（tc008 六场景 + tc009 六场景 + --analyze 复算 R6
      + 中止时 J6 处置日志核对：tc008 M1 / tc009 关闭状态单独张开）
"""

import os
import sys
import time
import types

# pyserial 缺失时注入桩（离线自检不碰真串口）
try:
    import serial  # noqa: F401
except ImportError:
    _stub = types.ModuleType('serial')
    _stub.EIGHTBITS = 8
    _stub.PARITY_NONE = 'N'
    _stub.STOPBITS_ONE = 1
    _stub.Serial = None  # 由 FakeSerial 替换
    _stub.SIMULATED = True  # 仿真件标记（fsbus.preflight 逃生门校验用）
    sys.modules['serial'] = _stub

import struct

sys.path.insert(0, '.')
import fsbus  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    tag = 'PASS' if cond else 'FAIL'
    print('[%s] %s %s' % (tag, name, detail))
    if not cond:
        FAILS.append(name)


def make_body(cmd, params):
    body = fsbus.RSP_HEADER + bytes([cmd, len(params)]) + params
    return body + bytes([sum(body) & 0xFF])


MON_PARAMS = bytes.fromhex('00831e1e00ea002c0701af0b00000000')  # 文档 §0x16 示例字段
MON_RESP = make_body(0x16, bytes([6]) + MON_PARAMS[1:])  # ID6 版本


class FakeSerial(object):
    """行为可注入的假串口：mode 控制响应到达方式
    instant 一次全放行 / byte_by_byte 每查询 1 字节 / batch 每 4 字节 /
    manual 只回显测试预装的字节（write 不自动应答）/
    none 永不放行（超时路径）"""

    def __init__(self, **kw):
        self.rx = bytearray()
        self.mode = 'instant'
        self._pending = bytearray()
        self.tx_log = []
        self.timeout = kw.get('timeout', 0.002)
        self.angles = {i: 10.0 for i in range(7)}   # S10：0x0A 应答用的静止角
        self.fail_read_sid = None                    # S10-c：注入某轴读不到

    # -- 主机侧 API --
    def write(self, b):
        self.tx_log.append((time.monotonic(), bytes(b)))
        if self.mode in ('manual', 'none'):
            return len(b)
        cmd = b[2] if len(b) > 2 else None
        if cmd == 0x16 and len(b) == 6:
            self._pending.extend(MON_RESP)
        elif cmd == 0x01:
            self._pending.extend(make_body(0x01, bytes([b[4]])))
        elif cmd == 0x0A and len(b) == 6:
            import struct as _st
            sid = b[4]
            if self.fail_read_sid is None or sid != self.fail_read_sid:
                raw = int(round(self.angles.get(sid, 10.0) * 10))
                self._pending.extend(make_body(0x0A, _st.pack('<Bh', sid, raw)))
        return len(b)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.rx.clear()

    @property
    def in_waiting(self):
        if self.mode == 'none':
            return 0
        n = {'byte_by_byte': 1, 'batch': 4, 'instant': len(self._pending),
             'manual': len(self._pending)}.get(self.mode, len(self._pending))
        return min(n, len(self._pending))

    def read(self, n=1):
        if self.mode == 'none':
            return b''
        n = min(n, len(self._pending))
        out = bytes(self._pending[:n])
        del self._pending[:n]
        return out

    def close(self):
        pass


def make_bus(mode='instant', **kw):
    fake = FakeSerial(**kw)
    fake.mode = mode
    orig = sys.modules['serial'].Serial
    sys.modules['serial'].Serial = lambda **k: fake
    try:
        bus = fsbus.Bus(log_path=None, **kw)
    finally:
        sys.modules['serial'].Serial = orig
    return bus, fake


def s1_frames():
    cases = [
        (fsbus.Bus.build(0x01, bytes([0])), '124c01010060'),
        (fsbus.Bus.build(0x16, bytes([0])), '124c16010075'),
        (fsbus.Bus.build(0x0A, bytes([0])), '124c0a010069'),
        (fsbus.Bus.build(0x08, struct.pack('<BhHH', 0, 900, 500, 0)), '124c0807008403f4010000e9'),
        (fsbus.Bus.build(0x18, struct.pack('<BBH', 0, 0x11, 6000)), '124c18040011701712'),
        (fsbus.Bus.build(0x19, struct.pack('<BBB', 0x08, 7, 2)
                         + struct.pack('<BhHH', 1, 300, 1000, 0)
                         + struct.pack('<BhHH', 2, 600, 2000, 0)),
         '124c1911080702012c01e8030000025802d0070000e5'),
    ]
    for frame, expect in cases:
        check('S1 帧构造 %s' % expect, frame.hex() == expect, frame.hex())


def s2_read_frame():
    for mode, desc in (('byte_by_byte', '逐字节'),
                       ('batch', '分批(4字节)'),
                       ('instant', '一次到达(含粘包)')):
        bus, fake = make_bus(mode, resp_timeout=0.5)
        fake._pending.extend(b'\x00\xff')          # 前导垃圾
        if mode == 'instant':
            fake._pending.extend(MON_RESP + MON_RESP)  # 粘包：两帧连发
        f = bus.read_monitor(6)
        ok1 = f is not None and f['verdict'] == 'ok' and abs(f['angle'] - 299.1) < 0.05
        check('S2 %s+垃圾前缀 解析' % desc, ok1,
              '' if ok1 else (f and f.get('verdict')))
        if mode == 'instant':
            f2 = bus.read_monitor(6)  # 第二帧应还在缓冲？——被上一事务消费则再读一次
            check('S2 粘包第二帧可再取', f2 is not None and f2['verdict'] == 'ok',
                  '' if (f2 and f2['verdict'] == 'ok') else 'second frame handling')
        bus.close()
    # 超时路径
    bus, fake = make_bus('none', resp_timeout=0.02)
    t0 = time.monotonic()
    f = bus.read_monitor(6)
    check('S2 超时返回 None', f is None, '%.0fms' % ((time.monotonic() - t0) * 1000))
    bus.close()


def s3_lengths():
    # 每个子用例独立 manual 假串口，预装唯一一条异常帧，杜绝残留污染
    def one_shot(preload, call, expect_verdict, extra_check=None):
        bus, fake = make_bus('manual', resp_timeout=0.1)
        fake._pending.extend(preload)
        f = call(bus)
        okv = f is not None and f['verdict'] == expect_verdict
        okx = extra_check(f) if extra_check else True
        bus.close()
        return okv and okx, (f and f['verdict'])

    ok, d = one_shot(make_body(0x01, b''), lambda b: b.ping(6), 'bad_len')
    check('S3 ping 空回包 → bad_len', ok, d)
    ok, d = one_shot(make_body(0x0A, b'\x06\x2c'), lambda b: b.read_angle(6), 'bad_len')
    check('S3 0x0A 短回包 → bad_len', ok, d)
    ok, d = one_shot(make_body(0x16, b'\x06' + MON_PARAMS[1:8]), lambda b: b.read_monitor(6), 'bad_len')
    check('S3 0x16 短回包 → bad_len', ok, d)
    sent_params = struct.pack('<BHHHHBih', 6, 7800, 30, 200, 1800, 0,
                              fsbus._SENTINEL_ANGLE, 0)
    ok, d = one_shot(make_body(0x16, sent_params), lambda b: b.read_monitor(6),
                     'ok', lambda f: f['valid_measure'] is False and f['sentinel'])
    check('S3 0x16 哨兵 → verdict=ok 但 valid_measure=False', ok, d)


def s4_sync():
    bus, fake = make_bus('instant', resp_timeout=0.2, min_gap_ms=0)
    # 请求 [6]，回的是 ID5（合法帧但非请求 ID）→ 不得算成功
    fake._pending.extend(make_body(0x16, bytes([5]) + MON_PARAMS[1:]))
    r = bus.sync_monitor([6], timeout=0.06)
    check('S4 非请求ID不充数', r['ok'] is False and r['extra'] == 1,
          'ok=%s extra=%d' % (r['ok'], r['extra']))
    # 请求 [5,6]，回 [5,5]（重复）→ 不算
    fake._pending.extend(make_body(0x16, bytes([5]) + MON_PARAMS[1:]) * 2)
    r = bus.sync_monitor([5, 6], timeout=0.06)
    check('S4 重复帧不充数', r['ok'] is False and len(r['got']) == 1)
    # 请求 [5,6]，回 [6,5]（乱序）→ 成功
    fake._pending.extend(make_body(0x16, bytes([6]) + MON_PARAMS[1:]))
    fake._pending.extend(make_body(0x16, bytes([5]) + MON_PARAMS[1:]))
    r = bus.sync_monitor([5, 6], timeout=0.15)
    check('S4 乱序集齐算成功', r['ok'] is True and set(r['got']) == {5, 6})
    # 坏帧不充数
    bad = bytearray(make_body(0x16, bytes([6]) + MON_PARAMS[1:]))
    bad[-1] ^= 0xFF
    fake._pending.extend(bytes(bad))
    r = bus.sync_monitor([6], timeout=0.06)
    check('S4 坏帧不充数', r['ok'] is False and r['bad'] == 1)
    bus.close()


def s5_min_gap():
    bus, fake = make_bus('instant', min_gap_ms=10)
    ts = []
    for _ in range(4):
        bus.send(0x01, bytes([6]))
        ts.append(fake.tx_log[-1][0])
    ivs = [(ts[i] - ts[i - 1]) * 1000 for i in range(1, len(ts))]
    check('S5 min_gap=10ms 实测间隔全部 ≥9.5ms', min(ivs) >= 9.5,
          'min=%.1fms' % min(ivs))
    bus.close()


def s6_norm():
    ok = all(abs(fsbus.norm_single_turn(x) - want) < 1e-9
             for x, want in ((299.1, -60.9), (-60.9, -60.9), (180.0, -180.0), (-190.0, 170.0)))
    check('S6 单圈归一化', ok)


def s7_speed_stats():
    from tc007_stream import speed_stats
    speed, dt, T = 25.0, 0.012, 4.0
    ideal = [('X', i * dt, speed * i * dt) for i in range(int(T / dt))]

    def gen_stutter(seg_t=0.1, vpk=50.0, a=1000.0):
        """100ms 每条重起模型：段内三角速度剖面 0→50°/s→0，段位移 2.5°（均值 25°/s）。
        0.5ms 步进数值积分，每 dt 采样一个点"""
        out, cur, pos, nxt = [], 0.0, 0.0, 0.0
        step = 0.0005
        while cur <= T:
            tau = (cur % seg_t)
            v = min(a * tau, a * (seg_t - tau), vpk)
            if cur >= nxt - 1e-9:
                out.append(('X', cur, pos))
                nxt += dt
            pos += v * step
            cur += step
        return out

    stut = gen_stutter()
    si = speed_stats(ideal, 'X', speed, win=(0.0, T))
    ss = speed_stats(stut, 'X', speed, win=(0.0, T))
    # 主判据 = 纹波展宽比 spread_ratio=(p95−p5)/mean（索引 6）：
    # ±w 中心差分会把 100ms 重起锯齿平滑掉一部分，但展宽比仍与理想匀速显著分离
    # （合成数据实测：理想 0.00 vs 重起 0.55；实机量化噪声约 +0.1~0.2 量级）
    check('S7 理想匀速: 展宽比<0.2 且均值≈指令速',
          si is not None and si[6] < 0.20 and abs(si[1] - speed) <= 2.0,
          'mean=%.1f spread=%.2f' % (si[1], si[6]) if si else 'None')
    check('S7 100ms重起: 展宽比>0.35',
          ss is not None and ss[6] > 0.35,
          'mean=%.1f spread=%.2f' % (ss[1], ss[6]) if ss else 'None')
    # 停后窗口不得混入运动均值：win 过滤应完全隔离窗外静止点
    mixed = ideal + [('X', T + 0.8, speed * T)]
    sm = speed_stats(mixed, 'X', speed, win=(0.0, T))
    check('S7 窗口过滤生效（窗外静止点不影响）',
          sm is not None and abs(sm[1] - si[1]) < 1e-9)


def s8_triangle_excl():
    """H-f：理想三角波剔除换向窗后，段内差分的展宽比必须 < 0.2
    （旧算法跨剔除区间差分 → 展宽比虚高 ~0.99 的假顿挫）"""
    from tc007_stream import speed_stats

    def tri(t, lo, hi, seg):
        ph = (t % (2.0 * seg)) / seg
        return lo + (hi - lo) * ph if ph <= 1.0 else hi - (hi - lo) * (ph - 1.0)

    lo, hi, seg, T, dt = 0.0, 25.0, 1.0, 4.0, 0.012
    samples = [('T', i * dt, tri(i * dt, lo, hi, seg)) for i in range(int(T / dt))]
    reversals = [seg * k for k in range(1, 4)]
    excl = [(r - 0.2, r + 0.2) for r in reversals]
    st = speed_stats(samples, 'T', 25.0, win=(0.05, T - 0.05), excl=excl)
    check('S8 理想三角波(剔换向窗): 展宽比<0.2 且均值≈25',
          st is not None and st[6] < 0.2 and abs(st[1] - 25.0) <= 2.0,
          'mean=%.1f spread=%.3f' % (st[1], st[6]) if st else 'None')


SIM_SOURCE = r'''
# 舵机仿真串口（fsbus_selftest S9 专用；替换 pyserial.Serial）
import os, struct, time
MODE = os.environ.get('P04_SIM_MODE', 'normal')
RESP_DELAY = float(os.environ.get('P04_SIM_DELAY', '0.002'))
# TC-008 用注入项（未设置时行为与 P0.4 版完全一致）：
#   P04_SIM_J6_STALL=33.5  J6 关不到比它更小的角（模拟瓶身挡住，停在该角）
#   P04_SIM_SLIP_AT=2      MODE=j6_slip 时，第几条 0x19+0x08 之后 J6 突降 4°（模拟打滑/掉瓶）
#   P04_SIM_FAIL_0A=3:2    舵机 3 的第 2 次之后的 0x0A 不应答（模拟示教读角失败）
_stall = os.environ.get('P04_SIM_J6_STALL')
J6_STALL = float(_stall) if _stall else None
SLIP_AT = int(os.environ.get('P04_SIM_SLIP_AT', '2'))
SLIP_DROP = float(os.environ.get('P04_SIM_SLIP_DROP', '4.0'))   # T6：打滑深度（停稳角−该值）
_f0a = os.environ.get('P04_SIM_FAIL_0A')
FAIL_0A = (int(_f0a.split(':')[0]), int(_f0a.split(':')[1])) if _f0a else None
TEMP_ADC = int(os.environ.get('P04_SIM_TEMP_ADC', '1750'))   # TC-009 温度暂停注入（1750≈32.8℃）
ERR_AT = int(os.environ.get('P04_SIM_ERR_AT', '5'))          # MODE=bus_err：第 N 条 0x16 请求起 write 抛错
J6_MTURN = float(os.environ.get('P04_SIM_J6_MTURN', '0'))    # J6 0x16 角度偏移（事故复现：388.6）
J6_MTURN_AFTER = int(os.environ.get('P04_SIM_J6_MTURN_AFTER_SYNC', '0'))  # 第 N 次 0x19 同步读后偏移生效（M2 运行中触发用）
SYNC_DROP_ID = int(os.environ['P04_SIM_SYNC_DROP_ID']) if os.environ.get('P04_SIM_SYNC_DROP_ID') else None
SYNC_DROP_AFTER = int(os.environ.get('P04_SIM_SYNC_DROP_AFTER', '0'))    # 第 N 次 0x19+0x16 起丢该轴
J0_MTURN = float(os.environ.get('P04_SIM_MTURN_J0', '0'))    # J0 0x16 角度偏移（拒动场景：200）
CLEAR_FAIL = os.environ.get('P04_SIM_CLEAR_FAIL') == '1'     # 0x11 清圈无效
# T4b4：J6 关闭试次角度曲线回放——'csv:<路径>:<trial>:<第N次关闭>'（现场 csv 原样回放）或
# 'inline:t=ang;t=ang:第N次'（合成折线）。激活帧签名 = J6 0x08 目标 29.0@1775ms（tc009 试次
# 关闭；标定点 29.0 是 550ms、张开是 64.5，均不冲突）；激活后 J6 角度按 t=激活时刻起算插值，
# 任何新的 J6 0x08/0x18/0x11 接管并结束回放。
REPLAY = None
_rp = os.environ.get('P04_SIM_J6_REPLAY')
if _rp:
    _spec, _nth = _rp.rsplit(':', 1)
    if _spec.startswith('csv:'):
        import csv as _csv
        _path, _tno = _spec[4:].rsplit(':', 1)
        _pts = [(float(r['t_s']), float(r['ang'])) for r in _csv.DictReader(open(_path))
                if r['trial'] == _tno]
    else:  # inline:t=ang;t=ang
        _pts = [(float(x.split('=')[0]), float(x.split('=')[1])) for x in _spec[len('inline:'):].split(';')]
    _pts.sort()
    assert _pts, 'replay 曲线为空'
    REPLAY = (_pts, int(_nth))
EIGHTBITS = 8
PARITY_NONE = 'N'
STOPBITS_ONE = 1
SIMULATED = True  # 仿真件标记：P04_SKIP_PREFLIGHT 逃生门只在带此标记时生效

class Serial(object):
    def __init__(self, **kw):
        self.timeout = kw.get('timeout', 0.002)
        self._pending = bytearray()
        self._rx = bytearray()
        self.angles = {i: 0.0 for i in range(7)}
        self.angles[6] = float(os.environ.get('P04_SIM_J6_INIT', '20'))
        self.traj = {}
        self.n_sync08 = 0
        self.n_0a = {}
        self.n_016 = 0
        self.n_j6_008 = 0
        self.n_sync = 0
        self.j6_drift = False
        self._replay = None      # (激活时刻, [(t, ang)...])：激活期间 J6 角度走回放曲线
        self._n_close08 = 0      # 第几次「关闭签名」0x08（目标 29.0@1775ms）

    def _angle(self, sid):
        if sid == 6 and self._replay:
            t0, pts = self._replay
            t = time.monotonic() - t0
            if t <= pts[0][0]:
                return pts[0][1]
            if t >= pts[-1][0]:
                return pts[-1][1]
            for k in range(1, len(pts)):
                if pts[k][0] >= t:
                    (t1, a1), (t2, a2) = pts[k - 1], pts[k]
                    return a1 + (a2 - a1) * (t - t1) / (t2 - t1)
        a = self.angles.get(sid, 0.0)
        tr = self.traj.get(sid)
        if tr:
            t0, a0, a1, dur = tr
            if MODE == 'no_move':
                return a0
            if dur <= 0:
                return a1
            ph = min(1.0, max(0.0, (time.monotonic() - t0) / dur))
            return a0 + (a1 - a0) * ph
        return a

    def _freeze(self, sid):
        if MODE in ('no_move', 'ignore_stop'):
            if MODE == 'ignore_stop':
                return  # 0x18 被忽略：轨迹继续向目标推进
            self.angles[sid] = self._angle(sid)
        else:
            self.angles[sid] = self._angle(sid)
        if sid == 6:
            self._replay = None  # 0x18 接管，回放结束
        self.traj.pop(sid, None)

    def _set_traj(self, sid, raw, itv):
        tgt = raw / 10.0
        if sid == 6 and J6_STALL is not None:
            tgt = max(tgt, J6_STALL)  # 瓶身挡住：关不到更小角
        if MODE == 'j6_short' and sid == 6:
            cur0 = self._angle(sid)
            tgt = cur0 + (tgt - cur0) * 0.9   # 只走 90%：到位校验失败场景
        if MODE == 'j6_reverse' and sid == 6:
            cur0 = self._angle(sid)
            tgt = cur0 - (tgt - cur0)          # 反向：S2 方向试探失败场景
        self.traj[sid] = (time.monotonic(), self._angle(sid), tgt, itv / 1000.0)

    @staticmethod
    def _resp(cmd, params):
        body = b'\x05\x1c' + bytes([cmd, len(params)]) + params
        return body + bytes([sum(body) & 0xFF])

    def _mon_params(self, sid):
        j6off = J6_MTURN if (J6_MTURN and (not J6_MTURN_AFTER or self.j6_drift)) else 0.0
        a = self._angle(sid) + (j6off if sid == 6 else (J0_MTURN if sid == 0 else 0.0))
        return struct.pack('<BHHHHBih', sid, 11800, 30, 240, TEMP_ADC, 0, int(round(a * 10)), 0)

    def write(self, b):
        if MODE == 'bus_err' and len(b) >= 4 and b[2] == 0x16:
            self.n_016 += 1
            if self.n_016 > ERR_AT:
                raise IOError('sim bus write fail')
        self._rx.extend(b)
        while len(self._rx) >= 4:
            cmd, size = self._rx[2], self._rx[3]
            total = 4 + size + 1
            if len(self._rx) < total:
                break
            frame = bytes(self._rx[:total])
            del self._rx[:total]
            if (sum(frame[:-1]) & 0xFF) != frame[-1]:
                continue
            self._handle(cmd, frame[4:4 + size])
        return len(b)

    def _handle(self, cmd, p):
        if MODE == 'silent':
            return
        out = b''
        if cmd == 0x01 and len(p) == 1:
            out += self._resp(0x01, bytes([p[0]]))
        elif cmd == 0x0A and len(p) == 1:
            sid = p[0]
            self.n_0a[sid] = self.n_0a.get(sid, 0) + 1
            if FAIL_0A and sid == FAIL_0A[0] and self.n_0a[sid] > FAIL_0A[1]:
                return  # 注入：该舵机此后 0x0A 不应答
            out += self._resp(0x0A, struct.pack('<Bh', sid, int(round(self._angle(sid) * 10))))
        elif cmd == 0x16 and len(p) == 1:
            out += self._resp(0x16, self._mon_params(p[0]))
        elif cmd == 0x18 and len(p) >= 1:
            self._freeze(p[0])
        elif cmd == 0x11 and len(p) == 1:
            if not CLEAR_FAIL:
                self.angles[p[0]] = ((self._angle(p[0]) + 180.0) % 360.0) - 180.0
                self.traj.pop(p[0], None)
                if p[0] == 6:
                    self._replay = None
                # 多圈偏移随清圈归零（全局变量按 sid 对应关系处理）
                globals()['J6_MTURN' if p[0] == 6 else 'J0_MTURN'] = 0.0
        elif cmd == 0x08 and len(p) == 7:
            sid, raw, itv, pw = struct.unpack('<BhHH', p)
            if sid == 6:
                self.n_j6_008 += 1
                if self._replay:   # 记住回放末角，后续轨迹从这里接续
                    self.angles[6] = self._angle(6)
                    self._replay = None
                if REPLAY is not None and abs(raw / 10.0 - 29.0) < 0.05 and itv == 1775:
                    self._n_close08 += 1
                    if self._n_close08 == REPLAY[1]:
                        self._replay = (time.monotonic(), REPLAY[0])
            self._set_traj(sid, raw, itv)
        elif cmd == 0x19 and len(p) >= 3:
            sub, d, n = p[0], p[1], p[2]
            if sub == 0x16 and d == 1:
                self.n_sync += 1
                if J6_MTURN_AFTER and self.n_sync > J6_MTURN_AFTER:
                    self.j6_drift = True
                ids = list(p[3:3 + n])
                if SYNC_DROP_ID is not None and self.n_sync > SYNC_DROP_AFTER:
                    ids = [x for x in ids if x != SYNC_DROP_ID]
                for sid in ids:
                    out += self._resp(0x16, self._mon_params(sid))
            elif sub == 0x08:
                off = 3
                for _ in range(n):
                    if off + 7 > len(p):
                        break
                    sid, raw, itv, pw = struct.unpack('<BhHH', p[off:off + 7])
                    off += 7
                    self._set_traj(sid, raw, itv)
                self.n_sync08 += 1
                if MODE == 'j6_slip' and self.n_sync08 >= SLIP_AT:
                    # 打滑/掉瓶：J6 突然比停稳角再闭合 SLIP_DROP 度（默认 4）
                    base = J6_STALL if J6_STALL is not None else self._angle(6)
                    self.traj.pop(6, None)
                    self.angles[6] = base - SLIP_DROP
        if out:
            time.sleep(RESP_DELAY)
            self._pending.extend(out)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._pending.clear()

    @property
    def in_waiting(self):
        return len(self._pending)

    def read(self, n=1):
        n = min(n, len(self._pending))
        out = bytes(self._pending[:n])
        del self._pending[:n]
        return out

    def close(self):
        pass
'''

# TC-009 场景共用的缩减参数（正式默认：空夹10+夹瓶10+功率对照6000/3000各3+拔瓶3）
_T9 = ['--n-empty', '1', '--n-bottle', '1', '--powers', '6000', '--n-power', '1', '--n-pull', '1']
_CAL = 'abc\n71.5\n62\n52\n44\n36\n28\n19\n9\n'   # 8 点标定 mm 输入（首行 abc 验证重问）
_POSE = '\n'   # T4b4：上电姿态确认（圈数门前的 input，各 tc008/tc009 场景 stdin 首位）
_FULL9 = _POSE + _CAL + '\n\n\n' + '\n' + '1\n' + '\n' + '1\n' + '\n'
# T4b4 回放源：v2 现场 051531 轮 samples csv（试次 11 = 首次夹瓶，旧窗误判未过的原始角度曲线）
_V2_T11CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                          'runs', '2026-11-03-TC-009', 'v2', 'tc009_samples_20231122_051531.csv')

S9_CASES = [
    # (脚本, 附加参数, 模式, stdin 答案, 断言子串列表, 说明)
    # 断言以 '!' 开头 = 该子串不得出现
    ('tc004_recon.py', [], 'normal', '',
     ['SUMMARY ping_online_ids = [0, 1, 2, 3, 4, 5, 6]'], '正常跑完'),
    ('tc004_recon.py', [], 'silent', '',
     ['SUMMARY', 'FAIL'], '全超时也完整收场(只读)'),
    ('tc005_timing.py',
     ['--cycles', '10,8', '--cycles-b', '6,4', '--n-cmd', '4', '--n7', '5'],
     'normal', '', ['收尾健康检查', 'ping J5: ok'], '正常跑完(缩减参数)'),
    ('tc005_timing.py', ['--n-cmd', '2'], 'silent', '',
     ['预检失败，终止'], '预检即挡下'),
    ('tc006_stop_sync.py', [], 'normal', 'n\nn\nn\ny\n',
     ['SUMMARY 0x11@0mW: pass', 'SUMMARY 0x11@6000mW: pass',
      'SUMMARY 0x12@6000mW: pass', 'SUMMARY 0x10@6000mW: pass',
      'SUMMARY sync: pass'], '四轮+同步全过，手拨 n/n/n/y'),
    ('tc006_stop_sync.py', [], 'silent', '',
     ['异常/中断（', 'safe_stop 收尾'], '读超时→中止+safe_stop'),
    ('tc006_stop_sync.py', [], 'no_move', '',
     ['归位未过验收', 'safe_stop 收尾'], '恢复失败→中止+safe_stop'),
    ('tc006_stop_sync.py', [], 'ignore_stop', 'n\n',
     ['OBS 本轮结论: fail（not_stopped）', '0x18 兜底手段不可信',
      'safe_stop 收尾', 'SUMMARY(部分) 0x11@0mW: fail（not_stopped）',
      '!SUMMARY(部分) 0x11@6000mW', '!SUMMARY sync'],
     '停止被忽略→第1轮即中止(复审·中)'),
    ('tc007_stream.py', ['--order', '6', '--periods', '100', '--round-trips', '1'],
     'normal', '\n\n\n\n\ny\n1\n\n',
     ['按回车只解锁 J6', '下垂检查] 7 轴偏差全部', '测试范围', '归位] 验收',
      '本轴全速 99.1', 'SUMMARY J6', '倾向', '口述顺滑度=连续', 'S2] 非被测轴偏差全部'],
     '合并式正常跑完(单轴缩减,H1-H4/S1/S2 链路+分轴速度)'),
    ('tc007_stream.py', ['--order', '6', '--vbat-low', '12000'],
     'normal', '\n\n\n\n\ny\n',
     ['异常/中断（', 'safe_stop_all 收尾', 'vbat_low'],
     '电压守卫触发→中止+全轴锁'),
    ('tc007_stream.py', ['--order', '6'], 'silent', '\n',
     ['异常/中断（', 'pose_read_fail', 'safe_stop_all 收尾'],
     'H2 姿态读失败→中止+全轴锁'),
    ('tc007_stream.py', ['--order', '6'], 'no_move', '\n\n\n\n\ny\n',
     ['home_not_verified', 'safe_stop_all 收尾'],
     'H3 归位验收失败→中止+全轴锁'),
    # ---- TC-008（P0.5-T2，判定=T6 旧阈值+grip_ref 相对窗）：第 7 元 = 附加环境变量。
    #      stdin 顺序 = 上电姿态(T4b4)/放瓶姿态/S1/放瓶回车/H1/姿态A/S1/H1/姿态B/S1/
    #      时长确认(y)/档间/收尾 ----
    ('tc008_sync_load.py', [], 'normal', '\n' * 10 + 'y\n' + '\n\n',
     ['[上电姿态]', '圈数门] 通过', '判定：通过（grasped', 'grip_ref=31.60', '夹持核查@A', '夹持核查@B',
      '夹持核查@归位后', '→ 通过', '[段 归位]', '[段 慢档/往返3/回程]', '[段 中档/往返3/回程]',
      '6 轴到位极差 = 0.0', '[矩阵] 到位时刻', '恢复服务', '!异常/中断（', '!safe_stop_all 收尾'],
     'T6①停住31.6按旧阈值grasped通过+正常跑完(grip_ref=31.6)',
     {'P04_SIM_J6_STALL': '31.6'}),
    ('tc008_sync_load.py', [], 'normal', '\n' * 10 + 'y\n' + '\n\n',
     ['判定：未过（empty', 'grip_empty', 'J0–J5 safe_stop_all 收尾', 'J6：已 0x18 释放', '!夹持核查@A'],
     'T6②停住29.2旧阈值|err|≤1→grip_empty→中止+J0–J5 锁+J6 释放（h）',
     {'P04_SIM_J6_STALL': '29.2'}),
    ('tc008_sync_load.py', ['--round-trips', '1'], 'j6_slip', '\n' * 10 + 'y\n' + '\n\n',
     ['→ 通过', 'j6_out_of_window', 'J0–J5 safe_stop_all 收尾', '夹持已验证停稳 → 保持',
      '仍在出力', '释放三步：① 托住瓶子', 'python3 -c "from fsbus import Bus, STOP_RELEASE',
      '（仍在出力，需按上面命令释放）', '[段 归位]', '[段 慢档/往返1/去程]', '!恢复服务（Thomas'],
     'T6③运动中 J6 连续3帧=grip_ref−1.5→中止：J0–J5 锁+J6 保持夹持(h/M1)+打印释放命令(T6b)',
     {'P04_SIM_J6_STALL': '31.6', 'P04_SIM_SLIP_AT': '2', 'P04_SIM_SLIP_DROP': '1.5'}),
    ('tc008_sync_load.py', ['--vbat-low', '12000'], 'normal', '\n' * 10 + 'y\n' + '\n\n',
     ['vbat_low', 'J0–J5 safe_stop_all 收尾', 'J6：已 0x18 释放', '!判定：通过'],
     '电压守卫触发(夹瓶读阶段)→中止+J0–J5 锁+J6 释放', {'P04_SIM_J6_STALL': '33.5'}),
    ('tc008_sync_load.py', [], 'normal', '\n' * 10 + 'y\n' + '\n\n',
     ['判定：通过', 'J3 读角失败', 'pose_read_fail@A', 'J0–J5 safe_stop_all 收尾', '夹持已验证停稳 → 保持'],
     '姿态 A 读角失败(J3 第 3 次起 0x0A 不应答)→中止：J0–J5 锁+J6 保持夹持',
     {'P04_SIM_J6_STALL': '33.5', 'P04_SIM_FAIL_0A': '3:3'}),
    ('tc008_sync_load.py', [], 'silent', '\n',
     ['mturn_gate_fail', '拒绝运动', '!恢复服务（Thomas'],
     '总线无应答→圈数门读不到→拒动（k：无运动帧无 safe_stop_all）', {}),
    # ---- TC-009 v2（P0.5-T4b 定稿任务单）。stdin：上电姿态(T4b4)+8 个标定 mm 输入（首个 abc
    #      验证解析重问）+ 示教 H1/姿态/S1 + 每试（空夹回车|夹瓶标签|拔瓶回车）----
    ('tc009_grip_char.py', _T9, 'normal', _FULL9,
     ['[上电姿态]', '圈数门] 通过', '请输入数字（毫米）', '[标定 8/8', '收尾] lock_all 发出 7/7', '[f) 开度标定',
      '拟合 mm =', '[b) t_stop − t_contact_I', '[c) 空夹合拢途中', '[d) 分组 kind×P',
      '退出状态] J6：已验证静止、未堵转', '!异常/中断（'],
     'v2正常跑完(圈数门+标定8点含mm解析+示教+5试+收尾)', {}),
    ('tc009_grip_char.py', _T9, 'bus_err', _FULL9,
     ['sim bus write fail', 'J0–J5 safe_stop_all 收尾', 'J6：已 0x18 释放', '!恢复服务（Thomas'],
     '采样中串口故障→中止：J0–J5 锁+J6 释放（不给 J6 发 0x08）', {}),
    ('tc009_grip_char.py', _T9, 'normal', _POSE + _CAL + '\n\n\n' + 'q\n',
     ['[温度]', 'temp_pause_abort', 'J6：已验证张开 → 保持', '!恢复服务（Thomas'],
     'J6≥55℃温度门→Thomas 选中止（已验证张开→保持）', {'P04_SIM_TEMP_ADC': '1000'}),
    ('tc009_grip_char.py', _T9 + ['--vbat-low', '12000'], 'normal', _FULL9,
     ['vbat_low', 'J6：已 0x18 释放', '!恢复服务（Thomas'],
     '电压守卫触发(试次采样)→中止+J6 释放', {}),
    ('tc009_grip_char.py', _T9, 'silent', '\n',
     ['mturn_gate_fail', '拒绝运动', '!恢复服务（Thomas'],
     '总线无应答→圈数门读不到→拒动（无 safe_stop_all、无运动帧）', {}),
    ('tc009_grip_char.py', _T9, 'normal', _FULL9,
     ['圈数门] 通过', "'J6': '20.0°/t0", '收尾] lock_all 发出 7/7', '!异常/中断（'],
     'J6 多圈 388.6 → 释放+0x11 清圈成功 → 全流程跑完', {'P04_SIM_J6_MTURN': '388.6'}),
    ('tc009_grip_char.py', _T9, 'normal', '\n',
     ['mturn_gate_fail', '拒绝运动', '!恢复服务（Thomas'],
     'J6 多圈 388.6 且清圈无效 → 拒动（无运动帧）',
     {'P04_SIM_J6_MTURN': '388.6', 'P04_SIM_CLEAR_FAIL': '1'}),
    ('tc009_grip_char.py', _T9, 'normal', '\n',
     ['mturn_gate_fail', '拒绝运动', '!恢复服务（Thomas'],
     'J0 多圈 200 → 拒动（J0–J5 只查不清）', {'P04_SIM_MTURN_J0': '200'}),
    ('tc009_grip_char.py', _T9, 'j6_reverse', '\n',
     ['j6_probe_fail', '已 0x18 释放', '!恢复服务（Thomas'],
     'J6 反向运动 → S2 方向试探失败 → 释放（≤2.3s）', {}),
    ('tc009_grip_char.py', _T9, 'j6_short', '\n',
     ['j6_open_not_verified', '已 0x18 释放', '!恢复服务（Thomas'],
     'J6 只走 90% → S3 张开终检未过 → 释放中止', {}),
    # ---- T4b4：关闭验证窗 [close_ms+0.2, close_ms+0.5] 两场景（v2 现场 051531 试次 11 复算）----
    ('tc009_grip_char.py', _T9, 'normal', _FULL9,
     ['[关闭验证@试次 2] 通过（31.6', '0.3s 窗极差≤0.5°（停稳）', '收尾] lock_all 发出 7/7',
      '!异常/中断（'],
     'T4b4正常跑完(回放v2现场csv试次11角度曲线:新窗停稳判过,旧窗会误判)',
     {'P04_SIM_J6_REPLAY': 'csv:%s:11:2' % _V2_T11CSV}),
    ('tc009_grip_char.py', _T9, 'normal', _FULL9,
     ['[关闭验证@试次 2] 未过→释放中止', 'j6_close_not_verified@trial2', 'J0–J5 safe_stop_all 收尾',
      'J6：已 0x18 释放', '!恢复服务（Thomas'],
     'T4b4窗内仍在走(回放合成曲线约每20ms−0.1°)→关闭验证未过→释放中止',
     {'P04_SIM_J6_REPLAY': 'inline:0=64.5;1.775=33.0;2.3=30.65:2'}),
    ('tc009_grip_char.py', _T9, 'normal', _POSE + _CAL,
     ['mturn_out_of_range', 'J0–J5 safe_stop_all 收尾', 'J6：已 0x18 释放', '!恢复服务（Thomas'],
     '运行中标定中多圈漂移 → M2②：S5 收尾+J6 释放，stop 后无 0x08/0x19',
     {'P04_SIM_J6_MTURN': '388.6', 'P04_SIM_J6_MTURN_AFTER_SYNC': '4'}),
    ('tc008_sync_load.py', ['--round-trips', '1'], 'normal', '\n' * 10 + 'y\n' + '\n\n',
     ['mturn_read_incomplete', '已重试 1 次', '夹持已验证停稳 → 保持', '!恢复服务（Thomas'],
     'tc008 夹持中 sync 读不齐×2 → M2②：J0–J5 锁、J6 保持不发帧',
     {'P04_SIM_J6_STALL': '33.5', 'P04_SIM_SYNC_DROP_ID': '5', 'P04_SIM_SYNC_DROP_AFTER': '170'}),
]


def s9_main_flows():
    """送审要求 1：仿真总线完整跑通 tc004–tc007 的 main，
    三情形：正常=跑完；读超时/恢复失败=中止并 safe_stop（tc004/005 为只读脚本，
    预期完整收场不崩溃——它们没有可中止的运动段）。T12b 起 tc007 为合并式
    （示教+测试每轴一个流程），stdin 序列=姿态/S1/H1/下限/上限/范围确认(y)/
    口述(1)/轮间；仿真无人推臂故上下限同值→范围确认 y 必现"""
    import shutil
    import subprocess
    import tempfile
    here = os.path.dirname(os.path.abspath(__file__))
    tmp = tempfile.mkdtemp(prefix='p04_sim_')
    try:
        shutil.copy(os.path.join(here, 'fsbus.py'), tmp)
        for s in ('tc004_recon.py', 'tc005_timing.py', 'tc006_stop_sync.py', 'tc007_stream.py',
                  'tc008_sync_load.py', 'tc009_grip_char.py'):
            shutil.copy(os.path.join(here, s), tmp)
        with open(os.path.join(tmp, 'serial.py'), 'w') as fh:
            fh.write(SIM_SOURCE)
        base_env = dict(os.environ)
        base_env['P04_SKIP_PREFLIGHT'] = '1'
        for case in S9_CASES:
            script, extra, mode, stdin, expects, desc = case[:6]
            env = dict(base_env)
            env['P04_SIM_MODE'] = mode
            if len(case) > 6:
                env.update(case[6])  # TC-008 起：第 7 元 = 本场景附加环境变量
            try:
                r = subprocess.run([sys.executable, script] + extra, cwd=tmp, env=env,
                                   input=stdin, capture_output=True, text=True, timeout=180)
                out = r.stdout + r.stderr
                need = [e[1:] for e in expects if e.startswith('!')]
                miss = [e for e in expects if not e.startswith('!') and e not in out]
                bad = [e for e in need if e in out]  # 不应出现却出现了
                ok = r.returncode == 0 and not miss and not bad
                check('S9 %s [%s]' % (script, desc), ok,
                      ('rc=%d 缺:%s 违禁出现:%s' % (r.returncode, miss, bad)) if not ok else '')
                if not ok:
                    print('---- %s [%s] 输出末 15 行 ----' % (script, mode))
                    for line in out.splitlines()[-15:]:
                        print('     | ' + line)
                elif script == 'tc008_sync_load.py':
                    if '正常跑完' in desc:
                        _s9_tc008_roundtrip(tmp, out, env)
                    elif '总线无应答' in desc:      # k：拒动路径——无 safe_stop_all、无运动帧
                        _s9_tc008_log_checks(tmp, out, desc, None, expect_ssa=False, no_motion_frames=True)
                    elif '保持夹持' in desc or 'J6 保持不发帧' in desc:  # h/M2②：已验证夹持 → stop 后不给 J6 发任何帧
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)),
                                             no_008_j6_after_stop=True, no_018_j6_after_close=True)
                    else:                           # h：未验证 → J6 释放帧、无 0x08
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)),
                                             no_008_j6_after_stop=True, j6_release_after_stop=True)
                elif script == 'tc009_grip_char.py':
                    if '正常跑完' in desc:
                        _s9_roundtrip(tmp, out, env, 'tc009', 'tc009_samples_*.csv', 'tc009_trials_*.json',
                                      'tc009_grip_char.py', '===== TC-009 v2 数据表', min_lines=8)
                    elif '串口故障' in desc or '电压守卫' in desc:   # h：未验证→释放帧、无 J6 0x08
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)),
                                             no_008_j6_after_stop=True, j6_release_after_stop=True)
                    elif '温度门' in desc:                          # 已验证张开→保持（无 J6 帧）
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)), no_008_j6_after_stop=True)
                    elif '运行中标定中多圈漂移' in desc:            # M2②：S5 收尾，stop 后无运动帧
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)),
                                             no_008_j6_after_stop=True, j6_release_after_stop=True,
                                             no_motion_after_stop=True)
                    elif '拒动' in desc:                            # k：无 safe_stop_all、无运动帧
                        _s9_tc008_log_checks(tmp, out, desc, None, expect_ssa=False, no_motion_frames=True)
                    elif '反向运动' in desc:                        # S2 失败→释放 ≤2.3s
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)), no_008_j6_after_stop=True,
                                             j6_release_after_stop=True, release_within=2.3)
                    elif '只走 90%' in desc:                        # S3 终检未过→释放
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)), no_008_j6_after_stop=True,
                                             j6_release_after_stop=True)
                    elif '仍在走' in desc:                          # T4b4：关闭验证未过→S5 释放
                        _s9_tc008_log_checks(tmp, out, desc, list(range(6)), no_008_j6_after_stop=True,
                                             j6_release_after_stop=True)
            except subprocess.TimeoutExpired:
                check('S9 %s [%s]' % (script, desc), False, '超时 180s')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _s9_roundtrip(tmp, live_out, env, name, csv_pat, json_pat, script, table_tag, min_lines=20):
    """R6：--analyze 从 CSV+JSON 复算出的数据表须与现场打印逐字一致（tc008/tc009 共用）"""
    import glob
    import subprocess
    csvs = sorted(glob.glob(os.path.join(tmp, csv_pat)))
    jsons = sorted(glob.glob(os.path.join(tmp, json_pat)))
    if not csvs or not jsons:
        check('S9 %s --analyze 复算(R6)' % name, False, '未找到 CSV/JSON')
        return
    r = subprocess.run([sys.executable, script, '--analyze', csvs[-1], jsons[-1]],
                       cwd=tmp, env=env, capture_output=True, text=True, timeout=60)

    def block(s):
        lines = s.splitlines()
        i = next((k for k, l in enumerate(lines) if l.startswith(table_tag)), None)
        if i is None:
            return None
        j = next((k for k in range(i, len(lines)) if lines[k].startswith('样本范围')), None)
        return lines[i:j + 1] if j is not None else None
    a, b = block(live_out), block(r.stdout + r.stderr)
    ok = (r.returncode == 0 and a is not None and b is not None and a == b and len(a) > min_lines)
    detail = '' if ok else ('rc=%d live=%s analyze=%s' % (
        r.returncode, len(a) if a else None, len(b) if b else None))
    if not ok and a and b:
        for x, y in zip(a, b):
            if x != y:
                detail += ' 首个差异: [%s] vs [%s]' % (x, y)
                break
    check('S9 %s --analyze 复算与现场数据表逐字一致(R6, %d 行)' % (name, len(a) if a else 0), ok, detail)


def _s9_tc008_roundtrip(tmp, live_out, env):
    _s9_roundtrip(tmp, live_out, env, 'tc008', 'tc008_frames_*.csv', 'tc008_segments_*.json',
                  'tc008_sync_load.py', '===== TC-008 数据表')


def _s9_tc008_log_checks(tmp, out, desc, stop_ids, j6_008_after_stop=None, no_018_j6_after_close=False,
                         no_008_j6_after_stop=False, expect_ssa=True, no_motion_frames=False,
                         release_within=None, j6_release_after_stop=False,
                         no_motion_after_stop=False):
    """日志核对（jsonl 原始 tx 帧）：
    - expect_ssa：safe_stop_all 事件 ids == stop_ids；expect_ssa=False 时要求不存在 safe_stop_all 事件
    - no_motion_frames：全日志不得出现任何 0x08/0x19 帧（多圈拒绝路径）
    - no_008_j6_after_stop：safe_stop_all 之后不得有发给 J6 的 0x08
    - release_within：自首条 J6 0x08 起，至 J6 的 0x18 释放帧，须 ≤ 该秒数"""
    """M1 复审断言（核对 tc008 jsonl 原始 tx 帧，不看终端打印）：
    - safe_stop_all 事件 ids == stop_ids（已夹瓶 → 只含 J0–J5；未夹瓶 → 7 轴）
    - no_018_j6_after_close：grip_close 事件之后不得有发给 J6 的 0x18 帧
    - j6_008_after_stop：safe_stop_all 之后须有一帧发给 J6 的 0x08，目标 = 该值"""
    import json as _json
    logp = None
    for line in out.splitlines():
        # T4b4：上电姿态 input 的提示与「日志: 」可能同处一行（管道 stdin 无回显换行），用 in 匹配
        if '日志: ' in line:
            logp = line.split('日志: ', 1)[1].strip()
    if not logp or not os.path.exists(logp):
        check('S9 tc008 日志核对[%s]' % desc, False, '未找到日志 %s' % logp)
        return
    with open(logp, encoding='utf-8') as fh:
        evs = [_json.loads(l) for l in fh if l.strip()]
    tx = [e for e in evs if e.get('ev') == 'tx']

    def cmd_of(h):
        return int(h[4:6], 16)

    def sid_of(h):
        return int(h[8:10], 16)

    def target_of_08(h):
        raw = int(h[12:14] + h[10:12], 16)
        return (raw - 65536 if raw > 32767 else raw) / 10.0

    ssa = [e for e in evs if e.get('ev') == 'safe_stop_all']
    if expect_ssa:
        ok = bool(ssa) and ssa[-1].get('ids') == stop_ids
    else:
        ok = not ssa
    detail = ['stop_ids=%s' % (ssa[-1].get('ids') if ssa else None)]
    if no_motion_frames:
        mv = [e for e in tx if cmd_of(e['hex']) in (0x08, 0x19)]
        ok = ok and not mv
        detail.append('运动帧(0x08/0x19)=%d' % len(mv))
    if release_within is not None:
        j6_008 = [e for e in tx if cmd_of(e['hex']) == 0x08 and sid_of(e['hex']) == 6]
        j6_rel = [e for e in tx if cmd_of(e['hex']) == 0x18 and sid_of(e['hex']) == 6
                  and e['hex'][10:12] == '10']
        if j6_008 and j6_rel:
            dt = j6_rel[0]['t'] - j6_008[0]['t']
            ok = ok and dt <= release_within
            detail.append('首0x08→J6释放=%.2fs(≤%.1f)' % (dt, release_within))
        else:
            ok = False
            detail.append('缺0x08或释放帧')
    if no_motion_after_stop:
        t_stop3 = ssa[-1]['t'] if ssa else None
        mv2 = [e for e in tx if t_stop3 is not None and e['t'] > t_stop3
               and cmd_of(e['hex']) in (0x08, 0x19)]
        ok = ok and not mv2
        detail.append('stop后运动帧(0x08/0x19)=%d' % len(mv2))
    if j6_release_after_stop:
        t_stop2 = ssa[-1]['t'] if ssa else None
        rel = [e for e in tx if t_stop2 is not None and e['t'] > t_stop2 and cmd_of(e['hex']) == 0x18
               and sid_of(e['hex']) == 6 and e['hex'][10:12] == '10']
        ok = ok and bool(rel)
        detail.append('stop后J6释放帧(0x18 0x10)=%d' % len(rel))
    if no_018_j6_after_close:
        gc = [e for e in evs if e.get('ev') == 'grip_close']
        if gc:
            bad = [e for e in tx if cmd_of(e['hex']) == 0x18 and sid_of(e['hex']) == 6 and e['t'] >= gc[-1]['t']]
            ok = ok and not bad
            detail.append('close后J6的0x18帧=%d' % len(bad))
        else:
            ok = False
            detail.append('无 grip_close 事件')
    if j6_008_after_stop is not None:
        t_stop = ssa[-1]['t'] if ssa else None
        got = [target_of_08(e['hex']) for e in tx
               if t_stop is not None and e['t'] > t_stop and cmd_of(e['hex']) == 0x08 and sid_of(e['hex']) == 6]
        ok = ok and any(abs(g - j6_008_after_stop) < 1e-9 for g in got)
        detail.append('stop后J6的0x08目标=%s' % got)
    if no_008_j6_after_stop:
        t_stop = ssa[-1]['t'] if ssa else None
        got = [e for e in tx
               if t_stop is not None and e['t'] > t_stop and cmd_of(e['hex']) == 0x08 and sid_of(e['hex']) == 6]
        ok = ok and not got
        detail.append('stop后J6的0x08帧=%d' % len(got))
    check('S9 tc008 日志核对[%s]' % desc, ok, '; '.join(detail))


def s11_tc008_stats():
    """TC-008 统计口径单元测试（文件头定义可复算）：到位时刻 / 最终误差 / 6 轴极差 /
    电流峰值 / 电压最低。合成段 T=1.0s、采样 20ms、采样至 2.0s（T+SETTLE 1.0）"""
    import tc008_sync_load as t8

    def build(j1_arrive_at):
        seg = {'gear': 'slow', 'trip': 1, 'leg': 'go', 't_cmd': 100.0, 'T_ms': 1000,
               'targets': {'0': 10.0, '1': 10.0, '2': 0.0, '3': 0.0, '4': 0.0, '5': 0.0}}
        frames = []
        dt = 0.02
        for i in range(int(2.0 / dt) + 1):
            t = round(i * dt, 4)
            a0 = (10.0 * t) if t < 1.0 else 10.2            # J0 线性 0→10，停后 10.2
            a1 = 10.0 if (j1_arrive_at is not None and t >= j1_arrive_at) else 0.0
            got = {}
            for s in range(7):
                ang = {0: a0, 1: a1, 6: 33.5}.get(s, 0.0)
                got[s] = {'ang': ang, 'mA': (100 + s if t <= 1.0 else 10),
                          'mV': (11900 if t <= 0.5 else 12000), 'mW': 0, 'st': 0, 'tC': 30.0,
                          'dt_ms': 2.0 * s}
            frames.append({'t_tx': round(100.0 + t, 4), 'gear': 'slow', 'trip': 1, 'leg': 'go',
                           'ok': True, 'got': got})
        return seg, frames

    seg, fr = build(j1_arrive_at=0.5)
    st = t8.analyze_segment(seg, fr)
    a0, a1, a2 = st['axes'][0], st['axes'][1], st['axes'][2]
    check('S11 到位时刻 J0=0.900（首次|a−目标|≤1°=9.0°@0.9s 且此后保持）',
          a0['arrive_s'] is not None and abs(a0['arrive_s'] - 0.900) < 1e-6, str(a0.get('arrive_s')))
    check('S11 到位时刻 J1=0.502（0.5s 跳到目标 + dt_ms 2ms 偏移）',
          a1['arrive_s'] is not None and abs(a1['arrive_s'] - 0.502) < 1e-6, str(a1.get('arrive_s')))
    check('S11 到位时刻 J2=0.004（首帧即在窗，dt_ms 4ms）',
          a2['arrive_s'] is not None and abs(a2['arrive_s'] - 0.004) < 1e-6, str(a2.get('arrive_s')))
    check('S11 6 轴极差=0.896（0.900−0.004）',
          st['arrive_range_s'] is not None and abs(st['arrive_range_s'] - 0.896) < 1e-6,
          str(st['arrive_range_s']))
    check('S11 最终误差 J0=+0.2（段末 0.5s 均值 10.2 − 10）',
          a0['final_err'] is not None and abs(a0['final_err'] - 0.2) < 1e-9, str(a0.get('final_err')))
    check('S11 电流峰值 J0=100 / 电压最低=11900（仅 [0,T] 内）',
          a0['mA_peak'] == 100 and a0['mV_min'] == 11900, '%s/%s' % (a0['mA_peak'], a0['mV_min']))
    check('S11 J6 不下发：无 target/arrive，有角度范围 33.5~33.5',
          'target' not in st['axes'][6] and st['axes'][6].get('ang_min') == 33.5)
    seg2, fr2 = build(j1_arrive_at=None)
    st2 = t8.analyze_segment(seg2, fr2)
    b1 = st2['axes'][1]
    check('S11 J1 从未进窗 → 到位=None、末出窗=2.002、最终误差=−10',
          b1['arrive_s'] is None and abs(b1['last_out_s'] - 2.002) < 1e-6
          and abs(b1['final_err'] + 10.0) < 1e-9, '%s/%s/%s' % (b1['arrive_s'], b1.get('last_out_s'), b1['final_err']))
    check('S11 有轴未到位 → 6 轴极差=None(不可判)', st2['arrive_range_s'] is None)


def s12_tc008_guard():
    """TC-008 关键代码单元测试：guard_frame（电压 / 保护位 / J6 出窗 / J6 不可读 / 连续帧计数）"""
    import tc008_sync_load as t8

    def mk(j6=33.5, mv=12000, st=0, j6_present=True):
        got = {s: {'ang': 0.0, 'mA': 0, 'mV': mv, 'mW': 0, 'st': st, 'tC': None, 'dt_ms': 0.0}
               for s in range(6)}
        if j6_present:
            got[6] = {'ang': j6, 'mA': 0, 'mV': mv, 'mW': 0, 'st': 0, 'tC': None, 'dt_ms': 0.0}
        return {'t_tx': 0.0, 'gear': 'slow', 'trip': 1, 'leg': 'go', 'ok': j6_present, 'got': got}

    def ctx(out_frames=1, ref=33.5):
        # T6：grip_ref±grip_dev 取 33.5±0.5 → [33.0,34.0]，边界语义与旧固定窗等价
        return types.SimpleNamespace(vbat_low=11000, grip_ref=ref, grip_dev=0.5,
                                     j6_out_frames=out_frames, j6_miss=0, j6_out=0,
                                     grip_verified=False, j6_release_now=False,
                                     j6_stall_since=None, j6_stall_ma=400)
    # 以下单帧出窗即抛的断言用 out_frames=1（阈值语义测试）；默认值断言见本函数末尾

    def raises(fr, c, sub):
        try:
            t8.guard_frame(fr, c)
            return False
        except RuntimeError as e:
            return sub in str(e)

    c = ctx()
    check('S12 正常帧不抛', not raises(mk(), c, ''))
    check('S12 J6=32.9 → j6_out_of_window', raises(mk(j6=32.9), ctx(), 'j6_out_of_window'))
    check('S12 J6=34.1 → j6_out_of_window', raises(mk(j6=34.1), ctx(), 'j6_out_of_window'))
    check('S12 J6=34.0 边界在窗内不抛', not raises(mk(j6=34.0), ctx(), ''))
    c = ctx(ref=None)
    check('S12 T6 grip_ref 未设(夹瓶判定前)→出窗守卫不启用(J6=20 不抛)',
          not raises(mk(j6=20.0), ctx(ref=None), '') and not raises(mk(j6=20.0), c, ''))
    check('S12 电压 10999 → vbat_low', raises(mk(mv=10999), ctx(), 'vbat_low'))
    check('S12 状态 0x04(bit2 堵转) → status_protect', raises(mk(st=0x04), ctx(), 'status_protect'))
    check('S12 状态 0x01(bit0 未知位) 不抛', not raises(mk(st=0x01), ctx(), ''))
    c = ctx()
    miss_ok = all(not raises(mk(j6_present=False), c, '') for _ in range(4))
    check('S12 J6 缺失 4 帧不抛、第 5 帧 j6_unreadable',
          miss_ok and raises(mk(j6_present=False), c, 'j6_unreadable'))
    c = ctx()
    for _ in range(3):
        t8.guard_frame(mk(j6_present=False), c)
    t8.guard_frame(mk(), c)
    check('S12 J6 回来一帧后缺失计数清零', c.j6_miss == 0)
    c = ctx(out_frames=3)
    two_ok = all(not raises(mk(j6=30.0), c, '') for _ in range(2))
    t8.guard_frame(mk(), c)  # 回窗一次 → 计数清零
    two_again = all(not raises(mk(j6=30.0), c, '') for _ in range(2))
    check('S12 --j6-out-frames=3：出窗 2 帧不抛、回窗清零、再 2 帧不抛、第 3 帧抛',
          two_ok and c.j6_out == 2 and two_again and raises(mk(j6=30.0), c, 'j6_out_of_window'))
    # Thomas 2026-11-02 定：默认连续 3 帧才中止——只出窗 1–2 帧不中止
    c = ctx(t8.J6_OUT_FRAMES_DEFAULT)
    one_two_ok = all(not raises(mk(j6=30.0), c, '') for _ in range(2))
    check('S12 默认 J6_OUT_FRAMES_DEFAULT=3（Thomas 2026-11-02 定）：出窗 1–2 帧不中止、第 3 帧中止',
          t8.J6_OUT_FRAMES_DEFAULT == 3 and one_two_ok and raises(mk(j6=30.0), c, 'j6_out_of_window'),
          'default=%s' % t8.J6_OUT_FRAMES_DEFAULT)


def s15_j6_release_cmd():
    """T6b：keep_grip 释放命令本体在仿真总线上验证——只发 1 帧 0x18 0x10（STOP_RELEASE，
    power=0）给 J6，无任何 0x08/0x19 运动帧；发后 0x0A 读回一次"""
    import shutil
    import subprocess
    import tempfile
    import ast as _ast
    import tc008_sync_load as t8
    cmd = t8.J6_RELEASE_CMD
    ok_pre = cmd.startswith('python3 -c "') and cmd.endswith('"')
    check('S15 J6_RELEASE_CMD 形如 python3 -c "..."（可整行粘贴）', ok_pre, cmd[:60])
    if not ok_pre:
        return
    inner = cmd[len('python3 -c "'):-1]
    here = os.path.dirname(os.path.abspath(__file__))
    tmp = tempfile.mkdtemp(prefix='p04_s15_')
    try:
        shutil.copy(os.path.join(here, 'fsbus.py'), tmp)
        shutil.copy(os.path.join(here, 'tc007_stream.py'), tmp)
        with open(os.path.join(tmp, 'serial.py'), 'w') as fh:
            fh.write(SIM_SOURCE)
        runner = (
            "import sys\n"
            "sys.path.insert(0, '')\n"
            "import fsbus, tc007_stream\n"
            "calls = {'stop': [], 'move': []}\n"
            "orig_stop = fsbus.Bus.stop\n"
            "def stop(self, sid, mode, power):\n"
            "    calls['stop'].append((sid, mode, power))\n"
            "    return orig_stop(self, sid, mode, power)\n"
            "fsbus.Bus.stop = stop\n"
            "for _m in ('set_angle', 'sync_set_angle', 'send', 'clear_turns',\n"
            "           'lock_all', 'safe_stop_all'):\n"
            "    def _mk(_m):\n"
            "        def f(self, *a, **k):\n"
            "            calls['move'].append((_m,) + tuple(a))\n"
            "        return f\n"
            "    setattr(fsbus.Bus, _m, _mk(_m))\n"
            "exec(%r)\n"
            "print('CALLS', calls)\n" % inner
        )
        with open(os.path.join(tmp, 'runner.py'), 'w') as fh:
            fh.write(runner)
        env = dict(os.environ)
        env['P04_SKIP_PREFLIGHT'] = '1'
        env['P04_SIM_MODE'] = 'normal'
        r = subprocess.run([sys.executable, 'runner.py'], cwd=tmp, env=env,
                           capture_output=True, text=True, timeout=60)
        out = r.stdout + r.stderr
        calls = None
        for line in out.splitlines():
            if line.startswith('CALLS '):
                calls = _ast.literal_eval(line[len('CALLS '):])
        # 帧级断言：send 记录 = 低层原始帧（stop/read_ang 都经 send）。要求：
        #   0x18 帧恰 1 条（sid=6, mode=0x10, power=0）；其余 send 全是 0x0A 读；无 0x08/0x19
        ok = (r.returncode == 0 and 'J6 释放帧已发(0x18 0x10)' in out and calls is not None
              and calls['stop'] == [(6, fsbus.STOP_RELEASE, 0)])
        if ok and calls is not None:
            f18 = [c for c in calls['move'] if c[1] == 0x18]
            oth = [c for c in calls['move'] if c[1] != 0x18]
            bad = [c for c in calls['move'] if c[1] in (0x08, 0x19)]
            ok = (len(f18) == 1 and f18[0][2][:4] == b'\x06\x10\x00\x00'
                  and all(c[1] == 0x0A for c in oth) and not bad)
            detail = '' if ok else ('18帧=%s 其他=%s' % (f18, oth))
        else:
            detail = ('rc=%d calls=%s err=%s' % (r.returncode, calls, out.splitlines()[-1][:120]))
        check('S15 释放命令仿真执行：仅 1 帧 0x18 0x10(sid6,power=0)+0x0A 读回，无 0x08/0x19',
              ok, detail)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def s10_lock_all():
    """T12 关键代码单元测试（交 审查方 审的两段）：lock_all / safe_stop_all。
    场景：a) 7 帧 0x18 参数逐帧正确；b) 全轴静止核实 still 全 True；
    c) 某轴读不到 → still=None 如实报告；d) 单帧发送失败 → sent<7 如实返回"""
    import struct as st2
    # a) lock_all：7 帧 0x11@6000，sid 顺序与参数逐帧正确
    bus, fake = make_bus('manual', min_gap_ms=0)
    sent = bus.lock_all(range(7), 6000, mode=fsbus.STOP_HOLD)
    frames = [f for (_t, f) in fake.tx_log if len(f) >= 8 and f[2] == 0x18]
    ok_a = (sent == 7 and len(frames) == 7)
    for i, f in enumerate(frames):
        sid, mode, pw = f[4], f[5], st2.unpack('<H', f[6:8])[0]
        ok_a = ok_a and (sid == i and mode == 0x11 and pw == 6000)
    check('S10 lock_all 7帧0x11@6000 参数逐帧正确', ok_a,
          'sent=%d frames=%d' % (sent, len(frames)))
    # b) safe_stop_all：instant 应答静止角 10.0° → still 全 True
    bus2, fake2 = make_bus('instant', min_gap_ms=0)
    r = bus2.safe_stop_all(range(7), 6000, repeats=1)
    ok_b = (r['still'] == {i: True for i in range(7)}) and (r['sent'] == 7)
    check('S10 safe_stop_all 静止读数→still 全 True', ok_b,
          'still=%s sent=%d' % (r['still'], r['sent']))
    # c) 某轴读不到（fail_read_sid=1 注入）→ still=None 如实出现，另一轴正常
    bus3, fake3 = make_bus('instant', min_gap_ms=0)
    fake3.fail_read_sid = 1
    r3 = bus3.safe_stop_all([0, 1], 6000, repeats=1)
    ok_c = (r3['still'].get(0) is True and r3['still'].get(1) is None)
    check('S10 safe_stop_all 某轴读失败→still=None 如实报告', ok_c,
          'still=%s' % r3['still'])
    # d) lock_all 单帧发送失败（第 3 帧注入 IOError）→ sent<7、其余轴仍发出
    bus4, fake4 = make_bus('manual', min_gap_ms=0)
    orig_write = fake4.write
    cnt = [0]

    def flaky_write(b):
        cnt[0] += 1
        if cnt[0] == 3:
            raise IOError('sim write fail')
        return orig_write(b)
    fake4.write = flaky_write
    sent4 = bus4.lock_all(range(7), 0, mode=fsbus.STOP_HOLD)
    frames4 = [f for (_t, f) in fake4.tx_log if len(f) >= 8 and f[2] == 0x18]
    ok_d = (sent4 == 6 and len(frames4) == 6)
    check('S10 lock_all 单帧失败→sent=6 如实返回', ok_d,
          'sent=%d frames=%d' % (sent4, len(frames4)))
    # e) T12c：分轴速度表 = Thomas 选定值（URDF 表 A②，J6 同型号推定）
    import tc007_stream as t7
    ok_e = (t7.AXIS_SPEEDS == {0: 60.2, 1: 60.2, 2: 60.2,
                               3: 99.1, 4: 99.1, 5: 99.1, 6: 99.1})
    check('S10 分轴速度表=URDF表A②(J0-J2 60.2/J3-J6 99.1)', ok_e, str(t7.AXIS_SPEEDS))


def s13_tc009_stats():
    """TC-009 统计口径单元测试（文件头定义可复算）：停住角 / 稳态电流 / 峰值及时刻 /
    接触到位时刻（100 ms 前向窗、5°/s）/ 未检出运动。合成：64.5→33.5 匀速 20.0°/s（31.0°÷1.55 s）至 1.55 s
    后静止，电流运动中 100、停住后 550，采样 20 ms，全程 4 s"""
    import tc009_grip_char as t9
    dt, T = 0.02, 4.0
    rows = []
    for i in range(int(T / dt) + 1):
        t = round(i * dt, 4)
        ang = 64.5 - min(t, 1.55) * 20.0
        ang = round(max(ang, 33.5), 2)
        rows.append({'trial': 1, 'kind': 'bottle', 'label': '中', 'P': 0, 't_s': t, 'ang': ang,
                     'mA': (100 if t < 1.55 else 550), 'mW': (1200 if t < 1.55 else 6600),
                     'mV': 12000, 'st': 0, 'tC': 30.0, 'rtt_ms': 11.0})
    rec = {'trial': 1, 'kind': 'bottle', 'label': '中', 'P': 0, 'dur_s': T, 'close_ms': 1775}
    st = t9.analyze_trial(rec, rows)
    check('S13 停住角=33.5（最后 1s 中位）', st['ang_hold'] == 33.5, str(st['ang_hold']))
    check('S13 稳态电流=550 极差=0（最后 1s）',
          st['mA_hold'] == 550 and st['mA_hold_range'] == 0, '%s/%s' % (st['mA_hold'], st['mA_hold_range']))
    check('S13 峰值电流=550 且时刻≥1.55', st['mA_peak'] == 550 and st['t_peak'] >= 1.55, str(st['t_peak']))
    check('S13 接触/到位时刻≈1.55（窗起点，容差 0.06）',
          st['t_stop'] is not None and abs(st['t_stop'] - 1.55) <= 0.06, str(st['t_stop']))
    check('S13 稳态功率=6600（最后 1s 中位）', st['mW_hold'] == 6600, str(st['mW_hold']))
    static = [dict(r, trial=2, ang=29.0) for r in rows]
    st2 = t9.analyze_trial(dict(rec, trial=2), static)
    check('S13 静止试次：未检出运动起始→接触/到位=None', st2['t_onset'] is None and st2['t_stop'] is None)


def s14_t4b_units():
    """T4b v2 单元：clear_turns 帧序（先 0x18 释放再 0x11）、read_raw_multiturn 原始透传、
    mturn_out、旧阈值分类边界、t_contact_I、calib_fit"""
    import tc009_grip_char as t9
    # a) clear_turns：两帧 = 0x18(mode 0x10, sid6) → 0x11(sid6)，0x11 帧逐字节
    bus, fake = make_bus('manual', min_gap_ms=0)
    bus.clear_turns(6)
    f18 = [f for (_t, f) in fake.tx_log if len(f) >= 6 and f[2] == 0x18]
    f11 = [f for (_t, f) in fake.tx_log if len(f) >= 5 and f[2] == 0x11]
    ok_a = (len(f18) == 1 and f18[0][4] == 6 and f18[0][5] == 0x10
            and len(f11) == 1 and f11[0].hex() == '124c11010676')
    check('S14 clear_turns=0x18释放→0x11清圈（帧序与字节）', ok_a,
          '0x18=%s 0x11=%s' % ([f.hex() for f in f18], [f.hex() for f in f11]))
    bus.close()
    # b) read_raw_multiturn：预装 angle=388.6/turn=1 的 0x16 帧 → 原样透传 + mturn_out=True
    bus2, fake2 = make_bus('manual', resp_timeout=0.1)
    params = struct.pack('<BHHHHBih', 6, 12000, 30, 240, 1750, 0, 3886, 1)
    fake2._pending.extend(make_body(0x16, params))
    m = fsbus.read_raw_multiturn(bus2, 6)
    ok_b = (m is not None and abs(m['angle'] - 388.6) < 1e-9 and m['turns'] == 1
            and fsbus.mturn_out(m) and not fsbus.mturn_out({'angle': 28.6, 'turns': 0}))
    check('S14 read_raw_multiturn 388.6/t1 透传且 mturn_out 判定', ok_b,
          '' if ok_b else str(m))
    bus2.close()
    # c) 旧阈值分类边界：err=+1→empty（|err|≤1），+1.5→uncertain，+2→grasped，−0.5→empty
    def cls(err):
        return ('empty' if abs(err) <= t9.OLD_EMPTY_ABS
                else ('grasped' if err >= t9.OLD_GRASP_MIN else 'uncertain'))
    check('S14 旧阈值分类边界(+1 empty/+1.5 uncertain/+2 grasped/−0.5 empty)',
          cls(1.0) == 'empty' and cls(1.5) == 'uncertain' and cls(2.0) == 'grasped'
          and cls(-0.5) == 'empty')
    # d) t_contact_I：基线 30，t=1.2 起持续 200 → Δ100/200 命中 1.2，Δ300 未检出
    rows = []
    for i in range(int(3.0 / 0.02) + 1):
        t = round(i * 0.02, 4)
        rows.append({'trial': 1, 'kind': 'bottle', 'label': '', 'P': 0, 't_s': t,
                     'ang': 40.0 - min(t, 1.2) * 8.0, 'mA': (30 if t < 1.2 else 260),
                     'mW': 300, 'mV': 12000, 'st': 0, 'tC': 30.0, 'rtt_ms': 11.0})
    st = t9.analyze_trial({'trial': 1, 'kind': 'bottle', 'label': '', 'P': 0, 'dur_s': 3.0,
                           'start_ang': 40.0, 'start_mA': 30}, rows)
    ok_d = (abs(st['t_contact_I'][100] - 1.2) < 0.03 and abs(st['t_contact_I'][200] - 1.2) < 0.03
            and st['t_contact_I'][300] is None and st['old_class'] == 'uncertain'
            and abs(st['old_err'] - 1.4) < 0.05 and abs(st['ang_hold'] - 30.4) < 0.2)
    check('S14 t_contact_I 候选门限对照与停住角', ok_d,
          'tCI=%s ang_hold=%s' % (st['t_contact_I'], st['ang_hold']))
    # e) calib_fit：精确直线 mm=0.85×ang+19.8 → a/b 复原、残差全 0
    calib = [{'pt': a, 'read_ang': a, 'mm': round(0.85 * a + 19.8, 2)}
             for a in (64.5, 50, 40, 29, 20, 10, 0, -30)]
    fit = t9.calib_fit(calib)
    ok_e = (abs(fit['a'] - 0.85) < 1e-3 and abs(fit['b'] - 19.8) < 0.05
            and all(abs(r) <= 0.05 for r in fit['residuals_mm']))
    check('S14 calib_fit 直线复原与残差', ok_e, 'a=%s b=%s res=%s' % (fit['a'], fit['b'], fit['residuals_mm']))


def main():
    print('fsbus_selftest —— 离线自检（无实机、无 pyserial 依赖；S9 需数分钟）')
    s1_frames()
    s2_read_frame()
    s3_lengths()
    s4_sync()
    s5_min_gap()
    s6_norm()
    s7_speed_stats()
    s8_triangle_excl()
    s10_lock_all()
    s11_tc008_stats()
    s12_tc008_guard()
    s15_j6_release_cmd()
    s13_tc009_stats()
    s14_t4b_units()
    s9_main_flows()
    print('====')
    if FAILS:
        print('结果：FAIL %d 项：%s' % (len(FAILS), FAILS))
        sys.exit(1)
    print('结果：全部通过')


if __name__ == '__main__':
    main()
