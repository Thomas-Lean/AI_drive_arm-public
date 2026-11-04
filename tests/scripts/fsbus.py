#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fsbus.py —— FashionStar 总线舵机探针基础库（P0.4，v2 按复审修订）

- 自包含：仅依赖 python3 标准库 + pyserial（臂载计算机已具备，F18）。
  不依赖 PyPI fashionstar-uart-sdk（臂上未确认安装；且其 update() 有已知缺陷，
  见 docs/hardware/Servo Motor/PROTOCOL_NOTES.md「SDK 缺陷」节）。
- 帧格式（PROTOCOL_NOTES.md，示例校验和均已本地复算一致）：
    请求  12 4C | cmd | len | params | cs     cs = 前序字节累加 % 256
    响应  05 1C | cmd | len | params | cs
- 角度单位：API 一律用「度」，线上单位 0.1°，打包时 ×10 —— 与臂上 C++ 库
  setRawAngle 语义一致（FashionStar_UartServoProtocol.cpp:241；厂商库
  FSUS_K_ANGLE_REAL2RAW=1，即 C++ 层的 raw 就是度）。
- 回包特性（说明书 §5.1 指令表，页 12）：0x01/0x03/0x0A/0x10/0x16 固定回包；
  0x19/0x12/0x13 无回包；0x08/0x09/0x18 属可配置响应类，默认不回包（不写
  「绝对无回包」——是否回包取决于舵机内响应开关配置，F 响应开关 data_id 33）。
- 舵机 ID = 关节号 J0–J6（marm_controller_node.cpp:37-44 实证）。
- 串口独占：打开即要求操作系统级独占（exclusive），失败直接抛错退出，
  **不降级**（复审 H1）；独占成功也不能代替占用检查——运动类脚本须先跑
  preflight()（4 服务 inactive + ttyS4 无占用），见 tc006/tc007。
- 最小帧间隔：send() 内置总线空闲调度 min_gap_ms（默认 10 ms，F14 官方建议
  ≥10 ms），正常流程全部生效；唯一例外 = tc005 阶段 A/B 的定频间隔实验
  （min_gap_ms=0，实验本身就是去测这个间隔的），该例外在用例文档中明示。
- verdict 词表（复审 M1/M2）：ok / bad_len / corrupt / cmd_mismatch / id_mismatch /
  timeout（调用方收到 None）。响应帧无序号：同命令同 ID 的迟到响应与当次响应
  在协议层不可辨识——本库以「接收窗口内的响应归属当次请求」为记法，
  rtt 超出等待上限者由离线分析按疑似迟到另行归类，不作超范围的归因结论。
- 时间：全部间隔用 time.monotonic()（相对秒）。板载时钟停在 2023-11（F18），
  日志里的 wall 字段仅作参考，真实时间以 tests/runs/ 的 README 登记为准。
"""

import json
import math
import os
import struct
import subprocess
import time

import serial

REQ_HEADER = b'\x12\x4c'  # 指令帧头
RSP_HEADER = b'\x05\x1c'  # 响应帧头

# 功能码（PROTOCOL_NOTES.md；说明书 §5.1 指令表；uservo.py:231-254 同）
CMD_PING = 0x01          # 舵机检测（固定回包）
CMD_SET_ANGLE = 0x08     # 简易单圈角度控制（可配置响应，默认不回包）
CMD_READ_ANGLE = 0x0A    # 单圈角度读取（固定回包）
CMD_MONITOR = 0x16       # 数据监控（固定回包，响应 21 字节）
CMD_STOP = 0x18          # 停止（可配置响应，默认不回包）
CMD_SYNC = 0x19          # 同步指令（无回包；子指令 0x16 时各舵机各自回 0x16 响应）
CMD_CLEAR_MTURN = 0x11   # 清除多圈圈数（说明书 HX8 §6.4.1；与 0x18 的模式参数 0x11 无关）

# 0x18 停止模式（PROTOCOL_NOTES §0x18 / 说明书 §6.6 页 17，F13）
STOP_RELEASE = 0x10   # 失去锁力：立即停止运动并释放锁力
STOP_HOLD = 0x11      # 保持锁力：立即停止运动并维持锁力（失锁状态可重建锁力）
STOP_DAMPING = 0x12   # 保持阻尼：立即停止运动并进入阻尼模式

# 臂上实际运行的关节限位（度，F7；索引 = 关节号 J0–J6）
JOINT_LIMITS = {
    0: (-163.0, 163.0), 1: (-90.3, 97.0), 2: (-97.0, 89.0), 3: (-166.0, 163.0),
    4: (-96.0, 113.0), 5: (-166.0, 162.0), 6: (-30.0, 64.5),
}

# 0x16 状态位（说明书 §4 页 11：bit2 堵转、bit6 功率保护；PROTOCOL_NOTES 同）
# bit0/1 含义未知（官方 0x16 示例状态=1，即 bit0 置位属正常样本），标未知
STATUS_BITS = {2: '堵转', 3: '电压', 4: '电压', 5: '电流', 6: '功率', 7: '温度'}
KNOWN_STATUS_MASK = 0b11111100

# 0x16 角度无效值哨兵（uservo.py:709，PROTOCOL_NOTES「SDK 缺陷」节）
_SENTINEL_ANGLE = -235929599

MARM_SERVICES = ('marm_controller', 'marm_communication', 'marm_control', 'marm_subscripber')


def temp_adc_to_celsius(adc):
    """NTC 温度换算（uservo.py:695-702；PROTOCOL_NOTES 本地复算 ADC1191→50.0℃ 已验证）"""
    if adc <= 0 or adc >= 4096:
        return None
    return 1.0 / (math.log(adc / (4096.0 - adc)) / 3435.0 + 1.0 / 298.15) - 273.15


def decode_status(st):
    """状态字节 → 已知置位标志 + 未知位提示（bit0/1 未定义，照实标出）"""
    flags = ['bit%d(%s)' % (b, name) for b, name in sorted(STATUS_BITS.items()) if (st >> b) & 1]
    unknown = st & ~KNOWN_STATUS_MASK & 0xFF
    if unknown:
        flags.append('未知位0x%02x【未验证】' % unknown)
    return flags


def norm_single_turn(deg):
    """任意角度 → 单圈 [-180,180)。0x16 的角度是 u32+圈数口径，0x0A 是单圈
    int16 口径；两者比较前必须归一化（口径换算本身【待实测核对】，
    归一化后仍不一致按口径问题记录，不判硬件失败——复审 M7）"""
    return ((deg + 180.0) % 360.0) - 180.0


def clamp_angle(sid, deg):
    """按 F7 关节限位 + 协议 ±180° 双重夹紧"""
    lo, hi = JOINT_LIMITS.get(sid, (-180.0, 180.0))
    return max(-180.0, min(180.0, max(lo, min(hi, deg))))


def deg_to_raw(deg):
    """度 → 线上 0.1° int16"""
    return max(-1800, min(1800, int(round(deg * 10))))


def preflight(port='/dev/ttyS4'):
    """开串口前的占用检查（复审 H1/H-d）：4 个 marm 服务必须全部 inactive，
    且 ttyS4 上无任何进程。fuser 返回码 1 且 stderr 为空才算无占用；
    返回码 1 但 stderr 非空（如 Cannot stat）按「无法确定」处理并退出。
    检查失败或无法确定 → (False, 原因)，调用方必须退出。
    只读检查（systemctl is-active / fuser），不改变任何状态。
    环境变量 P04_SKIP_PREFLIGHT=1 且 serial 模块带 SIMULATED=True 标记（仿真件）时
    才允许跳过，仅供 fsbus_selftest 离线仿真；真机即使误设该变量也照常执行检查。"""
    msgs = []
    if os.environ.get('P04_SKIP_PREFLIGHT') == '1' and getattr(serial, 'SIMULATED', False):
        return True, ['preflight 跳过（P04_SKIP_PREFLIGHT=1 且 serial 为仿真件，仅限离线自检）']
    try:
        out = subprocess.run(['systemctl', 'is-active'] + list(MARM_SERVICES),
                             capture_output=True, text=True, timeout=10)
        states = [s.strip() for s in out.stdout.splitlines()]
    except Exception as e:
        return False, ['服务状态无法确定（%s）' % e]
    if len(states) != 4 or any(s != 'inactive' for s in states):
        return False, ['服务状态非全 inactive：%s' % states]
    msgs.append('服务 4×inactive')
    try:
        out = subprocess.run(['fuser', '-v', port], capture_output=True, text=True, timeout=10)
        if out.returncode == 0:  # 0 = 有进程占用
            return False, ['%s 仍被占用：%s%s' % (port, out.stdout.strip(), out.stderr.strip())]
        if out.returncode == 1 and out.stderr.strip():
            # H-d：rc=1 但 stderr 非空（如 Cannot stat …）= 无法确定，不得放行
            return False, ['fuser 返回 1 且 stderr 非空，占用状态无法确定：%s' % out.stderr.strip()]
        if out.returncode != 1:  # 其他返回码 = 无法确定
            return False, ['fuser 返回码 %d，占用状态无法确定' % out.returncode]
    except FileNotFoundError:
        return False, ['fuser 不可用，占用状态无法确定']
    except Exception as e:
        return False, ['fuser 执行失败（%s）' % e]
    msgs.append('%s 无占用' % port)
    return True, msgs


def _try_extract(buf):
    """从缓冲提取一帧响应。返回 (frame_or_None, consumed)。
    - 找不到响应头：丢掉垃圾、只保留最后 1 字节（可能是 0x05）
    - 帧不完整：(None, 0)
    - 完整：({'raw','cmd','params','checksum_ok'}, 帧长)；校验错也整帧交出，
      由调用方计 corrupt（粘包天然支持：一次提取一帧，剩余留在缓冲）"""
    i = bytes(buf).find(RSP_HEADER)
    if i < 0:
        return None, max(0, len(buf) - 1)
    if i > 0:
        return None, i
    if len(buf) < 4:
        return None, 0
    size = buf[3]
    total = 4 + size + 1
    if len(buf) < total:
        return None, 0
    raw = bytes(buf[:total])
    ok = (sum(raw[:-1]) & 0xFF) == raw[-1]
    return {'raw': raw, 'cmd': raw[2], 'params': raw[4:4 + size], 'checksum_ok': ok}, total


def _decode_monitor_params(params):
    """0x16 响应参数 16 字节 → 字典（<BHHHHBih，PROTOCOL_NOTES §0x16）。
    valid_measure=False 表示角度/圈数为无效哨兵，不得进入角度/速度统计（复审 M1）"""
    i, v, c, p, adc, st, ang, turn = struct.unpack('<BHHHHBih', params)
    sentinel = (ang == _SENTINEL_ANGLE and turn == 0)
    return {
        'servo_id': i, 'voltage_mV': v, 'current_mA': c, 'power_mW': p,
        'temp_adc': adc, 'temp_c': temp_adc_to_celsius(adc), 'status': st,
        'status_flags': decode_status(st), 'angle': ang / 10.0, 'turns': turn,
        'sentinel': sentinel, 'valid_measure': (not sentinel),
    }


class Bus(object):
    """独占 /dev/ttyS4 的半双工总线事务层，带 JSONL 事件日志与耗时打点"""

    def __init__(self, port='/dev/ttyS4', baud=115200, log_path=None,
                 resp_timeout=0.03, min_gap_ms=10):
        # 独占打开，失败即抛错（复审 H1：不降级）。调用方负责先 preflight()。
        self.ser = serial.Serial(port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                 timeout=0.002, write_timeout=1.0, exclusive=True)
        self.port = port
        self.resp_timeout = resp_timeout
        self.min_gap_ms = min_gap_ms
        self.txn_late = 0  # 最近一次 transact 超时后的静默窗迟到帧数（M-a）
        self._last_tx_end = None
        self._buf = bytearray()
        self._logf = open(log_path, 'a', encoding='utf-8') if log_path else None
        self.t0 = time.monotonic()
        self.log('open', port=port, baud=baud, min_gap_ms=min_gap_ms, wall=time.time())

    # ---------- 基础 ----------

    def log(self, ev, **kw):
        if not self._logf:
            return
        rec = {'ev': ev, 't': round(time.monotonic() - self.t0, 6)}
        rec.update(kw)
        self._logf.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self._logf.flush()

    def close(self):
        self.log('close')
        if self._logf:
            self._logf.close()
        self.ser.close()

    def drain(self):
        self.ser.reset_input_buffer()
        self._buf.clear()

    @staticmethod
    def build(cmd, params=b''):
        head = REQ_HEADER + bytes([cmd, len(params)]) + params
        return head + bytes([sum(head) & 0xFF])

    def send(self, cmd, params=b''):
        """发送一帧。min_gap_ms > 0 时，距上一帧发送完成不足间隔的先睡足
        （正常流程统一 ≥10ms 帧间隔，复审 M4；实际发送时刻记入 tx 事件）"""
        if self.min_gap_ms and self._last_tx_end is not None:
            d = self.min_gap_ms / 1000.0 - (time.monotonic() - self._last_tx_end)
            if d > 0:
                time.sleep(d)
        frame = self.build(cmd, params)
        t0 = time.monotonic()
        self.ser.write(frame)
        self.ser.flush()
        t1 = time.monotonic()
        gap_ms = round((t0 - self._last_tx_end) * 1000, 3) if self._last_tx_end is not None else None
        self._last_tx_end = t1
        self.log('tx', cmd=cmd, hex=frame.hex(), t_tx=round(t1 - self.t0, 6), gap_ms=gap_ms)
        return {'t_start': t0, 't_end': t1, 'frame': frame}

    def read_frame(self, timeout=None):
        """读到一帧响应（含校验错的帧，交调用方判定）；超时返回 None。
        所有读到的字节一律进解析缓冲（复审 B1：短阻塞 read 的字节不得丢弃）"""
        timeout = self.resp_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while True:
            if time.monotonic() >= deadline:
                return None
            n = self.ser.in_waiting
            data = self.ser.read(n if n else 1)  # 无数据时短阻塞（ser.timeout=2ms）
            if data:
                self._buf.extend(data)
            while self._buf:
                frame, consumed = _try_extract(self._buf)
                if consumed:
                    del self._buf[:consumed]
                if frame is not None:
                    t = time.monotonic()
                    self.log('rx', cmd=frame['cmd'], ok=frame['checksum_ok'],
                             hex=frame['raw'].hex(), t_rx=round(t - self.t0, 6))
                    frame['t_rx'] = t
                    return frame
                if consumed == 0:
                    break  # 帧不完整，继续收字节

    def wait_quiet(self, window_s=0.01, deadline_s=0.2):
        """超时后的静默恢复窗（复审 M2）：窗口内还在到达的完整帧记录为迟到
        （ev=late_rx）并丢弃——不进任何统计。静默判据 = window_s 内无完整帧
        （不等于字节级空闲；零散字节以 late_drain 事件记录后清空，如实留痕）"""
        n_frames = 0
        end_all = time.monotonic() + deadline_s
        while time.monotonic() < end_all:
            f = self.read_frame(timeout=window_s)
            if f is None:
                break
            n_frames += 1
            self.log('late_rx', cmd=f['cmd'], hex=f['raw'].hex(), ok=f['checksum_ok'])
        if self.ser.in_waiting or self._buf:
            try:
                rest = self.ser.read(max(1, self.ser.in_waiting))
                self.log('late_drain', hex=bytes(rest).hex(), buf_left=len(self._buf))
            except Exception:
                pass
            self.drain()
        return n_frames

    def transact(self, cmd, params=b'', timeout=None, expect_len=None):
        """发一帧 + 等一帧。返回帧 dict（附 rtt/verdict）或 None（超时）。
        expect_len：该命令响应 params 的精确长度；不符 → verdict=bad_len（M1）。
        超时后统一在此调 wait_quiet 开静默窗（M-a：迟到帧集中处理，不散落
        到各脚本），迟到帧数记入 self.txn_late 供调用方统计。
        verdict: ok / bad_len / corrupt / cmd_mismatch"""
        tx = self.send(cmd, params)
        frame = self.read_frame(timeout)
        if frame is None:
            self.txn_late = self.wait_quiet(0.01)
            self.log('txn', cmd=cmd, verdict='timeout', late=self.txn_late)
            return None
        self.txn_late = 0
        frame['rtt'] = frame['t_rx'] - tx['t_start']
        if not frame['checksum_ok']:
            frame['verdict'] = 'corrupt'
        elif frame['cmd'] != cmd:
            frame['verdict'] = 'cmd_mismatch'
        elif expect_len is not None and len(frame['params']) != expect_len:
            frame['verdict'] = 'bad_len'
        else:
            frame['verdict'] = 'ok'
        self.log('txn', cmd=cmd, verdict=frame['verdict'], rtt_ms=round(frame['rtt'] * 1000, 3))
        return frame

    # ---------- 指令封装 ----------

    def ping(self, sid, timeout=None):
        f = self.transact(CMD_PING, bytes([sid]), timeout, expect_len=1)
        if f and f['verdict'] == 'ok':
            f['echo_id'] = f['params'][0]
            if f['echo_id'] != sid:
                f['verdict'] = 'id_mismatch'
        return f

    def read_angle(self, sid, timeout=None):
        """0x0A 单圈角度读取（官方控制器现行反馈通路用的同款指令）"""
        f = self.transact(CMD_READ_ANGLE, bytes([sid]), timeout, expect_len=3)
        if f and f['verdict'] == 'ok':
            i, a = struct.unpack('<Bh', f['params'])
            f['servo_id'], f['angle'] = i, a / 10.0
            if i != sid:
                f['verdict'] = 'id_mismatch'
        return f

    def read_monitor(self, sid, timeout=None):
        """0x16 单舵机数据监控：电压/电流/功率/温度ADC/状态/角度/圈数。
        角度为多圈口径（u32+turns），与 0x0A 比较须先 norm_single_turn（M7）"""
        f = self.transact(CMD_MONITOR, bytes([sid]), timeout, expect_len=16)
        if f and f['verdict'] == 'ok':
            f.update(_decode_monitor_params(f['params']))
            if f['servo_id'] != sid:
                f['verdict'] = 'id_mismatch'
        return f

    def set_angle(self, sid, angle_deg, interval_ms, power_mw=0):
        """0x08 简易单圈角度控制（可配置响应，默认不回包，发完即走）。
        power_mw=0 = 按舵机保护阈值执行——该 0 功率语义为 0x08 的协议页说明，
        与官方控制器一致（marm_controller_node.cpp:207）"""
        raw = deg_to_raw(clamp_angle(sid, angle_deg))
        self.send(CMD_SET_ANGLE, struct.pack('<BhHH', sid, raw, int(interval_ms), int(power_mw)))
        return raw

    def stop(self, sid, mode, power_mw):
        """0x18 停止（可配置响应，默认不回包）。
        power 语义：官方通用协议页 §16「停止指令」字段备注称 power=0 或大于功率
        保护阈值均按保护阈值执行——**该说法【待实测验证】**，本机行为以 TC-006
        两轮 power 对比实测为准（实测结果才进 FACTS）；说明书 §6.6（页 17）只列
        三种动作模式。具体取值由用例按 审查方 确认传入"""
        self.send(CMD_STOP, struct.pack('<BBH', sid, mode, int(power_mw)))

    def safe_stop(self, sid, power_mw, mode=STOP_HOLD, repeats=3):
        """异常/中断收尾（复审 M3）：尽量可发送地重发停止帧，再读角核实。
        返回 {'sent': n, 'angles': [...], 'still': True/False/None}；
        still=None 表示读数不足无法判——此时如实上报，不得宣称已停住。"""
        sent = 0
        for _ in range(repeats):
            try:
                self.wait_quiet(0.005, deadline_s=0.02)
                self.stop(sid, mode, power_mw)
                sent += 1
                time.sleep(0.05)
            except Exception as e:
                self.log('safe_stop_error', err=str(e))
                break
        angles = []
        try:
            for _ in range(3):
                f = self.read_angle(sid, timeout=0.05)
                if f and f['verdict'] == 'ok':
                    angles.append(f['angle'])
                time.sleep(0.15)
        except Exception as e:
            self.log('safe_stop_read_error', err=str(e))
        if len(angles) >= 2:
            still = (max(angles) - min(angles)) <= 0.5
        else:
            still = None
        self.log('safe_stop', sent=sent, angles=angles, still=still)
        return {'sent': sent, 'angles': angles, 'still': still}

    # ---------- 全轴锁力（TC-007 逐轴测试用，关键代码·交 审查方 审） ----------

    def lock_all(self, ids, power_mw, mode=STOP_HOLD):
        """对 ids 内每个舵机发一帧 0x18 保持锁力（T12 派单：示教「先锁后读」、
        中止「全轴保持锁力」的共用原语）。帧间隔由 send() 的 min_gap 统一保证。
        返回发送成功的帧数（= len(ids) 为全部成功）；单帧发送异常时记日志、
        继续发其余轴——部分失败必须由调用方核对返回值并如实上报。"""
        sent = 0
        for sid in ids:
            try:
                self.stop(sid, mode, power_mw)
                sent += 1
            except Exception as e:
                self.log('lock_all_error', sid=sid, err=str(e))
        self.log('lock_all', ids=list(ids), mode=mode, power_mw=power_mw, sent=sent)
        return sent

    def safe_stop_all(self, ids, power_mw, mode=STOP_HOLD, repeats=3):
        """safe_stop 的全轴版（T12 关键代码）：对 ids 内全部舵机重发 0x18（每轮
        lock_all 一遍，共 repeats 轮），再逐轴读角核实是否静止。
        返回 {'sent': 停止帧总数, 'angles': {sid: [...]}, 'still': {sid: True/False/None}}；
        任何轴 sent 不足 repeats×len(ids) 或 still 非 True 都必须如实上报，
        不得宣称「全部已锁住」。单轴读失败不阻塞其余轴的核实。"""
        sent = 0
        for _ in range(repeats):
            try:
                self.wait_quiet(0.005, deadline_s=0.02)
                sent += self.lock_all(ids, power_mw, mode=mode)
                time.sleep(0.05)
            except Exception as e:
                self.log('safe_stop_all_error', err=str(e))
                break
        angles = {sid: [] for sid in ids}
        for sid in ids:
            try:
                for _ in range(3):
                    f = self.read_angle(sid, timeout=0.05)
                    if f and f['verdict'] == 'ok':
                        angles[sid].append(f['angle'])
                    time.sleep(0.15)
            except Exception as e:
                self.log('safe_stop_all_read_error', sid=sid, err=str(e))
        still = {}
        for sid in ids:
            a = angles[sid]
            still[sid] = ((max(a) - min(a)) <= 0.5) if len(a) >= 2 else None
        self.log('safe_stop_all', ids=list(ids), sent=sent,
                 n_angles={sid: len(a) for sid, a in angles.items()}, still=still)
        return {'sent': sent, 'angles': angles, 'still': still}

    def clear_turns(self, sid, stop_power=0):
        """S1（v2）：清多圈圈数——**内部先发 0x18 释放帧（0x10）**，再发 0x11
        （帧 12 4C 11 01 <sid>，说明书 HX8 §6.4.1；2026-11-03 实机验证生效）。
        本方法不读回验证，由调用方 read_raw_multiturn 确认。返回是否两帧都发出。"""
        self.stop(sid, STOP_RELEASE, stop_power)
        time.sleep(0.1)
        self.send(CMD_CLEAR_MTURN, bytes([sid]))
        return True

    def sync_set_angle(self, items):
        """0x19 同步 0x08（无回包）。items = [(sid, angle_deg, interval_ms, power_mw), ...]
        帧格式与 PROTOCOL_NOTES §0x19 示例、uservo.py:746-762 一致（d=7）"""
        params = struct.pack('<BBB', CMD_SET_ANGLE, 7, len(items))
        for sid, ang, itv, pw in items:
            raw = deg_to_raw(clamp_angle(sid, ang))
            params += struct.pack('<BhHH', sid, raw, int(itv), int(pw))
        self.send(CMD_SYNC, params)

    def sync_monitor(self, ids, timeout=0.1):
        """0x19+0x16 d=1 同步数据监控（只读）：一帧询问 n 个舵机，
        各舵机各自回 21 字节 0x16 响应（0x16 固定回包）。
        成功标准 = **请求的 ID 集合**全部收到且长度/校验合格（复审 M1）；
        非请求 ID、重复帧、坏帧只计数不充数。返回：
        {'got': {id: 解码dict}, 'extra': n, 'bad': n, 'total_s':…, 'ok': bool}"""
        params = struct.pack('<BBB', CMD_MONITOR, 1, len(ids)) + bytes(ids)
        tx = self.send(CMD_SYNC, params)
        got, extra, bad = {}, 0, 0
        deadline = time.monotonic() + timeout
        while len(got) < len(ids):
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            f = self.read_frame(min(remain, 0.05))
            if f is None:
                continue
            if f['cmd'] != CMD_MONITOR or not f['checksum_ok'] or len(f['params']) != 16:
                bad += 1
                continue
            d = _decode_monitor_params(f['params'])
            if d['servo_id'] in ids and d['servo_id'] not in got:
                d['t_offset'] = f['t_rx'] - tx['t_start']
                d['checksum_ok'] = True
                got[d['servo_id']] = d
            else:
                extra += 1  # 非请求 ID / 重复 / 迟到
        total = time.monotonic() - tx['t_start']
        ok = set(got.keys()) == set(ids)
        self.log('sync_mon', n=len(ids), got=sorted(got.keys()), extra=extra, bad=bad,
                 total_ms=round(total * 1000, 3), ok=ok)
        return {'got': got, 'extra': extra, 'bad': bad, 'total_s': total,
                'n_ids': len(ids), 'ok': ok}


# ---------- 多圈圈数安全（P0.5-T4b k/k2；根因见 tests/runs/2026-11-03-TC-009/：
# J6 多圈漂到 388.6° 后 0x08 按多圈坐标执行 → 反向堵转过温。官方控制器开机只清 J0–J5） ----------

MTURN_LIMIT = 180.0


def read_raw_multiturn(bus, sid, timeout=None):
    """S1（T4b v2）：0x16 原始多圈读数，不做单圈归一化。返回 dict(angle, turns, st, mA, tC)
    或 None。判定是否在 ±180 内必须用本接口的原始 angle，不得用 norm_single_turn。"""
    f = bus.read_monitor(sid, timeout)
    if f is not None and f['verdict'] == 'ok' and f.get('valid_measure', False):
        return {'angle': f['angle'], 'turns': f['turns'], 'st': f['status'],
                'mA': f['current_mA'], 'tC': f['temp_c']}
    return None


read_mturn = read_raw_multiturn  # 兼容别名（tc008 等调用方）


def mturn_out(m):
    """原始多圈读数 → 是否超出 ±180（返回 bool）"""
    return m is None or abs(m['angle']) > MTURN_LIMIT


def mturn_gate(bus, ids, clear_ids=(), stop_power=0, tries=2):
    """k/k2 圈数门（只读检查 + 指定轴修复）：
    ① 逐轴 read_mturn 读原始多圈（读不到 = 不确定，按超限处理）；
    ② clear_ids 内的轴（默认调用方给 (6,)）超出 → 先 0x18 释放（0x10）→ 0x11 清多圈 →
       读回确认已回 ±180（重试 tries 次）；clear_turns 只允许在已释放后调用；
    ③ clear_ids 之外的轴超出 → 拒绝运动（返回 False，调用方必须不发任何运动帧直接退出）。
    返回 (ok, report)；ok=False 时 report 列出各轴原始读数与处置。"""
    rep = {}
    ok = True
    for sid in ids:
        m = read_mturn(bus, sid)
        rep[sid] = m
        if m is not None and (m['st'] & KNOWN_STATUS_MASK):
            ok = False      # S1：保护位 → 拒绝
            continue
        if not mturn_out(m):
            continue
        if sid not in clear_ids:
            ok = False
            continue
        for _ in range(tries):
            cleared = bus.clear_turns(sid, stop_power)   # 内部先发 0x18 释放再 0x11
            time.sleep(0.2)
            m = read_mturn(bus, sid)
            rep[sid] = m
            bus.log('mturn_clear', sid=sid, ok=cleared and not mturn_out(m),
                    read=(m or {}).get('angle'), turns=(m or {}).get('turns'))
            if not mturn_out(m):
                break
        else:
            ok = False
    return ok, rep


def mturn_assert(bus, ids, tag, retries=1):
    """k：每次运动段前的原始多圈复查（一帧 0x19+0x16 读全部轴，不发运动帧）。
    任一轴超出 ±180 → raise MturnRefused(runtime=True)；读不齐先重试 retries 次
    （T4b3 M2③），仍不齐再抛。返回 {sid: (angle, turns)}。"""
    for attempt in range(retries + 1):
        r = bus.sync_monitor(list(ids), timeout=0.1)
        out = {}
        incomplete = []
        for sid in ids:
            d = r['got'].get(sid)
            if d is None or not d['valid_measure']:
                out[sid] = None
                incomplete.append(sid)
                continue
            out[sid] = (d['angle'], d['turns'])
            if abs(d['angle']) > MTURN_LIMIT:
                raise MturnRefused('mturn_out_of_range@%s(J%d %.1f° turn=%d 需先圈数重置)'
                                   % (tag, sid, d['angle'], d['turns']), runtime=True)
        if not incomplete:
            return out
    raise MturnRefused('mturn_read_incomplete@%s(缺 %s，已重试 %d 次)'
                       % (tag, incomplete, retries), runtime=True)


class MturnRefused(RuntimeError):
    """多圈超限：拒绝运动。runtime=False（开机圈数门阶段）→ 不发任何帧直接退出；
    runtime=True（运行中 mturn_assert 触发）→ 按 S5 收尾但禁止 0x08/0x19
    （0x18 释放/保持不依赖目标角度，多圈超限时发送是安全的——审查方 T4b3 M2）。"""

    def __init__(self, msg, runtime=False):
        RuntimeError.__init__(self, msg)
        self.runtime = runtime


def fmt_monitor(sid, f):
    """0x16 结果一行式打印（给终端回贴用）"""
    if f is None:
        return 'J%d 0x16: TIMEOUT' % sid
    if f['verdict'] != 'ok':
        return 'J%d 0x16: %s' % (sid, f['verdict'])
    tc = f['temp_c']
    return ('J%d 0x16: %dmV %dmA %dmW T=%s st=0x%02x%s ang=%+.1f° turn=%d rtt=%.1fms%s' % (
        sid, f['voltage_mV'], f['current_mA'], f['power_mW'],
        ('%.1fC' % tc) if tc is not None else 'ADC%d' % f['temp_adc'],
        f['status'], ('[' + ','.join(f['status_flags']) + ']') if f['status_flags'] else '',
        f['angle'], f['turns'], f['rtt'] * 1000,
        ' SENTINEL(无效测量)' if f.get('sentinel') else ''))
