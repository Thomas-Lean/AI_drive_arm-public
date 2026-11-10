#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""send.py — 发 ArmCmd（命令话题，话题名与 QoS 取自 d1t_sdk）。

用法：
  python3 send.py STREAM --duration 60 --amp 5 --freq 0.2 --axes 0,1,2,3,4,5,6
  python3 send.py HOLD
  python3 send.py ESTOP
  python3 send.py RESET
  python3 send.py RELEASE --axes 6
  python3 send.py DAMP --axes 6

要点（控制器 README §5）：
- 信箱只存最新一条：发控制命令（HOLD/ESTOP/RESET 等）后要等 ArmState.ctrl_state 变化
  再开始流式，否则同一周期里控制命令会被流式命令覆盖。
- stamp_ns 用 CLOCK_REALTIME ns 且严格递增（任务书 §1.3：控制器按 stamp 判新旧，
  不大于上一条的直接丢弃）。seq 递增。
- RELEASE/DAMP 必须显式给 --axes；axis_mask=0 控制器拒收，这里直接禁止。
- --event-log PATH：把每次发送的（t_ns, mode, seq）追加进 jsonl，供 analyze.py 做
  E4/E5 原始时刻表（发送侧事件与 ArmState 的 rx_local_ns 同钟）。
"""
import argparse
import math
import sys
import time
from pathlib import Path

_SDK = Path(__file__).resolve().parents[2] / "sdk"  # 仓库根/sdk
if str(_SDK) not in sys.path:
    sys.path.insert(0, str(_SDK))  # 协议常量与消息类型统一从 d1t_sdk 导入

from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader

from _util import append_event, axes_to_mask, parse_axes
from d1t_sdk._common import (MODE_DAMP, MODE_ESTOP, MODE_HOLD, MODE_NAMES,
                            MODE_RELEASE, MODE_RESET, MODE_STREAM, CTRL_STATE_NAMES,
                            ArmCmd_, dds_qos, make_cmd_topic, make_participant,
                            make_state_topic)


def parse_args():
    p = argparse.ArgumentParser(description="发 ArmCmd（任务书 §1.2）")
    p.add_argument("mode", choices=["STREAM", "HOLD", "RELEASE", "DAMP", "ESTOP", "RESET"],
                   help="命令模式")
    p.add_argument("--axes", default=None, help="轴号逗号分隔（RELEASE/DAMP 必填；STREAM 可选，默认全部 7 轴）")
    p.add_argument("--duration", type=float, default=60.0, help="STREAM：时长 s（默认 60）")
    p.add_argument("--amp", type=float, default=5.0, help="STREAM：正弦幅值 度（默认 5）")
    p.add_argument("--freq", type=float, default=0.2, help="STREAM：正弦频率 Hz（默认 0.2）")
    p.add_argument("--rate", type=float, default=20.0, help="STREAM：发送速率 Hz（默认 20）")
    p.add_argument("--dt-ms", type=int, default=50, help="ArmCmd.dt_ms（默认 50）")
    p.add_argument("--grip-power", type=int, default=0, help="ArmCmd.grip_power_mw（默认 0=按舵机保护阈值）")
    p.add_argument("--center", default="auto", help="STREAM：正弦中心 auto=取首帧实测 q，或逗号分隔 7 个角度")
    p.add_argument("--end-hold", dest="end_hold", action="store_true", default=True,
                   help="STREAM 结束后发 HOLD（默认开）")
    p.add_argument("--no-end-hold", dest="end_hold", action="store_const", const=False,
                   help="STREAM 结束后不发 HOLD（E4 停发看门狗场景用）")
    p.add_argument("--sync-state", action="store_true",
                   help="STREAM：每收到一帧 ArmState 发一条（跟控制器节拍走，防两边时钟漂移导致"
                        "某周期没新命令/一周期两条）；此时 --rate 应等于控制器频率")
    p.add_argument("--event-log", default=None, help="事件日志 jsonl 路径（追加）")
    p.add_argument("--state-timeout", type=float, default=5.0, help="等 ctrl_state 变化的超时 s（默认 5）")
    return p.parse_args()


class Sender:
    def __init__(self):
        self.dp = make_participant()
        self.rd = DataReader(self.dp, make_state_topic(self.dp), dds_qos())
        self.wr = DataWriter(self.dp, make_cmd_topic(self.dp), dds_qos())
        self.seq = 0
        self.last_stamp = 0

    def send_cmd(self, mode, q=None, axis_mask=0, dt_ms=50, grip_power=0, event_log=None):
        self.seq += 1
        stamp = max(self.last_stamp + 1, time.time_ns())  # 严格递增（§1.3）
        self.last_stamp = stamp
        cmd = ArmCmd_(
            seq=self.seq, stamp_ns=stamp, mode=mode, axis_mask=axis_mask,
            q=list(q) if q is not None else [0.0] * 7, dt_ms=dt_ms, grip_power_mw=grip_power)
        self.wr.write(cmd)
        if event_log:
            append_event(event_log, {"t_ns": stamp, "event": "send", "mode": mode,
                                     "seq": self.seq, "axis_mask": axis_mask})
        return cmd

    def latest_state(self):
        """取 reader 里最新一帧（若有）。"""
        states = None
        for s in self.rd.take():
            states = s
        return states

    def wait_first_state(self, timeout_s):
        """等第一帧 ArmState（证明发现已完成），再等 1 s 让控制器那边也匹配上本进程的 writer。
        跨机时新进程的 writer 刚建好、对端 reader 还没匹配，第一条命令会被 BEST_EFFORT 静默丢掉
        （实机现象：HOLD 发出后控制器一直 IDLE）。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.latest_state()
            if s is not None:
                time.sleep(1.0)
                return s
            time.sleep(0.05)
        return None

    def send_until(self, mode, targets, timeout_s, q=None, axis_mask=0, event_log=None):
        """发命令，1 s 内没进 targets 就重发（新 seq/stamp），直到 timeout_s。返回 (ctrl_state 或 None)。"""
        deadline = time.monotonic() + timeout_s
        while True:
            self.send_cmd(mode, q=q, axis_mask=axis_mask, event_log=event_log)
            st, _ = self.wait_ctrl_state(targets, min(1.0, max(0.0, deadline - time.monotonic())))
            if st is not None or time.monotonic() >= deadline:
                return st

    def wait_ctrl_state(self, targets, timeout_s):
        """等 ctrl_state 进入 targets 之一；返回 (达成的 ctrl_state 或 None, 帧或 None)。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            s = self.latest_state()
            if s is not None and s.ctrl_state in targets:
                return s.ctrl_state, s
            time.sleep(0.05)
        return None, None


def stream(sender, args, event_log):
    axes = parse_axes(args.axes) if args.axes else list(range(7))
    mask = axes_to_mask(axes)
    n_samples = max(1, int(round(args.duration * args.rate)))

    # 1. 等第一帧 ArmState：确认通路 + 取正弦中心
    s0 = sender.wait_first_state(args.state_timeout)
    if s0 is None:
        print(f"NG：{args.state_timeout}s 内没收到 ArmState（控制器没起？CYCLONEDDS_URI 没配？）")
        return 1
    center = {i: float(s0.q[i]) for i in range(7)}
    if args.center != "auto":
        vals = [float(x) for x in args.center.split(",")]
        if len(vals) != 7:
            print(f"NG：--center 需要 7 个角度，收到 {len(vals)} 个")
            return 1
        center = dict(enumerate(vals))

    # 2. 先发 HOLD（清看门狗/旧状态），等 ctrl_state 变化（README §5 #4/#12）
    st = sender.send_until(MODE_HOLD, {3}, args.state_timeout, q=list(s0.q), event_log=event_log)
    if st is None:
        print(f"NG：发 HOLD 后 {args.state_timeout}s 内没等到 ctrl_state=HOLD"
              f"（当前帧：{sender.latest_state()}）")
        return 1

    # 3. 流式：绝对时刻排程（防漂移），q 非流式轴取中心（保持）
    print(f"STREAM 开始：轴={axes} amp={args.amp}° f={args.freq}Hz rate={args.rate}Hz"
          f" n={n_samples} center=auto:{all(abs(center[i]-s0.q[i]) < 1e-6 for i in range(7))}")
    append_event(event_log, {"event": "stream_start", "n": n_samples,
                             "axes": axes, "amp": args.amp, "freq": args.freq})
    def send_sample(i):
        t_i = i / args.rate
        q = [center[j] for j in range(7)]
        for a in axes:
            q[a] = center[a] + args.amp * math.sin(2 * math.pi * args.freq * t_i)
        sender.send_cmd(MODE_STREAM, q=q, axis_mask=0, dt_ms=args.dt_ms,
                        grip_power=args.grip_power, event_log=event_log)

    t_start = time.monotonic()
    late_warns = 0
    if args.sync_state:
        # 每来一帧新 ArmState 发一条；丢了状态帧就按 seq 差补进样本序号，保持波形时间正确
        last_seq, i = None, 0
        while i < n_samples:
            s = sender.latest_state()
            if s is None or s.seq == last_seq:
                time.sleep(0.001)
                continue
            if last_seq is not None and s.seq > last_seq + 1:
                late_warns += 1
                i += s.seq - last_seq - 1
            last_seq = s.seq
            if i < n_samples:
                send_sample(i)
            i += 1
    else:
        for i in range(n_samples):
            target_t = t_start + i / args.rate
            delay = target_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -args.rate * 3:
                late_warns += 1
            send_sample(i)
    elapsed = time.monotonic() - t_start
    print(f"STREAM 完成：发 {n_samples} 条，实际 {elapsed:.2f}s"
          f"（平均 {n_samples / elapsed:.2f} Hz），"
          + (f"丢状态帧 {late_warns} 次" if args.sync_state else f"迟截至警告 {late_warns} 次"))

    # 4. 收尾 HOLD（E4 停发场景用 --no-end-hold 跳过，让看门狗触发）
    if args.end_hold:
        s_last = sender.latest_state()
        sender.send_cmd(MODE_HOLD, q=list(s_last.q) if s_last is not None else None,
                        event_log=event_log)
        st, s_end = sender.wait_ctrl_state({3}, args.state_timeout)
        print(f"收尾 HOLD：ctrl_state={'HOLD' if st == 3 else '未确认'}")
    append_event(event_log, {"event": "stream_end"})
    return 0


def control_cmd(sender, args, event_log):
    """HOLD / ESTOP / RESET / RELEASE / DAMP：发送 → 等 ctrl_state 变化。"""
    mode = {v: k for k, v in MODE_NAMES.items()}[args.mode]  # "HOLD" → 1
    axis_mask = 0
    q = None
    if args.mode in ("RELEASE", "DAMP"):
        if not args.axes:
            print("NG：RELEASE/DAMP 必须给 --axes（axis_mask=0 控制器拒收）")
            return 1
        axes = parse_axes(args.axes)
        axis_mask = axes_to_mask(axes)
    s_now = sender.wait_first_state(args.state_timeout)
    if s_now is None:
        print(f"NG：{args.state_timeout}s 内没收到 ArmState（控制器没起？CYCLONEDDS_URI 没配？）")
        return 1
    q = list(s_now.q)
    # RELEASE/DAMP 只认 RELEASED(4)（把 IDLE/HOLD 算成功会提前报 OK）
    targets = {MODE_HOLD: {3}, MODE_RESET: {1, 3}, MODE_ESTOP: {5},
               MODE_RELEASE: {4}, MODE_DAMP: {4}}[mode]
    st = sender.send_until(mode, targets, args.state_timeout, q=q, axis_mask=axis_mask,
                           event_log=event_log)
    cur = sender.latest_state()
    cur_name = CTRL_STATE_NAMES.get(cur.ctrl_state, cur.ctrl_state) if cur is not None else "?"
    if st is not None:
        print(f"OK：{args.mode} 已发（seq={sender.seq}），ctrl_state → "
              f"{CTRL_STATE_NAMES[st]}（当前 {cur_name}）")
        return 0
    print(f"NG：{args.mode} 已发（seq={sender.seq}）但 {args.state_timeout}s 内没等到"
          f" ctrl_state 进入 {targets}（当前 {cur_name}）")
    return 1


def main():
    args = parse_args()
    event_log = args.event_log
    if event_log:
        append_event(event_log, {"event": "send_invoked", "mode": args.mode})
    sender = Sender()
    try:
        if args.mode == "STREAM":
            return stream(sender, args, event_log)
        return control_cmd(sender, args, event_log)
    except KeyboardInterrupt:
        print("中断")
        return 130
    finally:
        pass  # cyclonedds-python 无 Participant.close()，进程退出即释放


if __name__ == "__main__":
    sys.exit(main())
