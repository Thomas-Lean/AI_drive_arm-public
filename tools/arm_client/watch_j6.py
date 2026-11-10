#!/usr/bin/env python3
"""现场盯 J6：每秒打印一行 J6 温度 / 电流 / 角度 / 目标 / 状态与 fault、warn 位（夹瓶升温时用）。

record.py 录完才写文件，夹瓶升温时看不到实时温度，所以单独起这个只读订阅者。
用法：CYCLONEDDS_URI=...（ParticipantIndex 用 5）python3 watch_j6.py   Ctrl-C 退出
"""
import argparse
import sys
import time
from pathlib import Path

_SDK = Path(__file__).resolve().parents[2] / "sdk"  # 仓库根/sdk
if str(_SDK) not in sys.path:
    sys.path.insert(0, str(_SDK))  # 协议常量与消息类型统一从 d1t_sdk 导入

from cyclonedds.sub import DataReader

from d1t_sdk._common import CTRL_STATE_NAMES, dds_qos, make_participant, make_state_topic

J6 = 6
WARN_J6_HOT = 1 << 13
FAULT_J6_THERMAL = 1 << 5


def parse_args():
    p = argparse.ArgumentParser(description="J6 温度/电流 1 Hz 只读监视（Ctrl-C 退出）")
    return p.parse_args()


def main():
    parse_args()
    dp = make_participant()
    rd = DataReader(dp, make_state_topic(dp), dds_qos())
    t_last = 0.0
    while True:
        s = None
        for x in rd.take():
            s = x
        now = time.monotonic()
        if s is not None and now - t_last >= 1.0:
            t_last = now
            flags = []
            if s.warn & WARN_J6_HOT:
                flags.append("J6_HOT(≥45℃ 降出力)")
            if s.fault & FAULT_J6_THERMAL:
                flags.append("J6_THERMAL(已释放)")
            print(f"J6 {s.temp_c[J6]:5.1f}℃  {s.current_ma[J6]:4d} mA  q={s.q[J6]:6.1f}°  "
                  f"目标={s.q_target[J6]:6.1f}°  {CTRL_STATE_NAMES.get(s.ctrl_state, s.ctrl_state)}  "
                  f"fault=0x{s.fault:x} warn=0x{s.warn:x}  {' '.join(flags)}", flush=True)
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
