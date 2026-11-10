# arm_client · 维修工具（send / record / watch_j6）

对臂载控制器 `d1t_arm_ctrl`（真机或 `--sim`）发命令、录状态、盯 J6。
只依赖 `pip cyclonedds==0.10.2`（0.10.2 必须与臂上一致）与 numpy，不依赖 ROS2。

协议常量与消息类型统一从 `sdk/d1t_sdk` 导入（`sys.path` 指向仓库 `sdk/`），本目录不再自带协议副本。
配套的回放与分析脚本在 `tests/scripts/`（如 replay.py 示教回放、stream_ack_check.py 流式连续性复算）。

## CYCLONEDDS_URI（必配，否则发现不了控制器或收不到帧）

控制器和每个 Python 进程都要设，要点三条：

1. 带代理 TUN 的机器上，全网卡多播可能卡死/被劫持 → 限定具体网卡 + `AllowMulticast=false` + 单播 Peer；
2. `Discovery/ParticipantIndex` 用 auto 会绑不上标准 RTPS 端口（7410+，落到 5 万多的临时端口），
   单播发现互相找不到 → **每个进程固定一个不同的 index**（端口 = 7410+2×index）；
3. 同 index 的两个进程会抢端口直接起不来。

不用手写，用仓库的 `deploy/env.sh` 按角色设好（控制器 0、发命令 sdk 3、录制 record 5、显示 viz 6）：

```bash
source deploy/env.sh sim                      # 同机仿真，发命令用（index 3）
source deploy/env.sh sim '' '' record         # 同机仿真，record.py / watch_j6.py 用（index 5）
source deploy/env.sh real <本机网卡> '' record  # 连真臂（对端默认 192.168.123.100）
```

同时开几个工具时，每个终端给不同角色（或直接给数字，如 `source deploy/env.sh sim '' '' 4`）。

## send.py — 发命令（rt/d1t/arm_cmd）

```bash
python3 send.py STREAM --duration 60 --amp 5 --freq 0.2 --axes 0,1,2,3,4,5,6   # 正弦流式
python3 send.py HOLD            # 全轴 0x18 0x11 保持（也可作心跳）
python3 send.py ESTOP           # 软急停（锁存，RESET 清）
python3 send.py RESET           # 清 ESTOP/故障 → HOLD
python3 send.py RELEASE --axes 6
python3 send.py DAMP --axes 6
```

- `stamp_ns` 用 `time.time_ns()` 且强制严格递增（控制器按 stamp 丢旧命令）；`seq` 递增。
- RELEASE/DAMP 必须给 `--axes`（axis_mask=0 控制器拒收，本工具直接禁止）。
- 流式先发 HOLD 并等 `ctrl_state=HOLD` 再开始（控制器信箱只存最新一条，控制命令会被
  同周期的流式命令覆盖）；结束默认发收尾 HOLD，`--no-end-hold` 跳过（看门狗场景用）。
- `--event-log PATH`：把每次发送 `(t_ns, mode, seq)` 追加进 jsonl，分析脚本对齐时刻用。
- 跨机新进程首条命令可能被 BEST_EFFORT 静默丢（对端还没匹配上本进程）：本工具等首帧 + 1 s、
  未生效 1 s 内重发，直到看到 ctrl_state 变化。

## record.py — 录状态（rt/d1t/arm_state → jsonl + csv）

```bash
python3 record.py --duration 75 --out <目录> [--event-log events.jsonl]
```

- 用 listener 回调在回调里立刻 `take()`：BEST_EFFORT + KEEP_LAST(1) 下轮询读会丢帧。
- 每帧记 `rx_local_ns`（本机 `time.time_ns()` 收到时刻，与 send 的 stamp 同钟，时钟对齐分析依赖）。
- 输出三件：`arm_state.jsonl`（分析用原始数据）、`arm_state.csv`（67 列展平）、`run_meta.json`（起止/帧数/帧率）。
- **录制时长必须覆盖「发送启动约 2 s + 发送时长 + 收尾 ≥4 s」**；jsonl 是录完一次写盘，
  分析必须等录制进程结束后再跑。

## watch_j6.py — J6 温度/电流实时监视

```bash
source deploy/env.sh sim '' '' record   # 真臂：source deploy/env.sh real <本机网卡> '' record
python3 tools/arm_client/watch_j6.py      # Ctrl-C 退出
```

每秒打印一行 J6 温度 / 电流 / 角度 / 目标 / 状态与 fault、warn 位；录屏工具要等录完才能看，
现场盯夹瓶升温用这个。

## 出问题怎么办

| 现象 | 处置 |
|---|---|
| 5 s 收不到 ArmState | 控制器没起，或 CYCLONEDDS_URI 没配 / 两侧 Peer 没互指 / ParticipantIndex 冲突 |
| 发命令后一直 IDLE | 跨机首条被丢时本工具会重发；仍不行先 `ping` 对端通不通，再核 URI |
| 录制缺帧 | 确认用的是本目录 record.py（listener 版）；发送端与录制端 ParticipantIndex 不同 |
