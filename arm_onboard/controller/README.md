# d1t_arm_ctrl · 臂载控制器

替换臂上 marm_code 的 4 个服务：单线程独占 `/dev/ttyS4`，20 Hz 固定周期 0x19+0x08 同步下发 + 0x19+0x16 同步读回，
经 Cyclone DDS 0.10.2 收 `ArmCmd_`（`rt/d1t/arm_cmd`）、发 `ArmState_`（`rt/d1t/arm_state`），
内置看门狗、软急停、多圈检查、J6 限力与温度保护。
编译、部署、回滚的完整步骤见 `deploy/实机部署.md`（无臂仿真见 `deploy/虚拟部署.md`）。

## 1. 目录

| 路径 | 内容 |
|---|---|
| `src/config.hpp` | 全部常量（限位、限速、温度阈值等），逐个带注释 |
| `src/common/` | 内部消息结构（与 IDL 逐字段对应）、时钟、可唤醒等待 |
| `src/bus/` | 帧编解码 `frame`、串口 `PosixSerial`、事务层 `Bus` |
| `src/safety/` | 纯函数 + 状态机 `Core`（不做 I/O） |
| `src/loop/` | 信箱 `Mailbox`、主循环与启动流程 `Controller` |
| `src/dds_io/` | `DdsIo`（Cyclone 实现） |
| `src/sim/` | 仿真件：假舵机串口 `FakeSerial`、假上位机 `FakeClient`（`--sim` 用） |
| `idl/` | `d1t_arm.idl` + `generated/`（生成代码已提交，改 IDL 才需重新生成） |
| `tests/` | 编解码 / safety 单测、`--sim` 集成测试 |

线程只有三种：**主循环**（唯一碰串口的线程）、**DDS 收**（Cyclone 回调，只调 `Mailbox::put_cmd`）、
**DDS 发**（只调 `Mailbox::pop_state` 后写出）。两边之间只有 `Mailbox` 一个接口。

## 2. 编译

### 上位机 / 臂载计算机（Linux）

```bash
bash deploy/setup_host.sh      # 上位机装依赖（Cyclone DDS 0.10.2 + cyclonedds-cxx 0.10.2 源码装；已装跳过）
bash deploy/arm_install.sh     # 仓库根执行：Release 编译 + ctest（臂上与上位机通用）
```

手动编译等价于在本目录 `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4`。

ddscxx 头文件与库装在 `/usr/local` 时，CMake 找不到包配置会直接引 `/usr/local/include/ddscxx`
（及 `iceoryx/v2.0.2`）并链接 `ddsc ddscxx rt pthread`。代码只用 C++17、CMake ≤3.16 的功能。

找不到 CycloneDDS-CXX 或 `idl/generated/` 时自动只编核心与测试（`-DD1T_WITH_DDS=OFF` 可强制关闭）；
这时 `d1t_arm_ctrl` 只能配合 `--fake-client` 运行。

## 3. 运行

### 仿真（无硬件）

```bash
# 真 DDS：上位机工具（sdk、tools）可直接对着它测
./build/d1t_arm_ctrl --sim

# 不用 DDS：进程内假上位机发 J0/J3 ±5° 正弦 60 s，结束打印 seq 缺号
./build/d1t_arm_ctrl --sim --fake-client 60
```

仿真注入（可多次给）：`--sim-drop J:P` 丢包概率、`--sim-corrupt J:P` 坏帧概率、`--sim-status J:HEX` 状态位、
`--sim-temp J:C` 温度、`--sim-mturn J:DEG` 启动前多圈角。`-v` 打印周期超时明细。

DDS 互通（同机或跨机）需要两边固定网卡、关多播、单播 Peer 互指，且每个进程固定不同的
ParticipantIndex（端口 = 7410+2×index，不固定则互相发现不了）。不用手写：同机仿真
`bash deploy/sim_start.sh`（控制器 index 0），工具 / SDK 终端 `source deploy/env.sh sim`；
跨机见 `deploy/实机部署.md`（臂上 `deploy/arm_start.sh`，上位机 `source deploy/env.sh real <网卡>`）。

### 真机

```bash
bash deploy/arm_start.sh <臂上网卡名> <上位机IP>   # 停原 4 服务 + sudo 前台起控制器（URI 内联）；Ctrl-C 退出
```

等价的手动步骤：`sudo systemctl stop marm_controller marm_communication marm_control marm_subscripber`，
再 `sudo CYCLONEDDS_URI='…' ./build/d1t_arm_ctrl`（sudo 会丢环境变量，URI 必须写在命令行里）。

启动前自动检查：4 个原服务全部 inactive、`/proc` 里没有别的进程打开 `/dev/ttyS4`（判不了就退出，需 sudo）；
然后 `flock` 独占串口。任一不过 → 退出码 3，提示「先 systemctl stop 原服务」。

启动流程：7 轴 ping（各重试 2 次）→ 逐轴 0x10 读多圈（重试 2 次）→ 只对 |q| ≥ 180° 或读不到的轴发一次 0x11 →
再读 → J6 仍超限则 0x18 释放 → 0x11 → 再读（最多 2 次）→ J0–J5 仍超限 = MTURN 故障：**不发任何帧，等人工处理**。

**多圈超限只按角度判：|q| ≥ 180°**。turns = floor(角度/360)，负角度时 turns = −1 是正常读数
（多圈坐标下的正常负角），只记录、发布，不作判据。

启动后为 IDLE：只读监控、不发运动帧，看门狗在收到第一条合法命令后才启用，且**只在 STREAMING 下起作用**
（HOLD / RELEASED / DAMP / ESTOP / FAULT 下没有在执行的运动，不触发；RELEASE 后手托臂摆位不会被看门狗锁回）。

退出（Ctrl-C / SIGTERM）：全轴 0x18 0x11 保持（绝不释放；J6 过温已释放的不锁回）后退出进程；有帧写失败时日志提示物理急停 / 断电。

### 回滚

```bash
bash deploy/arm_restore.sh    # 停本控制器 → 等它交还串口 → systemctl start 原 4 服务
```

4 行 active 即恢复。原服务只 stop、从未 disable，新程序不覆盖任何原文件。

## 4. 测试

```bash
./build/test_frame          # 官方协议示例帧逐条
./build/test_safety         # 限位 / 步长 / 多圈 / 状态位 / 温度 / 看门狗 TARGET_FAR 等
./build/test_sim --quick    # 5 s 流式 + 丢包 + 看门狗 + ESTOP ×20（ctest 默认）
./build/test_sim --full     # 60 s 流式
./build/test_regress        # 历史缺陷复现场景（永久回归）
cd build && ctest           # 臂上 cmake 3.16 无 --test-dir
```

## 5. 行为要点（实现取舍）

| # | 行为 | 理由 |
|---|---|---|
| 1 | 每周期：取信箱 → safety.step → 下发（每帧本周期最多重试 2 次）→ safety.commit（按写出结果提交）→ 0x19+0x16 读回 → 发布 ArmState → 睡到下一个绝对时刻 | 单一判定点 |
| 2 | safety 用的是**上一周期**的读回；故障位因此比原始数据晚一帧出现，动作在下一周期开头执行（≤1 周期） | 每周期只有一次监控 |
| 3 | 读回超时 = clamp(周期剩余 − 5 ms, 20, 40) ms，从请求实际发出时刻算；超时后再开 ≤20 ms 静默窗收迟到帧（丢弃、计 `rx_late`），清缓冲放在睡完帧间隔、紧挨写请求之前 | 迟到帧不污染周期 |
| 4 | 信箱只存最新一条：同一周期里 RESET / HOLD 后紧跟 STREAM，前者会被覆盖。**上位机发控制命令后应看到 ctrl_state 变化再开始流式** | 避免控制命令被流式覆盖 |
| 5 | ESTOP 单独一格，放入即唤醒主循环；不做 stamp 新旧判断，**也不抬高普通命令的 stamp 水位**；放入时作废信箱里尚未取走的普通命令；只有接收序号晚于最近一次 ESTOP 的 RESET 才能解除 | 急停优先 |
| 6 | 已锁存 ESTOP 时再收到 ESTOP 不重复下发（但更新接收代次） | 避免每周期 8 帧拖垮周期 |
| 7 | 冻结帧只含本周期有新鲜读数、且不在多圈超限 / J6 过温释放 / J6 清圈中的轴（0x08 按多圈坐标执行）；其余轴只发 0x18 0x11。ESTOP 周期冻结帧排最前、不执行清圈；非 ESTOP 批次每帧、每次重试前都查信箱，ESTOP 在等就中止整批、立即执行急停序列（冻结帧 + 全轴 0x18 0x11），本周期不读回不发布 | 0x08 按多圈坐标语义，超限轴不能发 0x08 |
| 8 | J6 降出力（warn J6_HOT）：新鲜温度 ≥45 ℃ 进入、≤43 ℃ 才退出（回差）。**进入时锁定一次**目标 = 当时 J6 新鲜实测角，之后不跟随实测（防瓶子回弹顶开手指的棘轮）。流式时 J6 目标 = max(命令目标, 锁定值)，更张开的目标照常执行并更新锁定值；**流式 / HOLD / 看门狗保持 / 故障保持**下，J6 总线目标比锁定值更闭合 ≥0.2° 时发一帧 0x08 到锁定值（锁存中也发）。J6 不出力（释放 / 阻尼）时锁定值随实测。退出后恢复正常跟随 | 温度回差防抖动；锁定目标防棘轮 |
| 9 | J6_THERMAL（≥50 ℃，或 J6 温度连续 4 周期不可知）：J6 0x18 0x10 释放、其余保持；锁存。RESET 只在 J6 新鲜温度 ≤45 ℃ 时清，清除前 J6 一直释放 | 过温保护 |
| 10 | 运行中 J6 多圈超限（\|q\| ≥ 180°，不看 turns）：J6 退出 0x08；未锁存故障时自动处理，**分周期执行**：0x18 释放 →（≥100 ms）0x11 →（≥200 ms + 1 周期）看读数，最多 2 轮，仍超限 → MTURN | 0x08 按多圈坐标执行，超限时不能发 |
| 11 | RELEASE / DAMP：mask 内的轴发对应 0x18；mask 外正在流式或未下过令的轴发 0x18 0x11；重复命令不重复发帧（上次没写出去的除外） | 幂等 |
| 12 | 看门狗只在 STREAMING 下计时触发。恢复只对 STREAM 做 RESUME_TOL 检查；先发一条 HOLD 也能清掉看门狗状态，之后 STREAM 从实测起步、受步长钳制 | RELEASE 摆位不被锁回 |
| 13 | 锁存期间 / TARGET_FAR 的合法命令不 ack，但刷新看门狗计时（上位机在线）。**ack 只在本周期全部帧写成功后推进**；RESET 有位未清时不 ack，报 warn RESET_DENIED | ack = 一定已执行 |
| 14 | 卡住判定的「电流高」阈值 500 mA、跟踪误差用「最近写成功的目标 − 实测」 | 阈值在 config.hpp |
| 15 | COMM_LOSS_CYCLES = 4（50 ms 周期下 ≈200 ms，周期超时时按实际墙钟更长）；温度 ADC 无效（0 / ≥4096）也算该轴无效回包 | 保护性【暂定】值 |
| 16 | 退出时不走 DDS 析构直接 `_Exit`：Cyclone 0.10.2 退出时可能卡住不退，进程不退就一直占着串口 | 串口必须立刻交还 |
| 17 | 控制器**任何路径都不会自动释放 J0–J5**：0x18 0x10 只来自上位机显式 RELEASE（mask 指定）；J6 例外见 9、10 | 硬规则 |
| 18 | **写失败闭环**：每帧本周期重试 2 次；仍失败 → 锁存 TX_FAIL、打印「请用物理急停」、下周期全轴保持；写失败的停止帧（含 J6 热释放）每周期重发直到成功；写失败的 0x08 不更新 q_target、不 ack。串口写入 + 排空总期限 20 ms，超时丢弃未发出的输出。排空不用阻塞式调用：轮询 TIOCOUTQ 到 0 → 轮询 TIOCSERGETLSR 的 TEMT 位；任一 ioctl 不支持 / 出错 → 按字节数 ×10/115200 + 2 ms FIFO 余量估算，估算终点超期即失败 | 写失败必须被看见 |
| 19 | **停住确认**（HOLD / 看门狗 / 故障 / ESTOP 统一）：0x18 0x11 写成功后，要求保持帧之后最近 8 个**连续**新鲜采样（相邻间隔 ≤2 周期）的窗口范围 max(q)−min(q) ≤ 0.5°；缺回包 / 间隔过大则窗口清空重攒。不用相邻两帧差（静止锁住时读数也会跳 0.1–0.3°）。500 ms 内未确认 → 重发 0x18 0x11（最多 2 次）→ 仍未确认 → HOLD_FAIL。重复 HOLD 不重置未完成的确认 | 静止读数抖动会误触发相邻帧判据 |
| 20 | **q_target** = 每轴最近一次写成功的钳制后目标；0x18 0x11 写成功时 = 决策时的实测角；释放 / 阻尼 / 从未下令的轴 = **NaN** | 上位机可判「从未下令」 |

### fault / warn 位（本控制器新增）

| 位 | 名称 | 类别 | 触发 | 控制器动作 | RESET 清除条件 |
|---|---|---|---|---|---|
| 6 | TX_FAIL | fault | 任一帧本周期重试 2 次后仍写失败（写入 / 排空超 20 ms 或系统调用出错） | 全轴保持；停止帧每周期重发直到成功；日志提示物理急停 | 没有待重发的停止帧 |
| 7 | HOLD_FAIL | fault | 保持后 500 ms 未确认停住，重发 0x18 两次仍未确认 | 全轴保持（不释放）；日志点名轴号与原因（窗口范围 / 采样不足 / 无回包） | 七轴最近 8 个连续新鲜采样的窗口范围都 ≤ 0.5°（与保持确认同一判据） |
| 16 | RESET_DENIED | warn | RESET 时有锁存位的原因未经本周期新鲜反馈证明已解除 | 那些位继续锁存；该 RESET 不 ack | — |

已有位的 RESET 清除条件：ESTOP = RESET 晚于最近一次 ESTOP 收到；COMM_LOSS = 七轴本周期都有效；
MTURN = 七轴新鲜读数都在范围；SERVO_PROT = 七轴新鲜状态位里没有故障类位（J6 的 bit2/bit6 按告警处理，不算）；
J6_THERMAL = J6 新鲜温度 ≤45 ℃。

## 6. 出问题怎么办

| 现象 | 处置 |
|---|---|
| 启动即退出码 3 | 原服务没停干净：`systemctl stop` 4 个服务；`/proc` 里有别的进程占串口时重启臂 |
| 启动报 MTURN | J0–J5 多圈超限（\|q\| ≥ 180°）：重启臂（开机脚本只清 J6）；J6 的会自动清 |
| 日志出现 TX_FAIL / 「请用物理急停」 | 串口写失败已锁存，控制器只保持不再运动；按物理急停后排查串口/供电 |
| HOLD_FAIL 反复出现 | 看日志点名的轴：窗口范围超 0.5° 通常是真没停住（堵转/打滑），先查机械 |
| 周期超时 / overruns 增长 | 总线负载或系统睡眠；看 `-v` 明细定位是发送还是读回超时 |
| 上位机互相发现不了 | 查两侧 CYCLONEDDS_URI：AllowMulticast=false、Peer 互指、ParticipantIndex 各不相同且固定 |
| 退出后原服务拿不到串口 | 内核 close 等发送器排空最长约 30 s，等即可；保持帧已发出，臂是安全的 |
