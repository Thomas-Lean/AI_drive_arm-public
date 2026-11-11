# d1t_sdk — AI_drive_arm 上位机 SDK

给 AI_drive_arm 七轴臂（D1-550，6 关节 + 夹爪）的 Python 控制库。**接口与 D1 臂社区开源
项目 d1arm_sdk 同名同参同单位**，底层从 TCP 守护进程换成 DDS 直连臂载控制器
`d1t_arm_ctrl`（本仓库 `arm_onboard/controller/`），不起服务。

```
用户脚本 ── d1t_sdk.D1Arm（上位机或任一联网电脑，Python 库，不起服务）
              ├─ FK / IK、关节五次多项式、直线、圆弧（移植自 d1arm_sdk）
              └─ 规划结果切成 20 Hz 关节角，每收到一帧 ArmState 发一条 ArmCmd
                      ↓ DDS rt/d1t/arm_cmd      ↑ rt/d1t/arm_state
臂上 d1t_arm_ctrl：执行关节角 + 20 Hz 七轴反馈 + 看门狗/限位/限速/急停/J6 保护
```

## 安装

```bash
pip3 install --user cyclonedds==0.10.2 numpy   # 必须 0.10.2，与臂上一致
```

`sdk/` 目录整体拷走即可用（`import d1t_sdk` 需 sdk/ 在 PYTHONPATH，或直接在 sdk/ 下
运行示例；Python ≥3.8）。

## 快速开始（5 行）

```python
from d1t_sdk import D1Arm
arm = D1Arm()                        # 需先设 CYCLONEDDS_URI（见实机验收清单 §0.3）
arm.home(4.0); arm.wait_idle()       # 回全零位（URDF 全零，臂前伸）
print(arm.state()["tcp"]["pos"])     # TCP 位置 [m]
arm.move_joints([0, -40, 60, 0, 30, 0], vel_dps=30)   # URDF 角（度）
arm.gripper(45)                      # 夹爪开到 45 mm（金属指间距）
arm.close()
```

命令行（同 d1arm_sdk 的 `_cli`，IP 换成 CYCLONEDDS_URI 环境变量）：

```bash
python3 -m d1t_sdk state | ping | home | free | lock | estop | reset | park
python3 -m d1t_sdk joint 0,-40,60,0,30,0
python3 -m d1t_sdk line 0.30,0,0.38
python3 -m d1t_sdk gripper 45
```

`line` 让末端从**当前位置**沿直线走到目标。动臂之前，SDK 会先把整条直线逐点算一遍逆解；
只要有一个点臂够不着，就报 `路径 IK 失败` 并拒绝执行，臂不动，不用担心走到一半卡住。
碰到这个报错：换一个目标，或者先用 `joint` 把臂挪到离目标近一些的姿态再走。
上面的示例值是按「先跑过 `home`、再跑 `joint 0,-40,60,0,30,0`」之后的位置选的，照顺序敲一定能走通。

够不够得着，和目标的**姿态**关系很大：例如在接近伸直的 (0.35, 0, 0.42) 处要求腕部俯仰 90°
（rpy 0,1.5708,0）就够不着，同样的姿态放到 (0.15, 0, 0.22) 处则可以。

## 指令速查

**通用限制**：关节角一律是 URDF 角（度，6 轴 J0–J5）；限位 J0 [−163, 163]、J1 [−90.3, 97]、J2 [−97, 89]、
J3 [−163, 166]、J4 [−96, 113]、J5 [−166, 162]，超限的目标会被拒绝。关节速度 `vel_dps` 须在
(0, 100] °/s，夹爪 `vel_dps` 须在 (0, 20] °/s，超出报错。位置单位 m（基坐标系），姿态 rpy 单位弧度、
quat 顺序 [w, x, y, z]。夹爪是金属指间距 4–70 mm（J6 舵机角 −30–64.5°）。控制周期固定 50 ms（20 Hz）；
发送端退出或断流约 0.3 s，控制器看门狗停臂并保持。

**参数校验**：所有参数在读状态、发命令之前检查，类型 / 格式 / 个数不对、NaN/inf、超出下表范围
一律抛 `D1ArmError`（消息写明参数名、收到的值、可接受范围），臂不动；不会把越界值悄悄截断后执行。
- 关节角向量：恰好 6 个有限数字（list / tuple / 一维 numpy 数组）。
- 位置 `pos`/`via`：恰好 3 个有限数字（m）；`rpy` 恰好 3 个（弧度）；`quat` 恰好 4 个 [w, x, y, z]、
  模长不为 0（内部归一化）；`rpy` 与 `quat` 只能给一个，都不给 = 姿态 rpy [0, 0, 0]。
- `async_`：只能是 `True` / `False`。
- 超时类参数（`timeout`、`timeout_s`）：> 0 秒。
- 只支持固定值的参数传了别的值报「只支持 X」：`gripper(grip_power_mw=)` 只支持 0，`park(async_=)` 只支持 False。

### 命令行（`python3 -m d1t_sdk …`）

| 指令 | 示例 | 参数 / 限制 |
|---|---|---|
| 读状态 | `python3 -m d1t_sdk state` | — |
| 舵机在线 | `python3 -m d1t_sdk ping` | — |
| 回全零位 | `python3 -m d1t_sdk home` | 不带参数；4 s 走完；全零位 = 臂前伸，先确认前方无障碍 |
| 关节运动 | `python3 -m d1t_sdk joint 0,-40,60,0,30,0` | 恰好 6 个 URDF 角（度），逗号分隔、无空格；超限位报错；80 °/s |
| 直线运动 | `python3 -m d1t_sdk line 0.30,0,0.38` | 恰好 3 个 `x,y,z`（m）或 6 个 `x,y,z,roll,pitch,yaw`（弧度）；从当前位置出发，够不着就报错不动 |
| 夹爪 | `python3 -m d1t_sdk gripper 45` | 1 个数，4–70 mm，超出报错不动；20 °/s |
| 失力 | `python3 -m d1t_sdk free` | **臂会掉，先托住** |
| 锁住 | `python3 -m d1t_sdk lock` | 锁在当前姿态 |
| 软急停 | `python3 -m d1t_sdk estop` | 之后必须 `reset` 才能再动 |
| 复位 | `python3 -m d1t_sdk reset` | 清软急停 / 故障 |
| 回折叠位 | `python3 -m d1t_sdk park` | 先 `gripper 70` 张开夹爪；夹爪未张开时报错不动。腕部回零→大小臂一起收回→停 0.5 s 全轴失力，打印「已到折叠位并已失力，可以下电」 |

无参数指令（state、ping、home、park、free、lock、estop、reset）不接受任何参数。
指令缺失 / 未知、参数个数不对、数值不能解析（如 `joint 1,2,x`）、逗号分隔的个数不对、nan/inf：
stderr 打一行 `失败：原因` 再打一行用法，退出码 2，不连臂。
数值越界、规划不可达：只打一行 `失败：…`，退出码 2，臂不动。

### Python 接口（`from d1t_sdk import D1Arm; arm = D1Arm()`）

| 功能 | 示例 | 参数 / 限制 |
|---|---|---|
| 连接 / 断开 | `arm = D1Arm()` / `arm.close()` | 需先 `source deploy/env.sh sim\|real <网卡>`；`timeout` > 0 s、默认 30（运动等待的额外余量）；流式中 close → 看门狗停臂 |
| 读状态 | `arm.state()["q_deg"]` | 返回 dict，键见下文 `state()` 键；`timeout_s` > 0 s、默认 5 |
| 舵机在线 | `arm.ping()` | 返回 7 个 bool |
| 等运动完成 | `arm.wait_idle()` | `timeout` > 0 s、默认 30，超时返回 False |
| 等下一帧 | `arm.wait_frame()` | `timeout` > 0 s、默认 1.0；流式循环用它控节奏，不要用 sleep |
| 回全零位 | `arm.home(4.0)` | `duration_s` ≥ 0.5 s、默认 6.0，小于 0.5 报错；默认等走完 |
| 关节运动 | `arm.move_joints([0, -40, 60, 0, 30, 0], vel_dps=30)` | 6 个 URDF 角，超限位报错；`vel_dps` ∈ (0, 100] °/s、默认 80，超出报错；`acc_dps2` > 0 °/s²、默认 500 |
| 夹爪 | `arm.gripper(45)` | `mm` ∈ [4, 70]，超出报错不动；`vel_dps` ∈ (0, 20] °/s、默认 20，超出报错；`grip_power_mw` 只支持 0 |
| 失力 / 锁住 | `arm.free_all()` / `arm.lock_all()` | free 后**臂会掉** |
| 软急停 / 复位 | `arm.estop()` / `arm.reset()` | estop 后必须 reset |
| 回折叠位 | `arm.park()` | `vel_dps` ∈ (0, 100] °/s、默认 20；`async_` 只支持 False；夹爪须已张开（开口 ≥ 65 mm，否则抛错不动；park 不替你张夹爪）→腕部回零→大小臂一起收回，每段到位误差 ≤ 3° 才走下一段；最后停 0.5 s 全轴失力 |
| 正解 | `arm.fk([0, -40, 60, 0, 30, 0])["pos"]` | 6 个 URDF 角（不查限位）；返回 `pos`(m)、`quat`、`rpy`(rad) |
| 逆解 | `arm.ik([0.30, 0, 0.38], rpy=[0, 0, 0])` | 返回 6 个 URDF 角；无解抛 `D1ArmError`；`seed` 可给初值（恰好 6 个 URDF 角） |
| 走到位姿（关节插值） | `arm.move_j([0.30, 0, 0.38], rpy=[0, 0, 0])` | 先逆解一次再按关节走；速度参数同 move_joints |
| 直线 | `arm.move_l([0.30, 0, 0.38])` | `vel` > 0 m/s、默认 0.15（峰值约 1.875×vel）；`acc` > 0 m/s²、默认 2.0；路径上任一点够不着就报错不动 |
| 圆弧 | `arm.move_c([0.30, 0.05, 0.38], via=[0.33, 0.03, 0.38])` | 当前位置 → 经 `via` → 目标，三点定圆；速度参数同 move_l |
| 开始流式 | `arm.stream_start()` | 先锁当前姿态，之后每帧发最新目标 |
| 流式发关节角 | `arm.stream_joints([0, -40, 60, 0, 30, 0], j6_deg=40)` | 6 个 URDF 角，超限位报错；下一帧生效；`j6_deg` 是 J6 舵机角 [−30, 64.5]，超出报错，None = 保持上次命令角；流式中 J6 ≤ 100 °/s |
| 流式发位姿 | `arm.stream_pose([0.30, 0, 0.38], rpy=[0, 0, 0], j6_deg=40)` | 每帧本地逆解（耗时约 1 ms）后同 stream_joints |
| 结束流式 | `arm.stream_stop()` | 退流式并锁住；不发则看门狗约 0.3 s 停臂 |
| 按帧回调流式 | `arm.stream_run(lambda k, s: None if k >= 100 else [5 * math.sin(k / 10), -40, 60, 0, 30, 0])` | 每帧调 `fn(k, state)`（`fn` 须可调用），返回 6 个 URDF 角，返回 None 结束；fn 须 50 ms 内返回；`max_frames` None 或 ≥ 1 的整数 |

## 臂上准备

1. 停原厂 4 服务：`sudo systemctl stop marm_controller marm_communication marm_control marm_subscripber`
2. 起控制器（编译见 `arm_onboard/controller/README.md`；完整命令在 `deploy/实机部署.md`）。
   **跨机必须固定 ParticipantIndex**：臂上控制器 0、SDK 发送端 3（端口 = 7410+2×index）；
   不固定则互相发现不了，一条帧都收不到。同机仿真也建议各进程固定不同 index。
3. 上位机设 `CYCLONEDDS_URI`（AllowMulticast=false + 单播 Peer 互指，原样命令在
   `sdk/实机验收清单.md` §0.3 与 `deploy/实机部署.md` §4）。

## 单位约定（与 d1arm_sdk 一致）

| 量 | 单位 | 说明 |
|---|---|---|
| 关节角 `q_deg` | 度 | **URDF 方向**（`calib.sign`：下标 0/3 为 −1，即 J0/J3 反号；见 `arm_model/d1t.json`）；舵机角不对外 |
| 夹爪 | mm | **金属指间距**（垫面间距 = 金属 − 4）；可达 4–70 mm |
| 位置 `pos` | m | 基坐标系 |
| 姿态 | rpy 弧度 或 quat **[w,x,y,z]** | ZYX 欧拉；quat 与 d1arm_sdk fillState 同序 |
| `vel_dps`/`acc_dps2` | °/s、°/s² | 关节空间 |
| `vel`/`acc`（笛卡尔） | m/s、m/s² | vel 为期望平均末端速度，峰值 ≈1.875×vel |
| `dt_ms` | ms | 固定 50（20 Hz 控制频率），不可改 |

## API

### v1（基础，d1arm_sdk 客户端全部方法）

| 方法 | 行为 | 阻塞 |
|---|---|---|
| `state()` | 最新 ArmState 转 dict（见下节） | 等首帧 ≤5 s |
| `ping()` | 7 舵机在线 → `list[bool]`（valid_mask） | 同上 |
| `move_joints(q_deg, vel_dps=80, acc_dps2=500, async_=False)` | 关节五次多项式点到点；`vel_dps` ∈ (0, 100] °/s、`acc_dps2` > 0，超出报错 | 走完才返回 |
| `home(duration_s=6.0, async_=False)` | 回 URDF 全零位（d1arm_sdk 语义；**非原厂 HOME**），duration_s（≥ 0.5 s）走完 | 走完才返回 |
| `park(vel_dps=20.0, async_=False)` | 慢速回折叠位（原厂 HOME 姿势，J1/J2 留限位余量）后失力，供下电收臂：夹爪检查（开口 < 65 mm 即未张开 → 抛 `D1ArmError`，一轴不动；确认没夹东西后自行 `gripper(70)`）→腕部回零（J3–J5 → 0，J0–J2 保持命令角）→大小臂一起收回（J0–J2 同步到折叠位）→停 0.5 s `free_all()`；每段到位误差 ≤3° 才走下一段，超差/故障即停且不失力；已在折叠位时只做夹爪检查和失力；`vel_dps` ∈ (0, 100] °/s；`async_` 只支持 False（分段校验须逐段确认） | 走完才返回 |
| `wait_idle(timeout=30)` | 等运动结束且控制器不在运动（IDLE/HOLD/RELEASED/ESTOP/FAULT）→ bool；`timeout` > 0 s | — |
| `gripper(mm, vel_dps=20, grip_power_mw=0, async_=False)` | J6 走到夹爪标定表角（`arm_model/d1t.json`），mm ∈ [4, 70]、`vel_dps` ∈ (0, 20] °/s，超出抛 `D1ArmError`；`grip_power_mw` 只支持 0（舵机自保护兜底），传别的值报错 | 走完才返回 |
| `free_all()` | 全轴 RELEASE，**臂会掉，先托住** | 确认 RELEASED |
| `lock_all()` | 全轴 HOLD 锁当前姿态（失力后锁在当时实测角，不跳） | 确认 HOLD |
| `estop()` | 软急停（中止轨迹/流式）；实测 ESTOP 位 111 ms、各轴停住 129 ms | 确认 ESTOP |
| `reset()` | **本 SDK 加的**：ESTOP/故障恢复（d1arm_sdk 客户端没有；ESTOP 后必须先 reset） | 确认 IDLE/HOLD |
| `stream_start()` | 进流式：先 HOLD 当前姿态，之后每帧发 `stream_joints` 最新目标 | 确认 HOLD |
| `stream_joints(q_deg, j6_deg=None)` | 更新流式目标（6 关节 URDF 角；`j6_deg` 给 J6 舵机角 [−30, 64.5]，None = 保持上次命令角，不取实测角），下一帧生效 | 立即 |
| `stream_run(fn, max_frames=None)` | **按顺序播放**（回放、ACT 动作块、脚本轨迹）：SDK 每帧调 `fn(k, state)` 取第 k 帧目标（6 关节），返回 None 结束；节奏由 SDK 跟臂走，不写循环不写 sleep；结束自动 HOLD；返回 `frames`、`late_frames`（fn 超过 50 ms 的帧数）；`max_frames` None 或 ≥ 1 的整数 | 播完才返回 |
| `wait_frame(timeout=1.0)` | 等下一帧 ArmState 处理完（本帧命令已发；`timeout` > 0 s）。**流式循环用它控节奏，不要用 sleep**：本机时钟与臂节拍漂移会漏帧（停一下）或一帧跳两步（蹭一下） | 下一帧 |
| `stream_stop()` | 退流式并发 HOLD（若不发，看门狗约 0.3 s 也停臂） | 确认 HOLD |
| `close()` | 停发送线程；流式中 close → 看门狗停臂 | — |

### v2（运动学，与 d1arm_sdk 服务端命令集对齐）

| 方法 | 行为 |
|---|---|
| `fk(q_deg)` | URDF 角（度）→ `{"pos", "quat"[w,x,y,z], "rpy"(rad)}` |
| `ik(pos, rpy=None, quat=None, seed=None)` | 位姿 → URDF 角（度）；`seed` 恰好 6 个 URDF 角，None = 当前实测角；DLS + 限位 + 零空间偏置 + 随机重启（d1arm_sdk IkParams 默认值） |
| `move_j(pos, rpy, quat, vel_dps=80, acc_dps2=500, async_=False)` | IK 一次后关节空间走 |
| `move_l(pos, rpy, quat, vel=0.15, acc=2.0, async_=False)` | 末端直线（逐点 IK + 五次多项式时间最优化）；`vel` > 0 m/s、`acc` > 0 m/s² |
| `move_c(pos, via, rpy, quat, via_rpy, via_quat, vel=0.15, acc=2.0, async_=False)` | 三点定圆弧（d1arm_sdk 客户端缺，按其服务端 planArc 补上）；`via` 3 个数（m），`via_rpy`/`via_quat` 规则同 `rpy`/`quat` |
| `stream_pose(pos, rpy, quat, j6_deg=None)` | 流式目标改位姿（IK 后走 stream_joints；`j6_deg` 同上） |

### `state()` 键

d1arm_sdk `fillState` 键：`q_deg`（6 URDF 角）、`servo_raw_deg`（6 舵机角）、`gripper_mm`、
`calibrated`（恒 True）、`tcp`（`pos`/`quat`/`rpy`）、`online`（7 bool）、
`voltage_mv`（7）、`mode`、`stamp_ms`；本控制器独有：`seq`、`ctrl_state`（原始 0–6）、
`fault`、`warn`、`temp_c`（7）、`current_ma`（7）。

`mode` 取值（与 d1arm_sdk 不同：控制器状态原样给出，不合并）：

| `mode` | `ctrl_state` | 含义 |
|---|---|---|
| `MOVING` | — | SDK 正在执行 move_joints / move_j / move_l / move_c / home / gripper |
| `INIT` | 0 | 控制器启动中，不能运动 |
| `IDLE` | 1 | 控制器刚启动、尚未接管：只读，不发运动帧 |
| `STREAMING` | 2 | 按流式目标运动中（看门狗计时） |
| `HOLD` | 3 | 舵机出力锁在当前姿态 |
| `RELEASED` | 4 | 失力，可手推 |
| `ESTOP` | 5 | 软急停（锁存），`reset()` 恢复 |
| `FAULT` | 6 | 故障（锁存），查 `fault` 位后 `reset()` |

### 错误处理

全部抛 `d1t_sdk.D1ArmError`（与 d1arm_sdk 同名的异常口径）：参数类型 / 格式 / 范围不符报错带参数名、
收到的值和可接受范围（如 `参数 vel_dps = 200 超出范围，可接受 (0, 100] °/s`）；越界报错带关节号
（如 `joint 3 (J3) = 170.0° 超出 [-163.0, 166.0]`）、IK 无解带误差、跨机没收到
ArmState 会提示查 CYCLONEDDS_URI/ParticipantIndex。

## 发送机制（重要）

- 后台线程**每收到一帧 ArmState 发恰好一条 ArmCmd**：按自身时钟定时发会和控制器节拍
  漂移，出现一阵阵顿挫；帧同步发送消除了这个问题。
- 轨迹按帧序号 seq 推进样本下标，**丢帧按 seq 差补样本序号**（波形时间保持）；
  首样本重发直到控制器进 STREAMING。
- 单发命令（HOLD/ESTOP/RESET/RELEASE）每帧重发（新 seq/stamp 严格递增）直到
  ctrl_state 确认或 5 s 超时。
- stamp 用 CLOCK_REALTIME 且严格递增（控制器按 stamp 判新旧，不大于上一条的直接丢弃）。

## 安全须知

1. **free_all 臂会掉**——托住、撤手再调。
2. **estop 后必须 `reset()`** 才能再动；软急停时延实测 111/129 ms，
   真急停按物理急停钮。
3. 运动前确认臂姿态安全：离限位 ≥25°，不碰桌面/自身/周围物品。
4. **home = 臂前伸**（URDF 全零），执行前确认路径无障碍。
5. 夹持力暂用舵机自保护（grip_power_mw=0）+ 温度守卫（45 ℃ 降出力、50 ℃ 释放，
   控制器内置）。
6. 中止流式/轨迹：Ctrl-C 或 `close()`，看门狗约 0.3 s 停臂。
7. **park 前先张开夹爪**：夹爪夹物时，若目标贴着物体的停住位置，电流和位置读数与空夹无法区分，SDK 判断不了有没有夹东西。所以 park 只在夹爪已张开时执行，是否有物由人确认后用 `gripper 70` 张开。
8. **两条命令衔接处的小幅下沉**：上一条运动命令结束、下一条开始时，负载轴（J1/J2）在起转前会先下沉约 0.5° 再跟上轨迹，不影响目标值、不累积。流式控制（`stream_joints` / `stream_pose`，连续发送不切换保持）不受影响。

## 离线测试

```bash
cd sdk && python3 tests/run_all.py
```

64 项（不上臂、不连 DDS）：FK 金标逐项对照 d1arm_sdk `tests/test_main.cpp`（22 检查点，
1e-9/1e-6）；IK 300 随机位姿 ≥97% 收敛（实测 300/300，误差 ≤0.5 mm）；直线偏差
≤2 mm（实测 0.36 mm）、1/4 圆弧径向偏差 ≤2 mm（实测 0.20 mm）；角度换算往返；
夹爪标定表点精确 + 0–50° 段对旧公式 ≤1 mm；假控制器回放验证每帧一条、丢 3 帧样本下标
跳 3、estop 吞 3 条后重发确认、夹爪只动 J6 且 ≤20°/s、流式带 J6（给定值随动、None 保持
上次命令角）、取上限速度时逐轴不超限；全部方法与命令行的参数校验（`tests/test_param_validation.py`）；
臂参数金标逐位相同 + 限位与控制器 config.hpp 自动比对（`tests/test_arm_model_identity.py`）。

## 与 d1arm_sdk 的关系与差异

运动学、轨迹规划与接口设计源自开源项目 d1arm_sdk（MIT，授权移植并随本项目同许可开源）。
借了：接口（客户端与服务端命令集）、运动学、轨迹生成、关节模型、标定结构。
**没有**移植它的臂上守护进程、单舵机轮询、TCP 总线。

| 差异 | d1arm_sdk | 本 SDK |
|---|---|---|
| 传输 | TCP → 臂上守护进程 | DDS 直连 d1t_arm_ctrl（臂上跑控制器，上位机只跑库） |
| 发送节拍 | 守护进程内部时钟下发 | 每帧 ArmState 发一条 + 丢帧按 seq 补偿 |
| 夹爪 mm↔角 | 线性 0°↔0mm、64.5°↔65mm（不采用） | 标定表 8 点分段线性（0–50° 段与旧公式差 ≤1 mm；>50° 段必须用表） |
| 限位 | URDF ±135/±90（标定后） | 实测限位换算（`arm_model/d1t.json`，控制器是最终裁判） |
| 限速 | 全局 max_vel 100 °/s | `vel_dps` 超过控制器限速（各轴 100 °/s）报错不动 |
| estop 语义 | 服务端执行即返回 | 重发直到 ctrl_state 确认（跨机 BEST_EFFORT 不丢） |
| 新增 | — | `reset()`、`state()` 独有键（seq/fault/warn/temp_c/current_ma/ctrl_state） |
| 移除 | `set_joint`/`damping`/`watchdog`/`subscribe`/`events` | 与流式冲突或由臂上控制器固定承担，不做 |

已知未验证项：**FK/IK 绝对精度**——标定 offset 全 0、待米尺复核（首次部署必做，
见 `deploy/实机部署.md` §0）。方向换算（sign）已实测确认。直线/圆弧端点请按 ±1 cm
量级预期；另外同构型下 π 半圆弧的对侧超出工作空间（路径 IK 会报错），1/4 圆弧正常。

## 致谢

运动学、轨迹规划与接口设计源自 d1arm_sdk 开源项目（MIT）。本目录依其授权移植并
随本项目同许可开源，许可与署名见 [LICENSE](LICENSE)。
