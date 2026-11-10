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
python3 -m d1t_sdk state | ping | home | free | lock | estop | reset
python3 -m d1t_sdk joint 0,-40,60,0,30,0
python3 -m d1t_sdk line 0.35,0,0.42,0,1.5708,0
python3 -m d1t_sdk gripper 45
```

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
| `move_joints(q_deg, vel_dps=80, acc_dps2=500, async_=False)` | 关节五次多项式点到点（vel 逐轴钳到控制器限速 100 °/s） | 走完才返回 |
| `home(duration_s=6.0, async_=True)` | 回 URDF 全零位（d1arm_sdk 语义；**非原厂 HOME**），duration_s 走完 | 默认不阻塞 |
| `wait_idle(timeout=30)` | 等运动结束且控制器不在运动（IDLE/HOLD/RELEASED/ESTOP/FAULT）→ bool | — |
| `gripper(mm, vel_dps=20, grip_power_mw=0, async_=False)` | J6 走到夹爪标定表角（`arm_model/d1t.json`），≤20°/s；P=0 舵机自保护兜底 | 走完才返回 |
| `free_all()` | 全轴 RELEASE，**臂会掉，先托住** | 确认 RELEASED |
| `lock_all()` | 全轴 HOLD 锁当前姿态（失力后锁在当时实测角，不跳） | 确认 HOLD |
| `estop()` | 软急停（中止轨迹/流式）；实测 ESTOP 位 111 ms、各轴停住 129 ms | 确认 ESTOP |
| `reset()` | **本 SDK 加的**：ESTOP/故障恢复（d1arm_sdk 客户端没有；ESTOP 后必须先 reset） | 确认 IDLE/HOLD |
| `stream_start()` | 进流式：先 HOLD 当前姿态，之后每帧发 `stream_joints` 最新目标 | 确认 HOLD |
| `stream_joints(q_deg, j6_deg=None)` | 更新流式目标（6 关节 URDF 角；`j6_deg` 给 J6 舵机角，None = 保持上次命令角，不取实测角），下一帧生效 | 立即 |
| `stream_run(fn, max_frames=None)` | **按顺序播放**（回放、ACT 动作块、脚本轨迹）：SDK 每帧调 `fn(k, state)` 取第 k 帧目标（6 关节），返回 None 结束；节奏由 SDK 跟臂走，不写循环不写 sleep；结束自动 HOLD；返回 `frames`、`late_frames`（fn 超过 50 ms 的帧数） | 播完才返回 |
| `wait_frame(timeout=1.0)` | 等下一帧 ArmState 处理完（本帧命令已发）。**流式循环用它控节奏，不要用 sleep**：本机时钟与臂节拍漂移会漏帧（停一下）或一帧跳两步（蹭一下） | 下一帧 |
| `stream_stop()` | 退流式并发 HOLD（若不发，看门狗约 0.3 s 也停臂） | 确认 HOLD |
| `close()` | 停发送线程；流式中 close → 看门狗停臂 | — |

### v2（运动学，与 d1arm_sdk 服务端命令集对齐）

| 方法 | 行为 |
|---|---|
| `fk(q_deg)` | URDF 角（度）→ `{"pos", "quat"[w,x,y,z], "rpy"(rad)}` |
| `ik(pos, rpy=None, quat=None, seed=None)` | 位姿 → URDF 角（度）；DLS + 限位 + 零空间偏置 + 随机重启（d1arm_sdk IkParams 默认值） |
| `move_j(pos, rpy, quat, vel_dps=80, acc_dps2=500, async_=False)` | IK 一次后关节空间走 |
| `move_l(pos, rpy, quat, vel=0.15, acc=2.0, async_=False)` | 末端直线（逐点 IK + 五次多项式时间最优化） |
| `move_c(pos, via, rpy, quat, via_rpy, via_quat, vel=0.15, acc=2.0, async_=False)` | 三点定圆弧（d1arm_sdk 客户端缺，按其服务端 planArc 补上） |
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

全部抛 `d1t_sdk.D1ArmError`（与 d1arm_sdk 同名的异常口径）：越界报错带关节号
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

## 离线测试

```bash
cd sdk && python3 tests/run_all.py
```

46 项（不上臂、不连 DDS）：FK 金标逐项对照 d1arm_sdk `tests/test_main.cpp`（22 检查点，
1e-9/1e-6）；IK 300 随机位姿 ≥97% 收敛（实测 300/300，误差 ≤0.5 mm）；直线偏差
≤2 mm（实测 0.36 mm）、1/4 圆弧径向偏差 ≤2 mm（实测 0.20 mm）；角度换算往返；
夹爪标定表点精确 + 0–50° 段对旧公式 ≤1 mm；假控制器回放验证每帧一条、丢 3 帧样本下标
跳 3、estop 吞 3 条后重发确认、夹爪只动 J6 且 ≤20°/s、流式带 J6（给定值随动、None 保持
上次命令角）、限速逐轴钳制；
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
| 限速 | 全局 max_vel 100 °/s | 逐轴钳到控制器限速（各轴 100 °/s） |
| estop 语义 | 服务端执行即返回 | 重发直到 ctrl_state 确认（跨机 BEST_EFFORT 不丢） |
| 新增 | — | `reset()`、`state()` 独有键（seq/fault/warn/temp_c/current_ma/ctrl_state） |
| 移除 | `set_joint`/`damping`/`watchdog`/`subscribe`/`events` | 与流式冲突或由臂上控制器固定承担，不做 |

已知未验证项：**FK/IK 绝对精度**——标定 offset 全 0、待米尺复核（首次部署必做，
见 `deploy/实机部署.md` §0）。方向换算（sign）已实测确认。直线/圆弧端点请按 ±1 cm
量级预期；另外同构型下 π 半圆弧的对侧超出工作空间（路径 IK 会报错），1/4 圆弧正常。

## 致谢

运动学、轨迹规划与接口设计源自 d1arm_sdk 开源项目（MIT）。本目录依其授权移植并
随本项目同许可开源，许可与署名见 [LICENSE](LICENSE)。
