# Joy-Con 遥操作 D1

> **来源**：[`gitee.com/gccodespace/d1arm_sdk`](https://gitee.com/gccodespace/d1arm_sdk) 分支 `joycon`，
> 提交 [`f77d370`](https://gitee.com/gccodespace/d1arm_sdk/commit/f77d370) 的 `clients/tele/`
> （`joycon_teleop.py` 及其测试与说明），原作者 **bxdw**（署名 bxd，见 `sdk/LICENSE`），
> MIT 授权随本项目发布。`pico_teleop.py` 及其测试不迁。

## 是什么

用单个 Joy-Con（右/左手柄均可）遥操作 D1 末端的位置、朝向和夹爪开度：

- **姿态**：转动手腕驱动六轴 IK 目标朝向（`R_tcp = R_tcp0 · C · (R_joy0ᵀ · R_joy) · Cᵀ`，
  角速度限幅，四元数运算）；`--axes-deg` 调整手柄轴到 TCP 轴的固定旋转 C。
- **平移**：摇杆纵/横轴沿目标 TCP 的 X/Y 轴，R / 按下摇杆（左柄 L / 按下摇杆）沿基座 ±Z。
- **夹爪**：使能状态下 ZR（左柄 ZL）每按一次在 `--open-mm` / `--close-mm` 间切换。
- **接管 / 重新握持**：先松开使能键、再按住接管（右手柄 A，左手柄右方向键），以当时的实测
  TCP 与手腕朝向为参考；松开即停（HOLD），换个姿势再按住可从当前末端姿态继续。
- **退出**：Y（左柄左方向键）或 Ctrl-C；HID 断连、反馈停更、fault、舵机离线、跟随偏差 >18°
  都会自动退出。

手柄映射与姿态算法与来源版本**一致**；本迁移只换了驱动层：d1armd（TCP 7777 JSON RPC）→
`d1t_sdk.D1Arm`（DDS 直连 `d1t_arm_ctrl`），FK/IK 改在上位机本地算，主循环按控制器帧同步
（`wait_frame`，20 Hz，不写 sleep），夹爪改为流式 J6 目标角 + 停滞守卫。差异明细见下表。

| 操作 | 右手柄 | 左手柄 |
| --- | --- | --- |
| 按住使能（接管） | A | 右方向键 |
| 转动手腕 | 驱动 TCP 朝向 | 同左 |
| 摇杆纵轴 / 横轴 | 沿 TCP 的 X / Y 平移 | 同左 |
| 基座 +Z / −Z | R / 按下摇杆 | L / 按下摇杆 |
| 切换夹爪（使能中） | ZR | ZL |
| 结束程序 | Y 或 Ctrl+C | 左方向键或 Ctrl+C |

摇杆越过 joycon-robotics 原始阈值（4000 / 1000）才移动；纵轴高值沿 +X，横轴高值沿 −Y；
保持时间决定行程，回中停止累加。

## 接口对照表（d1armd → d1t_sdk，已逐行核实）

| 原调用（d1armd RPC） | 本项目对应（d1t_sdk） | 核对结论 |
| --- | --- | --- |
| `call('state')` | `D1Arm.state()` | `q_deg` 同为 **6 个 URDF 角（度）**：本项目 J6=夹爪不在 `q_deg` 里，单独以 `gripper_mm`（金属指间距）暴露；`online` 同为 7 元素（J0–J6），全 True 判据沿用。`mode` 直接给控制器状态名（IDLE/STREAMING/HOLD/RELEASED/ESTOP/FAULT）：**接管接受 IDLE 或 HOLD**（`stream_stop` 后停在 HOLD，仅首启是 IDLE），RELEASED/ESTOP/FAULT 拒绝接管。错误位从 `err` 改读 `fault`（≠0 即停）；`warn` 非致命不拦截。停更判据从 `stamp_ms` 改用 `seq`（每帧 +1），>0.5 s 不变即报错，逻辑不变 |
| `call('fk')` / `call('ik')` | `D1Arm.fk(q_deg)` / `D1Arm.ik(pos, quat=…, seed=…)` | **TCP 定义一致**：TCP 取两夹爪指 origin 中点（`sdk/arm_model/d1t.json`），SDK 与 d1arm_sdk 同名同参同单位，FK 金标逐项对照 d1arm_sdk `tests/test_main.cpp`（22 检查点，1e-9/1e-6）。位姿表示一致：`pos` 米（基坐标系）+ `quat` **[w,x,y,z]**（与 fillState 同序），四元数运算代码不改。IK 无解从 RPC 错误改为抛 `D1ArmError("IK 无解（位置误差 … mm，姿态误差 …°）")`，按前缀识别后继续缩步重试，口径不变。**IK 在上位机本地算**（不经 RPC）：实测上位机（x86_64，Ubuntu 22.04，Python 3.10）遥操作典型工况（seed=上一有效目标）**0.43 ms/次**、200/200 收敛；远种子带重启 34.6 ms/次——50 ms 帧预算内富余（macOS arm64 对照：0.31 / 25.5 ms） |
| `call('watchdog', ms=500)` | （删除，无对应） | `d1t_arm_ctrl` 看门狗**内置且不可配**：流式命令一停约 **0.3 s** 自动停臂，比 d1armd 的 500 ms 更紧。删掉该调用，断流停臂由控制器承担 |
| `stream_start` / `stream_joints` / `stream_stop` | `D1Arm.stream_start()` / `stream_joints(q_deg, j6_deg=None)` / `stream_stop()` | 控制器周期固定 **50 ms（20 Hz）不可改**：主循环改用 `arm.wait_frame()` 跟帧（每帧恰好更新一次目标，不写 sleep），删 `--hz` 参数，dt 固定 0.05 s，位移/角速度/关节速度限幅公式不变。控制器逐轴限速 100°/s；脚本自带 `--joint-dps 25` 的单帧步长检查**保留**。`stream_start` 先 HOLD 当前姿态再进流式（接管首帧仍发实测角、原位保持）；`stream_stop` 后控制器停在 HOLD |
| `call('gripper', mm=…)`（流式中） | `stream_joints` 的 `j6_deg` 参数 | 流式中**不能**另发 `gripper()`（SDK 会拒绝：正在流式）；mm↔J6 舵机角用 SDK 标定表函数 `gripper_mm_to_deg` / `gripper_deg_to_mm`（8 点分段线性实测标定；55 mm↔40.3°、10 mm↔−18.4°）。**合拢守卫**（新增）：J6 每帧发目标角，实测开度到位（\|Δmm\|≤1.5）或停滞（0.6 s 内进展 <0.5 mm）/超时（行程÷100°/s + 1 s）即冻结在当前实测开度，不让 J6 长时间向堵转方向压。已知差异：流式中 J6 受 100°/s 轴限速（`gripper()` 的 20°/s 不作用于流式），开合更快；「流式带 j6_deg」属 SDK 未做实机验收项（`sdk/README.md` 已知限制 #8），实机首用按验收清单低速验证 |

补充核对：

- 连接：`RpcArm(host, port, request_timeout_s=0.4)` → `D1Arm()`，无地址/端口参数，DDS 环境由
  `source deploy/env.sh sim|real` 提供（CYCLONEDDS_URI + ParticipantIndex）；脚本自行把仓库
  `sdk/` 加进 `sys.path`。
- 关节限位：原脚本硬编码对称限位 [±135, ±90, ±90, ±135, ±90, ±135]；本项目实测限位
  （`d1t.json`，URDF 空间）与其**逐轴取较小者**（实际仅 J2 高侧 89° 比原 90° 紧）。
  原脚本README 说的「J5 余量」作用于 `q_deg[4]`，即本项目 0 起编号的 **J4**（±90° 留 5°，
  `--j5-margin-deg`，参数名保留原拼写），逻辑不变。

## 怎么跑

### 仿真（无臂、无手柄，验证完整指令链）

```bash
# 终端 1：起 --sim 控制器（lo 单播；见 deploy/虚拟部署.md）
bash deploy/sim_start.sh
# 终端 2：假手柄全链路（接管 → 流式抬升/前进/转腕 → 夹爪合拢/张开 → 松开停臂）
source deploy/env.sh sim
python3 apps/joycon_teleop/sim_chain.py        # 结束打印 4 条 [PASS] 即通过
```

### 上位机 + 真手柄

```bash
source deploy/env.sh real <上位机接臂网卡名>     # 臂上需已起 d1t_arm_ctrl（deploy/实机部署.md）
python3 apps/joycon_teleop/joycon_teleop.py --side right --dry-run    # 先预演：只读 + 本地 IK，臂不动
python3 apps/joycon_teleop/joycon_teleop.py --side right \
    --speed-mps 0.02 --angular-dps 10          # 再低速接管；熟练后用默认 0.04 / 20
```

需要 `joyconrobotics`（Ubuntu 20.04/22.04，源码安装 + dkms 内核模块，步骤见
[`实机验收清单.md`](实机验收清单.md) §0）。**实机首次使用请按
[`实机验收清单.md`](实机验收清单.md) 走**（含两项首验：流式 J6、J6 100°/s）。

参数（默认同来源版本）：`--side right|left`、`--speed-mps 0.04`（≤0.15）、`--angular-dps 20`、
`--joint-dps 25`（≤100）、`--j5-margin-deg 5`、`--open-mm 55` / `--close-mm 10`、
`--axes-deg ROLL PITCH YAW`。相对来源删了 `--host/--port/--hz`（无 TCP；循环节拍锁定控制器 20 Hz）。

测试（不需要控制器与手柄）：`python3 -m pytest apps/joycon_teleop/tests -q`，10 项。

## 已知限制

1. **实机未验收项**：流式带 J6 目标（`sdk/README.md` 已知限制 #8）；流式中 J6 按 100°/s 轴限速
   开合（比 `gripper()` 快 5 倍），夹爪守卫的到位/停滞判据只在 `--sim` 上验证过——实机首用按
   验收清单第 3 步，手离开夹爪路径。
2. **持续手腕转动会频繁触发限幅保持**：本项目 IK（与原作者算法逐行等价，参数同源）对相邻帧
   小步长目标的解存在约 1–3° 跳变，与默认 `--joint-dps 25` 的 1.25°/帧步长检查同量级。实测
   （理想跟随臂 + 真运动学）：纯平移（摇杆 / 升降）40/40 帧全推进；持续转手腕时约 1/4 帧推进、
   其余保持（打印 `Holding:`），输入放缓或松开使能即恢复——这是原脚本「缩步重试、仍失败保持」
   的设计行为，不是故障。缓解：`--angular-dps 10` 或 `--joint-dps 50`（仍低于控制器 100°/s
   安全限速）；要根治需改 SDK IK（如收紧 rot_tol 或做解连续化），超出本迁移范围。
3. **循环节拍锁定 20 Hz**：控制器周期 50 ms 不可改，原 `--hz` 删除；dt 固定 0.05 s。
4. **限位口径**：接管与 IK 检查用「来源脚本对称限位 ∩ 实测限位」（J2 高侧 89°）；臂实测限位更
   宽的轴仍按来源的保守值。原「J5 余量」= 本项目 J4（0 起编号），参数名 `--j5-margin-deg` 保留
   原拼写以免老用户踩错。
5. **FK/IK 绝对精度 ±1 cm 量级**（零偏未精标、J2 重力下沉 SDK 不补偿）：遥操作看相对跟随，
   `--dry-run` 预演的绝对位置也按此预期。
6. **断流停臂 0.3 s**（控制器内置看门狗，替代原 500 ms 可配 watchdog）：松开使能、Ctrl-C、
   脚本崩溃都会触发，臂停在 HOLD。
7. **夹持力**：守卫冻结后靠舵机自保护（P=0）+ 控制器温度守卫（45 ℃ 降出力 / 50 ℃ 释放），
   无力控；长时间夹持发热仍可能触发 J6 温度保护。
8. `--dry-run` 仍需要控制器在线（读状态帧做 FK/IK 预演），不是完全离线。
9. joycon-robotics 仅支持 Ubuntu 原生环境（内核模块 + udev；24.04 需上游 FAQ 的补丁），不可用于
   WSL/虚拟机。
