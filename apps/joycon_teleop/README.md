# Joy-Con 遥操作 D1（迁移中）

> **来源**：[`gitee.com/gccodespace/d1arm_sdk`](https://gitee.com/gccodespace/d1arm_sdk) 分支 `joycon`，
> 提交 [`f77d370`](https://gitee.com/gccodespace/d1arm_sdk/commit/f77d370) 的 `clients/tele/`
> 三件（`joycon_teleop.py`、`test_joycon_teleop.py`、`README.md`），原作者 **bxdw**（署名 bxd，
> 见 `sdk/LICENSE`），MIT 授权随本项目发布。
>
> 迁移目标：驱动臂的调用从原作者的 d1armd（TCP 7777 JSON）换成 `d1t_sdk.D1Arm`（DDS 直连
> `d1t_arm_ctrl`），手柄映射与姿态算法不动。`pico_teleop.py` 及其测试不迁。
> 本 README 为迁移起点，接口对照表与使用说明随后补全。

## 接口对照表（d1armd → d1t_sdk，已逐行核实）

| 原调用（d1armd RPC） | 本项目对应（d1t_sdk） | 核对结论 |
| --- | --- | --- |
| `call('state')` | `D1Arm.state()` | `q_deg` 同为 **6 个 URDF 角（度）**：本项目 J6=夹爪不在 `q_deg` 里，单独以 `gripper_mm`（金属指间距）暴露；`online` 同为 7 元素（J0–J6），全 True 判据沿用。`mode` 直接给控制器状态名（IDLE/STREAMING/HOLD/RELEASED/ESTOP/FAULT）：**接管接受 IDLE 或 HOLD**（`stream_stop` 后停在 HOLD，仅首启是 IDLE），RELEASED/ESTOP/FAULT 拒绝接管。错误位从 `err` 改读 `fault`（≠0 即停）；`warn` 非致命不拦截。停更判据从 `stamp_ms` 改用 `seq`（每帧 +1），>0.5 s 不变即报错，逻辑不变 |
| `call('fk')` / `call('ik')` | `D1Arm.fk(q_deg)` / `D1Arm.ik(pos, quat=…, seed=…)` | **TCP 定义一致**：TCP 取两夹爪指 origin 中点（`sdk/arm_model/d1t.json`），SDK 与 d1arm_sdk 同名同参同单位，FK 金标逐项对照 d1arm_sdk `tests/test_main.cpp`（22 检查点，1e-9/1e-6）。位姿表示一致：`pos` 米（基坐标系）+ `quat` **[w,x,y,z]**（与 fillState 同序），四元数运算代码不改。IK 无解从 RPC 错误改为抛 `D1ArmError("IK 无解（位置误差 … mm，姿态误差 …°）")`，按前缀识别后继续缩步重试，口径不变。**IK 在上位机本地算**（不经 RPC）：实测（macOS arm64，Python 3.9）遥操作典型工况（seed=上一有效目标）0.31 ms/次、200/200 收敛；远种子带重启 25.5 ms/次——50 ms 帧预算内富余（上位机 x86_64 实测见「怎么跑」一节） |
| `call('watchdog', ms=500)` | （删除，无对应） | `d1t_arm_ctrl` 看门狗**内置且不可配**：流式命令一停约 **0.3 s** 自动停臂，比 d1armd 的 500 ms 更紧。删掉该调用，断流停臂由控制器承担 |
| `stream_start` / `stream_joints` / `stream_stop` | `D1Arm.stream_start()` / `stream_joints(q_deg, j6_deg=None)` / `stream_stop()` | 控制器周期固定 **50 ms（20 Hz）不可改**：主循环改用 `arm.wait_frame()` 跟帧（每帧恰好更新一次目标，不写 sleep），删 `--hz` 参数，dt 固定 0.05 s，位移/角速度/关节速度限幅公式不变。控制器逐轴限速 100°/s；脚本自带 `--joint-dps 25` 的单帧步长检查**保留**。`stream_start` 先 HOLD 当前姿态再进流式（接管首帧仍发实测角、原位保持）；`stream_stop` 后控制器停在 HOLD |
| `call('gripper', mm=…)`（流式中） | `stream_joints` 的 `j6_deg` 参数 | 流式中**不能**另发 `gripper()`（SDK 会拒绝：正在流式）；mm↔J6 舵机角用 SDK 标定表函数 `gripper_mm_to_deg` / `gripper_deg_to_mm`（8 点分段线性实测标定；55 mm↔40.3°、10 mm↔−18.4°）。**合拢守卫**（新增）：J6 每帧发目标角，实测开度到位（\|Δmm\|≤1.5）或停滞（0.6 s 内进展 <0.5 mm）/超时（行程÷100°/s + 1 s）即冻结在当前实测开度，不让 J6 长时间向堵转方向压。已知差异：流式中 J6 受 100°/s 轴限速（`gripper()` 的 20°/s 不作用于流式），开合更快；「流式带 j6_deg」属 SDK 未做实机验收项（`sdk/README.md` 已知限制 #8），实机首用按验收清单低速验证 |

补充核对：

- 连接：`RpcArm(host, port, request_timeout_s=0.4)` → `D1Arm()`，无地址/端口参数，DDS 环境由
  `source deploy/env.sh sim|real` 提供（CYCLONEDDS_URI + ParticipantIndex）；脚本自行把仓库
  `sdk/` 加进 `sys.path`。
- 关节限位：原脚本硬编码对称限位 [±135, ±90, ±90, ±135, ±90, ±135]；本项目实测限位
  （`d1t.json`，URDF 空间）与其**逐轴取较小者**（实际仅 J2 高侧 89° 比原 90° 紧），J5 余量
  （`--j5-margin-deg`）逻辑不变。

---

以下为原版说明（迁移前，针对 d1armd）：

# Joy-Con 遥操作 D1

PICO 用户见 [PICO 无相机遥操作](PICO.md)。

在连接手柄的 Ubuntu 主机运行。Python 环境需要 NumPy 和可用的 `joyconrobotics.JoyconRobotics`。机械臂侧启动已加载本机关节标定的 d1armd，等七个舵机在线、状态为 IDLE 后使用。L2 模式填写开发机 d1armd 地址；L3 填写板上 d1armd 地址，默认端口 7777。

在仓库根目录运行：

```bash
# 预览：读取实测关节角，用服务端 FK/IK 推演并打印目标
python clients/tele/joycon_teleop.py --side right --dry-run
# 控制：相同手柄映射，加入关节流式和夹爪指令
python clients/tele/joycon_teleop.py --side right --host 127.0.0.1
```

手柄启动时按 joycon-robotics 提示放平校准。先松开使能键，再按住接管；程序以此时的实测 TCP 和手柄朝向建立参考。松开即结束流式，可换个舒适的手腕姿势后重新按住。可以先用指向 `d1armd --sim` 的普通模式验证完整指令链。

| 操作 | 右手柄 | 左手柄 |
| --- | --- | --- |
| 按住使能 | A | 右方向键 |
| 转动手腕 | 相对接管姿态驱动六轴 IK 目标朝向 | 同左 |
| 推摇杆纵轴 / 横轴 | 沿目标 TCP 的 X / Y 平移 | 同左 |
| 沿基座 +Z / −Z 平移 | R / 按下摇杆 | L / 按下摇杆 |
| 使能时切换夹爪 | ZR | ZL |
| 结束程序 | Y 或 Ctrl+C | 左方向键或 Ctrl+C |

摇杆越过驱动原始阈值 4000 或 1000 才移动。纵轴高值沿 +X，横轴高值沿 −Y。保持摇杆的时间决定行程，回中后停止累加。夹爪从接管时的实测开度判定初始开合状态，每次扳机切换发一条毫米开度指令。

姿态采用 `R_tcp = R_tcp0 · C · (R_joy0ᵀ · R_joy) · Cᵀ`。默认手柄局部轴与 TCP 局部轴对应，`--axes-deg ROLL PITCH YAW` 可调整两者的固定旋转 C（度，ZYX 合成）。驱动提供相对握持参考所需的 RPY，脚本以四元数进行旋转组合和角速度限制。手柄平移通过摇杆输入，竖直升降通过按键输入。

默认 20 Hz，平移上限 0.04 m/s，转动上限 20°/s，关节指令上限 25°/s，可分别通过 `--hz`、`--speed-mps`、`--angular-dps`、`--joint-dps` 调整。六关节使用标定后的角度，以上一条有效关节目标作为 IK 种子。J5 机械范围 ±90°，默认各留 5°，通过 `--j5-margin-deg` 设置；接管时若已在余量区，允许向内退出。夹爪默认 55 mm / 10 mm，通过 `--open-mm` 和 `--close-mm` 修改。

遇到不可达、关节越限或步长过大，程序缩小本步重试；仍失败就重复最后有效目标。HID 报告停更、遥测停更、舵机离线或执行跟随偏差过大会结束控制。实控接管时将 d1armd 流式看门狗设为 500 ms，该设置在本次 daemon 进程内持续生效。服务端失联时，停止依赖板端舵机和控制器实际状态；流式停止属于软件保持。使用期间由此客户端独占运动指令。

参考 [LeRobot JoyConController](https://github.com/box2ai-robotics/lerobot-joycon/blob/main/lerobot/common/robot_devices/controllers/joycon_controller.py) 与 [joycon-robotics 输入接口](https://github.com/box2ai-robotics/joycon-robotics/blob/master/joyconrobotics/joyconrobotics.py)。D1 使用完整 TCP 位姿求解，保留独立侧移和 J5 余量。测试命令为 `python -m unittest discover -s clients/tele`，使用模拟输入和 RPC。
