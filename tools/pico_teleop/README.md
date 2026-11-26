# pico_teleop · PICO 手柄遥操作

PICO 手柄 → 离合映射 → `d1t_sdk`（逆解 + 流式）→ 臂。含夹爪全行程、夹住自动锁、
振动反馈、断流停臂。仿真和真臂用同一个程序，只换网卡参数（`deploy/env.sh sim|real`）。

```
PICO ──ZMQ──▶ PicoProbe（tools/pico_probe：离合映射，输出基座系目标位姿 + 扳机）
                 │  ee 参考每帧取 arm.state() 实测 TCP（不再是假姿态）
                 ▼
        TeleopLoop（本目录 teleop_loop.py，一帧一步，跟 arm.wait_frame() 走）
          ├─ heading：水平朝向修正（绕 Z 转 yaw）
          ├─ limiter：单帧步长 / 工作空间钳制
          ├─ GripLock：扳机目标 → 实际下发的 j6_deg（夹住后锁停住角）
          ├─ arm.stream_pose(pos, quat, j6_deg)
          └─ haptics.pulse(side, pattern)   # 事件发往 PICO 振动页面
```

## 1. 接口表（先接口后实现）

| 件 | 接口 | 行为 |
|---|---|---|
| `TeleopLoop.step(status, arm_state)` → dict | 入：PicoProbe 的 status、`arm.state()` 字典；出：一次 `stream_pose`（含是否发送、下发目标、j6、事件、限幅/逆解失败计数，供日志） | 接合 → `stream_start` 并发目标（首帧目标 = 当前实测 TCP，不跳）；脱开 → 持续发当前**命令**位姿保持（用命令角不用实测角），j6 继续随扳机；STALE → `stream_stop`（臂保持），冻结 GripLock；首次接合前不发命令也不推进 j6（避免接合瞬间跳变）；控制器 ESTOP/FAULT → 抛 `D1ArmError` 停循环。循环用 `arm.wait_frame()` 跟帧，不用 `sleep` |
| `heading` | `--yaw-deg <度>` 或 `--calibrate` 标定手势 | 修正量 γ（度）：对目标相对接合参考的水平位移与姿态左乘 Rz(γ)。标定手势：按住握把把手向正前方推约 20 cm 再松开，程序算出推程水平偏角 θ 并打印，γ = −θ，本次运行生效（偏角随站位变） |
| `limiter` | 常量 + `--max-vel`（m/s，默认 0.25，即 12.5 mm/帧 @20 Hz）；工作空间盒 x∈[0.05,0.50] y∈[−0.35,0.35] z∈[0,0.55] m（基座系） | 超单帧步长沿方向截到上限；出盒逐轴钳到边界；逆解失败 → 保持上一帧命令并计数，不抛出。启动时实测 TCP 不在盒内 → 拒绝起流 |
| `GripLock.update(j6_cmd, j6_meas)` → `(j6_out, event)` | 纯函数式小类（`griplock.py`），独立可测 | 判夹住（暂定）：目标比实测更闭 > 2° 且实测连续 3 帧每帧变化 < 0.3° → 锁在当时实测角，event=`grasp`；实测停 ≥ 62°（开到头）不判夹住（否则锁在开端无法解锁）；断流恢复后停住计数清零；扳机松回到比锁定角更开 > 3° → 解锁（event=`release`，无振动）；实测 ≤ −28° 且未锁 → event=`empty`（迟滞 +1° 清除，每段只发一次）；闭合方向限速 100°/s（= 控制器帧步上限），`--grip-vel` 可改（开方向不限，控制器另有 100°/s 帧步限） |
| `haptics.pulse(side, pattern)` | pattern：`grasp`（单次 300 ms）/ `empty`（两短 150 ms×2，间隔 0.25 s）；`haptics/haptics_server.py` | 发往 PICO 浏览器里的振动页面（独立页 `haptics/index.html`，或接入振动脚本的 TeleRobot VR 页，见已知限制 1）；**服务端丢弃超过 0.5 s 的旧命令**（进 VR 前积压的命令不再一次全振） |

**A/B 一键回位**（`goto.py`）：只在**脱开**（握把松开）时响应，按住 0.5 s 触发一次，手柄短振确认。

| 键 | 动作 | 规则 |
|---|---|---|
| A | 回准备位 `0,-40,60,0,30,0`（URDF 角，度） | 关节空间点到点，默认 65°/s（`--goto-vel`）；夹爪保持当前命令角（夹着东西不松手） |
| B | 回折叠位（同 `park()` 的目标与顺序：腕部先回，再收大小臂），**到位后保持、不失力** | 夹爪开口 < 65 mm 拒绝（两短振）；下电收臂仍用命令行 `python3 -m d1t_sdk park` |

走的过程中按握把、再按 A/B 或断流 → 立即停在当前位置；之后须先松握把才能接合。
到位后重新接合以新位置为参考（不跳）。在折叠位（各关节都在折叠角 ±10° 内）或工作空间盒外接合会被拒绝（两短振），先按 A。

调试开关 `--fake-grasp-at <度>`：把该角度以下的 J6 实测钳在该角度（模拟夹到物体），
用于仿真里验证锁定与振动通路（仿真实测恒等于目标，夹住判定永不触发）。

## 2. 怎么跑

**日常用一条命令**（上位机，tmux 会话 `teleop`；先检查网络 / 证书前提 / 端口 / 控制器，有问题直接告诉你改哪）：

```bash
bash deploy/teleop_up.sh sim --viz      # 仿真：自起仿真控制器 + TeleRobot + 遥操作 + 画面
bash deploy/teleop_up.sh real           # 真臂：臂上控制器须先起；遥操作用实机基线参数
bash deploy/teleop_up.sh stop           # 全部停止（先停遥操作，臂 HOLD 在原地）
bash deploy/teleop_up.sh check real     # 只检查不启动
```

脚本不发运动命令：起来后臂在哪就停在哪，在折叠位时按 A 回准备位再接合。看各进程输出：
`tmux attach -t teleop`（Ctrl-b 加数字切窗口，Ctrl-b d 退出查看）。下面是脚本内部做的分步命令，排查时用。


环境（上位机，一次）：`~/miniforge3/bin/conda` 环境 `d1t-teleop`（numpy/scipy/pyzmq/
cyclonedds 0.10.2），TeleRobot 目录 `~/TeleRobot`（见 `tools/pico_probe/README.md` §1/§4）。

**仿真（上位机，无头显先跑假发布器；戴头显换 TeleRobot 查看器）**

```bash
# 终端 1：仿真控制器（先确认没有不带 --sim 的 d1t_arm_ctrl 在跑：pgrep -af d1t_arm_ctrl）
bash deploy/sim_start.sh
# 终端 2（画面，可选）：source deploy/env.sh sim '' '' viz && __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia DISPLAY=:0 ~/.venvs/mujoco-3.14.0/bin/python tools/arm_viz/arm_viz.py
# 终端 3：
source ~/miniforge3/bin/activate d1t-teleop && cd ~/AI_drive_arm
source deploy/env.sh sim
# 无头显全链路自测（假发布器与查看器共用 55556，二者别同时起）：
python3 tools/pico_probe/fake_xr_pub.py --endpoint tcp://127.0.0.1:55557 &
python3 tools/pico_teleop/pico_teleop.py --endpoint tcp://127.0.0.1:55557
# 戴头显：TeleRobot 查看器按 tools/pico_probe/README.md §2–3 起，然后
python3 tools/pico_teleop/pico_teleop.py                # 默认 tcp://127.0.0.1:55556 右手柄
python3 tools/pico_teleop/pico_teleop.py --calibrate    # 先标定朝向，再进遥操作
```

**真臂（操作者在场，物理急停在手边）**：同一条命令换 `source deploy/env.sh real enp5s0`，
并加 `--real-ok`（程序默认拒绝非 sim 环境，防误发）。PICO 的 Wi-Fi 与臂有线网同为
192.168.123.0/24 时，先加主机路由（上位机重启后失效）：

```bash
sudo ip route add <PICO IP>/32 dev wlo1 src <wlo1 IP>
```

振动服务（仿真与真臂都由本程序自带，`--haptics-port 8000` 默认开；PICO 浏览器开
`https://<上位机 wlo1 IP>:8000` 并信任证书——需 `--haptics-ip <wlo1 IP>` 才生成证书并监听局域网）。
遥操作时的振动要由 TeleRobot 的 VR 页执行，需要在 TeleRobot 里接入振动脚本（本仓库不含，见已知限制 1）。
命令行参数：`--side`、`--yaw-deg`、`--calibrate`、`--max-vel`、`--max-rot-vel`（姿态转速上限，默认 60°/s）、`--rot-filter-hz`（手柄姿态低通，默认 3 Hz，0 = 关）、`--grip-vel`、`--goto-vel`、`--fake-grasp-at`、
`--endpoint`、`--haptics-port/--haptics-ip`、`--duration`（秒，0=Ctrl-C 退）、`--log`。

## 3. 离线测试

```bash
source ~/miniforge3/bin/activate d1t-teleop && cd ~/AI_drive_arm
python3 tools/pico_teleop/test_griplock.py    # 纯函数，喂合成 j6 序列
python3 tools/pico_teleop/test_teleop_loop.py # FakeCtrl 假臂（sdk/tests），无 DDS 之外的依赖
python3 tools/pico_teleop/test_goto.py        # A/B 一键回位：按住判定、回准备位/折叠位、中途停
```

## 4. 已知限制

1. 遥操作时的手柄振动需要改 TeleRobot（本仓库不含这部分改动）：PICO 浏览器同一时间只允许一个
   沉浸式页面，振动脚本须挂进 TeleRobot 的 VR 页（包一层 `navigator.xr.requestSession` 拿到会话），
   再跨端口从本程序的振动服务（8000，`haptics_server.py`，已放行跨源）拉命令并调手柄 `pulse`。
   不接入时遥操作照常可用，只是手柄不振，振动事件看终端打印。
   前提：PICO 浏览器信任过 `https://<IP>:8000` 的证书（证书存在 上位机 `~/.cache/d1t_haptics/` 复用，
   同一 IP 信任一次即可，换 IP 才重签）。命令过期只由服务端判（PICO 时钟与 上位机 可差 1 s 以上）。
   不振时看 `~/TeleRobot/runtime/logs/viewer.log` 里含 `d1t_haptics` 的行：没有 = 页面里没有这个
   脚本或没挂上 XR 会话；`cmd_error` = 页面拉不到 8000（证书没信任 / 遥操作没起）；
   `cmd_ok` 但没振 = 看当次 `pico-haptics-*.jsonl` 里有没有 `pulse` 行。
2. Wi-Fi 偶发不到 1 s 的断流会触发停流（臂看门狗 0.3 s）：按两份验收清单第 7 项的恢复方式
   处理（先松握把再接合），不做自动重连续跑。
3. 退出（Ctrl-C）后臂 HOLD 在当前姿态；若正夹着物体，J6 保持锁角继续出力（控制器内置
   P=0 自保护 + 45 ℃ 降出力/50 ℃ 释放）。松开夹爪用 `python3 -m d1t_sdk gripper 70`。
4. 工作空间盒、步长上限、GripLock 各阈值均为暂定值，真臂首跑按实机验收清单从低速开始调。
5. 朝向标定的推程方向取「松开握把前最后一帧目标 − 接合首帧目标」；推程不足 10 cm 报错重试。
