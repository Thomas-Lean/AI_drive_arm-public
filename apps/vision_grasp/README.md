# vision_grasp — 手眼标定 + YOLO 视觉抓取（眼在手上）

RealSense D435i 刚性固定在 D1-T 臂末端（eye-in-hand），工作台贴单个 ArUco 标记。
本目录提供完整闭环：**手眼标定**（采集 / 质量门 / 求解 / 离线重算）、
**工具链**（取景 / dry-run / 运动彩排）、**YOLO 视觉抓取**（检测预览 +
最小包围盒短轴抓取）。手眼结果 `T_tcp_camera`（相机彩色光学系 → TCP）由
`handeye.py` 产出，`grasp_yolo.py` 消费，单位均为米。

> **来源**：本目录移植自 D1 臂社区开源项目
> [d1arm_sdk](https://gitee.com/gccodespace/d1arm_sdk)（作者 bxd）的
> `calibration/` 目录，分支 `grasp_back`、提交 `f95f6c9`，经作者授权以 MIT
> 随本仓库发布（署名见 [LICENSE](LICENSE)）。驱动臂的一层由原 d1armd
> （TCP JSON-lines 服务）换成本项目的 `d1t_sdk.D1Arm`（DDS 直连臂载控制器
> `d1t_arm_ctrl`），只改 `utils/robot.py` 适配层，上层脚本不动。
> 接口逐项对照见下文「与 d1t_sdk 的接口对照」。

## 现状：做到了什么、做不到什么

**一句话**：在舵机固件 V330 的 D1-T 上，用腕部 RealSense D435i 完成了手眼标定和
YOLO 视觉抓取闭环——躺放在桌面上的矿泉水瓶能被识别、竖直下抓、夹起、抬起并放回；
视觉抓取全程控制器与舵机总线没有掉线。

| 环节 | 状态 | 说明 |
|---|---|---|
| 拖动示教采点位 | ✅ 实机跑通 | `scripts/teach_poses.py`：失力拖动 → 锁住 → 2 s 后记录，写回配置 |
| 手眼标定（ArUco / ChArUco） | ✅ 实机跑通 | 32 点位两种板都合格；以 ChArUco 为准，尺量核对一致（限制 #13） |
| 检测预览 | ✅ 实机跑通 | yoloe 开放词表，GPU 约 7 ms/帧；无 N 卡自动用 CPU |
| 躺放瓶子竖直下抓 | ✅ 实机跑通 | 识别 → 预抓取 → 直线下降 → 闭爪锁停住角 → 抬起 → 回准备位 |
| 透明瓶 / 透明托盘的高度 | ⚠️ 兜底 | 深度相机读不准透明物体，高度改用「支撑面 + 半爪宽」兜底，**深度基本不参与** |
| 抓取水平位置 | ⚠️ 有厘米级偏差 | 偏差来自手眼标定与支撑面高度；按下文「复刻时抓偏怎么调」修正 |
| 立放瓶子 | ❌ 不支持竖直下抓 | 手掌先压到瓶盖；改 `approach: camera_ray` |
| 多目标排序 / 分拣放置 | ❌ 未做 | 每次按 `g` 抓当前选中的一个 |

**方案边界**：本目录是「2D 检测框 + 深度分位数」的几何法——位置靠深度反投影，朝向靠
检测区域的短边。它对不透明、形状规则、躺放的物体够用；透明物体的深度读数不可信、
不规则物体的最佳夹持位无法从短边推出，这两类需要点云级的抓取网络（如 GraspNet 一类），
但它们同样依赖深度，透明物体仍需额外处理。

## 硬件与配置不通用，复刻前先核对

本目录的参数是按下面这套实物调出来的，换任何一项都要重新标定或改配置：

| 项 | 本仓库实物 | 换了要动什么 |
|---|---|---|
| 机械臂 | D1-T，舵机固件 V330，本仓库控制器 `d1t_arm_ctrl` | 原厂控制器不能用（接口不同）；固件见仓库主 README |
| 相机 | RealSense D435i，刚性固定在末端（眼在手上） | 相机支架或安装位置一动，**必须重新手眼标定** |
| 夹爪 TCP | SDK 的 TCP 在两指根部，指尖再伸 50 mm | 换夹爪 / 指长不同 → 改 `grasp_offset_m`，并重新标定 |
| 工作面 | 桌面与底座同高（基座系 z = 0） | 底座垫高或物体放在台子上 → 改 `support_z_m`（支撑面在基座系的高度，米） |
| 标定板 | ChArUco 5×5，格边 30 mm、标记边 22 mm（`config/charuco.yaml`） | 打印尺寸必须实测后写进配置，比例错了平移全错 |
| 点位 | 32 个采集点位按本臂 + 本支架示教 | 支架不同可能看不到板 → 用 `teach_poses.py` 重新示教 |

## 复刻时抓偏怎么调

按顺序做，前一步没过不做下一步：

1. **先确认标定本身对**：`yolo_detect.py --config config/charuco.yaml` 看标定板角或一个
   不透明小物体的基座系坐标，用卷尺量实物到底座中心的前向 x、左向 y。差 < 1 cm 算过；
   超了先重新标定（检查板尺寸、点位覆盖三个转角方向、相机是否松动）。
2. **再定高度**：量支撑面相对底座安装面的高度，填 `support_z_m`（米）。抓深压桌面 →
   `grasp_offset_m` 调大；抓浅夹不住 → 调小（每次 0.005）。
3. **最后补固定水平偏差**：同一位置抓 3 次，日志 `Grasp …: base xyz` 给出程序位置，
   卷尺量物体实际中点，算 `实际 − 程序`。**三次偏差方向和大小一致**（相差 < 5 mm），
   把平均值填进 `base_offset_m: [dx, dy, dz]`（米，x 前 / y 左 / z 上）；
   生效时日志会打印 `Applied base_offset_m`。
4. 偏差忽大忽小、或随物体位置明显变化 → 不要用 `base_offset_m` 硬补，问题在标定，回第 1 步。

## 目录骨架

```text
apps/vision_grasp/
├── handeye.py            # 入口：手眼标定（--check-config / --solve / 真机采集）
├── grasp_yolo.py         # 入口：YOLO 抓取（g 冻结抓取 / q 松开回安全）
├── yolo_detect.py        # 入口：YOLO 检测预览（基座系/相机系坐标，纯检测）
├── config/
│   ├── default.yaml      # 全部配置（标定+抓取，含中文注释）
│   ├── charuco.yaml      # 同上，板型为 ChArUco（产出 hand_eye_charuco.npz）；两板手眼解不一致时以此为准，抓取用 --config config/charuco.yaml
│   ├── hand_eye.npz      # 标定产出（合格才覆盖；抓取端读取；不入库）
│   └── sessions/         # 每次采集留档（图像/关节/样本 npz/attempts.json；不入库）
├── models/               # YOLO 权重目录（不随仓库发布，下载见 models/README.md）
├── utils/                # 辅助模块
│   ├── core.py           #   几何与数学：PnP/手眼求解/质量评估/ArUco 检测器
│   ├── robot.py          #   适配层：RpcArm → d1t_sdk.D1Arm（唯一改造的文件）
│   ├── camera.py         #   RealSense 彩色流封装（内参/畸变/预热）
│   └── paths.py          #   路径解析（配置在 config/，数据落根目录）
├── scripts/              # 工具脚本
│   ├── live_view.py      #   纯取景器（不连机器人）
│   ├── teach_poses.py    #   拖动示教：失力拖动 → 锁住记录，点位直接写回配置 poses_deg
│   ├── dryrun.py         #   点位运动学 dry-run（对 --sim 实例，FK 多样性判定）
│   ├── pose_test.py      #   点位运动彩排（真机逐点停留，输出保持漂移）
│   └── fk_compare.py     #   FK 与 d1arm_sdk 金标值逐项比对（离线可重跑）
├── tests/
│   └── test_handeye.py   # 无硬件测试（合成真值/流程/异常路径/几何用例）
├── 实机验收清单.md        # 实机操作步骤（交人工执行）
└── requirements.txt      # Python 依赖
```

约定：**配置与数据产出统一放 `config/`**（hand_eye.npz / sessions / 示教文件
与配置同目录）；TCP = Link6 法兰 × 内置工具偏移 xyz(-5.62, 0, 70.6) mm、
rpy(0, -90°, 0)（即 URDF 两夹爪指 origin 中点，与 `sdk/arm_model/d1t.json`
一致），不是裸法兰；换装工具或改动 TCP 定义后必须重新标定。

标定板两种可选（`board.kind`）：**单 ArUco**（默认）与 **ChArUco 棋盘**（角点
多、近正视不歧义，对比实验用）。ChArUco 时产出文件名自动带 `_charuco` 后缀，
与单 ArUco 结果互不覆盖；`yolo_detect`/`grasp_yolo` 读哪份由当时的 `board.kind`
决定。同臂同点位两轮对比的流程见实机验收清单 §3b。

## 依赖

- Python 3.10–3.11（`opencv-python==4.7.0.72` 无更高版本 wheel）
- `numpy<2`（OpenCV 4.7.0.72 按 NumPy 1.x ABI 构建）、PyYAML、`cyclonedds==0.10.2`（d1t_sdk 依赖）
- `pyrealsense2`（RealSense 相机，标定 / 检测 / 抓取需要）
- 抓取 / 检测另需 `ultralytics`（含 torch，体积较大）
- 臂侧：本仓库控制器 `d1t_arm_ctrl`（仿真 `--sim` 或实机；见 `deploy/`）

```bash
pip3 install --user -r apps/vision_grasp/requirements.txt
pip3 install --user pyrealsense2   # 相机，标定/检测/抓取需要
```

**torch/ultralytics 分两档，按本机装一种**（配置默认 `grasp.device: auto`：装了 GPU 版且有 N 卡就用 `cuda:0`，否则用 `cpu`，两种都不用改配置；要强制某一种就填 `cpu` / `cuda:0`，或命令行 `--device`）：

```bash
# ① CPU 版（默认，够用；首次推理几十秒/帧属正常）
pip3 install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip3 install --user ultralytics
# 国内可改用镜像直链（例：mirrors.aliyun.com/pytorch-wheels/cpu/ 下与
# 本机 Python/CPU 匹配的 torch-*.whl、torchvision-*.whl）

# ② GPU 版（有 N 卡时装；配置 auto 会自动用上）
#    注意 RTX 50 系（Blackwell，sm_120）必须 torch ≥ 2.7.1+cu128，
#    2.6.x 及更早的 cu124/cu126 轮子不含其内核（报 no kernel image available）
pip3 install --user torch==2.7.1+cu128 torchvision==0.22.1+cu128 \
  --index-url https://download.pytorch.org/whl/cu128
pip3 install --user ultralytics
```

yoloe 注意：`set_classes` 需要 clip 包 + ~240 MB 文本编码器
（`mobileclip2_b.ts`，按 `utils/paths.py` 的约定放 `apps/` 下；没有的话首次
运行会自动下载到进程 CWD）。yoloe 下 `classes` 不能清空，否则零检测。

## 模型权重

默认配置用 `models/yoloe-26s-seg.pt`（开放词表 + 分割），**不随仓库发布**，
下载方式见 [`models/README.md`](models/README.md)。

## 与 d1t_sdk 的接口对照

上层脚本实际用到的机器人调用共 7 组（原 d1armd 语义 → 本项目 `d1t_sdk.D1Arm`），
迁移时逐项核对，结论如下：

| 原接口（d1armd） | 次数 | 本项目对应 | 核对结论 |
|---|---|---|---|
| `state()` | 13 | `D1Arm.state()` | `q_deg` 同为 6 关节 URDF 角（度，J6 夹爪不混入）；`online` 同 7 项；`stamp_ms` 同名（ms）。`mode`/`err` 在适配层映射：IDLE/HOLD→`'IDLE'`，轨迹/流式执行中→`'MOVING'`，ESTOP→`'IDLE'`+`err='estop'`（复刻 d1armd 锁存语义），FAULT→`err`=fault 位串 |
| `move(pose, vel)` + `wait_still()` | 22 | `move_joints(async_=True)` + 轮询 | 到位容差与稳定窗沿用原配置（arrival 1.8° / settle 1.2 s / 波动 0.3°）；速度逐轴钳到控制器限速 100°/s，配置值 30/20°/s 远低于上限 |
| `call('fk')` | 5 | `D1Arm.fk(q_deg)` | **TCP 定义与单位逐项一致**：原 TCP = Link6 法兰 × xyz(-5.62, 0, 70.6) mm、rpy(0, -90°, 0) = 本项目 `sdk/arm_model/d1t.json` 的 tcp（URDF 两夹爪指 origin 中点）；连杆参数同源（d1_description URDF）；`pos` 米、`quat` [w,x,y,z] 同序。数值比对见 `scripts/fk_compare.py`（对照 d1arm_sdk `tests/test_main.cpp` 金标，可离线重跑） |
| `call('ik')` | 5 | `D1Arm.ik(pos, quat, seed)` | 参数同为 pos+quat(w,x,y,z)+seed → 6 URDF 角；无解时适配层转写为 `ik no solution …`（上层按前缀识别后换候选重试）；本项目限位宽于原校验限位，上层 J5 余量检查照常 |
| `call('move_l')` | 1 | `D1Arm.move_l(pos, quat, vel, acc, async_=True)` | 位姿表示一致（quat 优先、米、m/s、m/s²）；上层随后 `wait_still(expected_q)` 轮询确认，适配层照常支持 |
| `call('gripper')` | 3 | `D1Arm.gripper(mm)` | mm 语义同为金属指间距；本项目可达域 4–70 mm（实测标定表）⊇ 原 0–65 mm；J6 限速 ≤20°/s 相同 |
| `call('estop')` | 6 | `D1Arm.estop()` | **差异已处理**：本项目急停锁存至 `reset()`，原为下一条轨迹自动清 → 适配层只自动清**本进程自己发的** estop（下一条运动命令前 reset）；外部 estop 与任何 FAULT 拒绝运动并抛错（消息带 mode/fault 位），确认安全后手动 `python3 -m d1t_sdk reset` |
| `call('damping')` / `call('set_joint')` | 3 | **无对应** | 原工程拖动示教用；本项目改为 `call('free')`（`free_all` 整臂失力）+ `call('lock')`（`lock_all` 锁在当前实测角），见「已知限制」#1 |

## 怎么跑

### 仿真验证（无臂，先做这个）

```bash
bash deploy/setup_host.sh            # 依赖（已装自动跳过）
bash deploy/arm_install.sh           # 编译控制器 + ctest
source deploy/env.sh sim             # 设 CYCLONEDDS_URI / PYTHONPATH
bash deploy/sim_start.sh             # 终端 1：--sim 控制器（lo，前台）
# 另开终端 2（同样 source deploy/env.sh sim）：
python3 -m pytest apps/vision_grasp/tests -q        # 无硬件测试
python3 apps/vision_grasp/scripts/fk_compare.py     # FK 与 d1arm_sdk 金标比对（离线）
python3 apps/vision_grasp/scripts/dryrun.py         # 32 个采集点位 dry-run（对 --sim）
```

系统 OpenCV 不是 4.7.0.72 时（缺 `cv2.aruco.generateImageMarker` 等），建隔离环境：
`python3 -m venv --system-site-packages ~/vg-venv && ~/vg-venv/bin/pip install
-r apps/vision_grasp/requirements.txt pytest`，之后命令里的 `python3` 换
`~/vg-venv/bin/python`。

仿真全过的判据：pytest 全绿；fk_compare 4 项 PASS；dryrun 末尾
`VERDICT: pose list passes kinematic checks`。

### 实机（标定 + 抓取）

按 [`实机验收清单.md`](实机验收清单.md) 顺序：取景（live_view）→ 点位彩排
（pose_test）→ 手眼标定（handeye.py）→ 检查看坐标（yolo_detect.py）→
YOLO 抓取（grasp_yolo.py，先 `--dry-run`）。每步一条命令、一条通过判据、
中止手段都在清单里。

## 已知限制

| # | 限制 | 说明 |
|---|---|---|
| 1 | 拖动示教为整臂失力 | d1t_sdk 没有单轴阻尼，`scripts/teach_poses.py` 用整臂失力拖动、锁住后记录：**失力期间臂会掉，全程托住小臂**。用法：`python3 scripts/teach_poses.py --config config/<配置>.yaml [--n 32]`；回车依次 = 走准备位 → 失力 → 锁住、2 s 后记录 → 失力 …，`u` 删上一个点，`q` 结束；超软限位的点不记；每记一点写回配置 `poses_deg`（启动时备份 `<配置>.bak-时间`）；任何退出都先锁住 |
| 2 | estop 锁存语义差异 | 本项目急停锁存到 `reset()`（原工程下一条轨迹自动清）。适配层**只自动清本进程自己发的** estop；**外部** estop（别的终端/进程敲的）与任何 FAULT 都不自动清——运动命令抛错并带 fault 位，确认安全后手动 `python3 -m d1t_sdk reset`。脚本因 Ctrl-C/故障退出后重跑前，先 `python3 -m d1t_sdk state` 看一眼有无锁存 |
| 3 | FK/IK 绝对精度 ±1 cm 量级 | 零位偏移未精标 + 重力下沉约 1 cm（SDK 不补偿，见仓库主 README 已知限制 #4/#7）。标定与抓取全用实测角与相对变换，不受累积影响，但 `min_base_z_m` 等余量按此留 |
| 4 | J4 保持自激 | J4 保持时会自激抖动（实测 40–94° 多个姿态，窗口范围 0.7–0.8°），本分支所带控制器的停住判据 0.5° 会误报 HOLD_FAIL（`reset()` 可恢复）；下一版控制器把 J4 判据放宽到 2.0°（其余轴不变），发布后合入本分支（仓库主 README #5） |
| 5 | 夹爪 mm 为金属指间距 | 可达域 4–70 mm（实测标定表；垫面间距 = 金属 − 4）；不用原工程 0°↔0 mm 线性公式 |
| 6 | 对端控制器中途退出 | 实测一次：控制器进程消失时 d1t_sdk 发送线程可能因 cyclonedds `InvalidSample` 报错退出（d1t_sdk 既有行为，不改 SDK）；臂由看门狗约 0.3 s 停住，重启脚本即可 |
| 7 | 速度上限 100°/s | 控制器逐轴限速；采集 90°/s、ready/safe 接近 40°/s、抓取关节 30°/s、末段直线 0.08 m/s，均在限内 |
| 8 | 权重与文本编码器不入库 | 30 MB 模型与 ~240 MB yoloe 编码器按需下载（见 `models/README.md` 与「依赖」）；yoloe 下 `classes` 白名单不能清空，否则零检测 |
| 9 | 本期验证范围 | pytest 无硬件测试 + FK 金标比对 + `--sim` dry-run 全过；实机：示教、两种板手眼标定、检测、躺放瓶子竖直下抓夹起已跑通；抓取成功率未做统计 |
| 10 | `robot.host`/`robot.port` 为占位 | 上层配置校验要求非空；DDS 连接实际由 `source deploy/env.sh` 设置的 `CYCLONEDDS_URI` 决定，适配层忽略这两项 |
| 11 | 安全位姿 J2 = 89° | 实测限位 J2 ∈ [-97, 89]°，原工程 safe_pose 的 90° 超上界 1°，配置已改 89° |
| 12 | 闭爪后锁停住角 | 闭口目标（实测爪宽 × `gripper_close_factor`）**只决定闭合到哪为止**：夹住后轮询 J6 开度，稳定（连续 3 帧每帧 <0.3°）即重发实测开度锁在停住角——顶物体保持会堵转发热（实测目标比停住角多闭约 3° 即 >300 mA、82 s 内 J6 40→45 ℃）。空夹（开度 ≤ 闭口下限+2 mm）或 2 s 不稳定自动开爪报错，不保持 |
| 13 | 两种板手眼解不一致，以 ChArUco 为准 | 同臂同 32 点位两轮均 qualified，但单 ArUco 与 ChArUco 的 `T_tcp_camera` 差 4.9 mm / 4.2°（0.3 m 处约 2 cm）；互换解释对方数据误差升到 17–18 mm；尺量板角前向距离与 ChArUco 解一致（差 1–2 cm 内）、与单 ArUco 解差约 2 cm。故检测与抓取一律 `--config config/charuco.yaml`（读 `hand_eye_charuco.npz`）；单 ArUco 偏差原因未查 |
| 14 | 抓取进给方向 `grasp.approach` | `top_down`（配置默认）：TCP 竖直向下进给，爪开合方向取物体在水平面内的短边，检测点取物体区域最近 `top_depth_quantile`（默认 15%）的深度当顶面（透明瓶、透明托盘的红外穿透读数偏远，用中位数会抓深），目标降到顶面以下半个爪宽（躺放瓶子的中线高度），且不低于 `support_z_m`（支撑面高度，桌面 = 0）+ 半个爪宽（深度读低时兜底；实测透明瓶与不透明小瓶都触发过，高度偏低原因未查）；水平位置取物体长轴两端视线与中线高度水平面交点的中点（顺视线躺放时二维中点受透视偏向近端，实测会夹到瓶底），预抓取点在正上方 `pregrasp_offset_m`；仿真 IK 在 x 0.22–0.38 m、y ±0.10 m、任意朝向全部可达（J4 38–63°，无需倾斜）。**只适用躺放物体**：立着的瓶子斜看时短边是水平的，不会被拒绝，但竖直下抓够不到中线（手掌先压到瓶盖）——立放物体改用 `camera_ray`（沿相机视线进给，原工程行为） |
| 15 | TCP 在指根，不在指尖 | SDK 的 TCP 取两指关节原点中点（`sdk/arm_model/d1t.json`），指尖沿进给方向再伸出约 5 cm（实物尺量 50 mm，URDF 指模型 5.5 cm）。手眼与检测坐标不受影响（标定相对同一 TCP，互相抵消），但抓取时 TCP 去哪指尖就再深 5.5 cm——`grasp_offset_m` 因此取 0.040（指尖越过抓取点约 1 cm）；不改 SDK 的 TCP，免得牵动 FK、手眼结果与其他调用方 |

## 参考与验证

- 上游 [d1arm_sdk](https://gitee.com/gccodespace/d1arm_sdk)（分支 `grasp_back`、
  提交 `f95f6c9`）：本目录的直接来源，含 31 项无硬件测试与真机整定记录
  （迁移后为 39 项，新增 4 项适配层语义测试、替换 2 项传输层测试）。
- 上游参考工程 `reBot-DevArm-Grasp`（采集/抓取流程与几何约定的原始参考）。
- OpenCV `calibrateHandEye`（PARK/TSAI/HORAUD）；ArUco IPPE_SQUARE + 双解歧义拒收。

