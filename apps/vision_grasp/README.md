# vision_grasp — 手眼标定与视觉抓取

腕部 RealSense D435i（眼在手上）+ D1-T 机械臂的视觉抓取应用：
拖动示教采点 → 手眼标定 → YOLOE 开放词表检测与分割 → 点云几何定位 → 竖直下抓。
驱动层为本仓库的 `d1t_sdk`（DDS 直连臂载控制器 `d1t_arm_ctrl`），**需要控制器与 SDK ≥ 0.1.5**（夹爪碰到即锁）。

## 功能

- **拖动示教**：整臂失力拖到位、锁住后自动记录关节角，点位直接写回配置。
- **手眼标定**：单 ArUco 标记或 ChArUco 棋盘两种标定板；自动采集、质量评估（RMS、留出验证）、合格才保存；支持离线重算。
- **视觉检测**：yoloe 开放词表 + 实例分割，类别用自由文本配置（如 `"water bottle"`），无需训练。
- **视觉抓取**（`grasp.approach`，默认 `cloud_top_down`）：
  - `cloud_top_down`：掩码内的深度点转成基座系点云，求物体中心与开合方向，竖直下抓。适合桌面上躺放的瓶子、香蕉、椭球等；
  - `top_down`：竖直向下进给，夹物体水平短边（掩码最小外接矩形）；
  - `camera_ray`：沿相机视线进给，适合立放物体。
- **桌面自校正**：用物体周围一圈桌面拟合平面，反推相机俯仰、横滚和竖直方向的误差并修正（`cloud_top_down`）。
- **流式连续轨迹**：去程「准备位 → 预抓取点（只减速不停）→ 竖直直线到抓取点」、回程「抓取点 → 预抓取点（不停）→ 准备位」各一条，按控制器帧逐帧下发；夹爪预合拢与去程同步进行（夹爪限速 20 °/s，行程大时会延续到下降段，开口始终不小于「爪宽 + 余量」）。
- **闭爪**：夹爪往「爪宽 − 余量」合，碰到物体由控制器锁住并夹紧；没碰到（合到下限附近）判为抓空。
- **连续测试**：夹住后按 `r` 在准备位松开，再按 `g` 抓下一个；抓空不退出。
- **中文画面**：状态横幅、按键提示、当前目标与图例；没有中文字体时自动退回英文。
- **计算设备自动选择**：有 NVIDIA GPU 用 `cuda:0`，否则用 CPU，无需改配置。

## cloud_top_down 怎么算

```
掩码向内腐蚀 → 掩码内深度点反投影 → 基座系点云
掩码外一圈环带的点 → 拟合桌面平面（剔除离群点；不可用时用本次运行的缓存平面或 support_z_m）
桌面自校正：把拟合平面转平并移到 support_z_m 高度，相机位姿随之修正，点云重算
物体高度 = 点云高出桌面部分的 95% 分位；中心高度 = 桌面 + 高度 / 2
中心水平位置 = 过图像中心的视线与「中心高度平面」的交点，再沿物体长轴移到两端视线交点的中点（修透视）
开合方向 = 水平面内垂直于长轴；爪宽 = 中段点到长轴中心线距离的中位数 × 2
```

抓取时 TCP 竖直朝下，开合方向垂直于物体长轴；指尖最低不低于桌面以上 5 mm。

## 硬件

| 部件 | 型号 / 要求 | 没有时 |
|---|---|---|
| 机械臂 | Unitree D1-T，舵机固件 V330，臂上运行本仓库控制器 **≥ 0.1.5**（见仓库根 README 与 `deploy/实机部署.md`） | 用 `--sim` 仿真控制器跑测试与运动学 dry-run（见「仿真」） |
| 相机 | Intel RealSense D435i，刚性固定在末端，USB 3 接上位机 | — |
| 标定板 | ChArUco 5×5（格边 30 mm、标记边 22 mm，`DICT_4X4_50`），或单个 ArUco 标记（`DICT_4X4_50`，id 0） | 两种任选其一；打印后实测尺寸填入配置 |
| 上位机 | Ubuntu 22.04，x86_64，Python 3.10–3.11 | — |
| GPU | NVIDIA 显卡（可选） | CPU 推理，配置不用改 |
| 工作面 | 不透明桌面，与机械臂底座同一平面（桌面自校正的前提） | 不同高时填 `support_z_m`；透明面见「已知限制」 |

## 安装

```bash
pip3 install --user -r apps/vision_grasp/requirements.txt
pip3 install --user pyrealsense2
```

torch / ultralytics 按本机二选一：

```bash
# CPU
pip3 install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip3 install --user ultralytics

# NVIDIA GPU（RTX 50 系须 torch ≥ 2.7.1+cu128）
pip3 install --user torch==2.7.1+cu128 torchvision==0.22.1+cu128 \
  --index-url https://download.pytorch.org/whl/cu128
pip3 install --user ultralytics
```

系统 OpenCV 不是 4.7.0.72 时建独立环境，之后命令中的 `python3` 换成 `~/vg-venv/bin/python`：

```bash
python3 -m venv --system-site-packages ~/vg-venv
~/vg-venv/bin/pip install -r apps/vision_grasp/requirements.txt pytest
```

中文画面需要中文字体（Pillow 随 ultralytics 安装）；不装则画面显示英文：

```bash
sudo apt install fonts-noto-cjk
```

模型权重 `yoloe-26s-seg.pt` 放到 `models/`，下载方式见 [`models/README.md`](models/README.md)。
开放词表另需文本编码器 `mobileclip2_b.ts`（约 240 MB），放在 `apps/` 下；缺失时首次运行自动下载。

## 仿真（无机械臂）

```bash
bash deploy/arm_install.sh           # 编译控制器
source deploy/env.sh sim
bash deploy/sim_start.sh             # 终端 1：仿真控制器
# 终端 2（同样先 source deploy/env.sh sim）
python3 -m pytest apps/vision_grasp/tests -q
python3 apps/vision_grasp/scripts/fk_compare.py
python3 apps/vision_grasp/scripts/dryrun.py
```

通过标准：pytest 全部通过；`fk_compare` 4 项 PASS；`dryrun` 末尾输出 `VERDICT: pose list passes kinematic checks`。

## 实机运行

每个终端先执行（`<网卡>` 为上位机接臂的网卡名）：

```bash
source deploy/env.sh real <网卡>
python3 -m d1t_sdk state             # mode 为 IDLE/HOLD、online 七个 True
```

同一台电脑同时开两个 SDK 程序（例如抓取程序 + 另一个终端看状态）时，第二个终端用 `record` 角色：
`source deploy/env.sh real <网卡> '' record`，否则两个进程抢同一个 DDS 端口。

下文 `CFG=apps/vision_grasp/config/charuco.yaml`；用单 ArUco 标记时改为 `apps/vision_grasp/config/default.yaml`。

| 步骤 | 命令 | 完成标志 |
|---|---|---|
| 1 取景 | `python3 apps/vision_grasp/scripts/live_view.py` | 画面稳定，标定板完整可见 |
| 2 示教点位（可选，配置已自带 32 个） | `python3 apps/vision_grasp/scripts/teach_poses.py --config $CFG` | 记满点数自动结束并写回配置 |
| 3 点位彩排 | `python3 apps/vision_grasp/scripts/pose_test.py --config $CFG` | 全部点位走完无故障 |
| 4 手眼标定 | `python3 apps/vision_grasp/handeye.py --config $CFG` | 输出 `qualified` 并保存 `hand_eye*.npz` |
| 5 检测核对 | `python3 apps/vision_grasp/yolo_detect.py --config $CFG` | 物体基座系坐标与卷尺实测相差 1–2 cm 内 |
| 6 抓取预演 | 先 `python3 -m d1t_sdk joint 0,-40,40,0,40,0` 把臂摆到准备位，再 `python3 apps/vision_grasp/grasp_yolo.py --config $CFG --dry-run` | 预演**不动臂**；稳定选中目标，日志给出物体中心、预抓取位姿、闭口值与规划时长 |
| 7 抓取 | `python3 apps/vision_grasp/grasp_yolo.py --config $CFG` | 启动时自动开爪、走到准备位；按 `g` 抓取 |

示教按键：回车依次为「走准备位 → 失力 → 锁住并 2 s 后记录 → 失力 …」，`u` 删上一个点，`q` 结束。**失力期间整臂无力矩，须托住小臂。**

### 抓取按键

| 键 | 作用 |
|---|---|
| `g` | 抓取当前目标（画面底部「当前目标」那一个）；夹着物体时不响应 |
| `r` | 夹着物体时：在准备位张开夹爪，之后可以再按 `g` |
| `q` / `Esc` | 张开夹爪、回安全位并退出；抓取动作执行中按下时，等这一趟回到准备位再执行 |

### 画面

- 顶部横幅：状态（实时识别 / 预热中 / 夹持中 / 抓空）与按键提示。
- 物体框：绿框 = 有分割掩码；橙框 = 无掩码（退回轴对齐框）或无有效抓取。白线 = 开合方向（示意），红点 = 抓取中心。
- 底部：当前目标的名称、置信度、爪宽。
- 抓取执行期间画面停在按键那一帧（窗口保持响应）。

### 日志里看什么

| 日志 | 含义 |
|---|---|
| `cloud_top_down: correction tilt X deg, dz Y mm; plane offset … before -> … after` | 桌面自校正修正量；修正后应接近 0 mm / 0° |
| `cloud_top_down <名>: center (x,y,z) m, height …, table_z … (plane, ring kept N%), … jaw … cm, close … mm, …` | 物体中心、高度、桌面来源、爪宽与闭口值 |
| `Applied base_offset_m [...]` | 固定补偿已生效（补偿全 0 时不出现） |
| `stream plan: approach … pregrasp point passed at … with the TCP at … m/s …` | 去程 / 回程规划时长与预抓取点经过速度 |
| `j6 preclose: …` | 预合拢的开始、结束时刻（相对去程起点） |
| `timing: approach …, grip …, retreat …, total …` | 本次抓取的实际分段时长 |
| `MISS: …` | 抓空（夹爪合到下限附近），已张开，可直接再按 `g` |

每步的详细说明与中止方式见 [`实机验收清单.md`](实机验收清单.md)，调参依据见 [`调参说明.md`](调参说明.md)。

## 主要配置

`grasp` 段（`config/*.yaml`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `classes` | 握力球、瓶子、香蕉等 | 检测提示词，自由文本，不能为空 |
| `device` | `auto` | `auto` / `cpu` / `cuda:0` |
| `approach` | `cloud_top_down` | `cloud_top_down` / `top_down` / `camera_ray`，见「功能」 |
| `final_motion` | `linear` | `linear` = 流式连续轨迹；`joint` = 分段走、每段到位停稳 |
| `support_z_m` | `0.0` | 物体所在平面在基座系的高度（米），桌面与底座同高为 0；桌面自校正把桌面对齐到这个高度 |
| `table_self_correct` | `true` | 桌面自校正开关；物体所在面与底座不同高、又没填 `support_z_m` 时关掉 |
| `table_self_correct_max_deg` | `10` | 桌面倾角超过它视为平面不可信，不修正 |
| `grasp_offset_m` | `0.040` | TCP（两指根部）停在抓取点上方的距离；指尖长 50 mm，0.040 = 指尖越过物体中心约 1 cm。抓深调大，抓浅调小 |
| `base_offset_m` | `[0, 0, 0]` | 抓取点固定补偿（基座系 x 前 / y 左 / z 上，米），见「精度调整」 |
| `pregrasp_offset_m` | `0.05` | 预抓取点在抓取点上方的距离 |
| `gripper_open_mm` | `65` | 张口宽度（4–70 mm） |
| `gripper_preclose_margin_mm` | `20` | 去程中预合拢到「爪宽 + 这个值」（每边约 10 mm）；0 = 不预合拢 |
| `gripper_close_margin_mm` | `8` | 闭口上限 = 爪宽 − 这个值，夹爪最多合到这里 |
| `gripper_close_min_mm` | `15` | 闭口下限；夹爪停在下限 + 2 mm 以内判为抓空 |
| `linear_vel_mps` / `linear_acc_mps2` | `0.10` / `0.50` | 竖直直线段的 TCP 速度 / 加速度上限；经过预抓取点的速度约等于它 |
| `grasp_vel_dps` | `70` | 抓取段关节速度上限（°/s） |
| `wait_before_grip_s` | `0` | 到抓取点后等多久再闭爪 |
| `min_base_z_m` | `0.02` | 预抓取点与抓取点 TCP 高度下限（米），低于它不执行 |

`robot` 段相关：采集关节速度 `vel_dps` 90 °/s，回准备位 / 安全位 `approach_vel_dps` 80 °/s，关节加速度 `acc_dps2` 200 °/s²，
到位容差 `arrival_tolerance_deg` 3°（含夹物时腕部下沉），稳定窗 `settle_s` 0.3 s。控制器各轴上限 100 °/s。

## 精度调整

1. **标定核对**：`yolo_detect.py` 读出的基座系坐标与卷尺实测相差超过 1 cm 时重新标定——确认标定板尺寸为实测值、板面平整、相机固定牢靠。
2. **桌面自校正**：看日志 `correction … before -> after`，修正后平面偏差应接近 0 mm / 0°。修正前读数就是相机链的误差；超过 10° 不修正并打出原因。
3. **高度**：物体不在底座所在平面上时填 `support_z_m`；夹取深浅用 `grasp_offset_m` 调，每次 0.005。
4. **固定偏差**：自校正修不了水平方向的固定偏差（绕竖直轴的转动、水平平移）。同一位置抓取 3 次，比较日志 `Grasp …: base xyz` 与实测物体中点；偏差稳定时把均值填入 `base_offset_m`，日志出现 `Applied base_offset_m` 即生效。该值与相机安装有关，每台设备需各自测定。
5. **夹不住 / 压物体**：看日志 `jaw` 与物体真实宽度。爪宽估大超过 `gripper_close_margin_mm` 时夹爪碰不到物体，调大该值；预合拢后指尖压到物体顶上时调大 `gripper_preclose_margin_mm` 或填 0。

更换夹爪、相机支架或相机安装位置后须重新标定。

## 已知限制

| # | 限制 | 说明 |
|---|---|---|
| 1 | 透明物体、透明支架 | 深度相机会穿透：物高、爪宽测偏，桌面拟合到下面的面。透明支架表面需贴不透明面，且支撑面高度要填 `support_z_m` |
| 2 | 桌面自校正的前提 | 物体所在面 = `support_z_m` 高度的水平面。物体放在垫高的支架、波浪槽等非平面上时会算偏 |
| 3 | 爪宽估大 | 估大超过 `gripper_close_margin_mm`（默认 8 mm）时夹爪合到上限也碰不到物体，夹不住 |
| 4 | 抓空判据 | 只有夹爪合到下限附近才判抓空；爪宽较大的物体没夹到时，夹爪停在闭口上限，会显示「夹持中」 |
| 5 | 长时间夹持升温 | 控制器夹紧量约 150 mA，夹持约 3–7 分钟到 45 ℃ 后降出力、夹持变松；夹住后尽早按 `r` |
| 6 | 可达范围 | 竖直下抓时前向约 x ≤ 0.43 m；超出时日志报 `IK rejected` / `No feasible grasp for this frame`，不执行 |
| 7 | 画面边缘 | 物体靠画面边缘时深度误差放大，环带也可能部分落到桌外；尽量放在画面中部 |
| 8 | 腕部保持抖动 | 加速度 200 °/s² 下偶有控制器「重发保持指令」提示（腕部小幅自激），未触发故障 |
| 9 | 闭爪时长 | 夹爪限速 20 °/s，闭爪段 1–4 s 不等，是单次抓取中最长的一段 |

## 目录

```text
apps/vision_grasp/
├── handeye.py          # 手眼标定（--check-config / --solve / 实机采集）
├── yolo_detect.py      # 检测预览
├── grasp_yolo.py       # 视觉抓取（g 抓取 / r 松开 / q 退出）
├── config/             # default.yaml（单 ArUco）、charuco.yaml（ChArUco）；标定结果与采集记录也存于此
├── models/             # YOLO 权重（另行下载）
├── scripts/            # live_view / teach_poses / pose_test / dryrun / fk_compare
├── utils/              # 几何、相机、机械臂适配层（含流式下发）、设备选择
├── tests/              # 无硬件测试
├── 实机验收清单.md
└── 调参说明.md
```

## 致谢与许可

标定与抓取流程移植自 [d1arm_sdk](https://gitee.com/gccodespace/d1arm_sdk)（作者 bxd，分支 `grasp_back`），
经作者授权以 MIT 许可发布，署名见 [LICENSE](LICENSE)。参考了 `reBot-DevArm-Grasp` 的流程与几何约定、
OpenCV `calibrateHandEye`。模型权重版权归 Ultralytics。
