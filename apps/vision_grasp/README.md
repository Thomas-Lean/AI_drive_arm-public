# vision_grasp — 手眼标定与 YOLO 视觉抓取

腕部 RealSense D435i（眼在手上）+ D1-T 机械臂的视觉抓取应用：
拖动示教采点 → 手眼标定 → YOLO 开放词表检测 → 竖直下抓夹取。
驱动层为本仓库的 `d1t_sdk`（DDS 直连臂载控制器 `d1t_arm_ctrl`）。

## 功能

- **拖动示教**：整臂失力拖到位、锁住后自动记录关节角，点位直接写回配置。
- **手眼标定**：单 ArUco 标记或 ChArUco 棋盘两种标定板；自动采集、质量评估（RMS、留出验证）、合格才保存；支持离线重算。
- **视觉检测**：yoloe 开放词表，类别用自由文本配置（如 `"water bottle"`），无需训练；实时显示相机系与基座系坐标。
- **视觉抓取**：
  - `top_down`：竖直向下进给，夹物体水平短边，适合桌面躺放的瓶子、盒子；
  - `camera_ray`：沿相机视线进给，适合立放物体；
  - 预抓取 → 直线接近 → 闭爪锁停住角 → 原路退出，全程 IK 可达性与关节余量预检。
- **计算设备自动选择**：有 NVIDIA GPU 用 `cuda:0`，否则用 CPU，无需改配置。

## 硬件

| 部件 | 型号 / 要求 | 没有时 |
|---|---|---|
| 机械臂 | Unitree D1-T，舵机固件 V330，臂上运行本仓库控制器（见仓库根 README 与 `deploy/实机部署.md`） | 用 `--sim` 仿真控制器跑测试与运动学 dry-run（见「仿真」） |
| 相机 | Intel RealSense D435i，刚性固定在末端，USB 3 接上位机 | — |
| 标定板 | ChArUco 5×5（格边 30 mm、标记边 22 mm，`DICT_4X4_50`），或单个 ArUco 标记（`DICT_4X4_50`，id 0） | 两种任选其一；打印后实测尺寸填入配置 |
| 上位机 | Ubuntu 22.04，x86_64，Python 3.10–3.11 | — |
| GPU | NVIDIA 显卡（可选） | CPU 推理，配置不用改 |

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

下文 `CFG=apps/vision_grasp/config/charuco.yaml`；用单 ArUco 标记时改为 `apps/vision_grasp/config/default.yaml`。

| 步骤 | 命令 | 完成标志 |
|---|---|---|
| 1 取景 | `python3 apps/vision_grasp/scripts/live_view.py` | 画面稳定，标定板完整可见 |
| 2 示教点位（可选，配置已自带 32 个） | `python3 apps/vision_grasp/scripts/teach_poses.py --config $CFG` | 记满点数自动结束并写回配置 |
| 3 点位彩排 | `python3 apps/vision_grasp/scripts/pose_test.py --config $CFG` | 全部点位走完无故障 |
| 4 手眼标定 | `python3 apps/vision_grasp/handeye.py --config $CFG` | 输出 `qualified` 并保存 `hand_eye*.npz` |
| 5 检测核对 | `python3 apps/vision_grasp/yolo_detect.py --config $CFG` | 物体基座系坐标与卷尺实测相差 1–2 cm 内 |
| 6 抓取预演 | `python3 apps/vision_grasp/grasp_yolo.py --config $CFG --dry-run` | 稳定选中目标，日志给出预抓取位姿与闭口值 |
| 7 抓取 | `python3 apps/vision_grasp/grasp_yolo.py --config $CFG` | 预览窗按 `g` 抓取，`q` 松爪回安全位退出 |

示教按键：回车依次为「走准备位 → 失力 → 锁住并 2 s 后记录 → 失力 …」，`u` 删上一个点，`q` 结束。**失力期间整臂无力矩，须托住小臂。**

每步的详细说明与中止方式见 [`实机验收清单.md`](实机验收清单.md)。

## 主要配置（`config/*.yaml` 的 `grasp` 段）

| 参数 | 默认 | 说明 |
|---|---|---|
| `classes` | 瓶子、握力球 | 检测提示词，自由文本，不能为空 |
| `device` | `auto` | `auto` / `cpu` / `cuda:0` |
| `approach` | `top_down` | `top_down` 躺放物体；`camera_ray` 立放物体 |
| `support_z_m` | `0.0` | 物体所在平面在基座系的高度（米），桌面与底座同高为 0 |
| `grasp_offset_m` | `0.040` | TCP 停在抓取点外的距离；抓深调大，抓浅调小 |
| `base_offset_m` | `[0, 0, 0]` | 抓取点固定补偿（基座系 x 前 / y 左 / z 上，米） |
| `pregrasp_offset_m` | `0.05` | 预抓取点后退距离 |
| `gripper_open_mm` | `65` | 张口宽度（4–70 mm） |
| `min_base_z_m` | `0.02` | 目标高度下限，防止触碰工作面 |

速度：采集 90 °/s，准备位 / 安全位 40 °/s，抓取关节 30 °/s，末段直线 0.08 m/s（控制器上限 100 °/s）。

## 精度调整

1. **标定核对**：`yolo_detect.py` 读出的基座系坐标与卷尺实测相差超过 1 cm 时重新标定——确认标定板尺寸为实测值、板面平整、相机固定牢靠。
2. **高度**：物体不在桌面上时填 `support_z_m`；夹取深浅用 `grasp_offset_m` 调，每次 0.005。
3. **固定偏差**：同一位置抓取 3 次，比较日志 `Grasp …: base xyz` 与实测物体中点；偏差稳定时把均值填入 `base_offset_m`，日志出现 `Applied base_offset_m` 即生效。

更换夹爪、相机支架或相机安装位置后须重新标定。

## 目录

```text
apps/vision_grasp/
├── handeye.py          # 手眼标定（--check-config / --solve / 实机采集）
├── yolo_detect.py      # 检测预览
├── grasp_yolo.py       # 视觉抓取（g 抓取 / q 退出）
├── config/             # default.yaml（单 ArUco）、charuco.yaml（ChArUco）；标定结果与采集记录也存于此
├── models/             # YOLO 权重（另行下载）
├── scripts/            # live_view / teach_poses / pose_test / dryrun / fk_compare
├── utils/              # 几何、相机、机械臂适配层、设备选择
├── tests/              # 无硬件测试
└── 实机验收清单.md
```

## 致谢与许可

标定与抓取流程移植自 [d1arm_sdk](https://gitee.com/gccodespace/d1arm_sdk)（作者 bxd，分支 `grasp_back`），
经作者授权以 MIT 许可发布，署名见 [LICENSE](LICENSE)。参考了 `reBot-DevArm-Grasp` 的流程与几何约定、
OpenCV `calibrateHandEye`。模型权重版权归 Ultralytics。
