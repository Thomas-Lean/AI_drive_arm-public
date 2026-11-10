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

## 目录骨架

```text
apps/vision_grasp/
├── handeye.py            # 入口：手眼标定（--check-config / --solve / 真机采集）
├── grasp_yolo.py         # 入口：YOLO 抓取（g 冻结抓取 / q 松开回安全）
├── yolo_detect.py        # 入口：YOLO 检测预览（基座系/相机系坐标，纯检测）
├── config/
│   ├── default.yaml      # 全部配置（标定+抓取，含中文注释）
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
│   ├── teach_poses.py    #   拖动示教（**本平台不支持**，见「已知限制」）
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

## 依赖

- Python 3.10–3.11（`opencv-python==4.7.0.72` 无更高版本 wheel）
- `numpy<2`（OpenCV 4.7.0.72 按 NumPy 1.x ABI 构建）、PyYAML、`cyclonedds==0.10.2`（d1t_sdk 依赖）
- `pyrealsense2`（RealSense 相机，标定 / 检测 / 抓取需要）
- 抓取 / 检测另需 `ultralytics`（含 torch，体积较大）
- 臂侧：本仓库控制器 `d1t_arm_ctrl`（仿真 `--sim` 或实机；见 `deploy/`）

```bash
pip3 install --user -r apps/vision_grasp/requirements.txt
pip3 install --user pyrealsense2 ultralytics   # 仅相机/抓取需要
```

yoloe 注意：`set_classes` 需要 clip 包 + ~240 MB 文本编码器
（`mobileclip2_b.ts`，按 `utils/paths.py` 的约定放 `apps/` 下；没有的话首次
运行会自动下载到进程 CWD）。yoloe 下 `classes` 不能清空，否则零检测。

## 模型权重

默认配置用 `models/yoloe-26s-seg.pt`（开放词表 + 分割），**不随仓库发布**，
下载方式见 [`models/README.md`](models/README.md)。

## 与 d1t_sdk 的接口对照

上层脚本实际用到的机器人调用共 7 组，迁移时逐项核对了 d1armd 语义与
`d1t_sdk.D1Arm` 的对应关系，完整结论表随适配层一并提交（见下节）。
