# 更新记录

## 0.1.0

首个发布版：臂载控制器、上位机 SDK、仿真与实机部署脚本。

**臂载控制器 `d1t_arm_ctrl`**（`arm_onboard/controller/`）
- 替换原厂臂上 4 个服务：C++17 + Cyclone DDS 0.10.2，单线程独占舵机总线，
  20 Hz 固定周期 0x19+0x08 同步下发、0x19+0x16 同步读回七轴状态。
- 看门狗（流式断命令约 0.3 s 停臂）、软急停、限位与限速（各轴 100°/s）、多圈超限检查、
  J6 45 ℃ 降出力 / 50 ℃ 释放、写失败与停住确认闭环。
- `--sim` 假舵机仿真（可注入丢包、坏帧、状态位、温度、多圈角），同一套代码。
- 启动日志打印版本号。

**上位机 SDK `d1t_sdk` 0.1.0**（`sdk/`）
- 接口与 d1arm_sdk 同名同参同单位；底层改为 DDS 直连 `d1t_arm_ctrl`，不起服务。
- FK / IK、关节五次多项式、末端直线；圆弧 `move_c` 有实现但未做实机验收。
- 每收到一帧状态发一条命令的帧同步流式（`stream_joints` / `stream_pose` / `stream_run`），
  流式可带夹爪目标 `j6_deg`（未做实机验收）。
- `pip install -e sdk` 可编辑安装（需 pip ≥ 21.3）。

**部署**（`deploy/`）
- 上位机：`setup_host.sh`（幂等装依赖，`--with-viz` 另建 MuJoCo 显示环境）、
  `env.sh sim|real`（自动设 DDS URI / 库路径 / PYTHONPATH，按角色固定 ParticipantIndex）、
  `sim_start.sh`（起仿真控制器）。
- 臂上：`arm_install.sh`（编译 + 测试）、`arm_start.sh`（停原厂服务并起控制器）、
  `arm_restore.sh`（回原厂服务）；可选开机自启 `d1t-arm-ctrl.service`（默认不装）。

**工具**
- `tools/arm_viz`：MuJoCo 只读 3D 显示（不发命令）。
- `tools/arm_client`：发命令 / 录状态 / 盯 J6 温度的维修工具。
