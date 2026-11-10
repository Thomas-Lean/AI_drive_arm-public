# Joy-Con 遥操作 D1（迁移中）

> **来源**：[`gitee.com/gccodespace/d1arm_sdk`](https://gitee.com/gccodespace/d1arm_sdk) 分支 `joycon`，
> 提交 [`f77d370`](https://gitee.com/gccodespace/d1arm_sdk/commit/f77d370) 的 `clients/tele/`
> 三件（`joycon_teleop.py`、`test_joycon_teleop.py`、`README.md`），原作者 **bxdw**（署名 bxd，
> 见 `sdk/LICENSE`），MIT 授权随本项目发布。
>
> 迁移目标：驱动臂的调用从原作者的 d1armd（TCP 7777 JSON）换成 `d1t_sdk.D1Arm`（DDS 直连
> `d1t_arm_ctrl`），手柄映射与姿态算法不动。`pico_teleop.py` 及其测试不迁。
> 本 README 为迁移起点，接口对照表与使用说明随后补全。

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
