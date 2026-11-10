# D1-T 只读 3D 显示

订阅 `rt/d1t/arm_state`，用 MuJoCo 窗口显示 D1-T 六轴姿态和夹爪开合。
工具只创建状态 DataReader，不创建命令 DataWriter，不实例化 SDK 的 `D1Arm`。
每帧写入 `qpos` 后调用 `mj_forward` 和 `viewer.sync()`，不调用 `mj_step`，不运行动力学。

模型直接读取仓库 URDF/STL；只在内存中解析绝对 mesh 路径、移除重复 collision 外观并添加 TCP site。
红、绿、蓝轴分别表示 X、Y、Z，标出 `base_link` 与 `TCP`，使用 MuJoCo 原生 site 坐标系显示，长度均为 140 mm。
TCP 的 Link6 局部位姿来自 `sdk/arm_model/d1t.json`。

## 环境

需要本地图形桌面（从桌面终端运行；普通 SSH 会话缺 `DISPLAY` 不能开窗）。MuJoCo 装在独立 venv，
不动系统 Python、不需要激活 ROS：

```bash
bash deploy/setup_host.sh --with-viz      # 建 ~/.venvs/mujoco-3.14.0（MuJoCo 3.14.0 + 源码装 cyclonedds 0.10.2）；已有则跳过
```

验证环境为 MuJoCo 3.14.0、Python 3.10、NumPy 2.2.6、Cyclone DDS Python 0.10.2。
venv 里的 cyclonedds 用源码装、链接本机 0.10.2 C 库（预编译 wheel 可能附带不同版本的 C 库）；
自建 venv 时等价命令是 `CYCLONEDDS_HOME=<C 库前缀> <venv>/bin/python -m pip install --no-binary=cyclonedds cyclonedds==0.10.2`。
每个终端先 `source deploy/env.sh …`：它把 0.10.2 的库路径放到 `LD_LIBRARY_PATH` 最前，
避免装了 ROS2 的终端先加载另一版 `libddsc`（混用会崩溃）。
安装方式见 [Cyclone DDS 官方包说明](https://pypi.org/project/cyclonedds/0.10.2/)；
显示接口见 [MuJoCo Python 文档](https://mujoco.readthedocs.io/en/latest/python.html)。

## 怎么跑：本机仿真

以下终端都先进仓库根目录。

**终端 1：起仿真控制器**（脚本会先确认没有别的控制器在跑）：

```bash
bash deploy/sim_start.sh
```

**终端 2：打开显示。** 默认 `lo / 127.0.0.1 / index 6`，程序自行设置 DDS URI（覆盖终端里的 URI）。

```bash
source deploy/env.sh sim '' '' viz
~/.venvs/mujoco-3.14.0/bin/python tools/arm_viz/arm_viz.py
```

**终端 3：仿真命令源**（SDK，index 3）。这些命令会发运动命令，仅在上面的仿真环境运行：

```bash
source deploy/env.sh sim
python3 -m d1t_sdk state
python3 sdk/examples/05_stream.py
python3 sdk/examples/02_move_joints.py
python3 -m d1t_sdk gripper 70
python3 -m d1t_sdk gripper 4
```

`state` 的 `q_deg` 应与显示终端一致；`05_stream.py` 的 J0 应左右摆动；
`02_move_joints.py` 应整臂变姿再回零；最后两条命令应显示张开与闭合。
关闭窗口或按 Ctrl-C 只退出显示工具，不发送停止/保持命令。控制器需在其终端单独 Ctrl-C 退出。

## 怎么跑：真臂只读观察

真臂控制器已经运行（`deploy/实机部署.md`）、本机网卡能访问臂时，在桌面终端执行：

```bash
source deploy/env.sh real <本机接臂的网卡名> '' viz
~/.venvs/mujoco-3.14.0/bin/python tools/arm_viz/arm_viz.py --iface <本机接臂的网卡名> --peer 192.168.123.100 --index 6
```

该命令仍然只订阅状态。无需登录臂载计算机，也不需要启动任何运动示例。
`--index` 必须与同机其他 DDS 进程不同。控制器 / SDK / 录制 / 显示分别使用 0 / 3 / 5 / 6。

## 输出与一致性检查

每秒输出 `mode/ctrl_state`、六轴 `q_deg`（URDF 度，与 SDK 一致）、夹爪指间距（mm）、
TCP 在 base 下的位置（mm）、`FK_error`（MuJoCo TCP 与 SDK FK 的欧氏距离，mm）和 `valid_mask`。
`mode` 是控制器状态名，不推断其他 SDK 进程的 `MOVING` 状态。
误差超过 1 mm 标记 `MODEL_MISMATCH`。URDF 的 RPY 为有限小数，SDK 使用精确 π，存在微小误差。

离线检查不创建 DDS participant，不需要控制器：

```bash
~/.venvs/mujoco-3.14.0/bin/python tools/arm_viz/test_arm_viz.py   # 先 source deploy/env.sh sim '' '' viz
```

检查 500 组限位内随机姿态的 TCP 误差、4/20/45/70 mm 指间距、无效数值拒绝与默认 DDS 配置。

## 已知限制

- 这是运动学状态显示，不模拟负载、重力、碰撞或接触。窗口里的拖动不会控制机械臂。
- 未收到状态时显示 URDF 初始姿态并打印 `WAITING`；超过 1 秒未收到状态则保持最后姿态并打印 `STALE`。窗口仍保留模型，不代表实时反馈。
- 无效轴使用控制器提供的上次有效值，需结合 `valid_mask` 查看；不做插值、预测或硬件安全判断。
- 夹爪以 STL 两指内侧表面的投影间距匹配 SDK 金属指间距；不模拟软垫压缩，超过标定表范围由 SDK 截断。显示时不按 URDF 滑块旧限位裁剪，也不运行约束求解。
- 真臂只读观察已核对：J0、J3 正转时模型与真臂转向一致，折叠位整体姿态一致。需要本地图形桌面；普通 SSH 会话缺少 `DISPLAY`/显示授权时不能直接开窗。
