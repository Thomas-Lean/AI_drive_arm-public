# idl/ — DDS 消息定义与生成代码

控制器与上位机之间只有两条消息（话题 `rt/d1t/arm_cmd`、`rt/d1t/arm_state`，domain 0，
QoS BEST_EFFORT + KEEP_LAST(1) + VOLATILE）：

- `d1t_arm.idl` — `d1t_interfaces::msg::dds_` 模块下 `ArmCmd_` / `ArmState_`，每个字段的含义见文件内注释
- `generated/` — idlc（Cyclone DDS **0.10.2**）生成的 C++ 代码，**已提交入库**；编译控制器不需要再生成
- Python 类型在 `sdk/d1t_sdk/d1t_interfaces/`（由 pip cyclonedds==0.10.2 自带的 idlc 生成，已提交）
- 依赖安装：`deploy/setup_host.sh`（Cyclone DDS 0.10.2 + cyclonedds-cxx 0.10.2 源码装）

只有改了 `d1t_arm.idl` 才需要按下面两节重新生成，并让控制器与 SDK 一起更新。

## IDL 类型映射

消息设计用 ROS2 的类型写法（uint8/uint16/uint32/int64），IDL 里按 ROS2 rosidl 同款映射，
线上格式逐字节一致：

| ROS2 写法 | IDL 写法 | C++ 类型（生成代码） |
|---|---|---|
| uint8 | octet | uint8_t |
| uint16 | unsigned short | uint16_t |
| uint32 | unsigned long | uint32_t |
| int64 | long long | int64_t |
| int16 | short | int16_t |
| float | float | float（数组→std::array<float,7>） |

idlc 打印的 "No default extensibility ... default 'final'" 是提示不是错误：两条消息为
final（定长结构），与 ROS2 rosidl 生成的默认一致。

## 版本锁 0.10.2

臂载计算机出厂自带 Cyclone DDS C++ 0.10.2（原厂程序的生成代码头注为 `Cyclone DDS: v0.10.2`），
上位机与之保持同版本；不同版本不保证互通。

macOS 上用新版 Apple clang/libc++ 编译 ddscxx 0.10.2 头文件（refmacros.hpp 的
OMG_DDS_IMPLICIT_REF_BASE 宏，`shared_ptr` 相等比较）会报错，需要本地改成语义等价的
owner_before 双向比较；Linux（gcc）不受影响。

## 重新生成 C++ 代码（进 generated/）

```bash
# 注意：idlcxx 0.10.2 的 -o 选项会被忽略，必须在目标目录内执行
PATH="$HOME/opt/cyclonedds-0.10.2/bin:$PATH"
cd generated && idlc -l cxx ../d1t_arm.idl && cd ..
```

产出 `generated/d1t_arm.hpp` + `generated/d1t_arm.cpp`（一对文件包含两个类型，
头注 `Cyclone DDS: v0.10.2`）。

## 重新生成 Python 类型（进 sdk/d1t_sdk/d1t_interfaces/）

```bash
pip3 install --user cyclonedds==0.10.2
# 用 pip 包自带的 idlc 生成；-o 会被忽略，须在目标目录内执行
```

产出 `sdk/d1t_sdk/d1t_interfaces/msg/dds_/_d1t_arm.py`（`ArmCmd_`/`ArmState_`
dataclass）+ 各级 `__init__.py`。pip 包自带的 idlc 属于 cyclonedds-python wheel
（生成文件头注 idlpy 版本 v0.11.0），仅用于 Python 类型生成；运行时库版本以
`pip show cyclonedds` = 0.10.2 为准，与 C++ 0.10.2 互通。

## DDS 配置要点

- 固定网卡 + `AllowMulticast=false` + 单播 Peer：默认全网卡多播在带代理虚拟网卡（TUN）的
  机器上可能卡死或被劫持。
- 每个进程固定不同的 ParticipantIndex（端口 = 7410+2×index）：auto 模式可能绑不上标准
  RTPS 端口，单播下互相发现不了。
- 以上都由 `deploy/env.sh`（上位机）与 `deploy/arm_start.sh`（臂上）自动写好。

## 自己编译节点时

- include 路径：C 头 `include/`、C++ 头 `include/ddscxx/`（臂上为 `/usr/local/include/ddscxx`），
  链接 `-lddscxx -lddsc`
- 把 `generated/d1t_arm.cpp` 一起编入（`get_type_props` 等符号在其中）
- ddscxx API 注意：`DataWriter`/`DataReader` 没有 `(topic, qos)` 两参构造，
  要先建 `Publisher(dp)` / `Subscriber(dp)` 再 `(pub, topic, qos)`
