# arm_model — 臂身份证（d1t.json）

SDK 的全部臂参数都从 [`d1t.json`](d1t.json) 读；换臂 / 重标定时**只改这个文件，不改代码**。
`d1_description/` 是臂的 URDF 源包（SolidWorks 导出，含 mesh），JSON 里的运动学数值与它逐项一致。

## d1t.json 字段

| 字段 | 含义 | 来源 |
|---|---|---|
| `joints[i].xyz / rpy` | 第 i 关节（i=0–5 对应 J0–J5）的连杆变换：父系→子系平移（m）+ 欧拉角（rad，URDF 约定 RzRyRx） | URDF `<origin xyz rpy>`。注意 URDF 文本里角度是四位小数近似（如 `-3.1416`），JSON 里写的是 π 的双精度值，两者数值上等价 |
| `joints[i].axis` | 关节旋转轴（该连杆局部系） | URDF `<axis xyz>` |
| `tcp.xyz / rpy` | 末端工具点：Link6 系下的平移 + 欧拉角 | 两夹爪指（URDF Joint7_1/7_2）origin 的中点；换工具（如装相机）时改这里 |
| `calib.sign / offset_deg` | 舵机角 ↔ URDF 角换算：`URDF角 = sign × (舵机角 − offset_deg)`，下标 0–5 对应 J0–J5 | 出厂方向标定。**换臂或首次部署必须复核方向与零位**（见 `sdk/实机验收清单.md`）；offset=0 时 FK/IK 的绝对精度待米尺复核 |
| `servo_limits_deg` | 7 轴（含 J6）舵机角空间限位 `[lo, hi]`，度 | 与臂载控制器 `arm_onboard/controller/src/config.hpp` 的 `kJointLimit` 一致（tests 自动比对） |
| `axis_speed_dps` | 控制器按轴限速（°/s，舵机角空间），7 轴 | 与控制器 `kAxisSpeedDps` 一致（tests 自动比对）。规划时逐轴钳到该值，否则控制器截速会让各轴不同步 |
| `j6_speed_dps` | 夹爪（J6）运动速度 | 实机标定值，比轴限速保守 |
| `gripper_table_deg_mm` | J6 舵机角 ↔ 金属指间距 mm 标定表（8 点，分段线性插值，出界截断） | 实机标定。垫面间距 = 金属指间距 − 4 |
| `urdf_source` | 运动学数值对应的 URDF 文件（本目录内） | — |

## 换臂 / 重标定要改什么

1. **方向或零位变了**：改 `calib.sign / offset_deg`（量具对零后逐轴测）。
2. **限位或限速变了**（控制器 config.hpp 同步改过）：改 `servo_limits_deg / axis_speed_dps`，
   并确认控制器侧同步——SDK 预检与控制器是两道闸，数值必须一致。
3. **夹爪重新标定**：重测 8 点填 `gripper_table_deg_mm`。
4. **换了整个臂型**：换 `d1_description/`，重填 `joints / tcp`，其余按上述复核。
5. 改完跑 `python3 sdk/tests/run_all.py`：逐位金标测试会失败属预期（臂变了，行为就该变），
   需重新生成金标；限位一致性测试必须过。

## 生成代码说明

- `d1t.json` 不做 URDF 自动解析（16 个数不值得带一个解析器）；数值变更走人工核对 + 测试比对。
- `d1_description/` 原样保留（含 `meshes/`、`textures/`，RViz/MoveIt 可直接用其 CMakeLists/package.xml）。
