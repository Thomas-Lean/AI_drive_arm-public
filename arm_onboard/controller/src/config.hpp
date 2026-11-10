// config.hpp —— 臂载控制器全部常量（P1.1-T3）
//
// 规则（任务书 §0 阈值规则、§4 T3 验收第 4 条）：
//   每个常量必须带 F 号（docs/FACTS.md）或【暂定】注释；能沿用旧项目或实测值的一律沿用。
//   【暂定】= 无旧值、无实测，由 TC-010 实测再定。
//   「任务书 §x」指 P1.1 控制器设计任务书 v0.2（项目内部文档，未随源码发布）。
#pragma once

#include <array>
#include <cstdint>

namespace d1t {
namespace cfg {

// ---------------- 版本 ----------------

constexpr const char* kCtrlVersion = "0.1.0";  // 发布版本，与根 CHANGELOG.md 对应；启动日志打印

// ---------------- 总线与舵机 ----------------

constexpr int kNumAxes = 7;  // J0–J6，J6 = 夹爪（F48「ID6=夹爪」实机确认）
// 舵机 ID = 关节号（旧控制器 marm_controller_node.cpp:37-44，fsbus.py 同；F48 实机确认 ID6）
constexpr std::array<uint8_t, kNumAxes> kServoId = {0, 1, 2, 3, 4, 5, 6};

constexpr const char* kSerialPort = "/dev/ttyS4";  // F2、F29：唯一舵机总线
constexpr int kBaud = 115200;                       // F10：现用 115200

// 总线上任意两帧之间 ≥10 ms，从上一帧发送完成算起（F38 厂家口径、F14；fsbus.py send() 同）
constexpr int kCmdGapMs = 10;

// 启动阶段 0x01 / 0x10 单舵机等回包窗：20 ms（F41：TC-005 A/B 等回包窗统一 20 ms）
constexpr int kStartupRespTimeoutMs = 20;
// 0x01 ping 重试次数（任务书 §3.3「重试 2 次」）
constexpr int kPingRetries = 2;
// 启动阶段 0x10 读多圈重试次数：2（同 ping，任务书 §3.3「重试 2 次」；只读，重试不放宽判据。
//   读不到仍按超限处理——fsbus.py mturn_gate「读不到 = 不确定，按超限」）
constexpr int kMturnReadRetries = 2;
// 启动 / 运行中 J6 清圈重试次数（任务书 §3.6「重试 2 次」）
constexpr int kMturnClearRetries = 2;
// 0x18 释放 → 0x11 之间的等待：100 ms（fsbus.py clear_turns 的 sleep(0.1)，F58 处置实机生效）
constexpr int kReleaseToClearMs = 100;
// 0x11 之后到读回之间的等待：200 ms（fsbus.py mturn_gate 的 sleep(0.2)）
constexpr int kClearToReadMs = 200;

// 0x16 角度无效值哨兵：angle == -235929599 且 turns == 0（PROTOCOL_NOTES 0x16 节，uservo.py:709）
constexpr int32_t kMonitorSentinelAngle = -235929599;

// NTC 温度换算 B 值 3435（PROTOCOL_NOTES 附录 A，本地复算 ADC1191→50.0 ℃）
constexpr double kNtcB = 3435.0;

// ---------------- 周期 ----------------

constexpr int kPeriodMsDefault = 50;  // 20 Hz（F46：7 轴同步监控中位 16.4 / max 33.9 ms，20 Hz 余量充足）
constexpr int kPeriodMsMin = 50;      // 任务书 §3.1「可配 50–200 ms」
constexpr int kPeriodMsMax = 200;     // 任务书 §3.1「可配 50–200 ms」

// 0x19+0x16 收包超时 = 周期剩余 − 5 ms，且不超过 40 ms【暂定；任务书 §3.1；静止 max 33.9 ms F46，运动中更慢 F52】
// 余量须覆盖缺包后的迟到静默窗 kLateQuietWindowMs（10 ms）：原 5 ms 时任一轴缺一个回包 → 等到「周期末 −5 ms」
//   再静默 10 ms → 本周期超时约 5 ms → 下一周期立刻开始，上位机按帧发的命令还没到 → 一帧沿用旧命令、下一帧跳一条（流式顿挫）
constexpr int kMonitorTimeoutMarginMs = 16;
constexpr int kMonitorTimeoutMaxMs = 40;  // 【暂定；任务书 §3.1】
// 收包超时下限 20 ms【暂定；任务书未给值。周期已超时（如 ESTOP 后逐轴 0x18）时剩余时间可能为负，
//   取 F41 的 20 ms 等回包窗兜底，保证每周期仍有一次监控】
constexpr int kMonitorTimeoutMinMs = 20;

// ---------------- 限位与步长 ----------------

struct Limit {
  float lo;
  float hi;
};
// 臂上实际运行的限位（度）——F7（fsbus.py JOINT_LIMITS 同值）
constexpr std::array<Limit, kNumAxes> kJointLimit = {{
    {-163.0f, 163.0f},  // J0
    {-90.3f, 97.0f},    // J1
    {-97.0f, 89.0f},    // J2
    {-166.0f, 163.0f},  // J3
    {-96.0f, 113.0f},   // J4
    {-166.0f, 162.0f},  // J5
    {-30.0f, 64.5f},    // J6（舵机角，开度；F60 夹持区间 29–64.5）
}};

// 单周期最大步长 = 轴速 × 周期。舵机额定 J0–J2 210°/s、J3–J6 312°/s，但实机 3 s 往返（J3 峰值 185°/s）
//   与 2.1 s 往返（J3 235°/s）都在数轮后触发舵机保护 SERVO_PROT；实际可用上限暂按 100°/s（2026-11-09 实机定）
constexpr std::array<float, kNumAxes> kAxisSpeedDps = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f};

// J6 功率上限钳到 6000 mW（任务书 §1.2 grip_power_mw；F60：P=6000 接触冲击可触发功率位）
constexpr uint16_t kGripPowerMaxMw = 6000;
// J0–J5 下发 power = 0，按舵机保护阈值执行（任务书 §3.2；TC-006/008 同，F49、F61）
constexpr uint16_t kArmPowerMw = 0;
// 0x18 停止帧 power = 0（F49：0 与 6000 实测等效；F38 厂家「填 0 也有锁力」）
constexpr uint16_t kStopPowerMw = 0;

// ---------------- 写入失败（任务书 §3.7-5，R04/R09） ----------------

// 每帧本周期内最多再重试 2 次（任务书 §3.7-5「有界重试 ≤2 次」）
constexpr int kTxRetries = 2;
// 单帧 write + 排空的 MONOTONIC 总期限 20 ms【暂定；最长帧 0x19+0x08 七轴 55 字节线上 4.8 ms 的 4 倍余量】
constexpr int kTxDeadlineMs = 20;
// 排空查询 ioctl 不可用时按字节数估算发送时长，再加 UART FIFO 余量 2 ms【暂定；任务书 §3.7-5（V2-01）；
//   16 字节 FIFO 在 115200 下 ≈1.4 ms】
constexpr int kUartFifoMarginMs = 2;

// ---------------- 迟到帧隔离（任务书 §3.7-8，R11；fsbus.py wait_quiet） ----------------

// 同步读超时后开静默窗：窗口内仍到达的完整帧记为迟到并丢弃。窗 10 ms（fsbus.py wait_quiet window_s=0.01）
constexpr int kLateQuietWindowMs = 10;
// 静默窗总上限 20 ms【暂定；fsbus.py 用 200 ms，对 50 ms 周期过长】
constexpr int kLateQuietMaxMs = 20;

// ---------------- 看门狗、恢复 ----------------

constexpr int kWatchdogMs = 200;       // 【暂定，无旧值；任务书 §3.2，4 个周期】只在 STREAMING 下起作用（TC-010 实机）
constexpr float kResumeTolDeg = 5.0f;  // 【暂定；任务书 §3.2 RESUME_TOL_DEG】

// ---------------- 多圈 ----------------

// 任一轴 |q| ≥ 180 → 多圈超限（F58 规则；任务书 §1.4 MTURN、§3.3）。只看角度：turns = floor(角度/360)，
//   负角度时 turns = −1 属正常（F63），turns 只记录、不作判据
constexpr float kMturnLimitDeg = 180.0f;

// ---------------- 通信丢失 ----------------

// 任一轴连续 N 周期无有效回包 → COMM_LOSS 故障。
// 4【暂定；任务书 §1.4 只给名字未给值。E1 目标「单轴连续缺失 ≤2 周期」，取 4 = 50 ms 周期下 200 ms，与 kWatchdogMs 对齐】
constexpr int kCommLossCycles = 4;

// ---------------- 状态位（0x16 status 字节，PROTOCOL_NOTES 附录 A） ----------------

// 任一轴出现即 SERVO_PROT 故障：bit3 高压、bit4 低压、bit5 过流、bit7 温度（任务书 §1.4）
constexpr uint8_t kStatusFaultAnyAxis = (1u << 3) | (1u << 4) | (1u << 5) | (1u << 7);
// bit2 堵转、bit6 功率：J0–J5 故障，J6 只告警（F60：P=6000 夹持接触时 bit6 置位属限力生效）
constexpr uint8_t kStatusFaultArmOnly = (1u << 2) | (1u << 6);

// ---------------- J6 温度（任务书 §3.5） ----------------

constexpr float kJ6TempWarnC = 45.0f;     // 【暂定；F61 夹持 74 s 到 44.5 ℃】≥ 此值进入降出力
// 降出力回差：≤ 43 ℃ 才退出【暂定，2026-11-05 实机定；TC-010 E_thermal3：44.9/45.0 ℃ 抖动时无回差进出 90 次】
constexpr float kJ6TempWarnExitC = 43.0f;
// 50 ℃【暂定】：2026-11-04 定；F58 事故最后记录 51.7 ℃ 后舵机过温断电，F61 正常夹持 74 s 到 44.5 ℃（任务书 §3.7-9，R13）
constexpr float kJ6TempReleaseC = 50.0f;
// 45 ℃ 降出力：进入时把 J6 目标锁定为当时实测角（不再每周期跟随实测——TC-010 E_thermal3 瓶子回弹顶开手指，
// 目标跟着走，30.2°→38.2°，电流降到空夹 14 mA）。J6 总线目标比锁定值更闭合 ≥0.2° 时发一次 0x08 到锁定值
// 【暂定；> 协议分辨率 0.1°，避免每周期重发】
constexpr float kJ6HotRetargetDeg = 0.2f;

// ---------------- 卡住判定（任务书 §3.6，仅 J0–J5 流式） ----------------

constexpr float kTrackErrDeg = 10.0f;  // 【暂定；任务书 §3.6 TRACK_ERR_DEG】
constexpr int kTrackErrMs = 1000;      // 任务书 §3.6「持续 > 1 s」
// 「该轴电流高」的阈值 500 mA【暂定；任务书未给值。参照：静止 14 mA（F43）、J6 堵转 948 mA（F58）、
//   J6 夹持稳态 216–550 mA（F57、F60）；J0–J5 带载运动电流无实测，TC-010 C 阶段实测后再定】
constexpr uint16_t kTrackStuckCurrentMa = 500;

// ---------------- 停住确认（任务书 §3.6、§3.7-6；保持 / 看门狗 / 故障 / ESTOP 统一） ----------------

// 停住判据 = 窗口范围（TC-010 实机，PM 裁定；原「连续 3 个采样对 < 2 °/s」被静止读数抖动误触发 HOLD_FAIL）：
// 最近 kStillWindow 个连续新鲜采样，每轴 max(q) − min(q) ≤ kStillRangeDeg 即停住。
// 依据 tests/runs/2026-11-04-TC-010/A_readonly2（静止锁住，n≈1395 个窗口）：6 / 8 采样窗口范围最大都是
// J0 0.3°、J1/J2 0.2°、J5 0.1°、J3/J4/J6 0；相邻帧 |Δq| 最大 0.3°。
// 取 8 而非 6：2 °/s × 5 间隔 = 0.5° 正好等于阈值，6 采样抓不住 2 °/s 慢移；8 采样 ≈ 0.7°，两边各留约 0.2°
constexpr int kStillWindow = 8;           // 【暂定】约 350 ms
constexpr float kStillRangeDeg = 0.5f;    // 【暂定；STILL_RANGE_DEG】
// 相邻采样跨度 > 2 个周期 → 窗口清空重攒（缺回包 / 跨度过大不证明静止，R08）【暂定】
constexpr int kStillMaxDtPeriods = 2;
// 发 0x18 0x11 后须在此时间内确认停住；须能凑满 kStillWindow 个保持帧之后的采样（8 × 50 ms ≈ 370 ms + 余量）【暂定】
constexpr int kHoldConfirmMs = 500;
// 未确认 → 重发 0x18 最多 2 次，仍未停 → HOLD_FAIL（任务书 §3.7-6）
constexpr int kHoldResendMax = 2;

// ---------------- DDS（任务书 §1.1） ----------------

constexpr int kDdsDomain = 0;  // 与旧系统相同（任务书 §1.1）
constexpr const char* kTopicCmd = "rt/d1t/arm_cmd";      // 任务书 §1.1
constexpr const char* kTopicState = "rt/d1t/arm_state";  // 任务书 §1.1

// 主循环 → dds_io 的 ArmState 队列深度 64【暂定；实现细节，满了丢最旧并计数】
constexpr int kStateQueueDepth = 64;

}  // namespace cfg
}  // namespace d1t
