// types.hpp —— 控制器内部数据结构（与 DDS 生成代码解耦）
//
// Cmd / StateMsg 与任务书 §1.2 的 ArmCmd_ / ArmState_ 逐字段一一对应；
// dds_io 负责在两者之间拷贝。核心（bus / safety / loop）不依赖 DDS。
#pragma once

#include <array>
#include <cstdint>

#include "config.hpp"

namespace d1t {

constexpr int N = cfg::kNumAxes;

// ArmCmd_.mode（任务书 §1.2）
enum class Mode : uint8_t { STREAM = 0, HOLD = 1, RELEASE = 2, DAMP = 3, ESTOP = 4, RESET = 5 };

// ArmState_.ctrl_state（任务书 §1.2）
enum class CtrlState : uint8_t { INIT = 0, IDLE = 1, STREAMING = 2, HOLD = 3, RELEASED = 4, ESTOP = 5, FAULT = 6 };

// fault / warn 位（任务书 §1.4）
namespace bits {
constexpr uint32_t WATCHDOG = 1u << 0;  // 状态位（不锁存），放在 fault 字段
constexpr uint32_t ESTOP = 1u << 1;
constexpr uint32_t MTURN = 1u << 2;
constexpr uint32_t SERVO_PROT = 1u << 3;
constexpr uint32_t COMM_LOSS = 1u << 4;
constexpr uint32_t J6_THERMAL = 1u << 5;  // 含「J6 温度连续 COMM_LOSS_CYCLES 周期不可知」（§3.7-3）
// 以下两位由 T3 选号（任务书 §3.7）
constexpr uint32_t TX_FAIL = 1u << 6;    // 写帧本周期重试后仍失败（§3.7-5）；提示物理急停
constexpr uint32_t HOLD_FAIL = 1u << 7;  // 保持后重发 0x18 两次仍未确认停住（§3.7-6）
constexpr uint32_t LATCHED_MASK = ESTOP | MTURN | SERVO_PROT | COMM_LOSS | J6_THERMAL | TX_FAIL | HOLD_FAIL;

constexpr uint32_t LIMIT_CLAMPED = 1u << 8;
constexpr uint32_t STEP_CLAMPED = 1u << 9;
constexpr uint32_t CMD_STALE = 1u << 10;
constexpr uint32_t CMD_REJECTED = 1u << 11;
constexpr uint32_t J6_PROT = 1u << 12;
constexpr uint32_t J6_HOT = 1u << 13;
constexpr uint32_t TARGET_FAR = 1u << 14;
constexpr uint32_t OUT_OF_LIMIT = 1u << 15;
constexpr uint32_t RESET_DENIED = 1u << 16;  // RESET 有位未清（原因未经新鲜反馈证明解除，§3.7-2）；T3 选号
}  // namespace bits

// = ArmCmd_ + 控制器收到时刻
struct Cmd {
  uint32_t seq = 0;
  int64_t stamp_ns = 0;
  uint8_t mode = 0;
  uint8_t axis_mask = 0;
  std::array<float, N> q{};
  uint16_t dt_ms = 0;
  uint16_t grip_power_mw = 0;
  // 以下由 dds_io 收到时填（不在 IDL 里）
  int64_t rx_real_ns = 0;  // CLOCK_REALTIME，回显到 ArmState.cmd_rx_ns
  int64_t rx_mono_ns = 0;  // CLOCK_MONOTONIC，看门狗计时用
  uint64_t rx_no = 0;      // 信箱接收序号（全局递增）；ESTOP 接收代次判断用（§3.7-1）
};

// 一个轴的监控数据（0x16 回包解出）
struct AxisSample {
  uint16_t voltage_mv = 0;
  uint16_t current_ma = 0;
  uint16_t power_mw = 0;
  uint16_t temp_adc = 0;
  float temp_c = 0.0f;  // ADC 越界时为 NaN
  uint8_t status = 0;
  float q = 0.0f;  // 度（0x16 position / 10，多圈累计口径）
  int16_t turns = 0;
  int64_t t_ns = 0;  // 该回包收到时刻（MONOTONIC）；停住速度判据用
};

// 一个周期的总线反馈：valid[i] = 本周期收到 Ji 的有效回包（温度 ADC 无效也算无效，§3.7-3）；
// ax[i] 为最近一次有效值（无效轴保持上一次有效值）；have[i] = 曾经有过有效值。
struct Feedback {
  std::array<bool, N> valid{};
  std::array<bool, N> have{};
  std::array<AxisSample, N> ax{};
};

// = ArmState_
struct StateMsg {
  uint32_t seq = 0;
  int64_t stamp_ns = 0;
  uint32_t sample_span_us = 0;
  uint8_t ctrl_state = 0;
  uint32_t cmd_seq_ack = 0;
  int64_t cmd_stamp_ns = 0;
  int64_t cmd_rx_ns = 0;
  uint8_t valid_mask = 0;
  std::array<float, N> q{};
  std::array<float, N> q_target{};
  std::array<uint16_t, N> current_ma{};
  std::array<uint16_t, N> voltage_mv{};
  std::array<float, N> temp_c{};
  std::array<uint8_t, N> status{};
  std::array<int16_t, N> turns{};
  uint32_t fault = 0;
  uint32_t warn = 0;
  uint32_t overruns = 0;
};

}  // namespace d1t
