// mailbox.hpp —— dds_io 与主循环之间唯一的接口（任务书 §2 边界规则 1）
//
// 命令方向（dds_io → 主循环）：只存最新一条；ESTOP 单独一格，不会被后来的命令覆盖，
//   放入即唤醒主循环（任务书 §3.4）。新旧按 stamp_ns 严格递增判断（§1.3），过期计 CMD_STALE；
//   格式非法（safety::cmd_valid）计 CMD_REJECTED；两者都不进信箱。
//   例外：ESTOP 不做新旧判断——另一台机器发的急停可能时钟落后，不能被当成过期丢掉；
//   ESTOP 也不抬高普通命令的 stamp 水位（R12）。
// 接收代次（§3.7-1，R01）：每条收下的命令带全局递增 rx_no；ESTOP 放入时作废信箱里尚未取走的普通命令，
//   safety 只认 rx_no 大于最近一次 ESTOP 的 RESET。
// 状态方向（主循环 → dds_io）：有界队列（kStateQueueDepth），满了丢最旧并计数。
#pragma once

#include <cstdint>
#include <deque>

#include "common/clock.hpp"
#include "common/types.hpp"

namespace d1t {

class Mailbox {
 public:
  // ---- dds_io 线程调用 ----
  // 调用方填好 rx_real_ns / rx_mono_ns（为 0 时此处补填）
  void put_cmd(Cmd c);
  // 等一帧 ArmState，最多 timeout_ns；有则返回 true
  bool pop_state(StateMsg& out, int64_t timeout_ns);

  // ---- 主循环线程调用 ----
  bool take_cmd(Cmd& out);
  // 是否有未取走的 ESTOP（主循环下发长序列时检查，R05）
  bool estop_pending();
  void push_state(const StateMsg& s);
  // 睡到 deadline（MONOTONIC），有 ESTOP 待处理或 shutdown 时提前返回；返回 true = 被 ESTOP 唤醒
  bool wait_until(int64_t deadline_mono_ns);
  // 自上次调用以来是否有过期 / 非法命令（给 warn 位）
  void take_counts(bool& stale, bool& rejected);

  void shutdown();
  bool is_shutdown();

  struct Totals {
    uint64_t cmds = 0, stale = 0, rejected = 0, overwritten = 0, states_dropped = 0, dropped_by_estop = 0;
  };
  Totals totals();

 private:
  MonoEvent ev_;
  bool has_latest_ = false;
  Cmd latest_;
  bool has_estop_ = false;
  Cmd estop_;
  bool any_stamp_ = false;
  int64_t last_stamp_ = 0;
  uint64_t rx_no_ = 0;
  uint64_t stale_seen_ = 0, rejected_seen_ = 0;
  std::deque<StateMsg> states_;
  bool shutdown_ = false;
  Totals tot_;
};

}  // namespace d1t
