#include "loop/mailbox.hpp"

#include "safety/safety.hpp"

namespace d1t {

namespace {
struct Guard {
  MonoEvent& e;
  explicit Guard(MonoEvent& x) : e(x) { e.lock(); }
  ~Guard() { e.unlock(); }
};
}  // namespace

void Mailbox::put_cmd(Cmd c) {
  if (c.rx_real_ns == 0) c.rx_real_ns = now_real_ns();
  if (c.rx_mono_ns == 0) c.rx_mono_ns = now_mono_ns();
  Guard g(ev_);
  ++tot_.cmds;
  if (!safety::cmd_valid(c)) {
    ++tot_.rejected;
    return;
  }
  bool estop = Mode(c.mode) == Mode::ESTOP;
  if (estop) {
    // ESTOP 不做新旧判断，也不抬高普通命令的 stamp 水位（R12）；
    // 排在它之前、尚未取走的普通命令（旧 RESET / HOLD / STREAM）一律作废（§3.7-1，R01）
    c.rx_no = ++rx_no_;
    if (has_latest_) {
      has_latest_ = false;
      ++tot_.dropped_by_estop;
    }
    if (!has_estop_) {  // 已有未处理的 ESTOP 就保留最早那条（时延按最早的算），接收代次取最新
      estop_ = c;
      has_estop_ = true;
    } else {
      estop_.rx_no = c.rx_no;
    }
    ev_.notify_all();
    return;
  }
  if (any_stamp_ && c.stamp_ns <= last_stamp_) {
    ++tot_.stale;
    return;
  }
  last_stamp_ = c.stamp_ns;
  any_stamp_ = true;
  c.rx_no = ++rx_no_;
  if (has_latest_) ++tot_.overwritten;
  latest_ = c;
  has_latest_ = true;
}

bool Mailbox::take_cmd(Cmd& out) {
  Guard g(ev_);
  if (has_estop_) {
    out = estop_;
    has_estop_ = false;
    return true;
  }
  if (has_latest_) {
    out = latest_;
    has_latest_ = false;
    return true;
  }
  return false;
}

void Mailbox::push_state(const StateMsg& s) {
  Guard g(ev_);
  if (int(states_.size()) >= cfg::kStateQueueDepth) {
    states_.pop_front();
    ++tot_.states_dropped;
  }
  states_.push_back(s);
  ev_.notify_all();
}

bool Mailbox::pop_state(StateMsg& out, int64_t timeout_ns) {
  int64_t deadline = now_mono_ns() + timeout_ns;
  Guard g(ev_);
  while (states_.empty() && !shutdown_) {
    if (now_mono_ns() >= deadline) return false;
    ev_.wait_until_locked(deadline);
  }
  if (states_.empty()) return false;
  out = states_.front();
  states_.pop_front();
  return true;
}

bool Mailbox::estop_pending() {
  Guard g(ev_);
  return has_estop_;
}

bool Mailbox::wait_until(int64_t deadline) {
  Guard g(ev_);
  for (;;) {
    if (has_estop_) return true;
    if (shutdown_ || now_mono_ns() >= deadline) return false;
    ev_.wait_until_locked(deadline);
  }
}

void Mailbox::take_counts(bool& stale, bool& rejected) {
  Guard g(ev_);
  stale = tot_.stale != stale_seen_;
  rejected = tot_.rejected != rejected_seen_;
  stale_seen_ = tot_.stale;
  rejected_seen_ = tot_.rejected;
}

void Mailbox::shutdown() {
  Guard g(ev_);
  shutdown_ = true;
  ev_.notify_all();
}

bool Mailbox::is_shutdown() {
  Guard g(ev_);
  return shutdown_;
}

Mailbox::Totals Mailbox::totals() {
  Guard g(ev_);
  return tot_;
}

}  // namespace d1t
