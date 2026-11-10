#include "sim/fake_client.hpp"

#include <algorithm>
#include <cmath>

#include "common/clock.hpp"

namespace d1t {

void FakeClient::start(bool stream, float amp, float freq, uint8_t mask, uint16_t dt_ms) {
  amp_ = amp;
  freq_ = freq;
  mask_ = mask;
  dt_ms_ = dt_ms;
  streaming_ = stream;
  run_ = true;
  th_ = std::thread([this] { run(); });
}

void FakeClient::stop() {
  if (!run_.exchange(false)) return;
  if (th_.joinable()) th_.join();
}

int64_t FakeClient::send(Mode m, uint8_t axis_mask) {
  Cmd c;
  c.seq = ++seq_;
  c.stamp_ns = now_real_ns();
  c.mode = uint8_t(m);
  c.axis_mask = axis_mask;
  {
    std::lock_guard<std::mutex> lk(mu_);
    c.q = base_;
  }
  c.dt_ms = dt_ms_;
  int64_t t = now_mono_ns();
  c.rx_mono_ns = t;
  mb_.put_cmd(c);
  return t;
}

void FakeClient::run() {
  const int64_t P = int64_t(period_ms_) * 1000000LL;
  int64_t next = now_mono_ns();
  int64_t t0 = 0;
  while (run_) {
    // 收 ArmState 直到下一个发送时刻
    StateMsg s;
    while (mb_.pop_state(s, std::max<int64_t>(0, next - now_mono_ns()))) {
      std::lock_guard<std::mutex> lk(mu_);
      if (!have_base_) {
        base_ = s.q;
        have_base_ = true;
      }
      states_.push_back(s);
    }
    if (!run_ || mb_.is_shutdown()) break;
    if (streaming_ && have_base_) {
      if (!t0) t0 = now_mono_ns();
      double t = double(now_mono_ns() - t0) * 1e-9;
      Cmd c;
      c.seq = ++seq_;
      c.stamp_ns = now_real_ns();
      c.mode = uint8_t(Mode::STREAM);
      {
        std::lock_guard<std::mutex> lk(mu_);
        c.q = base_;
      }
      for (int i = 0; i < N; ++i)
        if (mask_ & (1u << i)) c.q[i] += amp_ * float(std::sin(2.0 * M_PI * freq_ * t));
      c.dt_ms = dt_ms_;
      mb_.put_cmd(c);
    }
    next += P;
    if (next < now_mono_ns()) next = now_mono_ns();
  }
}

std::vector<StateMsg> FakeClient::states() {
  std::lock_guard<std::mutex> lk(mu_);
  return states_;
}

size_t FakeClient::n_states() {
  std::lock_guard<std::mutex> lk(mu_);
  return states_.size();
}

}  // namespace d1t
