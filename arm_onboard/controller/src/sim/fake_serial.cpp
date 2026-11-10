// fake_serial.cpp —— --sim 用的 7 舵机仿真（无硬件）
#include <cmath>
#include <cstring>

#include "bus/serial.hpp"
#include "common/clock.hpp"

namespace d1t {

namespace {
constexpr int64_t kReplyFirstNs = 3000000;  // 发送完成后第一个回包 3.0 ms
constexpr int64_t kReplyStepNs = 2200000;   // 之后每个 +2.2 ms（7 轴 ≈16.2 ms，对齐 F46 中位 16.4）
constexpr double kByteNs = 10.0 * 1e9 / 115200.0;  // 8N1 每字节 10 bit

// 圈数 = 多圈角 / 360 向下取整（F63：负角度时为 −1）
int turns_of(double a) { return int(std::floor(a / 360.0)); }
// 0x11 清圈后的角度：归到 (−180, 180]（F58：388.6 → 28.6）
double wrap180(double a) { return a - 360.0 * std::ceil((a - 180.0) / 360.0); }

uint16_t adc_from_c(float c) {
  double r = std::exp(cfg::kNtcB * (1.0 / (double(c) + 273.15) - 1.0 / 298.15));
  double adc = 4096.0 * r / (1.0 + r);
  if (adc < 1) adc = 1;
  if (adc > 4095) adc = 4095;
  return uint16_t(std::lround(adc));
}

void put_u16(frame::Bytes& b, uint16_t v) {
  b.push_back(uint8_t(v & 0xFF));
  b.push_back(uint8_t(v >> 8));
}
void put_i32(frame::Bytes& b, int32_t v) {
  uint32_t u;
  std::memcpy(&u, &v, 4);
  for (int i = 0; i < 4; ++i) b.push_back(uint8_t((u >> (8 * i)) & 0xFF));
}
}  // namespace

FakeSerial::FakeSerial(uint32_t seed) : rng_(seed) {
  // 初始姿态：各轴在限位内的小角度，释放态（上电锁力关，F36）
  const double init[cfg::kNumAxes] = {0.0, -60.0, 60.0, 0.0, 10.0, 0.0, 40.0};
  for (int i = 0; i < cfg::kNumAxes; ++i) {
    sv_[i].a0 = sv_[i].target = init[i];
  }
}

double FakeSerial::pos_at(const Servo& s, int64_t t) const {
  double dt = double(t - s.t0) * 1e-9;
  if (dt < 0) dt = 0;
  switch (s.mode) {
    case SMode::MOVING:
      return s.target + (s.a0 - s.target) * std::exp(-dt / s.tau_s);
    case SMode::HOLD:
      return s.a0 + double(s.hold_drift) * dt;
    default:
      return s.a0;
  }
}

bool FakeSerial::write_all(const uint8_t* p, size_t n) {
  int64_t t_start = now_mono_ns();
  int64_t tx_end = t_start + int64_t(double(n) * kByteNs);
  sleep_until_mono(tx_end);  // 线上时长
  std::lock_guard<std::mutex> lk(mu_);
  req_buf_.insert(req_buf_.end(), p, p + n);
  // 解析请求帧（12 4C | cmd | len | content | cs）
  for (;;) {
    size_t i = 0;
    while (i + 1 < req_buf_.size() && !(req_buf_[i] == frame::REQ_H0 && req_buf_[i + 1] == frame::REQ_H1)) ++i;
    if (i > 0) req_buf_.erase(req_buf_.begin(), req_buf_.begin() + long(i));
    if (req_buf_.size() < 4) break;
    size_t total = 4 + size_t(req_buf_[3]) + 1;
    if (req_buf_.size() < total) break;
    bool ok = frame::checksum(req_buf_.data(), total - 1) == req_buf_[total - 1];
    uint8_t cmd = req_buf_[2];
    frame::Bytes c(req_buf_.begin() + 4, req_buf_.begin() + long(total) - 1);
    req_buf_.erase(req_buf_.begin(), req_buf_.begin() + long(total));
    ++frames_;
    if (ok) handle_request(cmd, c, tx_end);  // 校验错的请求舵机不响应
  }
  return true;
}

void FakeSerial::apply_move(uint8_t id, int16_t raw, uint16_t time_ms, int64_t t) {
  if (id >= cfg::kNumAxes) return;
  Servo& s = sv_[id];
  if (s.offline) return;
  s.a0 = pos_at(s, t);
  s.t0 = t;
  // 0x08 目标按多圈坐标执行（F58 事故机理），仿真照此建模
  s.target = raw / 10.0;
  s.tau_s = (time_ms < 1 ? 1 : time_ms) / 3000.0;  // time 内走完约 95%
  s.mode = SMode::MOVING;
  ++s.move_count;
}

bool FakeSerial::should_reply(Servo& s) {
  if (s.offline) return false;
  if (s.drop_next > 0) {
    --s.drop_next;
    return false;
  }
  if (s.drop_p > 0) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    if (u(rng_) < s.drop_p) return false;
  }
  return true;
}

frame::Bytes FakeSerial::make_reply(uint8_t cmd, const frame::Bytes& content, Servo& s) {
  frame::Bytes b{frame::RSP_H0, frame::RSP_H1, cmd, uint8_t(content.size())};
  b.insert(b.end(), content.begin(), content.end());
  uint8_t cs = frame::checksum(b.data(), b.size());
  if (s.corrupt_p > 0) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    if (u(rng_) < s.corrupt_p) cs ^= 0xFF;
  }
  b.push_back(cs);
  return b;
}

frame::Bytes FakeSerial::monitor_content(uint8_t id, int64_t t) {
  Servo& s = sv_[id];
  double pos = pos_at(s, t);
  bool moving = s.mode == SMode::MOVING && std::fabs(pos - s.target) > 0.1;
  frame::Bytes c{id};
  put_u16(c, 12200);                    // 电压 mV
  put_u16(c, moving ? 120 : 14);        // 电流 mA（静止 14 mA，F43）
  put_u16(c, moving ? 1500 : 172);      // 功率 mW
  put_u16(c, s.temp_adc >= 0 ? uint16_t(s.temp_adc) : adc_from_c(s.temp_c));  // 温度 ADC
  c.push_back(uint8_t((moving ? 0x01 : 0x00) | s.status_inj));  // bit0 运动中（F50 观察）
  put_i32(c, int32_t(std::lround(pos * 10.0)));
  put_u16(c, uint16_t(int16_t(turns_of(pos))));
  return c;
}

void FakeSerial::handle_request(uint8_t cmd, const frame::Bytes& c, int64_t tx_end) {
  auto schedule = [&](int k, frame::Bytes&& bytes, int64_t extra = 0) {
    Pending p{tx_end + kReplyFirstNs + k * kReplyStepNs + extra, std::move(bytes)};
    auto it = pending_.end();  // 按可读时刻有序插入（有延迟注入时会乱序）
    while (it != pending_.begin() && (it - 1)->ready_ns > p.ready_ns) --it;
    pending_.insert(it, std::move(p));
  };
  switch (cmd) {
    case frame::CMD_PING: {
      if (c.size() != 1 || c[0] >= cfg::kNumAxes) return;
      Servo& s = sv_[c[0]];
      if (should_reply(s)) schedule(0, make_reply(cmd, {c[0]}, s));
      return;
    }
    case frame::CMD_READ_MTURN: {
      if (c.size() != 1 || c[0] >= cfg::kNumAxes) return;
      Servo& s = sv_[c[0]];
      if (!should_reply(s)) return;
      double pos = pos_at(s, tx_end);
      frame::Bytes r{c[0]};
      put_i32(r, int32_t(std::lround(pos * 10.0)));
      put_u16(r, uint16_t(int16_t(turns_of(pos))));
      schedule(0, make_reply(cmd, r, s));
      return;
    }
    case frame::CMD_MONITOR: {
      if (c.size() != 1 || c[0] >= cfg::kNumAxes) return;
      Servo& s = sv_[c[0]];
      if (should_reply(s)) schedule(0, make_reply(cmd, monitor_content(c[0], tx_end), s));
      return;
    }
    case frame::CMD_CLEAR_MTURN: {
      if (c.size() != 1 || c[0] >= cfg::kNumAxes) return;
      Servo& s = sv_[c[0]];
      if (s.offline) return;
      ++s.clear_count;
      if (s.mode == SMode::RELEASED) {  // 只在释放态有效（PROTOCOL_NOTES 0x11 节）
        double pos = pos_at(s, tx_end);
        s.a0 = s.target = wrap180(pos);
        s.t0 = tx_end;
      }
      return;
    }
    case frame::CMD_STOP: {
      if (c.size() != 4 || c[0] >= cfg::kNumAxes) return;
      Servo& s = sv_[c[0]];
      if (s.offline) return;
      s.a0 = s.target = pos_at(s, tx_end);
      s.t0 = tx_end;
      s.mode = c[1] == frame::STOP_HOLD ? SMode::HOLD : c[1] == frame::STOP_DAMP ? SMode::DAMP : SMode::RELEASED;
      ++s.stop_count;
      last_stop_ = tx_end;
      return;
    }
    case frame::CMD_SET_ANGLE: {
      if (c.size() != 7) return;
      int16_t raw = int16_t(c[1] | (c[2] << 8));
      apply_move(c[0], raw, uint16_t(c[3] | (c[4] << 8)), tx_end);
      return;
    }
    case frame::CMD_SYNC: {
      if (c.size() < 3) return;
      uint8_t sub = c[0], d = c[1], n = c[2];
      if (c.size() != size_t(3 + d * n)) return;
      if (sub == frame::CMD_SET_ANGLE && d == 7) {
        for (int k = 0; k < n; ++k) {
          const uint8_t* q = &c[3 + k * 7];
          apply_move(q[0], int16_t(q[1] | (q[2] << 8)), uint16_t(q[3] | (q[4] << 8)), tx_end);
        }
        last_sync_move_ = tx_end;
      } else if (sub == frame::CMD_MONITOR && d == 1) {
        int k = 0;
        for (int j = 0; j < n; ++j) {
          uint8_t id = c[3 + j];
          if (id >= cfg::kNumAxes) continue;
          Servo& s = sv_[id];
          if (should_reply(s))
            schedule(k, make_reply(frame::CMD_MONITOR, monitor_content(id, tx_end), s), s.delay_ns);
          ++k;  // 丢包的轴也占时隙
        }
      }
      return;
    }
    default:
      return;
  }
}

size_t FakeSerial::read_some(uint8_t* p, size_t cap, int64_t timeout_ns) {
  int64_t deadline = now_mono_ns() + timeout_ns;
  for (;;) {
    int64_t now = now_mono_ns();
    int64_t next_ready = INT64_MAX;
    {
      std::lock_guard<std::mutex> lk(mu_);
      while (!pending_.empty() && pending_.front().ready_ns <= now) {
        rx_.insert(rx_.end(), pending_.front().bytes.begin(), pending_.front().bytes.end());
        pending_.pop_front();
      }
      if (!rx_.empty()) {
        size_t k = rx_.size() < cap ? rx_.size() : cap;
        std::memcpy(p, rx_.data(), k);
        rx_.erase(rx_.begin(), rx_.begin() + long(k));
        return k;
      }
      if (!pending_.empty()) next_ready = pending_.front().ready_ns;
    }
    if (now >= deadline) return 0;
    sleep_until_mono(next_ready < deadline ? next_ready : deadline);
  }
}

size_t FakeSerial::flush_input() {
  std::lock_guard<std::mutex> lk(mu_);
  int64_t now = now_mono_ns();
  while (!pending_.empty() && pending_.front().ready_ns <= now) {
    rx_.insert(rx_.end(), pending_.front().bytes.begin(), pending_.front().bytes.end());
    pending_.pop_front();
  }
  size_t n = rx_.size();
  rx_.clear();
  return n;
}

void FakeSerial::inject_drop_prob(int a, double p) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].drop_p = p;
}
void FakeSerial::inject_drop_next(int a, int count) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].drop_next = count;
}
void FakeSerial::inject_corrupt_prob(int a, double p) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].corrupt_p = p;
}
void FakeSerial::inject_status(int a, uint8_t st) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].status_inj = st;
}
void FakeSerial::inject_temp_c(int a, float c) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].temp_c = c;
}
void FakeSerial::inject_mturn(int a, float deg) {
  std::lock_guard<std::mutex> lk(mu_);
  Servo& s = sv_[a];
  s.a0 = s.target = deg;
  s.t0 = now_mono_ns();
  if (s.mode == SMode::MOVING) s.mode = SMode::HOLD;
}
void FakeSerial::inject_offline(int a, bool off) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].offline = off;
}
void FakeSerial::inject_angle_offset_on_hold(int a, float dps) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].hold_drift = dps;
}

void FakeSerial::inject_locked(int a) {
  std::lock_guard<std::mutex> lk(mu_);
  Servo& s = sv_[a];
  s.a0 = s.target = pos_at(s, now_mono_ns());
  s.t0 = now_mono_ns();
  s.mode = SMode::HOLD;
}

void FakeSerial::inject_temp_adc(int a, int adc) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].temp_adc = adc;
}
void FakeSerial::inject_reply_delay_ms(int a, double ms) {
  std::lock_guard<std::mutex> lk(mu_);
  sv_[a].delay_ns = int64_t(ms * 1e6);
}

FakeSerial::ServoView FakeSerial::view(int a) {
  std::lock_guard<std::mutex> lk(mu_);
  const Servo& s = sv_[a];
  return {float(pos_at(s, now_mono_ns())), float(s.target), s.mode, s.clear_count, s.stop_count, s.move_count};
}
uint64_t FakeSerial::frames_written() {
  std::lock_guard<std::mutex> lk(mu_);
  return frames_;
}
int64_t FakeSerial::last_sync_move_tx_end_ns() {
  std::lock_guard<std::mutex> lk(mu_);
  return last_sync_move_;
}
int64_t FakeSerial::last_stop_tx_end_ns() {
  std::lock_guard<std::mutex> lk(mu_);
  return last_stop_;
}

}  // namespace d1t
