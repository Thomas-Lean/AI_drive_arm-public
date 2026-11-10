#include "bus/frame.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include "config.hpp"

namespace d1t {
namespace frame {

namespace {
void put_u16(Bytes& b, uint16_t v) {
  b.push_back(uint8_t(v & 0xFF));
  b.push_back(uint8_t(v >> 8));
}
void put_i16(Bytes& b, int16_t v) { put_u16(b, uint16_t(v)); }
uint16_t get_u16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
int32_t get_i32(const uint8_t* p) {
  uint32_t u = uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  int32_t v;
  std::memcpy(&v, &u, 4);
  return v;
}
}  // namespace

uint8_t checksum(const uint8_t* p, size_t n) {
  unsigned s = 0;
  for (size_t i = 0; i < n; ++i) s += p[i];
  return uint8_t(s & 0xFF);
}

Bytes build(uint8_t cmd, const Bytes& content) {
  Bytes b;
  b.reserve(content.size() + 5);
  b.push_back(REQ_H0);
  b.push_back(REQ_H1);
  b.push_back(cmd);
  b.push_back(uint8_t(content.size()));
  b.insert(b.end(), content.begin(), content.end());
  b.push_back(checksum(b.data(), b.size()));
  return b;
}

int16_t deg_to_raw(float deg) {
  if (!std::isfinite(deg)) return 0;  // 调用方保证不传 NaN；此处只防溢出
  long r = std::lround(double(deg) * 10.0);
  if (r > 1800) r = 1800;
  if (r < -1800) r = -1800;
  return int16_t(r);
}

Bytes ping(uint8_t id) { return build(CMD_PING, {id}); }
Bytes read_mturn(uint8_t id) { return build(CMD_READ_MTURN, {id}); }
Bytes clear_mturn(uint8_t id) { return build(CMD_CLEAR_MTURN, {id}); }
Bytes monitor(uint8_t id) { return build(CMD_MONITOR, {id}); }

Bytes set_angle(uint8_t id, float deg, uint16_t time_ms, uint16_t power_mw) {
  Bytes c{id};
  put_i16(c, deg_to_raw(deg));
  put_u16(c, time_ms);
  put_u16(c, power_mw);
  return build(CMD_SET_ANGLE, c);
}

Bytes stop(uint8_t id, uint8_t mode, uint16_t power_mw) {
  Bytes c{id, mode};
  put_u16(c, power_mw);
  return build(CMD_STOP, c);
}

Bytes sync_set_angle(const std::vector<MoveItem>& items) {
  Bytes c{CMD_SET_ANGLE, 7, uint8_t(items.size())};
  for (const auto& it : items) {
    c.push_back(it.id);
    put_i16(c, deg_to_raw(it.deg));
    put_u16(c, it.time_ms);
    put_u16(c, it.power_mw);
  }
  return build(CMD_SYNC, c);
}

Bytes sync_monitor(const std::vector<uint8_t>& ids) {
  Bytes c{CMD_MONITOR, 1, uint8_t(ids.size())};
  c.insert(c.end(), ids.begin(), ids.end());
  return build(CMD_SYNC, c);
}

bool Parser::next(RspFrame& out) {
  for (;;) {
    // 找响应头 05 1C
    size_t i = 0;
    while (i + 1 < buf_.size() && !(buf_[i] == RSP_H0 && buf_[i + 1] == RSP_H1)) ++i;
    if (i + 1 >= buf_.size()) {
      // 没有完整的头：只保留最后 1 字节（可能是 0x05）
      if (buf_.size() > 1) {
        garbage_ += uint32_t(buf_.size() - 1);
        buf_.erase(buf_.begin(), buf_.end() - 1);
      }
      return false;
    }
    if (i > 0) {
      garbage_ += uint32_t(i);
      buf_.erase(buf_.begin(), buf_.begin() + long(i));
    }
    if (buf_.size() < 4) return false;
    size_t total = 4 + size_t(buf_[3]) + 1;
    if (buf_.size() < total) return false;
    out.cmd = buf_[2];
    out.content.assign(buf_.begin() + 4, buf_.begin() + long(total) - 1);
    out.checksum_ok = checksum(buf_.data(), total - 1) == buf_[total - 1];
    buf_.erase(buf_.begin(), buf_.begin() + long(total));
    return true;
  }
}

bool decode_monitor(const Bytes& c, MonitorData& o) {
  if (c.size() != 16) return false;
  const uint8_t* p = c.data();
  o.id = p[0];
  o.voltage_mv = get_u16(p + 1);
  o.current_ma = get_u16(p + 3);
  o.power_mw = get_u16(p + 5);
  o.temp_adc = get_u16(p + 7);
  o.status = p[9];
  o.angle_raw = get_i32(p + 10);
  o.turns = int16_t(get_u16(p + 14));
  o.sentinel = (o.angle_raw == cfg::kMonitorSentinelAngle && o.turns == 0);
  return true;
}

bool decode_mturn(const Bytes& c, MturnData& o) {
  if (c.size() != 7) return false;
  o.id = c[0];
  o.angle_raw = get_i32(c.data() + 1);
  o.turns = int16_t(get_u16(c.data() + 5));
  return true;
}

float temp_adc_to_c(uint16_t adc) {
  if (adc == 0 || adc >= 4096) return std::numeric_limits<float>::quiet_NaN();
  double r = std::log(double(adc) / (4096.0 - adc));
  return float(1.0 / (r / cfg::kNtcB + 1.0 / 298.15) - 273.15);
}

}  // namespace frame
}  // namespace d1t
