// frame.hpp —— FashionStar 总线舵机帧编解码（纯函数，无 I/O）
//
// 帧格式（docs/hardware/Servo Motor/PROTOCOL_NOTES.md「通用规则」）：
//   请求 12 4C | cmd | len | content | cs     响应 05 1C | cmd | len | content | cs
//   cs = 前序全部字节累加 % 256；多字节字段小端；角度单位 0.1°。
// 参照 tests/scripts/fsbus.py（已实测）的 build / _try_extract / _decode_monitor_params。
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace d1t {
namespace frame {

using Bytes = std::vector<uint8_t>;

constexpr uint8_t REQ_H0 = 0x12, REQ_H1 = 0x4C;
constexpr uint8_t RSP_H0 = 0x05, RSP_H1 = 0x1C;

constexpr uint8_t CMD_PING = 0x01;
constexpr uint8_t CMD_SET_ANGLE = 0x08;
constexpr uint8_t CMD_READ_MTURN = 0x10;
constexpr uint8_t CMD_CLEAR_MTURN = 0x11;
constexpr uint8_t CMD_MONITOR = 0x16;
constexpr uint8_t CMD_STOP = 0x18;
constexpr uint8_t CMD_SYNC = 0x19;

// 0x18 停止模式（PROTOCOL_NOTES 0x18 节；F48 三模式实测生效）
constexpr uint8_t STOP_RELEASE = 0x10;
constexpr uint8_t STOP_HOLD = 0x11;
constexpr uint8_t STOP_DAMP = 0x12;

uint8_t checksum(const uint8_t* p, size_t n);

// 通用请求帧
Bytes build(uint8_t cmd, const Bytes& content);

// 度 → 线上 0.1° int16，钳 ±1800（fsbus.py deg_to_raw）
int16_t deg_to_raw(float deg);

Bytes ping(uint8_t id);                                                  // 0x01
Bytes set_angle(uint8_t id, float deg, uint16_t time_ms, uint16_t power_mw);  // 0x08
Bytes read_mturn(uint8_t id);                                            // 0x10
Bytes clear_mturn(uint8_t id);                                           // 0x11
Bytes monitor(uint8_t id);                                               // 0x16
Bytes stop(uint8_t id, uint8_t mode, uint16_t power_mw);                 // 0x18

struct MoveItem {
  uint8_t id;
  float deg;
  uint16_t time_ms;
  uint16_t power_mw;
};
// 0x19 + 0x08（d=7，n=items.size()）——PROTOCOL_NOTES 0x19 节；F61 实测
Bytes sync_set_angle(const std::vector<MoveItem>& items);
// 0x19 + 0x16（d=1）——PROTOCOL_NOTES 0x19 节；F46 实测
Bytes sync_monitor(const std::vector<uint8_t>& ids);

// ---------- 响应解析 ----------

struct RspFrame {
  uint8_t cmd = 0;
  Bytes content;
  bool checksum_ok = false;
};

// 流式提取器：字节随到随喂，一次取一帧；垃圾字节丢弃，校验错的帧整帧交出（由调用方计坏帧）
class Parser {
 public:
  void feed(const uint8_t* p, size_t n) { buf_.insert(buf_.end(), p, p + n); }
  // 取出一帧返回 true；不完整返回 false
  bool next(RspFrame& out);
  void clear() { buf_.clear(); }
  size_t buffered() const { return buf_.size(); }
  uint32_t garbage_bytes() const { return garbage_; }

 private:
  Bytes buf_;
  uint32_t garbage_ = 0;
};

struct MonitorData {
  uint8_t id = 0;
  uint16_t voltage_mv = 0, current_ma = 0, power_mw = 0, temp_adc = 0;
  uint8_t status = 0;
  int32_t angle_raw = 0;  // 0.1°，按 int32 解（SDK <BHHHHBih；fsbus.py 同，F58 负角度实测可读）
  int16_t turns = 0;
  bool sentinel = false;  // angle == -235929599 且 turns == 0 → 无效测量
};
// 0x16 响应 content（16 字节）；长度不符返回 false
bool decode_monitor(const Bytes& content, MonitorData& out);

struct MturnData {
  uint8_t id = 0;
  int32_t angle_raw = 0;  // 0.1° 多圈累计
  int16_t turns = 0;
};
// 0x10 响应 content（7 字节）
bool decode_mturn(const Bytes& content, MturnData& out);

// NTC：T = 1 / (ln(adc/(4096−adc))/B + 1/298.15) − 273.15；adc 越界返回 NaN（PROTOCOL_NOTES 附录 A）
float temp_adc_to_c(uint16_t adc);

}  // namespace frame
}  // namespace d1t
