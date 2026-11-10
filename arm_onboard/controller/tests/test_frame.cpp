// test_frame.cpp —— 编解码单测：PROTOCOL_NOTES.md 官方示例帧逐条过（任务书 §4 T3 验收 1）
#include <cmath>
#include <cstdio>

#include "bus/frame.hpp"
#include "check.hpp"

using namespace d1t;
using frame::Bytes;

static Bytes H(std::initializer_list<int> v) {
  Bytes b;
  for (int x : v) b.push_back(uint8_t(x));
  return b;
}

static void feed(frame::Parser& p, const Bytes& b) { p.feed(b.data(), b.size()); }

int main() {
  // ---- 请求帧编码（每条都是 PROTOCOL_NOTES 中「复算一致」的官方示例） ----
  // 0x01 节：12 4C 01 01 00 60
  CHECK(frame::ping(0) == H({0x12, 0x4C, 0x01, 0x01, 0x00, 0x60}));
  // 0x08 节：ID0 +90.0° 500 ms 功率 0 → 12 4C 08 07 00 84 03 F4 01 00 00 E9
  CHECK(frame::set_angle(0, 90.0f, 500, 0) == H({0x12, 0x4C, 0x08, 0x07, 0x00, 0x84, 0x03, 0xF4, 0x01, 0x00, 0x00, 0xE9}));
  // 0x10 节：12 4C 10 01 00 6F
  CHECK(frame::read_mturn(0) == H({0x12, 0x4C, 0x10, 0x01, 0x00, 0x6F}));
  // 0x11 节：12 4C 11 01 00 70
  CHECK(frame::clear_mturn(0) == H({0x12, 0x4C, 0x11, 0x01, 0x00, 0x70}));
  // F58 实机生效帧：12 4C 11 01 06 → 校验和 0x76
  CHECK(frame::clear_mturn(6) == H({0x12, 0x4C, 0x11, 0x01, 0x06, 0x76}));
  // 0x16 节：12 4C 16 01 00 75
  CHECK(frame::monitor(0) == H({0x12, 0x4C, 0x16, 0x01, 0x00, 0x75}));
  // 0x18 节：ID0 保持锁力 6000 mW → 12 4C 18 04 00 11 70 17 12
  CHECK(frame::stop(0, frame::STOP_HOLD, 6000) == H({0x12, 0x4C, 0x18, 0x04, 0x00, 0x11, 0x70, 0x17, 0x12}));
  // 0x19 节：同步两个 0x08 → 12 4C 19 11 08 07 02 | 01 2C 01 E8 03 00 00 | 02 58 02 D0 07 00 00 | E5
  CHECK(frame::sync_set_angle({{1, 30.0f, 1000, 0}, {2, 60.0f, 2000, 0}}) ==
        H({0x12, 0x4C, 0x19, 0x11, 0x08, 0x07, 0x02, 0x01, 0x2C, 0x01, 0xE8, 0x03, 0x00, 0x00, 0x02, 0x58, 0x02,
           0xD0, 0x07, 0x00, 0x00, 0xE5}));
  // 0x19+0x16 7 轴：12 4C 19 0A 16 01 07 00..06 cs（PROTOCOL_NOTES 未给 cs；按通用规则累加 = 0xB4）
  CHECK(frame::sync_monitor({0, 1, 2, 3, 4, 5, 6}) ==
        H({0x12, 0x4C, 0x19, 0x0A, 0x16, 0x01, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0xB4}));
  // 7 轴同步 0x08：length=52，整帧 57 字节（PROTOCOL_NOTES 0x19 节）
  {
    std::vector<frame::MoveItem> it;
    for (uint8_t i = 0; i < 7; ++i) it.push_back({i, 0.0f, 50, 0});
    Bytes b = frame::sync_set_angle(it);
    CHECK(b.size() == 57);
    CHECK(b[3] == 52);
  }
  // 负角度编码：-42.4° → -424 = 0xFE58（小端 58 FE）
  {
    Bytes b = frame::set_angle(6, -42.4f, 50, 0);
    CHECK(b[5] == 0x58 && b[6] == 0xFE);
  }
  // 角度钳 ±1800
  CHECK(frame::deg_to_raw(250.0f) == 1800);
  CHECK(frame::deg_to_raw(-250.0f) == -1800);

  // ---- 响应解析 ----
  frame::Parser p;
  frame::RspFrame f;
  // 0x01 响应：05 1C 01 01 00 23
  feed(p, H({0x05, 0x1C, 0x01, 0x01, 0x00, 0x23}));
  CHECK(p.next(f) && f.checksum_ok && f.cmd == 0x01 && f.content == H({0x00}));
  // 0x08 响应：05 1C 08 02 00 01 2C
  feed(p, H({0x05, 0x1C, 0x08, 0x02, 0x00, 0x01, 0x2C}));
  CHECK(p.next(f) && f.checksum_ok && f.cmd == 0x08);
  // 0x0A 响应：05 1C 0A 03 00 86 03 B7（+90.2°）
  feed(p, H({0x05, 0x1C, 0x0A, 0x03, 0x00, 0x86, 0x03, 0xB7}));
  CHECK(p.next(f) && f.checksum_ok && f.cmd == 0x0A && int16_t(f.content[1] | (f.content[2] << 8)) == 902);
  // 0x03 响应示例页面校验和笔误（复算 0x1C ≠ 页面 0xB7）——解析器必须判坏帧
  feed(p, H({0x05, 0x1C, 0x03, 0x03, 0x00, 0xF4, 0x01, 0xB7}));
  CHECK(p.next(f) && !f.checksum_ok);
  // 0x10 响应：05 1C 10 07 00 23 13 00 00 01 00 6F → +489.9°、+1 圈
  feed(p, H({0x05, 0x1C, 0x10, 0x07, 0x00, 0x23, 0x13, 0x00, 0x00, 0x01, 0x00, 0x6F}));
  {
    frame::MturnData m;
    CHECK(p.next(f) && f.checksum_ok && frame::decode_mturn(f.content, m));
    CHECK(m.id == 0 && m.angle_raw == 4899 && m.turns == 1);
  }
  // 0x16 响应：05 1C 16 10 00 83 1E 1E 00 EA 00 2C 07 01 AF 0B 00 00 00 00 DE
  //   → 7811 mV、30 mA、234 mW、ADC 1836、状态 1、299.1°、0 圈
  const Bytes mon = H({0x05, 0x1C, 0x16, 0x10, 0x00, 0x83, 0x1E, 0x1E, 0x00, 0xEA, 0x00, 0x2C, 0x07, 0x01, 0xAF, 0x0B,
                       0x00, 0x00, 0x00, 0x00, 0xDE});
  feed(p, mon);
  {
    frame::MonitorData m;
    CHECK(p.next(f) && f.checksum_ok && frame::decode_monitor(f.content, m));
    CHECK(m.id == 0 && m.voltage_mv == 7811 && m.current_ma == 30 && m.power_mw == 234);
    CHECK(m.temp_adc == 1836 && m.status == 1 && m.angle_raw == 2991 && m.turns == 0 && !m.sentinel);
  }
  // 哨兵：angle = -235929599、turns = 0 → 无效测量（uservo.py:709）
  {
    Bytes c = H({0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    int32_t s = -235929599;
    uint32_t u = uint32_t(s);
    for (int i = 0; i < 4; ++i) c.push_back(uint8_t(u >> (8 * i)));
    c.push_back(0);
    c.push_back(0);
    frame::MonitorData m;
    CHECK(frame::decode_monitor(c, m) && m.sentinel);
    c[14] = 1;  // turns ≠ 0 → 不是哨兵
    CHECK(frame::decode_monitor(c, m) && !m.sentinel);
  }
  // 负多圈角（int32 解码）：-42.4° → -424
  {
    Bytes c = H({0x06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x58, 0xFE, 0xFF, 0xFF, 0, 0});
    frame::MonitorData m;
    CHECK(frame::decode_monitor(c, m) && m.angle_raw == -424);
  }
  // 长度不符拒解
  {
    frame::MonitorData m;
    CHECK(!frame::decode_monitor(H({0, 1, 2}), m));
  }

  // ---- 流式提取：垃圾前缀 + 粘包 + 拆包 ----
  {
    frame::Parser q;
    Bytes stream = H({0xFF, 0x00, 0x05, 0x33});  // 垃圾（含一个假 0x05）
    stream.insert(stream.end(), mon.begin(), mon.end());
    stream.insert(stream.end(), mon.begin(), mon.end());
    // 前 10 字节先到
    q.feed(stream.data(), 10);
    CHECK(!q.next(f));
    q.feed(stream.data() + 10, stream.size() - 10);
    CHECK(q.next(f) && f.checksum_ok && f.cmd == 0x16);
    CHECK(q.next(f) && f.checksum_ok && f.cmd == 0x16);
    CHECK(!q.next(f));
    CHECK(q.garbage_bytes() == 4);
  }

  // ---- 温度换算（PROTOCOL_NOTES 附录 A 复算：ADC 1191→50.0 ℃、598→79.0 ℃） ----
  CHECK_NEAR(frame::temp_adc_to_c(1191), 50.0, 0.1);
  CHECK_NEAR(frame::temp_adc_to_c(598), 79.0, 0.1);
  CHECK(std::isnan(frame::temp_adc_to_c(0)));
  CHECK(std::isnan(frame::temp_adc_to_c(4096)));

  return chk::report("test_frame");
}
