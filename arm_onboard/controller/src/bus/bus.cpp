#include "bus/bus.hpp"

#include <algorithm>

#include "common/clock.hpp"

namespace d1t {

int64_t Bus::send(const frame::Bytes& f, bool flush_before, int64_t* t_real) {
  if (last_tx_end_ != 0) sleep_until_mono(last_tx_end_ + int64_t(cfg::kCmdGapMs) * 1000000LL);
  if (flush_before) {
    st_.flushed_bytes += s_.flush_input();
    parser_.clear();
  }
  if (t_real) *t_real = now_real_ns();
  int64_t t0 = now_mono_ns();
  bool ok = s_.write_all(f.data(), f.size());
  last_tx_end_ = now_mono_ns();
  if (!ok) {
    ++st_.tx_fail;
    return 0;
  }
  ++st_.tx_frames;
  return t0;
}

uint32_t Bus::quiet_window() {
  const int64_t ms = 1000000LL;
  int64_t end_all = now_mono_ns() + cfg::kLateQuietMaxMs * ms;
  int64_t win_end = std::min(now_mono_ns() + cfg::kLateQuietWindowMs * ms, end_all);
  uint32_t late = 0;
  uint8_t buf[256];
  frame::RspFrame f;
  for (;;) {
    int64_t now = now_mono_ns();
    if (now >= win_end) break;
    size_t n = s_.read_some(buf, sizeof(buf), win_end - now);
    if (n) parser_.feed(buf, n);
    bool got = false;
    while (parser_.next(f)) {
      ++late;
      got = true;
    }
    if (got) win_end = std::min(now_mono_ns() + cfg::kLateQuietWindowMs * ms, end_all);  // 还在来：窗口顺延
  }
  st_.rx_late += late;
  st_.flushed_bytes += s_.flush_input();
  parser_.clear();
  return late;
}

bool Bus::transact(const frame::Bytes& req, uint8_t expect_cmd, size_t expect_len, int timeout_ms,
                   frame::RspFrame& out) {
  if (send(req, true) == 0) return false;
  int64_t deadline = now_mono_ns() + int64_t(timeout_ms) * 1000000LL;
  uint8_t buf[256];
  for (;;) {
    while (parser_.next(out)) {
      if (out.checksum_ok && out.cmd == expect_cmd && out.content.size() == expect_len) {
        ++st_.rx_ok;
        return true;
      }
      ++st_.rx_bad;
    }
    int64_t remain = deadline - now_mono_ns();
    if (remain <= 0) {
      quiet_window();
      return false;
    }
    size_t n = s_.read_some(buf, sizeof(buf), remain);
    if (n) parser_.feed(buf, n);
  }
}

bool Bus::ping(uint8_t id, int timeout_ms) {
  frame::RspFrame r;
  if (!transact(frame::ping(id), frame::CMD_PING, 1, timeout_ms, r)) return false;
  return r.content[0] == id;
}

bool Bus::read_mturn(uint8_t id, int timeout_ms, frame::MturnData& out) {
  frame::RspFrame r;
  if (!transact(frame::read_mturn(id), frame::CMD_READ_MTURN, 7, timeout_ms, r)) return false;
  return frame::decode_mturn(r.content, out) && out.id == id;
}

SyncMonitorResult Bus::sync_monitor(const std::vector<uint8_t>& ids, int64_t deadline_abs, int64_t min_ns,
                                     int64_t max_ns) {
  SyncMonitorResult res;
  // 迟到帧隔离（R11）：① 清缓冲放在睡完帧间隔之后、紧挨写请求之前；② 本次超时后开静默窗（见下）。
  // 协议回包无序号，剩余风险 = 清缓冲与写请求之间的微秒级窗口内恰好到达的上一轮迟到帧【未验证，实机迟到上界未测】
  res.req_tx_start_mono = send(frame::sync_monitor(ids), true, &res.req_tx_start_real);
  if (res.req_tx_start_mono == 0) return res;
  int64_t deadline = std::min(std::max(deadline_abs, res.req_tx_start_mono + min_ns), res.req_tx_start_mono + max_ns);
  size_t want = ids.size(), have = 0;
  uint8_t buf[256];
  frame::RspFrame f;
  while (have < want) {
    while (parser_.next(f)) {
      frame::MonitorData d;
      if (!f.checksum_ok || f.cmd != frame::CMD_MONITOR || !frame::decode_monitor(f.content, d)) {
        ++st_.rx_bad;
        continue;
      }
      bool requested = false;
      for (uint8_t id : ids) requested |= (id == d.id);
      if (!requested || d.id >= cfg::kNumAxes || res.got[d.id]) {
        ++st_.rx_extra;
        continue;
      }
      res.got[d.id] = true;
      res.data[d.id] = d;
      res.last_rx_mono = res.rx_mono[d.id] = now_mono_ns();
      ++st_.rx_ok;
      ++have;
    }
    if (have >= want) break;
    int64_t remain = deadline - now_mono_ns();
    if (remain <= 0) break;
    size_t n = s_.read_some(buf, sizeof(buf), remain);
    if (n) parser_.feed(buf, n);
  }
  if (have < want) res.late = quiet_window();
  return res;
}

}  // namespace d1t
