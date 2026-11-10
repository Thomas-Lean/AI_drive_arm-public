// bus.hpp —— 总线事务层：帧间隔调度 + 请求/回包配对
//
// 只由主循环线程调用（任务书 §2 边界规则 1）。
// 帧间隔：任意两帧之间 ≥ kCmdGapMs，从上一帧发送完成算起（F38；fsbus.py send() 同）。
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "bus/frame.hpp"
#include "bus/serial.hpp"
#include "common/types.hpp"

namespace d1t {

struct BusStats {
  uint64_t tx_frames = 0;
  uint64_t tx_fail = 0;
  uint64_t rx_ok = 0;
  uint64_t rx_bad = 0;    // 校验错 / 长度错 / 命令错
  uint64_t rx_extra = 0;  // 非请求 ID、重复
  uint64_t rx_late = 0;   // 超时后静默窗内到达的完整帧（迟到，丢弃，R11）
  uint64_t flushed_bytes = 0;  // 发请求前清掉的残留字节（含迟到的残包）
};

struct SyncMonitorResult {
  std::array<bool, cfg::kNumAxes> got{};
  std::array<frame::MonitorData, cfg::kNumAxes> data{};
  int64_t req_tx_start_mono = 0;  // 请求开始发送（mono）
  int64_t req_tx_start_real = 0;  // 同一时刻的 CLOCK_REALTIME → ArmState.stamp_ns
  int64_t last_rx_mono = 0;       // 最后一个有效回包收到时刻；无回包为 0
  std::array<int64_t, cfg::kNumAxes> rx_mono{};  // 各轴回包收到时刻
  uint32_t late = 0;              // 本次超时后静默窗内丢弃的迟到帧数
};

class Bus {
 public:
  explicit Bus(ISerial& s) : s_(s) {}

  // 发一帧（先睡足帧间隔）；返回发送开始时刻（mono），失败返回 0。
  // flush_before：睡完帧间隔、紧挨写入前清输入缓冲（请求-回包事务用；间隔期间到达的迟到字节不算本次，R11）
  int64_t send(const frame::Bytes& f, bool flush_before = false, int64_t* t_real = nullptr);
  int64_t last_tx_end() const { return last_tx_end_; }

  // 单舵机请求-回包（启动阶段用）。返回 true 且 out 为匹配 cmd 的合法帧
  bool transact(const frame::Bytes& req, uint8_t expect_cmd, size_t expect_len, int timeout_ms,
                frame::RspFrame& out);
  bool ping(uint8_t id, int timeout_ms);
  bool read_mturn(uint8_t id, int timeout_ms, frame::MturnData& out);

  // 0x19+0x16 同步监控：收到 ids 全部或超时为止；超时后开静默窗隔离迟到帧。
  // 收包截止 = clamp(deadline_abs, 请求发出 + min_ns, 请求发出 + max_ns)；截止按请求实际发出时刻算
  // （发出前要先睡足帧间隔，不能提前算相对超时）。
  SyncMonitorResult sync_monitor(const std::vector<uint8_t>& ids, int64_t deadline_abs_mono, int64_t min_ns,
                                 int64_t max_ns);

  const BusStats& stats() const { return st_; }

 private:
  // 超时后的静默恢复窗（fsbus.py wait_quiet，R11）：kLateQuietWindowMs 内仍有完整帧到达就记迟到并继续等，
  // 总上限 kLateQuietMaxMs；最后清掉残包。返回迟到帧数。
  uint32_t quiet_window();
  ISerial& s_;
  frame::Parser parser_;
  int64_t last_tx_end_ = 0;
  BusStats st_;
};

}  // namespace d1t
