// controller.hpp —— 单线程固定周期主循环 + 启动流程（任务书 §3.1、§3.3）
//
// 本类是唯一碰串口的地方（经 Bus）；与 dds_io 只通过 Mailbox 交互。
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "bus/bus.hpp"
#include "loop/mailbox.hpp"
#include "safety/safety.hpp"

namespace d1t {

struct CtrlOptions {
  int period_ms = cfg::kPeriodMsDefault;
  bool verbose = false;
};

struct CtrlStats {
  uint64_t cycles = 0;
  uint32_t overruns = 0;
  uint64_t estop_wakeups = 0;
  std::vector<double> estop_latency_ms;  // 信箱收到 ESTOP → 冻结帧开始发送
  uint32_t last_seq = 0;
  uint64_t tx_retries = 0;  // 写失败后的本周期重试次数
  uint64_t estop_preempts = 0;  // 下发途中被 ESTOP 抢占的周期数
};

class Controller {
 public:
  using LogFn = std::function<void(const std::string&)>;

  Controller(ISerial& serial, Mailbox& mb, CtrlOptions opt, LogFn log);

  // 启动流程：ping → 读多圈 → 7 轴 0x11 → 再读 → J6 仍超限则释放再清 → J0–J5 仍超限 = MTURN 故障。
  // 返回 false = 不能继续（有轴 ping 不通），此时未发任何运动 / 停止帧。
  bool startup(std::string& err);

  // 主循环，直到 stop 为 true 或信箱 shutdown；退出前全轴保持（Core::on_shutdown）
  void run(const std::atomic<bool>& stop);

  CtrlStats stats() const { return stats_; }
  safety::Core& core() { return core_; }
  const BusStats& bus_stats() const { return bus_.stats(); }

 private:
  std::vector<safety::OpResult> exec(const safety::Decision& d, int64_t& t_first, bool& aborted);
  void apply_monitor(const SyncMonitorResult& r);
  void log(const std::string& s) { if (log_) log_(s); }

  Bus bus_;
  Mailbox& mb_;
  CtrlOptions opt_;
  LogFn log_;
  safety::Core core_;
  Feedback fb_;
  uint32_t seq_ = 0;
  CtrlStats stats_;
  std::vector<uint8_t> ids_;
};

}  // namespace d1t
