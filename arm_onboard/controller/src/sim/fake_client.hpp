// fake_client.hpp —— 假 DDS 层：进程内模拟上位机（T2 生成代码到位前的测试替身）
//
// 扮演 dds_io 线程：只通过 Mailbox 放命令、取 ArmState。可按周期发正弦流式 STREAM，
// 也可单发 HOLD / ESTOP / RESET 等。录下收到的全部 ArmState 供检查。
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "loop/mailbox.hpp"

namespace d1t {

class FakeClient {
 public:
  FakeClient(Mailbox& mb, int period_ms) : mb_(mb), period_ms_(period_ms) {}
  ~FakeClient() { stop(); }

  // 开始后台线程：收 ArmState；stream=true 时每周期发正弦 STREAM（以第一帧实测 q 为中心）
  void start(bool stream, float amp_deg, float freq_hz, uint8_t axes_mask, uint16_t dt_ms);
  void set_streaming(bool on) { streaming_ = on; }
  void stop();

  // 单发一条命令（调用线程即「DDS 线程」）；返回 put 时刻（mono）
  int64_t send(Mode m, uint8_t axis_mask = 0);

  std::vector<StateMsg> states();
  size_t n_states();
  uint32_t sent() const { return seq_.load(); }

 private:
  void run();

  Mailbox& mb_;
  int period_ms_;
  std::thread th_;
  std::atomic<bool> run_{false};
  std::atomic<bool> streaming_{false};
  std::atomic<uint32_t> seq_{0};
  float amp_ = 0, freq_ = 0;
  uint8_t mask_ = 0;
  uint16_t dt_ms_ = 0;
  std::mutex mu_;
  std::vector<StateMsg> states_;
  bool have_base_ = false;
  std::array<float, N> base_{};
};

}  // namespace d1t
