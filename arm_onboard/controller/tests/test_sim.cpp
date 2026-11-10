// test_sim.cpp —— --sim 集成测试（任务书 §4 T3 验收 3）：真主循环 + FakeSerial + 假 DDS 层
//
//   test_sim --full   60 s 流式（验收用）
//   test_sim --quick  5 s 流式（ctest 默认）
// 场景：启动多圈（正常 / J6 自动清 / J0–J5 拒绝）、流式 seq 连续、丢包注入（valid_mask、COMM_LOSS）、
//       看门狗、ESTOP 唤醒时延 < 1 周期。
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>

#include "check.hpp"
#include "common/clock.hpp"
#include "sim/fake_client.hpp"
#include "loop/controller.hpp"

using namespace d1t;

static const int P = cfg::kPeriodMsDefault;

struct Rig {
  FakeSerial fs;
  Mailbox mb;
  Controller ctrl;
  FakeClient cl;
  std::atomic<bool> stop{false};
  std::thread th;
  bool verbose;
  explicit Rig(bool v = false)
      : ctrl(fs, mb, CtrlOptions{P, v},
             [v](const std::string& s) {
               if (v) std::printf("    [ctrl] %s\n", s.c_str());
             }),
        cl(mb, P),
        verbose(v) {}
  bool boot() {
    std::string err;
    bool ok = ctrl.startup(err);
    if (!ok) std::printf("    startup err: %s\n", err.c_str());
    return ok;
  }
  void run() { th = std::thread([this] { ctrl.run(stop); }); }
  void halt() {
    cl.stop();
    stop = true;
    mb.shutdown();
    if (th.joinable()) th.join();
  }
  ~Rig() { halt(); }
  StateMsg last() {
    auto v = cl.states();
    return v.empty() ? StateMsg{} : v.back();
  }
  // 等到满足条件或超时
  template <class F>
  bool wait_for(F f, int ms) {
    int64_t end = now_mono_ns() + int64_t(ms) * 1000000LL;
    while (now_mono_ns() < end) {
      auto v = cl.states();
      if (!v.empty() && f(v.back())) return true;
      sleep_ms(5);
    }
    return false;
  }
};

// warn 不锁存，只在收到命令的那个周期置位：看最近 4 帧
static bool recent_warn(FakeClient& cl, uint32_t bit) {
  auto v = cl.states();
  bool any = false;
  for (size_t k = v.size() > 4 ? v.size() - 4 : 0; k < v.size(); ++k) any |= (v[k].warn & bit) != 0;
  return any;
}

static uint32_t seq_gaps(const std::vector<StateMsg>& v) {
  uint32_t g = 0;
  for (size_t k = 1; k < v.size(); ++k) g += v[k].seq - v[k - 1].seq - 1;
  return g;
}

int main(int argc, char** argv) {
  bool full = argc > 1 && std::strcmp(argv[1], "--full") == 0;
  bool verbose = argc > 2 && std::strcmp(argv[2], "-v") == 0;
  int stream_sec = full ? 60 : 5;

  // ---------- 1. 启动：全部在范围内 ----------
  std::printf("== 1 启动（正常）\n");
  {
    Rig r(verbose);
    CHECK(r.boot());
    CHECK(r.ctrl.core().state() == CtrlState::IDLE);
    for (int i = 0; i < N; ++i) {
      auto v = r.fs.view(i);
      CHECK(v.clear_count == 0);  // 全在范围内（含 J1 −60° turn=−1，F63）：不发 0x11
      CHECK(v.stop_count == 0 && v.move_count == 0);
    }
  }

  // ---------- 1b. 启动：J3 丢 1 个回包（ping 与 0x10 都有重试）→ 仍正常 ----------
  std::printf("== 1b 启动（J3 丢包 2 次）\n");
  {
    Rig r(verbose);
    r.fs.inject_drop_next(3, 2);
    CHECK(r.boot());
    CHECK(r.ctrl.core().state() == CtrlState::IDLE);
  }
  // J3 完全离线 → 启动失败，且没有发出任何运动 / 停止帧
  {
    Rig r(verbose);
    r.fs.inject_offline(3, true);
    CHECK(!r.boot());
    for (int i = 0; i < N; ++i) CHECK(r.fs.view(i).stop_count == 0 && r.fs.view(i).move_count == 0);
  }

  // ---------- 2. 启动：J6 已锁定且多圈 388.6° → 0x11 无效 → 释放再清 ----------
  std::printf("== 2 启动（J6 多圈 388.6°，锁定态）\n");
  {
    Rig r(verbose);
    r.fs.inject_mturn(6, 388.6f);
    r.fs.inject_locked(6);
    CHECK(r.boot());
    auto v = r.fs.view(6);
    CHECK_NEAR(v.pos_deg, 28.6, 0.05);
    CHECK(v.stop_count == 1 && v.clear_count == 2);
    CHECK(r.ctrl.core().state() == CtrlState::IDLE);
    for (int i = 0; i < N - 1; ++i) CHECK(r.fs.view(i).stop_count == 0);
  }

  // ---------- 3. 启动：J1 已锁定且多圈 → MTURN，不动任何轴，绝不释放 ----------
  std::printf("== 3 启动（J1 多圈 -450°，锁定态）\n");
  {
    Rig r(verbose);
    r.fs.inject_mturn(1, -450.0f);
    r.fs.inject_locked(1);
    CHECK(r.boot());
    CHECK(r.ctrl.core().state() == CtrlState::FAULT && (r.ctrl.core().latched() & bits::MTURN));
    r.cl.start(true, 5.0f, 0.2f, 0x7F, P);
    r.run();
    sleep_ms(500);
    StateMsg s = r.last();
    CHECK(s.ctrl_state == uint8_t(CtrlState::FAULT) && (s.fault & bits::MTURN));
    CHECK(recent_warn(r.cl, bits::CMD_REJECTED));
    for (int i = 0; i < N; ++i) {
      auto v = r.fs.view(i);
      CHECK(v.move_count == 0 && v.stop_count == 0);  // 没有任何运动 / 停止帧
    }
    CHECK(r.fs.view(1).mode == FakeSerial::SMode::HOLD);  // 仍是原来的锁定态，没被释放
  }

  // ---------- 4. 流式 stream_sec 秒：seq 无缺号 ----------
  std::printf("== 4 流式 %d s（J0..J6 ±5° 0.2 Hz）\n", stream_sec);
  {
    Rig r(verbose);
    CHECK(r.boot());
    r.cl.start(true, 5.0f, 0.2f, 0x7F, P);
    r.run();
    sleep_ms(stream_sec * 1000);
    r.halt();
    auto v = r.cl.states();
    uint32_t gaps = seq_gaps(v);
    size_t streaming = 0, full_valid = 0, stamp_back = 0;
    uint32_t span_max = 0;
    for (size_t k = 0; k < v.size(); ++k) {
      if (v[k].ctrl_state == uint8_t(CtrlState::STREAMING)) ++streaming;
      if (v[k].valid_mask == 0x7F) ++full_valid;
      span_max = std::max(span_max, v[k].sample_span_us);
      if (k && v[k].stamp_ns <= v[k - 1].stamp_ns) ++stamp_back;
    }
    CtrlStats st = r.ctrl.stats();
    std::printf("   帧数 %zu（期望≈%d）seq %u..%u 缺号 %u；STREAMING %zu；7 轴齐 %zu；stamp 倒退 %zu；"
                "sample_span max %u us；overruns %u；fault 末帧 0x%x\n",
                v.size(), stream_sec * 1000 / P, v.empty() ? 0 : v.front().seq, v.empty() ? 0 : v.back().seq, gaps,
                streaming, full_valid, stamp_back, span_max, st.overruns, v.empty() ? 0 : v.back().fault);
    CHECK(!v.empty() && v.front().seq == 1);
    CHECK(gaps == 0);
    CHECK(v.size() >= size_t(stream_sec * 1000 / P * 98 / 100));
    CHECK(streaming >= v.size() * 95 / 100);
    CHECK(full_valid == v.size());
    CHECK(stamp_back == 0);
    CHECK(v.back().fault == 0);
    CHECK(v.back().cmd_seq_ack > 0 && v.back().cmd_rx_ns > 0);
    // 跟随：J0 实测跟着目标走（仿真一阶跟随）
    float maxerr = 0;
    for (size_t k = v.size() / 2; k < v.size(); ++k) maxerr = std::max(maxerr, std::fabs(v[k].q[0] - v[k].q_target[0]));
    std::printf("   后半段 J0 |q - q_target| max %.2f°\n", maxerr);
    CHECK(maxerr < 1.0f);
  }

  // ---------- 5. 丢包注入 ----------
  std::printf("== 5 丢包注入\n");
  {
    Rig r(verbose);
    CHECK(r.boot());
    r.cl.start(true, 5.0f, 0.2f, 0x7F, P);
    r.run();
    sleep_ms(500);
    size_t n0 = r.cl.n_states();
    r.fs.inject_drop_next(4, 2);  // J4 连丢 2 次：valid_mask 清零 2 帧，不判 COMM_LOSS
    sleep_ms(500);
    auto v = r.cl.states();
    int miss4 = 0;
    for (size_t k = n0; k < v.size(); ++k) miss4 += !(v[k].valid_mask & (1u << 4));
    std::printf("   J4 连丢 2：valid_mask bit4 清零帧 %d，fault 0x%x\n", miss4, v.back().fault);
    CHECK(miss4 == 2);
    CHECK(!(v.back().fault & bits::COMM_LOSS));
    // 丢包时 q 保持上一次有效值
    for (size_t k = n0 + 1; k < v.size(); ++k)
      if (!(v[k].valid_mask & (1u << 4))) CHECK(v[k].q[4] == v[k - 1].q[4]);

    int stops_before = r.fs.view(0).stop_count;
    r.fs.inject_drop_next(4, 50);  // 连丢 ≥ kCommLossCycles → COMM_LOSS
    bool got = r.wait_for([](const StateMsg& s) { return (s.fault & bits::COMM_LOSS) != 0; }, 1000);
    CHECK(got);
    StateMsg s = r.last();
    std::printf("   J4 连丢 50：COMM_LOSS=%d ctrl_state=%u\n", got, s.ctrl_state);
    CHECK(s.ctrl_state == uint8_t(CtrlState::FAULT));
    sleep_ms(200);
    CHECK(r.fs.view(0).stop_count == stops_before + 1);  // 全轴保持
    for (int i = 0; i < N - 1; ++i) CHECK(r.fs.view(i).mode == FakeSerial::SMode::HOLD);  // 不释放
  }
  // 随机丢包 5%：只报 valid_mask，seq 仍连续
  {
    Rig r(verbose);
    CHECK(r.boot());
    for (int i = 0; i < N; ++i) r.fs.inject_drop_prob(i, 0.05);
    r.fs.inject_corrupt_prob(2, 0.05);
    r.cl.start(true, 5.0f, 0.2f, 0x7F, P);
    r.run();
    sleep_ms(3000);
    r.halt();
    auto v = r.cl.states();
    std::array<int, N> ok{};
    for (auto& s : v)
      for (int i = 0; i < N; ++i) ok[i] += (s.valid_mask >> i) & 1;
    std::printf("   5%% 丢包 + J2 5%% 坏帧：%zu 帧，各轴有效率", v.size());
    for (int i = 0; i < N; ++i) std::printf(" J%d %.1f%%", i, 100.0 * ok[i] / double(v.size()));
    std::printf("；坏帧 %llu；seq 缺号 %u\n", (unsigned long long)r.ctrl.bus_stats().rx_bad, seq_gaps(v));
    CHECK(seq_gaps(v) == 0);
    CHECK(r.ctrl.bus_stats().rx_bad > 0);
    for (int i = 0; i < N; ++i) CHECK(ok[i] < int(v.size()));
  }

  // ---------- 6. 看门狗 ----------
  std::printf("== 6 看门狗\n");
  {
    Rig r(verbose);
    CHECK(r.boot());
    r.cl.start(true, 5.0f, 0.2f, 0x01, P);
    r.run();
    sleep_ms(500);
    int64_t t_cut = now_mono_ns();
    r.cl.set_streaming(false);  // 上位机停发
    bool tripped = r.wait_for([](const StateMsg& s) { return (s.fault & bits::WATCHDOG) != 0; }, 1000);
    double ms = double(now_mono_ns() - t_cut) / 1e6;
    std::printf("   停发 → WATCHDOG 置位 %.0f ms（WATCHDOG_MS=%d）\n", ms, cfg::kWatchdogMs);
    CHECK(tripped);
    CHECK(r.last().ctrl_state == uint8_t(CtrlState::HOLD));
    for (int i = 0; i < N; ++i) CHECK(r.fs.view(i).mode == FakeSerial::SMode::HOLD);
    r.cl.set_streaming(true);  // 正弦继续：目标在 5° 内 → 自动恢复
    CHECK(r.wait_for([](const StateMsg& s) { return s.ctrl_state == uint8_t(CtrlState::STREAMING); }, 1000));
  }

  // ---------- 7. ESTOP 唤醒时延 < 1 周期 ----------
  std::printf("== 7 ESTOP ×20\n");
  {
    Rig r(verbose);
    CHECK(r.boot());
    r.cl.start(true, 5.0f, 0.2f, 0x7F, P);
    r.run();
    sleep_ms(300);
    std::vector<double> stop_all_ms;
    for (int k = 0; k < 20; ++k) {
      sleep_ms(37 + 13 * (k % 4));  // 打在周期内不同相位
      r.cl.set_streaming(false);
      int64_t t_put = r.cl.send(Mode::ESTOP);
      bool latched = r.wait_for([](const StateMsg& s) { return s.ctrl_state == uint8_t(CtrlState::ESTOP); }, 500);
      CHECK(latched);
      // 冻结帧在 put 之后发出
      CHECK(r.fs.last_sync_move_tx_end_ns() > t_put);
      stop_all_ms.push_back(double(r.fs.last_stop_tx_end_ns() - t_put) / 1e6);
      for (int i = 0; i < N; ++i) CHECK(r.fs.view(i).mode == FakeSerial::SMode::HOLD);
      sleep_ms(120);
      // 锁存期间流式被拒
      r.cl.set_streaming(true);
      sleep_ms(120);
      CHECK(r.last().ctrl_state == uint8_t(CtrlState::ESTOP));
      CHECK(recent_warn(r.cl, bits::CMD_REJECTED));
      // 信箱只存最新一条（任务书 §2）：RESET 与 STREAM 同周期到达时 RESET 会被覆盖，所以先停流式再 RESET
      r.cl.set_streaming(false);
      sleep_ms(60);
      r.cl.send(Mode::RESET);
      CHECK(r.wait_for([](const StateMsg& s) { return s.ctrl_state == uint8_t(CtrlState::HOLD); }, 500));
      r.cl.set_streaming(true);
      CHECK(r.wait_for([](const StateMsg& s) { return s.ctrl_state == uint8_t(CtrlState::STREAMING); }, 500));
    }
    CtrlStats st = r.ctrl.stats();
    double mx = 0, sum = 0;
    for (double x : st.estop_latency_ms) {
      mx = std::max(mx, x);
      sum += x;
    }
    double mx_all = *std::max_element(stop_all_ms.begin(), stop_all_ms.end());
    std::printf("   收到 → 冻结帧发出：n=%zu 平均 %.2f ms，最大 %.2f ms（周期 %d ms）；收到 → 最后一帧 0x18 发完 最大 %.1f ms；"
                "ESTOP 提前唤醒 %llu 次\n",
                st.estop_latency_ms.size(), st.estop_latency_ms.empty() ? 0 : sum / st.estop_latency_ms.size(), mx, P,
                mx_all, (unsigned long long)st.estop_wakeups);
    CHECK(st.estop_latency_ms.size() == 20);
    CHECK(mx < P);
  }

  return chk::report("test_sim");
}
