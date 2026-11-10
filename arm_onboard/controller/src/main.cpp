// main.cpp —— d1t_arm_ctrl 入口
//
// 真机：sudo ./d1t_arm_ctrl                    （先 systemctl stop 原 4 个服务；推荐用 deploy/arm_start.sh）
// 仿真：./d1t_arm_ctrl --sim [--sim-drop 3:0.05] [--fake-client 60]
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "bus/serial.hpp"
#include "common/clock.hpp"
#include "sim/fake_client.hpp"
#include "loop/controller.hpp"
#ifdef D1T_WITH_DDS
#include "dds_io/dds_io.hpp"
#endif

using namespace d1t;

static std::atomic<bool> g_stop{false};
static void on_sig(int) { g_stop = true; }

static void usage() {
  std::printf(
      "用法：d1t_arm_ctrl [选项]\n"
      "  --sim                    用 FakeSerial 模拟 7 舵机（无硬件）\n"
      "  --port PATH              串口（默认 %s）\n"
      "  --period-ms N            周期 %d–%d ms（默认 %d）\n"
      "  --sim-drop J:P           仿真：Jj 回包丢包概率 P（可多次）\n"
      "  --sim-corrupt J:P        仿真：Jj 回包校验和写错概率 P\n"
      "  --sim-status J:HEX       仿真：Jj 状态字节叠加位\n"
      "  --sim-temp J:C           仿真：Jj 温度 ℃\n"
      "  --sim-mturn J:DEG        仿真：启动前 Jj 多圈累计角\n"
      "  --fake-client SEC        不用 DDS，进程内假上位机发 J0/J3 ±5° 0.2 Hz 正弦 SEC 秒后退出并打印统计\n"
      "  -v                       打印每条事件\n",
      cfg::kSerialPort, cfg::kPeriodMsMin, cfg::kPeriodMsMax, cfg::kPeriodMsDefault);
}

static bool parse_jv(const char* s, int& j, double& v) {
  return std::sscanf(s, "%d:%lf", &j, &v) == 2 && j >= 0 && j < N;
}

int main(int argc, char** argv) {
  bool sim = false;
  std::string port = cfg::kSerialPort;
  CtrlOptions opt;
  double fake_sec = 0;
  auto fake = std::make_unique<FakeSerial>();
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s 缺参数\n", name);
        std::exit(2);
      }
      return argv[++i];
    };
    int j;
    double v;
    if (a == "--sim") {
      sim = true;
    } else if (a == "--port") {
      port = need("--port");
    } else if (a == "--period-ms") {
      opt.period_ms = std::atoi(need("--period-ms"));
    } else if (a == "--sim-drop" && parse_jv(need("--sim-drop"), j, v)) {
      fake->inject_drop_prob(j, v);
    } else if (a == "--sim-corrupt" && parse_jv(need("--sim-corrupt"), j, v)) {
      fake->inject_corrupt_prob(j, v);
    } else if (a == "--sim-status") {
      unsigned st;
      if (std::sscanf(need("--sim-status"), "%d:%x", &j, &st) != 2 || j < 0 || j >= N) return usage(), 2;
      fake->inject_status(j, uint8_t(st));
    } else if (a == "--sim-temp" && parse_jv(need("--sim-temp"), j, v)) {
      fake->inject_temp_c(j, float(v));
    } else if (a == "--sim-mturn" && parse_jv(need("--sim-mturn"), j, v)) {
      fake->inject_mturn(j, float(v));
    } else if (a == "--fake-client") {
      fake_sec = std::atof(need("--fake-client"));
    } else if (a == "-v") {
      opt.verbose = true;
    } else {
      usage();
      return 2;
    }
  }
  if (opt.period_ms < cfg::kPeriodMsMin || opt.period_ms > cfg::kPeriodMsMax) {
    std::fprintf(stderr, "--period-ms 须在 %d–%d\n", cfg::kPeriodMsMin, cfg::kPeriodMsMax);
    return 2;
  }
#ifndef D1T_WITH_DDS
  if (fake_sec <= 0) {
    std::fprintf(stderr, "本次构建未编入 DDS（缺 CycloneDDS-CXX 或 idl/generated/），只能配合 --fake-client 运行\n");
    return 2;
  }
#endif

  std::signal(SIGINT, on_sig);
  std::signal(SIGTERM, on_sig);
  std::fprintf(stderr, "[d1t] d1t_arm_ctrl v%s\n", cfg::kCtrlVersion);

  std::unique_ptr<PosixSerial> real;
  ISerial* serial = nullptr;
  if (sim) {
    serial = fake.get();
    std::fprintf(stderr, "[d1t] --sim：FakeSerial 7 舵机\n");
  } else {
    std::string msg;
    if (!preflight_port(port, msg)) {
      std::fprintf(stderr, "[d1t] 占用检查不通过：%s\n", msg.c_str());
      return 3;
    }
    std::fprintf(stderr, "[d1t] %s\n", msg.c_str());
    real = std::make_unique<PosixSerial>();
    if (!real->open(port, cfg::kBaud, msg)) {
      std::fprintf(stderr, "[d1t] %s\n", msg.c_str());
      return 3;
    }
    serial = real.get();
  }

  Mailbox mb;
  Controller ctrl(*serial, mb, opt, [&](const std::string& s) { std::fprintf(stderr, "[d1t] %s\n", s.c_str()); });
  std::string err;
  if (!ctrl.startup(err)) {
    std::fprintf(stderr, "[d1t] 启动失败：%s\n", err.c_str());
    return 4;
  }

#ifdef D1T_WITH_DDS
  std::unique_ptr<DdsIo> dds;
#endif
  std::unique_ptr<FakeClient> fc;
  if (fake_sec > 0) {
    fc = std::make_unique<FakeClient>(mb, opt.period_ms);
    fc->start(true, 5.0f, 0.2f, (1u << 0) | (1u << 3), uint16_t(opt.period_ms));
  } else {
#ifdef D1T_WITH_DDS
    dds = std::make_unique<DdsIo>(mb);
    if (!dds->start(err)) {
      std::fprintf(stderr, "[d1t] DDS 启动失败：%s\n", err.c_str());
      return 5;
    }
    std::fprintf(stderr, "[d1t] DDS domain %d：收 %s，发 %s\n", cfg::kDdsDomain, cfg::kTopicCmd, cfg::kTopicState);
#endif
  }

  std::thread loop([&] { ctrl.run(g_stop); });
  int64_t t_end = fake_sec > 0 ? now_mono_ns() + int64_t(fake_sec * 1e9) : 0;
  while (!g_stop) {
    sleep_ms(100);
    if (t_end && now_mono_ns() >= t_end) g_stop = true;
  }
  mb.shutdown();
  loop.join();
  if (fc) fc->stop();
#ifdef D1T_WITH_DDS
  if (dds) dds->stop();
#endif

  CtrlStats st = ctrl.stats();
  std::fprintf(stderr, "[d1t] 结束：cycles=%llu overruns=%u last_seq=%u bus tx=%llu rx_ok=%llu rx_bad=%llu\n",
               (unsigned long long)st.cycles, st.overruns, st.last_seq,
               (unsigned long long)ctrl.bus_stats().tx_frames, (unsigned long long)ctrl.bus_stats().rx_ok,
               (unsigned long long)ctrl.bus_stats().rx_bad);
  if (fc) {
    auto ss = fc->states();
    uint32_t gaps = 0;
    for (size_t k = 1; k < ss.size(); ++k) gaps += ss[k].seq - ss[k - 1].seq - 1;
    std::fprintf(stderr, "[d1t] 假上位机：发 %u 条，收 ArmState %zu 帧，seq 缺号 %u\n", fc->sent(), ss.size(), gaps);
  }
  // 全轴保持已发、统计已打印。不走 DDS 析构：Cyclone 0.10.2 在 macOS 上实测会卡在 rtps_stop 等收包线程，
  // 进程不退就一直占着串口，回滚（systemctl start 原服务）会冲突。直接退出，由内核关串口与套接字。
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(0);
}
