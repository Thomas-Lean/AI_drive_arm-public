#include "loop/controller.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdint>
#include <cmath>
#include <cstdio>

#include "common/clock.hpp"

namespace d1t {

namespace {
std::string sfmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string sfmt(const char* f, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}
bool mturn_bad(bool ok, const frame::MturnData& m);
}  // namespace

// 0x10 读多圈，读不到重试 kMturnReadRetries 次
static bool read_mturn_retry(Bus& bus, uint8_t id, frame::MturnData& m) {
  for (int k = 0; k <= cfg::kMturnReadRetries; ++k)
    if (bus.read_mturn(id, cfg::kStartupRespTimeoutMs, m)) return true;
  return false;
}

namespace {
bool mturn_bad(bool ok, const frame::MturnData& m) {
  return !ok || safety::mturn_out(float(m.angle_raw) / 10.0f);
}
}  // namespace

Controller::Controller(ISerial& serial, Mailbox& mb, CtrlOptions opt, LogFn log)
    : bus_(serial), mb_(mb), opt_(opt), log_(std::move(log)), core_(opt.period_ms) {
  for (int i = 0; i < N; ++i) ids_.push_back(cfg::kServoId[i]);
}

bool Controller::startup(std::string& err) {
  // ① 7 轴 ping，各重试 kPingRetries 次
  for (int i = 0; i < N; ++i) {
    bool ok = false;
    for (int k = 0; k <= cfg::kPingRetries && !ok; ++k) ok = bus_.ping(ids_[i], cfg::kStartupRespTimeoutMs);
    if (!ok) {
      err = sfmt("J%d ping 无响应（重试 %d 次）——检查供电与总线；未发任何运动帧", i, cfg::kPingRetries);
      return false;
    }
  }
  log("启动：7 轴 ping 通过");

  // ② 逐轴读多圈（清圈前）
  std::array<frame::MturnData, N> m{};
  std::array<bool, N> ok{};
  for (int i = 0; i < N; ++i) {
    ok[i] = read_mturn_retry(bus_, ids_[i], m[i]);
    log(ok[i] ? sfmt("启动：J%d 清圈前 0x10 = %.1f° turns=%d", i, m[i].angle_raw / 10.0, m[i].turns)
              : sfmt("启动：J%d 清圈前 0x10 读失败", i));
  }
  // ③ 只对超限（|q| ≥ 180）或读不到的轴发一次 0x11（F63：范围内的轴不发；刚上电为释放态，0x11 生效；
  //    对已锁定舵机无效也无害——【未验证】，TC-010 A 验证）
  bool any_clear = false;
  bool cleared[N] = {};  // 该轴是否实际发过 0x11（只影响日志措辞）
  for (int i = 0; i < N; ++i) {
    if (!mturn_bad(ok[i], m[i])) continue;
    bus_.send(frame::clear_mturn(ids_[i]));
    any_clear = cleared[i] = true;
    log(sfmt("启动：J%d 超限或读不到 → 0x11 清圈", i));
  }
  if (any_clear) sleep_ms(cfg::kClearToReadMs);
  // ④ 再读（发过 0x11 的轴记「清圈后」，没发的记「未清圈，复读」）
  for (int i = 0; i < N; ++i) {
    ok[i] = read_mturn_retry(bus_, ids_[i], m[i]);
    const char* tag = cleared[i] ? "清圈后" : "未清圈，复读";
    log(ok[i] ? sfmt("启动：J%d %s 0x10 = %.1f° turns=%d", i, tag, m[i].angle_raw / 10.0, m[i].turns)
              : sfmt("启动：J%d %s 0x10 读失败", i, tag));
  }
  // ⑤ J6 仍超限 → 0x18 0x10 释放 → 0x11 → 再读（J6 无重力负载，F58 处置方法），最多 kMturnClearRetries 次
  const int j6 = N - 1;
  for (int k = 0; k < cfg::kMturnClearRetries && mturn_bad(ok[j6], m[j6]); ++k) {
    log(sfmt("启动：J6 仍超限，第 %d 次 0x18 释放 → 0x11", k + 1));
    bus_.send(frame::stop(ids_[j6], frame::STOP_RELEASE, cfg::kStopPowerMw));
    sleep_ms(cfg::kReleaseToClearMs);
    bus_.send(frame::clear_mturn(ids_[j6]));
    sleep_ms(cfg::kClearToReadMs);
    ok[j6] = read_mturn_retry(bus_, ids_[j6], m[j6]);
    cleared[j6] = true;
  }
  // ⑥ 判定：J0–J5 仍超限（或读不到）→ MTURN 故障，不动任何轴，绝不自动释放 J0–J5
  bool fault = false;
  for (int i = 0; i < N; ++i) {
    if (mturn_bad(ok[i], m[i])) {
      fault = true;
      log(ok[i] ? sfmt("启动：J%d %s仍超限（%.1f° turns=%d）→ MTURN 故障。不动任何轴；请托住臂、断电重启后再启动",
                       i, cleared[i] ? "清圈后" : "复读", m[i].angle_raw / 10.0, m[i].turns)
                : sfmt("启动：J%d 0x10 读多圈 %d 次都无回包 → 按超限处理（MTURN 故障）。不动任何轴；查总线后重启", i,
                       cfg::kMturnReadRetries + 1));
    }
  }
  // ⑦ 首次同步监控，给第一周期一个反馈基准
  const int64_t mx = int64_t(cfg::kMonitorTimeoutMaxMs) * 1000000LL;
  apply_monitor(bus_.sync_monitor(ids_, INT64_MAX, mx, mx));
  core_.on_startup_done(fault);
  log(fault ? "启动完成：FAULT(MTURN)" : "启动完成：IDLE（收到命令前只读、不发运动帧）");
  return true;
}

void Controller::apply_monitor(const SyncMonitorResult& r) {
  for (int i = 0; i < N; ++i) {
    const frame::MonitorData& d = r.data[i];
    // 温度 ADC 无效（0 / ≥4096 → NaN）= 该轴本周期反馈无效，走 COMM_LOSS 计数（§3.7-3，R02）
    bool v = r.got[i] && !d.sentinel && std::isfinite(frame::temp_adc_to_c(d.temp_adc));
    fb_.valid[i] = v;
    if (!v) continue;
    fb_.have[i] = true;
    AxisSample& a = fb_.ax[i];
    a.voltage_mv = d.voltage_mv;
    a.current_ma = d.current_ma;
    a.power_mw = d.power_mw;
    a.temp_adc = d.temp_adc;
    a.temp_c = frame::temp_adc_to_c(d.temp_adc);
    a.status = d.status;
    a.q = float(d.angle_raw) / 10.0f;
    a.turns = d.turns;
    a.t_ns = r.rx_mono[i];
  }
}

// 按序下发；每帧本周期最多重试 kTxRetries 次（§3.7-5）。返回每帧结果，t_first = 第一帧成功开始发送时刻。
// 非 ESTOP 批次：每帧（含每次重试）前查信箱，ESTOP 在等就中止整批（剩余帧 SKIPPED，aborted = true），
// 主循环随即切到急停序列（R05、V2-02）。已发出的停止帧不重发；没发出的停止帧由 safety 标为待重发，
// 急停序列本身也对全部轴发 0x18 0x11。ESTOP 批次本身不被打断。
std::vector<safety::OpResult> Controller::exec(const safety::Decision& d, int64_t& t_first, bool& aborted) {
  std::vector<safety::OpResult> res;
  t_first = 0;
  aborted = false;
  auto preempt = [&]() { return !d.estop && mb_.estop_pending(); };
  for (size_t k = 0; k < d.ops.size(); ++k) {
    const auto& op = d.ops[k];
    if (aborted || preempt()) {
      aborted = true;
      res.push_back(safety::OpResult::SKIPPED);
      continue;
    }
    frame::Bytes f;
    switch (op.kind) {
      case safety::BusOp::MOVE: {
        std::vector<frame::MoveItem> items;
        for (int i = 0; i < N; ++i)
          if (d.move_axis[i]) items.push_back({ids_[i], d.move_q[i], d.move_time_ms, d.move_power[i]});
        f = frame::sync_set_angle(items);
        break;
      }
      case safety::BusOp::STOP:
        f = frame::stop(ids_[op.axis], op.mode, cfg::kStopPowerMw);
        break;
      case safety::BusOp::CLEAR:  // 0x11 只在释放态有效；释放→清圈的等待由 safety 分周期保证（不在这里睡）
        f = frame::clear_mturn(ids_[op.axis]);
        break;
    }
    int64_t t = 0;
    for (int a = 0; a <= cfg::kTxRetries && t == 0; ++a) {
      if (a > 0 && preempt()) {  // 重试前 ESTOP 到了：不再重试，切急停
        aborted = true;
        break;
      }
      t = bus_.send(f);
      if (t == 0) ++stats_.tx_retries;
    }
    res.push_back(t ? safety::OpResult::OK : aborted ? safety::OpResult::SKIPPED : safety::OpResult::FAIL);
    if (t && !t_first) t_first = t;
  }
  return res;
}

void Controller::run(const std::atomic<bool>& stop) {
  const int64_t P = int64_t(opt_.period_ms) * 1000000LL;
  int64_t sched = now_mono_ns();
  while (!stop.load() && !mb_.is_shutdown()) {
    // 1) 取命令 → safety
    Cmd c;
    bool has = mb_.take_cmd(c);
    int64_t now = now_mono_ns();
    safety::Decision d = core_.step(has ? &c : nullptr, fb_, now);
    bool stale, rejected;
    mb_.take_counts(stale, rejected);
    if (stale) d.warn |= bits::CMD_STALE;
    if (rejected) d.warn |= bits::CMD_REJECTED;

    // 2)+3) 下发（冻结帧 / 0x19+0x08 / 0x18 / 0x11），帧间隔由 bus 保证；按写出结果提交状态
    int64_t t_first = 0;
    bool aborted = false;
    std::vector<safety::OpResult> res = exec(d, t_first, aborted);
    core_.commit(d, res, now_mono_ns());
    if (aborted) {
      // ESTOP 抢占（V2-02）：不读回、不发布，立即回到循环开头取 ESTOP 执行冻结帧 + 全轴保持。
      // 反馈沿用本周期开头那份（与被打断的决策同一份数据）。
      ++stats_.estop_preempts;
      for (const auto& e : d.events) log(sfmt("[seq %u] %s", seq_, e.c_str()));
      log("ESTOP 抢占：中止本周期剩余帧，立即执行急停序列");
      sched = now_mono_ns();
      continue;
    }
    if (d.estop && t_first) {
      double ms = double(t_first - c.rx_mono_ns) / 1e6;
      stats_.estop_latency_ms.push_back(ms);
      log(sfmt("ESTOP：信箱收到 → 冻结帧发出 %.2f ms", ms));
    }

    // 4) 0x19+0x16 同步读回；超时 = 周期剩余 − 5 ms，钳到 [20, 40] ms（从请求实际发出算）
    int64_t next = sched + P;
    SyncMonitorResult r = bus_.sync_monitor(ids_, next - int64_t(cfg::kMonitorTimeoutMarginMs) * 1000000LL,
                                            int64_t(cfg::kMonitorTimeoutMinMs) * 1000000LL,
                                            int64_t(cfg::kMonitorTimeoutMaxMs) * 1000000LL);
    apply_monitor(r);

    // 5) 组装并发布 ArmState（缺回包的轴 valid 位清零、数值为上一次有效值，不等）
    StateMsg s;
    s.seq = ++seq_;
    s.stamp_ns = r.req_tx_start_real;
    s.sample_span_us = r.last_rx_mono ? uint32_t((r.last_rx_mono - r.req_tx_start_mono) / 1000) : 0;
    s.ctrl_state = uint8_t(d.state);
    s.cmd_seq_ack = d.ack_seq;
    s.cmd_stamp_ns = d.ack_stamp_ns;
    s.cmd_rx_ns = d.ack_rx_ns;
    for (int i = 0; i < N; ++i) {
      if (fb_.valid[i]) s.valid_mask |= uint8_t(1u << i);
      s.q[i] = fb_.ax[i].q;
      s.q_target[i] = d.q_target[i];
      s.current_ma[i] = fb_.ax[i].current_ma;
      s.voltage_mv[i] = fb_.ax[i].voltage_mv;
      s.temp_c[i] = fb_.ax[i].temp_c;
      s.status[i] = fb_.ax[i].status;
      s.turns[i] = fb_.ax[i].turns;
    }
    s.fault = d.fault;
    s.warn = d.warn;
    s.overruns = stats_.overruns;
    mb_.push_state(s);
    stats_.last_seq = s.seq;
    ++stats_.cycles;
    for (const auto& e : d.events) log(sfmt("[seq %u] %s", s.seq, e.c_str()));

    // 6) 睡到下一个绝对时刻；超时则 overruns+1、不追帧、从当前时刻重新对齐；ESTOP 提前唤醒
    now = now_mono_ns();
    if (now > next) {
      ++stats_.overruns;
      if (opt_.verbose)
        log(sfmt("[seq %u] 周期超时 %.2f ms（下发帧 %zu，监控 %s）", s.seq, double(now - next) / 1e6, d.ops.size(),
                 s.valid_mask == 0x7F ? "收齐" : "缺包"));
      next = now;
    }
    if (mb_.wait_until(next)) {
      ++stats_.estop_wakeups;
      sched = now_mono_ns();  // ESTOP：立即开始新周期并重新对齐
    } else {
      sched = next;
    }
  }
  int64_t t_first = 0;
  bool aborted = false;
  safety::Decision d = core_.on_shutdown();
  d.estop = true;  // 退出前的保持序列不被信箱里的 ESTOP 打断（本身就是全轴保持）
  std::vector<safety::OpResult> res = exec(d, t_first, aborted);
  size_t bad = 0;
  for (auto r : res) bad += r != safety::OpResult::OK;
  log(bad ? sfmt("主循环退出：全轴 0x18 0x11 保持，其中 %zu 帧写失败——请用物理急停 / 断电", bad)
          : "主循环退出：全轴 0x18 0x11 保持");
}

}  // namespace d1t
