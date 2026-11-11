#include "safety/safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "config.hpp"

namespace d1t {
namespace safety {

namespace {
constexpr uint8_t kRelease = 0x10, kHold = 0x11, kDamp = 0x12;  // 0x18 模式（PROTOCOL_NOTES 0x18 节）

std::string fmt(const char* f, double a = 0, double b = 0, double c = 0) {
  char buf[160];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}
std::string hex(const char* prefix, uint32_t v) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "%s0x%04x", prefix, unsigned(v));
  return buf;
}
}  // namespace

// ================= 纯函数 =================

Clamp clamp_limit(int i, float q) {
  float lo = std::max(-cfg::kMturnLimitDeg, cfg::kJointLimit[i].lo);
  float hi = std::min(cfg::kMturnLimitDeg, cfg::kJointLimit[i].hi);
  if (q < lo) return {lo, true};
  if (q > hi) return {hi, true};
  return {q, false};
}

float max_step_deg(int i, int period_ms) { return cfg::kAxisSpeedDps[i] * float(period_ms) / 1000.0f; }

Clamp clamp_step(int i, float q_cmd, float q_prev, int period_ms) {
  float m = max_step_deg(i, period_ms);
  float d = q_cmd - q_prev;
  if (d > m) return {q_prev + m, true};
  if (d < -m) return {q_prev - m, true};
  return {q_cmd, false};
}

bool mturn_out(float q) { return std::fabs(q) >= cfg::kMturnLimitDeg; }

bool out_of_limit(int i, float q) { return q < cfg::kJointLimit[i].lo || q > cfg::kJointLimit[i].hi; }

StatusClass classify_status(int i, uint8_t st) {
  StatusClass r{false, false};
  if (st & cfg::kStatusFaultAnyAxis) r.fault = true;
  if (st & cfg::kStatusFaultArmOnly) {
    if (i == N - 1)
      r.j6_warn = true;  // J6 bit2/bit6：夹持接触时预期出现（F60），只告警
    else
      r.fault = true;
  }
  return r;
}

J6Temp j6_temp_level(float c) {
  if (!std::isfinite(c)) return J6Temp::UNKNOWN;
  if (c >= cfg::kJ6TempReleaseC) return J6Temp::RELEASE;
  if (c >= cfg::kJ6TempWarnC) return J6Temp::HOT;
  return J6Temp::OK;
}

bool cmd_valid(const Cmd& c) {
  if (c.mode > uint8_t(Mode::RESET)) return false;
  Mode m = Mode(c.mode);
  if (m == Mode::RELEASE || m == Mode::DAMP) {
    if (c.axis_mask == 0 || (c.axis_mask & ~uint8_t((1u << N) - 1))) return false;
  }
  if (m == Mode::STREAM) {
    for (float q : c.q)
      if (!std::isfinite(q)) return false;
  }
  return true;
}

uint16_t clamp_dt(uint16_t dt, int period_ms) {
  int lo = period_ms, hi = 3 * period_ms;
  int v = std::min(std::max(int(dt), lo), hi);
  return uint16_t(v);
}

uint16_t clamp_grip_power(uint16_t mw) { return std::min(mw, cfg::kGripPowerMaxMw); }

// ================= 状态机 =================

void Core::on_startup_done(bool mturn_fault) {
  if (mturn_fault) {
    latched_ |= bits::MTURN;
    st_ = CtrlState::FAULT;
    startup_noact_ = true;
  } else {
    st_ = CtrlState::IDLE;
  }
}

bool Core::j6_excluded() const { return j6_pending_ || (latched_ & bits::J6_THERMAL); }

uint8_t Core::stop_mode_of(int i) const {
  return ax_[i] == Ax::RELEASED ? kRelease : ax_[i] == Ax::DAMPED ? kDamp : kHold;
}

// 对第 i 轴发 0x18 mode。同一 Decision 里每轴最多一帧停止帧（后来的覆盖模式）。
void Core::want_stop(Decision& d, int i, uint8_t mode, const Feedback& fb) {
  ax_[i] = mode == kRelease ? Ax::RELEASED : mode == kDamp ? Ax::DAMPED : Ax::HOLD;
  for (auto& o : d.ops) {
    if (o.kind == BusOp::STOP && o.axis == i) {
      o.mode = mode;
      o.q_meas = fb.ax[i].q;
      return;
    }
  }
  d.ops.push_back({BusOp::STOP, uint8_t(i), mode, fb.ax[i].q});
}

// 全轴 0x18 0x11 保持。force=false 时已确认写出的 HOLD 轴不重发（省总线时间）。
// J6 过温释放后不再锁回（§1.4 J6_THERMAL；§3.7-2 清除前 J6 保持释放）。
void Core::hold_all(Decision& d, bool force, const Feedback& fb) {
  for (int i = 0; i < N; ++i) {
    if (i == N - 1 && (latched_ & bits::J6_THERMAL)) continue;
    if (!force && ax_[i] == Ax::HOLD && !tx_pending_[i]) continue;
    want_stop(d, i, kHold, fb);
  }
  stream_base_ok_ = false;
}

void Core::track_still(const Feedback& fb, const Fresh& fresh) {
  const int64_t max_dt = int64_t(cfg::kStillMaxDtPeriods) * period_ms_ * 1000000LL;
  constexpr int W = cfg::kStillWindow;
  for (int i = 0; i < N; ++i) {
    if (!fresh[i]) {  // 缺回包：窗口清空，不证明任何事（R08）
      win_n_[i] = 0;
      continue;
    }
    int64_t t = fb.ax[i].t_ns;
    if (win_n_[i] > 0) {
      if (t == win_last_t_[i]) continue;  // 同一采样重复送入：不算新采样
      int64_t dt = t - win_last_t_[i];
      if (dt <= 0 || dt > max_dt) win_n_[i] = 0;  // 采样不连续：重攒
    }
    win_q_[i][win_head_[i]] = fb.ax[i].q;
    win_t_[i][win_head_[i]] = t;
    win_head_[i] = (win_head_[i] + 1) % W;
    if (win_n_[i] < W) ++win_n_[i];
    win_last_t_[i] = t;
  }
}

bool Core::still_window(int i, int64_t t_from, float& range) const {
  constexpr int W = cfg::kStillWindow;
  if (win_n_[i] < W) return false;
  float lo = win_q_[i][0], hi = lo;
  int64_t oldest = win_t_[i][0];
  for (int k = 0; k < W; ++k) {
    lo = std::min(lo, win_q_[i][k]);
    hi = std::max(hi, win_q_[i][k]);
    oldest = std::min(oldest, win_t_[i][k]);
  }
  if (oldest < t_from) return false;  // 窗口里还有保持帧之前的采样
  range = hi - lo;
  return true;
}

// 保持后停住确认（§3.6、§3.7-6；TC-010 实机改为窗口范围）：保持帧之后的 kStillWindow 个连续新鲜采样，
// max(q)−min(q) ≤ kStillRangeDeg 即停住。不再看相邻两帧差（静止读数会跳 0.1–0.3°）
void Core::eval_hold_confirm(Decision& d, const Feedback& fb, const Fresh& fresh, int64_t now, uint32_t& newf) {
  for (int i = 0; i < N; ++i) {
    Confirm& c = conf_[i];
    if (!c.active) continue;
    if (ax_[i] != Ax::HOLD) {  // 已改为流式 / 释放 / 阻尼：不再确认
      c.active = false;
      continue;
    }
    float rg = 0;
    bool full = fresh[i] && still_window(i, c.t0, rg);
    if (full && rg <= cfg::kStillRangeDeg) {
      c.active = false;
      continue;
    }
    if (now - c.t0 < int64_t(cfg::kHoldConfirmMs) * 1000000LL) continue;
    std::string why = !fresh[i] ? std::string("无新鲜回包")
                      : full    ? fmt("窗口范围 %.1f°", rg)
                                : fmt("连续新鲜采样不足 %d/%d", win_n_[i], cfg::kStillWindow);
    if (c.resends < cfg::kHoldResendMax) {
      ++c.resends;
      c.t0 = now;
      want_stop(d, i, kHold, fb);
      d.events.push_back("J" + std::to_string(i) + " 保持后 " + std::to_string(cfg::kHoldConfirmMs) +
                         " ms 未确认停住（" + why + "），重发 0x18 0x11 第 " + std::to_string(c.resends) + " 次");
    } else {
      c.active = false;
      newf |= bits::HOLD_FAIL;
      d.events.push_back("J" + std::to_string(i) + " 重发 0x18 " + std::to_string(cfg::kHoldResendMax) +
                         " 次后仍未确认停住（" + why + "）→ HOLD_FAIL。请用物理急停");
    }
  }
}

void Core::eval_feedback(Decision& d, const Feedback& fb, const Fresh& fresh, int64_t now, bool estop_now,
                         uint32_t& newf, bool& j6_hot) {
  for (int i = 0; i < N; ++i) {
    miss_[i] = fresh[i] ? 0 : miss_[i] + 1;
    if (miss_[i] >= cfg::kCommLossCycles) newf |= bits::COMM_LOSS;
    if (!fresh[i]) continue;
    const AxisSample& a = fb.ax[i];

    bool out = mturn_out(a.q);
    if (out) {
      if (i == N - 1) {
        if (!j6_pending_) {
          j6_pending_ = true;
          j6_tries_ = 0;
          j6_stage_ = J6Stage::NONE;
          d.events.push_back(fmt("J6 多圈超限 q=%.1f turns=%.0f，J6 停止 0x08，转自动清圈", a.q, a.turns));
        }
      } else {
        newf |= bits::MTURN;
      }
    } else {
      if (i == N - 1 && j6_pending_) {
        j6_pending_ = false;
        j6_tries_ = 0;
        j6_stage_ = J6Stage::NONE;
        d.events.push_back(fmt("J6 多圈已回到范围 q=%.1f", a.q));
      }
      if (out_of_limit(i, a.q)) d.warn |= bits::OUT_OF_LIMIT;
    }

    StatusClass sc = classify_status(i, a.status);
    if (sc.fault) newf |= bits::SERVO_PROT;
    if (sc.j6_warn) d.warn |= bits::J6_PROT;

    if (i == N - 1) {
      J6Temp t = j6_temp_level(a.temp_c);
      if (t == J6Temp::RELEASE) newf |= bits::J6_THERMAL;
      // 降出力回差（TC-010 实机：44.9/45.0 ℃ 抖动时 82–188 s 进出 90 次，夹爪反复开合）
      if (!j6_derate_on_ && a.temp_c >= cfg::kJ6TempWarnC) {
        j6_derate_on_ = true;
        j6_derate_tgt_ = clamp_limit(i, a.q).q;  // 只在进入时锁定一次
        d.events.push_back(fmt("J6 %.1f ℃ ≥%.0f ℃：进入降出力，锁定目标 %.1f°", a.temp_c, cfg::kJ6TempWarnC,
                               j6_derate_tgt_));
      } else if (j6_derate_on_ && a.temp_c <= cfg::kJ6TempWarnExitC) {
        j6_derate_on_ = false;
        d.events.push_back(fmt("J6 %.1f ℃ ≤%.0f ℃：退出降出力，恢复正常跟随", a.temp_c, cfg::kJ6TempWarnExitC));
      } else if (j6_derate_on_ && !j6_forcing()) {
        j6_derate_tgt_ = clamp_limit(i, a.q).q;  // J6 不出力（释放 / 阻尼 / 排除）时没有棘轮，锁定值随实测
      }
    }
    if (i == N - 1 && j6_derate_on_) {
      d.warn |= bits::J6_HOT;
      j6_hot = true;
    }

    // 卡住判定（任务书 §3.6）：仅 J0–J5、流式中；目标 = 最近写成功的目标
    if (i < N - 1 && st_ == CtrlState::STREAMING && std::isfinite(bus_tgt_[i])) {
      float err = std::fabs(bus_tgt_[i] - a.q);
      bool bad = err > cfg::kTrackErrDeg && a.current_ma > cfg::kTrackStuckCurrentMa;
      if (!bad) {
        track_since_[i] = 0;
      } else if (track_since_[i] == 0) {
        track_since_[i] = now;
      } else if (now - track_since_[i] > int64_t(cfg::kTrackErrMs) * 1000000LL) {
        newf |= bits::SERVO_PROT;
        d.events.push_back(fmt("J%.0f 跟踪误差 %.1f° 持续 >1 s 且电流高 → 判卡住", i, err));
      }
    } else if (i < N - 1) {
      track_since_[i] = 0;
    }
  }

  // J6 温度连续 COMM_LOSS_CYCLES 周期不可知 → 按 J6_THERMAL 处理（§3.7-3）
  if (miss_[N - 1] >= cfg::kCommLossCycles && !(latched_ & bits::J6_THERMAL)) {
    newf |= bits::J6_THERMAL;
    d.events.push_back("J6 温度连续 " + std::to_string(cfg::kCommLossCycles) + " 周期不可知 → 按 J6_THERMAL 释放 J6");
  }

  // 运行中 J6 多圈：未锁存故障且本周期没有 ESTOP 时，按 §3.3 分阶段自动处理（R05）：
  //   释放 → (≥kReleaseToClearMs) 0x11 → (≥kClearToReadMs + 1 周期，保证用的是清圈后的读数) 判定；最多 kMturnClearRetries 轮
  if (j6_pending_ && !latched_ && !estop_now && fresh[N - 1] && mturn_out(fb.ax[N - 1].q)) {
    const int64_t ms = 1000000LL;
    if (j6_stage_ == J6Stage::CLEAR_SENT && now - j6_stage_t_ >= (cfg::kClearToReadMs + period_ms_) * ms)
      j6_stage_ = J6Stage::NONE;  // 清圈后仍超限 → 下一轮
    if (j6_stage_ == J6Stage::NONE) {
      if (j6_tries_ < cfg::kMturnClearRetries) {
        ++j6_tries_;
        want_stop(d, N - 1, kRelease, fb);
        j6_stage_ = J6Stage::RELEASE_SENT;
        j6_stage_t_ = now;
        d.events.push_back(fmt("J6 自动清圈第 %.0f 轮：0x18 释放", j6_tries_));
      } else {
        newf |= bits::MTURN;
      }
    } else if (j6_stage_ == J6Stage::RELEASE_SENT && now - j6_stage_t_ >= cfg::kReleaseToClearMs * ms) {
      d.ops.push_back({BusOp::CLEAR, uint8_t(N - 1), 0, 0.0f});
      j6_stage_ = J6Stage::CLEAR_SENT;
      j6_stage_t_ = now;
    }
  }

  eval_hold_confirm(d, fb, fresh, now, newf);

  // ESTOP 后汇总：保持帧都已写出、确认全部结束
  if (estop_watch_ && estop_conf_started_) {
    bool busy = false;
    for (int i = 0; i < N; ++i) busy |= conf_[i].active || tx_pending_[i];
    if (!busy) {
      double ms = double(now - estop_t_) / 1e6;
      d.events.push_back((latched_ | newf) & bits::HOLD_FAIL
                             ? fmt("ESTOP 后有轴未确认停住（见 HOLD_FAIL，%.0f ms）——靠物理急停", ms)
                             : fmt("ESTOP 后全部保持轴确认停住（%.0f ms）", ms));
      estop_watch_ = false;
    }
  }
}

void Core::do_estop(Decision& d, const Cmd& c, const Feedback& fb, const Fresh& fresh, int64_t now) {
  if (c.rx_no > estop_rx_no_) estop_rx_no_ = c.rx_no;  // 接收代次：此后收到的 RESET 才能解除（§3.7-1）
  d.estop = true;
  if (latched_ & bits::ESTOP) return;  // 已锁存：不重复下发
  latched_ |= bits::ESTOP;
  st_ = CtrlState::ESTOP;
  j6_stage_ = J6Stage::NONE;  // 抢占 J6 清圈序列（R05）
  // ① 一帧 0x19+0x08 冻结：目标 = 本周期新鲜实测 q，time = 周期（任务书 §3.4）。
  //    无新鲜读数 / 多圈超限 / J6 过温释放或清圈中的轴不进冻结帧（0x08 按多圈坐标执行，F58）。
  bool any = false;
  for (int i = 0; i < N; ++i) {
    if (!fresh[i] || mturn_out(fb.ax[i].q)) continue;
    if (i == N - 1 && j6_excluded()) continue;
    d.move_axis[i] = true;
    d.move_q[i] = clamp_limit(i, fb.ax[i].q).q;
    d.move_power[i] = 0;
    any = true;
  }
  d.move_time_ms = uint16_t(period_ms_);
  if (any) d.ops.push_back({BusOp::MOVE, 0, 0, 0.0f});
  // ② 逐轴 0x18 0x11（帧间隔由 bus 保证）
  hold_all(d, true, fb);
  estop_watch_ = true;
  estop_conf_started_ = false;
  estop_t_ = now;
  d.events.push_back("ESTOP：冻结帧 + 全轴 0x18 0x11，锁存至 RESET");
}

void Core::do_stream(Decision& d, const Cmd& c, const Feedback& fb, bool j6_hot, bool& applied) {
  for (int i = 0; i < N; ++i) {
    if (!fb.have[i] && !(i == N - 1 && j6_excluded())) {
      d.warn |= bits::CMD_REJECTED;  // 没有实测就没有步长基准
      return;
    }
  }
  if (wd_tripped_) {
    for (int i = 0; i < N; ++i) {
      if (i == N - 1 && j6_excluded()) continue;
      if (std::fabs(c.q[i] - fb.ax[i].q) > cfg::kResumeTolDeg) {
        d.warn |= bits::TARGET_FAR;
        return;  // 保持，等近的目标（任务书 §3.2）
      }
    }
    wd_tripped_ = false;
    d.events.push_back("看门狗恢复：STREAM 目标在 RESUME_TOL 内");
  }
  // 步长基准：本次流式里最近写成功的目标；刚进入流式 / 上一帧没写出去 → 当前实测（防跳变）
  bool base_ok = st_ == CtrlState::STREAMING && stream_base_ok_;
  for (int i = 0; i < N; ++i) {
    if (i == N - 1 && j6_excluded()) continue;
    float base = base_ok && std::isfinite(bus_tgt_[i]) ? bus_tgt_[i] : fb.ax[i].q;
    Clamp s = clamp_step(i, c.q[i], base, period_ms_);
    Clamp l = clamp_limit(i, s.q);
    if (s.clamped) d.warn |= bits::STEP_CLAMPED;
    if (l.clamped) d.warn |= bits::LIMIT_CLAMPED;
    float t = l.q;
    if (i == N - 1 && j6_hot) {
      // 降出力：目标不比锁定值更闭合（夹爪闭合 = 角度变小，F60）；更张开的照常执行并更新锁定值（§3.5，TC-010 实机）
      if (t < j6_derate_tgt_) t = j6_derate_tgt_;
      else j6_derate_tgt_ = t;
    }
    d.move_axis[i] = true;
    d.move_q[i] = t;
    d.move_power[i] = (i == N - 1) ? clamp_grip_power(c.grip_power_mw) : cfg::kArmPowerMw;
    ax_[i] = Ax::MOVING;
  }
  j6_power_ = clamp_grip_power(c.grip_power_mw);
  d.move_time_ms = clamp_dt(c.dt_ms, period_ms_);
  d.ops.push_back({BusOp::MOVE, 0, 0, 0.0f});
  st_ = CtrlState::STREAMING;
  applied = true;
}

// RESET 逐位清除（§3.7-2）：只清本周期有新鲜有效反馈证明原因已解除的位
void Core::do_reset(Decision& d, const Cmd& c, const Feedback& fb, const Fresh& fresh, bool& applied) {
  bool all_fresh = true;
  for (int i = 0; i < N; ++i) all_fresh &= fresh[i];
  uint32_t clr = 0;
  if ((latched_ & bits::ESTOP) && c.rx_no > estop_rx_no_) clr |= bits::ESTOP;
  if ((latched_ & bits::COMM_LOSS) && all_fresh) clr |= bits::COMM_LOSS;
  if ((latched_ & bits::MTURN) && all_fresh) {
    bool ok = true;
    for (int i = 0; i < N; ++i) ok &= !mturn_out(fb.ax[i].q);
    if (ok) clr |= bits::MTURN;
  }
  if ((latched_ & bits::SERVO_PROT) && all_fresh) {
    bool ok = true;
    for (int i = 0; i < N; ++i) ok &= !classify_status(i, fb.ax[i].status).fault;
    if (ok) clr |= bits::SERVO_PROT;
  }
  if ((latched_ & bits::J6_THERMAL) && fresh[N - 1] && fb.ax[N - 1].temp_c <= cfg::kJ6TempWarnC)
    clr |= bits::J6_THERMAL;
  if (latched_ & bits::TX_FAIL) {
    bool ok = true;
    for (int i = 0; i < N; ++i) ok &= !tx_pending_[i];
    if (ok) clr |= bits::TX_FAIL;
  }
  if ((latched_ & bits::HOLD_FAIL) && all_fresh) {
    bool ok = true;
    for (int i = 0; i < N; ++i) {
      float rg = 0;
      ok &= still_window(i, INT64_MIN, rg) && rg <= cfg::kStillRangeDeg;  // 与保持确认同一判据
    }
    if (ok) clr |= bits::HOLD_FAIL;
  }
  latched_ &= ~clr;
  if (clr) d.events.push_back(hex("RESET 清除 fault=", clr));
  if (latched_) {
    // 未清的位继续锁存；不 ack（上位机看 fault 与 warn RESET_DENIED）
    d.warn |= bits::RESET_DENIED;
    d.events.push_back(hex("RESET 未清除 fault=", latched_) + "（原因未经本周期新鲜反馈证明已解除）");
    if (st_ == CtrlState::ESTOP) st_ = CtrlState::FAULT;
    return;
  }
  wd_tripped_ = false;
  startup_noact_ = false;
  j6_tries_ = 0;
  estop_watch_ = false;
  st_ = CtrlState::HOLD;
  hold_all(d, false, fb);
  applied = true;
}

void Core::handle_cmd(Decision& d, const Cmd& c, const Feedback& fb, const Fresh& fresh, bool j6_hot,
                      bool& applied) {
  if (!cmd_valid(c)) {
    d.warn |= bits::CMD_REJECTED;
    return;
  }
  Mode m = Mode(c.mode);
  if (m == Mode::RESET) {
    do_reset(d, c, fb, fresh, applied);
    return;
  }
  if (latched_) {  // ESTOP / FAULT 锁存期间只认 ESTOP、RESET
    d.warn |= bits::CMD_REJECTED;
    return;
  }
  switch (m) {
    case Mode::HOLD:
      wd_tripped_ = false;
      st_ = CtrlState::HOLD;
      hold_all(d, false, fb);
      applied = true;
      break;
    case Mode::RELEASE:
    case Mode::DAMP: {
      Ax target = m == Mode::RELEASE ? Ax::RELEASED : Ax::DAMPED;
      uint8_t mode = m == Mode::RELEASE ? kRelease : kDamp;
      for (int i = 0; i < N; ++i) {
        if (c.axis_mask & (1u << i)) {
          if (ax_[i] != target || tx_pending_[i]) want_stop(d, i, mode, fb);
        } else if (ax_[i] == Ax::MOVING || ax_[i] == Ax::UNKNOWN) {
          want_stop(d, i, kHold, fb);  // 不在 mask 里的轴：停流式、保持
        }
      }
      wd_tripped_ = false;
      stream_base_ok_ = false;
      st_ = CtrlState::RELEASED;
      applied = true;
      break;
    }
    case Mode::STREAM:
      do_stream(d, c, fb, j6_hot, applied);
      break;
    default:
      break;
  }
}

// 45 ℃ 降出力（§3.7-4；TC-010 实机改）：J6 处于持续出力状态（流式 / 保持 / 看门狗保持 / 故障保持），
// 且总线目标比锁定值更闭合 → 发一次 0x08 到锁定值。不再每周期跟随实测（棘轮）。故障锁存时也执行。
bool Core::j6_forcing() const {
  const int j = N - 1;
  return (ax_[j] == Ax::MOVING || ax_[j] == Ax::HOLD) && !j6_excluded();
}

void Core::j6_derate(Decision& d, const Feedback& fb, const Fresh& fresh, bool j6_hot) {
  (void)fb;
  const int j = N - 1;
  if (!j6_hot || !fresh[j] || d.move_axis[j] || !j6_forcing() || startup_noact_) return;
  const float m = j6_derate_tgt_;
  if (std::isfinite(bus_tgt_[j]) && m - bus_tgt_[j] < cfg::kJ6HotRetargetDeg) return;
  d.move_axis[j] = true;
  d.move_q[j] = m;
  d.move_power[j] = j6_power_ ? j6_power_ : cfg::kGripPowerMaxMw;
  if (!d.has_move()) {
    d.move_time_ms = uint16_t(period_ms_);
    d.ops.push_back({BusOp::MOVE, 0, 0, 0.0f});
  }
  d.events.push_back(fmt("J6 降出力：0x08 目标移到锁定角 %.1f°", m));
}

Decision Core::step(const Cmd* cmd, const Feedback& fb, int64_t now) {
  Decision d;
  uint32_t newf = 0;
  bool j6_hot = false;
  bool estop_now = cmd && cmd_valid(*cmd) && Mode(cmd->mode) == Mode::ESTOP;

  // 0) 新鲜反馈：回包有效且温度可解（§3.7-3：温度无效 = 该轴本周期无效）
  Fresh fresh{};
  for (int i = 0; i < N; ++i) fresh[i] = fb.valid[i] && std::isfinite(fb.ax[i].temp_c);
  track_still(fb, fresh);

  // 1) 上周期没写出去的停止帧：重发直到成功（§3.7-5）
  for (int i = 0; i < N; ++i)
    if (tx_pending_[i]) want_stop(d, i, stop_mode_of(i), fb);

  // 2) 反馈检查（上一周期的 0x16）
  eval_feedback(d, fb, fresh, now, estop_now, newf, j6_hot);
  newf &= ~latched_;
  if (newf) {
    latched_ |= newf;
    d.events.push_back(hex("新故障 → 全轴保持 fault=", newf));
    if (newf & bits::J6_THERMAL) {
      want_stop(d, N - 1, kRelease, fb);  // J6 释放，瓶子会掉（§3.5 安全优先）
      j6_stage_ = J6Stage::NONE;
    }
    if (!startup_noact_) hold_all(d, false, fb);
    if (st_ != CtrlState::ESTOP) st_ = CtrlState::FAULT;
  }
  if (need_hold_all_) {  // 上周期 commit 新锁存了 TX_FAIL
    need_hold_all_ = false;
    if (!startup_noact_) hold_all(d, false, fb);
  }

  // 3) 命令
  if (cmd) {
    bool applied = false;
    if (estop_now) {
      do_estop(d, *cmd, fb, fresh, now);
      applied = true;
    } else {
      handle_cmd(d, *cmd, fb, fresh, j6_hot, applied);
    }
    if (applied) {
      d.cmd_applied = true;
      d.cand_seq = cmd->seq;
      d.cand_stamp_ns = cmd->stamp_ns;
      d.cand_rx_ns = cmd->rx_real_ns;
      last_cmd_mono_ = cmd->rx_mono_ns;
      wd_armed_ = true;
    } else if (cmd_valid(*cmd)) {
      // 合法但未执行（锁存中 / TARGET_FAR / 无基准）：上位机仍在线，刷新看门狗计时，不改 ack
      last_cmd_mono_ = cmd->rx_mono_ns;
    }
  }

  // 4) 看门狗（任务书 §3.2）：收到第一条合法命令后启用，**只在 STREAMING 下起作用**（TC-010 实机，PM 裁定：
  //    目的是上位机断了就停止运动；HOLD / RELEASED / DAMP / ESTOP / FAULT 下没有在执行的运动。
  //    RELEASE 后若看门狗把轴锁回，人正托着臂摆位时会突然锁死）
  if (wd_armed_ && st_ == CtrlState::STREAMING && !latched_ && !wd_tripped_ &&
      now - last_cmd_mono_ > int64_t(cfg::kWatchdogMs) * 1000000LL) {
    wd_tripped_ = true;
    st_ = CtrlState::HOLD;
    hold_all(d, false, fb);
    d.events.push_back("看门狗超时 → 全轴 0x18 0x11 保持");
  }

  // 5) J6 45 ℃ 降出力（独立安全动作）
  j6_derate(d, fb, fresh, j6_hot);

  // 6) ESTOP 周期：冻结帧排最前，不执行清圈（R05）
  if (d.estop) {
    std::stable_partition(d.ops.begin(), d.ops.end(), [](const BusOp& o) { return o.kind == BusOp::MOVE; });
    d.ops.erase(std::remove_if(d.ops.begin(), d.ops.end(), [](const BusOp& o) { return o.kind == BusOp::CLEAR; }),
                d.ops.end());
  }

  // 未 commit 时的默认输出（commit 会覆盖）
  d.state = st_;
  d.fault = latched_ | (wd_tripped_ ? bits::WATCHDOG : 0u);
  d.q_target = bus_tgt_;
  d.ack_seq = ack_seq_;
  d.ack_stamp_ns = ack_stamp_;
  d.ack_rx_ns = ack_rx_;
  return d;
}

void Core::commit(Decision& d, const std::vector<OpResult>& res, int64_t now) {
  bool all_ok = true, failed = false;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (size_t k = 0; k < d.ops.size(); ++k) {
    const BusOp& o = d.ops[k];
    OpResult r = k < res.size() ? res[k] : OpResult::SKIPPED;
    if (r != OpResult::OK) all_ok = false;
    if (r == OpResult::FAIL) failed = true;
    switch (o.kind) {
      case BusOp::MOVE:
        if (r == OpResult::OK) {
          for (int i = 0; i < N; ++i)
            if (d.move_axis[i]) bus_tgt_[i] = d.move_q[i];
          if (st_ == CtrlState::STREAMING && !d.estop) stream_base_ok_ = true;
        } else {
          stream_base_ok_ = false;  // 舵机还在旧目标：下一帧步长从实测起算
        }
        break;
      case BusOp::STOP:
        if (r == OpResult::OK) {
          tx_pending_[o.axis] = false;
          if (o.mode == kHold) {
            // 保持目标沿用上次写成功的命令角；只有从未下令 / 失力后（NaN）才取实测角。
            // 用实测角会把重力下沉写进 q_target，下一个 SDK 进程据此保持 → 臂逐次下掉（F76 跨进程版）
            if (!std::isfinite(bus_tgt_[o.axis])) bus_tgt_[o.axis] = o.q_meas;
            if (!conf_[o.axis].active) conf_[o.axis] = Confirm{true, now, 0};
          } else {
            bus_tgt_[o.axis] = nan;
            conf_[o.axis].active = false;
          }
        } else {
          tx_pending_[o.axis] = true;
        }
        break;
      case BusOp::CLEAR:
        if (r != OpResult::OK && j6_stage_ == J6Stage::CLEAR_SENT) j6_stage_ = J6Stage::RELEASE_SENT;
        break;
    }
  }
  if (d.estop && !estop_conf_started_) estop_conf_started_ = true;
  if (failed && !(latched_ & bits::TX_FAIL)) {
    latched_ |= bits::TX_FAIL;
    if (st_ != CtrlState::ESTOP) st_ = CtrlState::FAULT;
    need_hold_all_ = true;
    stream_base_ok_ = false;
    d.events.push_back("总线写帧重试 " + std::to_string(cfg::kTxRetries) +
                       " 次仍失败 → TX_FAIL（锁存）；停止帧每周期重发直到成功。请用物理急停");
  }
  if (d.cmd_applied && all_ok) {
    ack_seq_ = d.cand_seq;
    ack_stamp_ = d.cand_stamp_ns;
    ack_rx_ = d.cand_rx_ns;
  }
  d.state = st_;
  d.fault = latched_ | (wd_tripped_ ? bits::WATCHDOG : 0u);
  d.q_target = bus_tgt_;
  d.ack_seq = ack_seq_;
  d.ack_stamp_ns = ack_stamp_;
  d.ack_rx_ns = ack_rx_;
}

Decision Core::on_shutdown() {
  Decision d;
  Feedback none;
  for (int i = 0; i < N; ++i) none.ax[i].q = std::isfinite(bus_tgt_[i]) ? bus_tgt_[i] : 0.0f;
  if (!startup_noact_) hold_all(d, true, none);
  d.state = st_;
  return d;
}

}  // namespace safety
}  // namespace d1t
