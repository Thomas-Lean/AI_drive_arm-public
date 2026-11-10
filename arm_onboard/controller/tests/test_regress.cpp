// test_regress.cpp —— 历史审查复现场景的永久回归（记录见 docs/reviews/P1.1-控制器审查/）
//
// 每个 R* 块的注释写着原实现的输出；断言的是修复后的行为（与原输出相反）。
// §5 表 9 个场景：R01 R02 R03a R03b R07 R05 R06 R08 R04；另加 R09（pty 写入期限）、R10、R11、R12、
// 以及 R02 端到端（FakeSerial 合法校验和 + 非法 ADC）。
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "bus/bus.hpp"
#include "check.hpp"
#include "common/clock.hpp"
#include "loop/controller.hpp"
#include "loop/mailbox.hpp"
#include "safety/safety.hpp"

using namespace d1t;
using namespace d1t::safety;

static const int P = cfg::kPeriodMsDefault;
static const int64_t MS = 1000000LL;
static const float NaN = std::numeric_limits<float>::quiet_NaN();
static const std::array<float, N> Q0 = {0, -60, 60, 0, 10, 0, 40};

// 一个周期的反馈：所有轴新鲜，采样时刻 = t - 10 ms
static Feedback fb_at(std::array<float, N> q, int64_t t) {
  Feedback f;
  for (int i = 0; i < N; ++i) {
    f.valid[i] = f.have[i] = true;
    f.ax[i].q = q[i];
    f.ax[i].temp_c = 30.0f;
    f.ax[i].current_ma = 14;
    f.ax[i].t_ns = t - 10 * MS;
  }
  return f;
}
static void stamp(Feedback& f, int64_t t) {
  for (int i = 0; i < N; ++i) f.ax[i].t_ns = t - 10 * MS;
}

static Cmd mk(Mode m, uint32_t seq, int64_t t, std::array<float, N> q = Q0, uint8_t mask = 0) {
  Cmd c;
  c.seq = seq;
  c.stamp_ns = t;
  c.mode = uint8_t(m);
  c.axis_mask = mask;
  c.q = q;
  c.dt_ms = 50;
  c.rx_mono_ns = t;
  c.rx_real_ns = t;
  c.rx_no = seq;
  return c;
}

static Decision S(Core& c, const Cmd* cmd, const Feedback& f, int64_t t) {
  Decision d = c.step(cmd, f, t);
  c.commit(d, std::vector<OpResult>(d.ops.size(), OpResult::OK), t);
  return d;
}
static int count_ops(const Decision& d, BusOp::Kind k, int axis = -1, int mode = -1) {
  int n = 0;
  for (auto& o : d.ops)
    if (o.kind == k && (axis < 0 || o.axis == axis) && (mode < 0 || o.mode == mode)) ++n;
  return n;
}
static bool released_arm(const Decision& d) {
  for (auto& o : d.ops)
    if (o.kind == BusOp::STOP && o.mode == 0x10 && o.axis < N - 1) return true;
  return false;
}
static bool has_event(const Decision& d, const char* sub) {
  for (auto& e : d.events)
    if (e.find(sub) != std::string::npos) return true;
  return false;
}

// 只让某一类帧写失败的串口包装（R04）
struct FailSerial : ISerial {
  FakeSerial& fs;
  std::atomic<int> fail_stop_axis{-1};
  std::atomic<int> stop_attempts{0};
  explicit FailSerial(FakeSerial& f) : fs(f) {}
  bool write_all(const uint8_t* p, size_t n) override {
    if (n >= 5 && p[2] == frame::CMD_STOP && p[4] == fail_stop_axis.load()) {
      ++stop_attempts;
      return false;
    }
    return fs.write_all(p, n);
  }
  size_t read_some(uint8_t* p, size_t cap, int64_t t) override { return fs.read_some(p, cap, t); }
  size_t flush_input() override { return fs.flush_input(); }
};

// 真主循环 + 线程（Controller 级场景用）
struct Loop {
  Mailbox mb;
  Controller ctrl;
  std::atomic<bool> stop{false};
  std::thread th;
  std::vector<StateMsg> st;
  std::thread rd;
  std::mutex mu;
  explicit Loop(ISerial& s) : ctrl(s, mb, CtrlOptions{P, false}, nullptr) {}
  void start() {
    th = std::thread([this] { ctrl.run(stop); });
    rd = std::thread([this] {
      StateMsg m;
      while (!mb.is_shutdown())
        if (mb.pop_state(m, 20 * MS)) {
          std::lock_guard<std::mutex> lk(mu);
          st.push_back(m);
        }
    });
  }
  void put(Mode m, uint32_t seq, uint8_t mask = 0) {
    Cmd c;
    c.seq = seq;
    c.stamp_ns = now_real_ns();
    c.mode = uint8_t(m);
    c.axis_mask = mask;
    c.dt_ms = uint16_t(P);
    mb.put_cmd(c);
  }
  StateMsg last() {
    std::lock_guard<std::mutex> lk(mu);
    return st.empty() ? StateMsg{} : st.back();
  }
  void halt() {
    stop = true;
    mb.shutdown();
    if (th.joinable()) th.join();
    if (rd.joinable()) rd.join();
  }
  ~Loop() { halt(); }
};

// ---- V2-01：可注入的排空查询 ----
struct FakeTty : TtyQuery {
  int outq_mode = 0;   // 0 = 正常返回 0；1 = 不支持；2 = 一直不为 0（停滞）
  int temt_mode = 0;   // 0 = 空；1 = 不支持；2 = 一直不空（停滞）
  int out_queue(int, int& b) override {
    if (outq_mode == 1) return -1;
    b = outq_mode == 2 ? 17 : 0;
    return 0;
  }
  int tx_empty(int, bool& e) override {
    if (temt_mode == 1) return -1;
    e = temt_mode != 2;
    return 0;
  }
};

// ---- V2-02：第一帧 0x18 写入时（或写失败时）放进 ESTOP，记冻结帧开始时刻 ----
struct EstopInjectSerial : ISerial {
  FakeSerial& fs;
  Mailbox* mb = nullptr;
  bool fail_first_stop = false;
  std::atomic<bool> armed{false};
  std::atomic<int64_t> estop_rx{0}, freeze_start{0};
  std::atomic<int> stops_after_estop{0};
  explicit EstopInjectSerial(FakeSerial& f) : fs(f) {}
  bool write_all(const uint8_t* p, size_t n) override {
    int64_t t = now_mono_ns();
    if (n >= 5 && p[2] == frame::CMD_STOP && armed.exchange(false)) {
      Cmd e;
      e.seq = 900;
      e.stamp_ns = now_real_ns();
      e.mode = uint8_t(Mode::ESTOP);
      estop_rx = now_mono_ns();
      e.rx_mono_ns = estop_rx;
      mb->put_cmd(e);  // 「第 1 帧发出途中 ESTOP 到达」
      if (fail_first_stop) return false;
    } else if (estop_rx && !freeze_start && n >= 5 && p[2] == frame::CMD_STOP) {
      ++stops_after_estop;  // ESTOP 之后、冻结帧之前还在发的旧批次停止帧
    }
    if (estop_rx && !freeze_start && n >= 5 && p[2] == frame::CMD_SYNC && p[4] == frame::CMD_SET_ANGLE)
      freeze_start = t;
    return fs.write_all(p, n);
  }
  size_t read_some(uint8_t* p, size_t cap, int64_t t) override { return fs.read_some(p, cap, t); }
  size_t flush_input() override { return fs.flush_input(); }
};

int main() {
  // ===== R01：ESTOP 之前排队的 RESET 不得解除急停 =====
  // 原：old_RESET: fault_before=2 fault_after=0 consumed_seq=1
  std::printf("== R01 旧 RESET 排在 ESTOP 前\n");
  {
    Mailbox mb;
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd r = mk(Mode::RESET, 1, 100), e = mk(Mode::ESTOP, 2, 200);
    r.rx_no = e.rx_no = 0;  // 由信箱编号
    mb.put_cmd(r);
    mb.put_cmd(e);
    Cmd got;
    CHECK(mb.take_cmd(got) && Mode(got.mode) == Mode::ESTOP);
    Decision d = S(c, &got, f, 10 * MS);
    uint32_t before = d.fault;
    CHECK(!mb.take_cmd(got));  // 旧 RESET 已作废
    CHECK(mb.totals().dropped_by_estop == 1);
    stamp(f, 60 * MS);
    d = S(c, nullptr, f, 60 * MS);
    std::printf("   fault_before=0x%x fault_after=0x%x\n", before, d.fault);
    CHECK((before & bits::ESTOP) && (d.fault & bits::ESTOP) && c.state() == CtrlState::ESTOP);
    // 第二道：即使旧 RESET 绕过信箱到了 safety，接收序号早于 ESTOP 也不清
    Cmd old = mk(Mode::RESET, 1, 100);
    old.rx_no = got.rx_no;  // 不大于 ESTOP 的
    Cmd e2 = mk(Mode::ESTOP, 7, 300);
    e2.rx_no = 7;
    Core c2(P);
    c2.on_startup_done(false);
    S(c2, &e2, f, 70 * MS);
    old.rx_no = 6;
    d = S(c2, &old, f, 80 * MS);
    CHECK((d.fault & bits::ESTOP) && (d.warn & bits::RESET_DENIED));
    Cmd fresh_reset = mk(Mode::RESET, 8, 400);
    d = S(c2, &fresh_reset, f, 90 * MS);
    CHECK(!(d.fault & bits::ESTOP) && c2.state() == CtrlState::HOLD && d.ack_seq == 8);
  }

  // ===== R12：ESTOP 不抬高普通命令的 stamp 水位 =====
  std::printf("== R12 时钟快的 ESTOP 不让原控制机 RESET 变 stale\n");
  {
    Mailbox mb;
    mb.put_cmd(mk(Mode::STREAM, 1, 1000));
    Cmd tmp;
    mb.take_cmd(tmp);
    Cmd e = mk(Mode::ESTOP, 1, int64_t(4e18));  // 另一台机器，时钟快很多
    e.rx_no = 0;
    mb.put_cmd(e);
    mb.take_cmd(tmp);
    Cmd r = mk(Mode::RESET, 2, 2000);
    r.rx_no = 0;
    mb.put_cmd(r);
    CHECK(mb.totals().stale == 0);
    CHECK(mb.take_cmd(tmp) && Mode(tmp.mode) == Mode::RESET && tmp.rx_no > e.rx_no);
  }

  // ===== R02：温度 NaN 不得当成正常温度 =====
  // 原：NaN_temp_5s: fault=0 warn=0 move=1
  std::printf("== R02 J6 温度 NaN 持续流式 5 s\n");
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 32.0f;
    auto tgt = q;
    tgt[6] = 29.0f;
    Decision d;
    int j6_moves_after = 0, j6_release = 0;
    bool latched = false;
    for (int k = 0; k < 100; ++k) {
      int64_t t = k * 50 * MS;
      Feedback f = fb_at(q, t);
      if (k > 0) f.ax[6].temp_c = NaN;
      Cmd s = mk(Mode::STREAM, 1 + k, t, tgt);
      d = S(c, &s, f, t);
      if (latched && d.move_axis[6]) ++j6_moves_after;
      j6_release += count_ops(d, BusOp::STOP, 6, 0x10);
      CHECK(!released_arm(d));
      latched = latched || (d.fault & bits::J6_THERMAL);
    }
    std::printf("   fault=0x%x warn=0x%x J6 释放帧=%d 锁存后 J6 0x08=%d\n", d.fault, d.warn, j6_release, j6_moves_after);
    CHECK((d.fault & bits::COMM_LOSS) && (d.fault & bits::J6_THERMAL));
    CHECK(j6_release == 1 && j6_moves_after == 0 && !d.has_move());
    CHECK(c.axis_mode(6) == Core::Ax::RELEASED);
    for (int i = 0; i < N - 1; ++i) CHECK(c.axis_mode(i) == Core::Ax::HOLD);  // J0–J5 保持，不释放
  }

  // ===== R03a：J6 60 ℃ 热释放后 RESET 不得锁回 =====
  // 原：RESET_at60C: fault=0 J6_mode=2(HOLD) ops=1
  std::printf("== R03a 60 ℃ 热释放后 RESET（含连续 RESET）\n");
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    f = fb_at(Q0, 50 * MS);
    f.ax[6].temp_c = 60.0f;
    Decision d = S(c, nullptr, f, 50 * MS);
    CHECK((d.fault & bits::J6_THERMAL) && count_ops(d, BusOp::STOP, 6, 0x10) == 1);
    for (int k = 0; k < 5; ++k) {
      int64_t t = (100 + 50 * k) * MS;
      f = fb_at(Q0, t);
      f.ax[6].temp_c = 60.0f;
      Cmd r = mk(Mode::RESET, 2 + k, t);
      d = S(c, &r, f, t);
      CHECK((d.fault & bits::J6_THERMAL) && (d.warn & bits::RESET_DENIED));
      CHECK(count_ops(d, BusOp::STOP, 6) == 0 && c.axis_mode(6) == Core::Ax::RELEASED);
      CHECK(d.ack_seq == 1);  // 未全部清除不 ack
    }
    std::printf("   60 ℃ RESET×5：fault=0x%x J6_mode=%d ops=%zu\n", d.fault, int(c.axis_mode(6)), d.ops.size());
    // 47 ℃（>45）仍不清；44 ℃ 才清，清后才锁回
    f = fb_at(Q0, 400 * MS);
    f.ax[6].temp_c = 47.0f;
    Cmd r1 = mk(Mode::RESET, 20, 400 * MS);
    d = S(c, &r1, f, 400 * MS);
    CHECK(d.fault & bits::J6_THERMAL);
    f = fb_at(Q0, 450 * MS);
    f.ax[6].temp_c = 44.0f;
    Cmd r2 = mk(Mode::RESET, 21, 450 * MS);
    d = S(c, &r2, f, 450 * MS);
    CHECK(!d.fault && c.state() == CtrlState::HOLD && count_ops(d, BusOp::STOP, 6, 0x11) == 1);
  }

  // ===== R03b：COMM_LOSS 后 J0 仍无新回包，RESET + STREAM 不得驱动 J0 =====
  // 原：RESET_no_feedback: fault=0 J0_move=1
  std::printf("== R03b COMM_LOSS 未恢复时 RESET + STREAM\n");
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    Decision d;
    for (int k = 1; k <= cfg::kCommLossCycles; ++k) {
      f = fb_at(Q0, k * 50 * MS);
      f.valid[0] = false;
      d = S(c, nullptr, f, k * 50 * MS);
    }
    CHECK(d.fault & bits::COMM_LOSS);
    f = fb_at(Q0, 300 * MS);
    f.valid[0] = false;
    Cmd r = mk(Mode::RESET, 2, 300 * MS);
    d = S(c, &r, f, 300 * MS);
    CHECK((d.fault & bits::COMM_LOSS) && (d.warn & bits::RESET_DENIED));
    f = fb_at(Q0, 350 * MS);
    f.valid[0] = false;
    Cmd s2 = mk(Mode::STREAM, 3, 350 * MS);
    d = S(c, &s2, f, 350 * MS);
    std::printf("   fault=0x%x J0_move=%d\n", d.fault, int(d.move_axis[0]));
    CHECK(!d.move_axis[0] && !d.has_move() && (d.warn & bits::CMD_REJECTED));
    // 同周期 RESET + 持续故障（J2 状态位仍在）
    f = fb_at(Q0, 400 * MS);
    f.ax[2].status = 0x20;
    Cmd r2 = mk(Mode::RESET, 4, 400 * MS);
    d = S(c, &r2, f, 400 * MS);
    CHECK((d.fault & bits::SERVO_PROT) && !(d.fault & bits::COMM_LOSS));  // COMM_LOSS 已有七轴新鲜回包 → 清
  }

  // ===== R07：45 ℃ 降出力覆盖无新命令 / HOLD / 故障锁存 =====
  // 原：HOT_no_new_cmd: warn=8192 ops=0 q_target=32.0
  std::printf("== R07 45 ℃ 降出力（无新命令 / HOLD / 故障保持）\n");
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 32.0f;
    auto tgt = q;
    tgt[6] = 29.0f;
    Feedback f = fb_at(q, 0);
    Cmd s = mk(Mode::STREAM, 1, 0, tgt);
    S(c, &s, f, 0);
    CHECK_NEAR(c.bus_target(6), 29.0, 1e-4);
    f = fb_at(q, 50 * MS);
    f.ax[6].temp_c = 46.0f;
    Decision d = S(c, nullptr, f, 50 * MS);
    std::printf("   无新命令：warn=0x%x J6 move=%d q=%.1f q_target=%.1f\n", d.warn, int(d.move_axis[6]), d.move_q[6],
                d.q_target[6]);
    CHECK((d.warn & bits::J6_HOT) && count_ops(d, BusOp::MOVE) == 1 && d.move_axis[6] && !d.move_axis[0]);
    CHECK_NEAR(d.move_q[6], 32.0, 1e-4);
    CHECK_NEAR(d.q_target[6], 32.0, 1e-4);  // 真的写出去了 32°
    f = fb_at(q, 100 * MS);
    f.ax[6].temp_c = 46.0f;
    d = S(c, nullptr, f, 100 * MS);
    CHECK(!d.move_axis[6]);  // 目标已不比实测更闭合：不重复发
  }
  {  // HOLD 中：保持帧时的实测 31°，之后夹爪被物体顶开到 33°（目标更闭合）→ 46 ℃ 时移到 33°
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 31.0f;
    Feedback f = fb_at(q, 0);
    Cmd h = mk(Mode::HOLD, 1, 0);
    S(c, &h, f, 0);
    q[6] = 33.0f;
    f = fb_at(q, 50 * MS);
    f.ax[6].temp_c = 46.0f;
    Decision d = S(c, nullptr, f, 50 * MS);
    CHECK(d.move_axis[6]);
    CHECK_NEAR(d.move_q[6], 33.0, 1e-4);
  }
  {  // 故障锁存中（J3 状态位）也执行
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 32.0f;
    auto tgt = q;
    tgt[6] = 29.0f;
    Feedback f = fb_at(q, 0);
    Cmd s = mk(Mode::STREAM, 1, 0, tgt);
    S(c, &s, f, 0);
    f = fb_at(q, 50 * MS);
    f.ax[3].status = 0x80;
    Decision d = S(c, nullptr, f, 50 * MS);  // FAULT → 全轴 0x18 0x11（J6 保持帧时实测 32）
    CHECK(c.state() == CtrlState::FAULT);
    q[6] = 32.5f;
    f = fb_at(q, 100 * MS);
    f.ax[3].status = 0x80;
    f.ax[6].temp_c = 46.0f;
    d = S(c, nullptr, f, 100 * MS);
    CHECK(d.move_axis[6] && !d.move_axis[0]);
    CHECK_NEAR(d.move_q[6], 32.5, 1e-4);
  }

  // ===== R05：J6 多圈处理与 ESTOP 同周期 / 处理中到达 =====
  // 原：STOP J6/0x10 → CLEAR J6 → MOVE → J0–J6 STOP/0x11
  std::printf("== R05 J6 多圈 + ESTOP\n");
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    f.ax[6].q = 388.6f;
    f.ax[6].turns = 1;
    Cmd e = mk(Mode::ESTOP, 1, 0);
    Decision d = S(c, &e, f, 0);
    std::string order;
    for (auto& o : d.ops) order += o.kind == BusOp::MOVE ? "M " : o.kind == BusOp::CLEAR ? "C " : "S ";
    std::printf("   顺序：%s\n", order.c_str());
    CHECK(!d.ops.empty() && d.ops[0].kind == BusOp::MOVE);
    CHECK(count_ops(d, BusOp::CLEAR) == 0 && count_ops(d, BusOp::STOP, 6, 0x10) == 0);
    CHECK(!d.move_axis[6]);
  }
  {  // 清圈序列进行中（已释放、等 100 ms）到达 ESTOP：冻结帧在最前，不发 0x11
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    f = fb_at(Q0, 50 * MS);
    f.ax[6].q = 388.6f;
    f.ax[6].turns = 1;
    Decision d = S(c, nullptr, f, 50 * MS);
    CHECK(count_ops(d, BusOp::STOP, 6, 0x10) == 1);
    stamp(f, 150 * MS);
    Cmd e = mk(Mode::ESTOP, 2, 150 * MS);
    d = S(c, &e, f, 150 * MS);  // 正好到清圈时刻
    CHECK(d.ops[0].kind == BusOp::MOVE && count_ops(d, BusOp::CLEAR) == 0);
    stamp(f, 300 * MS);
    d = S(c, nullptr, f, 300 * MS);
    CHECK(count_ops(d, BusOp::CLEAR) == 0);  // 锁存中不再继续清圈
  }

  // ===== R06：HOLD 后轴仍在动（每 50 ms +2°）=====
  // 原：HOLD_axis_moving: fault=0 ops=0 events=0
  std::printf("== R06 HOLD 后 J0 每 50 ms +2°，重复 HOLD\n");
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    Feedback f = fb_at(q, 0);
    Cmd h0 = mk(Mode::HOLD, 1, 0);
    S(c, &h0, f, 0);
    int resend = 0, k_fail = -1;
    Decision d;
    for (int k = 1; k <= 30; ++k) {
      int64_t t = k * 50 * MS;
      q[0] += 2.0f;
      f = fb_at(q, t);
      Cmd h = mk(Mode::HOLD, 1 + k, t);  // 一直重复 HOLD
      d = S(c, &h, f, t);
      resend += count_ops(d, BusOp::STOP, 0, 0x11);
      CHECK(count_ops(d, BusOp::STOP, 1) == 0);  // 静止轴不被重复 HOLD 刷帧
      if ((d.fault & bits::HOLD_FAIL) && k_fail < 0) k_fail = k;
    }
    std::printf("   J0 重发 0x18 %d 次；HOLD_FAIL 于 %d ms；fault=0x%x\n", resend, k_fail * 50, d.fault);
    CHECK(resend == cfg::kHoldResendMax);
    CHECK(k_fail > 0 && (d.fault & bits::HOLD_FAIL) && c.state() == CtrlState::FAULT);
    CHECK(!released_arm(d));
  }
  {  // 静止轴：凑满 kStillWindow 个新鲜采样后确认，不重发、不报故障
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd h0 = mk(Mode::HOLD, 1, 0);
    S(c, &h0, f, 0);
    CHECK(c.hold_confirm_active(0));
    Decision d;
    int ops = 0;
    for (int k = 1; k <= 10; ++k) {
      f = fb_at(Q0, k * 50 * MS + 20 * MS);
      d = S(c, nullptr, f, k * 50 * MS + 20 * MS);
      ops += int(d.ops.size());
    }
    CHECK(!c.hold_confirm_active(0) && ops == 0 && !(d.fault & bits::LATCHED_MASK));  // 只有看门狗状态位
  }
  {  // 缺回包不算证明：采样断续 → kHoldConfirmMs 后重发
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd h0 = mk(Mode::HOLD, 1, 0);
    S(c, &h0, f, 0);
    int resend = 0;
    for (int k = 1; k * 50 <= cfg::kHoldConfirmMs + 100; ++k) {
      f = fb_at(Q0, k * 50 * MS + 20 * MS);
      if (k % 2) f.valid[0] = false;
      resend += count_ops(S(c, nullptr, f, k * 50 * MS + 20 * MS), BusOp::STOP, 0, 0x11);
    }
    CHECK(resend >= 1);
  }

  // ===== R08：ESTOP 后 50 ms 移动 0.4°（8 °/s）不得报「静止」=====
  // 原：「ESTOP 后全部轴静止（50 ms 内）」
  std::printf("== R08 ESTOP 后 J0 8 °/s 慢移\n");
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    Feedback f = fb_at(q, 0);
    Cmd e = mk(Mode::ESTOP, 1, 0);
    S(c, &e, f, 0);
    bool said_still = false, hold_fail = false;
    for (int k = 1; k <= 30; ++k) {
      int64_t t = k * 50 * MS + 20 * MS;
      q[0] += 0.4f;
      f = fb_at(q, t);
      Decision d = S(c, nullptr, f, t);
      said_still |= has_event(d, "全部保持轴确认停住");
      hold_fail |= (d.fault & bits::HOLD_FAIL) != 0;
      if (k == 1) CHECK(!has_event(d, "静止"));
    }
    std::printf("   报静止=%d HOLD_FAIL=%d\n", int(said_still), int(hold_fail));
    CHECK(!said_still && hold_fail);
  }
  {  // 真静止：确认后报告
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd e = mk(Mode::ESTOP, 1, 0);
    S(c, &e, f, 0);
    bool said = false;
    int k_said = -1;
    for (int k = 1; k <= 10 && !said; ++k) {
      f = fb_at(Q0, k * 50 * MS + 20 * MS);
      said = has_event(S(c, nullptr, f, k * 50 * MS + 20 * MS), "全部保持轴确认停住");
      if (said) k_said = k;
    }
    CHECK(said && k_said * 50 + 20 <= cfg::kHoldConfirmMs);
  }

  // ===== R10：q_target = 最近一次写成功的目标 =====
  std::printf("== R10 q_target 语义\n");
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 32.0f;
    auto tgt = q;
    tgt[6] = 29.0f;
    Feedback f = fb_at(q, 0);
    CHECK(std::isnan(S(c, nullptr, f, 0).q_target[0]));  // 从未下令：NaN
    Cmd s = mk(Mode::STREAM, 1, 0, tgt);
    S(c, &s, f, 10 * MS);
    f = fb_at(q, 60 * MS);
    Decision d = S(c, nullptr, f, 60 * MS);
    CHECK_NEAR(d.q_target[6], 29.0, 1e-4);  // 不用实测 32 冒充
    // 写失败的 0x08 不更新目标、不 ack
    auto tgt2 = tgt;
    tgt2[0] = 2.0f;
    Cmd s2 = mk(Mode::STREAM, 2, 70 * MS, tgt2);
    d = c.step(&s2, f, 70 * MS);
    c.commit(d, std::vector<OpResult>(d.ops.size(), OpResult::FAIL), 70 * MS);
    CHECK_NEAR(d.q_target[0], 0.0, 1e-4);
    CHECK(d.ack_seq == 1 && (d.fault & bits::TX_FAIL));
    // RELEASE → NaN
    Core c2(P);
    c2.on_startup_done(false);
    Cmd r = mk(Mode::RELEASE, 1, 0, Q0, 1u << 6);
    d = S(c2, &r, f, 0);
    CHECK(std::isnan(d.q_target[6]));
    CHECK_NEAR(d.q_target[0], 0.0, 1e-4);  // mask 外的轴 HOLD：发 0x18 时的实测
  }

  // ===== R04：只让 J0 的 0x18 写失败，HOLD 后运行 350 ms =====
  // 原：write_fail_HOLD: ack=42 fault=1(WATCHDOG) J0_stop_count=0
  std::printf("== R04 J0 停止帧写失败（真主循环）\n");
  {
    FakeSerial fs;
    FailSerial ser(fs);
    Loop L(ser);
    std::string err;
    CHECK(L.ctrl.startup(err));
    L.start();
    sleep_ms(120);
    ser.fail_stop_axis = 0;
    L.put(Mode::HOLD, 42);
    sleep_ms(350);
    StateMsg s = L.last();
    int att = ser.stop_attempts.load();
    std::printf("   ack=%u fault=0x%x J0 0x18 写尝试=%d（真到舵机 %d）\n", s.cmd_seq_ack, s.fault, att,
                fs.view(0).stop_count);
    CHECK(s.cmd_seq_ack != 42);
    CHECK((s.fault & bits::TX_FAIL) && s.ctrl_state == uint8_t(CtrlState::FAULT));
    CHECK(att > 1 + cfg::kTxRetries);  // 本周期 3 次 + 之后每周期重发
    CHECK(L.ctrl.core().tx_pending(0));
    for (int i = 1; i < N; ++i) CHECK(fs.view(i).mode == FakeSerial::SMode::HOLD);
    // 写恢复 → 停止帧补发成功、RESET 可清 TX_FAIL
    ser.fail_stop_axis = -1;
    sleep_ms(150);
    CHECK(!L.ctrl.core().tx_pending(0) && fs.view(0).mode == FakeSerial::SMode::HOLD);
    L.put(Mode::RESET, 43);
    sleep_ms(150);
    s = L.last();
    CHECK(!(s.fault & bits::TX_FAIL) && s.cmd_seq_ack == 43);
    L.halt();
  }

  // ===== R02 端到端：合法校验和 + 非法温度 ADC =====
  std::printf("== R02 端到端（FakeSerial J6 ADC=0、J3 ADC=4096）\n");
  {
    FakeSerial fs;
    Loop L(fs);
    std::string err;
    CHECK(L.ctrl.startup(err));
    L.start();
    L.put(Mode::HOLD, 1);
    sleep_ms(150);
    fs.inject_temp_adc(6, 0);
    sleep_ms(400);
    StateMsg s = L.last();
    std::printf("   J6 ADC=0：valid_mask=0x%02x fault=0x%x J6=%s\n", s.valid_mask, s.fault,
                fs.view(6).mode == FakeSerial::SMode::RELEASED ? "释放" : "未释放");
    CHECK(!(s.valid_mask & 0x40) && (s.fault & bits::J6_THERMAL) && (s.fault & bits::COMM_LOSS));
    CHECK(fs.view(6).mode == FakeSerial::SMode::RELEASED);
    fs.inject_temp_adc(3, 4096);
    sleep_ms(300);
    s = L.last();
    CHECK(!(s.valid_mask & 0x08));
    for (int i = 0; i < N - 1; ++i) CHECK(fs.view(i).mode == FakeSerial::SMode::HOLD);  // J0–J5 不释放
    L.halt();
  }

  // ===== R11：上一轮迟到的回包不得算作本轮新鲜数据 =====
  std::printf("== R11 跨周期迟到帧\n");
  {
    FakeSerial fs;
    Bus bus(fs);
    std::vector<uint8_t> ids = {0, 1, 2, 3, 4, 5, 6};
    fs.inject_reply_delay_ms(2, 45.0);  // J2 回包 ≈ 请求后 52 ms 才到（> 40 ms 超时）
    SyncMonitorResult a = bus.sync_monitor(ids, INT64_MAX, 40 * MS, 40 * MS);
    CHECK(!a.got[2] && a.got[0]);
    bus.send(frame::ping(0));  // 再插一帧：下一次读请求要先睡帧间隔，迟到帧正好落在这段时间
    SyncMonitorResult b = bus.sync_monitor(ids, INT64_MAX, 40 * MS, 40 * MS);
    std::printf("   第二轮 J2 got=%d；rx_late=%llu flushed=%llu\n", int(b.got[2]),
                (unsigned long long)bus.stats().rx_late, (unsigned long long)bus.stats().flushed_bytes);
    CHECK(!b.got[2]);  // 本轮 J2 的真回包也迟到；收到的只能是上一轮的
    CHECK(bus.stats().rx_late + bus.stats().flushed_bytes > 0);
    fs.inject_reply_delay_ms(2, 0.0);
    sleep_ms(80);
    SyncMonitorResult c = bus.sync_monitor(ids, INT64_MAX, 40 * MS, 40 * MS);
    CHECK(c.got[2]);
  }

  // ===== R09：串口永久写不出去时 write_all 有期限 =====
  std::printf("== R09 pty 写满（对端不读）\n");
  {
    int m = posix_openpt(O_RDWR | O_NOCTTY);
    bool ok = m >= 0 && grantpt(m) == 0 && unlockpt(m) == 0;
    const char* name = ok ? ptsname(m) : nullptr;
    PosixSerial ps;
    std::string err;
    ok = ok && name && ps.open(name, 115200, err);
    CHECK(ok);
    if (ok) {
      std::vector<uint8_t> big(256 * 1024, 0x55);
      int64_t t0 = now_mono_ns();
      bool w = ps.write_all(big.data(), big.size());
      double ms = double(now_mono_ns() - t0) / 1e6;
      std::printf("   256 KiB 写入：返回 %s，用时 %.1f ms（期限 %d ms）\n", w ? "成功" : "失败", ms, cfg::kTxDeadlineMs);
      CHECK(!w && ms < cfg::kTxDeadlineMs + 30);
    }
    if (m >= 0) close(m);
  }

  // ===== V2-01：排空全程有界（不支持 / 返回 0 但发送器未空 / 停滞）=====
  std::printf("== V2-01 有界排空（注入 ioctl）\n");
  {
    struct Case {
      const char* name;
      int outq, temt;
      bool expect_ok;
    } cases[] = {
        {"正常（队列 0、发送器空）", 0, 0, true},
        {"TIOCOUTQ 不支持 → 估算", 1, 0, true},
        {"TIOCOUTQ=0 但发送器一直不空", 0, 2, false},
        {"TIOCOUTQ 一直不为 0（停滞）", 2, 0, false},
        {"TIOCSERGETLSR 不支持 → 估算", 0, 1, true},
    };
    for (auto& c : cases) {
      FakeTty ft;
      ft.outq_mode = c.outq;
      ft.temt_mode = c.temt;
      int64_t t0 = now_mono_ns();
      int64_t dl = t0 + cfg::kTxDeadlineMs * MS;
      bool ok = drain_bounded(ft, -1, 55, t0, dl);
      double late_ms = double(now_mono_ns() - dl) / 1e6;
      double used = double(now_mono_ns() - t0) / 1e6;
      std::printf("   %-28s → %s，用时 %.2f ms\n", c.name, ok ? "发完" : "失败", used);
      CHECK(ok == c.expect_ok);
      CHECK(late_ms <= 2.0);  // 总期限 + 2 ms 内返回
    }
    // 估算超过期限：直接失败，不睡
    FakeTty ft;
    ft.outq_mode = 1;
    int64_t t0 = now_mono_ns();
    CHECK(!drain_bounded(ft, -1, 4096, t0, t0 + cfg::kTxDeadlineMs * MS));
    CHECK(now_mono_ns() - t0 < 2 * MS);
  }
  {  // 走完整 write_all：pty + 注入「发送器一直不空」
    int m = posix_openpt(O_RDWR | O_NOCTTY);
    bool ok = m >= 0 && grantpt(m) == 0 && unlockpt(m) == 0;
    const char* name = ok ? ptsname(m) : nullptr;
    PosixSerial ps;
    std::string err;
    ok = ok && name && ps.open(name, 115200, err);
    CHECK(ok);
    if (ok) {
      FakeTty ft;
      ft.temt_mode = 2;
      ps.set_tty_query(&ft);
      frame::Bytes f = frame::ping(1);
      int64_t t0 = now_mono_ns();
      bool w = ps.write_all(f.data(), f.size());
      double ms = double(now_mono_ns() - t0) / 1e6;
      std::printf("   write_all + 发送器停滞：%s，用时 %.1f ms\n", w ? "成功" : "失败", ms);
      CHECK(!w && ms <= cfg::kTxDeadlineMs + 2);
      ft.temt_mode = 0;
      CHECK(ps.write_all(f.data(), f.size()));
    }
    if (m >= 0) close(m);
  }

  // ===== V2-02：七轴 STOP 批次发出第 1 帧后 ESTOP 到达（含第 1 帧写失败重试中）=====
  std::printf("== V2-02 STOP 批次中 ESTOP 抢占\n");
  for (int fail = 0; fail <= 1; ++fail) {
    double worst = 0;
    int worst_stale = 0, ok_runs = 0;
    const int reps = 5;
    for (int k = 0; k < reps; ++k) {
      FakeSerial fs;
      EstopInjectSerial ser(fs);
      ser.fail_first_stop = fail != 0;
      Loop L(ser);
      ser.mb = &L.mb;
      std::string err;
      if (!L.ctrl.startup(err)) continue;
      L.start();
      sleep_ms(80 + 7 * k);  // 错开周期相位
      ser.armed = true;
      L.put(Mode::HOLD, 1);  // IDLE 下 HOLD → 七轴 0x18 0x11 批次
      for (int w = 0; w < 100 && !ser.freeze_start; ++w) sleep_ms(2);
      sleep_ms(200);  // 冻结帧后还有 7 帧保持 + 一次读回才发布状态
      StateMsg s = L.last();
      L.halt();
      if (!ser.freeze_start) continue;
      ++ok_runs;
      double ms = double(ser.freeze_start - ser.estop_rx) / 1e6;
      worst = std::max(worst, ms);
      worst_stale = std::max(worst_stale, ser.stops_after_estop.load());
      CHECK(s.fault & bits::ESTOP);
      for (int i = 0; i < N; ++i) CHECK(fs.view(i).mode == FakeSerial::SMode::HOLD);
    }
    std::printf("   %s：%d 次，ESTOP 收到 → 冻结帧开始 最坏 %.2f ms；ESTOP 后旧批次又发出的停止帧 最多 %d\n",
                fail ? "第 1 帧写失败重试中" : "第 1 帧已发出", ok_runs, worst, worst_stale);
    CHECK(ok_runs == reps);
    CHECK(worst_stale == 0);
    CHECK(worst < 2.0 * cfg::kCmdGapMs);  // 原实现：其余 6 帧 ×（10 ms 间隔 + 线上时长）≈ 70 ms 之后
  }

  // ===== F63：多圈只按角度判（turns = floor(角度/360)，负角度 turn=−1 正常）=====
  std::printf("== F63 多圈判据只看角度\n");
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[1] = -90.0f;  // HOME J1
    q[5] = -0.4f;   // TC-004 J5
    Feedback f = fb_at(q, 0);
    for (int i = 0; i < N; ++i) f.ax[i].turns = q[i] < 0 ? -1 : 0;  // 按 F63 填真实圈数
    Cmd s = mk(Mode::STREAM, 1, 0, q);
    Decision d = S(c, &s, f, 0);
    CHECK(!(d.fault & bits::MTURN) && d.has_move() && c.state() == CtrlState::STREAMING);
    for (int k = 1; k <= 5; ++k) {
      f = fb_at(q, k * 50 * MS);
      for (int i = 0; i < N; ++i) f.ax[i].turns = q[i] < 0 ? -1 : 0;
      Cmd sk = mk(Mode::STREAM, 1 + k, k * 50 * MS, q);
      d = S(c, &sk, f, k * 50 * MS);
    }
    std::printf("   J1 −90°/J5 −0.4°（turn=−1）：fault=0x%x state=%d\n", d.fault, int(c.state()));
    CHECK(!(d.fault & bits::MTURN) && c.state() == CtrlState::STREAMING);
    // q = 388.6、turns = 1（J2）→ MTURN
    f = fb_at(q, 400 * MS);
    f.ax[2].q = 388.6f;
    f.ax[2].turns = 1;
    d = S(c, nullptr, f, 400 * MS);
    CHECK((d.fault & bits::MTURN) && !released_arm(d));
    // RESET：七轴回到范围（J1/J5 仍 turn=−1）→ MTURN 可清
    f = fb_at(q, 450 * MS);
    for (int i = 0; i < N; ++i) f.ax[i].turns = q[i] < 0 ? -1 : 0;
    Cmd r = mk(Mode::RESET, 20, 450 * MS);
    d = S(c, &r, f, 450 * MS);
    CHECK(!(d.fault & bits::MTURN));
  }
  {  // 启动：J1 −90°（仿真圈数 −1）→ 不判超限、不发 0x11
    FakeSerial fs;
    fs.inject_mturn(1, -90.0f);
    Mailbox mb;
    Controller ctrl(fs, mb, CtrlOptions{P, false}, nullptr);
    std::string err;
    CHECK(ctrl.startup(err));
    CHECK(ctrl.core().state() == CtrlState::IDLE);
    for (int i = 0; i < N; ++i) CHECK(fs.view(i).clear_count == 0);
  }
  {  // 启动：J1 −450°（释放态）→ 只对 J1 发 0x11，清后 −90° 在范围
    FakeSerial fs;
    fs.inject_mturn(1, -450.0f);
    Mailbox mb;
    Controller ctrl(fs, mb, CtrlOptions{P, false}, nullptr);
    std::string err;
    CHECK(ctrl.startup(err));
    CHECK(ctrl.core().state() == CtrlState::IDLE);
    CHECK(fs.view(1).clear_count == 1 && std::fabs(fs.view(1).pos_deg + 90.0f) < 0.05f);
    for (int i = 0; i < N; ++i)
      if (i != 1) CHECK(fs.view(i).clear_count == 0);
  }

  // ===== TC-010 实机①：看门狗只在 STREAMING 下起作用 =====
  std::printf("== TC-010 看门狗仅 STREAMING\n");
  {  // RELEASE J0–J5 后 5 s 不发命令：轴保持释放，无 WATCHDOG、无任何停止帧
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0, 0);
    Cmd r = mk(Mode::RELEASE, 1, 0, Q0, 0x3F);
    Decision d = S(c, &r, f, 0);
    CHECK(count_ops(d, BusOp::STOP, -1, 0x10) == 6);
    int ops = 0;
    bool wd = false;
    for (int k = 1; k <= 100; ++k) {
      int64_t t = k * 50 * MS;
      d = S(c, nullptr, fb_at(Q0, t), t);
      ops += int(d.ops.size());
      wd |= (d.fault & bits::WATCHDOG) || has_event(d, "看门狗");
    }
    std::printf("   RELEASE 后 5 s：停止帧 %d 个，WATCHDOG=%d\n", ops, int(wd));
    CHECK(ops == 0 && !wd && !c.watchdog_tripped());
    for (int i = 0; i < N - 1; ++i) CHECK(c.axis_mode(i) == Core::Ax::RELEASED);
  }
  {  // HOLD 后 5 s 不发命令：不再报看门狗超时
    Core c(P);
    c.on_startup_done(false);
    Cmd h = mk(Mode::HOLD, 1, 0);
    S(c, &h, fb_at(Q0, 0), 0);
    bool wd = false;
    for (int k = 1; k <= 100; ++k) {
      int64_t t = k * 50 * MS;
      Decision d = S(c, nullptr, fb_at(Q0, t), t);
      wd |= (d.fault & bits::WATCHDOG) || has_event(d, "看门狗");
    }
    CHECK(!wd);
  }
  {  // STREAM 中停发：200 ms 后照常保持
    Core c(P);
    c.on_startup_done(false);
    Cmd s0 = mk(Mode::STREAM, 1, 0);
    S(c, &s0, fb_at(Q0, 0), 0);
    CHECK(c.state() == CtrlState::STREAMING);
    int k_trip = -1;
    Decision d;
    for (int k = 1; k <= 10 && k_trip < 0; ++k) {
      int64_t t = k * 50 * MS;
      d = S(c, nullptr, fb_at(Q0, t), t);
      if (d.fault & bits::WATCHDOG) k_trip = k;
    }
    std::printf("   STREAM 停发 → WATCHDOG 于 %d ms\n", k_trip * 50);
    CHECK(k_trip * 50 > cfg::kWatchdogMs && k_trip * 50 <= cfg::kWatchdogMs + 50);
    CHECK(c.state() == CtrlState::HOLD && count_ops(d, BusOp::STOP, -1, 0x11) == N);
  }

  // ===== TC-010 实机②：停住判据 = 窗口范围（静止读数抖动不误报，慢移仍报）=====
  std::printf("== TC-010 停住判据（窗口 %d 采样，范围 ≤ %.1f°）\n", cfg::kStillWindow, cfg::kStillRangeDeg);
  {  // 静止 + 读数抖动：按 A_readonly2 分布，J0/J1/J2/J5 每帧在 base 附近跳 −0.1 / 0 / +0.1 / +0.2（窗口范围 ≤ 0.3°）
    uint32_t lcg = 12345;
    auto rnd = [&lcg]() { lcg = lcg * 1103515245u + 12345u; return (lcg >> 16) & 0x7fff; };
    int trials = 0, resends = 0, fails = 0, unconfirmed = 0;
    for (int trial = 0; trial < 200; ++trial) {
      Core c(P);
      c.on_startup_done(false);
      Cmd h = mk(Mode::HOLD, 1, 0);
      S(c, &h, fb_at(Q0, 0), 0);
      for (int k = 1; k <= 40; ++k) {
        int64_t t = k * 50 * MS + 20 * MS;
        auto q = Q0;
        for (int j : {0, 1, 2, 5}) {
          int r = rnd() % 100;  // 约 60% 不跳，35% ±0.1，5% +0.2
          q[j] += r < 60 ? 0.0f : r < 78 ? 0.1f : r < 95 ? -0.1f : 0.2f;
        }
        Decision d = S(c, nullptr, fb_at(q, t), t);
        resends += count_ops(d, BusOp::STOP, -1, 0x11);
        fails += (d.fault & bits::HOLD_FAIL) ? 1 : 0;
      }
      for (int i = 0; i < N; ++i) unconfirmed += c.hold_confirm_active(i);
      ++trials;
    }
    std::printf("   静止抖动 %d 次试验：重发 %d、HOLD_FAIL %d、未确认 %d\n", trials, resends, fails, unconfirmed);
    CHECK(resends == 0 && fails == 0 && unconfirmed == 0);
  }
  for (float dps : {2.0f, 8.0f}) {  // 匀速慢移：必须报 HOLD_FAIL
    Core c(P);
    c.on_startup_done(false);
    Cmd h = mk(Mode::HOLD, 1, 0);
    S(c, &h, fb_at(Q0, 0), 0);
    int resend = 0, k_fail = -1;
    Decision d;
    for (int k = 1; k <= 40; ++k) {
      int64_t t = k * 50 * MS + 20 * MS;
      auto q = Q0;
      q[0] += std::round(dps * 0.05f * k * 10.0f) / 10.0f;  // 读数按 0.1° 量化
      d = S(c, nullptr, fb_at(q, t), t);
      resend += count_ops(d, BusOp::STOP, 0, 0x11);
      if ((d.fault & bits::HOLD_FAIL) && k_fail < 0) k_fail = k;
    }
    std::printf("   J0 %.0f °/s 慢移：重发 %d 次，HOLD_FAIL 于 %d ms\n", dps, resend, k_fail * 50 + 20);
    CHECK(resend == cfg::kHoldResendMax && k_fail > 0 && !released_arm(d));
  }
  {  // RESET 清 HOLD_FAIL 用同一判据：抖动静止可清，仍在 2 °/s 移动不可清
    for (int moving = 0; moving < 2; ++moving) {
      Core c(P);
      c.on_startup_done(false);
      Cmd h = mk(Mode::HOLD, 1, 0);
      S(c, &h, fb_at(Q0, 0), 0);
      auto q = Q0;
      int k = 1;
      Decision d;
      for (; k <= 40 && !(c.latched() & bits::HOLD_FAIL); ++k) {  // 8 °/s 慢移制造 HOLD_FAIL
        q[0] += 0.4f;
        d = S(c, nullptr, fb_at(q, k * 50 * MS), k * 50 * MS);
      }
      CHECK(c.latched() & bits::HOLD_FAIL);
      for (int n = 0; n < 12; ++n, ++k) {
        auto qq = q;
        if (moving) q[0] += 0.1f, qq = q;
        else qq[0] += (n % 3 == 0) ? 0.1f : (n % 3 == 1) ? -0.1f : 0.0f;  // 静止抖动
        d = S(c, nullptr, fb_at(qq, k * 50 * MS), k * 50 * MS);
      }
      Cmd rs = mk(Mode::RESET, 99, k * 50 * MS);
      d = S(c, &rs, fb_at(q, k * 50 * MS), k * 50 * MS);
      std::printf("   RESET（J0 %s）：HOLD_FAIL %s\n", moving ? "仍 2 °/s 移动" : "静止抖动",
                  (c.latched() & bits::HOLD_FAIL) ? "保留" : "清除");
      CHECK(bool(c.latched() & bits::HOLD_FAIL) == bool(moving));
    }
  }

  // ===== TC-010 实机③：J6 降出力 = 进入时锁定目标 + 43 ℃ 回差 =====
  std::printf("== TC-010 J6 降出力锁定 + 回差\n");
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 30.2f;  // 夹住瓶子停在 30.2°
    auto tgt = q;
    tgt[6] = 27.0f;  // STREAM 一直往 27° 压（E_thermal3 现场）
    int64_t t = 0;
    uint32_t seq = 1;
    auto run = [&](float temp) {
      t += 50 * MS;
      Feedback f = fb_at(q, t);
      f.ax[6].temp_c = temp;
      Cmd s = mk(Mode::STREAM, ++seq, t, tgt);
      return S(c, &s, f, t);
    };
    Decision d = run(40.0f);
    CHECK_NEAR(d.move_q[6], 27.0, 1e-4);
    d = run(45.0f);  // 进入：锁定 30.2
    CHECK(c.j6_derate_on() && (d.warn & bits::J6_HOT) && has_event(d, "进入降出力"));
    CHECK_NEAR(d.move_q[6], 30.2, 1e-4);
    // 瓶子回弹：实测每周期往张开方向 +0.2°（0.4°/100 ms），共 20 周期 → 目标必须停在 30.2，不跟着张开
    float max_tgt = 0;
    for (int k = 0; k < 20; ++k) {
      q[6] += 0.2f;
      d = run(46.0f);
      max_tgt = std::max(max_tgt, d.move_q[6]);
      CHECK(!has_event(d, "进入降出力"));
    }
    std::printf("   回弹：实测 30.2→%.1f°，降出力期间 J6 目标最大 %.1f°（锁定 %.1f°）\n", q[6], max_tgt, c.j6_derate_tgt());
    CHECK_NEAR(max_tgt, 30.2, 1e-4);
    CHECK_NEAR(c.bus_target(6), 30.2, 1e-4);
    // 无新命令（只剩 J6 降出力自己发帧）时也不跟随实测
    {
      t += 50 * MS;
      q[6] += 0.4f;
      Feedback f = fb_at(q, t);
      f.ax[6].temp_c = 46.0f;
      d = S(c, nullptr, f, t);
      CHECK(!d.move_axis[6]);
      CHECK_NEAR(c.bus_target(6), 30.2, 1e-4);
    }
    // 温度在 44.8–45.2 抖动 30 s：进出切换只有进入那 1 次（已进入 → 0 次新切换）
    int enters = 0, exits = 0;
    const float jit[] = {44.8f, 45.2f, 44.9f, 45.0f, 45.1f, 44.8f, 45.0f, 44.9f};
    for (int k = 0; k < 600; ++k) {
      d = run(jit[k % 8]);
      enters += has_event(d, "进入降出力");
      exits += has_event(d, "退出降出力");
      CHECK(d.move_q[6] >= 30.2f - 1e-4f);
    }
    std::printf("   44.8–45.2 ℃ 抖动 30 s：进入 %d 次、退出 %d 次\n", enters, exits);
    CHECK(enters == 0 && exits == 0 && c.j6_derate_on());
    // 43.5 ℃ 仍在降出力；43.0 ℃ 退出，下一帧恢复跟随 STREAM 27°
    d = run(43.5f);
    CHECK(c.j6_derate_on());
    d = run(43.0f);
    CHECK(!c.j6_derate_on() && has_event(d, "退出降出力") && !(d.warn & bits::J6_HOT));
    std::printf("   43.0 ℃ 退出后 J6 目标 %.1f°\n", d.move_q[6]);
    CHECK_NEAR(d.move_q[6], 27.0, 1e-4);
  }
  {  // 从 cold 一路抖动：44.8–45.2 ℃ 30 s 只进入 1 次
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 30.0f;
    auto tgt = q;
    tgt[6] = 27.0f;
    int enters = 0, exits = 0;
    const float jit[] = {44.8f, 45.2f, 44.9f, 45.0f, 45.1f, 44.8f, 45.0f, 44.9f};
    for (int k = 0; k < 600; ++k) {
      int64_t t = k * 50 * MS;
      Feedback f = fb_at(q, t);
      f.ax[6].temp_c = jit[k % 8];
      Cmd s = mk(Mode::STREAM, 1 + k, t, tgt);
      Decision d = S(c, &s, f, t);
      enters += has_event(d, "进入降出力");
      exits += has_event(d, "退出降出力");
    }
    std::printf("   冷启动后 44.8–45.2 ℃ 抖动 30 s：进入 %d 次、退出 %d 次\n", enters, exits);
    CHECK(enters == 1 && exits == 0);
  }
  {  // 更张开的 STREAM 目标照常执行并更新锁定值；之后更闭合的目标钳到新锁定值
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 30.0f;
    Feedback f = fb_at(q, 0);
    f.ax[6].temp_c = 46.0f;
    auto tgt = q;
    tgt[6] = 27.0f;
    Cmd s1 = mk(Mode::STREAM, 1, 0, tgt);
    Decision d = S(c, &s1, f, 0);
    CHECK_NEAR(d.move_q[6], 30.0, 1e-4);
    tgt[6] = 32.0f;
    f = fb_at(q, 50 * MS);
    f.ax[6].temp_c = 46.0f;
    Cmd s2 = mk(Mode::STREAM, 2, 50 * MS, tgt);
    d = S(c, &s2, f, 50 * MS);
    CHECK_NEAR(d.move_q[6], 32.0, 1e-4);
    CHECK_NEAR(c.j6_derate_tgt(), 32.0, 1e-4);
    tgt[6] = 28.0f;
    f = fb_at(q, 100 * MS);
    f.ax[6].temp_c = 46.0f;
    Cmd s3 = mk(Mode::STREAM, 3, 100 * MS, tgt);
    d = S(c, &s3, f, 100 * MS);
    std::printf("   张开到 32° 后再给 28°：J6 目标 %.1f°（锁定 %.1f°）\n", d.move_q[6], c.j6_derate_tgt());
    CHECK_NEAR(d.move_q[6], 32.0, 1e-4);
  }
  {  // HOLD 中进入降出力：发一次 0x08 到锁定角；之后回弹 0.4° 也不再跟随
     // （持续顶开属于「保持后未停住」，由停住确认重发 0x18 / HOLD_FAIL 处理，不在本用例）
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 29.0f;
    auto tgt = q;
    tgt[6] = 27.0f;
    Cmd s = mk(Mode::STREAM, 1, 0, tgt);
    S(c, &s, fb_at(q, 0), 0);  // 总线目标 27
    Cmd h = mk(Mode::HOLD, 2, 50 * MS);
    S(c, &h, fb_at(q, 50 * MS), 50 * MS);  // 保持在实测 29
    q[6] = 30.0f;
    Feedback f = fb_at(q, 100 * MS);
    f.ax[6].temp_c = 45.5f;
    Decision d = S(c, nullptr, f, 100 * MS);
    CHECK(d.move_axis[6]);
    CHECK_NEAR(d.move_q[6], 30.0, 1e-4);
    int moves = 0;
    q[6] = 30.4f;
    for (int k = 3; k < 23; ++k) {
      f = fb_at(q, k * 50 * MS);
      f.ax[6].temp_c = 46.0f;
      moves += S(c, nullptr, f, k * 50 * MS).move_axis[6];
    }
    CHECK(moves == 0);
    CHECK_NEAR(c.bus_target(6), 30.0, 1e-4);
  }

  return chk::report("test_regress");
}
