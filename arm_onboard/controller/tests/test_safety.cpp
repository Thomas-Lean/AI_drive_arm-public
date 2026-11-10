// test_safety.cpp —— safety 单测（任务书 §4 T3 验收 2）
// 覆盖：限位、步长、多圈（J6 自动处理 / J0–J5 拒绝）、状态位分类（J6 bit6 只告警）、
//      温度两档、看门狗恢复的 TARGET_FAR；另加 ESTOP / RESET / 命令合法性 / 「绝不自动释放 J0–J5」。
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>

#include "check.hpp"
#include "safety/safety.hpp"

using namespace d1t;
using namespace d1t::safety;

static const int P = 50;
static const int64_t MS = 1000000LL;

static Feedback fb_at(std::array<float, N> q) {
  Feedback f;
  for (int i = 0; i < N; ++i) {
    f.valid[i] = f.have[i] = true;
    f.ax[i].q = q[i];
    f.ax[i].temp_c = 30.0f;
    f.ax[i].current_ma = 14;
  }
  return f;
}
static const std::array<float, N> Q0 = {0, -60, 60, 0, 10, 0, 40};

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
  c.rx_no = seq;  // 测试里接收序号 = seq（信箱里同样递增）
  return c;
}

// 一个周期：step + 全部帧写成功后 commit（写失败路径见 test_regress）
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
// J0–J5 有没有被释放（0x18 0x10）——控制器绝不自动释放 J0–J5
static bool released_arm(const Decision& d) {
  for (auto& o : d.ops)
    if (o.kind == BusOp::STOP && o.mode == 0x10 && o.axis < N - 1) return true;
  return false;
}

int main() {
  // ================= 纯函数 =================
  // 限位（F7）
  CHECK(clamp_limit(1, -95.0f).q == -90.3f && clamp_limit(1, -95.0f).clamped);
  CHECK(clamp_limit(6, 70.0f).q == 64.5f);
  CHECK(!clamp_limit(0, 100.0f).clamped);
  CHECK(out_of_limit(1, -90.4f));   // F44 J1 -90.4 读数
  CHECK(out_of_limit(5, 162.6f));   // F61 J5 162.6 读数
  CHECK(!out_of_limit(5, 162.0f));
  // 步长：50 ms 时各轴 5.0°（100°/s）
  CHECK_NEAR(max_step_deg(0, 50), 5.0, 1e-4);
  CHECK_NEAR(max_step_deg(3, 50), 5.0, 1e-4);
  CHECK(clamp_step(0, 20.0f, 0.0f, 50).clamped);
  CHECK_NEAR(clamp_step(0, 20.0f, 0.0f, 50).q, 5.0, 1e-4);
  CHECK_NEAR(clamp_step(4, -20.0f, 0.0f, 50).q, -5.0, 1e-4);
  CHECK(!clamp_step(0, 2.0f, 0.0f, 50).clamped);
  // 多圈
  CHECK(mturn_out(180.0f));
  CHECK(mturn_out(-180.0f));
  CHECK(!mturn_out(28.6f));
  CHECK(mturn_out(388.6f));  // F58 事故读数（turns = 1）
  CHECK(!mturn_out(179.9f));
  CHECK(!mturn_out(-0.4f));   // F63：TC-004 J5 −0.4° 读 turn=−1，正常
  CHECK(!mturn_out(-90.0f));  // F63：HOME J1 −90°（turn=−1）不得误报
  // 状态位
  CHECK(classify_status(6, 0x41).j6_warn && !classify_status(6, 0x41).fault);  // F60 J6 0x41 只告警
  CHECK(classify_status(6, 0x04).j6_warn && !classify_status(6, 0x04).fault);
  CHECK(classify_status(2, 0x40).fault);  // J0–J5 bit6 → 故障
  CHECK(classify_status(2, 0x04).fault);  // J0–J5 bit2 → 故障
  for (int b : {3, 4, 5, 7}) CHECK(classify_status(6, uint8_t(1 << b)).fault);  // 任一轴 bit3/4/5/7
  CHECK(!classify_status(3, 0x03).fault && !classify_status(3, 0x03).j6_warn);    // bit0/1 不管
  // 温度两档
  CHECK(j6_temp_level(44.9f) == J6Temp::OK);
  CHECK(j6_temp_level(45.0f) == J6Temp::HOT);
  CHECK(j6_temp_level(49.9f) == J6Temp::HOT);
  CHECK(j6_temp_level(50.0f) == J6Temp::RELEASE);  // §3.7-9：50 ℃
  CHECK(j6_temp_level(std::numeric_limits<float>::quiet_NaN()) == J6Temp::UNKNOWN);  // §3.7-3
  // 命令合法性
  CHECK(cmd_valid(mk(Mode::STREAM, 1, 1)));
  CHECK(!cmd_valid(mk(Mode::RELEASE, 1, 1, Q0, 0)));      // axis_mask=0 拒收
  CHECK(!cmd_valid(mk(Mode::DAMP, 1, 1, Q0, 0x80)));      // 只含 J0–J6
  {
    Cmd c = mk(Mode::STREAM, 1, 1);
    c.q[3] = std::numeric_limits<float>::quiet_NaN();
    CHECK(!cmd_valid(c));
    c.mode = 9;
    CHECK(!cmd_valid(c));
  }
  CHECK(clamp_dt(10, 50) == 50 && clamp_dt(75, 50) == 75 && clamp_dt(500, 50) == 150);
  CHECK(clamp_grip_power(9000) == 6000 && clamp_grip_power(0) == 0);

  // ================= 状态机 =================
  // --- 流式：进入时基准取实测、步长 / 限位钳制、warn 位 ---
  {
    Core c(P);
    c.on_startup_done(false);
    CHECK(c.state() == CtrlState::IDLE);
    Feedback f = fb_at(Q0);
    Decision d = S(c, nullptr, f, 0);
    CHECK(d.ops.empty());  // IDLE 不发任何帧
    auto q = Q0;
    q[0] = 20.0f;  // 远目标 → 步长钳
    q[1] = -100.0f;  // 超下限；步长先钳到 -65.0，未超限
    d = S(c, &*std::make_unique<Cmd>(mk(Mode::STREAM, 1, 10 * MS, q)), f, 10 * MS);
    CHECK(c.state() == CtrlState::STREAMING);
    CHECK(count_ops(d, BusOp::MOVE) == 1);
    CHECK_NEAR(d.move_q[0], 5.0, 1e-4);
    CHECK_NEAR(d.move_q[1], -65.0, 1e-4);
    CHECK(d.warn & bits::STEP_CLAMPED);
    CHECK(d.move_time_ms == 50);
    CHECK(d.ack_seq == 1);
    // 一直推 J1 → 最终被 F7 下限 -90.3 钳住
    for (int k = 2; k < 20; ++k) d = S(c, &*std::make_unique<Cmd>(mk(Mode::STREAM, k, k * 10 * MS, q)), f, k * 10 * MS);
    CHECK_NEAR(d.move_q[1], -90.3, 1e-4);
    CHECK(d.warn & bits::LIMIT_CLAMPED);
    // J6 功率钳 6000，J0–J5 power 0
    Cmd g = mk(Mode::STREAM, 30, 300 * MS, q);
    g.grip_power_mw = 9000;
    d = S(c, &g, f, 300 * MS);
    CHECK(d.move_power[6] == 6000 && d.move_power[0] == 0);
  }

  // --- 看门狗：超时保持；远目标 TARGET_FAR；近目标恢复 ---
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    Decision d = S(c, &s, f, 0);
    d = S(c, nullptr, f, 150 * MS);
    CHECK(!c.watchdog_tripped());
    d = S(c, nullptr, f, 201 * MS);
    CHECK(c.watchdog_tripped() && c.state() == CtrlState::HOLD);
    CHECK(count_ops(d, BusOp::STOP, -1, 0x11) == 7);
    CHECK(d.fault & bits::WATCHDOG);
    auto far = Q0;
    far[2] += 6.0f;  // > RESUME_TOL 5°
    Cmd s2 = mk(Mode::STREAM, 2, 250 * MS, far);
    d = S(c, &s2, f, 250 * MS);
    CHECK((d.warn & bits::TARGET_FAR) && c.state() == CtrlState::HOLD && !d.has_move());
    CHECK(d.ack_seq == 1);  // 未作用到总线，不 ack
    auto near = Q0;
    near[2] += 4.0f;
    Cmd s3 = mk(Mode::STREAM, 3, 300 * MS, near);
    d = S(c, &s3, f, 300 * MS);
    CHECK(c.state() == CtrlState::STREAMING && !c.watchdog_tripped() && d.has_move());
    CHECK(!(d.fault & bits::WATCHDOG));
    CHECK(d.ack_seq == 3);
  }
  // 看门狗只在收到第一条合法命令后启用（IDLE 只读阶段不锁轴）
  {
    Core c(P);
    c.on_startup_done(false);
    Decision d = S(c, nullptr, fb_at(Q0), 10000 * MS);
    CHECK(!c.watchdog_tripped() && d.ops.empty());
  }

  // --- 多圈：J0–J5 超限 → MTURN 故障、全轴保持、不释放 ---
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    f.ax[3].q = -200.0f;  // 按角度构造超限（F63：turns 不作判据）
    f.ax[3].turns = -1;
    Cmd s2 = mk(Mode::STREAM, 2, 50 * MS);
    Decision d = S(c, &s2, f, 50 * MS);
    CHECK((d.fault & bits::MTURN) && c.state() == CtrlState::FAULT);
    CHECK(!d.has_move());
    CHECK(count_ops(d, BusOp::STOP, -1, 0x11) == 7);
    CHECK(!released_arm(d));
    CHECK(d.warn & bits::CMD_REJECTED);  // 锁存期间拒绝运动
  }
  // --- 多圈：J6 运行中超限 → J6 退出 0x08、自动释放 + 0x11；恢复后继续 ---
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    f.ax[6].q = 388.6f;  // F58
    f.ax[6].turns = 1;
    Cmd s2 = mk(Mode::STREAM, 2, 50 * MS);
    Decision d = S(c, &s2, f, 50 * MS);
    CHECK(c.j6_mturn_pending() && !(d.fault & bits::MTURN));
    // 分阶段（R05）：本周期只释放，不在同一周期里等 100 ms 再清圈
    CHECK(count_ops(d, BusOp::STOP, 6, 0x10) == 1 && count_ops(d, BusOp::CLEAR, 6) == 0);
    CHECK(d.has_move() && !d.move_axis[6] && d.move_axis[0]);  // J6 不进 0x08
    CHECK(!released_arm(d));
    Cmd s2b = mk(Mode::STREAM, 3, 100 * MS);
    d = S(c, &s2b, f, 100 * MS);  // 释放后 50 ms：还不清
    CHECK(count_ops(d, BusOp::CLEAR, 6) == 0);
    Cmd s2c = mk(Mode::STREAM, 4, 150 * MS);
    d = S(c, &s2c, f, 150 * MS);  // 释放后 100 ms：0x11
    CHECK(count_ops(d, BusOp::CLEAR, 6) == 1 && count_ops(d, BusOp::STOP, 6) == 0);
    // 清圈生效
    f.ax[6].q = 28.6f;
    f.ax[6].turns = 0;
    Cmd s3 = mk(Mode::STREAM, 5, 200 * MS);
    d = S(c, &s3, f, 200 * MS);
    CHECK(!c.j6_mturn_pending() && d.move_axis[6]);
  }
  // J6 清圈两次都不生效 → MTURN 故障
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    f.ax[6].q = 388.6f;
    f.ax[6].turns = 1;
    Decision d;
    int rel = 0, clr = 0;
    for (int k = 0; k < 40 && !(c.latched() & bits::MTURN); ++k) {
      d = S(c, nullptr, f, k * 50 * MS);
      rel += count_ops(d, BusOp::STOP, 6, 0x10);
      clr += count_ops(d, BusOp::CLEAR, 6);
    }
    CHECK(c.latched() & bits::MTURN);
    CHECK(rel == 2 && clr == 2);
    CHECK(!released_arm(d));
  }

  // --- 状态位：J6 bit6 只告警继续流式；J0–J5 bit6 故障 ---
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    f.ax[6].status = 0x41;
    Cmd s2 = mk(Mode::STREAM, 2, 50 * MS);
    Decision d = S(c, &s2, f, 50 * MS);
    CHECK((d.warn & bits::J6_PROT) && !d.fault && c.state() == CtrlState::STREAMING && d.has_move());
    f.ax[6].status = 0;
    f.ax[1].status = 0x40;
    Cmd s3 = mk(Mode::STREAM, 3, 100 * MS);
    d = S(c, &s3, f, 100 * MS);
    CHECK((d.fault & bits::SERVO_PROT) && c.state() == CtrlState::FAULT && !d.has_move());
    CHECK(!released_arm(d));
  }

  // --- 温度：45 ℃ J6 目标不再更闭合（允许张开）；55 ℃ J6 释放、其余保持 ---
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    q[6] = 32.0f;  // 夹住停在 32°
    Feedback f = fb_at(q);
    auto cmdq = q;
    cmdq[6] = 29.0f;  // 命令继续往闭合方向压
    Cmd s = mk(Mode::STREAM, 1, 0, cmdq);
    Decision d = S(c, &s, f, 0);
    CHECK_NEAR(d.move_q[6], 29.0, 1e-4);
    f.ax[6].temp_c = 46.0f;
    Cmd s2 = mk(Mode::STREAM, 2, 50 * MS, cmdq);
    d = S(c, &s2, f, 50 * MS);
    CHECK((d.warn & bits::J6_HOT) && !d.fault);
    CHECK_NEAR(d.move_q[6], 32.0, 1e-4);  // 目标移到实测角
    auto open = q;
    open[6] = 34.0f;  // 张开方向允许
    Cmd s3 = mk(Mode::STREAM, 3, 100 * MS, open);
    d = S(c, &s3, f, 100 * MS);
    CHECK_NEAR(d.move_q[6], 34.0, 1e-4);
    f.ax[6].temp_c = 55.5f;
    Cmd s4 = mk(Mode::STREAM, 4, 150 * MS, open);
    d = S(c, &s4, f, 150 * MS);
    CHECK((d.fault & bits::J6_THERMAL) && c.state() == CtrlState::FAULT);
    CHECK(count_ops(d, BusOp::STOP, 6, 0x10) == 1);
    CHECK(count_ops(d, BusOp::STOP, 6, 0x11) == 0);  // J6 不锁回
    CHECK(count_ops(d, BusOp::STOP, -1, 0x11) == 6);
    CHECK(!released_arm(d));
  }

  // --- COMM_LOSS：连续 kCommLossCycles 周期无回包 ---
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    f.valid[4] = false;
    Decision d;
    for (int k = 1; k < cfg::kCommLossCycles; ++k) {
      Cmd sk = mk(Mode::STREAM, 1 + k, k * 50 * MS);
      d = S(c, &sk, f, k * 50 * MS);
    }
    CHECK(!(d.fault & bits::COMM_LOSS));
    Cmd sk = mk(Mode::STREAM, 99, 1000 * MS);
    d = S(c, &sk, f, 1000 * MS);
    CHECK((d.fault & bits::COMM_LOSS) && !d.has_move() && !released_arm(d));
  }

  // --- 卡住：跟踪误差 >10° 且电流高持续 >1 s → SERVO_PROT ---
  {
    Core c(P);
    c.on_startup_done(false);
    auto q = Q0;
    Feedback f = fb_at(q);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    auto tgt = Q0;
    tgt[0] = 30.0f;
    Decision d;
    for (int k = 1; k <= 30; ++k) {  // 1.5 s；J0 实测不动
      f.ax[0].current_ma = 800;
      Cmd sk = mk(Mode::STREAM, 1 + k, k * 50 * MS, tgt);
      d = S(c, &sk, f, k * 50 * MS);
    }
    CHECK(d.fault & bits::SERVO_PROT);
    CHECK(!released_arm(d));
  }

  // --- ESTOP：冻结帧 + 7 帧保持；锁存；只认 RESET ---
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    Cmd e = mk(Mode::ESTOP, 2, 10 * MS);
    Decision d = S(c, &e, f, 10 * MS);
    CHECK(c.state() == CtrlState::ESTOP && (d.fault & bits::ESTOP));
    CHECK(!d.ops.empty() && d.ops[0].kind == BusOp::MOVE);  // 冻结帧在最前
    for (int i = 0; i < N; ++i) CHECK(d.move_axis[i] && d.move_q[i] == Q0[i]);
    CHECK(d.move_time_ms == P);
    CHECK(count_ops(d, BusOp::STOP, -1, 0x11) == 7);
    CHECK(!released_arm(d));
    Cmd s2 = mk(Mode::STREAM, 3, 60 * MS);
    d = S(c, &s2, f, 60 * MS);
    CHECK(!d.has_move() && (d.warn & bits::CMD_REJECTED) && c.state() == CtrlState::ESTOP);
    Cmd h = mk(Mode::HOLD, 4, 70 * MS);
    d = S(c, &h, f, 70 * MS);
    CHECK(c.state() == CtrlState::ESTOP);
    Cmd r = mk(Mode::RESET, 5, 80 * MS);
    d = S(c, &r, f, 80 * MS);
    CHECK(c.state() == CtrlState::HOLD && !d.fault);
    CHECK(d.ops.empty());  // 全轴已是保持态，不重发
    Cmd s3 = mk(Mode::STREAM, 6, 90 * MS);
    d = S(c, &s3, f, 90 * MS);
    CHECK(c.state() == CtrlState::STREAMING && d.has_move());
  }
  // ESTOP 冻结帧不含多圈超限轴（0x08 按多圈坐标执行，F58）
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    f.ax[6].q = 388.6f;
    f.ax[6].turns = 1;
    Cmd e = mk(Mode::ESTOP, 1, 0);
    Decision d = S(c, &e, f, 0);
    CHECK(!d.move_axis[6] && d.move_axis[0]);
    CHECK(count_ops(d, BusOp::STOP, 6, 0x11) == 1);
  }

  // --- RELEASE / DAMP：只作用于 mask；其余轴停流式保持 ---
  {
    Core c(P);
    c.on_startup_done(false);
    Feedback f = fb_at(Q0);
    Cmd s = mk(Mode::STREAM, 1, 0);
    S(c, &s, f, 0);
    Cmd r = mk(Mode::RELEASE, 2, 50 * MS, Q0, 1u << 6);
    Decision d = S(c, &r, f, 50 * MS);
    CHECK(c.state() == CtrlState::RELEASED);
    CHECK(count_ops(d, BusOp::STOP, 6, 0x10) == 1 && count_ops(d, BusOp::STOP, -1, 0x11) == 6);
    Cmd r2 = mk(Mode::RELEASE, 3, 60 * MS, Q0, 1u << 6);  // 重复命令不重复发帧
    d = S(c, &r2, f, 60 * MS);
    CHECK(d.ops.empty());
    Cmd dm = mk(Mode::DAMP, 4, 70 * MS, Q0, 1u << 5);
    d = S(c, &dm, f, 70 * MS);
    CHECK(count_ops(d, BusOp::STOP, 5, 0x12) == 1 && d.ops.size() == 1);
  }

  // --- 启动 MTURN（J0–J5 清后仍超限）：FAULT，RESET 前不发任何帧 ---
  {
    Core c(P);
    c.on_startup_done(true);
    CHECK(c.state() == CtrlState::FAULT && (c.latched() & bits::MTURN));
    Feedback f = fb_at(Q0);
    f.ax[1].q = -450.0f;
    f.ax[1].turns = -2;
    Decision d = S(c, nullptr, f, 0);
    CHECK(d.ops.empty());
    f.ax[2].status = 0x20;  // 再来一个新故障也不动
    d = S(c, nullptr, f, 50 * MS);
    CHECK(d.ops.empty() && (d.fault & bits::SERVO_PROT));
  }

  return chk::report("test_safety");
}
