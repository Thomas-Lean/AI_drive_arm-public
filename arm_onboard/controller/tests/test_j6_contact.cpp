// test_j6_contact.cpp —— J6 碰到即锁（warn J6_CONTACT）单测：safety::Core + 测试内的假 J6 舵机
// 场景：空夹走到底不触发；电流触发；停滞触发；锁定后更闭合目标被钳住；张开解锁；保持态触发；
//      45 ℃ 降出力共存；过温仍释放；J0–J5 不受影响。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>

#include "check.hpp"
#include "safety/safety.hpp"

using namespace d1t;
using namespace d1t::safety;

static const int P = 50;
static const int64_t MS = 1000000LL;
static const float NaN = std::numeric_limits<float>::quiet_NaN();
static const std::array<float, N> Q0 = {0, -60, 60, 0, 10, 0, 64.5f};

// 假 J6 舵机：实测角 = lag 个周期前的总线目标（每周期最多走 5°），被物体挡在 obstacle 不再变小；
// 贴住时：目标比 obstacle 更闭合 > 3.0° → push_ma；0.5–3.0°（夹紧量）→ min(151, push_ma)（F69 差 2.2° 约 151 mA）；否则 14 mA（F60）
struct J6Sim {
  float q = 64.5f;
  float tgt = 64.5f;
  float obstacle = NaN;
  uint16_t push_ma = 300;
  int lag = 1;
  bool released = false;
  float temp_c = 30.0f;
  std::deque<float> hist;
  uint16_t current() const {
    bool touch = std::isfinite(obstacle) && !released && q <= obstacle + 0.05f;
    float gap = touch ? obstacle - tgt : 0.0f;
    return gap > 3.0f ? push_ma : gap > 0.5f ? std::min<uint16_t>(151, push_ma) : uint16_t(14);
  }
  // 按帧顺序执行：0x08 → 目标；0x18 0x11 → 锁在当时位置（目标 = 实测，夹紧量消失，F69）；0x10/0x12 → 失力
  void apply(const Decision& d) {
    for (auto& o : d.ops) {
      if (o.kind == BusOp::MOVE && d.move_axis[6]) {
        tgt = d.move_q[6];
        released = false;
      } else if (o.kind == BusOp::STOP && o.axis == 6) {
        released = o.mode == 0x10 || o.mode == 0x12;
        if (o.mode == 0x11) tgt = q;
      }
    }
    hist.push_back(tgt);
    while (int(hist.size()) > lag) hist.pop_front();
    if (released) return;
    float goal = hist.front();
    float step = std::fmax(-5.0f, std::fmin(5.0f, goal - q));
    q += step;
    if (std::isfinite(obstacle) && q < obstacle) q = obstacle;
  }
};

struct Rig {
  Core c{P};
  J6Sim j6;
  std::array<float, N> arm = Q0;
  int64_t t = 0;
  uint32_t seq = 0;
  Decision last;
  Rig() { c.on_startup_done(false); }
  Feedback fb() const {
    Feedback f;
    for (int i = 0; i < N; ++i) {
      f.valid[i] = f.have[i] = true;
      f.ax[i].q = arm[i];
      f.ax[i].temp_c = 30.0f;
      f.ax[i].current_ma = 14;
      f.ax[i].t_ns = t;
    }
    f.ax[6].q = j6.q;
    f.ax[6].current_ma = j6.current();
    f.ax[6].temp_c = j6.temp_c;
    return f;
  }
  Decision cycle(Mode m, float j6_cmd) {
    Cmd cmd;
    cmd.seq = ++seq;
    cmd.stamp_ns = cmd.rx_mono_ns = cmd.rx_real_ns = t;
    cmd.rx_no = seq;
    cmd.mode = uint8_t(m);
    cmd.q = arm;
    cmd.q[6] = j6_cmd;
    cmd.dt_ms = 50;
    Decision d = c.step(&cmd, fb(), t);
    c.commit(d, std::vector<OpResult>(d.ops.size(), OpResult::OK), t);
    j6.apply(d);
    t += P * MS;
    last = d;
    return d;
  }
  Decision idle() {  // 无新命令（上位机只是没发）
    Decision d = c.step(nullptr, fb(), t);
    c.commit(d, std::vector<OpResult>(d.ops.size(), OpResult::OK), t);
    j6.apply(d);
    t += P * MS;
    last = d;
    return d;
  }
  void hold_first() { cycle(Mode::HOLD, j6.q); }
  // SDK 夹爪：从 from 到 to 每周期 1°（20°/s），走完后继续发终点 extra 个周期；返回首次 J6_CONTACT 的周期号（-1 = 无）
  int close(float from, float to, int extra, bool* ever = nullptr) {
    int first = -1, k = 0;
    float q = from;
    for (;;) {
      q = std::fmax(to, q - 1.0f);
      Decision d = cycle(Mode::STREAM, q);
      if ((d.warn & bits::J6_CONTACT) && first < 0) first = k;
      ++k;
      if (q <= to && --extra < 0) break;
    }
    if (ever) *ever = first >= 0;
    return first;
  }
};

int main() {
  // ---- 1. 空夹 64.5 → 29.0 走到底：不触发（滞后 1、2 周期两种舵机）----
  for (int lag : {1, 2}) {
    Rig r;
    r.j6.lag = lag;
    r.hold_first();
    bool ever = true;
    r.close(64.5f, 29.0f, 20, &ever);
    CHECK(!ever);
    CHECK(!r.c.j6_contact_on());
    CHECK_NEAR(r.j6.q, 29.0, 0.01);
    CHECK(r.c.bus_target(6) == 29.0f);
  }

  // ---- 2. 电流触发：瓶在 32°，命令合到 22°（比停住角多约 1 cm），被顶住 300 mA ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.hold_first();
    int first = r.close(64.5f, 22.0f, 20);
    CHECK(first >= 0);
    // 碰到（q 到 32 的那周期）后几个周期内触发：64.5→32 约 33 周期，触发不晚于碰到后 4 周期
    CHECK(first <= 33 + 4);
    CHECK(r.c.j6_contact_on());
    CHECK_NEAR(r.c.j6_contact_tgt(), 32.0 - 2.5, 0.01);  // 锁定目标 = 实测 − 夹紧量 2.5°
    CHECK(r.c.bus_target(6) == 29.5f);  // 锁定后总线目标 = 锁定目标，不再更闭合
    CHECK(r.j6.current() == 151);       // 只剩夹紧量的电流（F69 约 151 mA），不再硬顶
    CHECK(r.last.warn & bits::J6_CONTACT);
    CHECK(!(r.last.fault));             // 非故障
    CHECK(r.c.state() == CtrlState::STREAMING);
    // J0–J5 照命令执行
    for (int i = 0; i < 6; ++i) CHECK(r.last.move_axis[i] && r.last.move_q[i] == Q0[i]);
  }

  // ---- 3. 停滞触发：透明瓶电流低（只到 14 mA），角度停在 32° 而命令继续往里 ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.j6.push_ma = 14;
    r.hold_first();
    int first = r.close(64.5f, 22.0f, 20);
    CHECK(first >= 0 && first <= 33 + 6);
    CHECK(r.c.j6_contact_on());
    CHECK_NEAR(r.c.j6_contact_tgt(), 29.5, 0.01);
    CHECK(r.c.bus_target(6) == 29.5f);
  }

  // ---- 4. 锁定后：更闭合目标钳到锁定目标（实测 − 2.5°）；回弹 0.7° 不跟随；按回弹后实测角下令不解锁；
  //         张开 ≤ 锁定目标 + 3.5°（= 触发实测 + 1°）仍钳在锁定目标；超过才解锁 ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.hold_first();
    r.close(64.5f, 22.0f, 10);
    CHECK(r.c.j6_contact_on());
    const float L = r.c.j6_contact_tgt();
    CHECK_NEAR(L, 29.5, 0.01);
    Decision d = r.cycle(Mode::STREAM, 10.0f);
    CHECK(d.move_axis[6] && d.move_q[6] == L && (d.warn & bits::J6_CONTACT));
    // 回弹：手指被顶开 0.7°，锁定目标不变
    r.j6.obstacle = 32.7f;
    r.j6.q = 32.7f;
    for (int k = 0; k < 5; ++k) d = r.cycle(Mode::STREAM, 22.0f);
    CHECK_NEAR(r.c.j6_contact_tgt(), 29.5, 0.01);
    CHECK(d.move_q[6] == L);
    // 上位机按回弹后的实测角 32.7° 下令（app「锁在实测停住角」）：不解锁、仍钳在锁定目标（保住夹紧量）
    d = r.cycle(Mode::STREAM, 32.7f);
    CHECK(r.c.j6_contact_on() && d.move_q[6] == L && (d.warn & bits::J6_CONTACT));
    // 张开到锁定目标 + 3.5°（边界）：仍锁
    d = r.cycle(Mode::STREAM, L + 3.5f);
    CHECK(r.c.j6_contact_on() && d.move_q[6] == L);
    // 张开超过锁定目标 + 3.5°：解锁，照常执行
    d = r.cycle(Mode::STREAM, L + 3.6f);
    CHECK(!r.c.j6_contact_on() && d.move_q[6] == L + 3.6f && !(d.warn & bits::J6_CONTACT));
    // 解锁后可以再次触发
    r.j6.obstacle = 32.0f;
    int first = r.close(L + 3.6f, 22.0f, 10);
    CHECK(first >= 0 && r.c.j6_contact_on());
  }

  // ---- 4b. 锁定后上位机发 HOLD：J6 的 0x18 0x11 之后紧跟一帧 0x08 到锁定目标，夹紧量不丢 ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.hold_first();
    r.close(64.5f, 22.0f, 10);
    CHECK(r.c.j6_contact_on() && r.j6.current() == 151);
    Decision d = r.cycle(Mode::HOLD, 0);
    int k_stop = -1, k_move = -1;
    for (size_t k = 0; k < d.ops.size(); ++k) {
      if (d.ops[k].kind == BusOp::STOP && d.ops[k].axis == 6 && d.ops[k].mode == 0x11) k_stop = int(k);
      if (d.ops[k].kind == BusOp::MOVE) k_move = int(k);
    }
    CHECK(k_stop >= 0 && k_move > k_stop && d.move_axis[6] && d.move_q[6] == 29.5f);
    for (int i = 0; i < 6; ++i) CHECK(!d.move_axis[i]);  // 补发的 0x08 只含 J6
    CHECK(r.j6.tgt == 29.5f && r.j6.current() == 151);   // 舵机仍夹紧 2.5°
    // 之后几个周期不再重复发
    int moves = 0;
    for (int k = 0; k < 10; ++k) moves += r.idle().move_axis[6];
    CHECK(moves == 0 && r.j6.current() == 151);
  }

  // ---- 5. 保持态：流式停在比瓶更闭合的目标后转 HOLD，控制器仍检出并补发一帧 0x08 到锁定目标 ----
  {
    Rig r;
    r.j6.obstacle = 40.0f;
    r.j6.push_ma = 14;  // 只靠停滞
    r.hold_first();
    // 快速一步把目标推到 36（比瓶闭合 4°），立刻 HOLD
    r.cycle(Mode::STREAM, 59.5f);
    r.cycle(Mode::STREAM, 54.5f);
    r.cycle(Mode::STREAM, 49.5f);
    r.cycle(Mode::STREAM, 44.5f);
    r.cycle(Mode::STREAM, 39.5f);
    r.cycle(Mode::STREAM, 36.0f);
    CHECK(!r.c.j6_contact_on());
    r.cycle(Mode::HOLD, 0);
    bool sent = false;
    for (int k = 0; k < 8 && !r.c.j6_contact_on(); ++k) r.idle();
    CHECK(r.c.j6_contact_on());
    Decision d = r.idle();
    sent = r.c.bus_target(6) == 37.5f;  // 锁定目标 = 40 − 2.5
    CHECK(sent && (d.warn & bits::J6_CONTACT));
    // J0–J5 在保持态没有被 0x08 带动（0x08 只含 J6）
    for (int i = 0; i < 6; ++i) CHECK(r.c.bus_target(i) == Q0[i]);
    // HOLD 不解锁
    d = r.cycle(Mode::HOLD, 0);
    CHECK(r.c.j6_contact_on() && (d.warn & bits::J6_CONTACT));
  }

  // ---- 6. 45 ℃ 降出力共存：锁定中升温，两个位都报，J6 目标不比任一锁定角更闭合 ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.hold_first();
    r.close(64.5f, 22.0f, 10);
    CHECK(r.c.j6_contact_on());
    r.j6.temp_c = 46.0f;
    Decision d = r.cycle(Mode::STREAM, 22.0f);
    CHECK((d.warn & bits::J6_HOT) && (d.warn & bits::J6_CONTACT) && !d.fault);
    CHECK(d.move_q[6] >= 29.5f && d.move_q[6] >= r.c.j6_derate_tgt());
  }

  // ---- 7. 过温仍按原逻辑：锁定中 ≥50 ℃ → J6_THERMAL、J6 释放、其余保持；锁解除 ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.hold_first();
    r.close(64.5f, 22.0f, 10);
    CHECK(r.c.j6_contact_on());
    r.j6.temp_c = 55.0f;
    Decision d = r.cycle(Mode::STREAM, 22.0f);
    CHECK((d.fault & bits::J6_THERMAL) && r.c.state() == CtrlState::FAULT);
    int rel6 = 0, rel_arm = 0;
    for (auto& o : d.ops) {
      if (o.kind == BusOp::STOP && o.mode == 0x10) (o.axis == 6 ? rel6 : rel_arm)++;
      if (o.kind == BusOp::MOVE) CHECK(!d.move_axis[6]);
    }
    CHECK(rel6 == 1 && rel_arm == 0);
    CHECK(!r.c.j6_contact_on() && !(d.warn & bits::J6_CONTACT));
    d = r.idle();
    CHECK(!d.move_axis[6] && !(d.warn & bits::J6_CONTACT));
  }

  // ---- 8. 夹得很轻（命令只比停住角闭合 0.8°）：不触发 ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.hold_first();
    bool ever = true;
    r.close(64.5f, 31.2f, 20, &ever);
    CHECK(!ever);
  }

  // ---- 9. 释放 J6 后解锁 ----
  {
    Rig r;
    r.j6.obstacle = 32.0f;
    r.hold_first();
    r.close(64.5f, 22.0f, 10);
    CHECK(r.c.j6_contact_on());
    Cmd cmd;
    cmd.seq = 999;
    cmd.rx_no = 999;
    cmd.mode = uint8_t(Mode::RELEASE);
    cmd.axis_mask = 0x40;
    Decision d = r.c.step(&cmd, r.fb(), r.t);
    r.c.commit(d, std::vector<OpResult>(d.ops.size(), OpResult::OK), r.t);
    r.t += P * MS;
    d = r.idle();
    CHECK(!r.c.j6_contact_on() && !(d.warn & bits::J6_CONTACT));
  }

  return chk::report("test_j6_contact");
}
