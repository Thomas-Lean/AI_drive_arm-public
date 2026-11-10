// safety.hpp —— 安全逻辑：纯函数 + 状态机（不做任何 I/O，任务书 §2 边界规则 2）
//
// 每周期两步：
//   step(cmd, fb, now)   → Decision = 计划下发的总线动作（有序）+ 本周期告警 / 事件
//   commit(d, results)   → 按每个动作的写出结果（OK / FAIL / SKIPPED）提交状态：
//                          只有写成功的帧才更新 q_target、ack、停住确认；写失败的停止帧下周期重发（§3.7-5）
// 主循环（loop）负责把 Decision 变成串口帧；safety 本身可单独单测。
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "common/types.hpp"

namespace d1t {
namespace safety {

// ---------------- 纯函数 ----------------

struct Clamp {
  float q;
  bool clamped;
};
// F7 限位 + 协议 ±180 双重钳制（fsbus.py clamp_angle 同）
Clamp clamp_limit(int axis, float q);
// 单周期最大步长 = 轴速 × 周期（任务书 §3.2，F51）
Clamp clamp_step(int axis, float q_cmd, float q_prev, int period_ms);
float max_step_deg(int axis, int period_ms);
// |q| ≥ 180（F58；任务书 §1.4 MTURN）。不看 turns：负角度时 turns = −1 是正常读数（F63）
bool mturn_out(float q);
bool out_of_limit(int axis, float q);

struct StatusClass {
  bool fault;    // → SERVO_PROT
  bool j6_warn;  // → J6_PROT（只告警）
};
StatusClass classify_status(int axis, uint8_t status);

enum class J6Temp { OK, HOT, RELEASE, UNKNOWN };
J6Temp j6_temp_level(float temp_c);  // NaN → UNKNOWN（§3.7-3：温度无效不得当成正常温度）

// 命令格式合法性：mode 0–5；RELEASE/DAMP 的 axis_mask ≠ 0 且只含 J0–J6；STREAM 的 q 全为有限数
bool cmd_valid(const Cmd& c);
// 0x08 time = clamp(dt_ms, 周期, 3×周期)（任务书 §3.2）
uint16_t clamp_dt(uint16_t dt_ms, int period_ms);
// J6 power = clamp(grip_power_mw, 0, 6000)
uint16_t clamp_grip_power(uint16_t mw);

// ---------------- 决策输出 ----------------

struct BusOp {
  enum Kind : uint8_t { MOVE, STOP, CLEAR } kind;
  uint8_t axis;  // STOP / CLEAR 用
  uint8_t mode;  // STOP 用：0x10 / 0x11 / 0x12
  float q_meas;  // STOP 0x11 用：发帧时刻的实测角 → 写成功后成为该轴 q_target（§3.7-7）
};

enum class OpResult : uint8_t { OK, FAIL, SKIPPED };  // SKIPPED：ESTOP 待处理时主循环放弃的运动 / 清圈帧

struct Decision {
  // 0x19+0x08 的内容（ops 中出现 MOVE 时发送）
  std::array<bool, N> move_axis{};
  std::array<float, N> move_q{};
  uint16_t move_time_ms = 0;
  std::array<uint16_t, N> move_power{};
  std::vector<BusOp> ops;  // 按顺序执行

  bool estop = false;        // 本周期执行了 ESTOP（冻结帧在最前）
  bool cmd_applied = false;  // 本周期命令被采纳；commit 时全部帧写成功才推进 ack
  uint32_t cand_seq = 0;
  int64_t cand_stamp_ns = 0, cand_rx_ns = 0;

  // 以下由 commit 填写
  CtrlState state = CtrlState::INIT;
  uint32_t fault = 0;
  uint32_t warn = 0;                // step 填本周期告警
  std::array<float, N> q_target{};  // 每轴最近一次写成功的钳制后目标；释放 / 阻尼 / 从未下令 = NaN（§3.7-7）
  uint32_t ack_seq = 0;
  int64_t ack_stamp_ns = 0;
  int64_t ack_rx_ns = 0;
  std::vector<std::string> events;  // 给日志
  bool has_move() const {
    for (auto& o : ops)
      if (o.kind == BusOp::MOVE) return true;
    return false;
  }
};

// ---------------- 状态机 ----------------

class Core {
 public:
  explicit Core(int period_ms) : period_ms_(period_ms) { bus_tgt_.fill(std::numeric_limits<float>::quiet_NaN()); }

  // 启动流程结束后调用一次（loop/startup）。mturn_fault=true：J0–J5 清圈后仍超限 →
  // MTURN 故障，且在 RESET 前不发任何总线动作（任务书 §3.3「不动任何轴」）。
  void on_startup_done(bool mturn_fault);

  Decision step(const Cmd* cmd, const Feedback& fb, int64_t now_mono_ns);
  // results[k] 对应 d.ops[k]；缺的按 SKIPPED。now = 写完之后的时刻（停住确认窗从这里起算）
  void commit(Decision& d, const std::vector<OpResult>& results, int64_t now_mono_ns);

  // 程序正常退出前：全轴 0x18 0x11 保持（绝不释放；J6 过温已释放的不锁回；启动 MTURN 未 RESET 时不动）
  Decision on_shutdown();

  CtrlState state() const { return st_; }
  uint32_t latched() const { return latched_; }
  bool watchdog_tripped() const { return wd_tripped_; }
  bool j6_derate_on() const { return j6_derate_on_; }
  float j6_derate_tgt() const { return j6_derate_tgt_; }
  bool j6_mturn_pending() const { return j6_pending_; }

  enum class Ax : uint8_t { UNKNOWN, MOVING, HOLD, RELEASED, DAMPED };
  Ax axis_mode(int i) const { return ax_[i]; }
  bool tx_pending(int i) const { return tx_pending_[i]; }
  bool hold_confirm_active(int i) const { return conf_[i].active; }
  float bus_target(int i) const { return bus_tgt_[i]; }

 private:
  using Fresh = std::array<bool, N>;
  void want_stop(Decision& d, int i, uint8_t mode, const Feedback& fb);
  void hold_all(Decision& d, bool force, const Feedback& fb);
  void track_still(const Feedback& fb, const Fresh& fresh);
  // 该轴最近 kStillWindow 个连续新鲜采样是否齐全（且最早一个不早于 t_from）；齐全时给出 max(q)−min(q)
  bool still_window(int i, int64_t t_from, float& range) const;
  void eval_feedback(Decision& d, const Feedback& fb, const Fresh& fresh, int64_t now, bool estop_now,
                     uint32_t& newf, bool& j6_hot);
  void eval_hold_confirm(Decision& d, const Feedback& fb, const Fresh& fresh, int64_t now, uint32_t& newf);
  void handle_cmd(Decision& d, const Cmd& c, const Feedback& fb, const Fresh& fresh, bool j6_hot, bool& applied);
  void do_reset(Decision& d, const Cmd& c, const Feedback& fb, const Fresh& fresh, bool& applied);
  void do_stream(Decision& d, const Cmd& c, const Feedback& fb, bool j6_hot, bool& applied);
  void do_estop(Decision& d, const Cmd& c, const Feedback& fb, const Fresh& fresh, int64_t now);
  void j6_derate(Decision& d, const Feedback& fb, const Fresh& fresh, bool j6_hot);
  bool j6_forcing() const;  // J6 是否处于持续出力状态（流式 / 保持，且未被排除）
  bool j6_excluded() const;
  uint8_t stop_mode_of(int i) const;

  int period_ms_;
  CtrlState st_ = CtrlState::INIT;
  uint32_t latched_ = 0;
  std::array<Ax, N> ax_{};          // 控制器要求各轴处于的状态（意图）
  std::array<bool, N> tx_pending_{};  // 该轴最近一次停止帧没写出去 → 每周期重发直到成功（§3.7-5）
  std::array<float, N> bus_tgt_;    // 每轴最近一次写成功的目标（q_target；NaN = 无）
  bool stream_base_ok_ = false;     // bus_tgt_ 可作流式步长基准（本次流式里已有成功写出的 0x08）
  bool need_hold_all_ = false;      // commit 中新锁存 TX_FAIL → 下周期全轴保持
  uint16_t j6_power_ = 0;           // 最近一次流式的 J6 功率（45 ℃ 降出力帧沿用）
  // 45 ℃ 降出力（TC-010 实机改）：≥ kJ6TempWarnC 进入、≤ kJ6TempWarnExitC 退出（回差）；
  // 进入时锁定一次目标 = 当时新鲜实测角，之后不跟随实测（防瓶子回弹顶开手指的棘轮），只被更张开的 STREAM 目标更新
  bool j6_derate_on_ = false;
  float j6_derate_tgt_ = 0.0f;

  bool wd_armed_ = false;  // 收到过第一条合法命令后才启用看门狗
  bool wd_tripped_ = false;
  int64_t last_cmd_mono_ = 0;

  std::array<int, N> miss_{};
  std::array<int64_t, N> track_since_{};

  bool j6_pending_ = false;  // 运行中 J6 多圈超限，处理中（J6 不参与 0x08）
  int j6_tries_ = 0;
  // 运行中 J6 清圈分阶段执行（R05：不在一个周期里睡 100 ms；ESTOP 可随时抢占）
  enum class J6Stage : uint8_t { NONE, RELEASE_SENT, CLEAR_SENT } j6_stage_ = J6Stage::NONE;
  int64_t j6_stage_t_ = 0;
  bool startup_noact_ = false;

  uint32_t ack_seq_ = 0;
  int64_t ack_stamp_ = 0, ack_rx_ = 0;
  uint64_t estop_rx_no_ = 0;  // 最近一次 ESTOP 的信箱接收序号（§3.7-1）

  // 停住窗口（TC-010 实机改）：每轴最近 kStillWindow 个**连续**新鲜采样（相邻跨度 ≤ kStillMaxDtPeriods 周期）；
  // 缺回包 / 跨度过大 → 清空重攒（R08：不新鲜不证明静止）
  std::array<std::array<float, cfg::kStillWindow>, N> win_q_{};
  std::array<std::array<int64_t, cfg::kStillWindow>, N> win_t_{};
  std::array<int, N> win_n_{};     // 已攒的连续采样数（≤ kStillWindow）
  std::array<int, N> win_head_{};  // 下一个写入位置（环形）
  std::array<int64_t, N> win_last_t_{};

  // 停住确认（§3.7-6）：保持帧写成功后启动；重复 HOLD 不重置
  struct Confirm {
    bool active = false;
    int64_t t0 = 0;
    int resends = 0;
  };
  std::array<Confirm, N> conf_{};

  // ESTOP 后汇总报告
  bool estop_watch_ = false;
  bool estop_conf_started_ = false;
  int64_t estop_t_ = 0;
};

}  // namespace safety
}  // namespace d1t
