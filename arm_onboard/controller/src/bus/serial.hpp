// serial.hpp —— 串口抽象：PosixSerial（真机 /dev/ttyS4）与 FakeSerial（--sim）
//
// 线程规则（任务书 §2 边界规则 1）：串口对象只由主循环线程持有和调用。
// FakeSerial 的注入接口（inject_*）带锁，测试线程可以调用；它们不碰「串口」，只改仿真舵机的内部状态。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <random>
#include <string>

#include "bus/frame.hpp"
#include "config.hpp"

namespace d1t {

class ISerial {
 public:
  virtual ~ISerial() = default;
  // 写完整帧并等到发送完成（有总期限）；失败返回 false
  virtual bool write_all(const uint8_t* p, size_t n) = 0;
  // 读可用字节，最多等 timeout_ns；返回读到的字节数（0 = 超时）
  virtual size_t read_some(uint8_t* p, size_t cap, int64_t timeout_ns) = 0;
  // 丢弃输入缓冲中尚未读出的字节，返回丢弃数
  virtual size_t flush_input() = 0;
};

// 排空用到的两个查询 ioctl，抽成接口以便注入故障（V2-01）。返回 0 = 成功，-1 = 不支持 / 出错。
class TtyQuery {
 public:
  virtual ~TtyQuery() = default;
  virtual int out_queue(int fd, int& bytes);         // TIOCOUTQ：驱动软件缓冲里还没发的字节数
  virtual int tx_empty(int fd, bool& empty);         // TIOCSERGETLSR & TIOCSER_TEMT（仅 Linux；别处 -1）
};

// 有界排空（V2-01）：不用阻塞式排空（tc 系列排空调用在 Linux 上不可中断）。全部步骤受同一 MONOTONIC 期限 deadline 约束：
//   ① 轮询 out_queue 到 0；② 轮询 tx_empty 到真；③ 任一查询不支持 / 出错 → 退化为按字节数估算：
//      写完时刻 written_ns + n×10/115200 s + kUartFifoMarginMs，估算终点超过期限即失败。
// 返回 true = 可认为已发完；false = 超期 / 估算超期（调用方 tcflush 后按写失败处理）。
bool drain_bounded(TtyQuery& q, int fd, size_t n, int64_t written_ns, int64_t deadline_ns);

// 真机串口。open() 失败（含被占用）返回 false 并写 err。
class PosixSerial : public ISerial {
 public:
  ~PosixSerial() override;
  bool open(const std::string& port, int baud, std::string& err);
  bool write_all(const uint8_t* p, size_t n) override;
  size_t read_some(uint8_t* p, size_t cap, int64_t timeout_ns) override;
  size_t flush_input() override;
  void set_tty_query(TtyQuery* q) { tq_ = q ? q : &real_tq_; }  // 测试注入用

 private:
  int fd_ = -1;
  TtyQuery real_tq_;
  TtyQuery* tq_ = &real_tq_;
};

// 开串口前的占用检查（Linux）：4 个 marm 服务必须全部 inactive，且 /proc 里没有其他进程打开该端口。
// 判不了（无权限读 /proc/<pid>/fd、systemctl 不可用）按「无法确定」返回 false——与 fsbus.py preflight 同口径。
bool preflight_port(const std::string& port, std::string& msg);

// ---------------- 仿真 ----------------

// 7 个仿真舵机：回包、角度一阶跟随、可注入丢包 / 坏帧 / 状态位 / 温度 / 多圈 / 离线。
// 时序模型：写入按 115200 8N1 的线上时长阻塞；0x16 回包在发送完成后 3.0 ms + k×2.2 ms 依次可读
// （7 轴约 16.2 ms，对齐 F46 中位 16.4 ms）。0x11 只在释放态生效（PROTOCOL_NOTES 0x11 节）。
class FakeSerial : public ISerial {
 public:
  explicit FakeSerial(uint32_t seed = 12345);

  bool write_all(const uint8_t* p, size_t n) override;
  size_t read_some(uint8_t* p, size_t cap, int64_t timeout_ns) override;
  size_t flush_input() override;

  // ---- 注入（线程安全） ----
  void inject_drop_prob(int axis, double p);      // 每次回包按概率丢
  void inject_drop_next(int axis, int count);     // 接下来 count 次回包必丢
  void inject_corrupt_prob(int axis, double p);   // 回包校验和按概率写错
  void inject_status(int axis, uint8_t st);       // 叠加到状态字节
  void inject_temp_c(int axis, float c);          // 温度
  void inject_mturn(int axis, float angle_deg);   // 直接设多圈累计角（圈数由角度推出）
  void inject_offline(int axis, bool off);        // 完全不回包（含 ping）
  void inject_angle_offset_on_hold(int axis, float dps);  // 保持态漂移速度 °/s（ESTOP 未停住测试用）
  void inject_locked(int axis);                           // 置为保持锁力态（此时 0x11 无效）
  void inject_temp_adc(int axis, int adc);                // 温度 ADC 原值（0 / ≥4096 = 非法；<0 = 恢复按温度换算）
  void inject_reply_delay_ms(int axis, double ms);        // 该轴回包额外延迟（迟到帧测试）

  // ---- 观测（线程安全，测试用） ----
  enum class SMode { RELEASED, HOLD, MOVING, DAMP };
  struct ServoView {
    float pos_deg;
    float target_deg;
    SMode mode;
    int clear_count;
    int stop_count;
    int move_count;
  };
  ServoView view(int axis);
  uint64_t frames_written();
  // 最近一次收到的 0x19+0x08 帧发送完成时刻（mono ns）；ESTOP 时延测量用
  int64_t last_sync_move_tx_end_ns();
  int64_t last_stop_tx_end_ns();

 private:
  struct Servo {
    double a0 = 10.0;     // 运动起点（多圈累计角，度）
    double target = 10.0;
    int64_t t0 = 0;       // 运动起点时刻
    double tau_s = 0.05;  // 一阶时间常数
    SMode mode = SMode::RELEASED;  // 上电默认释放（F36：上电锁力关）
    uint8_t status_inj = 0;
    float temp_c = 30.0f;
    double drop_p = 0.0;
    int drop_next = 0;
    double corrupt_p = 0.0;
    bool offline = false;
    float hold_drift = 0.0f;
    int temp_adc = -1;  // ≥0：直接用这个 ADC 原值
    int64_t delay_ns = 0;
    int clear_count = 0, stop_count = 0, move_count = 0;
  };
  struct Pending {
    int64_t ready_ns;
    frame::Bytes bytes;
  };

  double pos_at(const Servo& s, int64_t t) const;
  void handle_request(uint8_t cmd, const frame::Bytes& c, int64_t tx_end);
  void apply_move(uint8_t id, int16_t raw, uint16_t time_ms, int64_t t);
  bool should_reply(Servo& s);
  frame::Bytes make_reply(uint8_t cmd, const frame::Bytes& content, Servo& s);
  frame::Bytes monitor_content(uint8_t id, int64_t t);

  std::mutex mu_;
  std::array<Servo, cfg::kNumAxes> sv_;
  std::deque<Pending> pending_;
  frame::Bytes rx_;  // 已到达、可读的字节
  frame::Bytes req_buf_;
  std::mt19937 rng_;
  uint64_t frames_ = 0;
  int64_t last_sync_move_ = 0;
  int64_t last_stop_ = 0;
};

}  // namespace d1t
