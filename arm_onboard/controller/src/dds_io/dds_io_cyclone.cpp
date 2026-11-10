// dds_io_cyclone.cpp —— DdsIo 的 Cyclone DDS C++ 0.10.2 实现（任务书 §1.1）
//
// 收：rt/d1t/arm_cmd（ArmCmd_），listener 回调里拷贝成 Cmd、打收到时间戳，放进信箱（ESTOP 由信箱唤醒主循环）。
// 发：rt/d1t/arm_state（ArmState_），本模块线程从信箱取 StateMsg 写出。
// QoS：BEST_EFFORT + KEEP_LAST 1（任务书 §1.1【暂定】）。
// 本文件从不碰串口。
#include "dds_io/dds_io.hpp"

#include <atomic>
#include <thread>

#include "common/clock.hpp"
#include "d1t_arm.hpp"
#include "dds/dds.hpp"

namespace d1t {

namespace msg = d1t_interfaces::msg::dds_;

namespace {

class CmdListener : public dds::sub::NoOpDataReaderListener<msg::ArmCmd_> {
 public:
  explicit CmdListener(Mailbox& mb) : mb_(mb) {}
  void on_data_available(dds::sub::DataReader<msg::ArmCmd_>& r) override {
    auto samples = r.take();
    for (const auto& s : samples) {
      if (!s.info().valid()) continue;
      const msg::ArmCmd_& m = s.data();
      Cmd c;
      c.rx_real_ns = now_real_ns();
      c.rx_mono_ns = now_mono_ns();
      c.seq = m.seq();
      c.stamp_ns = m.stamp_ns();
      c.mode = m.mode();
      c.axis_mask = m.axis_mask();
      for (int i = 0; i < N; ++i) c.q[i] = m.q()[i];
      c.dt_ms = m.dt_ms();
      c.grip_power_mw = m.grip_power_mw();
      mb_.put_cmd(c);
    }
  }

 private:
  Mailbox& mb_;
};

msg::ArmState_ to_dds(const StateMsg& s) {
  msg::ArmState_ m;
  m.seq(s.seq);
  m.stamp_ns(s.stamp_ns);
  m.sample_span_us(s.sample_span_us);
  m.ctrl_state(s.ctrl_state);
  m.cmd_seq_ack(s.cmd_seq_ack);
  m.cmd_stamp_ns(s.cmd_stamp_ns);
  m.cmd_rx_ns(s.cmd_rx_ns);
  m.valid_mask(s.valid_mask);
  m.q(s.q);
  m.q_target(s.q_target);
  m.current_ma(s.current_ma);
  m.voltage_mv(s.voltage_mv);
  m.temp_c(s.temp_c);
  m.status(s.status);
  m.turns(s.turns);
  m.fault(s.fault);
  m.warn(s.warn);
  m.overruns(s.overruns);
  return m;
}

}  // namespace

struct DdsIo::Impl {
  Mailbox& mb;
  dds::domain::DomainParticipant dp{dds::core::null};
  dds::topic::Topic<msg::ArmCmd_> t_cmd{dds::core::null};
  dds::topic::Topic<msg::ArmState_> t_state{dds::core::null};
  dds::sub::Subscriber sub{dds::core::null};
  dds::pub::Publisher pub{dds::core::null};
  CmdListener listener;
  dds::sub::DataReader<msg::ArmCmd_> reader{dds::core::null};
  dds::pub::DataWriter<msg::ArmState_> writer{dds::core::null};
  std::atomic<bool> run{false};
  std::thread th;
  explicit Impl(Mailbox& m) : mb(m), listener(m) {}
};

DdsIo::DdsIo(Mailbox& mb) : impl_(new Impl(mb)) {}
DdsIo::~DdsIo() { stop(); }

bool DdsIo::start(std::string& err) {
  try {
    Impl& I = *impl_;
    I.dp = dds::domain::DomainParticipant(cfg::kDdsDomain);
    I.t_cmd = dds::topic::Topic<msg::ArmCmd_>(I.dp, cfg::kTopicCmd);
    I.t_state = dds::topic::Topic<msg::ArmState_>(I.dp, cfg::kTopicState);
    I.sub = dds::sub::Subscriber(I.dp);
    I.pub = dds::pub::Publisher(I.dp);

    dds::sub::qos::DataReaderQos rq = I.sub.default_datareader_qos();
    rq << dds::core::policy::Reliability::BestEffort() << dds::core::policy::History::KeepLast(1);
    I.reader = dds::sub::DataReader<msg::ArmCmd_>(I.sub, I.t_cmd, rq, &I.listener,
                                                  dds::core::status::StatusMask::data_available());

    dds::pub::qos::DataWriterQos wq = I.pub.default_datawriter_qos();
    wq << dds::core::policy::Reliability::BestEffort() << dds::core::policy::History::KeepLast(1);
    I.writer = dds::pub::DataWriter<msg::ArmState_>(I.pub, I.t_state, wq);

    I.run = true;
    I.th = std::thread([&I] {
      StateMsg s;
      while (I.run) {
        if (!I.mb.pop_state(s, 100000000LL)) continue;
        try {
          I.writer.write(to_dds(s));
        } catch (const std::exception&) {
          // 写失败不影响主循环；下一帧继续
        }
      }
    });
  } catch (const std::exception& e) {
    err = e.what();
    return false;
  }
  return true;
}

void DdsIo::stop() {
  if (!impl_) return;
  Impl& I = *impl_;
  if (I.run.exchange(false) && I.th.joinable()) I.th.join();
  // 先摘掉 listener，避免析构期间回调进信箱；实体由析构统一关闭（显式 close 会在析构时二次关闭抛异常）
  if (I.reader != dds::core::null) I.reader.listener(nullptr, dds::core::status::StatusMask::none());
}

}  // namespace d1t
