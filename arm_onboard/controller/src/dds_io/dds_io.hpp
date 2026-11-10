// dds_io.hpp —— DDS 收发（Cyclone DDS C++ 0.10.2，任务书 §1.1）
//
// 线程规则：DDS 回调 / 本模块线程只调用 Mailbox（put_cmd、pop_state），从不碰串口。
// 实现：dds_io_cyclone.cpp，只在 CMake 找到 CycloneDDS-CXX 且 idl/generated/ 存在时编入（D1T_WITH_DDS）。
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "loop/mailbox.hpp"

namespace d1t {

class DdsIo {
 public:
  explicit DdsIo(Mailbox& mb);
  ~DdsIo();
  bool start(std::string& err);  // 建 participant / topic / reader / writer，起发布线程
  void stop();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace d1t
