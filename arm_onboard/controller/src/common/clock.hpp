// clock.hpp —— 时钟与可唤醒等待
//
// 任务书 §1.3：调度用 CLOCK_MONOTONIC，对外时间戳一律 CLOCK_REALTIME。
// MonoEvent：按 MONOTONIC 绝对时刻等待、可被另一线程提前唤醒（ESTOP 唤醒主循环，§3.4）。
//   Linux：pthread_cond + pthread_condattr_setclock(CLOCK_MONOTONIC)，不受 chrony 拨钟影响；
//   macOS（仅开发/仿真）：std::condition_variable + steady_clock。
#pragma once

#include <chrono>
#include <cstdint>
#include <ctime>
#include <thread>

#if defined(__linux__)
#include <pthread.h>
#else
#include <condition_variable>
#include <mutex>
#endif

namespace d1t {

inline int64_t now_mono_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

inline int64_t now_real_ns() {
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

inline void sleep_until_mono(int64_t t_ns) {
  for (;;) {
    int64_t d = t_ns - now_mono_ns();
    if (d <= 0) return;
    std::this_thread::sleep_for(std::chrono::nanoseconds(d));
  }
}

inline void sleep_ms(int ms) { sleep_until_mono(now_mono_ns() + int64_t(ms) * 1000000LL); }

// 互斥锁 + 条件变量；lock()/unlock() 供信箱保护自身数据，wait_until 必须在持锁时调用
class MonoEvent {
 public:
#if defined(__linux__)
  MonoEvent() {
    pthread_mutex_init(&mu_, nullptr);
    pthread_condattr_t a;
    pthread_condattr_init(&a);
    pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
    pthread_cond_init(&cv_, &a);
    pthread_condattr_destroy(&a);
  }
  ~MonoEvent() {
    pthread_cond_destroy(&cv_);
    pthread_mutex_destroy(&mu_);
  }
  void lock() { pthread_mutex_lock(&mu_); }
  void unlock() { pthread_mutex_unlock(&mu_); }
  void notify_all() { pthread_cond_broadcast(&cv_); }
  // 持锁调用；返回时仍持锁。到时或被唤醒（含虚假唤醒）都会返回，调用方自行判条件。
  void wait_until_locked(int64_t deadline_mono_ns) {
    timespec ts;
    ts.tv_sec = deadline_mono_ns / 1000000000LL;
    ts.tv_nsec = deadline_mono_ns % 1000000000LL;
    pthread_cond_timedwait(&cv_, &mu_, &ts);
  }

 private:
  pthread_mutex_t mu_;
  pthread_cond_t cv_;
#else
  void lock() { mu_.lock(); }
  void unlock() { mu_.unlock(); }
  void notify_all() { cv_.notify_all(); }
  void wait_until_locked(int64_t deadline_mono_ns) {
    // steady_clock 在 macOS 上即 CLOCK_MONOTONIC_RAW 系；仅开发用，按相对时长换算
    int64_t d = deadline_mono_ns - now_mono_ns();
    if (d <= 0) return;
    std::unique_lock<std::mutex> lk(mu_, std::adopt_lock);
    cv_.wait_for(lk, std::chrono::nanoseconds(d));
    lk.release();
  }

 private:
  std::mutex mu_;
  std::condition_variable cv_;
#endif
};

}  // namespace d1t
