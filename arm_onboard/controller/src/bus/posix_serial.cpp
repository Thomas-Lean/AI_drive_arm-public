// posix_serial.cpp —— 真机串口（/dev/ttyS4，115200 8N1 raw）
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "bus/serial.hpp"
#include "common/clock.hpp"

namespace d1t {

PosixSerial::~PosixSerial() {
  if (fd_ >= 0) {
    tcflush(fd_, TCOFLUSH);  // 输出队列清空后 close 不会在内核里等 closing_wait
    ::close(fd_);
  }
}

int TtyQuery::out_queue(int fd, int& bytes) { return ::ioctl(fd, TIOCOUTQ, &bytes) == 0 ? 0 : -1; }

int TtyQuery::tx_empty(int fd, bool& empty) {
#if defined(TIOCSERGETLSR) && defined(TIOCSER_TEMT)
  unsigned int lsr = 0;
  if (::ioctl(fd, TIOCSERGETLSR, &lsr) != 0) return -1;
  empty = (lsr & TIOCSER_TEMT) != 0;
  return 0;
#else
  (void)fd;
  (void)empty;
  return -1;
#endif
}

bool drain_bounded(TtyQuery& q, int fd, size_t n, int64_t written_ns, int64_t deadline) {
  const int64_t poll_step = 200000;  // 0.2 ms
  bool fallback = false;
  for (;;) {  // ① 软件缓冲
    int left = 0;
    if (q.out_queue(fd, left) != 0) {
      fallback = true;
      break;
    }
    if (left == 0) break;
    if (now_mono_ns() >= deadline) return false;
    sleep_until_mono(std::min(now_mono_ns() + poll_step, deadline));
  }
  while (!fallback) {  // ② 硬件发送器（FIFO + 移位寄存器）
    bool empty = false;
    if (q.tx_empty(fd, empty) != 0) {
      fallback = true;
      break;
    }
    if (empty) return true;
    if (now_mono_ns() >= deadline) return false;
    sleep_until_mono(std::min(now_mono_ns() + poll_step, deadline));
  }
  // ③ 估算：从「最后一个字节交给驱动」起整帧再发一遍的时长 + FIFO 余量（偏保守）
  int64_t est_end = written_ns + int64_t(double(n) * 10.0 * 1e9 / 115200.0) + cfg::kUartFifoMarginMs * 1000000LL;
  if (est_end > deadline) return false;
  sleep_until_mono(est_end);
  return true;
}

bool PosixSerial::open(const std::string& port, int baud, std::string& err) {
  if (baud != 115200) {
    err = "只支持 115200（F10）";
    return false;
  }
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    err = "打开 " + port + " 失败：" + std::strerror(errno);
    return false;
  }
  // 独占：flock 与 pyserial exclusive=True 同机制；TIOCEXCL 阻止之后的其他 open
  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    err = port + " 已被占用（flock 失败）——先 systemctl stop 原服务";
    ::close(fd_);
    fd_ = -1;
    return false;
  }
#ifdef TIOCEXCL
  ::ioctl(fd_, TIOCEXCL);
#endif
  termios t{};
  if (tcgetattr(fd_, &t) != 0) {
    err = std::string("tcgetattr 失败：") + std::strerror(errno);
    return false;
  }
  cfmakeraw(&t);
  cfsetispeed(&t, B115200);
  cfsetospeed(&t, B115200);
  t.c_cflag &= ~(CSTOPB | PARENB | CSIZE);
  t.c_cflag |= CS8 | CLOCAL | CREAD;
#ifdef CRTSCTS
  t.c_cflag &= ~CRTSCTS;
#endif
  t.c_iflag &= ~(IXON | IXOFF | IXANY);
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &t) != 0) {
    err = std::string("tcsetattr 失败：") + std::strerror(errno);
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

// 写入 + 排空都受 MONOTONIC 总期限 kTxDeadlineMs 约束（R09、V2-01）：驱动长期写不进 / 排不空时
// 丢掉未发出的输出、返回 false，由主循环按写失败处理（§3.7-5），唯一的总线线程不会卡死。
// 本函数里没有不受期限约束的阻塞调用：write 非阻塞（O_NONBLOCK）、poll 超时 ≤ 剩余期限、
// 排空见 drain_bounded（不用阻塞式排空调用）。
bool PosixSerial::write_all(const uint8_t* p, size_t n) {
  const int64_t deadline = now_mono_ns() + int64_t(cfg::kTxDeadlineMs) * 1000000LL;
  size_t off = 0;
  while (off < n) {
    ssize_t w = ::write(fd_, p + off, n - off);
    if (w > 0) {
      off += size_t(w);
      continue;
    }
    if (w < 0 && errno != EAGAIN && errno != EINTR) break;
    int64_t remain = deadline - now_mono_ns();
    if (remain <= 0) break;
    pollfd pf{fd_, POLLOUT, 0};
    ::poll(&pf, 1, int(std::min<int64_t>(5, (remain + 999999) / 1000000)));
  }
  // 等发送完成：帧间隔从发送完成算起（F38，fsbus.py 同）
  if (off == n && drain_bounded(*tq_, fd_, n, now_mono_ns(), deadline)) return true;
  tcflush(fd_, TCOFLUSH);  // 丢掉没发出去的半帧，免得之后夹在别的帧里发出
  return false;
}

size_t PosixSerial::read_some(uint8_t* p, size_t cap, int64_t timeout_ns) {
  int ms = int((timeout_ns + 999999) / 1000000);
  if (ms < 0) ms = 0;
  pollfd pf{fd_, POLLIN, 0};
  int r = ::poll(&pf, 1, ms);
  if (r <= 0) return 0;
  ssize_t n = ::read(fd_, p, cap);
  return n > 0 ? size_t(n) : 0;
}

size_t PosixSerial::flush_input() {
  size_t total = 0;
  uint8_t tmp[256];
  for (;;) {
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n <= 0) break;
    total += size_t(n);
  }
  return total;
}

bool preflight_port(const std::string& port, std::string& msg) {
#if defined(__linux__)
  // ① 4 个原服务必须全部 inactive（fsbus.py MARM_SERVICES / preflight 同口径；F33）
  FILE* f = ::popen(
      "systemctl is-active marm_controller marm_communication marm_control marm_subscripber 2>/dev/null", "r");
  if (!f) {
    msg = "systemctl 不可用，原服务状态无法确定";
    return false;
  }
  char line[64];
  int n = 0, inactive = 0;
  std::string states;
  while (fgets(line, sizeof(line), f)) {
    std::string s(line);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    states += s + " ";
    ++n;
    if (s == "inactive") ++inactive;
  }
  ::pclose(f);
  if (n != 4 || inactive != 4) {
    msg = "原服务未全部停止（" + states + "）——先 systemctl stop 原服务";
    return false;
  }
  // ② /proc/<pid>/fd 中无其他进程打开该端口（fuser 同原理）
  char real[PATH_MAX];
  if (!::realpath(port.c_str(), real)) {
    msg = "realpath(" + port + ") 失败：" + std::strerror(errno);
    return false;
  }
  DIR* d = ::opendir("/proc");
  if (!d) {
    msg = "/proc 不可读，占用状态无法确定";
    return false;
  }
  bool denied = false;
  std::string holders;
  pid_t self = ::getpid();
  while (dirent* e = ::readdir(d)) {
    char* end;
    long pid = std::strtol(e->d_name, &end, 10);
    if (*end || pid <= 0 || pid == self) continue;
    std::string fdd = std::string("/proc/") + e->d_name + "/fd";
    DIR* fd = ::opendir(fdd.c_str());
    if (!fd) {
      if (errno == EACCES) denied = true;
      continue;
    }
    while (dirent* x = ::readdir(fd)) {
      if (x->d_name[0] == '.') continue;
      char tgt[PATH_MAX];
      std::string lp = fdd + "/" + x->d_name;
      ssize_t k = ::readlink(lp.c_str(), tgt, sizeof(tgt) - 1);
      if (k > 0) {
        tgt[k] = 0;
        if (std::strcmp(tgt, real) == 0) holders += std::string(e->d_name) + " ";
      }
    }
    ::closedir(fd);
  }
  ::closedir(d);
  if (!holders.empty()) {
    msg = port + " 仍被进程占用（pid " + holders + "）——先 systemctl stop 原服务";
    return false;
  }
  if (denied) {
    msg = "部分进程的 /proc/<pid>/fd 无权读取，占用状态无法确定（用 sudo 运行）";
    return false;
  }
  msg = "原服务 4×inactive，" + port + " 无占用";
  return true;
#else
  (void)port;
  msg = "非 Linux：跳过占用检查（只允许 --sim）";
  return false;
#endif
}

}  // namespace d1t
