// check.hpp —— 最小单测宏（不引第三方框架，臂上也能编）
#pragma once

#include <cmath>
#include <cstdio>

namespace chk {
inline int& fails() {
  static int f = 0;
  return f;
}
inline int& total() {
  static int t = 0;
  return t;
}
inline int report(const char* name) {
  std::printf("[%s] %d/%d 通过%s\n", name, total() - fails(), total(), fails() ? "  ← 有失败" : "");
  return fails() ? 1 : 0;
}
}  // namespace chk

#define CHECK(cond)                                                       \
  do {                                                                    \
    ++chk::total();                                                       \
    if (!(cond)) {                                                        \
      ++chk::fails();                                                     \
      std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
    }                                                                     \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                    \
  do {                                                                                           \
    ++chk::total();                                                                              \
    double _a = (a), _b = (b);                                                                   \
    if (!(std::fabs(_a - _b) <= (tol))) {                                                        \
      ++chk::fails();                                                                            \
      std::printf("  FAIL %s:%d  %s=%g vs %s=%g\n", __FILE__, __LINE__, #a, _a, #b, _b);         \
    }                                                                                            \
  } while (0)
