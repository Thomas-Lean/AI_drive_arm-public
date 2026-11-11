#!/usr/bin/env bash
# arm_install.sh — 编译 d1t_arm_ctrl 并跑 ctest（不动臂）。臂载计算机与上位机（仿真用）通用。
# 用法：在仓库根（或拷到臂上的仓库副本根）  bash deploy/arm_install.sh
#   Cyclone DDS 在 /usr/local（臂上出厂即有）时自动直引；装在 setup_host.sh 的前缀
#   （默认 ~/opt/cyclonedds-0.10.2，可用 PREFIX= 改）时自动传给 CMake。
# 产物：arm_onboard/controller/build/d1t_arm_ctrl（arm_start.sh / sim_start.sh 用它启动）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CTRL="$ROOT/arm_onboard/controller"
BUILD="$CTRL/build"
PREFIX="${PREFIX:-$HOME/opt/cyclonedds-0.10.2}"
if [ -z "${CMAKE_PREFIX_PATH:-}" ] && [ -d "$PREFIX/lib" ]; then
  export CMAKE_PREFIX_PATH="$PREFIX"
fi

echo "== 1/3 编译（Release） =="
cmake -S "$CTRL" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
# 臂上时钟常与上位机不一致：旧产物若时间戳在「未来」，make 会跳过重新链接，跑的仍是旧版本 → 先删产物强制链接
rm -f "$BUILD/d1t_arm_ctrl"
cmake --build "$BUILD" -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)"
# 配置输出里「D1T DDS: ON」才有 DDS（跨机收发）；OFF 时只能 --fake-client

echo "== 2/3 ctest（不上臂：编解码 / safety 单测 + --sim 集成） =="
(cd "$BUILD" && ctest --output-on-failure)

echo "== 3/3 完成 =="
ls -l "$BUILD/d1t_arm_ctrl"
echo "臂上启动：bash deploy/arm_start.sh <臂上网卡名> <上位机IP>；上位机仿真：bash deploy/sim_start.sh"
