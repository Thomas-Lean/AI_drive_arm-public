#!/usr/bin/env bash
# sim_start.sh — 起 --sim 控制器：假舵机、同机 lo 单播、ParticipantIndex 0。前台运行，Ctrl-C 退出。
# 用法：bash deploy/sim_start.sh [控制器参数...]
#   例：bash deploy/sim_start.sh --sim-drop 3:0.05      # 仿真 J3 丢包 5%（其余注入参数见控制器 README §3）
# 没编译过会先调 deploy/arm_install.sh（编译 + ctest）。另一终端 source deploy/env.sh sim 后跑 SDK。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/arm_onboard/controller/build/d1t_arm_ctrl"

# 同机只能有一个控制器（它固定占 ParticipantIndex 0 / 端口 7410）
if pgrep -x d1t_arm_ctrl >/dev/null 2>&1; then
  echo "已有 d1t_arm_ctrl 在跑（PID $(pgrep -x d1t_arm_ctrl | tr '\n' ' ')），先结束它再起仿真" >&2
  exit 1
fi
[ -x "$BIN" ] || bash "$ROOT/deploy/arm_install.sh"

# shellcheck source=deploy/env.sh
source "$ROOT/deploy/env.sh" sim '' '' ctrl
if [ "$(uname)" = Darwin ]; then
  export DYLD_LIBRARY_PATH="${PREFIX:-$HOME/opt/cyclonedds-0.10.2}/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
fi
echo "[sim] 看到「启动完成：IDLE」与「DDS domain 0：收 … 发 …」即就绪；另一终端：source deploy/env.sh sim"
exec "$BIN" --sim -v "$@"
