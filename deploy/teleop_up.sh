#!/usr/bin/env bash
# teleop_up.sh — 上位机 上一条命令起 PICO 遥操作全套（tmux 会话 teleop），停也一条命令。
#
# 用法：bash deploy/teleop_up.sh sim  [--viz] [-- 遥操作附加参数...]
#       bash deploy/teleop_up.sh real [-- 遥操作附加参数...]
#       bash deploy/teleop_up.sh stop
#       bash deploy/teleop_up.sh check sim|real      # 只做启动前检查，不起任何进程
#
#   sim ：起仿真控制器 + TeleRobot + 遥操作（--viz 再开 MuJoCo 画面）
#   real：真臂控制器须已在臂上跑（臂上用 deploy/arm_start.sh 起）；本脚本只起 TeleRobot + 遥操作，
#         遥操作用实机基线参数。本脚本不发任何运动命令：起步时臂在哪就停在哪，按 A 回准备位。
#   环境变量：PICO_IP（默认 192.168.123.253）、WIFI_IF（默认 wlo1）、ARM_IF（默认 enp5s0）
#
# 看进程：tmux attach -t teleop（Ctrl-b 再按数字切窗口，Ctrl-b d 退出查看、进程继续跑）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TELEROBOT="$HOME/TeleRobot"
CONDA="$HOME/miniforge3/bin/activate"
MUJOCO_PY="$HOME/.venvs/mujoco-3.14.0/bin/python"
PICO_IP="${PICO_IP:-192.168.123.253}"
WIFI_IF="${WIFI_IF:-wlo1}"
ARM_IF="${ARM_IF:-enp5s0}"
SESSION=teleop
REAL_ARGS="--real-ok --max-vel 0.20 --grip-vel 60 --goto-vel 40"   # 实机基线（实机验收时定）

die() { echo "✗ $*" >&2; exit 1; }
ok()  { echo "✓ $*"; }

wifi_ip() { ip -4 -br addr show "$WIFI_IF" 2>/dev/null | awk '{print $3}' | cut -d/ -f1; }

read_state() {   # $1 = sim|real：只读一次臂状态，读到返回 0
  if [ "$1" = sim ]; then set -- sim; else set -- real "$ARM_IF"; fi
  (source "$CONDA" d1t-teleop && cd "$ROOT" && source deploy/env.sh "$@" >/dev/null \
    && timeout 8 python3 -m d1t_sdk state >/dev/null 2>&1)
}

checks() {   # $1 = sim|real
  local mode=$1 ip
  ip=$(wifi_ip); [ -n "$ip" ] || die "$WIFI_IF 没有 IPv4 地址：上位机 没连上路由器 Wi-Fi"
  ok "上位机 Wi-Fi $WIFI_IF = $ip"

  grep -q "host: $ip" "$TELEROBOT/config.yaml" \
    || die "TeleRobot config.yaml 的 xr.host 不是 $ip：vim $TELEROBOT/config.yaml 改成 $ip"
  ok "TeleRobot xr.host = $ip"

  if grep -q d1t_haptics "$TELEROBOT/telerobot/viewer.py" 2>/dev/null; then
    ok "TeleRobot 已接入振动脚本"
  else
    echo "! TeleRobot 未接入振动脚本：遥操作照常可用，手柄不振（见 tools/pico_teleop/README.md 已知限制 1）"
  fi

  if ip route get "$PICO_IP" 2>/dev/null | grep -q "dev $WIFI_IF"; then
    ok "到 PICO $PICO_IP 走 $WIFI_IF"
  else
    die "到 PICO $PICO_IP 不走 $WIFI_IF（上位机重启后路由会丢）。先敲：
    sudo ip route add $PICO_IP/32 dev $WIFI_IF src $ip
  PICO 换了 IP 就用 PICO_IP=<新IP> 重跑本脚本"
  fi

  tmux has-session -t "$SESSION" 2>/dev/null && die "tmux 会话 $SESSION 已在跑：先 bash deploy/teleop_up.sh stop"
  for port in 8012 8000; do
    ss -ltn "sport = :$port" | grep -q LISTEN && die "端口 $port 被占（上次的 TeleRobot / 遥操作没退干净？）：ss -ltnp | grep $port"
  done
  pgrep -f tools/pico_teleop/pico_teleop.py >/dev/null && die "已有遥操作进程在跑：pgrep -af pico_teleop"
  ok "端口 8012 / 8000 空闲，无残留进程"

  if [ "$mode" = sim ]; then
    if pgrep -x d1t_arm_ctrl >/dev/null; then
      pgrep -af d1t_arm_ctrl | grep -q -- --sim || die "本机有不带 --sim 的控制器在跑：停手，先确认是谁起的"
      read_state sim || die "本机仿真控制器在跑但按 sim 环境读不到状态（可能是别的会话测试用的、换了 DDS 端口组）：
    确认没人在用后 pkill -x d1t_arm_ctrl，再重跑本脚本（脚本会自己起一个）"
      ok "仿真控制器已在跑且能读到状态（复用）"
    fi
  else
    pgrep -x d1t_arm_ctrl >/dev/null && die "real 模式下 上位机 本机不该有控制器（真控制器在臂上）：pgrep -af d1t_arm_ctrl"
    read_state real \
      || die "读不到真臂状态：臂上先起控制器（ssh ubuntu@192.168.123.100，bash deploy/arm_start.sh eth0 192.168.123.233 -v）"
    ok "真臂控制器在线（只读了一次状态）"
  fi
}

win() {   # $1 = 窗口名，$2 = 命令；进程退出后窗口停住，方便看最后的报错
  tmux new-window -t "$SESSION" -n "$1" "bash -c '$2; echo; echo \"[$1] 已退出，按回车关闭窗口\"; read'"
}

up() {   # $1 = sim|real，其余 = 附加参数
  local mode=$1; shift
  local viz=0 extra=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --viz) viz=1 ;;
      --) shift; extra=("$@"); break ;;
      *) die "不认识的参数 $1（遥操作参数放在 -- 后面）" ;;
    esac
    shift
  done
  [ "$mode" = real ] && [ "$viz" = 1 ] && die "real 模式不开画面（看真臂）"

  checks "$mode"
  local ip; ip=$(wifi_ip)

  tmux new-session -d -s "$SESSION" -n info "bash -c 'echo 遥操作会话 $mode；bash deploy/teleop_up.sh stop 全部停止; exec bash'"
  if [ "$mode" = sim ] && ! pgrep -x d1t_arm_ctrl >/dev/null; then
    win ctrl "cd $ROOT && bash deploy/sim_start.sh"
    for _ in $(seq 20); do sleep 0.5; pgrep -x d1t_arm_ctrl >/dev/null && break; done
    sleep 2
    read_state sim || { stop; die "仿真控制器起来后读不到状态：tmux 里看 ctrl 窗口的报错"; }
    ok "仿真控制器已起"
  fi
  win telerobot "source $CONDA telerobot && cd $TELEROBOT && python start.py --no-image --ip $ip"
  if [ "$viz" = 1 ]; then
    win viz "cd $ROOT && source deploy/env.sh sim \"\" \"\" viz && __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia DISPLAY=:0 $MUJOCO_PY tools/arm_viz/arm_viz.py"
  fi
  local env_args args
  if [ "$mode" = sim ]; then env_args="sim"; args=""; else env_args="real $ARM_IF"; args="$REAL_ARGS"; fi
  sleep 3   # 等 TeleRobot 的 ZMQ 起来
  win teleop "source $CONDA d1t-teleop && cd $ROOT && source deploy/env.sh $env_args && python3 tools/pico_teleop/pico_teleop.py $args --calibrate --haptics-ip $ip ${extra[*]:-}"

  cat <<EOF

已起 tmux 会话 $SESSION（$mode）。看输出：tmux attach -t $SESSION，Ctrl-b 再按数字切窗口。

PICO 上：
  1. 浏览器开 https://$ip:8012/dashboard →「进入 Pico XR 视图」进 VR
     （第一次用、或 上位机 Wi-Fi IP 变了：先分别开 https://$ip:8000 和 https://$ip:8012 信任证书）
  2. 面朝基座 +X（与臂同向）站，按住握把沿 +X 推约 25 cm 再松开 → teleop 窗口打印偏角
  3. 臂若在折叠位：松开握把按住 A 0.5 s 回准备位，再按握把接管
停止：bash deploy/teleop_up.sh stop
EOF
}

stop() {
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux send-keys -t "$SESSION:teleop" C-c 2>/dev/null || true   # 先让遥操作 stream_stop（臂 HOLD 在原地）
    sleep 2
    tmux send-keys -t "$SESSION:telerobot" C-c 2>/dev/null || true
    tmux send-keys -t "$SESSION:viz" C-c 2>/dev/null || true
    tmux send-keys -t "$SESSION:ctrl" C-c 2>/dev/null || true
    sleep 3
    tmux kill-session -t "$SESSION"
    echo "已停止 tmux 会话 $SESSION（臂 HOLD 在当前姿态；下电收臂另用 python3 -m d1t_sdk park）"
  else
    echo "没有在跑的 $SESSION 会话"
  fi
}

case "${1:-}" in
  sim|real) up "$@" ;;
  stop) stop ;;
  check) [ "${2:-}" = sim ] || [ "${2:-}" = real ] || die "用法：bash deploy/teleop_up.sh check sim|real"; checks "$2"; ok "检查全部通过" ;;
  *) sed -n 2,16p "$0"; exit 1 ;;
esac
