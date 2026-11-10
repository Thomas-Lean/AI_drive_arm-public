#!/usr/bin/env bash
# arm_start.sh — 臂载计算机上：停原厂 4 服务 → sudo 启动 d1t_arm_ctrl（URI 内联）
# 用法：bash deploy/arm_start.sh <臂上网卡名> <上位机IP> [额外的控制器参数...]
#   例：bash deploy/arm_start.sh eth0 192.168.123.233 -v
# 网卡名在臂上 `ip addr` 查（接上位机网线的那个）。回官方服务用 deploy/arm_restore.sh。
set -euo pipefail

if [ $# -lt 2 ]; then
  echo "用法：bash $0 <臂上网卡名> <上位机IP> [控制器参数...]" >&2
  echo "  例：bash $0 eth0 192.168.123.233 -v" >&2
  exit 2
fi
IFACE="$1"; PEER="$2"; shift 2

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/arm_onboard/controller/build/d1t_arm_ctrl"
[ -x "$BIN" ] || { echo "缺 ${BIN}，先跑 bash deploy/arm_install.sh" >&2; exit 2; }
ip link show "$IFACE" >/dev/null 2>&1 || { echo "臂上没有网卡 ${IFACE}（ip addr 查）" >&2; exit 2; }
if pgrep -x d1t_arm_ctrl >/dev/null 2>&1; then
  echo "已有 d1t_arm_ctrl 在跑（PID $(pgrep -x d1t_arm_ctrl | tr '\n' ' ')），不重复启动" >&2
  exit 2
fi

echo "== 1/2 停原厂 4 服务（控制器启动自检要求它们 inactive） =="
sudo systemctl stop marm_controller marm_communication marm_control marm_subscripber
systemctl status marm_controller marm_communication marm_control marm_subscripber --no-pager | grep Active || true

echo "== 2/2 启动控制器（前台，sudo；Ctrl-C 退出并自动发全轴保持） =="
# sudo 会丢环境变量 → URI 放进 sudo env 的命令行。ParticipantIndex 固定 0（端口 7410），
# 单播 Peer 指向上位机；上位机侧用 deploy/env.sh real（SDK 为 index 3）。
URI='<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="0"><General><Interfaces><NetworkInterface autodetermine="false" name="'"$IFACE"'"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>0</ParticipantIndex><Peers><Peer address="'"$PEER"'"/></Peers></Discovery></Domain></CycloneDDS>'
# 退出码 3 = 原服务没停干净或串口被占；4 = 启动自检失败（如 MTURN）；见 arm_onboard/controller/README.md §6
exec sudo env CYCLONEDDS_URI="$URI" "$BIN" "$@"
