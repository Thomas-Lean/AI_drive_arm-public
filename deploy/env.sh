# env.sh — 给当前终端设 DDS / 库 / Python 路径。必须 source，不要 bash 运行。
#
# 用法：source deploy/env.sh sim  [网卡] [对端IP] [角色]
#       source deploy/env.sh real <网卡> [对端IP] [角色]
#   sim ：同机仿真。网卡默认 lo（macOS 为 lo0），对端默认 127.0.0.1
#   real：连真臂。网卡 = 本机接臂网线的网卡（ip addr 查），对端默认臂 IP 192.168.123.100
#   角色：sdk（默认）| record | viz | ctrl，对应 DDS ParticipantIndex 3 / 5 / 6 / 0
#         同一台机器上每个 DDS 进程用不同角色（端口 = 7410 + 2×index，撞了起不来；
#         不固定 index 则单播下互相发现不了）。也可直接给数字。
#   网卡/对端传空串 '' 取默认，例：source deploy/env.sh sim '' '' record
#
# 设置：CYCLONEDDS_URI（固定网卡 + 关多播 + 单播 Peer + 固定 ParticipantIndex）、
#       LD_LIBRARY_PATH（Cyclone DDS 0.10.2 的库放最前：装了 ROS2 的终端会先加载另一版 libddsc，混用即崩）、
#       PYTHONPATH（加仓库 sdk/，import d1t_sdk 即可用）。fish 用户先敲 bash 再 source。

_d1t_env() {
  local mode="${1:-}" iface="${2:-}" peer="${3:-}" role="${4:-sdk}" idx root src p

  if [ -n "${BASH_VERSION:-}" ]; then src="${BASH_SOURCE[0]}"; else src="${(%):-%x}"; fi
  root="$(cd "$(dirname "$src")/.." && pwd)"

  case "$mode" in
    sim)
      [ -n "$iface" ] || { if [ "$(uname)" = Darwin ]; then iface=lo0; else iface=lo; fi; }
      [ -n "$peer" ] || peer=127.0.0.1 ;;
    real)
      if [ -z "$iface" ]; then
        echo "env.sh real 要给本机接臂的网卡名，例：source deploy/env.sh real enp5s0" >&2
        command -v ip >/dev/null 2>&1 && ip -br addr >&2
        return 2
      fi
      [ -n "$peer" ] || peer=192.168.123.100 ;;
    *)
      echo "用法：source deploy/env.sh sim|real [网卡] [对端IP] [sdk|record|viz|ctrl]" >&2
      return 2 ;;
  esac

  case "$role" in
    ctrl) idx=0 ;; sdk) idx=3 ;; record) idx=5 ;; viz) idx=6 ;;
    ''|*[!0-9]*) echo "角色只能是 sdk|record|viz|ctrl 或数字，收到：$role" >&2; return 2 ;;
    *) idx="$role" ;;
  esac

  export CYCLONEDDS_URI='<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="0"><General><Interfaces><NetworkInterface autodetermine="false" name="'"$iface"'"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>'"$idx"'</ParticipantIndex><Peers><Peer address="'"$peer"'"/></Peers></Discovery></Domain></CycloneDDS>'

  # 库路径：setup_host.sh 的安装前缀与 /usr/local，谁有 libddsc 谁放最前（去重后前置）
  for p in /usr/local/lib "${PREFIX:-$HOME/opt/cyclonedds-0.10.2}/lib"; do
    if ls "$p" 2>/dev/null | grep -q '^libddsc'; then
      LD_LIBRARY_PATH="$(printf '%s' ":${LD_LIBRARY_PATH:-}:" | sed "s#:$p:#:#g; s#^:##; s#:\$##")"
      export LD_LIBRARY_PATH="$p${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
  done

  PYTHONPATH="$(printf '%s' ":${PYTHONPATH:-}:" | sed "s#:$root/sdk:#:#g; s#^:##; s#:\$##")"
  export PYTHONPATH="$root/sdk${PYTHONPATH:+:$PYTHONPATH}"

  export D1T_ENV="$mode $iface $peer $role/$idx"
  echo "[env] $mode  网卡=$iface  对端=$peer  角色=${role}（ParticipantIndex ${idx}）"
  echo "[env] PYTHONPATH 前置 $root/sdk；LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-（未改）}"
}

if [ -n "${BASH_VERSION:-}" ] && [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "env.sh 要 source 才生效：source deploy/env.sh sim" >&2
  exit 2
fi
_d1t_env "$@"
_d1t_rc=$?
unset -f _d1t_env
return $_d1t_rc
