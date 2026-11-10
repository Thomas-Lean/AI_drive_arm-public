#!/usr/bin/env bash
# setup_host.sh — 上位机装依赖：Cyclone DDS 0.10.2（C/C++，源码）+ Python 包；可选 MuJoCo 显示环境。
# 幂等：已装且版本对就跳过，可反复跑。Linux（Ubuntu）为主；macOS 只做 Python 包。
# 用法：bash deploy/setup_host.sh                  （DDS 装到 ~/opt/cyclonedds-0.10.2）
#       bash deploy/setup_host.sh --with-viz       （另建 MuJoCo venv，给 tools/arm_viz 用）
#       PREFIX=/opt/cyclonedds-0.10.2 VIZ_VENV=~/.venvs/d1t-viz bash deploy/setup_host.sh --with-viz
# 版本必须 0.10.2：与臂上一致，不同版本不保证互通。
# （本文件由旧版上位机安装脚本改名扩充而来。）
set -euo pipefail

WITH_VIZ=0
for a in "$@"; do
  case "$a" in
    --with-viz) WITH_VIZ=1 ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "未知参数：${a}（可用 --with-viz）" >&2; exit 2 ;;
  esac
done

PREFIX="${PREFIX:-$HOME/opt/cyclonedds-0.10.2}"
VIZ_VENV="${VIZ_VENV:-$HOME/.venvs/mujoco-3.14.0}"
VER=0.10.2
DDS_HOME=""   # 实际找到 / 装好的 C 库前缀（MuJoCo venv 里源码装 cyclonedds 要用）

if [ "$(uname)" = Linux ]; then
  echo "== 1. 系统依赖（缺才装，需 sudo） =="
  miss=()
  for c in cmake git make gcc g++; do command -v "$c" >/dev/null 2>&1 || miss+=("$c"); done
  dpkg -s build-essential >/dev/null 2>&1 || miss+=(build-essential)
  python3 -m pip --version >/dev/null 2>&1 || miss+=(python3-pip)
  if [ "$WITH_VIZ" = 1 ] && ! python3 -c 'import ensurepip' >/dev/null 2>&1; then miss+=(python3-venv); fi
  if [ ${#miss[@]} -gt 0 ]; then
    echo "缺：${miss[*]}"
    sudo apt-get update -qq
    sudo apt-get install -y -qq cmake git build-essential python3-pip python3-venv
  else
    echo "cmake / git / 编译器 / pip 已装，跳过"
  fi

  echo "== 2. Cyclone DDS C + C++ ${VER}（源码装到 ${PREFIX}） =="
  # /usr/local 下已有（如出厂带 DDS 的机器）也认：libddsc 与 libddscxx 都在即跳过
  for p in "$PREFIX" /usr/local; do
    if ls "$p"/lib/libddsc.so* >/dev/null 2>&1 && ls "$p"/lib/libddscxx.so* >/dev/null 2>&1; then
      echo "已检测到 $p/lib 下 libddsc + libddscxx，跳过编译（版本不核对；要重装先删 ${PREFIX}）"
      DDS_HOME="$p"; break
    fi
  done
  if [ -z "$DDS_HOME" ]; then
    WORK="$(mktemp -d)"
    trap 'rm -rf "$WORK"' EXIT
    git clone --branch ${VER} --depth 1 https://github.com/eclipse-cyclonedds/cyclonedds "$WORK/cyclonedds"
    cmake -S "$WORK/cyclonedds" -B "$WORK/cyclonedds/build" \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
    cmake --build "$WORK/cyclonedds/build" -j"$(nproc)"
    cmake --install "$WORK/cyclonedds/build"
    # 仓库路径是 eclipse-cyclonedds/cyclonedds-cxx（旧 cyclonedds/ 路径已 404）
    git clone --branch ${VER} --depth 1 https://github.com/eclipse-cyclonedds/cyclonedds-cxx "$WORK/cyclonedds-cxx"
    cmake -S "$WORK/cyclonedds-cxx" -B "$WORK/cyclonedds-cxx/build" \
      -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF \
      -DCMAKE_PREFIX_PATH="$PREFIX"
    cmake --build "$WORK/cyclonedds-cxx/build" -j"$(nproc)"
    cmake --install "$WORK/cyclonedds-cxx/build"
    trap - EXIT
    rm -rf "$WORK"
    DDS_HOME="$PREFIX"
  fi
else
  echo "== macOS：跳过系统依赖与 DDS 编译（仿真控制器需自行把 Cyclone DDS 0.10.2 装到 ${PREFIX}） =="
fi

echo "== 3. Python 依赖（用户级，不动系统包） =="
PYVER="$(python3 -c 'import importlib.metadata as m; print(m.version("cyclonedds"))' 2>/dev/null || true)"
if [ "$PYVER" = "$VER" ]; then echo "pip cyclonedds $PYVER 已装，跳过"; else python3 -m pip install --user "cyclonedds==${VER}"; fi
NUMPY_VER="$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || true)"
if [ -n "$NUMPY_VER" ]; then echo "numpy $NUMPY_VER 已装，跳过"; else python3 -m pip install --user numpy; fi

if [ "$WITH_VIZ" = 1 ]; then
  echo "== 4. 可选：MuJoCo 显示环境（独立 venv ${VIZ_VENV}） =="
  if "$VIZ_VENV/bin/python" -c 'import mujoco, numpy, importlib.metadata as m; assert m.version("cyclonedds") == "0.10.2"' >/dev/null 2>&1; then
    echo "$VIZ_VENV 已有 mujoco $("$VIZ_VENV/bin/python" -c 'import mujoco; print(mujoco.__version__)') + cyclonedds 0.10.2，跳过"
  else
    [ -n "$DDS_HOME" ] || { echo "没找到 Cyclone DDS C 库前缀，无法在 venv 里源码装 cyclonedds" >&2; exit 1; }
    python3 -m venv "$VIZ_VENV"
    "$VIZ_VENV/bin/python" -m pip install -q --upgrade pip
    "$VIZ_VENV/bin/python" -m pip install "mujoco==3.14.0" numpy
    # 源码装 cyclonedds 并链接本机 0.10.2 C 库（预编译 wheel 可能附带不同版本的 C 库）
    CYCLONEDDS_HOME="$DDS_HOME" "$VIZ_VENV/bin/python" -m pip install --no-binary=cyclonedds "cyclonedds==${VER}"
  fi
fi

echo "== 自检 =="
[ -z "$DDS_HOME" ] || echo "C 库: $DDS_HOME/lib"
python3 -c 'import cyclonedds, importlib.metadata as m; print("pip cyclonedds", m.version("cyclonedds"))'
python3 -c 'import numpy; print("numpy", numpy.__version__)'
[ "$WITH_VIZ" = 0 ] || "$VIZ_VENV/bin/python" -c 'import mujoco, importlib.metadata as m; print("viz venv: mujoco", mujoco.__version__, "/ cyclonedds", m.version("cyclonedds"))'

cat <<EOF

完成。下一步（仿真）：
  bash deploy/sim_start.sh                  # 终端 1：起仿真控制器（首次自动编译 + 测试）
  source deploy/env.sh sim                  # 终端 2：设好环境后跑 python3 sdk/examples/01_read_state.py
EOF
