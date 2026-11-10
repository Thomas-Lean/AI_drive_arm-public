#!/usr/bin/env bash
# arm_restore.sh — 回官方控制：停 d1t_arm_ctrl（若在跑）→ 等它交还串口 → 拉起原厂 4 服务
# 用法：bash deploy/arm_restore.sh
# 本项目只 stop 过原服务、从未 disable；本脚本也不改任何官方文件。
set -euo pipefail

echo "== 1/3 停本控制器 =="
if systemctl is-active --quiet d1t-arm-ctrl 2>/dev/null; then
  sudo systemctl stop d1t-arm-ctrl          # 装了可选自启服务时
fi
if pgrep -x d1t_arm_ctrl >/dev/null 2>&1; then
  sudo pkill -INT -x d1t_arm_ctrl || true   # 同 Ctrl-C：先发全轴保持帧再退出
else
  echo "d1t_arm_ctrl 未在跑"
fi

echo "== 2/3 等进程退出、交还串口（内核排空发送缓冲最长约 30 s） =="
for _ in $(seq 1 40); do
  pgrep -x d1t_arm_ctrl >/dev/null 2>&1 || break
  sleep 1
done
if pgrep -x d1t_arm_ctrl >/dev/null 2>&1; then
  echo "40 s 后 d1t_arm_ctrl 仍在，先不拉原服务；查 ps / 必要时重启臂" >&2
  exit 1
fi

echo "== 3/3 拉起原厂 4 服务 =="
sudo systemctl start marm_controller marm_communication marm_control marm_subscripber
systemctl status marm_controller marm_communication marm_control marm_subscripber --no-pager | grep Active || true
