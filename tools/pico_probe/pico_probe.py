#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pico_probe — PICO 手柄输入只读探针（遥操作输入链路验证用）。

订阅 TeleRobot 查看器的 ZMQ 遥测（默认 tcp://127.0.0.1:55556，topic "xr"），用 TeleRobot
的 WebXRController + RelativePoseMapper 把指定手柄的相对位姿映射成末端目标位姿（基座系），
扳机线性映射 J6 开度，每秒打印一行：接收频率、LIVE/STALE、离合状态、目标 TCP（mm）、j6 目标。

只读：不连臂控制器、不发任何臂命令。末端参考用 SDK 运动学 fk() 在固定起始姿态下的结果
代替真臂读数（假硬件；实机遥操作时换成 arm.state()["tcp"]）。

依赖：numpy、scipy、pyzmq（TeleRobot 输入侧）、cyclonedds（导入 d1t_sdk 需要；本工具不建
DDS 参与者）。TeleRobot 目录：--telerobot-dir 或环境变量 TELEROBOT_DIR（默认 ~/TeleRobot）。

用法（TeleRobot 查看器已起，见本目录 README.md）：
    python3 pico_probe.py                       # 默认右手柄
    python3 pico_probe.py --side left
没有 PICO 时用假数据：python3 fake_xr_pub.py & python3 pico_probe.py
"""
from __future__ import annotations

import argparse
import collections
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SDK_DIR = REPO_ROOT / "sdk"
DEFAULT_TELEROBOT_DIR = Path(os.environ.get("TELEROBOT_DIR", Path.home() / "TeleRobot"))

# 假硬件起始姿态（URDF 角，度）
DEFAULT_HOME_Q = [0.0, -40.0, 60.0, 0.0, 30.0, 0.0]

# 扳机 → J6 全行程线性映射（控制器 J6 限位 −30…64.5°：扳机 0 = 全开约 70 mm，1 = 全闭约 4 mm）
J6_OPEN_DEG = 64.5
J6_CLOSED_DEG = -30.0

CTRL_STALE_S = 0.2    # WebXRController：超过此接收间隔判 STALE
MAPPER_STALE_S = 0.3  # RelativePoseMapper：超过此接收间隔清参考、须重新接合
GRIP_PRESS = 0.85     # 握把 > 此值接合
GRIP_RELEASE = 0.75   # 握把 < 此值脱开（滞回防抖）


def trigger_to_j6(trigger: float) -> float:
    """扳机模拟量 0–1 → J6 目标角（度）：64.5 − 94.5 × trigger。超出 [0,1] 截断。"""
    t = float(np.clip(trigger, 0.0, 1.0))
    return J6_OPEN_DEG + (J6_CLOSED_DEG - J6_OPEN_DEG) * t


def load_modules(telerobot_dir: Path, sdk_dir: Path) -> dict:
    """把 TeleRobot 与 sdk 目录加进 sys.path 后导入（两者都不是安装包）。"""
    for p in (str(sdk_dir), str(telerobot_dir)):
        if p not in sys.path:
            sys.path.insert(0, p)
    from scipy.spatial.transform import Rotation
    from teleop.vendor.pose_mapping import RelativePoseMapper
    from teleop.webxr_source import WebXRConfig, WebXRController

    from d1t_sdk import Kinematics
    from d1t_sdk._joint_model import deg2rad
    from d1t_sdk._kinematics import to_pose_dict
    return {"Rotation": Rotation, "RelativePoseMapper": RelativePoseMapper,
            "WebXRConfig": WebXRConfig, "WebXRController": WebXRController,
            "Kinematics": Kinematics, "deg2rad": deg2rad, "to_pose_dict": to_pose_dict}


def fake_tcp_pose(home_q, mods):
    """fk(固定起始姿态) → (位置 m, 3×3 旋转矩阵)；与 D1Arm.fk 同一实现，不建 DDS 参与者。"""
    pose = mods["to_pose_dict"](mods["Kinematics"]().fk([mods["deg2rad"](float(q)) for q in home_q]))
    w, x, y, z = pose["quat"]
    return np.array(pose["pos"], dtype=float), mods["Rotation"].from_quat([x, y, z, w]).as_matrix()


class PicoProbe:
    """探针核心：每帧把最新样本喂给离合映射；统计手柄样本接收频率。"""

    def __init__(self, mapper, ee_pos, ee_rot):
        self.mapper = mapper
        self.ee_pos = ee_pos
        self.ee_rot = ee_rot
        self.controller = None        # attach() 设定
        self.state = None
        self._rx = collections.deque()  # 每个被接收的手柄样本的时刻（monotonic）

    def attach(self, controller):
        self.controller = controller
        return controller

    def on_tracking(self, sample, message):
        """WebXRController 收到本侧手柄的有效新鲜样本时回调（订阅线程）。"""
        self._rx.append(time.monotonic())

    def rx_rate_hz(self, now, window_s=1.0):
        while self._rx and now - self._rx[0] > window_s:
            self._rx.popleft()
        return len(self._rx) / window_s

    def step(self, now=None):
        """跑一帧：取最新样本 → 离合映射。返回 (状态 dict, 离合状态是否变化)。"""
        now = time.monotonic() if now is None else now
        sample = self.controller.latest_sample()
        update = self.mapper.update(sample, self.ee_pos, self.ee_rot)
        trigger = sample.trigger if sample is not None else 0.0  # 断流时保留最后一次读数
        changed = update.state is not self.state
        self.state = update.state
        return {
            "rx_hz": self.rx_rate_hz(now),
            "live": self.controller.is_tracking,
            "state": update.state,
            "target": update.target,      # PoseTarget（基座系，m）；未接合时 None = 目标保持不动
            # 目标相对接合时 TCP 的位移（m）：假 TCP 固定，接合参考即 ee_pos
            "delta": None if update.target is None else update.target.position - self.ee_pos,
            "reference_captured": update.reference_captured,
            "require_release": update.require_release,
            "trigger": trigger,
            "j6_deg": trigger_to_j6(trigger),
        }, changed


STATE_LABEL = {"waiting": "等待输入", "idle": "脱开", "active": "接合", "stale": "断流", "hold": "保持"}


def _mm(v):
    return "[" + " ".join(f"{x * 1000:+7.1f}" for x in v) + "]"


def format_line(status, wall_time=None):
    target = status["target"]
    if target is not None:
        tgt = f"{_mm(target.position)} mm  Δ{_mm(status['delta'])}"
    else:
        tgt = "—（保持不动）"
    state = STATE_LABEL.get(status["state"].value, status["state"].value)
    if status["require_release"] and status["state"].value == "idle":
        state += "·须先松握把"
    stamp = time.strftime("%H:%M:%S", time.localtime(wall_time))
    return (f"[{stamp}] 接收 {status['rx_hz']:5.1f} Hz  {'LIVE ' if status['live'] else 'STALE'}  "
            f"{state:<8}  目标TCP {tgt}  j6 {status['j6_deg']:+6.1f}°")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:55556")
    parser.add_argument("--side", default="right", choices=("left", "right"))
    parser.add_argument("--telerobot-dir", type=Path, default=DEFAULT_TELEROBOT_DIR)
    parser.add_argument("--sdk-dir", type=Path, default=DEFAULT_SDK_DIR)
    parser.add_argument("--home-q", default=",".join(f"{q:g}" for q in DEFAULT_HOME_Q),
                        help="假硬件起始姿态（URDF 角，度，逗号分隔 6 个）")
    args = parser.parse_args(argv)

    mods = load_modules(args.telerobot_dir, args.sdk_dir)
    home_q = [float(x) for x in args.home_q.split(",")]
    if len(home_q) != 6:
        parser.error("--home-q 需要 6 个关节角")
    ee_pos, ee_rot = fake_tcp_pose(home_q, mods)

    mapper = mods["RelativePoseMapper"](side=args.side, grip_press_threshold=GRIP_PRESS,
                                        grip_release_threshold=GRIP_RELEASE,
                                        stale_timeout_s=MAPPER_STALE_S)
    probe = PicoProbe(mapper, ee_pos, ee_rot)
    config = mods["WebXRConfig"](hand_side=args.side, stale_timeout=CTRL_STALE_S,
                                 endpoint=args.endpoint)
    controller = probe.attach(mods["WebXRController"](config, on_tracking=probe.on_tracking))

    print(f"pico_probe：{args.endpoint}  {args.side} 手柄  假TCP "
          f"[{' '.join(f'{v * 1000:+.1f}' for v in ee_pos)}] mm = fk({args.home_q})")
    print(f"握把 > {GRIP_PRESS} 接合、< {GRIP_RELEASE} 脱开；j6 = {J6_OPEN_DEG} − "
          f"{J6_OPEN_DEG - J6_CLOSED_DEG} × 扳机。只读，不发臂命令。Ctrl+C 退出。", flush=True)
    controller.connect()
    report_at = 0.0
    try:
        while True:
            now = time.monotonic()
            status, changed = probe.step(now)
            if changed:
                note = "（捕获参考：目标 = 当前 TCP，不跳）" if status["reference_captured"] else ""
                label = STATE_LABEL.get(status["state"].value, status["state"].value)
                print(f"[{time.strftime('%H:%M:%S')}] → {label}{note}", flush=True)
            if now >= report_at:
                print(format_line(status), flush=True)
                report_at = now + 1.0
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        controller.disconnect()
        print("已停止。", flush=True)


if __name__ == "__main__":
    main()
