#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pico_teleop — PICO 手柄遥操作主程序。

PICO（或假发布器）──ZMQ──▶ PicoProbe（离合映射）──▶ TeleopLoop（heading/limiter/
GripLock）──▶ d1t_sdk stream_pose ──▶ d1t_arm_ctrl；grasp/empty 事件 ─▶ 振动页面。
仿真和真臂同一程序，只换 deploy/env.sh sim|real 的网卡参数。

环境守卫：D1T_ENV 不以 sim 开头且未加 --real-ok 时拒绝启动（防对真控制器误发）。
循环节奏跟 arm.wait_frame()（20 Hz），不用 sleep；每帧写一行 jsonl（--log）。

用法（ubox；先起控制器与 TeleRobot 查看器/假发布器，见本目录 README）：
    source deploy/env.sh sim && python3 tools/pico_teleop/pico_teleop.py [参数]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for _p in (str(HERE), str(HERE.parent / "pico_probe"), str(HERE / "haptics")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import pico_probe  # noqa: E402
from pico_probe import (CTRL_STALE_S, GRIP_PRESS, GRIP_RELEASE, MAPPER_STALE_S,  # noqa: E402
                        STATE_LABEL, PicoProbe)
from d1t_sdk import D1Arm, D1ArmError  # noqa: E402
from d1t_sdk._common import CTRL_ESTOP, CTRL_FAULT  # noqa: E402

from griplock import CLOSE_SPEED_DPS, GripLock  # noqa: E402
from goto import (FOLD_GRIP_OPEN_MIN_MM, GOTO_VEL_DPS, ButtonHold, GotoMotion,  # noqa: E402
                  near_fold)
from teleop_loop import (DEFAULT_MAX_ROT_VEL_DPS, DT_S, TeleopLoop, WS_BOX_M,  # noqa: E402
                         state_str, yaw_from_push)

ROT_FILTER_HZ = 3.0   # 手柄姿态低通截止（Hz）：有意转腕 < 2 Hz 通过，手抖 8–12 Hz 削弱；滞后约 1 帧
from haptics_server import HapticsServer  # noqa: E402

REPO_ROOT = HERE.parents[1]


def step_probe(probe: PicoProbe, arm: D1Arm, rotation_cls):
    """走一帧探针：ee 参考换成臂当前实测 TCP（不再用假姿态）。"""
    st = arm.state()
    w, x, y, z = st["tcp"]["quat"]
    probe.ee_pos = np.array(st["tcp"]["pos"], dtype=float)
    probe.ee_rot = rotation_cls.from_quat([x, y, z, w]).as_matrix()
    return probe.step()


def box_violations(tcp) -> list:
    """TCP 不在工作空间盒内的轴（空 = 在盒内）。"""
    return [f"{'xyz'[i]}={tcp[i]:.3f}∉[{lo:.2f},{hi:.2f}]"
            for i, (lo, hi) in enumerate(WS_BOX_M) if not lo <= tcp[i] <= hi]


def read_ab(probe: PicoProbe, live: bool):
    """本侧手柄 A/B（右手；左手为 X/Y）是否按下；断流时一律视为未按。"""
    sample = probe.controller.latest_sample() if live else None
    if sample is None:
        return False, False
    return bool(getattr(sample, "primary_button", False)), \
        bool(getattr(sample, "secondary_button", False))


GOTO_LABEL = {"ready": "回准备位", "fold": "回折叠位"}


def calibrate_yaw(arm, probe, rotation_cls, timeout_s: float = 120.0) -> float:
    """朝向标定手势：握住握把把手向正前方推约 20 cm 再松开，返回修正 yaw（度）。"""
    print("朝向标定：握住握把，把手向正前方推约 20 cm，然后松开（120 s 内完成）", flush=True)
    t0 = time.monotonic()
    ref = last = None
    while time.monotonic() - t0 < timeout_s:
        arm.wait_frame()
        status, _ = step_probe(probe, arm, rotation_cls)
        st = status["state"].value
        if st == "active" and status["target"] is not None:
            if ref is None:
                ref = np.array(status["target"].position, dtype=float)
                print("已接合，推完就松开握把", flush=True)
            last = np.array(status["target"].position, dtype=float)
        elif ref is not None and st != "active":
            try:
                yaw = yaw_from_push(ref, last)
            except ValueError as e:
                print(f"{e}；重新握住再推一次", flush=True)
                ref = last = None
                continue
            theta = -yaw
            print(f"推程水平偏角 {theta:+.1f}°（相对基座 +X 前方），本次运行修正 yaw {yaw:+.1f}°",
                  flush=True)
            return yaw
    raise TimeoutError(f"{timeout_s:.0f}s 内没完成标定手势")


def format_line(out: dict, rx_hz: float, live: bool) -> str:
    state = STATE_LABEL.get(out["probe_state"], out["probe_state"])
    if out["event"]:
        state += f"·{out['event']}"
    tgt = ("[" + " ".join(f"{v * 1000:+6.1f}" for v in out["pos"]) + "] mm"
           if out["pos"] is not None else "—保持")
    flags = []
    if out["locked"]:
        flags.append(f"锁{out['lock_deg']:+.1f}°")
    if out["step_clamped"]:
        flags.append("步长截断")
    if out["box_clamped"]:
        flags.append("出盒钳制")
    if out["ik_fail"]:
        flags.append(f"逆解失败×{out['ik_fail']}")
    return (f"[{time.strftime('%H:%M:%S')}] {rx_hz:5.1f} Hz {'LIVE ' if live else 'STALE'} "
            f"{state:<10} 目标 {tgt}  j6 {out['j6_out']:+6.1f}°"
            f"（cmd {out['j6_cmd']:+6.1f} meas {out['j6_meas']:+6.1f}）"
            f"  {' '.join(flags)}")


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--endpoint", default="tcp://127.0.0.1:55556",
                    help="TeleRobot 查看器/假发布器的 ZMQ 遥测端点")
    ap.add_argument("--side", default="right", choices=("left", "right"))
    ap.add_argument("--telerobot-dir", type=Path, default=pico_probe.DEFAULT_TELEROBOT_DIR)
    ap.add_argument("--sdk-dir", type=Path, default=pico_probe.DEFAULT_SDK_DIR)
    ap.add_argument("--yaw-deg", type=float, default=None,
                    help="水平朝向修正（度，绕 Z）；不给了又没 --calibrate 则为 0")
    ap.add_argument("--calibrate", action="store_true",
                    help="启动时先做朝向标定手势（推前方 20 cm），算出本次运行的 yaw")
    ap.add_argument("--max-vel", type=float, default=0.25,
                    help="TCP 速度上限 m/s（单帧步长 = 值×0.05 s；实机首跑建议 0.10）")
    ap.add_argument("--max-rot-vel", type=float, default=DEFAULT_MAX_ROT_VEL_DPS,
                    help=f"TCP 姿态转速上限 °/s（默认 {DEFAULT_MAX_ROT_VEL_DPS:g}，单帧转角 = 值×0.05 s）")
    ap.add_argument("--rot-filter-hz", type=float, default=ROT_FILTER_HZ,
                    help=f"手柄姿态低通截止频率 Hz（默认 {ROT_FILTER_HZ:g}，滤手抖；0 = 关）")
    ap.add_argument("--grip-vel", type=float, default=CLOSE_SPEED_DPS,
                    help=f"夹爪闭合限速 °/s（默认 {CLOSE_SPEED_DPS:g}，上限 100；实机首跑建议 60）")
    ap.add_argument("--goto-vel", type=float, default=GOTO_VEL_DPS,
                    help=f"A/B 一键回位的关节速度 °/s（默认 {GOTO_VEL_DPS:g}）")
    ap.add_argument("--fake-grasp-at", type=float, default=None,
                    help="调试：把该角度以下的 J6 实测钳在该角度（模拟夹到物体，验证锁定/振动）")
    ap.add_argument("--haptics-port", type=int, default=8000, help="振动页面端口，0 = 不起")
    ap.add_argument("--haptics-ip", default=None,
                    help="Wi-Fi 接法：本机局域网 IP（生成证书并监听 0.0.0.0，PICO 才能开页面）")
    ap.add_argument("--duration", type=float, default=0.0, help="跑 N 秒自动退出（0 = Ctrl-C 退）")
    ap.add_argument("--log", type=Path, default=None,
                    help="jsonl 路径；默认 tests/runs/<日期>-遥操作/teleop-<时分秒>.jsonl")
    ap.add_argument("--real-ok", action="store_true",
                    help="确认允许连非 sim 环境（实机启用；默认拒绝，防误发）")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not 0 < args.max_rot_vel <= 180 or args.rot_filter_hz < 0:
        print("--max-rot-vel 须在 (0, 180] °/s，--rot-filter-hz 须 ≥ 0", file=sys.stderr)
        return 2
    if not 0 < args.grip_vel <= 100 or not 0 < args.goto_vel <= 100:
        print("--grip-vel / --goto-vel 须在 (0, 100] °/s", file=sys.stderr)
        return 2

    env = os.environ.get("D1T_ENV", "")
    if not env:
        print("未检测到 D1T_ENV：先 source deploy/env.sh sim（实机 source deploy/env.sh real <网卡>）",
              file=sys.stderr)
        return 2
    if not env.startswith("sim") and not args.real_ok:
        print(f"当前环境 D1T_ENV='{env}' 不是 sim：加 --real-ok 才允许连真控制器", file=sys.stderr)
        return 2
    print(f"环境 D1T_ENV='{env}'  endpoint {args.endpoint}  {args.side} 手柄"
          f"{'  --fake-grasp-at ' + str(args.fake_grasp_at) if args.fake_grasp_at is not None else ''}",
          flush=True)

    mods = pico_probe.load_modules(args.telerobot_dir, args.sdk_dir)
    rotation_cls = mods["Rotation"]

    arm = D1Arm()
    st0 = arm.state()
    tcp0 = np.array(st0["tcp"]["pos"], dtype=float)
    outside = box_violations(tcp0)
    if outside:
        print(f"当前 TCP 不在工作空间盒内（{'，'.join(outside)}）；先手动摆位再遥操作", file=sys.stderr)
        arm.close()
        return 2

    loop = TeleopLoop(arm, yaw_deg=args.yaw_deg if args.yaw_deg is not None else 0.0,
                      max_vel_mps=args.max_vel, max_rot_vel_dps=args.max_rot_vel,
                      fake_grasp_at=args.fake_grasp_at,
                      griplock=GripLock(dt_s=DT_S, close_speed_dps=args.grip_vel))

    haptics = None
    if args.haptics_port:
        try:
            haptics = HapticsServer(port=args.haptics_port, https_ip=args.haptics_ip,
                                    out_dir=(args.log.parent if args.log is not None
                                             else REPO_ROOT / "tests/runs" /
                                             (time.strftime("%Y-%m-%d") + "-遥操作")))
            haptics.start()
        except OSError as e:
            print(f"振动页面起不来（{e}），继续遥操作但无振动反馈", flush=True)
            haptics = None

    mapper = mods["RelativePoseMapper"](side=args.side, grip_press_threshold=GRIP_PRESS,
                                        grip_release_threshold=GRIP_RELEASE,
                                        stale_timeout_s=MAPPER_STALE_S,
                                        orientation_filter_hz=args.rot_filter_hz)
    probe = PicoProbe(mapper, tcp0, np.eye(3))   # ee 参考每帧换实测（step_probe）
    config = mods["WebXRConfig"](hand_side=args.side, stale_timeout=CTRL_STALE_S,
                                 endpoint=args.endpoint)
    controller = probe.attach(mods["WebXRController"](config, on_tracking=probe.on_tracking))

    log_path = args.log if args.log is not None else (
        REPO_ROOT / "tests/runs" / (time.strftime("%Y-%m-%d") + "-遥操作") /
        time.strftime("teleop-%H%M%S.jsonl"))
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = log_path.open("w", encoding="utf-8")   # 覆盖写：一个文件一次运行，不混 run

    exit_code = 0
    goto = None
    controller.connect()
    try:
        if args.calibrate:
            loop.set_yaw_deg(calibrate_yaw(arm, probe, rotation_cls))
        elif args.yaw_deg is not None:
            print(f"朝向修正 yaw {args.yaw_deg:+.1f}°（--yaw-deg）", flush=True)
        else:
            print("朝向修正 yaw +0.0°（默认；--yaw-deg / --calibrate 可改）", flush=True)
        print(f"Ctrl-C 退出（臂 HOLD 在当前位置）。日志 → {log_path}", flush=True)
        print(f"松开握把时：按住 A 0.5 s = 回准备位，按住 B 0.5 s = 回折叠位（夹爪须张开）；"
              f"走的过程中按握把或再按 A/B = 停。夹爪闭合限速 {args.grip_vel:g}°/s", flush=True)

        def pulse(pattern):
            if haptics is not None:
                haptics.pulse(args.side, pattern)

        buttons = ButtonHold()
        goto = None             # 进行中的一键回位（GotoMotion）
        need_release = False    # 中止/拒绝后须先松握把再接合
        prev_ab = (False, False)
        t0 = time.monotonic()
        report_at = 0.0
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            arm.wait_frame()
            status, _ = step_probe(probe, arm, rotation_cls)
            arm_state = arm.state()
            st = state_str(status["state"])
            live = bool(status["live"])
            now = time.monotonic()
            a, b = read_ab(probe, live)
            fire = buttons.update(now, a, b)
            ab_edge = (a and not prev_ab[0]) or (b and not prev_ab[1])
            prev_ab = (a, b)

            # ---- 一键回位进行中：只走轨迹，不跑遥操作 ----
            if goto is not None:
                if arm_state["ctrl_state"] in (CTRL_ESTOP, CTRL_FAULT):
                    raise D1ArmError("控制器进入 ESTOP/FAULT，一键回位中止")
                if st == "active" or ab_edge or not live:
                    arm.stream_stop()
                    print(f"{GOTO_LABEL[goto.kind]}已中止（{'握把' if st == 'active' else 'A/B' if ab_edge else '断流'}），"
                          "臂停在当前位置", flush=True)
                    goto, need_release = None, True
                elif goto.step():
                    arm.stream_stop()
                    print(f"{GOTO_LABEL[goto.kind]}到位", flush=True)
                    pulse("ack")
                    goto = None
                log_f.write(json.dumps({"t": round(now - t0, 3), "wall_ms": int(time.time() * 1000),
                                        "seq": arm_state["seq"], "mode": arm_state["mode"],
                                        "probe": st, "live": live,
                                        "goto": None if goto is None else goto.kind,
                                        "goto_i": None if goto is None else goto.i,
                                        "tcp_meas": [round(v, 6) for v in arm_state["tcp"]["pos"]]},
                                       ensure_ascii=False) + "\n")
                if now >= report_at and goto is not None:
                    print(f"[{time.strftime('%H:%M:%S')}] {GOTO_LABEL[goto.kind]} "
                          f"{goto.i * DT_S:4.1f}/{goto.duration_s:4.1f} s", flush=True)
                    report_at = now + 1.0
                log_f.flush()
                continue

            # ---- A/B 触发（只在脱开时）----
            if fire is not None and st == "idle":
                if fire == "fold" and arm_state["gripper_mm"] < FOLD_GRIP_OPEN_MIN_MM:
                    print(f"回折叠位拒绝：夹爪未张开（{arm_state['gripper_mm']:.1f} mm < "
                          f"{FOLD_GRIP_OPEN_MIN_MM:g} mm）；确认没夹东西、松开扳机后再按 B", flush=True)
                    pulse("empty")
                else:
                    loop.detach()
                    arm.stream_start()
                    goto = GotoMotion(arm, fire, vel_dps=args.goto_vel)
                    print(f"{GOTO_LABEL[fire]}：约 {goto.duration_s:.1f} s", flush=True)
                    pulse("ack")
                    continue

            # ---- 中止/拒绝后：等松握把 ----
            if need_release:
                if st == "active":
                    continue
                need_release = False
            # ---- 折叠位或盒外不许接合（先按 A）----
            if st == "active" and not loop.streaming:
                outside = box_violations(arm_state["tcp"]["pos"])
                if outside or near_fold(arm_state["q_deg"]):
                    why = (f"TCP 不在工作空间盒内（{'，'.join(outside)}）" if outside
                           else "臂在折叠位")
                    print(f"{why}，不接合；松开握把，按住 A 回准备位", flush=True)
                    pulse("empty")
                    need_release = True
                    continue

            out = loop.step(status, arm_state)
            if out["event"] in ("grasp", "empty") and haptics is not None:
                haptics.pulse(args.side, out["event"])
            log_f.write(json.dumps({
                "t": round(time.monotonic() - t0, 3),
                "wall_ms": int(time.time() * 1000),
                "seq": arm_state["seq"], "ctrl_state": arm_state["ctrl_state"],
                "mode": arm_state["mode"], "probe": out["probe_state"],
                "live": bool(status["live"]), "rx_hz": round(status["rx_hz"], 1),
                "sent": out["sent"], "streaming": out["streaming"],
                "pos": None if out["pos"] is None else [round(v, 6) for v in out["pos"]],
                "quat": None if out["quat"] is None else [round(v, 6) for v in out["quat"]],
                "tcp_meas": [round(v, 6) for v in arm_state["tcp"]["pos"]],
                "j6_cmd": round(out["j6_cmd"], 3), "j6_meas_raw": round(out["j6_meas_raw"], 3),
                "j6_meas": round(out["j6_meas"], 3), "j6_out": round(out["j6_out"], 3),
                "locked": out["locked"], "lock_deg": out["lock_deg"], "event": out["event"],
                "step_clamped": out["step_clamped"], "box_clamped": out["box_clamped"],
                "rot_clamped": out["rot_clamped"],
                "ik_fail": out["ik_fail"], "trigger": round(float(status["trigger"]), 3),
            }, ensure_ascii=False) + "\n")
            now = time.monotonic()
            if now >= report_at:
                print(format_line(out, status["rx_hz"], bool(status["live"])), flush=True)
                report_at = now + 1.0
            log_f.flush()
    except KeyboardInterrupt:
        print("\nCtrl-C：退出中…", flush=True)
    except (D1ArmError, TimeoutError) as e:
        print(f"停止：{e}", flush=True)
        exit_code = 1
    finally:
        try:
            if loop.streaming or goto is not None:
                st = arm.state()
                if st["ctrl_state"] not in (CTRL_ESTOP, CTRL_FAULT):
                    arm.stream_stop()
                    print("已 stream_stop（臂 HOLD 在当前姿态）", flush=True)
                else:
                    print("控制器 ESTOP/FAULT：不再发 stream_stop（看门狗已停臂；恢复先 reset）",
                          flush=True)
        except D1ArmError as e:
            print(f"收尾 stream_stop 失败（看门狗约 0.3 s 也会停臂）：{e}", flush=True)
        controller.disconnect()
        arm.close()
        if haptics is not None:
            haptics.stop()
        log_f.close()
        print("已停止。", flush=True)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
