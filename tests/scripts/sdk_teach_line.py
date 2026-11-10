#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""示教两点 + move_l 直线往返：直线长短由人决定，每一轮前可改速度。

流程：失力 → 手托着摆到 A → 回车锁住记 A → 失力 → 摆到 B → 回车锁住记 B
      → 关节运动回 A → 每轮先输入速度（m/s），move_l 走 A→B→A（姿态按两点 slerp）。
每次锁住打印「锁住前后关节角差」（看跳不跳）。
⚠ 失力 = 舵机完全不出力，臂会掉：回车前手先托住臂。夹爪 J6 也失力。
   示教点避开手腕水平伸出（J4≈90° 保持会抖、HOLD_FAIL）。
用法（上位机，已设 CYCLONEDDS_URI）：python3 tests/scripts/sdk_teach_line.py
"""
import math
import select
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sdk"))

from d1t_sdk import D1Arm, D1ArmError  # noqa: E402

VEL_MIN, VEL_MAX = 0.01, 1.00  # m/s 请求平均速度；超过关节限速（臂模型 axis_speed_dps）时 SDK 自动拉长时间


def r1(q):
    return [round(x, 1) for x in q]


def mm(p):
    return [round(v * 1000) for v in p]


def teach_point(arm, name):
    """失力 → 手托摆位 → 回车锁住；返回锁住后的 state。"""
    input(f"\n[{name}] 手先托住臂，回车失力 ...")
    arm.free_all()
    print("   已失力。摆到位后回车锁住：")
    while True:
        print("   q =", r1(arm.state()["q_deg"]))
        if select.select([sys.stdin], [], [], 1.0)[0]:
            sys.stdin.readline()
            break
    q_before = arm.state()["q_deg"]
    arm.lock_all()
    time.sleep(0.5)
    st = arm.state()
    jump = [round(a - b, 1) for a, b in zip(st["q_deg"], q_before)]
    print(f"   锁住 {st['mode']}  q = {r1(st['q_deg'])}  TCP = {mm(st['tcp']['pos'])} mm")
    print(f"   锁住前后差 = {jump} °（|差| ≤ 0.5 视为不跳）")
    return st


def ask_vel(last):
    """输入本轮速度；回车沿用上一轮，q 结束。返回 None 表示结束。"""
    while True:
        s = input(f"\n本轮速度 m/s（{VEL_MIN}–{VEL_MAX}，回车 = {last}，q = 结束）：").strip().lower()
        if s == "q":
            return None
        if s == "":
            return last
        try:
            v = float(s)
        except ValueError:
            print("   不是数字，重输")
            continue
        if VEL_MIN <= v <= VEL_MAX:
            return v
        print(f"   超出 {VEL_MIN}–{VEL_MAX}，重输")


def run_round(arm, a, b, vel):
    """走一轮 A→B→A。"""
    for name, tgt in (("A→B", b), ("B→A", a)):
        r = arm.move_l(tgt["tcp"]["pos"], quat=tgt["tcp"]["quat"], vel=vel)
        got = arm.state()["tcp"]["pos"]
        err = [round((g - t) * 1000) for g, t in zip(got, tgt["tcp"]["pos"])]
        d = math.dist(a["tcp"]["pos"], b["tcp"]["pos"])
        v_avg = d / r["duration_s"]
        note = "（被关节限速拉长）" if v_avg < vel * 0.95 else ""
        print(f"   {name}：用时 {r['duration_s']:.2f}s  实际平均 {v_avg:.3f} m/s{note}  "
              f"实到 {mm(got)} mm  误差 {err} mm")


def main():
    arm = D1Arm()
    try:
        a = teach_point(arm, "A 点")
        b = teach_point(arm, "B 点")
        d = math.dist(a["tcp"]["pos"], b["tcp"]["pos"])
        print(f"\nA→B 直线长 {d * 1000:.0f} mm")
        if d < 0.01:
            print("两点不到 1 cm，不执行")
            return 1
        print("回 A（关节运动 20°/s）...")
        arm.move_joints(a["q_deg"], vel_dps=20)
        vel, n = 0.05, 0
        while True:
            vel = ask_vel(vel)
            if vel is None:
                break
            n += 1
            print(f"第 {n} 轮，{vel} m/s：")
            run_round(arm, a, b, vel)
        print("\n完成（HOLD 在 A）")
    except D1ArmError as e:
        print(f"NG：{e}")
        return 1
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
