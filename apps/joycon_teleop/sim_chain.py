"""假手柄全链路仿真：接管 → 流式（抬升 + 前进 + 手腕转动）→ 夹爪合拢 → 张开 → 松开停臂。

对 `d1t_arm_ctrl --sim`（本机 lo）跑，验证 joycon_teleop 迁移后的完整指令链。
判定：全程控制器 fault=0；结束后回到 HOLD（或 IDLE），夹爪回到张开开度。

用法（先在终端 1 起 --sim 控制器，见 deploy/虚拟部署.md；另开终端 2）：

    source deploy/env.sh sim
    python3 apps/joycon_teleop/sim_chain.py

退出码 0 = 通过；期间打印 'Holding: …' 属于限幅保持提示，不是失败。
注意 run() 每个循环会读两次手柄（sample + IK 后复读），帧段消耗约快一倍，总时长约 3–4 s。
"""
import sys
from types import SimpleNamespace

import numpy as np

from joycon_teleop import D1Arm, D1ArmError, Input, RobotFault, from_rpy, run
from d1t_sdk._common import CONTROL_HZ


class FakeHandset:
    """按帧脚本化的假手柄（每帧 = 控制器 50 ms 周期）。"""

    def __init__(self):
        self.frame = 0
        self.wrist0 = from_rpy(np.deg2rad([0., 0., 0.]))       # 接管时的握持姿态
        self.wrist = from_rpy(np.deg2rad([12., -8., 15.]))     # 动作段的手腕转动

    def read(self):
        k, self.frame = self.frame, self.frame + 1
        if k == 0:    # 先观察到松开，才允许接管（run() 的安全逻辑）
            return Input(self.wrist0, False, 0, 0, 0, True, False)
        if k < 4:     # 接管，原地保持
            return Input(self.wrist0, True, 0, 0, 0, True, False)
        if k < 44:    # 纯抬升（平移段 IK 连续，实测全帧推进）
            return Input(self.wrist0, True, 0, 0, 1, True, False)
        if k < 60:    # 纯前进
            return Input(self.wrist0, True, 1, 0, 0, True, False)
        if k < 68:    # 手腕转动（持续转动会触发限幅保持，属算法设计行为，见 README 已知限制）
            return Input(self.wrist, True, 0, 0, 0, True, False)
        if k < 84:    # 扳机切换（合拢）
            return Input(self.wrist, True, 0, 0, 0, False, False)
        if k < 104:   # 扳机再切换（张开）
            return Input(self.wrist, True, 0, 0, 0, True, False)
        if k < 122:   # 张开到位
            return Input(self.wrist, True, 0, 0, 0, True, False)
        if k < 124:   # 松开使能 → stream_stop → HOLD
            return Input(self.wrist, False, 0, 0, 0, True, False)
        return Input(self.wrist, True, 0, 0, 0, True, True)   # 退出


def main():
    args = SimpleNamespace(dry_run=False, speed_mps=0.04, angular_dps=20., joint_dps=25.,
                           j5_margin_deg=5., open_mm=55., close_mm=10., axes_deg=[0., 0., 0.],
                           period_s=1.0 / CONTROL_HZ)
    arm = D1Arm()
    handset = FakeHandset()
    try:
        # --sim 假舵机从折叠姿态起步，先规划走到中性位，给手腕/抬升留出运动空间。
        q0 = arm.state()["q_deg"]
        arm.move_joints([0., -30., 40., 0., 20., 0.], vel_dps=40)
        arm.wait_idle()
        print(f"起始 q_deg={q0} → 中性位 {[round(q, 1) for q in arm.state()['q_deg']]}")
        run(handset, arm, args)
        s = arm.state()
        checks = [
            ('控制器 fault=0', s['fault'] == 0),
            ('松开后回 HOLD/IDLE', s['mode'] in ('HOLD', 'IDLE')),
            ('夹爪回到张开 ≥ 45 mm', s['gripper_mm'] >= 45.),
            ('遥操作段有实际运动（q 相对中性位变化 >2°）',
             max(abs(q - t) for q, t in zip(s['q_deg'], [0., -30., 40., 0., 20., 0.])) > 2.),
        ]
        print(f"结束状态：mode={s['mode']} fault={s['fault']} gripper={s['gripper_mm']:.1f} mm "
              f"q_deg={[round(q, 1) for q in s['q_deg']]}")
    except (RobotFault, D1ArmError) as exc:
        checks = [(f'全链路异常中断：{exc}', False)]
    finally:
        arm.close()
    ok = True
    for name, passed in checks:
        print(f"[{'PASS' if passed else 'FAIL'}] {name}")
        ok = ok and passed
    print('结果：' + ('全部通过' if ok else '存在失败项'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
