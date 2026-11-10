"""用单个 Joy-Con 控制 D1 末端的位置、朝向和夹爪开度。按键映射见同目录 README.md。

迁移自 d1arm_sdk（gitee gccodespace/d1arm_sdk，joycon 分支 f77d370，原作者 bxdw）：
驱动臂的调用从 d1armd（TCP 7777 JSON RPC）换成 d1t_sdk.D1Arm（DDS 直连 d1t_arm_ctrl），
手柄映射与姿态算法不动；接口差异（状态键名、看门狗删除、wait_frame 帧同步节拍、
流式中 J6 夹爪守卫）见同目录 README.md 接口对照表。
"""
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import numpy as np

# d1t_sdk 在仓库 sdk/ 下（部署时由 deploy/env.sh 设 PYTHONPATH），这里自行补路径。
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'sdk'))
from d1t_sdk import D1Arm, D1ArmError
from d1t_sdk._common import CONTROL_HZ
from d1t_sdk._joint_model import gripper_mm_to_deg, urdf_limits


class RobotFault(RuntimeError):
    """遥操作会话判定臂侧状态异常（原 calibration.utils.robot.RobotFault 的本地等价物）。"""


# 夹爪守卫（README 接口表·gripper 行）：流式中 J6 目标角的到位 / 停滞 / 硬超时判据。
GRIP_ARRIVE_MM = 1.5    # 实测开度与目标差 ≤ 此值即到位
GRIP_PROBE_MM = 0.5     # 实测开度变化 ≥ 此值视为仍有进展
GRIP_STALL_S = 0.6      # 这么久没有进展判停滞（通常是夹到物体）
GRIP_SPEED_DPS = 100.0  # 控制器流式 J6 轴限速（arm_model/d1t.json axis_speed_dps[6]）
GRIP_MARGIN_S = 1.0     # 按行程估的硬超时之外的余量


def vector(values, size):
    out = np.asarray(values, dtype=float)
    if out.shape != (size,) or not np.isfinite(out).all():
        raise ValueError(f'Expected {size} finite numbers')
    return out


def unit_quat(values):
    q = vector(values, 4)
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError('Invalid quaternion')
    return q / norm


def product(a, b):
    """按 D1 通信协议的 [w, x, y, z] 顺序计算四元数的哈密顿积。"""
    return np.r_[a[0]*b[0] - a[1:] @ b[1:],
                 a[0]*b[1:] + b[0]*a[1:] + np.cross(a[1:], b[1:])]


def inverse(q):
    # 调用方传入单位四元数，取共轭即可得到逆旋转。
    return q * [1, -1, -1, -1]


def from_rpy(rpy):
    """将以弧度表示的 roll、pitch、yaw 按 Rz(yaw)·Ry(pitch)·Rx(roll) 合成。"""
    r, p, y = vector(rpy, 3) / 2
    return product(product(np.array([math.cos(y), 0, 0, math.sin(y)]),
                           np.array([math.cos(p), 0, math.sin(p), 0])),
                   np.array([math.cos(r), math.sin(r), 0, 0]))


def toward(a, b, max_angle):
    """从姿态 a 沿最短旋转方向接近 b，单次转角上限为 max_angle（弧度）。"""
    delta = unit_quat(product(inverse(a), b))
    if delta[0] < 0:
        delta = -delta
    length = np.linalg.norm(delta[1:])
    if length < 1e-10:
        return a.copy()
    half = min(math.atan2(length, delta[0]), max_angle / 2)
    return unit_quat(product(a, np.r_[math.cos(half), math.sin(half)*delta[1:]/length]))


def stick(value):
    # 沿用 joycon-robotics 的摇杆阈值，将读数映射为后退、保持或前进。
    return int(value > 4000) - int(value < 1000)


@dataclass
class Input:
    """一次采样的控制输入。forward、lateral 沿工具 X、Y 轴，lift 沿基座 Z 轴。"""
    wrist: np.ndarray
    enabled: bool
    forward: int
    lateral: int
    lift: int
    grip_open: bool
    quit: bool


class Handset:
    def __init__(self, side):
        from joyconrobotics import JoyconRobotics
        self.right = side == 'right'
        self.device = JoyconRobotics(device=side, common_rad=False, lerobot=False,
                                     horizontal_stick_mode='y', pure_dx=False,
                                     gripper_open=1., gripper_close=0.)
        self.raw = self.device.joycon
        self.last_report = None
        self.raw.register_update_hook(self.report)
        deadline = time.monotonic() + 2
        while self.last_report is None and time.monotonic() < deadline:
            time.sleep(0.01)

    def report(self, device):
        self.last_report = time.monotonic()

    def read(self):
        # 蓝牙断连后 get_control() 仍可能返回缓存，用最近一份 HID 报告判断输入是否有效。
        if self.last_report is None or time.monotonic() - self.last_report > 0.25:
            raise RuntimeError('Joy-Con HID reports timed out')
        pose, gripper, _ = self.device.get_control()
        raw = self.raw
        if self.right:
            h, v = raw.get_stick_right_horizontal(), raw.get_stick_right_vertical()
            enable, quit_button = raw.get_button_a(), raw.get_button_y()
            up, down = raw.get_button_r(), raw.get_button_r_stick()
        else:
            h, v = raw.get_stick_left_horizontal(), raw.get_stick_left_vertical()
            enable, quit_button = raw.get_button_right(), raw.get_button_left()
            up, down = raw.get_button_l(), raw.get_button_l_stick()
        # pose 后三项是手腕 RPY。平移由摇杆独立控制，横向高值对应工具 −Y。
        return Input(from_rpy(vector(pose, 6)[3:]), bool(enable), stick(v), -stick(h),
                     int(up)-int(down), bool(gripper), bool(quit_button))

    def close(self):
        self.device.running = False
        self.device.thread.join(timeout=0.25)
        self.device.disconnnect()  # 此处保留 joycon-robotics 接口名的原始拼写。


class Session:
    def __init__(self, arm, args):
        self.arm = arm
        self.args = args
        self.active = False
        self.streaming = False
        self.last_warning = -math.inf
        self.seq = None
        self.axes = from_rpy(np.deg2rad(args.axes_deg))
        # 原脚本硬编码的对称限位，与本项目实测限位（d1t.json）逐轴取较小者
        # （实际仅 J2 高侧 89° 比原 90° 紧），见 README 接口表·补充核对。
        model = np.array([min(abs(lo), hi) for lo, hi in urdf_limits()])
        self.limits = np.minimum(np.array([135., 90., 90., 135., 90., 135.]), model)
        # 夹爪守卫状态：grip_target_deg 为 None 表示 J6 保持上次命令角。
        self.grip_target_deg = None
        self.grip_target_mm = 0.
        self.grip_progress_mm = 0.
        self.grip_progress_t = 0.
        self.grip_deadline = math.inf

    def state(self, expected):
        if isinstance(expected, str):
            expected = (expected,)
        s = self.arm.state()
        # fault（d1armd 的 err）非 0、舵机离线或状态不符都结束控制；warn 非致命不拦。
        if s['mode'] not in expected or s.get('fault') or list(s['online']) != [True]*7:
            raise RobotFault(f"Unexpected feedback: mode={s['mode']} fault={s.get('fault')} online={s['online']}")
        seq = s.get('seq')
        if not isinstance(seq, int) or seq < 0:
            raise RobotFault('Invalid robot feedback sequence')
        if self.seq != seq:
            self.seq, self.updated = seq, time.monotonic()
        elif time.monotonic() - self.updated > 0.5:
            raise RobotFault('Robot feedback stalled')
        vector(s['q_deg'], 6)
        return s

    def engage(self, sample):
        """用实测关节角计算起始 TCP，同时记录本次接管时的手腕朝向。"""
        # 控制器上次 stream_stop 后停在 HOLD，仅首次启动是 IDLE（接口表·state 行）。
        state = self.state(('IDLE', 'HOLD'))
        self.q = vector(state['q_deg'], 6)
        if np.any(np.abs(self.q) > self.limits):
            raise RobotFault('Measured starting pose outside calibrated D1 limits')
        self.jaws_open = state['gripper_mm'] >= (self.args.open_mm + self.args.close_mm)/2
        pose = self.arm.fk(self.q.tolist())
        self.pos = vector(pose['pos'], 3)
        self.rot = self.start_rot = unit_quat(pose['quat'])
        self.start_wrist = sample.wrist.copy()
        self.grip = sample.grip_open
        self.grip_target_deg = None
        if not self.args.dry_run:
            # 首帧发实测角，接管时原位保持。断流后由控制器内置看门狗约 0.3 s 停臂
            # （原 d1armd 的 watchdog RPC 已删，见接口表）。
            self.arm.stream_start()
            self.streaming = True
            self.arm.stream_joints(q_deg=self.q.tolist())
            deadline = time.monotonic() + 2.0
            while True:
                s = self.arm.state()
                if s['mode'] == 'STREAMING':
                    break
                if s.get('fault') or time.monotonic() > deadline:
                    raise RobotFault(f"Controller did not enter STREAMING: "
                                     f"mode={s['mode']} fault={s.get('fault')}")
                self.arm.wait_frame()
        self.active = True
        print('Engaged at measured TCP', self.pos.round(4), flush=True)

    def release(self):
        self.active = False
        if self.streaming:
            self.streaming = False
            try:
                self.arm.stream_stop()
            except D1ArmError as exc:
                # 已在退出路径上：控制器故障时 stream_stop 可能超时，不掩盖原始异常。
                print('stream_stop failed:', exc, flush=True)

    def start_grip(self, state, now=None):
        """扳机切换：以实测开度起步记进展，按控制器 J6 轴限速估硬超时。"""
        if now is None:
            now = time.monotonic()
        self.grip_target_mm = self.args.open_mm if self.jaws_open else self.args.close_mm
        self.grip_target_deg = gripper_mm_to_deg(self.grip_target_mm)
        self.grip_progress_mm = state['gripper_mm']
        self.grip_progress_t = now
        travel = abs(self.grip_target_deg - gripper_mm_to_deg(state['gripper_mm']))
        self.grip_deadline = now + travel / GRIP_SPEED_DPS + GRIP_MARGIN_S

    def grip_j6_target(self, state, now=None):
        """本帧 J6 目标角（舵机角，度）：无动作 None（保持上次命令角）；动作中发目标角，
        到位或停滞后冻结在当前实测开度，不让 J6 长时间向堵转方向压（接口表·gripper 行）。"""
        if self.grip_target_deg is None:
            return None
        if now is None:
            now = time.monotonic()
        mm = state['gripper_mm']
        if abs(mm - self.grip_target_mm) <= GRIP_ARRIVE_MM:
            self.grip_target_deg = None
            return None
        if abs(mm - self.grip_progress_mm) >= GRIP_PROBE_MM:
            self.grip_progress_mm, self.grip_progress_t = mm, now
        elif now - self.grip_progress_t >= GRIP_STALL_S or now >= self.grip_deadline:
            print(f'Gripper stalled at {mm:.1f} mm; freezing J6 command', flush=True)
            self.grip_target_deg = None
            return gripper_mm_to_deg(mm)
        return self.grip_target_deg

    def propose(self, sample, dt):
        # delta 是相对接管瞬间的手腕转动。用 C·delta·C⁻¹ 将它换算到工具轴系，
        # 再右乘到接管时的 TCP 姿态上，因此重新握持手柄后可以从当前末端姿态继续。
        delta = product(inverse(self.start_wrist), sample.wrist)
        desired = product(self.start_rot, product(product(self.axes, delta), inverse(self.axes)))
        rot = toward(self.rot, desired, math.radians(self.args.angular_dps)*dt)
        local = np.array([0., sample.forward, sample.lateral, 0.])
        # 用纯四元数 [0, vx, vy, 0] 表示工具系平移方向，旋转到基座系后叠加升降。
        velocity = product(product(rot, local), inverse(rot))[1:]
        velocity[2] += sample.lift
        velocity /= max(1., np.linalg.norm(velocity))
        step = velocity * self.args.speed_mps * dt
        return self.solve_step(step, rot, dt)

    def solve_step(self, step, rot, dt):
        """对一个已限速的 TCP 步长做 IK、关节限位和关节速度检查。"""
        if np.linalg.norm(step) < 1e-10 and abs(self.rot @ rot) > 1-1e-12:
            return self.pos, self.rot, self.q
        # 始终用上次接受的关节目标作 IK 初值。依次尝试整步、半步、四分之一步，
        # 只接受满足限位和单周期关节速度约束的解；全部失败则保持上次目标。
        for scale in (1., 0.5, 0.25):
            pos = self.pos + scale*step
            quat = toward(self.rot, rot, scale*math.radians(self.args.angular_dps)*dt)
            try:
                # IK 在上位机本地算（无 RPC）；无解抛 D1ArmError（'IK 无解（…）'）。
                q = vector(self.arm.ik(pos=pos.tolist(), quat=quat.tolist(),
                                       seed=self.q.tolist()), 6)
            except (RobotFault, D1ArmError) as exc:
                if str(exc).startswith('IK 无解'):
                    continue
                raise
            limits = self.limits.copy()
            # 若接管时 J5 已进入预留区，允许向内退回，同时限制 |J5| 继续增大。
            limits[4] = min(self.limits[4], max(self.limits[4]-self.args.j5_margin_deg,
                                                abs(self.q[4])))
            if np.all(np.abs(q) <= limits) and np.max(np.abs(q-self.q)) <= self.args.joint_dps*dt:
                return pos, quat, q
        if time.monotonic() - self.last_warning > 1:
            print('Holding: IK, joint limit or joint speed bound', flush=True)
            self.last_warning = time.monotonic()
        return self.pos, self.rot, self.q

    def tick(self, sample, dt, read_again):
        state = self.state('STREAMING' if self.streaming else ('IDLE', 'HOLD'))
        measured = vector(state['q_deg'], 6)
        if self.streaming and np.max(np.abs(measured-self.q)) > 18:
            raise RobotFault('Joint feedback is too far behind the command')
        pos, rot, q = self.propose(sample, dt)
        # IK 期间操作者可能松开使能键，下发关节指令前再读一次手柄。
        latest = read_again()
        if not latest.enabled or latest.quit:
            self.release()
            return not latest.quit
        if self.streaming:
            self.arm.stream_joints(q_deg=q.tolist(), j6_deg=self.grip_j6_target(state))
            if latest.grip_open != self.grip:
                # 只在扳机状态切换时改目标；实际开合方向以上次夹爪状态为准。
                # 流式中不另发 gripper()：J6 目标角随 stream_joints 逐帧下发（接口表）。
                self.grip = latest.grip_open
                self.jaws_open = not self.jaws_open
                self.start_grip(state)
        elif time.monotonic() - getattr(self, 'last_print', -math.inf) >= 0.5:
            print('Preview pos(m)', pos.round(4), 'q(deg)', q.round(1), flush=True)
            self.last_print = time.monotonic()
        self.pos, self.rot, self.q, self.grip = pos, rot, q, latest.grip_open
        return True


def run(handset, arm, args, session_type=Session):
    session = session_type(arm, args)
    released = False
    last = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            # 循环延迟时最多推进一个标称周期，避免补算积压的位移。
            dt = min(now-last, args.period_s)
            last = now
            sample = handset.read()
            if sample.quit:
                break
            if not sample.enabled:
                # 先观察到松开，再允许接管，避免程序启动时按住的键直接触发运动。
                session.release()
                released = True
            elif released and not session.active:
                session.engage(sample)
                released = False
            elif session.active and not session.tick(sample, dt, handset.read):
                break
            # 跟控制器帧走（20 Hz），不写 sleep：本机时钟与臂节拍漂移会漏帧或一帧跳两步。
            arm.wait_frame()
    finally:
        session.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=['left', 'right'], default='right')
    parser.add_argument('--dry-run', action='store_true',
                        help='只读状态 + 本地 FK/IK 预演目标，不发流式')
    parser.add_argument('--speed-mps', type=float, default=0.04)
    parser.add_argument('--angular-dps', type=float, default=20.)
    parser.add_argument('--joint-dps', type=float, default=25.)
    parser.add_argument('--j5-margin-deg', type=float, default=5.)
    parser.add_argument('--open-mm', type=float, default=55.)
    parser.add_argument('--close-mm', type=float, default=10.)
    parser.add_argument('--axes-deg', type=float, nargs=3, default=[0., 0., 0.],
                        metavar=('ROLL', 'PITCH', 'YAW'),
                        help='Joy-Con local axes to TCP local axes')
    args = parser.parse_args()
    args.period_s = 1.0 / CONTROL_HZ  # 控制器固定 20 Hz（接口表·stream 行）
    if not (0 < args.speed_mps <= 0.15
            and 0 < args.angular_dps <= 90 and 0 < args.joint_dps <= 100
            and 0 <= args.j5_margin_deg < 90 and 0 <= args.close_mm < args.open_mm <= 65
            and np.isfinite(args.axes_deg).all()):
        parser.error('Invalid speed, axes, margin or gripper range; see README.md')
    handset = Handset(args.side)
    try:
        arm = D1Arm()  # DDS 环境由 deploy/env.sh 提供（CYCLONEDDS_URI + ParticipantIndex）
        try:
            print('Release then hold A (left: Right arrow) to control. Y (left: Left arrow) exits.')
            run(handset, arm, args)
        finally:
            arm.close()
    finally:
        handset.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
    except (RuntimeError, OSError, ValueError) as exc:
        sys.exit(str(exc))
