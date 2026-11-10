"""用单个 Joy-Con 控制 D1 末端的位置、朝向和夹爪开度。按键映射见同目录 README.md。"""
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import numpy as np

# 从仓库根目录导入已有的 d1armd 客户端，复用其顺序请求和超时处理。
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from calibration.utils.robot import RobotFault, RpcArm


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
        self.stamp = None
        self.axes = from_rpy(np.deg2rad(args.axes_deg))

    def state(self, expected):
        s = self.arm.call('state')['state']
        if s['mode'] != expected or s.get('err') or list(s['online']) != [True]*7:
            raise RobotFault(f"Unexpected feedback: mode={s['mode']} online={s['online']} err={s.get('err')}")
        stamp = s['stamp_ms']
        if not math.isfinite(stamp) or stamp <= 0:
            raise RobotFault('Invalid robot timestamp')
        if self.stamp != stamp:
            self.stamp, self.updated = stamp, time.monotonic()
        elif time.monotonic() - self.updated > 0.5:
            raise RobotFault('Robot feedback stalled')
        vector(s['q_deg'], 6)
        return s

    def engage(self, sample):
        """用实测关节角计算起始 TCP，同时记录本次接管时的手腕朝向。"""
        state = self.state('IDLE')
        self.q = vector(state['q_deg'], 6)
        if np.any(np.abs(self.q) > [135.,90.,90.,135.,90.,135.]):
            raise RobotFault('Measured starting pose outside calibrated D1 limits')
        self.jaws_open = state['gripper_mm'] >= (self.args.open_mm + self.args.close_mm)/2
        pose = self.arm.call('fk', q=self.q.tolist())['pose']
        self.pos = vector(pose['pos'], 3)
        self.rot = self.start_rot = unit_quat(pose['quat'])
        self.start_wrist = sample.wrist.copy()
        self.grip = sample.grip_open
        if not self.args.dry_run:
            # 首帧发实测角，接管时原位保持。后续流式指令中断超过 500 ms 时由服务端保持。
            self.arm.call('watchdog', ms=500)
            self.arm.call('stream_start')
            self.streaming = True
            self.arm.call('stream_joints', q=self.q.tolist())
        self.active = True
        print('Engaged at measured TCP', self.pos.round(4), flush=True)

    def release(self):
        self.active = False
        if self.streaming:
            self.streaming = False
            self.arm.call('stream_stop')

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
                reply = self.arm.call('ik', pose=dict(pos=pos.tolist(), quat=quat.tolist()),
                                      seed=self.q.tolist())
            except RobotFault as exc:
                if str(exc).startswith('ik no solution'):
                    continue
                raise
            q = vector(reply['q_deg'], 6)
            limits = np.array([135.,90.,90.,135.,90.,135.])
            # 若接管时 J5 已进入预留区，允许向内退回，同时限制 |J5| 继续增大。
            limits[4] = min(90., max(90.-self.args.j5_margin_deg, abs(self.q[4])))
            if np.all(np.abs(q) <= limits) and np.max(np.abs(q-self.q)) <= self.args.joint_dps*dt:
                return pos, quat, q
        if time.monotonic() - self.last_warning > 1:
            print('Holding: IK, joint limit or joint speed bound', flush=True)
            self.last_warning = time.monotonic()
        return self.pos, self.rot, self.q

    def tick(self, sample, dt, read_again):
        measured = vector(self.state('STREAMING' if self.streaming else 'IDLE')['q_deg'], 6)
        if self.streaming and np.max(np.abs(measured-self.q)) > 18:
            raise RobotFault('Joint feedback is too far behind the command')
        pos, rot, q = self.propose(sample, dt)
        # IK 请求期间操作者可能松开使能键，下发关节指令前再读一次手柄。
        latest = read_again()
        if not latest.enabled or latest.quit:
            self.release()
            return not latest.quit
        if self.streaming:
            self.arm.call('stream_joints', q=q.tolist())
            if latest.grip_open != self.grip:
                # 只在扳机状态切换时发一次命令；实际开合方向以上次夹爪状态为准。
                self.jaws_open = not self.jaws_open
                self.arm.call('gripper', mm=self.args.open_mm if self.jaws_open else self.args.close_mm)
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
            dt = min(now-last, 1/args.hz)
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
            time.sleep(max(0., 1/args.hz - (time.monotonic()-now)))
    finally:
        session.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7777)
    parser.add_argument('--side', choices=['left','right'], default='right')
    parser.add_argument('--dry-run', action='store_true', help='Use only state/FK/IK RPCs')
    parser.add_argument('--hz', type=float, default=20.)
    parser.add_argument('--speed-mps', type=float, default=0.04)
    parser.add_argument('--angular-dps', type=float, default=20.)
    parser.add_argument('--joint-dps', type=float, default=25.)
    parser.add_argument('--j5-margin-deg', type=float, default=5.)
    parser.add_argument('--open-mm', type=float, default=55.)
    parser.add_argument('--close-mm', type=float, default=10.)
    parser.add_argument('--axes-deg', type=float, nargs=3, default=[0.,0.,0.],
                        metavar=('ROLL','PITCH','YAW'), help='Joy-Con local axes to TCP local axes')
    args = parser.parse_args()
    if not (1 <= args.port <= 65535 and 5 <= args.hz <= 50 and 0 < args.speed_mps <= 0.15
            and 0 < args.angular_dps <= 90 and 0 < args.joint_dps <= 100
            and 0 <= args.j5_margin_deg < 90 and 0 <= args.close_mm < args.open_mm <= 65
            and np.isfinite(args.axes_deg).all()):
        parser.error('Invalid rate, port, speed, axes, margin or gripper range; see README.md')
    handset = Handset(args.side)
    try:
        arm = RpcArm(dict(host=args.host, port=args.port, request_timeout_s=0.4))
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
