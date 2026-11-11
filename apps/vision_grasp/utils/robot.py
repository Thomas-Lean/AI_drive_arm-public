"""d1t_sdk 适配层：原 d1armd TCP JSON-lines 适配器，改连本仓库 d1t_sdk.D1Arm（DDS）。

保留类名 RpcArm / RobotFault 与原有方法签名（state/wait_still/move/call/close），
上层脚本不动；语义映射要点（原 d1armd 语义逐项核对过，修改前先读 README）：
- 所有运动请求一律 async，由 wait_still 轮询确认到位（静止 + 到位容差 + 稳定窗）。
- state 键沿用 d1armd 口径：q_deg（6 URDF 角）/online（7）/stamp_ms/mode/err/degraded；
  控制器 HOLD 视为静止（'IDLE'），轨迹/流式执行中为 'MOVING'。
- estop 锁存语义：本进程经 call('estop') 发出的急停，锁存在 state() 里照常可见
  （err='estop'），并在下一条运动命令前自动 reset() 清掉（d1armd「下一条轨迹
  自动清」的等价复刻，上层恢复路径零改动）。**外部** estop 与任何 FAULT 都不
  自动清：运动命令直接抛 RobotFault（消息带 mode 与 fault 位），确认安全后
  手动 `python3 -m d1t_sdk reset`——绝不悄悄放行别人按下的急停或未读的故障。
- 链路彻底断开时本进程未必还能补发急停（DDS 同进程重复建 participant 会撞端口），
  兜底是臂载控制器看门狗：停止发送约 0.3 s 后自行停臂。
- 拖动示教用 'free'（free_all 整臂失力，臂会掉）+ 'lock'（lock_all 锁在当前实测角），
  见 scripts/teach_poses.py；原工程的 'damping'/'set_joint' 无对应接口，显式拒绝。
"""
from datetime import datetime
import sys
import time
from pathlib import Path

import numpy as np

_SDK_DIR = Path(__file__).resolve().parents[3] / 'sdk'  # 仓库根/sdk（d1t_sdk 所在）
if _SDK_DIR.is_dir() and str(_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(_SDK_DIR))  # 未 source deploy/env.sh 也能 import
from d1t_sdk import D1Arm, D1ArmError  # noqa: E402


class RobotFault(RuntimeError):
    """机器人侧故障（通信/协议/遥测异常统一抛出）。"""


def log(message):
    """带时间戳的统一日志输出。"""
    print(f'[{datetime.now().strftime("%H:%M:%S")}] {message}', flush=True)


# d1t_sdk state()['mode'] → d1armd 遥测口径：静止=IDLE，执行中=MOVING
_STILL_MODES = ('IDLE', 'HOLD')
_MOVING_MODES = ('MOVING', 'STREAMING')
_LATCHED_MODES = ('ESTOP', 'FAULT')  # d1armd 里表现为 mode='IDLE' + err 锁存串
# fault 位名（与臂载控制器/ sdk README 的位定义一致，见 d1t_sdk._common）
_FAULT_BITS = {0x01: 'watchdog', 0x02: 'estop', 0x04: 'multi-turn',
               0x08: 'servo-prot', 0x10: 'comm-loss', 0x20: 'j6-thermal'}


def _fault_names(bits):
    names = [name for bit, name in _FAULT_BITS.items() if bits & bit]
    return f"0x{bits:02x}" + (f" ({'|'.join(names)})" if names else '')


class RpcArm:
    """上层脚本与 d1t_sdk.D1Arm 之间的顺序 RPC 适配器；上层见到的 state 键与
    d1armd 时代一致。arm 参数仅供测试注入假 D1Arm，生产用默认 DDS。"""

    def __init__(self, cfg, arm=None):
        self.cfg = cfg
        self.last_stamp = None
        self.offline_since = None
        self.broken = False
        self.self_estop = False  # 本进程发过 estop 且尚未复位（唯一允许自动清的锁存）
        if arm is None:
            try:
                arm = D1Arm()
            except Exception as exc:  # CYCLONEDDS_URI 没配 / DDS 起不来
                raise RobotFault(f'Cannot start d1t_sdk transport: {exc}') from exc
        self.arm = arm

    def close(self):
        self.arm.close()

    # ---------- d1t_sdk 状态 → d1armd 遥测口径 ----------

    def _read_arm_state(self):
        """读一帧 D1Arm.state()。读不到状态一律视为链路级故障：置 broken。"""
        try:
            return self.arm.state(timeout_s=float(self.cfg.get('request_timeout_s', 5.0)))
        except D1ArmError as exc:
            self.broken = True
            raise RobotFault(f'SDK communication failed: {exc}') from exc

    def _state_payload(self):
        """d1t_sdk state dict → d1armd 遥测 dict（mode/err 映射）。"""
        s = self._read_arm_state()
        out = dict(q_deg=s['q_deg'], online=s['online'], stamp_ms=s['stamp_ms'],
                   servo_raw_deg=s.get('servo_raw_deg'), gripper_mm=s.get('gripper_mm'),
                   seq=s.get('seq'))
        mode = s['mode']
        if mode in _MOVING_MODES:
            out['mode'] = 'MOVING'
        elif mode in _STILL_MODES:
            out['mode'] = 'IDLE'
        elif mode == 'ESTOP':
            out['mode'], out['err'] = 'IDLE', 'estop'
        elif mode == 'FAULT':
            out['mode'], out['err'] = 'IDLE', f"fault 0x{int(s.get('fault', 0)):02x}"
        else:  # INIT / RELEASED：原样透传，state() 的校验会拒收
            out['mode'] = mode
        return out

    def _ensure_movable(self):
        """运动前的锁存检查。只自动清**本进程**发出的 estop（d1armd「下一条轨迹
        自动清」的等价复刻）；外部 estop 与任何 FAULT 一律拒绝并抛错——别人按的
        急停、没读过的故障位，绝不悄悄放行。确认安全后手动
        `python3 -m d1t_sdk reset`。"""
        s = self._read_arm_state()
        mode = s['mode']
        if mode == 'ESTOP' and self.self_estop:
            log('clearing self-issued estop via reset() before motion')
            try:
                self.arm.reset()
            except D1ArmError as exc:
                raise RobotFault(f'reset after self-issued estop failed: {exc}') from exc
            self.self_estop = False
            return
        if mode in _LATCHED_MODES:
            bits = _fault_names(int(s.get('fault', 0)))
            raise RobotFault(
                f'motion refused: latched {mode} (fault {bits}); confirm it is safe, '
                'then clear it manually with: python3 -m d1t_sdk reset')
        self.self_estop = False  # 运动得以执行说明锁存已不在，标志随之作废

    # ---------- 上层接口（签名与 d1armd 时代一致） ----------

    def call(self, command, **args):
        """d1armd 时代的 RPC 入口：按命令分发到 D1Arm 对应方法。"""
        if self.broken:
            raise RobotFault('SDK connection is unusable after a transport/protocol error')
        try:
            return self._dispatch(command, **args)
        except RobotFault:
            raise
        except D1ArmError as exc:
            if '没收到 ArmState' in str(exc):  # d1t_sdk 等首帧超时的固定文案：链路级
                self.broken = True
            raise RobotFault(str(exc)) from exc

    def _dispatch(self, command, **args):
        if command == 'state':
            return {'ok': True, 'state': self._state_payload()}
        if command == 'fk':
            return {'ok': True, 'pose': self.arm.fk(args['q'])}
        if command == 'ik':
            try:
                q = self.arm.ik(args['pose']['pos'], rpy=args['pose'].get('rpy'),
                                quat=args['pose'].get('quat'), seed=args.get('seed'))
            except D1ArmError as exc:
                if str(exc).startswith('IK 无解'):  # d1t_sdk 固定文案；上层按
                    # 'ik no solution' 前缀识别后换候选重试，不能笼统当故障
                    raise RobotFault(f'ik no solution: {exc}') from exc
                raise
            return {'ok': True, 'q_deg': q}
        if command == 'move_joints':
            self._ensure_movable()
            self.arm.move_joints(args['q'], vel_dps=args.get('vel_dps', 80.0),
                                 acc_dps2=args.get('acc_dps2', 500.0), async_=True)
            return {'ok': True}
        if command == 'move_l':
            self._ensure_movable()
            self.arm.move_l(args['pose']['pos'], rpy=args['pose'].get('rpy'),
                            quat=args['pose'].get('quat'), vel=args.get('vel', 0.15),
                            acc=args.get('acc', 2.0), async_=True)
            return {'ok': True}
        if command == 'gripper':
            self._ensure_movable()
            self.arm.gripper(args['mm'])
            return {'ok': True}
        if command == 'estop':
            self.arm.estop()
            self.self_estop = True  # 自己发的急停才允许下一条运动前自动清
            return {'ok': True}
        if command == 'free':  # 拖动示教：整臂失力（臂会掉，调用方负责先托住）
            self.arm.free_all()
            return {'ok': True}
        if command == 'lock':  # 全轴 HOLD，目标取当前实测角（失力拖完锁住不会弹回）
            self.arm.lock_all()
            return {'ok': True}
        if command in ('damping', 'set_joint'):
            raise RobotFault(
                f"'{command}' has no d1t_sdk counterpart; drag-teach uses "
                "'free' + 'lock' (see scripts/teach_poses.py)")
        raise RobotFault(f'unsupported command: {command}')

    def state(self):
        """读取并校验遥测；瞬时掉线标注 degraded，持续超窗或全部掉线才抛错。"""
        s = self.call('state')['state']
        q = np.asarray(s['q_deg'], float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise RobotFault('Invalid joint feedback')
        online = s.get('online', [])
        if len(online) != 7:
            raise RobotFault('Servo feedback not initialized')
        offline = [i for i, up in enumerate(online) if not up]
        now = time.monotonic()
        if len(offline) == 7:
            raise RobotFault('All servos offline (bus/link failure)')
        if offline:
            if self.offline_since is None:
                self.offline_since = (tuple(offline), now)
            elif now - self.offline_since[1] > self.cfg['offline_grace_s']:
                raise RobotFault(f"Servos {offline} offline for more than "
                                 f"{self.cfg['offline_grace_s']}s")
            s = dict(s, degraded=offline)  # 降级标注：到位确认/采图不得依赖
        else:
            self.offline_since = None
        if s.get('mode') not in ['IDLE','MOVING'] or s.get('err'):
            raise RobotFault(f'Unexpected robot state: {s.get("mode")} {s.get("err", "")}')
        stamp = s.get('stamp_ms')
        if not isinstance(stamp, (int,float)) or not np.isfinite(stamp):
            raise RobotFault('Invalid telemetry timestamp')
        if self.last_stamp is None or stamp != self.last_stamp[0]:
            self.last_stamp = (stamp, now)
        elif now-self.last_stamp[1] > 1:
            raise RobotFault('Telemetry timestamp stopped advancing')
        return s

    def wait_still(self, target=None):
        """等待到位：IDLE + 到位容差内 + 稳定窗（settle_s 内波动不超过
        stable_tolerance_deg）；降级遥测不接受为"已稳定"。"""
        deadline = time.monotonic()+self.cfg['move_timeout_s']
        window = []
        while time.monotonic() < deadline:
            s = self.state()
            q = np.asarray(s['q_deg'], float)
            arrived = target is None or np.max(np.abs(q-target)) <= self.cfg['arrival_tolerance_deg']
            if s['mode'] == 'IDLE' and arrived and not s.get('degraded'):
                if window and np.max(np.abs(q-window[0][1])) > self.cfg['stable_tolerance_deg']:
                    window = []
                window.append((time.monotonic(), q))
                if window[-1][0]-window[0][0] >= self.cfg['settle_s']:
                    return s
            else:
                window = []
            time.sleep(self.cfg['poll_s'])
        raise RobotFault('Motion/settling timeout; no automatic safe-pose move will follow')

    def move(self, pose, vel=None):
        """关节运动：默认用配置的 vel_dps/acc_dps2，可用 vel 覆盖速度
        （ready/safe 低速接近就用它）。阻塞直到到位并稳定。"""
        self.call('move_joints', q=[float(v) for v in pose],
                  vel_dps=self.cfg['vel_dps'] if vel is None else vel,
                  acc_dps2=self.cfg['acc_dps2'], **{'async':True})
        return self.wait_still(np.asarray(pose, float))
