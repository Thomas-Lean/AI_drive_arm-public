"""d1t_sdk 适配层：原 d1armd TCP JSON-lines 适配器，改连本仓库 d1t_sdk.D1Arm（DDS）。

保留类名 RpcArm / RobotFault 与原有方法签名（state/wait_still/move/call/close），
上层脚本不动；语义映射要点（原 d1armd 语义逐项核对过，修改前先读 README）：
- 所有运动请求一律 async，由 wait_still 轮询确认到位（静止 + 到位容差 + 稳定窗）。
- 流式轨迹走 stream()：每收到一帧 ArmState 发下一个目标，节奏跟臂走，不用 sleep。
- 带 pump 回调的等待（wait_still / wait_arrived / stream / call(..., pump=)）在阻塞期间每拍调它一次，
  上层用它处理 GUI 窗口事件。
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
import threading
import time
from pathlib import Path

import numpy as np

_SDK_DIR = Path(__file__).resolve().parents[3] / 'sdk'  # 仓库根/sdk（d1t_sdk 所在）
if _SDK_DIR.is_dir() and str(_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(_SDK_DIR))  # 未 source deploy/env.sh 也能 import
from d1t_sdk import D1Arm, D1ArmError  # noqa: E402
from d1t_sdk._common import CTRL_STREAMING  # noqa: E402


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

# 关节流式（RpcArm.stream）
STREAM_FRAME_TIMEOUT_S = 1.0   # 等下一帧 ArmState 的超时；控制器 20 Hz，1 s 没帧就是链路/控制器出事了
STREAM_WARMUP_FRAMES = 60      # 首样本发出后最多等这么多帧进入 STREAMING（约 3 s，同 SDK 轨迹任务）


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
        """d1armd 时代的 RPC 入口：按命令分发到 D1Arm 对应方法。
        任何命令都可以带 pump=回调：SDK 调用阻塞期间主线程每 50 ms 调它一次（保持 GUI 窗口响应）。"""
        if self.broken:
            raise RobotFault('SDK connection is unusable after a transport/protocol error')
        return self._sdk(lambda: self._dispatch(command, **args), args.pop('pump', None))

    def _sdk(self, call, pump=None):
        """调一个 SDK 方法，把它抛的 D1ArmError 转成 RobotFault（'没收到 ArmState' = 链路坏了）。"""
        try:
            return self._run(call, pump)
        except RobotFault:
            raise
        except D1ArmError as exc:
            if '没收到 ArmState' in str(exc):  # d1t_sdk 等首帧超时的固定文案：链路级
                self.broken = True
            raise RobotFault(str(exc)) from exc

    @staticmethod
    def _run(call, pump=None):
        """不给 pump 就直接调。给了 pump：SDK 调用放到后台线程，主线程每 50 ms 调一次 pump 直到它返回——
        SDK 的运动/夹爪/流式收尾方法内部是干等事件，几十毫秒到几秒不等，期间不处理窗口事件，
        GNOME 会判定窗口未响应（check-alive-timeout 5 s）让人「强制退出」。"""
        if pump is None:
            return call()
        done, box = threading.Event(), {}

        def worker():
            try:
                box['value'] = call()
            except BaseException as exc:  # noqa: BLE001 — 原样交回主线程
                box['error'] = exc
            done.set()

        threading.Thread(target=worker, daemon=True, name='d1t-sdk-call').start()
        while not done.wait(0.05):
            pump()
        if 'error' in box:
            raise box['error']
        return box['value']

    def command_q(self):
        """当前保持目标（URDF 角，度），流式轨迹从它起步。取法同 SDK 自己发轨迹用的起点
        _start_q_urdf：命令角优先，与实测差超过 3° 的轴才退回实测——从实测角起步，第一帧目标
        就落到下沉后的角度，负载轴出力消失，臂起转前先掉一下。SDK 没公开这个值，只能调它的
        私有方法（同一个仓库）；注入的假臂没有这个方法时退回实测角。"""
        start = getattr(self.arm, '_start_q_urdf', None)
        if start is None:
            return list(self.state()['q_deg'])
        return [float(v) for v in self._sdk(lambda: start(self.arm._wait_state()))]

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
        if command == 'gripper':   # 阻塞的是 arm.gripper（SDK 内部等它走完）；call(..., pump=) 让它在后台线程里等
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

    def wait_still(self, target=None, pump=None):
        """等待到位：IDLE + 到位容差内 + 稳定窗（settle_s 内波动不超过
        stable_tolerance_deg）；降级遥测不接受为"已稳定"。每拍调一次 pump（保持 GUI 窗口响应）。"""
        deadline = time.monotonic()+self.cfg['move_timeout_s']
        window = []
        s = q = None
        while time.monotonic() < deadline:
            if pump is not None:
                pump()
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
        detail = ''
        if q is not None:
            detail = f"; mode={s.get('mode')} q={np.round(q, 1).tolist()}"
            if target is not None:
                err = np.abs(q - np.asarray(target, float))
                detail += (f" target={np.round(np.asarray(target, float), 1).tolist()} "
                           f"max error J{int(np.argmax(err))}={float(err.max()):.1f} deg "
                           f"(tolerance {self.cfg['arrival_tolerance_deg']})")
        raise RobotFault('Motion/settling timeout; no automatic safe-pose move will follow' + detail)

    def move(self, pose, vel=None, pump=None):
        """关节运动：默认用配置的 vel_dps/acc_dps2，可用 vel 覆盖速度
        （ready/safe 低速接近就用它）。阻塞直到到位并稳定；等待期间每拍调一次 pump。"""
        self.call('move_joints', q=[float(v) for v in pose],
                  vel_dps=self.cfg['vel_dps'] if vel is None else vel,
                  acc_dps2=self.cfg['acc_dps2'], **{'async':True})
        return self.wait_still(np.asarray(pose, float), pump)

    def wait_arrived(self, target, pump=None):
        """只等到位容差（不等稳定窗）：流式轨迹走完、闭爪之前用，轨迹末尾已经减速到停。
        降级遥测不算到位；超时抛 RobotFault。每拍调一次 pump。"""
        target = np.asarray(target, float)
        deadline = time.monotonic() + self.cfg['move_timeout_s']
        while True:
            if pump is not None:
                pump()
            s = self.state()
            err = np.abs(np.asarray(s['q_deg'], float) - target)
            if s['mode'] == 'IDLE' and err.max() <= self.cfg['arrival_tolerance_deg'] and not s.get('degraded'):
                return s
            if time.monotonic() > deadline:
                raise RobotFault('Not at the end of the streamed trajectory within '
                                 f"{self.cfg['move_timeout_s']}s; max error "
                                 f"J{int(np.argmax(err)) + 1}={float(err.max()):.1f} deg "
                                 f"(tolerance {self.cfg['arrival_tolerance_deg']})")
            time.sleep(self.cfg['poll_s'])

    def stream(self, samples, j6=None, pump=None):
        """按帧下发一条关节轨迹：samples 每行 6 个 URDF 角（度），行间隔一个控制帧（50 ms）；
        j6 = 与 samples 等长的 J6 目标（舵机角，度），None（或某项为 None）= J6 保持上次命令角。

        节奏跟臂走：stream_start（HOLD 在当前姿态）→ 首样本重发到控制器进入 STREAMING → 之后每等来
        一帧 ArmState（上一目标已随那帧发出）再发下一个目标，不用 sleep。每拍调一次 pump。
        遥测降级（舵机掉线）时不进流式；流式中遥测校验失败（掉线超窗、时间戳停走、控制器急停/故障、
        掉出 STREAMING）抛 RobotFault。正常走完是 stream_stop；任何异常（含 Ctrl-C）直接 estop，见 _end_stream。
        返回 dict(frames, duration_s, j6_start_s, j6_end_s)：J6 开始/结束变化的时刻相对本次流式起点，
        J6 全程没动时没有这两个键。"""
        if self.state().get('degraded'):
            raise RobotFault('Servo feedback degraded; not starting a streamed trajectory')
        samples = np.asarray(samples, float)
        if samples.ndim != 2 or samples.shape[1] != 6 or len(samples) < 2:
            raise ValueError('stream samples must be an [N>=2, 6] array of URDF joint angles')
        j6 = None if j6 is None else [None if v is None else float(v) for v in j6]
        if j6 is not None and len(j6) != len(samples):
            raise ValueError('j6 must have one entry per sample, or be None')
        started = time.monotonic()
        self._sdk(self.arm.stream_start, pump)
        try:
            self._sdk(lambda: self.arm.stream_joints(samples[0].tolist(), j6_deg=None if j6 is None else j6[0]))
            warm = 0
            while self.arm.state()['ctrl_state'] != CTRL_STREAMING:
                if pump is not None:
                    pump()
                self._sdk(lambda: self.arm.wait_frame(STREAM_FRAME_TIMEOUT_S))
                warm += 1
                if warm > STREAM_WARMUP_FRAMES:
                    raise RobotFault('Controller did not enter STREAMING within '
                                     f'{STREAM_WARMUP_FRAMES} frames of the first sample')
            marks = {}
            for k in range(1, len(samples)):
                if pump is not None:
                    pump()
                self._sdk(lambda: self.arm.wait_frame(STREAM_FRAME_TIMEOUT_S))
                self._sdk(lambda: self.arm.stream_joints(samples[k].tolist(),
                                                         j6_deg=None if j6 is None else j6[k]))   # 立即返回，不用泵
                if j6 is not None and (k == 1 or j6[k] != j6[k - 1]):
                    marks.setdefault('j6_start_s', time.monotonic() - started)   # 第一次开始变的帧
                    marks['j6_end_s'] = time.monotonic() - started               # 最后一次还在变的帧
                self.state()                                                    # 遥测校验（同 wait_still）
                if self.arm.state()['ctrl_state'] != CTRL_STREAMING:
                    raise RobotFault('Controller left STREAMING during the streamed trajectory')
            self._sdk(lambda: self.arm.wait_frame(STREAM_FRAME_TIMEOUT_S))      # 末样本随下一帧发出
        except BaseException:                 # 含 Ctrl-C、RobotFault、窗口泵抛的错
            self._end_stream(estop=True)
            raise
        self._end_stream(estop=False)
        return dict(frames=len(samples), duration_s=time.monotonic() - started, **marks)

    def _end_stream(self, estop):
        """退出流式，任何路径都走这里。收尾本身失败只记日志：之后什么都不再发，控制器看门狗约 0.3 s 停臂。

        - 正常走完：stream_stop → HOLD 在最后一个流式目标（命令角）上，不锁存急停。
        - 任何异常（RobotFault、Ctrl-C、窗口泵抛的错）：直接 estop。它一次就清掉 SDK 的流式状态并让
          控制器立刻停（SDK 实测 ~110 ms），和以前分段走时 Ctrl-C / 故障落到 stop() 的软急停是同一个
          终态；而控制器已经急停/故障时 stream_stop 的 HOLD 永远确认不了，要白等 5 s 才报错。"""
        try:
            (self.arm.estop if estop else self.arm.stream_stop)()
        except D1ArmError as exc:
            log(f"WARN: {'estop' if estop else 'stream_stop'} after the streamed trajectory failed: {exc}; "
                'nothing is being sent, the controller watchdog stops the arm')
        if estop:
            self.self_estop = True  # 自己发的急停才允许下一条运动前自动清
