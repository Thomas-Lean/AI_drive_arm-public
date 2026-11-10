"""d1armd 通信适配层：顺序 JSON-lines RPC、遥测校验、到位等待。

要点（真机验证过的语义，修改前先读 README）：
- 所有运动请求一律 async，由 wait_still 轮询确认到位（IDLE + 到位容差 + 稳定窗）。
- 运动期间总线优先给控制帧、轮询降频，个别舵机会被瞬时标记离线：
  offline_grace_s 容忍窗内只标注 degraded 不报错；7/7 掉线立即报错。
- estop 会锁存 err='estop' 直到下一条轨迹（home 可清除），调用方慎用。
"""
from datetime import datetime
import json
import socket
import time

import numpy as np


class RobotFault(RuntimeError):
    """机器人侧故障（通信/协议/遥测异常统一抛出）。"""


def log(message):
    """带时间戳的统一日志输出。"""
    print(f'[{datetime.now().strftime("%H:%M:%S")}] {message}', flush=True)


class RpcArm:
    """d1armd 顺序 JSON-lines 适配器；不订阅推送、不复用官方 Python 客户端
    （规避其空闲读超时与共享应答槽问题）。"""

    def __init__(self, cfg):
        self.cfg, self.buf = cfg, b''
        self.sock = socket.create_connection((cfg['host'], cfg['port']), cfg['request_timeout_s'])
        self.sock.settimeout(cfg['request_timeout_s'])
        self.last_stamp = None
        self.offline_since = None
        self.broken = False

    def close(self):
        self.sock.close()

    def call(self, command, **args):
        """发送一条请求并等待下一条非事件应答；evt 推送行被跳过。"""
        if self.broken:
            raise RobotFault('SDK connection is unusable after a transport/protocol error')
        try:
            self.sock.sendall((json.dumps(dict(cmd=command, **args), allow_nan=False)+'\n').encode())
            while True:
                while b'\n' not in self.buf:
                    data = self.sock.recv(65536)
                    if not data:
                        raise ConnectionError('d1armd disconnected')
                    self.buf += data
                    if len(self.buf) > 1024*1024:
                        raise ValueError('Oversized d1armd response')
                line, self.buf = self.buf.split(b'\n', 1)
                response = json.loads(line)
                if not isinstance(response, dict):
                    raise ValueError('SDK response is not a JSON object')
                if 'evt' in response:
                    continue
                if not response.get('ok'):
                    raise RobotFault(str(response.get('err', 'SDK request failed')))
                return response
        except (OSError, ValueError) as exc:
            self.broken = True
            self.close()
            raise RobotFault(f'SDK communication failed: {exc}') from exc

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
        self.call('move_joints', q=pose,
                  vel_dps=self.cfg['vel_dps'] if vel is None else vel,
                  acc_dps2=self.cfg['acc_dps2'], **{'async':True})
        return self.wait_still(np.asarray(pose))
