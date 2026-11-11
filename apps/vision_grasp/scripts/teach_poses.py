#!/usr/bin/env python3
"""拖动示教采集点位：失力拖动 → 锁住 → 记录，结果直接写回配置文件的 poses_deg。

流程（全部在终端按键，预览窗只看画面）：
  启动先锁住全臂 → 回车：低速走到 ready_pose_deg
  → 回车：全臂失力（先托住小臂！）→ 拖到想要的姿态
  → 回车：锁住、等 0.6 s、记录当前关节角 → 回车：再失力拖下一个 …
  其他按键：u = 删掉上一个点，q = 结束。
记满 --n 个自动结束。每记一个点就把全部点位写回 --config 的 poses_deg
（开始时先备份原文件为 <文件名>.bak-时间），另存一份 teach_poses_*.yaml 留档。
失力 = 整臂失力，臂会掉：拖动期间始终托住。出错 / Ctrl-C / 结束时一律锁住，不在失力状态退出。
"""
import argparse
from datetime import datetime
from pathlib import Path
import re
import select
import shutil
import sys
import time

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from handeye import Camera, RobotFault, RpcArm, load_config, log  # noqa: E402
from utils.paths import data_root  # noqa: E402
from d1t_sdk._common import SERVO_LIMITS  # noqa: E402  （utils.robot 已把 sdk/ 加进 sys.path）

SETTLE_S = 0.6  # 锁住后等这么久再读角度（含锁定瞬间的下沉）
POSE_LINE = re.compile(r'^\s*- \[')


def overlay(image, lines):
    y = 26
    for line in lines:
        cv2.putText(image, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)
        y += 26


def read_joints(arm):
    """(URDF 关节角 6 个, 舵机角 J0–J5)；任一舵机反馈不在线返回 (None, None)。"""
    s = arm.call('state')['state']
    q, raw, online = s.get('q_deg'), s.get('servo_raw_deg'), s.get('online', [])
    if (not isinstance(q, list) or len(q) != 6 or len(online) != 7 or not all(online)
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in q)):
        return None, None
    return [float(v) for v in q], (list(raw)[:6] if raw is not None else None)


def out_of_limits(raw):
    """舵机角超出软限位的轴号列表（读不到舵机角时返回空，交给采集时的 SDK 校验）。"""
    if raw is None:
        return []
    return [i for i, v in enumerate(raw) if not SERVO_LIMITS[i][0] <= float(v) <= SERVO_LIMITS[i][1]]


def write_poses(config, poses):
    """把 config 文件里 poses_deg: 下面那一串 `- [...]` 整体替换成 poses（其余行原样保留）。"""
    lines = config.read_text(encoding='utf-8').splitlines(keepends=True)
    idx = next((i for i, ln in enumerate(lines) if ln.lstrip().startswith('poses_deg:')), None)
    if idx is None:
        raise ValueError(f'{config.name} 里找不到 poses_deg:')
    end = idx + 1
    while end < len(lines) and POSE_LINE.match(lines[end]):
        end += 1
    indent = ' ' * (len(lines[idx]) - len(lines[idx].lstrip()) + 2)
    new = [indent + '- [' + ', '.join(f'{v:.2f}' for v in q) + ']\n' for q in poses]
    config.write_text(''.join(lines[:idx + 1] + new + lines[end:]), encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config' / 'default.yaml')
    parser.add_argument('--n', type=int, default=32, help='要记录的点数，记满自动结束（默认 32）')
    args = parser.parse_args(argv)
    if args.n < 1:
        raise ValueError('--n 必须 ≥ 1')
    config = args.config.resolve()
    cfg = load_config(config, motion=False)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    archive = data_root(config) / f'teach_poses_{stamp}.yaml'
    backup = config.with_name(f'{config.name}.bak-{stamp}')
    camera = Camera(cfg['camera'])
    arm = RpcArm(cfg['robot'])
    captured = []
    stage = 'start'  # start → held ⇄ drag
    try:
        arm.call('lock')
        shutil.copy2(config, backup)
        log(f'已锁住。原配置备份为 {backup.name}；点位写回 {config.name}，留档 {archive.name}')
        log('回车 = 低速走到准备位（ready_pose_deg）')
        window = 'teach preview  (terminal: Enter / u undo / q quit)'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        while len(captured) < args.n:
            image, _ = camera.frame()
            overlay(image, [{'start': 'LOCKED - Enter: go ready', 'held': 'LOCKED - Enter: free',
                             'drag': 'FREE - drag, Enter: lock+record'}[stage],
                            f'captured {len(captured)}/{args.n}'])
            cv2.imshow(window, image)
            cv2.waitKey(1)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            key = sys.stdin.readline().strip().lower()
            if key == 'q':
                break
            if key == 'u':
                if stage == 'drag':
                    log('拖动中不能删点：先回车锁住')
                elif captured:
                    gone = captured.pop()
                    write_poses(config, captured)
                    log(f'已删除第 {len(captured) + 1} 个点 {[round(v, 1) for v in gone]}，剩 {len(captured)} 个')
                else:
                    log('还没有点可删')
                continue
            if key:
                log('只认 回车 / u / q')
                continue
            if stage == 'start':
                log('低速走向准备位 …')
                arm.move(cfg['robot']['ready_pose_deg'], vel=cfg['robot']['approach_vel_dps'])
                stage = 'held'
                log('已到准备位。托住小臂后回车 = 失力')
            elif stage == 'held':
                arm.call('free')
                stage = 'drag'
                log(f'已失力，拖到第 {len(captured) + 1} 个姿态，托稳后回车 = 锁住并记录')
            else:  # drag → 锁住、记录
                arm.call('lock')
                stage = 'held'
                time.sleep(SETTLE_S)
                q, raw = read_joints(arm)
                if q is None:
                    log('舵机反馈未就绪，本点没记。已锁住：回车失力后重新拖')
                    continue
                bad = out_of_limits(raw)
                if bad:
                    log(f'J{bad} 超出软限位，本点没记。已锁住：回车失力，往回拖一点再记')
                    continue
                captured.append(q)
                write_poses(config, captured)
                with archive.open('w', encoding='utf-8') as stream:
                    stream.write('# Taught poses\nposes_deg:\n')
                    stream.writelines('  - [' + ', '.join(f'{v:.2f}' for v in p) + ']\n' for p in captured)
                log(f'记录 {len(captured)}/{args.n}: {[round(v, 1) for v in q]}'
                    + ('；回车失力拖下一个' if len(captured) < args.n else ''))
    except KeyboardInterrupt:
        log('Ctrl-C')
    except RobotFault as exc:
        log(f'失败：{exc}')
        return 2
    finally:
        try:
            arm.call('lock')  # 任何退出路径都锁住，不留失力状态
            log('已锁住')
        except Exception as exc:  # noqa: BLE001 — 链路已断时只能提示
            log(f'退出时锁住失败（{exc}）——臂可能仍失力，托住后用 python3 -m d1t_sdk lock')
        camera.close()
        arm.close()
        cv2.destroyAllWindows()
    log(f'共 {len(captured)} 个点' + (f'，已写入 {config.name} 的 poses_deg' if captured else '，配置未改'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
