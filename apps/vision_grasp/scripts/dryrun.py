#!/usr/bin/env python3
"""Kinematic dry-run of poses_deg against a d1armd instance (use --sim there).

Runs the same ready pose -> collection poses -> safe pose sequence as handeye.py
but without any camera: every pose is executed on the given server, FK of the
measured joints is recorded, duplicate poses are flagged with the collection
rule, and the FK diversity is compared with the quality thresholds. Verdict is
necessary, not sufficient: marker visibility and real-arm reachability still
need live confirmation. Exit 0 only if all poses ran, none is a duplicate and
the diversity thresholds pass.
"""
import argparse
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utils.core import diversity  # noqa: E402
from handeye import RobotFault, RpcArm, load_config, log, pose_matrix  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config' / 'default.yaml')
    parser.add_argument('--host', help='Override robot.host (point at the sim instance)')
    parser.add_argument('--port', type=int, help='Override robot.port')
    args = parser.parse_args(argv)
    cfg = load_config(args.config.resolve(), motion=True)
    if args.host:
        cfg['robot']['host'] = args.host
    if args.port:
        cfg['robot']['port'] = args.port
    poses = cfg['robot']['poses_deg'][:cfg['collection']['max_pose_attempts']]
    if len(poses) < cfg['quality']['min_samples']:
        log(f'WARNING: only {len(poses)} poses < min_samples {cfg["quality"]["min_samples"]}')
    arm = RpcArm(cfg['robot'])
    accepted, problems = [], []
    previous = None
    try:
        arm.wait_still()
        log(f'Ready pose {cfg["robot"]["ready_pose_deg"]}')
        arm.move(cfg['robot']['ready_pose_deg'],
                 cfg['robot']['approach_vel_dps'])
        previous = np.asarray(arm.state()['q_deg'], float)
        for index, target in enumerate(poses, 1):
            start = time.monotonic()
            arm.move(target)
            duration = time.monotonic() - start
            q = np.asarray(arm.state()['q_deg'], float)
            bg = pose_matrix(arm.call('fk', q=q.tolist())['pose'])
            jump = float(np.max(np.abs(q - previous))) if previous is not None else 0.
            previous = q
            duplicate = any(
                np.linalg.norm(bg[:3, 3] - a[:3, 3]) < cfg['collection']['min_pose_translation_m']
                and diversity(np.array([bg, a]))['rotation_span_deg'] < cfg['collection']['min_pose_rotation_deg']
                for a in accepted)
            accepted.append(bg)
            tcp = bg[:3, 3]
            note = ''
            if duplicate:
                problems.append(f'pose {index:02d} duplicates an earlier pose')
                note = '  DUPLICATE'
            print(f'{index:02d}/{len(poses)} q={np.round(q, 1).tolist()} '
                  f'tcp=[{tcp[0]:+.3f},{tcp[1]:+.3f},{tcp[2]:+.3f}] '
                  f'move {duration:.1f}s jump {jump:.1f}deg{note}', flush=True)
        log(f'Safe pose {cfg["robot"]["safe_pose_deg"]}')
        arm.move(cfg['robot']['safe_pose_deg'],
                 cfg['robot']['approach_vel_dps'])
    except KeyboardInterrupt:
        log('Interrupted')
        try:
            arm.call('estop')
        except Exception:
            pass
        return 130
    except RobotFault as exc:
        log(f'DRY-RUN FAILED at pose {len(accepted) + 1}/{len(poses)}: {exc}')
        try:
            arm.call('estop')
        except Exception:
            pass
        return 2
    finally:
        arm.close()

    metrics = diversity(np.array(accepted))
    quality = cfg['quality']
    checks = [('rotation_span_deg', metrics['rotation_span_deg'], quality['min_rotation_span_deg']),
              ('translation_span_m', metrics['translation_span_m'], quality['min_translation_span_m']),
              ('axis_ratio', metrics['axis_ratio'], quality['min_axis_ratio'])]
    print(f'\n== dry-run summary ==')
    print(f'poses executed {len(accepted)}/{len(poses)}; duplicates {len(problems)}; '
          f'min_samples required {quality["min_samples"]}')
    for name, value, limit in checks:
        ok = value >= limit
        if not ok:
            problems.append(f'{name} {value:.3f} below required {limit}')
        print(f'{name}: {value:.3f} vs >= {limit}  {"OK" if ok else "FAIL"}')
    for problem in problems:
        print('PROBLEM:', problem)
    if not problems:
        print('VERDICT: pose list passes kinematic checks; confirm marker visibility '
              'on the real arm (live_view) before collecting.')
        return 0
    print('VERDICT: fix the problems above before live collection.')
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
