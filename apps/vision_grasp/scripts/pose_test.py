#!/usr/bin/env python3
"""Motion rehearsal of the calibration pose list; no camera involved.

Sequence: ready pose -> every collection pose in order (move, settle, dwell)
-> ready pose -> safe pose. Ready/safe transits use approach_vel_dps. Each
pose prints measured joints and the drift observed while holding it, which
makes servo hold jitter (limit cycles) visible per pose.

Startup waits until telemetry is healthy (all servos online, no latched
controller error); a latched estop from a previous stop must be cleared once
with `home` before this script will move anything. A soft stop is sent on
failure only when a motion may still be executing: estop latches err='estop'
until the next trajectory runs, so stopping a stationary arm would block the
next run. No automatic travel to any pose on failure.
"""
import argparse
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from handeye import RobotFault, RpcArm, load_config, log  # noqa: E402


def healthy(arm):
    """Return None when telemetry looks motion-safe, else a short reason."""
    try:
        s = arm.call('state')['state']
    except (RobotFault, KeyError) as exc:
        return f'unreadable state: {exc}'
    if len(s.get('online', [])) != 7 or not all(s['online']):
        offline = [i for i, up in enumerate(s.get('online', [])) if not up]
        return f'servo feedback not initialized (offline: {offline})'
    if s.get('mode') not in ('IDLE', 'MOVING'):
        return f"mode={s.get('mode')} err={s.get('err', '')}"
    if s.get('err'):
        return f"latched controller error: {s.get('err')}"
    return None


def stop(arm, motion_sent, message, code):
    """Soft-stop only if a motion may still be executing; never auto-travel."""
    try:
        moving = arm.call('state')['state'].get('mode') != 'IDLE'
    except Exception:
        moving = True  # Unverifiable telemetry: assume the worst.
    if motion_sent and moving:
        log(message + '; sending soft stop (clear the latched estop with '
            f'"python3 clients/python/d1arm.py <host> home" before the next run)')
        try:
            arm.call('estop')
        except Exception:
            pass
    else:
        log(message + '; arm reports stationary, no stop sent')
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config' / 'default.yaml')
    parser.add_argument('--host', help='Override robot.host')
    parser.add_argument('--port', type=int, help='Override robot.port')
    parser.add_argument('--dwell', type=float, default=2.0,
                        help='Seconds to hold each collection pose after settling (default 2)')
    parser.add_argument('--ready-timeout', type=float, default=10.0,
                        help='Seconds to wait at startup for healthy servo telemetry')
    args = parser.parse_args(argv)
    if not args.dwell > 0 or not args.ready_timeout > 0:
        raise ValueError('--dwell and --ready-timeout must be positive')
    cfg = load_config(args.config.resolve(), motion=True)
    if args.host:
        cfg['robot']['host'] = args.host
    if args.port:
        cfg['robot']['port'] = args.port
    poses = cfg['robot']['poses_deg'][:cfg['collection']['max_pose_attempts']]
    robot = cfg['robot']
    arm = RpcArm(robot)
    started = time.monotonic()
    done, motion_sent = 0, False
    try:
        reason = healthy(arm)
        deadline = time.monotonic() + args.ready_timeout
        while reason and time.monotonic() < deadline:
            print(f'waiting for healthy telemetry: {reason}', flush=True)
            time.sleep(0.5)
            reason = healthy(arm)
        if reason:
            if 'estop' in reason:
                log(f'Latched estop detected; clear it once with '
                    f'"python3 clients/python/d1arm.py {robot["host"]} home" or restart d1armd')
            log(f'FAILED at startup: {reason}; no motion was commanded')
            return 2
        arm.wait_still()
        motion_sent = True
        log(f'Moving to ready pose {robot["ready_pose_deg"]} at approach speed')
        arm.move(robot['ready_pose_deg'], robot['approach_vel_dps'])
        for index, target in enumerate(poses, 1):
            print(f'{index:02d}/{len(poses)} target {target}', flush=True)
            arm.move(target)
            arrived = np.asarray(arm.state()['q_deg'], float)
            time.sleep(args.dwell)
            drift = float(np.max(np.abs(np.asarray(arm.state()['q_deg'], float) - arrived)))
            print(f'{index:02d}/{len(poses)} reached q={np.round(arrived, 1).tolist()}; '
                  f'dwell {args.dwell:g}s, hold drift {drift:.2f} deg', flush=True)
            done = index
        log(f'All {done} poses executed; returning to ready pose, then safe pose')
        arm.move(robot['ready_pose_deg'], robot['approach_vel_dps'])
        arm.move(robot['safe_pose_deg'], robot['approach_vel_dps'])
    except KeyboardInterrupt:
        return stop(arm, motion_sent, f'Interrupted at pose {done + 1}/{len(poses)}', 130)
    except RobotFault as exc:
        return stop(arm, motion_sent, f'FAILED at pose {done + 1}/{len(poses)}: {exc}', 2)
    finally:
        arm.close()
    log(f'Motion test finished: {done}/{len(poses)} poses in {time.monotonic() - started:.0f}s')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
