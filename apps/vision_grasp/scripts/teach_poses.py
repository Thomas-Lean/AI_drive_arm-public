#!/usr/bin/env python3
"""Teach calibration poses by hand-dragging; see README section 3.

Runs the camera preview while the arm is in damping mode so you can drag it
into a pose. Terminal keys: c = hold position and append the measured joints
to a teach YAML, f = re-enter damping for the next drag, q (or q/ESC in the
window) = quit. lock_all is a passive damping lock that sags under gravity;
the capture instead actively holds every joint with a position command, so
keep supporting the arm until ~0.5s after pressing c.
"""
import argparse
from datetime import datetime
from pathlib import Path
import select
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from handeye import Camera, RobotFault, RpcArm, load_config, log  # noqa: E402
from utils.paths import data_root  # noqa: E402

HOLD_INTERVAL_MS = 200


def overlay(image, lines):
    y = 26
    for line in lines:
        cv2.putText(image, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)
        y += 26


def joints(arm):
    """Measured joints, or None while any servo feedback is unavailable."""
    s = arm.call('state')['state']
    q, online = s.get('q_deg'), s.get('online', [])
    if (not isinstance(q, list) or len(q) != 6
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in q)
            or len(online) != 7 or not all(online)):
        return None
    return [float(v) for v in q]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'config' / 'default.yaml')
    parser.add_argument('--host', help='Override robot.host')
    parser.add_argument('--port', type=int, help='Override robot.port')
    parser.add_argument('--power', type=int, default=500,
                        help='Damping power while dragging (500=factory soft; 0=free)')
    args = parser.parse_args(argv)
    if not 0 <= args.power <= 1000:
        raise ValueError('--power must be a drag-friendly damping value (e.g. 500)')
    cfg = load_config(args.config.resolve(), motion=False)
    if args.host:
        cfg['robot']['host'] = args.host
    if args.port:
        cfg['robot']['port'] = args.port
    out = data_root(args.config) / ('teach_poses_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.yaml')
    camera = Camera(cfg['camera'])
    arm = RpcArm(cfg['robot'])
    captured = []
    dragging = True
    log(f'Teach mode: drag the arm (damping {args.power}); output file {out.name}')
    log('Terminal keys: c = hold & record, f = damping again, q = quit. Keep supporting '
        'the arm until ~0.5s after c: the hold is a position command, not the passive '
        'damping lock that sags.')
    try:
        arm.call('damping', id=-1, power=args.power)
        window = 'teach preview  (terminal: c record / f drag / q quit)'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        while True:
            image, _ = camera.frame()
            overlay(image, ['DRAG (damping)' if dragging else 'HELD (position lock)',
                            f'captured {len(captured)}'])
            cv2.imshow(window, image)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                break
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            line = sys.stdin.readline().strip().lower()
            if line == 'c':
                q = joints(arm)
                if q is None:
                    log('Capture skipped: servo feedback not ready; try again')
                    continue
                try:
                    for i, v in enumerate(q):
                        arm.call('set_joint', id=i, angle=v, interval_ms=HOLD_INTERVAL_MS)
                except RobotFault as exc:
                    log(f'Hold failed ({exc}); a joint may be outside URDF limits. NOT recorded')
                    continue
                dragging = False
                time.sleep(0.6)
                held = joints(arm)
                if held is not None:
                    sag = float(np.max(np.abs(np.array(held) - np.array(q))))
                    log(f'Hold engaged, sag {sag:.2f} deg')
                captured.append(q)
                with out.open('a', encoding='utf-8') as stream:
                    if len(captured) == 1:
                        stream.write('# Taught poses; paste under poses_deg in default.yaml\nposes_deg:\n')
                    stream.write('  - [' + ', '.join(f'{v:.2f}' for v in q) + ']\n')
                log(f'Recorded {len(captured)}: {[round(v, 1) for v in q]}')
            elif line == 'f':
                arm.call('damping', id=-1, power=args.power)
                dragging = True
                log('Damping again; drag to the next pose')
            elif line == 'q':
                break
    except KeyboardInterrupt:
        pass
    except RobotFault as exc:
        log(f'FAILED: {exc}; the arm stays in its last damping/hold state')
        return 2
    finally:
        camera.close()
        arm.close()
        cv2.destroyAllWindows()
    if captured:
        log(f'{len(captured)} poses saved to {out}; paste the lines into default.yaml poses_deg '
            '(check the joint limits before live collection)')
    else:
        log(f'No poses captured ({out.name} not written)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
