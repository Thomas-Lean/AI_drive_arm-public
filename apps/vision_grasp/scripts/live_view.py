#!/usr/bin/env python3
"""Live RealSense color stream in an OpenCV window; nothing else.

Reads serial/resolution/fps from the calibration YAML camera section so the
view matches what handeye.py will capture. Keys: q/ESC or closing the window.
"""
import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml


def log(message):
    print(f'[{datetime.now().strftime("%H:%M:%S")}] {message}', flush=True)


def main(argv=None):
    here = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=here / 'config' / 'default.yaml',
                        help='YAML whose camera section provides serial/width/height/fps')
    parser.add_argument('--serial', help='Override camera serial')
    parser.add_argument('--width', type=int, help='Override image width')
    parser.add_argument('--height', type=int, help='Override image height')
    parser.add_argument('--fps', type=int, help='Override frame rate')
    args = parser.parse_args(argv)
    camera = {}
    if args.config and args.config.is_file():
        document = yaml.safe_load(args.config.read_text(encoding='utf-8'))
        if isinstance(document, dict):
            camera = document.get('camera') or {}
    serial = args.serial if args.serial is not None else str(camera.get('serial') or '')
    width = args.width or camera.get('width') or 1280
    height = args.height or camera.get('height') or 720
    fps = args.fps or camera.get('fps') or 30
    if not (isinstance(width, int) and width > 0 and isinstance(height, int) and height > 0
            and isinstance(fps, int) and fps > 0):
        raise ValueError('width/height/fps must be positive integers')

    import pyrealsense2 as rs  # Imported here so --help works without the device SDK.
    config = rs.config()
    if serial:
        config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline = rs.pipeline()
    try:
        pipeline.start(config)
    except Exception as exc:
        log(f'Cannot start color stream: {exc}; check connection, another program using the '
            'camera, or udev rules (README 1.2)')
        return 2
    window = f'RealSense color {width}x{height}@{fps}  (q/ESC quit)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame = pipeline.wait_for_frames(timeout_ms=3000).get_color_frame()
            if not frame:
                continue
            cv2.imshow(window, np.asanyarray(frame.get_data()))
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        return 130
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
