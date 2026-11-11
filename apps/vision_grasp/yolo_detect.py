#!/usr/bin/env python3
"""Live YOLO detection preview with per-box camera-frame coordinates.

Detection only: no robot, no hand-eye. Mirrors reBot-DevArm-Grasp's
scripts/object_detection.py on the RealSense D435i stack: boxes are drawn
with class/confidence and the backprojected X/Y/Z (meters, color camera
frame, median depth in a small window); boxes without depth turn orange.
Keys: left click samples depth at a pixel; q/ESC or closing the window quits.
Model/device/conf come from the calibration YAML `grasp:` section.
"""
import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml


def depth_window_mm(depth_mm, u, v, size=5):
    """Median valid depth (mm) in a small pixel window; 0 when none."""
    height, width = depth_mm.shape
    half = size // 2
    roi = depth_mm[max(0, v - half):min(height, v + half + 1),
                   max(0, u - half):min(width, u + half + 1)]
    valid = roi[roi > 0]
    return float(np.median(valid)) if len(valid) else 0.0


def main(argv=None):
    here = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=here / 'config' / 'default.yaml')
    parser.add_argument('--model', help='Override grasp.model YOLO weights')
    parser.add_argument('--device', help="Override grasp.device ('cpu'/'cuda:0'/...)")
    args = parser.parse_args(argv)
    cfg = yaml.safe_load(args.config.read_text(encoding='utf-8')) or {}
    camera = cfg.get('camera') or {}
    grasp = dict(conf=0.5, iou=0.45, device='cpu', classes=[])
    grasp.update(cfg.get('grasp') or {})
    if args.model:
        grasp['model'] = args.model
    if args.device:
        grasp['device'] = args.device
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from utils.paths import data_root, ensure_text_encoder, resolve_model
    model_path = resolve_model(grasp.get('model', ''), args.config)
    if not model_path.is_file():
        raise ValueError(f'YOLO weights not found: {model_path} '
                         '(set grasp.model in the YAML or pass --model)')

    print(f'Load model: {model_path} on {grasp["device"]}')
    from ultralytics import YOLO
    model = YOLO(str(model_path))
    classes = [str(c) for c in (grasp.get('classes') or [])]
    if classes and ('world' in model_path.name.lower() or 'yoloe' in model_path.name.lower()):
        if ensure_text_encoder(args.config) is None:
            print('[WARN] config/mobileclip2_b.ts 不存在，首次 set_classes 将下载 ~240MB')
        model.set_classes(classes)
        print('Open-vocabulary classes:', classes)
    elif classes:
        known = set(str(v) for v in getattr(model, 'names', {}).values())
        missing = [c for c in classes if c not in known]
        if missing:
            print(f'[WARN] not in {model_path.name} vocabulary (will never match): {missing}')
        print('Restricted classes:', classes)

    import pyrealsense2 as rs
    pipeline, config = rs.pipeline(), rs.config()
    if camera.get('serial'):
        config.enable_device(camera['serial'])
    width, height, fps = (camera.get('width', 1280), camera.get('height', 720),
                          camera.get('fps', 30))
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    print(f'[Camera] {width}x{height}@{fps} fx={fx:.1f} cx={cx:.1f} '
          f'depth_scale={scale} m/step')

    # Optional base-frame overlay: hand-eye result + live robot state.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    t_tcp_camera, arm = None, None
    name = (cfg.get('output') or {}).get('filename', 'hand_eye.npz')
    if (cfg.get('board') or {}).get('kind', 'aruco') == 'charuco' and 'charuco' not in name:
        name = name[:-len('.npz')] + '_charuco.npz'  # 与 handeye.load_config 同规则：板型入文件名
    result_path = data_root(args.config) / name
    if result_path.is_file():
        with np.load(result_path, allow_pickle=False) as result:
            t_tcp_camera = result['T_tcp_camera'].copy()
        print(f'[Hand-eye] {result_path.name} loaded')
    else:
        print(f'[Hand-eye] {result_path.name} not found; camera-frame coordinates only')
    if t_tcp_camera is not None:
        from handeye import RobotFault, RpcArm, pose_matrix
        try:
            arm = RpcArm(dict(cfg.get('robot') or {}))
            print('[Robot] polling state; showing base-frame XYZ per box')
        except (RobotFault, OSError) as exc:
            print(f'[Robot] unavailable ({exc}); camera-frame coordinates only')

    clicked = {'u': -1, 'v': -1}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.update(u=x, v=y)

    window = 'YOLO detection (click: sample depth | q: quit)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, width, height)
    cv2.setMouseCallback(window, on_mouse)
    print('[Keys] Left click=sample point  Q/ESC=quit')
    try:
        while True:
            frames = pipeline.wait_for_frames(3000)
            aligned = align.process(frames)
            image = np.asanyarray(aligned.get_color_frame().get_data()).copy()
            # depth_scale is meters per raw step; convert once to millimeters.
            depth_mm = np.asanyarray(aligned.get_depth_frame().get_data()) * scale * 1000.0
            base_camera = None
            if arm is not None:
                try:
                    q = arm.call('state')['state'].get('q_deg')
                    if q:
                        base_camera = pose_matrix(arm.call('fk', q=q)['pose']) @ t_tcp_camera
                except Exception:
                    arm = None
                    print('[Robot] state/fk failed; base-frame XYZ disabled')

            for result in model.predict(image, verbose=False, device=grasp['device'],
                                        conf=grasp['conf'], iou=grasp['iou']):
                boxes = getattr(result, 'boxes', None)
                if boxes is None:
                    continue
                for index in range(len(boxes)):
                    x1, y1, x2, y2 = map(int, boxes.xyxy[index].cpu().numpy())
                    conf = float(boxes.conf[index])
                    name = result.names.get(int(boxes.cls[index]),
                                            str(int(boxes.cls[index])))
                    if classes and name not in classes:
                        continue
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    z_mm = depth_window_mm(depth_mm, u, v)
                    if z_mm > 0:
                        z = z_mm / 1000.0
                        camera_xyz = ((u - cx) * z / fx, (v - cy) * z / fy, z)
                        labels = [f'{name} {conf:.2f}',
                                  f'CAM X{camera_xyz[0]:+.3f} Y{camera_xyz[1]:+.3f} Z{camera_xyz[2]:+.3f} m']
                        if base_camera is not None:
                            p = (base_camera @ np.append(camera_xyz, 1.0))[:3]
                            labels.insert(1, f'BAS X{p[0]:+.3f} Y{p[1]:+.3f} Z{p[2]:+.3f} m')
                        color = (0, 255, 0)
                        cv2.circle(image, (u, v), 4, (0, 0, 255), -1)
                    else:
                        labels = [f'{name} (no depth)']
                        color = (0, 165, 255)
                    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                    if y1 >= 22 * (len(labels) + 1):
                        line_h = 22
                        bg_w = max(len(t) for t in labels) * 11
                        cv2.rectangle(image, (x1, y1 - line_h * (len(labels) + 1)),
                                      (x1 + bg_w, y1), (0, 0, 0), -1)
                        for i, text in enumerate(labels):
                            shade = color if i == 0 else (0, 255, 255) if i == 1 else (200, 200, 0)
                            cv2.putText(image, text, (x1 + 5, y1 - line_h * (len(labels) - i)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, shade, 2)

            if clicked['u'] >= 0:
                u, v = clicked['u'], clicked['v']
                z_mm = depth_window_mm(depth_mm, u, v)
                if z_mm > 0:
                    z = z_mm / 1000.0
                    cv2.drawMarker(image, (u, v), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
                    text = f'TEST X:{(u-cx)*z/fx:+.3f} Y:{(v-cy)*z/fy:+.3f} Z:{z:.3f} m'
                    cv2.rectangle(image, (u + 6, v - 28), (u + 330, v + 4), (0, 0, 0), -1)
                    cv2.putText(image, text, (u + 10, v - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            cv2.putText(image, f'{model_path.name} | {grasp["device"]} | '
                        f'classes: {", ".join(classes) if classes else "ALL"}', (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.imshow(window, image)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), ord('Q'), 27):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if arm is not None:
            arm.close()
        pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
