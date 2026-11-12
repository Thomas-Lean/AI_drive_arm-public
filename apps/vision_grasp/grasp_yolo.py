#!/usr/bin/env python3
"""YOLO min-area-rect/short-axis grasping on top of the hand-eye result.

Flow (mirrors reBot-DevArm-Grasp): move to the ready pose, open the gripper,
then show live YOLO detections in an OpenCV window. Key g selects the best
instance, preflights symmetric grasp poses against D1 IK and J5 limits, then
moves to pregrasp. Approach and retreat use the configured final_motion.
Key q opens the gripper, returns to the safe pose and exits.

SDK TCP convention used here (kinematics.hpp default tool): +x = tool forward
(approach into the object), and the gripper opens along `open_axis` (y by
default; flip to z if the first dry run shows a 90-degree jaw rotation).
Install ultralytics first; see README section 10.
"""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handeye import RobotFault, RpcArm, load_config, log, pose_matrix  # noqa: E402
from utils.device import resolve_device  # noqa: E402
from utils.paths import data_root, ensure_text_encoder, resolve_model  # noqa: E402

WINDOW = 'YOLO grasp  g:freeze+grasp  q:open+safe+quit'
DEFAULTS = dict(conf=0.5, iou=0.45, device='auto', approach='camera_ray', classes=[],
                depth_quantile=0.5, top_depth_quantile=0.15, support_z_m=0.0, open_axis='y', final_motion='linear',
                linear_vel_mps=0.04, linear_acc_mps2=0.25, j5_margin_deg=5.0,
                pregrasp_offset_m=0.05, grasp_offset_m=0.015,
                grasp_vel_dps=10.0, wait_before_grip_s=1.0,
                gripper_open_mm=65.0, gripper_close_factor=1.0,
                gripper_close_min_mm=15.0, min_base_z_m=0.02)

# 闭爪后锁停住角的判据（顶物体保持会堵转发热：实测目标比停住角多闭约 3°
# 电流即超 300 mA、82 s 内 J6 从 40 ℃ 升到 45 ℃，20 mm 的差更甚）
GRIP_SETTLE_TIMEOUT_S = 2.0  # 闭爪命令走完后等稳定的超时
GRIP_STABLE_FRAMES = 3       # 连续稳定帧数（每帧变化 < GRIP_STABLE_MM）
GRIP_STABLE_MM = 0.2         # 每帧开度变化阈值，≈0.3°（夹爪标定表斜率 0.55–0.92 mm/°）
GRIP_POLL_S = 0.05           # 轮询间隔（一帧 20 Hz 遥测）


def short_edge(rect):
    """Shortest edge vector and length of a 4-point min-area rect."""
    vec, length = rect[1] - rect[0], float(np.linalg.norm(rect[1] - rect[0]))
    for i in range(4):
        v = rect[(i + 1) % 4] - rect[i]
        if np.linalg.norm(v) < length:
            vec, length = v, float(np.linalg.norm(v))
    return vec.astype(float), length


def depth_quantile_mm(depth_mm, mask, quantile):
    values = depth_mm[mask > 0]
    values = values[values > 0]
    return float(np.quantile(values, quantile)) if len(values) else 0.0


def grasp_camera(rect, depth_mm, k, quantile, contour=None):
    """Grasp geometry in the color camera frame from one min-area rect.

    Returns None without usable depth. approach points from the object toward
    the camera; open follows the rect short edge (gripper closing direction).
    """
    center = rect.mean(axis=0)
    mask = np.zeros(depth_mm.shape, np.uint8)
    cv2.fillPoly(mask, [np.rint(rect if contour is None else contour).astype(np.int32)], 1)
    short, span_px = short_edge(rect)
    if span_px < 2:
        return None
    direction = short / span_px
    if contour is not None:
        # Measure the central cross-section of the instance along its short axis.
        ys, xs = np.nonzero(mask)
        if len(xs) < 8:
            return None
        pixels = np.column_stack((xs, ys)) - center
        long_dir = np.array([-direction[1], direction[0]])
        along = pixels @ long_dir
        lo_end, hi_end = np.percentile(along, [2, 98])
        ends_px = np.array([center + lo_end * long_dir, center + hi_end * long_dir])  # 长轴两端（像素）
        mid = np.median(along)
        band = np.abs(along - mid) <= np.clip(np.ptp(along) * 0.04, 2, 12)
        if band.sum() < 8:
            return None
        low, high = np.percentile(pixels[band] @ direction, [5, 95])
        span_px = high - low
        center = center + mid * long_dir + (low + high) / 2 * direction
        if span_px < 2:
            return None
    z_mm = depth_quantile_mm(depth_mm, mask, quantile)
    if z_mm <= 0:
        return None
    z = z_mm / 1000.0
    position = np.array([(center[0] - k[0, 2]) * z / k[0, 0],
                         (center[1] - k[1, 2]) * z / k[1, 1], z])
    approach = -position / np.linalg.norm(position)
    open_axis = np.array([direction[0] * z / k[0, 0], direction[1] * z / k[1, 1], 0.0])
    open_axis -= float(open_axis @ approach) * approach
    norm = np.linalg.norm(open_axis)
    if norm < 1e-9:
        return None
    open_axis /= norm
    jaw_m = float(np.linalg.norm([direction[0] * span_px * z / k[0, 0],
                                  direction[1] * span_px * z / k[1, 1]]))
    return dict(position=position, approach=approach, open_axis=open_axis,
                jaw_m=jaw_m, ends_px=ends_px if contour is not None else None, line_vector=direction * span_px, center=(int(round(center[0])), int(round(center[1]))))


def tcp_rotation(grasp, t_base_camera, open_axis='y'):
    """Map the camera ray and measured short axis into the D1 TCP convention."""
    rotation = t_base_camera[:3, :3]
    return _tcp_frame(-(rotation @ grasp['approach']), rotation @ grasp['open_axis'], open_axis)


def top_down_rotation(grasp, t_base_camera, open_axis='y'):
    """竖直向下进给（TCP x = 基座 −Z），爪开合方向 = 物体短边在水平面内的方向。
    短边投影到水平面后过短（物体在图里几乎竖着看）返回 None。"""
    open_3d = t_base_camera[:3, :3] @ grasp['open_axis']
    open_3d = np.array([open_3d[0], open_3d[1], 0.0])
    norm = np.linalg.norm(open_3d)
    if norm < 0.2:
        return None
    return _tcp_frame(np.array([0.0, 0.0, -1.0]), open_3d / norm, open_axis)


def top_down_target(object_pos, jaw_m, support_z):
    """竖直下抓的目标点：物体中线高度 = max(顶面 − 半爪宽, 支撑面 + 半爪宽)。
    顶面来自深度，透明物体会读低；躺放物体的中线不可能低于「支撑面 + 半径」，取两者较高者防止压桌。
    返回 (目标点, 是否被支撑面兜底)。"""
    from_top = float(object_pos[2]) - jaw_m / 2
    floor = float(support_z) + jaw_m / 2
    out = np.array(object_pos, float)
    out[2] = max(from_top, floor)
    return out, floor > from_top


def axis_midpoint_xy(ends_px, k, t_base_camera, z):
    """物体长轴两端像素各沿视线与基座系水平面 z 求交，返回两交点中点的 (x, y)。
    纠正透视：顺视线躺放的物体远端在图里被压短，二维中点会偏向近端。
    视线不朝下（与平面无前向交点）时返回 None。"""
    rotation, origin = t_base_camera[:3, :3], t_base_camera[:3, 3]
    hits = []
    for u, v in ends_px:
        ray = rotation @ np.array([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], 1.0])
        if abs(ray[2]) < 1e-9:
            return None
        scale = (z - origin[2]) / ray[2]
        if scale <= 0:
            return None
        hits.append(origin + scale * ray)
    return (hits[0][:2] + hits[1][:2]) / 2


def _tcp_frame(into, open_3d, open_axis):
    """进给方向 into + 爪开合方向 → TCP 旋转矩阵（列 = TCP x/y/z，x 为进给方向）。"""
    if open_axis == 'y':
        y, z = open_3d, np.cross(into, open_3d)
    else:
        z, y = open_3d, np.cross(open_3d, into)
    out = np.column_stack([into, y, z])
    if np.linalg.det(out) < 0:
        out[:, 2] *= -1
    return out


def close_target_mm(jaw_m, factor, minimum_mm, maximum_mm):
    """Closing cap from the measured grasp dimension: measured x factor, clipped
    to [minimum, fully open]. It only decides how far closing may go — after
    the jaws stop (on the object) the grip locks at the stopped angle instead
    (see close_and_lock); the cap is never held against the object."""
    target = float(jaw_m) * 1000.0 * float(factor)
    return float(np.clip(target, minimum_mm, maximum_mm))


def close_and_lock(arm, close_mm, g):
    """Close onto the target, then re-target the stopped angle (no stall hold).

    After the closing move, poll gripper_mm until GRIP_STABLE_FRAMES
    consecutive frames each change less than GRIP_STABLE_MM (jaws blocked by
    the object, or fully closed), then re-send that measured opening so the
    servo holds at the stopped angle instead of pushing against the object.
    No stabilization within GRIP_SETTLE_TIMEOUT_S, or an opening at/below the
    minimum (nothing between the jaws), opens the gripper and raises: a
    stalled grip is never held. Returns the locked opening in mm."""
    log(f'Closing gripper to {close_mm:.1f}mm (cap only; locks at the stopped angle)')
    arm.call('gripper', mm=close_mm)
    readings, stable = [], 0
    deadline = time.monotonic() + GRIP_SETTLE_TIMEOUT_S
    while time.monotonic() < deadline:
        mm = arm.call('state')['state'].get('gripper_mm')
        if mm is None or not np.isfinite(mm):
            raise RobotFault('Gripper feedback unavailable while closing')
        stable = stable + 1 if readings and abs(mm - readings[-1]) < GRIP_STABLE_MM else 0
        readings.append(mm)
        if stable >= GRIP_STABLE_FRAMES:
            break
        time.sleep(GRIP_POLL_S)
    else:
        arm.call('gripper', mm=g['gripper_open_mm'])
        raise RobotFault(f'Gripper did not settle within {GRIP_SETTLE_TIMEOUT_S:.0f}s; '
                         'jaws opened, no hold')
    mm_now = float(readings[-1])
    if mm_now <= g['gripper_close_min_mm'] + 2.0:
        arm.call('gripper', mm=g['gripper_open_mm'])
        raise RobotFault(f'Gripper closed to {mm_now:.1f}mm (<= minimum '
                         f"{g['gripper_close_min_mm']:.0f}+2mm): nothing held; "
                         'jaws opened')
    log(f'Holding at the stopped angle: {mm_now:.1f}mm')
    arm.call('gripper', mm=mm_now)
    return mm_now


def quat_from_matrix(rotation):
    trace = float(np.trace(rotation))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        quat = [0.25 * s, (rotation[2, 1] - rotation[1, 2]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s, (rotation[1, 0] - rotation[0, 1]) / s]
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
        quat = [(rotation[2, 1] - rotation[1, 2]) / s, 0.25 * s,
                (rotation[0, 1] + rotation[1, 0]) / s, (rotation[0, 2] + rotation[2, 0]) / s]
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
        quat = [(rotation[0, 2] - rotation[2, 0]) / s, (rotation[0, 1] + rotation[1, 0]) / s,
                0.25 * s, (rotation[1, 2] + rotation[2, 1]) / s]
    else:
        s = np.sqrt(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
        quat = [(rotation[1, 0] - rotation[0, 1]) / s, (rotation[0, 2] + rotation[2, 0]) / s,
                (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s]
    return [float(v) for v in quat]


def detections(results, shape, allowed=()):
    """Return label, confidence, image box, rotated rectangle and original-pixel contour."""
    out = []
    for result in results:
        boxes = getattr(result, 'boxes', None)
        if boxes is None or len(boxes) == 0:
            continue
        masks = getattr(result, 'masks', None)
        for i in range(len(boxes)):
            name = result.names.get(int(boxes.cls[i]), str(int(boxes.cls[i])))
            if allowed and name not in allowed:
                continue
            xyxy = boxes.xyxy[i].cpu().numpy().astype(float)
            conf = float(boxes.conf[i])
            contour = None
            if masks is not None and len(masks.xy) == len(boxes):
                polygon = np.asarray(masks.xy[i], np.float32)
                if (len(polygon) >= 3 and np.isfinite(polygon).all()
                        and cv2.contourArea(polygon) >= 8):
                    contour = polygon
            if contour is None:
                rect = np.array([[xyxy[0], xyxy[1]], [xyxy[2], xyxy[1]],
                                 [xyxy[2], xyxy[3]], [xyxy[0], xyxy[3]]], float)
            else:
                rect = cv2.boxPoints(cv2.minAreaRect(contour))
            out.append((name, conf, xyxy.astype(int), rect, contour))
    return out


def draw(image, name, conf, rect, grasp, valid):
    """Annotate one detection: box, short-axis line, and a label block that
    follows the box (above it, or below when too close to the top edge)."""
    color = (0, 255, 0) if valid else (0, 165, 255)
    cv2.polylines(image, [np.round(rect).astype(np.int32)], True, color, 2)
    if grasp is not None:
        vec = grasp['line_vector']
        center = np.array(grasp['center'], float)
        p0 = tuple(np.round(center - vec / 2).astype(int))
        p1 = tuple(np.round(center + vec / 2).astype(int))
        cv2.line(image, p0, p1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(image, grasp['center'], 4, (0, 0, 255), -1)
        x, y, z = grasp['position']
        lines = [f"{name} {conf:.2f} jaw={grasp['jaw_m']*100:.1f}cm",
                 f"cam({x:+.2f},{y:+.2f},{z:+.2f})m"]
    else:
        lines = [f"{name} {conf:.2f} no valid grasp geometry"]
    top = int(round(float(rect[:, 1].min())))
    left = int(round(float(rect[:, 0].min())))
    line_h, bg_w = 22, max(len(t) for t in lines) * 12
    if top >= line_h * (len(lines) + 1):
        y0, y1 = top - line_h * (len(lines) + 1), top
    else:  # Too close to the top edge: put the block below the box.
        bottom = int(round(float(rect[:, 1].max())))
        y0, y1 = bottom, bottom + line_h * (len(lines) + 1)
    cv2.rectangle(image, (left, y0), (left + bg_w, y1), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        cv2.putText(image, text, (left + 5, y1 - line_h * (len(lines) - i)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    color if i == 0 else (0, 255, 255), 2)


def draw_best(image, best):
    """One summary line for the current grasp target, bottom-left."""
    if best is None:
        cv2.putText(image, 'no valid grasp target', (10, image.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        return
    conf, name, rect, grasp = best
    x, y, z = grasp['position']
    cv2.putText(image, f'BEST {name} {conf:.2f}  jaw={grasp["jaw_m"]*100:.1f}cm  '
                f'cam({x:+.2f},{y:+.2f},{z:+.2f})m  [g] grasp',
                (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 255, 140), 2)


def plan_grasp(arm, object_pos, nominal, current_q, g):
    """Read-only IK preflight. Tilt about the jaw line so its direction is preserved."""
    jaw = nominal[:, 1 if g['open_axis'] == 'y' else 2]
    distance = g['pregrasp_offset_m']
    linear = g['final_motion'] == 'linear'
    intervals = max(1, int(np.ceil(distance / 0.01))) if linear else 1
    # Prefer the observed camera ray; relax pitch only when both wrist rolls fail.
    for magnitude in (0, 15, 30):
        candidates = []
        for tilt in ([0] if magnitude == 0 else [magnitude, -magnitude]):
            tilted = cv2.Rodrigues(jaw * np.deg2rad(tilt))[0] @ nominal
            for flipped in (False, True):
                rotation = tilted @ np.diag([1., -1., -1.]) if flipped else tilted
                quat = quat_from_matrix(rotation)
                end = object_pos - rotation[:, 0] * g['grasp_offset_m']
                start = end - rotation[:, 0] * distance
                if min(start[2], end[2]) < g['min_base_z_m']:
                    continue
                joints = []
                seed = np.asarray(current_q, float)
                for i, xyz in enumerate(np.linspace(start, end, intervals + 1)):
                    label = 'pregrasp' if i == 0 else ('grasp' if i == intervals else f'line[{i}]')
                    try:
                        reply = arm.call('ik', pose=dict(pos=xyz.tolist(), quat=quat), seed=seed.tolist())
                    except RobotFault as exc:
                        if not str(exc).startswith('ik no solution'):
                            raise
                        log(f'{label} IK rejected at {xyz.round(3)}, tilt={tilt}, flip={flipped}')
                        break
                    q = np.asarray(reply['q_deg'], float)
                    if q.shape != (6,) or not np.isfinite(q).all():
                        raise RobotFault('Invalid IK response')
                    if abs(q[4]) > 90 - g['j5_margin_deg']:
                        log(f'{label} J5={q[4]:.1f} outside configured margin')
                        break
                    if linear and i and np.max(np.abs(q-seed)) > 25:
                        log(f'{label} rejected: IK branch discontinuity')
                        break
                    joints.append(q)
                    seed = q
                else:
                    score = (max(abs(q[4]) for q in joints),
                             np.linalg.norm(joints[0]-current_q))
                    plan = dict(pre=start, end=end, quat=quat, pre_q=joints[0], end_q=joints[-1],
                                tilt=tilt, flipped=flipped, max_j5=score[0])
                    candidates.append((score, plan))
        if candidates:
            return min(candidates, key=lambda item: item[0])[1]
    return None


def traverse(arm, xyz, quat, expected_q, g):
    if g['final_motion'] == 'joint':
        arm.move(expected_q.tolist(), g['grasp_vel_dps'])
    else:
        arm.call('move_l', pose=dict(pos=xyz.tolist(), quat=quat),
                 vel=g['linear_vel_mps'], acc=g['linear_acc_mps2'], **{'async': True})
        arm.wait_still(expected_q)


def stop(arm, motion_sent, message, code):
    try:
        moving = arm.call('state')['state'].get('mode') != 'IDLE'
    except Exception:
        moving = True
    if motion_sent and moving:
        log(message + '; soft stop sent (clear with home)')
        try:
            arm.call('estop')
        except Exception:
            pass
    else:
        log(message)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).resolve().with_name('config') / 'default.yaml')
    parser.add_argument('--host', help='Override robot.host')
    parser.add_argument('--port', type=int, help='Override robot.port')
    parser.add_argument('--model', help='Override grasp.model YOLO weights')
    parser.add_argument('--dry-run', action='store_true',
                        help='Compute and print targets; do not move the arm')
    args = parser.parse_args(argv)
    cfg = load_config(args.config.resolve(), motion=False)
    if args.host:
        cfg['robot']['host'] = args.host
    if args.port:
        cfg['robot']['port'] = args.port
    g = {**DEFAULTS, **cfg.get('grasp', {})}
    g['device'] = resolve_device(g['device'])  # auto → 有 N 卡用 cuda:0，否则 cpu
    if args.model:
        g['model'] = args.model
    # 相对权重路径按标定根目录（models/）解析
    g['model'] = str(resolve_model(g['model'], args.config))
    if g['open_axis'] not in ('y', 'z'):
        raise ValueError('grasp.open_axis must be y or z')
    if g['approach'] not in ('top_down', 'camera_ray'):
        raise ValueError('grasp.approach must be top_down or camera_ray')
    if g['final_motion'] not in ('linear', 'joint'):
        raise ValueError('grasp.final_motion must be linear or joint')
    for key in ('linear_vel_mps', 'linear_acc_mps2', 'pregrasp_offset_m', 'grasp_vel_dps'):
        if not np.isfinite(g[key]) or g[key] <= 0:
            raise ValueError(f'grasp.{key} must be finite and positive')
    if not 0 <= g['j5_margin_deg'] < 90:
        raise ValueError('grasp.j5_margin_deg must be in [0, 90)')
    result_path = data_root(args.config) / cfg['output']['filename']
    with np.load(result_path, allow_pickle=False) as result:
        t_tcp_camera = result['T_tcp_camera'].copy()
    if not args.dry_run and not Path(str(g['model'])).is_file():
        raise ValueError(f"YOLO weights not found: {g['model']} (set grasp.model)")

    log('Loading YOLO ' + str(g['model']))
    from ultralytics import YOLO  # Heavy import stays behind the checks above.
    model = YOLO(str(g['model']))
    classes = [str(c) for c in (g.get('classes') or [])]
    if classes and ('world' in str(g['model']).lower() or 'yoloe' in str(g['model']).lower()):
        if ensure_text_encoder(args.config) is None:
            log('WARN: config/mobileclip2_b.ts 不存在，首次 set_classes 将下载 ~240MB')
        model.set_classes(classes)
        log('Open-vocabulary classes: ' + str(classes))
    elif classes:
        known = set(str(v) for v in getattr(model, 'names', {}).values())
        missing = [c for c in classes if c not in known]
        if missing:
            log(f'WARN: not in model vocabulary (never matches): {missing}')
        log('Restricted classes: ' + str(classes))

    import pyrealsense2 as rs
    pipeline, config = rs.pipeline(), rs.config()
    if cfg['camera']['serial']:
        config.enable_device(cfg['camera']['serial'])
    config.enable_stream(rs.stream.color, cfg['camera']['width'], cfg['camera']['height'],
                         rs.format.bgr8, cfg['camera']['fps'])
    config.enable_stream(rs.stream.depth, cfg['camera']['width'], cfg['camera']['height'],
                         rs.format.z16, cfg['camera']['fps'])
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    k = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.0]])
    scale = profile.get_device().first_depth_sensor().get_depth_scale()

    arm = RpcArm(cfg['robot'])
    robot = cfg['robot']
    motion_sent, grasped = False, False
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, cfg['camera']['width'], cfg['camera']['height'])
    try:
        arm.wait_still()
        if not args.dry_run:
            motion_sent = True
            log('Opening gripper and moving to ready pose')
            arm.call('gripper', mm=g['gripper_open_mm'])
            arm.move(robot['ready_pose_deg'], robot['approach_vel_dps'])
        log('Live view starting; the first YOLO inference can take tens of seconds on CPU')
        frame_index, latest, best = 0, None, None
        while True:
            frames = pipeline.wait_for_frames(3000)
            aligned = align.process(frames)
            color = np.asanyarray(aligned.get_color_frame().get_data()).copy()
            # depth_scale is meters per raw step; convert to millimeters once.
            depth_mm = np.asanyarray(aligned.get_depth_frame().get_data()) * scale * 1000.0
            frame_index += 1
            # Infer and annotate every 3rd capture; keep showing the last
            # annotated frame in between (mirrors reBot infer_every_live=3).
            if latest is None or frame_index % 3 == 0:
                results = model.predict(color, verbose=False, device=g['device'],
                                        conf=g['conf'], iou=g['iou'])
                grasps = []
                # 竖直下抓取物体区域里最近的一小撮深度=顶面：透明物体/托盘的红外穿透读数偏远，被自然排除
                quantile = g['top_depth_quantile'] if g['approach'] == 'top_down' else g['depth_quantile']
                for name, conf, _box, rect, contour in detections(results, color.shape, classes):
                    grasp = grasp_camera(rect, depth_mm, k, quantile, contour) if contour is not None else None
                    grasps.append((conf, name, rect, grasp))
                    draw(color, name, conf, rect, grasp, grasp is not None)
                valid = [item for item in grasps if item[3] is not None]
                best = max(valid, key=lambda item: item[0]) if valid else None
                latest = color
            status = ('LIVE' if best is not None else
                      'LIVE (no valid grasp yet)' if latest is not None else 'warming up')
            cv2.putText(latest, f'{status} | g grasp | q open+safe+quit', (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            draw_best(latest, best if latest is not None else None)
            cv2.imshow(WINDOW, latest)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                log('Opening gripper, returning to safe pose')
                if not args.dry_run:
                    arm.call('gripper', mm=g['gripper_open_mm'])
                    arm.move(robot['safe_pose_deg'], robot['approach_vel_dps'])
                break
            if key in (ord('g'), ord('G')) and best is not None:
                if grasped:
                    log('Gripper is holding the previous target; q releases it and returns to safe')
                    continue
                _, name, rect, grasp = best
                state = arm.state()
                if state['mode'] != 'IDLE' or state.get('degraded'):
                    log('Skipping frame: grasp planning requires stationary, healthy feedback')
                    continue
                q_meas = state['q_deg']
                t_base_tcp = pose_matrix(arm.call('fk', q=q_meas)['pose'])
                t_base_camera = t_base_tcp @ t_tcp_camera
                object_pos = (t_base_camera @ np.append(grasp['position'], 1.0))[:3]
                if g['approach'] == 'top_down':
                    nominal = top_down_rotation(grasp, t_base_camera, g['open_axis'])
                    if nominal is None:
                        log('Short side is not horizontal enough for a top-down grasp; '
                            'select again or use grasp.approach: camera_ray')
                        continue
                    # 检测点 = 物体顶面（近端深度分位）；目标降到物体中线高度，且不低于支撑面 + 半爪宽
                    object_pos, floored = top_down_target(object_pos, grasp['jaw_m'], g['support_z_m'])
                    if grasp.get('ends_px') is not None:
                        mid = axis_midpoint_xy(grasp['ends_px'], k, t_base_camera, object_pos[2])
                        if mid is not None:
                            shift = np.linalg.norm(mid - object_pos[:2])
                            object_pos[:2] = mid
                            log(f'Grasp point moved {shift*100:.1f}cm to the 3D midpoint of the long axis')
                    if floored:
                        log(f'Depth top looks too low (transparent object?); grasp height '
                            f'raised to support_z_m + jaw/2 = {object_pos[2]:.3f} m')
                else:
                    nominal = tcp_rotation(grasp, t_base_camera, g['open_axis'])
                plan = plan_grasp(arm, object_pos, nominal, q_meas, g)
                if plan is None:
                    log('No feasible grasp for this frame; waiting for another selection')
                    continue
                pos, quat = plan['end'], plan['quat']
                log(f"Pregrasp {plan['pre'].round(3)}, tilt={plan['tilt']} deg, "
                    f"flip={plan['flipped']}, max sampled |J5|={plan['max_j5']:.1f} deg, "
                    f"final motion={g['final_motion']}")
                close_mm = close_target_mm(grasp['jaw_m'], g['gripper_close_factor'],
                                            g['gripper_close_min_mm'], g['gripper_open_mm'])
                log(f'Grasp {name}: base xyz ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) '
                    f'jaw {grasp["jaw_m"]*100:.1f}cm -> close {close_mm:.1f}mm '
                    f'(offset {g["grasp_offset_m"]*100:.1f}cm) '
                    f'quat {np.round(quat, 3).tolist()}')
                if args.dry_run:
                    continue
                # The blocking sequence below naturally freezes the window on
                # the frame captured at key-press time.
                arm.move(plan['pre_q'].tolist(), g['grasp_vel_dps'])
                traverse(arm, pos, quat, plan['end_q'], g)
                for _ in range(int(g['wait_before_grip_s'] * 10)):
                    cv2.waitKey(100)  # Keep the window alive during the settle wait.
                close_and_lock(arm, close_mm, g)
                log('Retreating to pregrasp before returning to ready pose')
                traverse(arm, plan['pre'], quat, plan['pre_q'], g)
                arm.move(robot['ready_pose_deg'], robot['approach_vel_dps'])
                grasped = True
                latest, best = None, None  # Re-annotate from the new scene.
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        return stop(arm, motion_sent, 'Interrupted', 130)
    except RobotFault as exc:
        return stop(arm, motion_sent, f'FAILED: {exc}', 2)
    finally:
        pipeline.stop()
        arm.close()
        cv2.destroyAllWindows()
    log('Finished' + (' (grasped at least once)' if grasped else ''))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
