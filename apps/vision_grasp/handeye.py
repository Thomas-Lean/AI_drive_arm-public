#!/usr/bin/env python3
"""手眼标定入口（eye-in-hand）：D1 + RealSense D435i 末端相机标定。

用法：
  python handeye.py --check-config                 # 仅校验配置（不连设备）
  python handeye.py --solve <samples.npz>          # 离线重算（不连设备）
  python handeye.py                                # 真机采集 + 求解

流程：相机先就绪（含预热与畸变检查）→ 低速到 ready → 逐采集点
（到位 + 稳定窗 + 丢帧后采图，多帧一致性/漂移/重复点判据）→ 质量门
（多样性 + 残差 + 1/4 留出验证 + 早停）→ 回 safe 后才最终求解并原子写
hand_eye.npz。详见 README。
通信与几何实现拆分在 utils/（robot/camera/core），此处保留再导出以兼容
scripts/ 与测试的既有 `from handeye import ...` 用法。
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

from utils.core import EarlyStop, MarkerDetector, assess, diversity, mean_transform, pose_matrix, spread
from utils.robot import RobotFault, RpcArm, log  # 再导出供 scripts/ 与测试引用
from utils.camera import Camera
from utils.paths import data_root


def load_config(path, motion=True):
    if not hasattr(cv2, 'calibrateHandEye'):
        raise ValueError('OpenCV wheel lacks calibrateHandEye; install requirements.txt (OpenCV 4.x)')
    cfg = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(cfg, dict):
        raise ValueError('Config must be a YAML mapping')
    for section in ['board', 'robot', 'camera', 'collection', 'quality', 'output']:
        if not isinstance(cfg.get(section), dict):
            raise ValueError('Missing mapping: ' + section)
    b, r, cam, c, q = [cfg[k] for k in ['board','robot','camera','collection','quality']]
    if b.get('type') != 'aruco_single':
        raise ValueError('This implementation supports board.type: aruco_single')
    if b.get('kind', 'aruco') not in ('aruco', 'charuco'):
        raise ValueError('board.kind must be aruco or charuco')
    if not isinstance(b.get('dictionary'), str) or not b['dictionary'].startswith('DICT_'):
        raise ValueError('Invalid ArUco dictionary')
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, b['dictionary']))
    if b.get('kind', 'aruco') == 'aruco':
        if type(b.get('marker_id')) is not int or not 0 <= b['marker_id'] < len(dictionary.bytesList):
            raise ValueError('Invalid marker_id')
    else:  # charuco:marker_id 不用;校验棋盘几何
        for key in ('squares_x', 'squares_y'):
            if type(b.get(key)) is not int or b[key] < 3:
                raise ValueError(f'board.{key} must be an integer >= 3')
        square = b.get('square_length_m')
        if (isinstance(square, bool) or not isinstance(square, (int, float))
                or not np.isfinite(square) or square <= 0):
            raise ValueError('board.square_length_m must be a finite positive number')
        if not 0 < b['marker_length_m'] < square:
            raise ValueError('ChArUco requires 0 < marker_length_m < square_length_m')
    positive = {'board': ['marker_length_m','min_side_px','max_reprojection_px'],
                'robot': ['port','request_timeout_s','move_timeout_s','vel_dps','acc_dps2',
                          'approach_vel_dps','offline_grace_s',
                          'settle_s','poll_s','arrival_tolerance_deg','stable_tolerance_deg'],
                'camera': ['width','height','fps','warmup_frames','frame_timeout_ms'],
                'collection': ['max_pose_attempts','capture_frames','discard_frames',
                               'max_capture_drift_deg','min_pose_rotation_deg','min_pose_translation_m',
                               'min_detected_frames','max_marker_translation_spread_m','max_marker_rotation_spread_deg'],
                'quality': ['min_samples','min_rotation_span_deg','min_translation_span_m','min_axis_ratio',
                            'max_translation_rms_m','max_rotation_rms_deg','max_translation_error_m',
                            'max_rotation_error_deg','solution_change_m','solution_change_deg','consecutive_passes']}
    for section, names in positive.items():
        for name in names:
            v = cfg[section].get(name)
            if isinstance(v, bool) or not isinstance(v, (int,float)) or not np.isfinite(v) or v <= 0:
                raise ValueError(f'{section}.{name} must be a finite positive number')
    for name in ['edge_margin_px','min_pnp_error_gap_px']:
        v = b.get(name)
        if isinstance(v, bool) or not isinstance(v,(int,float)) or not np.isfinite(v) or v < 0:
            raise ValueError('Invalid board.' + name)
    for section, names in {'board':['border_bits'], 'robot':['port'],
                           'camera':['width','height','fps','warmup_frames','frame_timeout_ms'],
                           'collection':['max_pose_attempts','capture_frames','discard_frames','min_detected_frames'],
                           'quality':['min_samples','consecutive_passes']}.items():
        for name in names:
            if type(cfg[section].get(name)) is not int or cfg[section][name] < 1:
                raise ValueError(f'{section}.{name} must be a positive integer')
    if not 8 <= q['min_samples'] <= c['max_pose_attempts'] <= 50:
        raise ValueError('Require 8 <= min_samples <= max_pose_attempts <= 50')
    if q['consecutive_passes'] < 2 or q['min_axis_ratio'] > 1:
        raise ValueError('Require consecutive_passes >= 2 and min_axis_ratio <= 1')
    if not 2 <= c['min_detected_frames'] <= c['capture_frames']:
        raise ValueError('Require 2 <= min_detected_frames <= capture_frames')
    if q['method'] not in ['PARK','TSAI','HORAUD']:
        raise ValueError('Unsupported hand-eye method')
    if r['port'] > 65535 or r['vel_dps'] > 100 or r['acc_dps2'] > 500:
        raise ValueError('Invalid robot port or velocity/acceleration above SDK baseline limits')
    if r['approach_vel_dps'] > r['vel_dps']:
        raise ValueError('approach_vel_dps must not exceed collection vel_dps')
    if r['settle_s'] < 1 or r['poll_s'] > r['settle_s']/3:
        raise ValueError('settle_s >= 1 and poll_s <= settle_s/3 required for sequential SDK telemetry')
    if not isinstance(r.get('host'), str) or not r['host']:
        raise ValueError('robot.host required')
    if not isinstance(cam.get('serial'), str):
        raise ValueError('camera.serial must be a string (empty selects the default device)')
    if 'preview' in cam and not isinstance(cam['preview'], bool):
        raise ValueError('camera.preview must be true or false')
    for name in ['filename','session_dir']:
        v = cfg['output'].get(name)
        if not isinstance(v,str) or not v or v in ['.','..'] or '/' in v or '\\' in v:
            raise ValueError('output paths must be simple names within the config directory')
    if not cfg['output']['filename'].endswith('.npz'):
        raise ValueError('output.filename must end in .npz')
    if b.get('kind', 'aruco') == 'charuco' and 'charuco' not in cfg['output']['filename']:
        # 板型入文件名:两种板各存各的,互不覆盖;yolo_detect/grasp_yolo 读同一配置键,自动跟随
        cfg['output']['filename'] = cfg['output']['filename'][:-len('.npz')] + '_charuco.npz'
    if motion:
        poses = r.get('poses_deg')
        if not isinstance(poses, list) or not poses:
            raise ValueError('robot.poses_deg must list at least one user-defined pose')
        if c['max_pose_attempts'] > len(poses):
            raise ValueError(f"max_pose_attempts {c['max_pose_attempts']} exceeds the {len(poses)} filled poses")
        limits = np.array([135,90,90,135,90,135])
        for label, pose in [('ready_pose_deg',r.get('ready_pose_deg')),
                            ('safe_pose_deg',r.get('safe_pose_deg'))] + list(enumerate(poses, 1)):
            if (not isinstance(pose,list) or len(pose)!=6 or
                    any(isinstance(v,bool) or not isinstance(v,(int,float)) for v in pose)):
                raise ValueError(f'Fill robot pose {label}: six joint angles in degrees')
            a = np.array(pose, float)
            if not np.isfinite(a).all() or np.any(np.abs(a)>limits):
                raise ValueError(f'Pose {label} exceeds URDF joint limits or is nonfinite')
    return cfg


def save_npz(path, **arrays):
    temporary = path.with_suffix('.tmp')
    with temporary.open('wb') as stream:
        np.savez_compressed(stream, **arrays)
    temporary.replace(path)


def dataset_arrays(samples, k, dist, cfg, metadata):
    return dict(T_base_tcp=np.array([s['bg'] for s in samples]).reshape(-1,4,4),
                T_camera_target=np.array([s['ct'] for s in samples]).reshape(-1,4,4),
                q_deg=np.array([s['q'] for s in samples]).reshape(-1,6),
                pose_indices=np.array([s['index'] for s in samples],dtype=int),
                reprojection_px=np.array([s['error'] for s in samples]),
                image_paths=np.array([s['image'] for s in samples],dtype=str),
                camera_matrix=k, distortion_coefficients=dist,
                config_yaml=np.array(yaml.safe_dump(cfg, allow_unicode=True)),
                camera_metadata_json=np.array(json.dumps(metadata, ensure_ascii=False)),
                frame_convention=np.array('T_A_B maps B to A; distances m; robot reference is SDK TCP; camera is color optical'))


def finish(arrays, cfg, output, provenance):
    bg, ct = arrays['T_base_tcp'], arrays['T_camera_target']
    x, metrics, reason = assess(bg, ct, cfg['quality'])
    if x is None:
        raise ValueError('Calibration NOT qualified: '+reason+'; '+json.dumps(metrics))
    result = dict(arrays)
    result.update(T_tcp_camera=x, T_camera_tcp=np.linalg.inv(x),
                  T_base_target=mean_transform(bg @ x @ ct),
                  quality_json=np.array(json.dumps(metrics)),
                  result_config_yaml=np.array(yaml.safe_dump(cfg, allow_unicode=True)),
                  T_result=x, mode=np.array(['eye_in_hand']), n_samples=np.array([len(bg)]),
                  method=np.array([cfg['quality']['method']]),
                  created_utc=np.array(datetime.now(timezone.utc).isoformat()),
                  provenance=np.array(provenance))
    save_npz(output, **result)
    print('\nT_tcp_camera (camera color optical -> SDK TCP; translation in meters):')
    print(np.array2string(x, precision=9, suppress_small=True))
    print('\nT_camera_tcp (inverse):')
    print(np.array2string(np.linalg.inv(x), precision=9, suppress_small=True))
    print('\nQuality:', json.dumps(metrics, indent=2))
    log('Saved: '+str(output))
    return x


def preview_frame(image, lines, corners=None):
    """Show one annotated capture frame; False when no GUI is available."""
    try:
        canvas = image.copy()
        if corners is not None:
            cv2.polylines(canvas, [np.int32(corners).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        y = 26
        for line in lines:
            cv2.putText(canvas, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)
            y += 26
        cv2.imshow('hand-eye capture preview (passive)', canvas)
        cv2.waitKey(1)
        return True
    except cv2.error:
        return False


def collect(cfg, arm, camera, session):
    """Normal completion goes to safe pose BEFORE final solve; caller handles aborts."""
    samples, attempts = [], []
    detector, early = MarkerDetector(cfg['board']), EarlyStop(cfg['quality'])
    preview = bool(cfg['camera'].get('preview'))

    def show(image, lines, corners=None):
        nonlocal preview
        if not preview:
            return
        if not preview_frame(image, lines, corners):
            preview = False
            log('Preview disabled: no display available; continuing headless')

    session.mkdir(parents=True, exist_ok=False)
    (session/'config.yaml').write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding='utf-8')

    def persist():
        arrays = dataset_arrays(samples, camera.k, camera.dist, cfg, camera.metadata)
        save_npz(session/'samples.npz', **arrays)
        (session/'attempts.json').write_text(json.dumps(attempts, indent=2, ensure_ascii=False), encoding='utf-8')
        return arrays

    persist()
    arm.wait_still()
    log('Moving to ready pose (approach speed)')
    arm.move(cfg['robot']['ready_pose_deg'],
             cfg['robot']['approach_vel_dps'])
    targets = cfg['robot']['poses_deg'][:cfg['collection']['max_pose_attempts']]
    for index, target in enumerate(targets, 1):
        log(f'Pose {index}/{len(targets)}; accepted {len(samples)}')
        arm.move(target)
        # Drop queued/exposure-transition frames after the settling window.
        for _ in range(cfg['collection']['discard_frames']):
            discarded, _ = camera.frame()
            show(discarded, [f'pose {index}/{len(targets)}  settling, discarding frames'])
        before = arm.state()
        best = None
        detected = []
        for frame_index in range(cfg['collection']['capture_frames']):
            image, info = camera.frame()
            image_name = f'pose_{index:02d}_{frame_index:02d}.png'
            if not cv2.imwrite(str(session/image_name), image):
                raise OSError('Cannot write captured image')
            record = dict(index=index, frame=info, image=image_name, before=before)
            attempts.append(record)
            status, preview_corners = '', None
            try:
                ct, error, uv = detector.detect(image, camera.k, camera.dist)
                detected.append(ct)
                record.update(detected=True, reprojection_px=error, corners=uv.tolist())
                if best is None or error < best[0]:
                    best = (error, ct, image_name)
                status, preview_corners = f'marker OK, reproj {error:.2f} px', uv
            except ValueError as exc:
                record.update(detected=False, reason=str(exc))
                status = 'marker: ' + str(exc)
            show(image, [f'pose {index}/{len(targets)}   accepted so far {len(samples)}',
                         status], preview_corners)
        after = arm.state()
        drift = float(np.max(np.abs(np.array(after['q_deg'])-before['q_deg'])))
        for record in attempts[-cfg['collection']['capture_frames']:]:
            record.update(after=after, drift_deg=drift, sample_accepted=False)
        if before['mode'] != 'IDLE' or after['mode'] != 'IDLE':
            raise RobotFault('Robot moved during capture (another controller may be active)')
        if before.get('degraded') or after.get('degraded'):
            log('Rejected: telemetry degraded during capture')
            attempts[-1]['sample_rejection'] = 'telemetry degraded'
        elif drift > cfg['collection']['max_capture_drift_deg']:
            log('Rejected: robot drifted during capture')
            attempts[-1]['sample_rejection'] = 'robot drift'
        elif len(detected) < cfg['collection']['min_detected_frames']:
            log('Rejected: too few unambiguous marker frames; see attempts.json')
            attempts[-1]['sample_rejection'] = 'too few marker frames'
        else:
            dt, dr = spread(np.array(detected), mean_transform(np.array(detected)))
            if (max(dt) > cfg['collection']['max_marker_translation_spread_m'] or
                    max(dr) > cfg['collection']['max_marker_rotation_spread_deg']):
                log('Rejected: marker pose unstable over captured frames')
                attempts[-1]['sample_rejection'] = 'marker pose unstable'
                persist()
                continue
            # Use measured joints, never commanded joints, and server FK for this exact vector.
            q = ((np.array(before['q_deg'])+after['q_deg'])/2).tolist()
            bg = pose_matrix(arm.call('fk', q=q)['pose'])
            duplicate = any(
                np.linalg.norm(bg[:3,3]-s['bg'][:3,3]) < cfg['collection']['min_pose_translation_m'] and
                diversity(np.array([bg,s['bg']]))['rotation_span_deg'] < cfg['collection']['min_pose_rotation_deg']
                for s in samples)
            if duplicate:
                log('Rejected: pose too similar to an accepted sample')
                attempts[-1]['sample_rejection'] = 'duplicate pose'
            else:
                error, ct, image_name = best
                samples.append(dict(index=index,q=q,bg=bg,ct=ct,error=error,image=image_name))
                for record in attempts[-cfg['collection']['capture_frames']:]:
                    record['sample_accepted'] = record['image'] == image_name
        arrays = persist()
        # Only a NEW sample can advance the early-stop streak.
        if samples and samples[-1]['index'] == index:
            x, metrics, reason = assess(arrays['T_base_tcp'],arrays['T_camera_target'],cfg['quality'])
            log(f'Quality check: {reason}; '+json.dumps(metrics))
            if early.update(x):
                log('Early-stop criteria satisfied')
                break
    log('Collection finished; moving to safe pose before final calculation')
    arm.move(cfg['robot']['safe_pose_deg'],
             cfg['robot']['approach_vel_dps'])
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass
    return persist()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=Path(__file__).with_name('config') / 'default.yaml')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-config', action='store_true', help='Validate without robot/camera access')
    mode.add_argument('--solve', type=Path, metavar='SAMPLES_NPZ', help='Offline solve; no robot/camera access')
    args = parser.parse_args(argv)
    path = args.config.resolve()
    arm = camera = None
    motion_started = False
    safe_reached = False
    cfg = None
    try:
        cfg = load_config(path, motion=not bool(args.solve))
        if args.check_config:
            log(f"Configuration valid; {min(len(cfg['robot']['poses_deg']), cfg['collection']['max_pose_attempts'])} "
                'poses checked. No connections or motion performed.')
            return 0
        output = data_root(path)/cfg['output']['filename']  # 结果固定落标定根目录
        if args.solve:
            with np.load(args.solve, allow_pickle=False) as data:
                arrays = {key:data[key].copy() for key in data.files}
            old = yaml.safe_load(str(arrays['config_yaml']))
            for key in ['type','dictionary','marker_id','marker_length_m','border_bits']:
                if old['board'][key] != cfg['board'][key]:
                    raise ValueError('Board config differs from recorded samples; recollect/re-detect images')
            bg, ct = arrays['T_base_tcp'], arrays['T_camera_target']
            if bg.shape != ct.shape or bg.ndim != 3 or bg.shape[1:] != (4,4) or not np.isfinite([bg,ct]).all():
                raise ValueError('Malformed transform dataset')
            finish(arrays, cfg, output, 'offline: '+str(args.solve.resolve()))
            return 0
        # Camera and marker configuration must work before any motion is requested.
        camera = Camera(cfg['camera'])
        arm = RpcArm(cfg['robot'])
        session = data_root(path)/cfg['output']['session_dir']/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        log('Session: '+str(session))
        # Freshly restarted d1armd is required; see SDK limitations in README.
        arm.wait_still()
        motion_started = True
        arrays = collect(cfg,arm,camera,session)
        safe_reached = True
        camera.close()
        finish(arrays,cfg,output,'live session: '+str(session))
        return 0
    except KeyboardInterrupt:
        log('Interrupted: stopping; no automatic travel to safe pose')
        return 130
    except Exception as exc:
        log(f'FAILED: {exc}')
        # Acquisition/storage errors may return to safe only while the robot is healthy/idle.
        if arm is not None and motion_started and not safe_reached and not isinstance(exc,RobotFault):
            try:
                s = arm.state()
                if s['mode'] == 'IDLE':
                    log('Acquisition failed; returning healthy robot to configured safe pose')
                    arm.move(cfg['robot']['safe_pose_deg'],
                             cfg['robot']['approach_vel_dps'])
                    safe_reached = True
            except Exception as recovery:
                log('Safe-pose recovery failed: '+str(recovery))
        return 2
    finally:
        if arm is not None:
            if motion_started and not safe_reached:
                try:
                    if getattr(arm, 'broken', False):
                        # Separate connection avoids mistaking a delayed prior reply for stop ACK.
                        stop_arm = RpcArm(cfg['robot'])
                        try:
                            stop_arm.call('estop')
                        finally:
                            stop_arm.close()
                    else:
                        arm.call('estop')
                except Exception as exc:
                    log('Could not deliver soft stop: '+str(exc))
            arm.close()
        if camera is not None:
            try:
                camera.close()
            except Exception as exc:
                log('Camera cleanup failed: '+str(exc))


if __name__ == '__main__':
    sys.exit(main())
