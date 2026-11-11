"""No robot/camera access: python -m pytest apps/vision_grasp/tests -q"""
import contextlib
import copy
import io
import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]          # 标定根目录
sys.path.insert(0, str(_ROOT))
_SDK = str(_ROOT.parents[1] / 'sdk')                 # 仓库根/sdk（d1t_sdk；同 utils/robot.py 的引导）
if _SDK not in sys.path:
    sys.path.insert(0, _SDK)
from d1t_sdk import D1ArmError
from utils import core
import handeye as app
import grasp_yolo as grasp
from utils import robot as app_robot


def synthetic(n=50):
    rng = np.random.default_rng(501)
    x = core.transform(cv2.Rodrigues(np.array([0.25,-0.16,0.13]))[0], [0.04,-0.03,0.08])
    fixed = core.transform(cv2.Rodrigues(np.array([0.2,0.5,-0.1]))[0], [0.4,0.1,0.2])
    bg = np.array([core.transform(cv2.Rodrigues(rng.uniform(-0.65,0.65,3))[0],
                                 rng.uniform([0.1,-0.2,0.2],[0.4,0.2,0.5])) for _ in range(n)])
    ct = np.array([np.linalg.inv(x) @ np.linalg.inv(b) @ fixed for b in bg])
    return x, bg, ct


def as_pose(t):
    rv = cv2.Rodrigues(t[:3,:3])[0].ravel()
    a = np.linalg.norm(rv)
    xyz = rv/a*np.sin(a/2) if a > 1e-12 else np.zeros(3)
    return dict(pos=t[:3,3].tolist(), quat=[float(np.cos(a/2)), *xyz.tolist()])


def charuco_scene(k, rvec, tvec, size=(640, 480)):
    """5x5 ChArUco 板按已知位姿渲染成灰度图(单应变换,无畸变)。"""
    board = cv2.aruco.CharucoBoard((5, 5), 0.030, 0.022,
                                   cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    tile = board.generateImage((500, 500), 0, 1)
    side = 5 * 0.030
    quad = np.array([[0, 0, 0], [side, 0, 0], [side, side, 0], [0, side, 0]], np.float64)
    proj, _ = cv2.projectPoints(quad, np.asarray(rvec), np.asarray(tvec), k, np.zeros(5))
    homography, _ = cv2.findHomography(
        np.array([[0, 0], [500, 0], [500, 500], [0, 500]], np.float64), proj.reshape(-1, 2))
    return cv2.warpPerspective(tile, homography, size, flags=cv2.INTER_CUBIC, borderValue=255)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = app.load_config(Path(__file__).resolve().parents[1] / 'config' / 'default.yaml', motion=False)
        self.x,self.bg,self.ct = synthetic()

    def test_known_transform_and_direction(self):
        x, metrics, reason = core.assess(self.bg,self.ct,self.cfg['quality'])
        self.assertEqual(reason,'qualified')
        np.testing.assert_allclose(x,self.x,atol=1e-8)
        self.assertLess(metrics['holdout_translation_max_m'],1e-8)
        np.testing.assert_allclose(core.pose_matrix(as_pose(x)),x,atol=1e-8)

    def test_noise_tolerance(self):
        rng = np.random.default_rng(12)
        noisy = self.ct.copy()
        noisy[:,:3,3] += rng.normal(0,0.0002,(50,3))
        x,_,reason = core.assess(self.bg,noisy,self.cfg['quality'])
        self.assertEqual(reason,'qualified')
        self.assertLess(np.linalg.norm(x[:3,3]-self.x[:3,3]),0.002)

    def test_bad_sample_rejected(self):
        broken = self.ct.copy()
        broken[3,:3,3] += [0.08,0,0]
        self.assertIsNone(core.assess(self.bg,broken,self.cfg['quality'])[0])

    def test_single_axis_degenerate(self):
        bg = np.array([core.transform(cv2.Rodrigues(np.array([0.,0.,a]))[0],[a,0,0])
                       for a in np.linspace(-0.5,0.5,20)])
        self.assertIsNone(core.assess(bg,self.ct[:20],self.cfg['quality'])[0])

    def test_insufficient_data(self):
        self.assertIsNone(core.assess(self.bg[:3],self.ct[:3],self.cfg['quality'])[0])

    def test_nonrigid_input_rejected(self):
        bad=self.bg.copy()
        bad[0,0,0]+=0.5
        with self.assertRaises(ValueError):
            core.assess(bad,self.ct,self.cfg['quality'])

    def test_early_stop_requires_consecutive_stability(self):
        stop = core.EarlyStop(self.cfg['quality'])
        self.assertFalse(stop.update(self.x))
        self.assertFalse(stop.update(None))
        self.assertFalse(stop.update(self.x))
        self.assertFalse(stop.update(self.x))
        self.assertTrue(stop.update(self.x))

    def test_pose_list_length_rules(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'default.yaml'
            cfg=copy.deepcopy(self.cfg)
            cfg['robot'].update(ready_pose_deg=[0]*6,safe_pose_deg=[0]*6)
            cfg['quality'].update(min_samples=8)
            cfg['robot']['poses_deg']=[None]*8
            cfg['collection']['max_pose_attempts']=8
            path.write_text(yaml.safe_dump(cfg),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'Fill robot pose'):
                app.load_config(path)
            cfg['robot']['poses_deg']=[[0]*6 for _ in range(10)]
            cfg['collection']['max_pose_attempts']=50
            path.write_text(yaml.safe_dump(cfg),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'max_pose_attempts'):
                app.load_config(path)
            cfg['collection']['max_pose_attempts']=10
            path.write_text(yaml.safe_dump(cfg),encoding='utf-8')
            self.assertEqual(len(app.load_config(path)['robot']['poses_deg']),10)

    def test_config_numeric_and_pose_validation(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'default.yaml'
            cfg = copy.deepcopy(self.cfg)
            cfg['robot'].update(ready_pose_deg=[0]*6,safe_pose_deg=[0]*6,poses_deg=[[0]*6 for _ in range(50)])
            for section,key,value in [('robot','vel_dps',0),('robot','settle_s',0.2),
                                       ('quality','min_samples',3),('output','filename','../bad.npz'),
                                       ('board','marker_id',9999)]:
                bad = copy.deepcopy(cfg)
                bad[section][key] = value
                path.write_text(yaml.safe_dump(bad),encoding='utf-8')
                with self.assertRaises(ValueError):
                    app.load_config(path)
            path.write_text(yaml.safe_dump(cfg),encoding='utf-8')
            self.assertEqual(len(app.load_config(path)['robot']['poses_deg']),50)

    def test_charuco_detector_recovers_known_pose(self):
        b = copy.deepcopy(self.cfg['board'])
        b.update(kind='charuco', squares_x=5, squares_y=5,
                 square_length_m=0.030, marker_length_m=0.022)
        k = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1.]])
        rvec = np.array([0.15, -0.10, 0.05])
        tvec = np.array([0.02, -0.01, 0.35])
        image = charuco_scene(k, rvec, tvec)
        t, err, uv = core.MarkerDetector(b).detect(image, k, np.zeros(5))
        self.assertLess(err, 1.0)                              # 重投影上限同单 ArUco
        # 位置 ≤3mm:合成图经单应重采样,角点亚像素地板 ~2.7mm@0.35m,再紧会误报
        self.assertLess(float(np.linalg.norm(t[:3, 3] - tvec)), 0.003)
        rot_err = core.angle_deg(cv2.Rodrigues(rvec)[0].T @ t[:3, :3])
        self.assertLess(rot_err, 0.5)                          # 姿态 ≤0.5°
        self.assertGreaterEqual(len(uv), 6)                    # 返回的角点用于预览
        occluded = image.copy()
        occluded[:, :500] = 255                                # 只剩右侧一条 → 角点不足
        with self.assertRaisesRegex(ValueError, 'not visible enough'):
            core.MarkerDetector(b).detect(occluded, k, np.zeros(5))

    def test_config_board_kind_charuco_rules_and_filename(self):
        base = copy.deepcopy(self.cfg)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'default.yaml'
            def load(board):
                cfg = copy.deepcopy(base)
                cfg['board'] = board
                path.write_text(yaml.safe_dump(cfg), encoding='utf-8')
                return app.load_config(path, motion=False)
            bad = copy.deepcopy(base['board']); bad['kind'] = 'april'
            with self.assertRaisesRegex(ValueError, 'board.kind'):
                load(bad)
            incomplete = copy.deepcopy(base['board'])
            incomplete.update(kind='charuco', square_length_m=0.030, marker_length_m=0.022)
            incomplete.pop('squares_x')                        # default.yaml 自带,显式删掉再测
            with self.assertRaisesRegex(ValueError, 'squares_x'):
                load(incomplete)
            bad_ratio = copy.deepcopy(base['board'])
            bad_ratio.update(kind='charuco', squares_x=5, squares_y=5,
                             square_length_m=0.030, marker_length_m=0.030)
            with self.assertRaisesRegex(ValueError, 'marker_length_m'):
                load(bad_ratio)
            good = copy.deepcopy(base['board'])
            good.update(kind='charuco', squares_x=5, squares_y=5,
                        square_length_m=0.030, marker_length_m=0.022, marker_id=9999)
            cfg = load(good)                                   # marker_id 对 charuco 无意义,不校验
            self.assertEqual(cfg['output']['filename'], 'hand_eye_charuco.npz')
            self.assertEqual(load(copy.deepcopy(base['board']))['output']['filename'],
                             'hand_eye.npz')                   # 默认 aruco 不加后缀

    def test_detector_generated_marker(self):
        b = copy.deepcopy(self.cfg['board'])
        # A planar frontal marker is intentionally ambiguous; test decode/PnP with gap disabled.
        b['min_pnp_error_gap_px'] = 0
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),0,300)
        image = np.full((480,640),255,np.uint8)
        image[80:380,170:470] = marker
        k = np.array([[600.,0,320],[0,600.,240],[0,0,1.]])
        t,err,_ = core.MarkerDetector(b).detect(image,k,np.zeros(5))
        self.assertLess(err,1)
        self.assertAlmostEqual(t[2,3],0.2,delta=0.003)
        with self.assertRaises(ValueError):
            core.MarkerDetector(b).detect(np.full_like(image,255),k,np.zeros(5))

    def test_save_offline_and_reference_compatibility(self):
        with tempfile.TemporaryDirectory() as td, contextlib.redirect_stdout(io.StringIO()):
            root=Path(td)
            config=root/'default.yaml'
            config.write_text(yaml.safe_dump(self.cfg),encoding='utf-8')
            samples=[dict(bg=b,ct=c,q=[0]*6,index=i,error=0.,image=f'{i}.png')
                     for i,(b,c) in enumerate(zip(self.bg,self.ct))]
            arrays=app.dataset_arrays(samples,np.eye(3),np.zeros(5),self.cfg,{'serial':'test'})
            app.save_npz(root/'samples.npz',**arrays)
            self.assertEqual(app.main(['--config',str(config),'--solve',str(root/'samples.npz')]),0)
            with np.load(root/'hand_eye.npz',allow_pickle=False) as result:
                np.testing.assert_allclose(result['T_result'],self.x,atol=1e-8)
                self.assertEqual(result['mode'][0],'eye_in_hand')
                self.assertEqual(result['n_samples'][0],50)
                np.testing.assert_allclose(result['T_camera_tcp']@result['T_tcp_camera'],np.eye(4),atol=1e-8)

    def test_failed_solve_keeps_previous_result(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'result.npz'
            path.write_bytes(b'previous calibration')
            with self.assertRaises(ValueError):
                app.finish(dict(T_base_tcp=self.bg[:2],T_camera_target=self.ct[:2]),self.cfg,path,'test')
            self.assertEqual(path.read_bytes(),b'previous calibration')

    def test_rpc_state_maps_d1t_modes(self):
        class FakeArm:
            def __init__(self, mode, fault=0):
                self.mode, self.fault, self.n = mode, fault, 0
            def state(self, timeout_s=None):
                self.n += 1
                return dict(q_deg=[0.0]*6, online=[True]*7, stamp_ms=self.n,
                            mode=self.mode, fault=self.fault)
            def close(self): pass
        def payload(mode, fault=0):
            arm = app.RpcArm(self.cfg['robot'], arm=FakeArm(mode, fault))
            return arm.call('state')['state']
        self.assertEqual(payload('IDLE')['mode'], 'IDLE')
        self.assertEqual(payload('HOLD')['mode'], 'IDLE')     # 控制器保持=静止
        self.assertEqual(payload('MOVING')['mode'], 'MOVING')
        self.assertEqual(payload('STREAMING')['mode'], 'MOVING')
        s = payload('ESTOP')
        self.assertEqual((s['mode'], s['err']), ('IDLE', 'estop'))  # d1armd 锁存口径
        s = payload('FAULT', fault=0x10)
        self.assertEqual(s['mode'], 'IDLE')
        self.assertIn('0x10', s['err'])
        self.assertEqual(payload('INIT')['mode'], 'INIT')     # 透传，state() 会拒收

    def test_rpc_link_failure_invalidates_connection(self):
        class DeadArm:
            def state(self, timeout_s=None):
                raise D1ArmError('5s 内没收到 ArmState（控制器没起？）')
            def close(self): pass
        arm = app.RpcArm(self.cfg['robot'], arm=DeadArm())
        with self.assertRaises(app.RobotFault):
            arm.call('state')
        self.assertTrue(arm.broken)
        with self.assertRaisesRegex(app.RobotFault, 'unusable'):
            arm.call('state')

    def test_rpc_self_estop_auto_clears_before_motion(self):
        class LatchedArm:
            def __init__(self):
                self.calls, self.mode = [], 'HOLD'
            def state(self, timeout_s=None):
                return dict(q_deg=[0.0]*6, online=[True]*7, stamp_ms=1,
                            mode=self.mode, fault=0)
            def estop(self):
                self.calls.append('estop'); self.mode = 'ESTOP'
                return {'ok': True}
            def reset(self):
                self.calls.append('reset'); self.mode = 'HOLD'
                return {'ok': True}
            def move_joints(self, q, vel_dps=80.0, acc_dps2=500.0, async_=False):
                self.calls.append(('move_joints', tuple(q), vel_dps, async_))
                return {'ok': True}
            def close(self): pass
        fake = LatchedArm()
        arm = app.RpcArm(self.cfg['robot'], arm=fake)
        arm.call('estop')                                   # 本进程发的急停
        arm.call('move_joints', q=[0.0]*6, vel_dps=30.0)
        self.assertEqual(fake.calls[:3], ['estop', 'reset', ('move_joints', (0.0,)*6, 30.0, True)])
        # 复位后标志作废：未经本进程 estop（如外部 CLI）的锁存不许自动清
        fake.mode = 'ESTOP'
        with self.assertRaisesRegex(app.RobotFault, 'latched ESTOP'):
            arm.call('move_joints', q=[0.0]*6)
        self.assertEqual(fake.calls.count('reset'), 1)

    def test_rpc_external_estop_is_not_auto_cleared(self):
        class ExternalEstopArm:  # 锁存来自别处（CLI/其他进程），本进程没发过 estop
            def __init__(self):
                self.resets = 0
            def state(self, timeout_s=None):
                return dict(q_deg=[0.0]*6, online=[True]*7, stamp_ms=1,
                            mode='ESTOP', fault=0x02)
            def reset(self):
                self.resets += 1
                return {'ok': True}
            def move_joints(self, *a, **k):
                pass
            def close(self): pass
        fake = ExternalEstopArm()
        arm = app.RpcArm(self.cfg['robot'], arm=fake)
        with self.assertRaisesRegex(app.RobotFault, 'latched ESTOP.*d1t_sdk reset'):
            arm.call('move_joints', q=[0.0]*6)
        self.assertEqual(fake.resets, 0)   # 绝不悄悄复位
        self.assertFalse(arm.broken)       # 拒绝运动 ≠ 连接作废，可重试/可查状态

    def test_rpc_fault_is_not_auto_cleared(self):
        class FaultArm:  # 舵机保护 0x08 + 通信丢失 0x10：读了故障位才能决定复位
            def __init__(self):
                self.resets = 0
            def state(self, timeout_s=None):
                return dict(q_deg=[0.0]*6, online=[True]*7, stamp_ms=1,
                            mode='FAULT', fault=0x18)
            def reset(self):
                self.resets += 1
                return {'ok': True}
            def move_joints(self, *a, **k):
                pass
            def close(self): pass
        fake = FaultArm()
        arm = app.RpcArm(self.cfg['robot'], arm=fake)
        with self.assertRaisesRegex(app.RobotFault, 'latched FAULT.*servo-prot.*comm-loss'):
            arm.call('move_joints', q=[0.0]*6)
        self.assertEqual(fake.resets, 0)

    def test_rpc_ik_no_solution_is_retryable(self):
        class IkArm:
            def __init__(self):
                self.n = 0
            def ik(self, pos, rpy=None, quat=None, seed=None):
                self.n += 1
                raise D1ArmError('IK 无解（位置误差 8.5 mm，姿态误差 12.30°）')
            def close(self): pass
        fake = IkArm()
        arm = app.RpcArm(self.cfg['robot'], arm=fake)
        for _ in range(2):  # plan_grasp 换候选重试依赖：前缀可识别且连接不作废
            with self.assertRaisesRegex(app.RobotFault, '^ik no solution'):
                arm.call('ik', pose=dict(pos=[0.5, 0, 0.2], quat=[1, 0, 0, 0]))
            self.assertFalse(arm.broken)
        self.assertEqual(fake.n, 2)

    def test_rpc_rejects_teach_commands(self):
        class IdleArm:
            def state(self, timeout_s=None):
                return dict(q_deg=[0.0]*6, online=[True]*7, stamp_ms=1, mode='IDLE', fault=0)
            def close(self): pass
        arm = app.RpcArm(self.cfg['robot'], arm=IdleArm())
        for command in ('damping', 'set_joint'):
            with self.assertRaisesRegex(app.RobotFault, 'no d1t_sdk counterpart'):
                arm.call(command, id=0, power=500)
        self.assertFalse(arm.broken)

    def test_rpc_free_lock_passthrough(self):
        class Fake:
            def __init__(self): self.calls=[]
            def free_all(self): self.calls.append('free')
            def lock_all(self): self.calls.append('lock')
            def close(self): pass
        fake=Fake()
        arm=app.RpcArm(self.cfg['robot'], arm=fake)
        arm.call('free'); arm.call('lock')
        self.assertEqual(fake.calls,['free','lock'])

    def test_offline_grace_window(self):
        arm=object.__new__(app.RpcArm)
        arm.cfg=dict(self.cfg['robot'],offline_grace_s=2.0)  # Fixed window; yaml value may differ.
        arm.last_stamp=None; arm.offline_since=None; arm.broken=False
        def reply(q,online,**extra):
            out=dict(q_deg=q,online=online,mode='MOVING',stamp_ms=reply.n)
            reply.n+=1
            out.update(extra)
            return dict(state=out)
        reply.n=100
        degraded=[True,False]+[True]*5
        states=iter([reply([0]*6,degraded),reply([0]*6,degraded),reply([0]*6,degraded),
                     reply([0]*6,degraded),reply([0]*6,degraded),reply([0]*6,[True]*7),
                     reply([0]*6,degraded),reply([0]*6,[False]*7)])
        ticks=iter([0.0,0.5,1.0,1.5,2.2,3.0,3.2,4.0])
        arm.call=lambda cmd,**kw: next(states)
        with patch.object(app_robot.time,'monotonic',side_effect=lambda: next(ticks)):
            s=arm.state()   # t=0: transient offline starts, annotated only
            self.assertEqual(s.get('degraded'),[1])
            arm.state(); arm.state(); arm.state()  # within 2s grace
            with self.assertRaisesRegex(app.RobotFault,'offline for more than'):
                arm.state()   # t=2.2: outage outlived the grace window
            arm.state()   # t=3: recovered -> window reset
            s=arm.state()  # t=3.2: fresh transient annotated again
            self.assertEqual(s.get('degraded'),[1])
            with self.assertRaisesRegex(app.RobotFault,'All servos offline'):
                arm.state()  # total loss is immediate

    def test_idle_alone_does_not_mean_arrived(self):
        arm=object.__new__(app.RpcArm)
        arm.cfg=dict(self.cfg['robot'],move_timeout_s=2)
        arm.state=lambda: dict(mode='IDLE',q_deg=[0]*6)
        ticks=iter(np.arange(0,10,0.1))
        with patch.object(app_robot.time,'monotonic',side_effect=lambda: next(ticks)),patch.object(app_robot.time,'sleep'):
            with self.assertRaises(app.RobotFault): arm.wait_still(np.ones(6)*10)

    def test_settle_window_required(self):
        arm=object.__new__(app.RpcArm)
        arm.cfg=self.cfg['robot']
        count=[]
        def state():
            count.append(1)
            return dict(mode='IDLE',q_deg=[0]*6)
        arm.state=state
        ticks=iter(np.arange(0,10,0.1))
        with patch.object(app_robot.time,'monotonic',side_effect=lambda: next(ticks)),patch.object(app_robot.time,'sleep'):
            arm.wait_still(np.zeros(6))
        self.assertGreater(len(count),3)

    def run_workflow(self, fault=None, no_marker=False, preview=False):
        events=[]
        bg,ct=self.bg,self.ct
        class Arm:
            def __init__(self,_): self.index=0; self.speeds=[]
            def wait_still(self,*args): return self.state()
            def state(self): return dict(q_deg=[self.index]*6,mode='IDLE',online=[True]*7,stamp_ms=100)
            def move(self,q,vel=None):
                events.append(('move',q[0])); self.speeds.append((q[0],vel))
                if q[0]>=0:
                    if fault: raise fault
                    self.index=int(q[0])
                return self.state()
            def call(self,cmd,**kw):
                events.append((cmd,None))
                return dict(pose=as_pose(bg[self.index])) if cmd=='fk' else dict(ok=True)
            def close(self): events.append(('close_arm',None))
        arm=Arm(None)
        class Camera:
            k=np.eye(3); dist=np.zeros(5); metadata={'serial':'test'}
            def __init__(self,_): pass
            def frame(self): return np.zeros((20,20,3),np.uint8),{'frame_number':1}
            def close(self): events.append(('close_camera',None))
        class Detector:
            def __init__(self,_): pass
            def detect(self,*args):
                if no_marker: raise ValueError('not visible')
                return ct[arm.index],0.,np.zeros((4,2))
        with tempfile.TemporaryDirectory() as td, contextlib.redirect_stdout(io.StringIO()):
            config=Path(td)/'default.yaml'
            cfg=copy.deepcopy(self.cfg)
            cfg['robot'].update(ready_pose_deg=[-1]*6,safe_pose_deg=[-2]*6,
                                poses_deg=[[i]*6 for i in range(50)])
            cfg['camera']['preview']=preview
            cfg['collection'].update(capture_frames=3,min_detected_frames=3,discard_frames=1,
                                     max_pose_attempts=50)
            config.write_text(yaml.safe_dump(cfg),encoding='utf-8')
            original_finish=app.finish
            def finish(*args):
                events.append(('solve',None))
                return original_finish(*args)
            with patch.object(app,'RpcArm',return_value=arm),patch.object(app,'Camera',Camera),\
                 patch.object(app,'MarkerDetector',Detector),patch.object(app,'finish',side_effect=finish):
                code=app.main(['--config',str(config)])
            exists=(Path(td)/'hand_eye.npz').exists()
            return code,events,exists,arm.speeds

    def test_workflow_early_exit_safe_before_solve(self):
        code,events,exists,speeds=self.run_workflow()
        self.assertEqual(code,0)
        self.assertTrue(exists)
        moves=[v for k,v in events if k=='move']
        self.assertEqual(moves[0],-1)
        self.assertEqual(moves[-1],-2)
        self.assertLess(len(moves),52)
        self.assertLess(events.index(('move',-2)),events.index(('solve',None)))
        approach=self.cfg['robot']['approach_vel_dps']
        self.assertEqual(speeds[0],(-1,approach))
        self.assertEqual(speeds[-1],(-2,approach))
        # Collection poses pass no override; RpcArm.move then sends cfg vel_dps.
        self.assertTrue(all(vel is None for _,vel in speeds[1:-1]))

    def test_robot_fault_stops_without_safe_travel(self):
        code,events,exists,_=self.run_workflow(app.RobotFault('offline'))
        self.assertEqual(code,2)
        self.assertFalse(exists)
        self.assertIn(('estop',None),events)
        self.assertNotIn(('move',-2),events)

    def test_interrupt_stops_without_safe_travel(self):
        code,events,exists,_=self.run_workflow(KeyboardInterrupt())
        self.assertEqual(code,130)
        self.assertFalse(exists)
        self.assertIn(('estop',None),events)
        self.assertNotIn(('move',-2),events)

    def test_other_acquisition_error_returns_safe(self):
        code,events,exists,_=self.run_workflow(RuntimeError('acquisition failed'))
        self.assertEqual(code,2)
        self.assertFalse(exists)
        self.assertIn(('move',-2),events)

    def test_all_markers_missing_still_returns_safe(self):
        code,events,exists,_=self.run_workflow(no_marker=True)
        self.assertEqual(code,2)
        self.assertFalse(exists)
        self.assertEqual(len([k for k,v in events if k=='move']),52)
        self.assertIn(('move',-2),events)

    def test_collection_preview_shows_frames_and_survives_headless(self):
        with patch.object(app.cv2,'imshow') as shown,patch.object(app.cv2,'waitKey'),\
             patch.object(app.cv2,'destroyAllWindows'):
            code,events,exists,_=self.run_workflow(preview=True)
            self.assertEqual(code,0)
            self.assertGreater(shown.call_count,0)
            # Headless fallback: a failing window disables preview, not the run.
            shown.reset_mock()
            shown.side_effect=cv2.error('no GUI')
            code,events,exists,_=self.run_workflow(no_marker=True,preview=True)
        self.assertEqual(code,2)  # all poses rejected -> unqualified; preview must not abort
        self.assertEqual(shown.call_count,1)

    def test_live_view_shows_color_stream_and_quits(self):
        import types
        import scripts.live_view as live_view
        fake=types.ModuleType('pyrealsense2')
        class Frame:
            def get_data(self): return np.zeros((6,8,3),np.uint8)
        class Frames:
            def get_color_frame(self): return Frame()
        class Pipeline:
            def start(self,config): return object()
            def wait_for_frames(self,timeout_ms=None): return Frames()
            def stop(self): pass
        class Config:
            def enable_device(self,serial): pass
            def enable_stream(self,*args): pass
        fake.pipeline=Pipeline; fake.config=Config
        fake.stream=types.SimpleNamespace(color='color')
        fake.format=types.SimpleNamespace(bgr8='bgr8')
        keys=[-1,-1,-1,ord('q')]
        with contextlib.redirect_stdout(io.StringIO()):
            with patch.dict('sys.modules',{'pyrealsense2':fake}),\
                 patch.object(live_view.cv2,'namedWindow'),\
                 patch.object(live_view.cv2,'imshow') as shown,\
                 patch.object(live_view.cv2,'waitKey',side_effect=keys),\
                 patch.object(live_view.cv2,'getWindowProperty',return_value=1),\
                 patch.object(live_view.cv2,'destroyAllWindows'):
                self.assertEqual(live_view.main(['--config',
                    str(Path(__file__).resolve().parents[1] / 'config' / 'default.yaml')]),0)
            self.assertEqual(shown.call_count,4)

    def test_dryrun_diversity_verdict(self):
        import scripts.dryrun as dryrun
        bg=self.bg[:12]
        class Arm:
            def __init__(self,_,fixed=False): self.index,self.fixed=0,fixed
            def wait_still(self,*args): return self.state()
            def state(self): return dict(q_deg=[self.index]*6,mode='IDLE')
            def move(self,q,vel=None):
                if q[0]>=0: self.index=int(q[0])
                return self.state()
            def call(self,cmd,**kw):
                pose=bg[0] if self.fixed else bg[self.index]
                return dict(pose=as_pose(pose)) if cmd=='fk' else dict(ok=True)
            def close(self): pass
        with tempfile.TemporaryDirectory() as td, contextlib.redirect_stdout(io.StringIO()) as out:
            config=Path(td)/'default.yaml'
            cfg=copy.deepcopy(self.cfg)
            cfg['robot'].update(ready_pose_deg=[-1]*6,safe_pose_deg=[-2]*6,
                                poses_deg=[[i]*6 for i in range(12)])
            cfg['collection'].update(max_pose_attempts=12)
            cfg['quality'].update(min_samples=8)
            config.write_text(yaml.safe_dump(cfg),encoding='utf-8')
            with patch.object(dryrun,'RpcArm',return_value=Arm(None)):
                self.assertEqual(dryrun.main(['--config',str(config)]),0)
            self.assertIn('VERDICT: pose list passes',out.getvalue())
            with patch.object(dryrun,'RpcArm',return_value=Arm(None,fixed=True)):
                self.assertEqual(dryrun.main(['--config',str(config)]),2)
            self.assertIn('duplicates',out.getvalue())

    def test_pose_test_sequence_dwell_and_fault_stop(self):
        import scripts.pose_test as pose_test
        class Arm:
            def __init__(self,_,mode='IDLE'): self.index=0; self.moves=[]; self.mode=mode
            def wait_still(self,*args): return self.state()
            def state(self): return dict(q_deg=[self.index]*6,mode=self.mode,online=[True]*7,stamp_ms=100)
            def call(self,cmd,**kw):
                if cmd=='state': return dict(state=self.state())
                self.moves.append((cmd,None))
                return dict(ok=True)
            def move(self,q,vel=None):
                self.moves.append((int(q[0]),vel))
                if q[0]>=0: self.index=int(q[0])
                return self.state()
            def close(self): pass
        class FaultArm(Arm):  # Trajectory active when the fault arrives -> estop expected.
            def __init__(self,_): super().__init__(_); self.mode='MOVING'
            def move(self,q,vel=None):
                if q[0]==3: raise app.RobotFault('offline')
                return Arm.move(self,q,vel)
        class EstopLatchArm(Arm):  # Latched estop from an earlier run -> refuse without motion.
            def state(self): return dict(q_deg=[0]*6,mode='IDLE',online=[True]*7,err='estop')
        with tempfile.TemporaryDirectory() as td, contextlib.redirect_stdout(io.StringIO()) as out:
            config=Path(td)/'default.yaml'
            cfg=copy.deepcopy(self.cfg)
            cfg['robot'].update(ready_pose_deg=[-1]*6,safe_pose_deg=[-2]*6,
                                poses_deg=[[i]*6 for i in range(10)])
            cfg['collection'].update(max_pose_attempts=10)
            cfg['quality'].update(min_samples=8)
            config.write_text(yaml.safe_dump(cfg),encoding='utf-8')
            arm=Arm(None)
            with patch.object(pose_test,'RpcArm',return_value=arm):
                self.assertEqual(pose_test.main(['--config',str(config),'--dwell','0.001']),0)
            approach=self.cfg['robot']['approach_vel_dps']
            self.assertEqual(arm.moves[0],(-1,approach))
            self.assertEqual(arm.moves[-2],(-1,approach))
            self.assertEqual(arm.moves[-1],(-2,approach))
            self.assertTrue(all(vel is None for _,vel in arm.moves[1:-2]))
            self.assertEqual([q for q,_ in arm.moves],[-1]+list(range(10))+[-1,-2])
            fault=FaultArm(None)
            with patch.object(pose_test,'RpcArm',return_value=fault):
                self.assertEqual(pose_test.main(['--config',str(config),'--dwell','0.001']),2)
            self.assertIn(('estop',None),fault.moves)
            self.assertNotIn((-2,approach),fault.moves)
            latched=EstopLatchArm(None)
            with patch.object(pose_test,'RpcArm',return_value=latched):
                self.assertEqual(pose_test.main(['--config',str(config),'--dwell','0.001',
                                                 '--ready-timeout','0.2']),2)
            self.assertEqual(latched.moves,[])
            self.assertIn('home',out.getvalue())

    def test_teach_poses_capture_and_hold(self):
        import scripts.teach_poses as teach
        class Arm:
            def __init__(self,_): self.cmds=[]; self.moves=[]
            def call(self,cmd,**kw):
                self.cmds.append(cmd)
                if cmd=='state':
                    return dict(state=dict(q_deg=[1.5,-40.2,40.1,0.0,30.0,0.0],online=[True]*7,
                                           servo_raw_deg=[0.0]*7))
                return dict(ok=True)
            def move(self,pose,vel=None): self.moves.append((list(pose),vel))
            def close(self): pass
        class CameraT:
            def __init__(self,_): pass
            def frame(self): return np.zeros((4,6,3),np.uint8),{}
            def close(self): pass
        # ready, free, lock+record, free, lock+record, u(undo), free, lock+record, q
        import queue
        class Keys:  # 逐个交出按键；empty() 恒真，模拟等待期间没有误按
            def __init__(self,keys): self.keys=list(keys)
            def empty(self): return True
            def get_nowait(self):
                if not self.keys: raise queue.Empty
                return self.keys.pop(0)+'\n'
        q=Keys(['','','','','','u','','','q'])
        with tempfile.TemporaryDirectory() as td, contextlib.redirect_stdout(io.StringIO()):
            config=Path(td)/'charuco.yaml'
            config.write_text(yaml.safe_dump(self.cfg,default_flow_style=None),encoding='utf-8')
            arm=Arm(None)
            with patch.object(teach,'RpcArm',return_value=arm),patch.object(teach,'Camera',CameraT),\
                 patch.object(teach,'start_stdin_reader',return_value=q),\
                 patch.object(teach.cv2,'namedWindow'),patch.object(teach.cv2,'imshow'),\
                 patch.object(teach.cv2,'waitKey',return_value=-1),\
                 patch.object(teach.cv2,'destroyAllWindows'),patch.object(teach,'SETTLE_S',0.0):
                self.assertEqual(teach.main(['--config',str(config)]),0)
            self.assertEqual(arm.moves,[(self.cfg['robot']['ready_pose_deg'],
                                         self.cfg['robot']['approach_vel_dps'])])
            self.assertEqual(arm.cmds[0],'lock')   # 启动先锁住
            self.assertEqual(arm.cmds[-1],'lock')  # 退出必锁住
            self.assertEqual(arm.cmds.count('free'),3)
            saved=yaml.safe_load(config.read_text(encoding='utf-8'))
            self.assertEqual(saved['robot']['poses_deg'],[[1.5,-40.2,40.1,0.0,30.0,0.0]]*2)  # 3 记 1 删
            self.assertEqual(len(list(Path(td).glob('charuco.yaml.bak-*'))),1)
            self.assertEqual(len(list(Path(td).glob('teach_poses_*.yaml'))),1)

    def test_teach_poses_rejects_out_of_limit(self):
        import scripts.teach_poses as teach
        self.assertEqual(teach.out_of_limits([0.0]*6),[])
        self.assertEqual(teach.out_of_limits([0.0,0.0,500.0,0.0,0.0,0.0]),[2])
        self.assertEqual(teach.out_of_limits(None),[])

    def test_paths_and_text_encoder_link(self):
        from utils import paths
        with tempfile.TemporaryDirectory() as td:
            cfg_dir=Path(td)/'calibration'/'config'; cfg_dir.mkdir(parents=True)
            (cfg_dir/'default.yaml').write_text('board: {}',encoding='utf-8')
            cfg=str(cfg_dir/'default.yaml')
            self.assertEqual(paths.data_root(cfg),cfg_dir)
            self.assertEqual(paths.resolve_model('models/x.pt',cfg),
                             Path(td)/'calibration'/'models'/'x.pt')
            self.assertEqual(paths.resolve_model('/abs/x.pt',cfg),Path('/abs/x.pt'))
            self.assertIsNone(paths.ensure_text_encoder(cfg))  # 无编码器 -> None
            (Path(td)/'mobileclip2_b.ts').write_bytes(b'encoder')  # 仓库根位置
            other=Path(td)/'work'; other.mkdir()
            with patch.object(paths.Path,'cwd',return_value=other):
                target=paths.ensure_text_encoder(cfg)
            self.assertEqual(target,Path(td)/'mobileclip2_b.ts')
            link=other/'mobileclip2_b.ts'
            self.assertTrue(link.is_symlink())
            self.assertEqual(link.read_bytes(),b'encoder')

    def test_grasp_close_target_from_measured_jaw(self):
        # factor 1.0 -> contact hold at the measured width
        self.assertAlmostEqual(grasp.close_target_mm(0.059,1.0,15.0,65.0),59.0,places=6)
        # soft-object squeeze: 0.8 x measured
        self.assertAlmostEqual(grasp.close_target_mm(0.059,0.8,15.0,65.0),47.2,places=6)
        # tiny measurement clamps to the safety minimum; huge factor clamps to open
        self.assertEqual(grasp.close_target_mm(0.005,1.0,15.0,65.0),15.0)
        self.assertEqual(grasp.close_target_mm(0.059,2.0,15.0,65.0),65.0)

    def test_grasp_yolo_geometry(self):
        k=np.array([[600.,0,320],[0,600.,240],[0,0,1.]])
        depth=np.full((480,640),400.0)
        c=np.array([320.,240.]); th=np.deg2rad(30.)
        rot=np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]])
        rect=np.array([c+rot@[-60,-30],c+rot@[60,-30],c+rot@[60,30],c+rot@[-60,30]])
        g=grasp.grasp_camera(rect,depth,k,0.75)
        np.testing.assert_allclose(g['position'],[0,0,0.4],atol=1e-6)
        np.testing.assert_allclose(g['approach'],[0,0,-1],atol=1e-9)
        np.testing.assert_allclose(g['open_axis'],[-0.5,np.sqrt(3)/2,0],atol=1e-6)
        self.assertAlmostEqual(g['jaw_m'],0.04,places=6)  # 60px * 0.4m / 600
        tcp=grasp.tcp_rotation(g,np.eye(4),'y')
        np.testing.assert_allclose(tcp[:,0],[0,0,1],atol=1e-9)   # +x into the object
        np.testing.assert_allclose(tcp[:,1],[-0.5,np.sqrt(3)/2,0],atol=1e-6)
        self.assertAlmostEqual(np.linalg.det(tcp),1.0,places=9)
        quat=grasp.quat_from_matrix(tcp)
        self.assertAlmostEqual(np.linalg.norm(quat),1.0,places=9)
        np.testing.assert_allclose(core.pose_matrix(dict(pos=[0,0,0],quat=quat))[:3,:3],
                                   tcp,atol=1e-9)
        tcpz=grasp.tcp_rotation(g,np.eye(4),'z')
        np.testing.assert_allclose(tcpz[:,2],[-0.5,np.sqrt(3)/2,0],atol=1e-6)  # jaws on z
        self.assertIsNone(grasp.grasp_camera(rect,np.zeros_like(depth),k,0.75))

    def test_grasp_detections_class_whitelist(self):
        import types
        class Tensor:
            def __init__(self,v): self.v=v
            def cpu(self): return self
            def numpy(self): return np.array(self.v)
        class Boxes:
            xyxy=[Tensor([1,1,9,9]),Tensor([20,20,40,40])]; conf=[0.9,0.8]; cls=[0,1]
            def __len__(self): return 2
        result=types.SimpleNamespace(names={0:'cup',1:'person'},boxes=Boxes())
        all_hits=grasp.detections([result],(480,640,3))
        self.assertEqual([h[0] for h in all_hits],['cup','person'])
        kept=grasp.detections([result],(480,640,3),('cup',))
        self.assertEqual([h[0] for h in kept],['cup'])
        self.assertEqual(kept[0][1],0.9)

    def test_yolo_detect_runs_and_quits(self):
        import types
        import yolo_detect as yolo
        class Tensor:
            def __init__(self,v): self.v=v
            def cpu(self): return self
            def numpy(self): return np.array(self.v)
        class Boxes:
            xyxy=[Tensor([10,10,50,60])]; conf=[0.9]; cls=[0]
            def __len__(self): return 1
        class Result:
            names={0:'cup'}; boxes=Boxes()
        ultralytics=types.ModuleType('ultralytics')
        class YOLO:
            def __init__(self,path): pass
            def set_classes(self,classes): pass
            def predict(self,image,**kw): return [Result()]
        ultralytics.YOLO=YOLO
        rs=types.ModuleType('pyrealsense2')
        class Frames:
            def get_color_frame(self): return types.SimpleNamespace(get_data=lambda: np.zeros((6,8,3),np.uint8))
            def get_depth_frame(self): return types.SimpleNamespace(get_data=lambda: np.zeros((6,8),np.uint16))
        class Pipeline:
            def start(self,config):
                intr=types.SimpleNamespace(fx=600.,fy=600.,ppx=4.,ppy=3.)
                stream=types.SimpleNamespace(as_video_stream_profile=lambda: types.SimpleNamespace(get_intrinsics=lambda: intr))
                device=types.SimpleNamespace(first_depth_sensor=lambda: types.SimpleNamespace(get_depth_scale=lambda: 1.0))
                return types.SimpleNamespace(get_stream=lambda s: stream, get_device=lambda: device)
            def wait_for_frames(self,timeout_ms=None): return Frames()
            def stop(self): pass
        class Config:
            def enable_device(self,serial): pass
            def enable_stream(self,*args): pass
        rs.pipeline=Pipeline; rs.config=Config
        rs.align=lambda s: types.SimpleNamespace(process=lambda f: Frames())
        rs.stream=types.SimpleNamespace(color='color',depth='depth'); rs.format=types.SimpleNamespace(bgr8='bgr8',z16='z16')
        with tempfile.TemporaryDirectory() as td, contextlib.redirect_stdout(io.StringIO()):
            weights=Path(td)/'yolo11n.pt'; weights.write_bytes(b'weights')
            config=Path(td)/'default.yaml'
            config.write_text(yaml.safe_dump(dict(camera=dict(serial='',width=8,height=6,fps=30),
                                                  grasp=dict(model=str(weights),classes=['cup']))),
                              encoding='utf-8')
            with patch.dict('sys.modules',{'ultralytics':ultralytics,'pyrealsense2':rs}),\
                 patch.object(yolo.cv2,'namedWindow'),patch.object(yolo.cv2,'setMouseCallback'),\
                 patch.object(yolo.cv2,'resizeWindow'),\
                 patch.object(yolo.cv2,'imshow') as shown,\
                 patch.object(yolo.cv2,'waitKey',side_effect=[-1,ord('q')]),\
                 patch.object(yolo.cv2,'getWindowProperty',return_value=1),\
                 patch.object(yolo.cv2,'destroyAllWindows'):
                self.assertEqual(yolo.main(['--config',str(config)]),0)
            self.assertGreaterEqual(shown.call_count,1)


class GraspPlanTests(unittest.TestCase):
    def test_interior_j5_failure_selects_equivalent_roll(self):
        from unittest.mock import Mock
        arm = Mock()
        def ik(cmd, pose, seed):
            self.assertEqual(cmd, 'ik')
            interior = 0.35 < pose['pos'][0] < 0.38
            original_roll = abs(pose['quat'][0]) > 0.9
            return {'q_deg': [0, 0, 0, 0, 89 if interior and original_roll else 60, 0]}
        arm.call.side_effect = ik
        plan = grasp.plan_grasp(arm, np.array([0.4, 0., 0.1]), np.eye(3),
                                np.zeros(6), grasp.DEFAULTS)
        self.assertTrue(plan['flipped'])
        self.assertEqual(plan['tilt'], 0)
        self.assertEqual(plan['max_j5'], 60)
        np.testing.assert_allclose(plan['end']-plan['pre'], [0.05, 0, 0])
        arm.move.assert_not_called()

    def test_tilt_preserves_jaw_line_when_nominal_unreachable(self):
        from unittest.mock import Mock
        arm = Mock()
        def ik(cmd, pose, seed):
            if abs(core.pose_matrix(pose)[2, 0]) < 0.1:
                raise grasp.RobotFault('ik no solution')
            return {'q_deg': [0]*6}
        arm.call.side_effect = ik
        plan = grasp.plan_grasp(arm, np.array([0.3, 0, 0.15]), np.eye(3),
                                np.zeros(6), grasp.DEFAULTS)
        self.assertEqual(abs(plan['tilt']), 15)
        r = core.pose_matrix(dict(pos=plan['end'], quat=plan['quat']))[:3,:3]
        np.testing.assert_allclose(abs(r[:, 1]), [0, 1, 0], atol=1e-8)
        arm.call.side_effect = grasp.RobotFault('link lost')
        with self.assertRaisesRegex(grasp.RobotFault, 'link lost'):
            grasp.plan_grasp(arm, np.array([0.3, 0, 0.15]), np.eye(3), np.zeros(6), grasp.DEFAULTS)

    def test_final_segment_uses_requested_mode_only(self):
        from unittest.mock import Mock
        arm = Mock()
        xyz, quat, q = np.array([0.2, 0.1, 0.1]), [1, 0, 0, 0], np.zeros(6)
        grasp.traverse(arm, xyz, quat, q, grasp.DEFAULTS)
        arm.call.assert_called_once_with('move_l', pose=dict(pos=xyz.tolist(), quat=quat),
                                         vel=0.04, acc=0.25, **{'async': True})
        arm.wait_still.assert_called_once_with(q)
        arm.move.assert_not_called()
        arm.reset_mock()
        grasp.traverse(arm, xyz, quat, q, dict(grasp.DEFAULTS, final_motion='joint'))
        arm.call.assert_not_called()
        arm.move.assert_called_once_with(q.tolist(), grasp.DEFAULTS['grasp_vel_dps'])
        arm.reset_mock()
        arm.call.side_effect = grasp.RobotFault('ik no solution')
        with self.assertRaises(grasp.RobotFault):
            grasp.traverse(arm, xyz, quat, q, grasp.DEFAULTS)
        arm.move.assert_not_called()

    def test_close_and_lock_grip_air_and_timeout(self):
        class GripArm:
            """state 按 script 出 gripper_mm（有限序列停在末值，或无限 callable）。"""
            def __init__(self, script):
                self.script, self.i, self.calls = script, 0, []
            def call(self, cmd, **kw):
                self.calls.append((cmd, kw))
                if cmd == 'state':
                    mm = (self.script(self.i) if callable(self.script)
                          else self.script[min(self.i, len(self.script) - 1)])
                    self.i += 1
                    return dict(state=dict(q_deg=[0.0]*6, mode='IDLE', online=[True]*7,
                                           stamp_ms=self.i, gripper_mm=mm))
                return dict(ok=True)
            def close(self): pass
        g = dict(grasp.DEFAULTS)
        # 夹住：闭向 45mm 被物体挡在 ~65mm → 稳定后锁停住角，不再顶
        arm = GripArm([65.2, 65.1, 65.15, 65.1])
        with patch.object(grasp.time, 'sleep'):
            locked = grasp.close_and_lock(arm, 45.0, g)
        self.assertAlmostEqual(locked, 65.1, places=6)
        gripped = [kw.get('mm') for cmd, kw in arm.calls if cmd == 'gripper']
        self.assertEqual(gripped[0], 45.0)                    # 先按上限闭
        self.assertAlmostEqual(gripped[1], 65.1, places=6)    # 再锁停住角
        # 空夹：一路闭到 14.5mm（≤ 15+2）→ 开爪报错，不保持
        arm = GripArm([30.0, 20.0, 15.0, 14.5, 14.6, 14.5, 14.5])
        with patch.object(grasp.time, 'sleep'):
            with self.assertRaisesRegex(grasp.RobotFault, 'nothing held'):
                grasp.close_and_lock(arm, 15.0, g)
        gripped = [kw.get('mm') for cmd, kw in arm.calls if cmd == 'gripper']
        self.assertIn(g['gripper_open_mm'], gripped)          # 开爪了
        self.assertNotIn(14.5, gripped)                       # 空爪不当保持目标
        # 超时：一直没稳定 → 开爪报错
        arm = GripArm(lambda i: 40.0 + 0.5 * i)               # 每帧变化 0.5mm > 阈值
        ticks = iter([0.0, 1.0, 1.05, 1.1, 1.15, 5.0])        # 越过 2s 截止
        with patch.object(grasp.time, 'sleep'):
            with patch.object(grasp.time, 'monotonic', side_effect=lambda: next(ticks)):
                with self.assertRaisesRegex(grasp.RobotFault, 'did not settle'):
                    grasp.close_and_lock(arm, 40.0, g)
        gripped = [kw.get('mm') for cmd, kw in arm.calls if cmd == 'gripper']
        self.assertEqual(gripped, [40.0, g['gripper_open_mm']])  # 只闭了一次然后开爪

    def test_instance_depth_excludes_background(self):
        rect = np.array([[10, 10], [90, 10], [90, 40], [10, 40]], float)
        contour = np.array([[10, 10], [90, 30], [90, 40], [10, 20]], np.int32)
        depth = np.full((100, 100), 1200., dtype=np.float32)
        cv2.fillPoly(depth, [contour], 400.)
        k = np.array([[500.,0,50], [0,500.,50], [0,0,1.]])
        result = grasp.grasp_camera(rect, depth, k, 0.5, contour)
        self.assertAlmostEqual(result['position'][2], 0.4)
        self.assertLess(result['jaw_m'], 30 * 0.4 / 500)


if __name__=='__main__':
    unittest.main()
