"""cloud_top_down 的离线测试（不连臂/相机）：python -m pytest apps/vision_grasp/tests/test_cloud.py -q

合成 640x480 深度图：相机悬在桌面上方 0.40 m、斜向下 40°，桌面（基座系 z=0）上放物体。
深度用解析光线求交渲染并量化到 1 mm（同 z16 传感器），掩码 = 物体可见像素。
各几何用例会打印一行实测数值（pytest -s 或 -rA 可见）。
连续抓取流程用假臂 + 假夹爪走实机路径（不 dry-run）：流式去程 → 闭爪 → 流式回程，按键脚本、模拟时钟
（执行段里窗口泵的调用间隔拿它量）；画面中文标注用假画面渲染。
流式轨迹的单元测试用仓库 SDK 的真实运动学（d1t_sdk._kinematics，纯计算）：合成场景要落在竖直下抓够得着的
范围里（x ≈ 0.40 m；x = 0.46 m 处竖直姿态逆解无解，plan_grasp 会靠倾斜角兜底，这里不用）。
"""
import contextlib
import copy
import io
import re
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]          # 标定根目录
sys.path.insert(0, str(_ROOT))
_SDK = str(_ROOT.parents[1] / 'sdk')                 # 仓库根/sdk（d1t_sdk；同 utils/robot.py 的引导）
if _SDK not in sys.path:
    sys.path.insert(0, _SDK)
import handeye as app
import grasp_yolo as grasp
from d1t_sdk import D1ArmError
from d1t_sdk._joint_model import gripper_mm_to_deg, urdf_limits
from d1t_sdk._kinematics import IkParams, Kinematics, from_pose_dict, to_pose_dict
from utils import robot as app_robot

_KIN = Kinematics()                               # 仓库 SDK 的真实运动学（纯计算，不连臂）
_LIMITS = np.array(urdf_limits(), float) * np.deg2rad(1.0)   # URDF 限位（弧度）× 6


def solve_ik(pose, seed):
    """真实逆解：pos/quat 字典 + 种子关节角（度）→ 关节角（度）；无解抛 RobotFault（同适配层）。"""
    result = _KIN.ik(from_pose_dict(pose['pos'], quat=pose['quat']), np.deg2rad(seed), _LIMITS[:, 0], _LIMITS[:, 1],
                     IkParams())
    if not result.success:
        raise app.RobotFault(f'ik no solution: {result.pos_err * 1000:.1f} mm')
    return np.degrees(result.q_rad)


def solve_fk(q):
    """真实正解：关节角（度）→ SDK 的位姿字典（pos / quat / rpy）。"""
    return to_pose_dict(_KIN.fk(np.deg2rad(q)))

K = np.array([[450., 0, 320], [0, 450., 240], [0, 0, 1.]])     # 640x480 针孔内参
BALL = dict(center=np.array([0.40, -0.10, 0.03]), radius=0.03)
CLOUD_DEFAULTS = dict(cloud_erode_px=4, table_ring_inner_px=8, table_ring_outer_px=25,
                      min_height_above_table_m=0.005, gripper_close_margin_mm=8.0,
                      table_self_correct=True, table_self_correct_max_deg=10.0,
                      gripper_preclose_margin_mm=0, wait_before_grip_s=0.0)
# 发布配置（两个 yaml）里与代码默认值不同的项：实机调定的预合拢余量
YAML_OVERRIDES = dict(gripper_preclose_margin_mm=20)
# 时序类用例自己固定速度 / 加速度 / 预合拢 / 到位参数，不随 yaml 调参变化（期望时长按这组值算）
TIMING_GRASP = dict(linear_vel_mps=0.08, linear_acc_mps2=0.25, grasp_vel_dps=30.0, gripper_preclose_margin_mm=0)
TIMING_ROBOT = dict(acc_dps2=100.0, approach_vel_dps=40.0, settle_s=1.2, poll_s=0.1,
                    arrival_tolerance_deg=1.8, stable_tolerance_deg=0.3)
CLOUD_LINE = (r'cloud_top_down {label}: center \((?P<center>[^)]*)\) m, height (?P<height>\d+)mm, '
              r'table_z (?P<table_z>[^ ]+)m \((?P<source>\w+)(?:, ring kept (?P<kept>\d+)%)?\), '
              r'plane offset (?P<offset>[^ ]+)mm tilt (?P<tilt>[^ ]+)deg, jaw (?P<jaw>[\d.]+)cm '
              r'\((?P<jaw_source>\w+)\), close (?P<close>[\d.]+)mm, '
              r'along-axis shift (?P<shift>[\d.]+)cm(?P<box>, box=aabb)?, (?P<points>\d+) points')
CAMERA = object()            # solve() 的默认：center、ends_px、rect 取主循环同款的二维测量
NO_KEY = 255                 # waitKey 没有按键（-1 & 0xFF）：给「预热中」那一帧和空转用
SINK, TILT = 0.024, 1.5      # 坐标链误差：相机位姿下沉（米）与倾斜（度）
PITCH, ROLL, YAW = 4.5, 1.0, 1.0   # 相机姿态误差（度）：俯仰绕基座 y、横滚绕基座 x、绕竖直轴转


def tube_at(yaw_deg, y=0.13, x=0.46, radius=0.02, length=0.150):
    """躺放圆柱：绕竖直轴转 yaw（0° = 轴沿基座 x，即相机视线方向），中心在 (x, y, radius)。"""
    a = np.deg2rad(yaw_deg)
    return dict(center=np.array([x, y, radius]), axis=np.array([np.cos(a), np.sin(a), 0.0]),
                half=length / 2, radius=radius)


TUBE = tube_at(90.0)                     # 轴沿基座 y，偏光轴 13 cm
TUBE_TURNED = tube_at(30.0)              # 同一根，轴在水平面内转 30°
TUBE_ON_AXIS = tube_at(90.0, y=0.0)      # 同一根，摆到相机光轴所在平面
TUBE_ALONG_VIEW = tube_at(0.0, y=0.0)    # 轴沿相机视线方向
YAWS = (0, 30, 45, 60, 90)
PLACEMENTS = (('on the optical axis', 0.0), ('10 cm off the optical axis', 0.10))


def crescent(mid, facing_deg, arc=0.10, radius=0.018, sweep_deg=60.0):
    """躺放的弯月形物体：中心线是半径 arc 的水平圆弧（圆心角 sweep，两端切平），截面半径 radius。
    mid = 圆弧中点的截面中心（真值），facing_deg = 从圆心指向中点的水平方向（基座系，度）。
    弦的方向 = facing_deg − 90°，所以 crescent(mid, yaw + 90) 是弦绕竖直轴转 yaw、鼓面朝 +y 一侧。"""
    out = np.array([np.cos(np.deg2rad(facing_deg)), np.sin(np.deg2rad(facing_deg)), 0.0])
    return dict(pivot=np.array([mid[0], mid[1], 0.0]) - arc * out, out=out, arc=arc,
                half=np.deg2rad(sweep_deg / 2), radius=radius,
                truth=np.array([mid[0], mid[1], radius]))


def sweep_scene(kind, yaw, y):
    """转角 / 摆位扫描用的一种摆法：圆柱或弯月形（弦沿 yaw），中点 (0.45~0.46, y)。"""
    if kind == 'tube':
        return tube_at(yaw, y=y)
    return crescent((0.45, y), yaw + 90.0)


def misled(t_true, sink_m, tilt_deg, pivot_xy):
    """把相机位姿「测歪」：绕过 (pivot_xy, 0)、沿基座 y 的水平轴倾斜 tilt_deg，再整体下沉 sink_m。
    返回给算法的 T_base_camera；渲染仍用真值 t_true。"""
    error = np.eye(4)
    error[:3, :3] = cv2.Rodrigues(np.deg2rad(tilt_deg) * np.array([0.0, 1.0, 0.0]))[0]
    pivot = np.array([pivot_xy[0], pivot_xy[1], 0.0])
    error[:3, 3] = pivot - error[:3, :3] @ pivot - [0.0, 0.0, sink_m]
    return error @ t_true


def camera_pose_error(t_true, pitch_deg, roll_deg, sink_m, yaw_deg=0.0):
    """相机位姿误差（实机那种）：绕基座 y 轴俯仰 pitch_deg、绕基座 x 轴横滚 roll_deg、绕竖直轴转
    yaw_deg，转轴都过光心，再整体下沉 sink_m。返回给算法的 T_base_camera = 误差 @ 真值；渲染仍用
    真值 t_true。误差是绕光心的旋转：离相机越远的点偏得越多，桌面看起来又歪又矮；
    正的俯仰把物体偏向底座，同实机现象一致。"""
    center = t_true[:3, 3]
    error = np.eye(4)
    error[:3, :3] = cv2.Rodrigues(np.deg2rad([roll_deg, pitch_deg, yaw_deg]))[0]
    error[:3, 3] = center - error[:3, :3] @ center - [0.0, 0.0, sink_m]
    return error @ t_true


def short_axis(tube):
    """躺放圆柱的真实短轴：水平、垂直于圆柱轴的单位向量。"""
    return np.array([tube['axis'][1], -tube['axis'][0], 0.0])


def camera_pose(tilt_deg, height=0.40):
    """相机在 (0, 0, height)，光轴沿基座 +x 向下倾 tilt；返回 T_base_camera（相机系 x 右 y 下 z 前）。"""
    tilt = np.deg2rad(tilt_deg)
    forward = np.array([np.cos(tilt), 0.0, -np.sin(tilt)])
    right = np.array([0.0, -1.0, 0.0])
    out = np.eye(4)
    out[:3, :3] = np.column_stack([right, np.cross(forward, right), forward])
    out[:3, 3] = [0.0, 0.0, height]
    return out


def pixel_rays(t_base_camera, k=K, shape=(480, 640)):
    """每个像素的光线（基座系，相机系 z=1）：沿光线走 s 时相机系深度就是 s。"""
    v, u = np.mgrid[0:shape[0], 0:shape[1]]
    rays = np.stack([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], np.ones(u.shape)], -1)
    return rays @ t_base_camera[:3, :3].T


def project_pixels(points, t_base_camera, k=K):
    """基座系点 → 像素 (u, v)。"""
    camera = (np.asarray(points, float) - t_base_camera[:3, 3]) @ t_base_camera[:3, :3]
    return camera[..., :2] / camera[..., 2:3] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]


def smaller_root(a, b, c):
    """a s^2 + b s + c = 0 的较小根；无实根或 a=0 记 inf。"""
    with np.errstate(divide='ignore', invalid='ignore'):
        disc = b * b - 4 * a * c
        return np.where((a > 0) & (disc >= 0), (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)


def nearest(obj, origin, rays):
    """各像素光线与球 / 圆柱的最近交点参数 s（相机系深度，米），无交为 inf。"""
    with np.errstate(all='ignore'):
        delta = origin - obj['center']
        axis = obj.get('axis')
        if axis is None:                                   # 球
            return smaller_root((rays * rays).sum(-1), 2 * (rays @ delta),
                                delta @ delta - obj['radius'] ** 2)
        side = rays - (rays @ axis)[..., None] * axis      # 圆柱：去掉轴向分量后按圆求交
        offset = delta - (delta @ axis) * axis
        s = smaller_root((side * side).sum(-1), 2 * (side @ offset),
                         offset @ offset - obj['radius'] ** 2)
        s[np.abs((delta + s[..., None] * rays) @ axis) > obj['half']] = np.inf   # 侧面交点落在长度之外
        for sign in (-1, 1):                               # 两个端面圆盘
            cap = obj['center'] + sign * obj['half'] * axis
            t = ((cap - origin) @ axis) / (rays @ axis)
            radial = origin + t[..., None] * rays - cap
            on = (t > 0) & ((radial * radial).sum(-1) - (radial @ axis) ** 2 <= obj['radius'] ** 2)
            s[on] = np.minimum(s[on], t[on])
        return s


def arc_tube_nearest(obj, t_base_camera, origin, rays, k=K, samples=1801):
    """各像素光线与躺放圆弧管的最近交点参数 s（相机系深度，米），无交为 inf。

    管体 = 中心线圆弧（高度 = 截面半径）上扫过的一串球；光线与管体的首交 = 各球首交距离的最小值。
    侧面交点只保留落在圆心角内的，两端另按端面圆盘求交。只算物体投影附近的一块像素。"""
    pivot, arc, tube, half = obj['pivot'], obj['arc'], obj['radius'], obj['half']
    out = obj['out']
    side = np.array([-out[1], out[0], 0.0])                # out 在水平面内转 90°
    angles = np.linspace(-half, half, samples)
    centers = pivot + arc * (np.cos(angles)[:, None] * out + np.sin(angles)[:, None] * side)
    centers[:, 2] = tube
    camera = (centers - t_base_camera[:3, 3]) @ t_base_camera[:3, :3]
    uv = camera[:, :2] / camera[:, 2:3] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    pad = k[0, 0] * tube / camera[:, 2].min() + 3
    height, width = rays.shape[:2]
    u0, u1 = np.clip([uv[:, 0].min() - pad, uv[:, 0].max() + pad], 0, width).astype(int)
    v0, v1 = np.clip([uv[:, 1].min() - pad, uv[:, 1].max() + pad], 0, height).astype(int)
    sub = rays[v0:v1, u0:u1].reshape(-1, 3)
    best = np.full(len(sub), np.inf)
    with np.errstate(all='ignore'):
        norm2 = (sub * sub).sum(1)[:, None]               # 光线不是单位向量（相机系 z=1）
        for start in range(0, len(centers), 150):          # 分块，免得 (像素 x 采样) 数组过大
            away = centers[start:start + 150] - origin
            b = sub @ away.T
            disc = b * b - norm2 * ((away * away).sum(1) - tube ** 2)
            s = (b - np.sqrt(np.maximum(disc, 0))) / norm2
            s[(disc < 0) | (s <= 0)] = np.inf
            hit = origin + sub[:, None, :] * s[..., None] - pivot
            s[np.abs(np.arctan2(hit @ side, hit @ out)) > half] = np.inf   # 侧面交点落在圆心角之外
            best = np.minimum(best, s.min(1))
        for sign in (-1, 1):                               # 两端的端面圆盘
            a = sign * half
            cap = pivot + arc * (np.cos(a) * out + np.sin(a) * side) + [0.0, 0.0, tube]
            normal = -np.sin(a) * out + np.cos(a) * side
            t = ((cap - origin) @ normal) / (sub @ normal)
            on = (t > 0) & (((origin + t[:, None] * sub - cap) ** 2).sum(1) <= tube ** 2)
            best[on] = np.minimum(best[on], t[on])
    out_s = np.full(rays.shape[:2], np.inf)
    out_s[v0:v1, u0:u1] = best.reshape(v1 - v0, u1 - u0)
    return out_s


def render(t_base_camera, objects, k=K, shape=(480, 640), y_edge=None, floor_z=-0.30):
    """解析渲染：返回 (深度 uint16 毫米, 各物体的可见像素掩码)。桌面 = 基座系 z=0。
    给了 y_edge：桌面只到 y = y_edge，再往外是低 floor_z 的地面（环带混进桌外的场景）。"""
    origin, rays = t_base_camera[:3, 3], pixel_rays(t_base_camera, k, shape)
    with np.errstate(divide='ignore', invalid='ignore'):
        s = -origin[2] / rays[..., 2]
        if y_edge is not None:
            s = np.where(origin[1] + s * rays[..., 1] > y_edge, (floor_z - origin[2]) / rays[..., 2], s)
    depth = np.where((rays[..., 2] < 0) & (s > 0), s, np.inf)
    masks = []
    for obj in objects:
        s = (arc_tube_nearest(obj, t_base_camera, origin, rays, k) if 'arc' in obj
             else nearest(obj, origin, rays))
        seen = (s > 0) & (s < depth)
        depth[seen] = s[seen]
        masks.append(seen)
    return np.where(np.isfinite(depth), np.rint(depth * 1000), 0).astype(np.uint16), masks


def contour_of(mask):
    """掩码的外轮廓 (N, 2) float32，同分割模型给出的 masks.xy。"""
    return max(cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0],
               key=cv2.contourArea).reshape(-1, 2).astype(np.float32)


def camera_measure(depth, mask, quantile):
    """主循环对一个实例的二维测量：返回 (grasp_camera 的结果, rect)。"""
    depth_mm, contour = np.asarray(depth, float), contour_of(mask)
    rect = cv2.boxPoints(cv2.minAreaRect(contour))
    return grasp.grasp_camera(rect, depth_mm, K, quantile, contour), rect


class FakeGripper:
    """夹爪模拟（实机流程用）：每读一次开度朝目标走 10 mm。合的时候两指之间有物体（held_at）
    就被挡在这个开度；没有就一路合到命令的目标——同实机：闭口上限就是命令值，抓空时停在它上面。"""

    def __init__(self, open_mm, held_at=None, jitter_mm=0.0):
        self.mm, self.target, self.held_at, self.jitter, self.reads = open_mm, open_mm, held_at, jitter_mm, 0

    def command(self, mm):
        """下达开口目标，返回这一程的模拟时长（J6 按 ≤20°/s 的五次多项式走）。"""
        seconds = 1.875 * abs(gripper_mm_to_deg(mm) - gripper_mm_to_deg(self.mm)) / 20.0
        self.target = float(mm)
        return max(seconds, 0.05)

    def reading(self):
        self.reads += 1
        if self.mm > self.target:                        # 合
            stop = self.target
            if self.held_at is not None and self.held_at >= self.target:
                stop = max(stop, self.held_at)           # 被物体挡住
            self.mm = max(stop, self.mm - 10.0)
        else:                                            # 开
            self.mm = min(self.target, self.mm + 10.0)
        return self.mm + (self.jitter if self.reads % 2 else -self.jitter)


def as_pose(t):
    rv = cv2.Rodrigues(t[:3, :3])[0].ravel()
    a = np.linalg.norm(rv)
    xyz = rv / a * np.sin(a / 2) if a > 1e-12 else np.zeros(3)
    return dict(pos=t[:3, 3].tolist(), quat=[float(np.cos(a / 2)), *xyz.tolist()])


def report(label, result, truth):
    """打印一行实测数值，返回中心误差向量（毫米）。"""
    err = (result['position'] - truth) * 1000
    print(f'\n[{label}] center error xyz {err.round(2).tolist()} mm, |xy| {np.linalg.norm(err[:2]):.2f} mm, '
          f'|e| {np.linalg.norm(err):.2f} mm | height {result["height_m"]*1000:.1f} mm, '
          f'{result["table_source"]}, plane offset {result["plane_offset_m"]*1000:+.1f} mm, '
          f'tilt {result["plane_tilt_deg"]:.1f} deg'
          + (f" (before self-correction {result['raw_plane_offset_m']*1000:+.1f} mm, "
             f"{result['raw_plane_tilt_deg']:.1f} deg; correction "
             f"{result['correction']['tilt_deg']:.1f} deg, {result['correction']['dz_m']*1000:+.1f} mm)"
             if result['correction'] else '') + f' | n_points {result["n_points"]}, '
          f'jaw {result["jaw_m"]*1000:.1f} mm ({result["jaw_source"]}), '
          f'close {grasp.close_margin_mm(result["jaw_m"], 8.0, 15.0, 65.0):.1f} mm, '
          f'along-axis shift {abs(result["along_shift_m"])*100:.1f} cm, '
          f'open_dir {result["open_dir"].round(4).tolist()}')
    return err


class CloudGeometryTests(unittest.TestCase):
    # 中心判据：球、圆柱（轴沿 y）< 3 mm；圆柱水平转 30° < 5 mm；圆柱与弯月形绕竖直轴转 0/45/90°、
    # 在光轴上或偏 10 cm：圆柱 < 5 mm，弯月形 xy 距圆弧中点 < 8 mm 且中心投回图像落在掩码内。
    # 爪宽判据：圆柱、弯月形转 0/30/45/60/90°，与真实直径之差在 [-2, +6] mm，闭口 = 直径 - 8 mm ± 3 mm。
    # xy 来自「图像中心的视线 ∩ 中线高度平面」再加沿长轴的透视修正，z 来自物高。

    @classmethod
    def setUpClass(cls):
        cls.view = camera_pose(40.0)
        cls.sweep = {}
        for kind in ('tube', 'crescent'):
            for yaw in YAWS:
                for _, y in PLACEMENTS:
                    obj = sweep_scene(kind, yaw, y)
                    depth, (mask,) = render(cls.view, [obj])
                    cls.sweep[(kind, yaw, y)] = (obj, depth, mask)

    def setUp(self):
        self.t_base_camera = self.view
        self.depth, (self.ball, self.tube) = render(self.t_base_camera, [BALL, TUBE])
        self.g = dict(grasp.DEFAULTS)

    def solve(self, mask, depth=None, view=None, center=CAMERA, ends=CAMERA, rect=CAMERA, q=None,
              **overrides):
        """与主循环同一条计算链：二维测量给 center_px / ends_px / rect，再进 object_from_cloud。"""
        g = dict(self.g, **overrides)
        depth = self.depth if depth is None else depth
        camera, measured_rect = camera_measure(depth, mask, g['depth_quantile'])
        return grasp.object_from_cloud(
            camera['mask'], depth, K, self.t_base_camera if view is None else view, g,
            camera['center_px'] if center is CAMERA else center,
            camera['ends_px'] if ends is CAMERA else ends,
            measured_rect if rect is CAMERA else rect, q_deg=q)

    def logged(self, *call):
        """调用 solve 并返回 (结果, 日志文本)。"""
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            result = call[0]()
        return result, out.getvalue()

    def test_ball_center(self):
        result = self.solve(self.ball)
        self.assertIsNotNone(result)
        err = report('ball r=30', result, BALL['center'])
        self.assertEqual(result['table_source'], 'plane')
        self.assertLess(np.linalg.norm(err), 3.0, msg=f'ball center error {np.linalg.norm(err):.2f} mm, '
                                                     f'xyz {err.round(2).tolist()} mm')

    def test_tube_center(self):
        depth, (mask,) = render(self.t_base_camera, [TUBE_ON_AXIS])
        scenes = (('13 cm off the optical axis', TUBE, self.depth, self.tube),
                  ('on the optical axis', TUBE_ON_AXIS, depth, mask))
        for label, tube, depth, mask in scenes:
            with self.subTest(label):
                result = self.solve(mask, depth)
                self.assertIsNotNone(result)
                err = report(f'tube r=20 L=150, {label}', result, tube['center'])
                self.assertLess(np.linalg.norm(err), 3.0, msg=f'tube center error {np.linalg.norm(err):.2f} mm, '
                                                             f'xyz {err.round(2).tolist()} mm')

    def test_turned_tube_center(self):
        depth, (mask,) = render(self.t_base_camera, [TUBE_TURNED])
        result = self.solve(mask, depth)
        self.assertIsNotNone(result)
        err = report('tube turned 30 deg', result, TUBE_TURNED['center'])
        self.assertLess(np.linalg.norm(err), 5.0, msg=f'turned tube center error {np.linalg.norm(err):.2f} mm, '
                                                     f'xyz {err.round(2).tolist()} mm')

    def test_center_across_yaw_and_placement(self):
        # 沿视线躺放时二维中段被透视带向近端：没有沿长轴修正，圆柱转 0° 会偏十几毫米
        for kind in ('tube', 'crescent'):
            for yaw in (0, 45, 90):
                for placement, y in PLACEMENTS:
                    with self.subTest(kind=kind, yaw=yaw, placement=placement):
                        obj, depth, mask = self.sweep[(kind, yaw, y)]
                        result = self.solve(mask, depth)
                        self.assertIsNotNone(result)
                        truth = obj['center'] if kind == 'tube' else obj['truth']
                        err = report(f'{kind} yaw {yaw}, {placement}', result, truth)
                        if kind == 'tube':
                            self.assertLess(np.linalg.norm(err), 5.0, msg=f'tube center error '
                                                                         f'{np.linalg.norm(err):.2f} mm')
                            angle = np.degrees(np.arccos(np.clip(abs(result['open_dir'] @ short_axis(obj)), 0, 1)))
                            self.assertLessEqual(angle, 5.0, msg=f'open_dir vs short axis {angle:.2f} deg')
                        else:
                            u, v = np.rint(project_pixels(result['position'], self.t_base_camera)).astype(int)
                            self.assertTrue(mask[v, u], 'center does not project back into the mask')
                            self.assertLess(np.linalg.norm(err[:2]), 8.0, msg=f'crescent xy error '
                                                                             f'{np.linalg.norm(err[:2]):.2f} mm')

    def test_tube_open_dir_follows_short_axis(self):
        for label, tube in (('tube', TUBE), ('tube turned 30 deg', TUBE_TURNED)):
            with self.subTest(label):
                depth, (mask,) = render(self.t_base_camera, [tube])
                result = self.solve(mask, depth)
                self.assertIsNotNone(result)
                report(label, result, tube['center'])
                angle = np.degrees(np.arccos(np.clip(abs(result['open_dir'] @ short_axis(tube)), 0, 1)))
                self.assertAlmostEqual(float(result['open_dir'][2]), 0.0, places=9)     # 水平
                self.assertAlmostEqual(float(np.linalg.norm(result['open_dir'])), 1.0, places=9)
                self.assertLessEqual(angle, 5.0, msg=f'open_dir vs short axis {angle:.2f} deg')

    def test_crescent_center_lands_on_the_body(self):
        for label, facing in (('belly toward the camera', 180.0), ('horns toward the camera', 0.0),
                              ('turned 30 deg', 210.0)):
            with self.subTest(label):
                tube = crescent((0.45, 0.05), facing)
                depth, (mask,) = render(self.t_base_camera, [tube])
                rows, cols = np.nonzero(mask)                  # 夹具自检：可见点都在管面上
                points = grasp.backproject(cols, rows, depth[rows, cols] / 1000.0, K, self.t_base_camera)
                rel = points - tube['pivot']
                across = np.array([-tube['out'][1], tube['out'][0], 0.0])   # 中心线：半径 arc、高度 = 截面半径
                to_axis = np.hypot(np.hypot(rel @ tube['out'], rel @ across) - tube['arc'],
                                   rel[:, 2] - tube['radius'])
                turn = np.abs(np.degrees(np.arctan2(rel @ across, rel @ tube['out'])))
                side = np.abs(to_axis - tube['radius']) <= 0.0015            # 侧面：到中心线圆弧 = 截面半径
                caps = (to_axis <= tube['radius'] + 0.0015) & (turn >= np.degrees(tube['half']) - 0.5)
                self.assertTrue((side | caps).all(), 'rendered points off the tube surface')  # 端面圆盘在两端
                result = self.solve(mask, depth)
                self.assertIsNotNone(result)
                err = report(f'crescent R=100 r=18, {label}', result, tube['truth'])
                u, v = np.rint(project_pixels(result['position'], self.t_base_camera)).astype(int)
                self.assertTrue(mask[v, u], 'center does not project back into the mask')
                self.assertLess(np.linalg.norm(err[:2]), 8.0, msg=f'crescent xy error '
                                                                 f'{np.linalg.norm(err[:2]):.2f} mm, '
                                                                 f'xy {err[:2].round(2).tolist()} mm')

    def test_jaw_is_section_diameter_across_yaw(self):
        for kind, diameter in (('tube', 2 * 0.020), ('crescent', 2 * 0.018)):
            for yaw in YAWS:
                for placement, y in PLACEMENTS:
                    with self.subTest(kind=kind, yaw=yaw, placement=placement):
                        obj, depth, mask = self.sweep[(kind, yaw, y)]
                        result = self.solve(mask, depth)
                        self.assertIsNotNone(result)
                        report(f'{kind} yaw {yaw}, {placement}', result,
                               obj['center'] if kind == 'tube' else obj['truth'])
                        self.assertEqual(result['jaw_source'], 'radius')
                        self.assertTrue(-0.002 <= result['jaw_m'] - diameter <= 0.006,
                                        f'jaw - diameter {(result["jaw_m"] - diameter)*1000:+.1f} mm')
                        close = grasp.close_margin_mm(result['jaw_m'], self.g['gripper_close_margin_mm'],
                                                      self.g['gripper_close_min_mm'], self.g['gripper_open_mm'])
                        self.assertAlmostEqual(close, diameter * 1000 - 8.0, delta=3.0,
                                               msg=f'close_mm {close:.1f} for diameter {diameter*1000:.0f} mm')

    def test_close_margin_mm(self):
        self.assertEqual(grasp.close_margin_mm(0.040, 8.0, 15.0, 65.0), 32.0)   # 爪宽减余量
        self.assertEqual(grasp.close_margin_mm(0.020, 8.0, 15.0, 65.0), 15.0)   # 不低于闭口下限
        self.assertEqual(grasp.close_margin_mm(0.080, 8.0, 15.0, 65.0), 65.0)   # 不超过张口
        self.assertEqual(grasp.close_target_mm(0.040, 0.7, 15.0, 65.0), 28.0)   # 其余模式仍是 jaw×系数
        self.assertEqual(grasp.preclose_target_mm(0.040, 15.0, 65.0), 55.0)    # 预合拢 = 爪宽 + 余量
        self.assertEqual(grasp.preclose_target_mm(0.020, 2.0, 65.0), 28.0)     # 余量不足 8 mm：至少留 8 mm
        self.assertEqual(grasp.preclose_target_mm(0.055, 15.0, 65.0), 65.0)    # 不超过张口
        self.assertEqual(grasp.preclose_target_mm(0.080, 15.0, 65.0), 65.0)

    def test_long_axis_fallback_keeps_image_center(self):
        obj, depth, mask = self.sweep[('tube', 0, 0.0)]
        camera, _ = camera_measure(depth, mask, self.g['depth_quantile'])
        corrected = self.solve(mask, depth)
        self.assertGreater(abs(corrected['along_shift_m']), 0.010)              # 有长轴：沿轴挪了一厘米多
        # ends_px 缺失（轴对齐框就是这种情况）
        result, text = self.logged(lambda: self.solve(mask, depth, ends=None))
        self.assertIn('long axis unavailable (ends_px missing); keeping the image-center point', text)
        self.assertEqual((result['along_shift_m'], result['aabb']), (0.0, True))
        self.assertGreater(abs(result['position'][0] - obj['center'][0]), 0.010)  # 没修透视：仍偏向近端
        # 两端视线交点太近
        close_ends = [(camera['center_px'][0] - 2, camera['center_px'][1]),
                      (camera['center_px'][0] + 2, camera['center_px'][1])]
        result, text = self.logged(lambda: self.solve(mask, depth, ends=close_ends))
        self.assertRegex(text, r'long axis unavailable \(end hits only \d+ mm apart\); keeping the image-center')
        self.assertEqual(result['along_shift_m'], 0.0)
        # 一端的视线不朝下
        view = camera_pose(25.0)                           # 图像最上面几行的视线在地平线以上
        depth, (mask,) = render(view, [BALL])
        camera, _ = camera_measure(depth, mask, self.g['depth_quantile'])
        result, text = self.logged(lambda: self.solve(mask, depth, view=view,
                                                      ends=[(320.0, 5.0), camera['center_px']]))
        self.assertIn('long axis unavailable (an end ray does not reach the mid-height plane); '
                      'keeping the image-center point', text)
        self.assertEqual(result['along_shift_m'], 0.0)

    def test_center_ray_not_downward_rejected(self):
        view = camera_pose(25.0)                           # 图像最上面几行的视线在地平线以上
        depth, (ball,) = render(view, [BALL])
        result, text = self.logged(lambda: self.solve(ball, depth, view=view, center=(320.0, 5.0)))
        self.assertIsNone(result)
        self.assertLess(5.0, K[1, 2] - K[1, 1] * np.tan(np.deg2rad(25.0)))   # 该行确实朝上
        self.assertIn('rejected: the ray through the image center does not reach', text)

    def test_chain_offset_and_tilt_are_absorbed(self):
        # 相机链把整张桌面测低 2.4 cm 并倾斜 1.5°：物高、中心高度仍以真实桌面为准，并报出偏差读数
        scenes = (('tube along y', TUBE, TUBE['center'], 0.040),
                  ('tube along the line of sight', TUBE_ALONG_VIEW, TUBE_ALONG_VIEW['center'], 0.040),
                  ('crescent chord along y', sweep_scene('crescent', 90, 0.0),
                   sweep_scene('crescent', 90, 0.0)['truth'], 0.036))
        for label, obj, truth, true_height in scenes:
            with self.subTest(label):
                depth, (mask,) = render(self.t_base_camera, [obj])
                view = misled(self.t_base_camera, SINK, TILT, truth[:2])
                # 这组读数是「不修正」时的坐标链偏差；自校正开着时它们应接近 0（见 TableSelfCorrectTests）
                result = self.solve(mask, depth, view=view, table_self_correct=False)
                self.assertIsNotNone(result)
                err = report(f'{label}, camera chain {SINK*1000:.0f} mm low and {TILT} deg tilted, '
                             'no self-correction', result, truth)
                self.assertEqual(result['table_source'], 'plane')
                self.assertLess(abs(result['height_m'] - true_height), 0.003,
                                msg=f'height error {(result["height_m"] - true_height)*1000:+.1f} mm')
                self.assertLess(abs(err[2]), 3.0, msg=f'center z error {err[2]:+.1f} mm')
                self.assertAlmostEqual(result['plane_offset_m'], -SINK, delta=0.003,
                                       msg=f'plane offset {result["plane_offset_m"]*1000:+.1f} mm')
                self.assertAlmostEqual(result['plane_tilt_deg'], TILT, delta=0.3,
                                       msg=f'plane tilt {result["plane_tilt_deg"]:.2f} deg')
                # 指尖下限用配置的真实桌面，不跟着测低的桌面往下走
                self.assertAlmostEqual(grasp.finger_floor_z(self.g['support_z_m'], dict(self.g, grasp_offset_m=0.04)),
                                       0.015, places=9)

    def test_ring_without_depth_falls_back_to_support_z(self):
        clean = self.solve(self.ball)
        depth = self.depth.copy()
        depth[~self.ball] = 0                               # 掩码外全是无效深度：环带里取不到桌面
        result, text = self.logged(lambda: self.solve(self.ball, depth))
        self.assertIsNotNone(result)
        report('ball, ring depth all zero', result, BALL['center'])
        self.assertIn('table ring unusable (0 points); using support_z_m = 0.000 m', text)
        self.assertIsNone(result['ring_kept'])
        self.assertEqual((result['table_source'], result['table_z']), ('config', self.g['support_z_m']))
        self.assertEqual((result['plane_offset_m'], result['plane_tilt_deg']), (0.0, 0.0))
        np.testing.assert_allclose(result['position'], clean['position'], atol=1e-4)  # 配置值=真桌面高度
        raised = self.solve(self.ball, depth, support_z_m=0.01)
        self.assertEqual((raised['table_source'], raised['table_z']), ('config', 0.01))
        self.assertAlmostEqual(raised['height_m'], clean['height_m'] - 0.01, delta=2e-5)  # 确实用了配置值

    def test_too_few_points_returns_none(self):
        rows, cols = np.nonzero(self.ball)
        v, u = int(np.median(rows)), int(np.median(cols))
        patch_ = np.zeros_like(self.ball)
        patch_[v - 3:v + 3, u - 3:u + 3] = True          # 6x6 像素：腐蚀 4 px 后什么都不剩
        result, text = self.logged(
            lambda: [self.solve(patch_), self.solve(patch_, cloud_erode_px=0)])   # 不腐蚀也只有 36 个点
        self.assertEqual(result, [None, None])
        self.assertIn('rejected: no valid depth inside the instance', text)
        self.assertIn('rejected: only 36 object points above the table (need 50)', text)

    def test_height_out_of_range_returns_none(self):
        pad = dict(center=np.array([0.42, 0.0, 0.004]), axis=np.array([0.0, 0.0, 1.0]),
                   half=0.004, radius=0.04)                # 立着的 8 mm 薄圆片
        depth, (mask,) = render(self.t_base_camera, [pad])
        result, text = self.logged(lambda: self.solve(mask, depth))
        self.assertIsNone(result)
        self.assertIn('rejected: height 8 mm outside [10, 120] mm', text)

    def test_erosion_drops_edge_fliers(self):
        edge = self.ball & ~(cv2.erode(self.ball.astype(np.uint8), grasp.disc_kernel(1)) > 0)
        depth = self.depth.astype(float)
        depth[edge] *= 0.7                                 # 边缘一圈飞点：比真实深度近三成（约 6 cm）
        clean, filtered = self.solve(self.ball), self.solve(self.ball, depth)
        self.assertEqual(filtered['n_points'], clean['n_points'])      # 飞点一个没进物体点
        np.testing.assert_allclose(filtered['position'], clean['position'], atol=1e-9)
        result, text = self.logged(lambda: self.solve(self.ball, depth, cloud_erode_px=0))
        self.assertIsNone(result)                                # 对照：不腐蚀时飞点把物高顶出合理范围
        self.assertIn('rejected: height', text)

    def test_finger_floor(self):
        g = dict(self.g, grasp_offset_m=0.040)                        # 同 default.yaml 的 grasp_offset_m
        self.assertAlmostEqual(grasp.finger_floor_z(0.0, g), 0.005 + 0.050 - 0.040)
        self.assertAlmostEqual(grasp.finger_floor_z(0.02, g), 0.035)
        self.assertAlmostEqual(grasp.finger_floor_z(0.0, dict(g, grasp_offset_m=0.060)), -0.005)

    def test_configs_and_defaults_carry_cloud_settings(self):
        self.assertNotIn('top_band', grasp.DEFAULTS)
        for name in ('default.yaml', 'charuco.yaml'):
            config = yaml.safe_load((_ROOT / 'config' / name).read_text(encoding='utf-8'))['grasp']
            self.assertEqual(config['approach'], 'cloud_top_down')
            self.assertIn('banana', config['classes'])
            self.assertNotIn('top_band', config)
            for key, value in CLOUD_DEFAULTS.items():
                self.assertEqual(config[key], YAML_OVERRIDES.get(key, value), f'{name}: grasp.{key}')
                self.assertEqual(grasp.DEFAULTS[key], value, f'DEFAULTS: {key}')

class RingOutlierTests(unittest.TestCase):
    """环带混进台沿 / 桌外：离群剔除后平面仍可用；实在不行退回本次运行的缓存平面，再不行退回 support_z_m。"""

    def setUp(self):
        self.view = camera_pose(40.0)
        self.g = dict(grasp.DEFAULTS)
        grasp._TABLE_CACHE.clear()
        self.ball = dict(center=np.array([0.40, -0.10, 0.03]), radius=0.03)
        self.depth, (self.mask,) = render(self.view, [self.ball])
        self.clean = self.solve(self.mask)

    def tearDown(self):
        grasp._TABLE_CACHE.clear()

    def solve(self, mask, depth=None, view=None, q=None, **overrides):
        g = dict(self.g, **overrides)
        depth = self.depth if depth is None else depth
        camera, rect = camera_measure(depth, mask, g['depth_quantile'])
        return grasp.object_from_cloud(camera['mask'], depth, K, self.view if view is None else view, g,
                                       camera['center_px'], camera['ends_px'], rect, q_deg=q)

    def logged(self, *call):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            result = call[0]()
        return result, out.getvalue()

    def ring(self, mask):
        grown = cv2.dilate(mask.astype(np.uint8), grasp.disc_kernel(self.g['table_ring_outer_px'])) > 0
        kept = cv2.dilate(mask.astype(np.uint8), grasp.disc_kernel(self.g['table_ring_inner_px'])) > 0
        return grown & ~kept

    def with_lip(self, fraction):
        """环带靠左的 fraction 部分变成高 3 cm 的台沿，返回新的深度图。"""
        depth = self.depth.copy()
        ring = self.ring(self.mask)
        lip = ring & (np.arange(depth.shape[1]) < np.quantile(np.nonzero(ring)[1], fraction))
        with np.errstate(divide='ignore', invalid='ignore'):
            depth[lip] = np.rint(((0.03 - self.view[2, 3]) / pixel_rays(self.view)[..., 2])[lip] * 1000)
        return depth

    def test_raised_lip_is_trimmed(self):
        for fraction in (0.1, 0.2, 0.3):
            with self.subTest(fraction):
                depth = self.with_lip(fraction)
                result, text = self.logged(lambda: self.solve(self.mask, depth))
                self.assertIsNotNone(result)
                report(f'ball, {fraction:.0%} of the ring raised 30 mm', result, self.ball['center'])
                self.assertEqual(result['table_source'], 'plane')
                self.assertAlmostEqual(result['ring_kept'], 1 - fraction, delta=0.03)
                self.assertNotIn('unusable', text)
                np.testing.assert_allclose(result['position'], self.clean['position'], atol=0.0005)

    def test_ring_half_off_the_table_is_trimmed(self):
        ball = dict(center=np.array([0.46, 0.205, 0.03]), radius=0.03)     # 摆到桌边，掩码半个环带在桌外
        for y_edge in (0.225, 0.235, 0.245):
            with self.subTest(y_edge):
                depth, (mask,) = render(self.view, [ball], y_edge=y_edge)
                result, text = self.logged(lambda: self.solve(mask, depth))
                self.assertIsNotNone(result)
                err = report(f'ball at the table edge (edge at y={y_edge})', result, ball['center'])
                self.assertEqual(result['table_source'], 'plane')
                self.assertTrue(0.45 <= result['ring_kept'] <= 0.75, result['ring_kept'])
                self.assertLess(np.linalg.norm(err), 3.0, f'center error {np.linalg.norm(err):.2f} mm')

    def test_majority_outliers_reject_the_fit(self):
        # 台沿占到六成：起手落在台沿那一层，平面从球体中间切过 → 这层不能用 → 退回 support_z_m
        depth = self.with_lip(0.6)
        result, text = self.logged(lambda: self.solve(self.mask, depth))
        report('ball, 60% of the ring raised 30 mm', result, self.ball['center'])
        self.assertEqual(result['table_source'], 'config')
        self.assertRegex(text, r'table ring unusable \(fitted plane rejected: \d+% of the instance lies below it\); '
                               r'using support_z_m')
        np.testing.assert_allclose(result['position'], self.clean['position'], atol=0.0005)

    def test_unusable_ring_uses_the_cached_plane(self):
        view = camera_pose_error(self.view, PITCH, ROLL, SINK)
        depth, (mask,) = render(self.view, [self.ball])
        blank = depth.copy()
        blank[~mask] = 0                                              # 掩码外全是无效深度：环带取不到桌面
        q = [10.0, -20.0, 30.0, 5.0, 40.0, 0.0]
        good = self.solve(mask, depth, view=view, q=q)                # 第一次：环带可用，拟合成功
        self.assertEqual(good['table_source'], 'plane')
        np.testing.assert_array_equal(grasp._TABLE_CACHE['q_deg'], q)
        result, text = self.logged(lambda: self.solve(mask, blank, view=view, q=q))
        self.assertEqual(result['table_source'], 'cached')           # 同一位姿再拍：环带废了，用缓存
        self.assertIn('table ring unusable (0 points); using the cached plane from this run', text)
        np.testing.assert_allclose(result['position'], good['position'], atol=1e-6)   # 与第一次拟合一致
        self.assertIsNotNone(result['correction'])                    # 缓存的平面同样参与自校正
        result, text = self.logged(lambda: self.solve(                # 各关节角差都在 1° 内才算同一拍照位姿
            mask, blank, view=view, q=[value + 0.9 for value in q]))
        self.assertEqual(result['table_source'], 'cached')
        for label, pose in (('pose changed by 2 deg', [q[0], q[1] + 2.0, *q[2:]]), ('no joint angles', None)):
            with self.subTest(label):                                # 位姿变了 / 没给关节角：不复用
                result, text = self.logged(lambda: self.solve(mask, blank, q=pose))
                self.assertEqual(result['table_source'], 'config')
                self.assertIn('table ring unusable (0 points); using support_z_m = 0.000 m', text)
        result, text = self.logged(lambda: self.solve(mask, blank, view=view, q=q))   # 再回到原位姿：仍可用
        self.assertEqual(result['table_source'], 'cached')
        err = report('ball, ring without depth + camera pose error, cached plane', good, self.ball['center'])
        self.assertLess(np.linalg.norm(err), 5.0, 'cached plane + self-correction keeps the center')


class TableSelfCorrectTests(unittest.TestCase):
    """桌面自校正：渲染用真值位姿，算法拿到带俯仰 / 横滚 / 下沉误差的位姿。"""

    def setUp(self):
        self.view = camera_pose(40.0)
        self.g = dict(grasp.DEFAULTS)
        grasp._TABLE_CACHE.clear()

    def tearDown(self):
        grasp._TABLE_CACHE.clear()

    def solve(self, obj, view, **overrides):
        g = dict(self.g, **overrides)
        depth, (mask,) = render(self.view, [obj])
        camera, rect = camera_measure(depth, mask, g['depth_quantile'])
        return grasp.object_from_cloud(camera['mask'], depth, K, view, g,
                                       camera['center_px'], camera['ends_px'], rect)

    def test_correction_absorbs_camera_pose_error(self):
        scenes = (('ball', BALL, BALL['center'], 0.060),
                  ('tube along y', TUBE, TUBE['center'], 0.040),
                  ('crescent, chord along y', sweep_scene('crescent', 90, 0.0),
                   sweep_scene('crescent', 90, 0.0)['truth'], 0.036))
        view = camera_pose_error(self.view, PITCH, ROLL, SINK)
        for label, obj, truth, true_height in scenes:
            with self.subTest(label):
                plain = self.solve(obj, view, table_self_correct=False)
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    fixed = self.solve(obj, view)
                self.assertEqual((plain['table_source'], fixed['table_source']), ('plane', 'plane'))
                err_plain = report(f'{label}, camera pitch {PITCH} deg / roll {ROLL} deg / {SINK*1000:.0f} mm sink, '
                                   'no self-correction', plain, truth)
                err = report(f'{label}, same camera error, self-corrected', fixed, truth)
                self.assertGreater(np.linalg.norm(err_plain), 20.0, 'control: uncorrected error')
                self.assertLess(np.linalg.norm(err), 5.0, f'center error {np.linalg.norm(err):.2f} mm, '
                                                          f'xyz {err.round(2).tolist()} mm')
                self.assertLess(abs(fixed['height_m'] - true_height), 0.003,
                                f'height error {(fixed["height_m"] - true_height)*1000:+.1f} mm')
                self.assertLess(abs(fixed['plane_offset_m']), 0.0005)     # 修正后桌面落在 support_z_m 上
                self.assertLess(fixed['plane_tilt_deg'], 0.2)
                self.assertAlmostEqual(fixed['raw_plane_tilt_deg'], np.degrees(np.hypot(np.deg2rad(PITCH), np.deg2rad(ROLL))), delta=0.15)
                self.assertAlmostEqual(fixed['correction']['tilt_deg'], fixed['raw_plane_tilt_deg'], delta=1e-9)
                self.assertRegex(out.getvalue(), r'correction tilt [\d.]+ deg, dz [+-]?[\d.]+ mm; plane offset '
                                                 r'[+-]?[\d.]+ mm tilt [\d.]+ deg before -> [+-]?[\d.]+ mm [\d.]+ deg after')

    def test_zero_error_correction_is_negligible(self):
        for label, obj, truth in (('ball', BALL, BALL['center']), ('tube along y', TUBE, TUBE['center'])):
            with self.subTest(label):
                plain = self.solve(obj, self.view, table_self_correct=False)
                fixed = self.solve(obj, self.view)
                report(f'{label}, no camera error, self-corrected', fixed, truth)
                self.assertLess(fixed['correction']['tilt_deg'], 0.2)
                self.assertLess(abs(fixed['correction']['dz_m']), 0.001)
                np.testing.assert_allclose(fixed['position'], plain['position'], atol=0.001)
                self.assertAlmostEqual(fixed['height_m'], plain['height_m'], delta=0.0005)
                self.assertAlmostEqual(fixed['jaw_m'], plain['jaw_m'], delta=0.0005)

    def test_correction_skipped_above_the_limit(self):
        view = camera_pose_error(self.view, -12.0, 0.0, 0.0)         # 桌面倾角 12° > 上限 10°
        plain = self.solve(TUBE, view, table_self_correct=False)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            result = self.solve(TUBE, view)
        report('tube, 12 deg camera pitch (above the limit), not corrected', result, TUBE['center'])
        self.assertIsNone(result['correction'])
        self.assertIn('table self-correction skipped (tilt 12.0 deg above table_self_correct_max_deg 10)',
                      out.getvalue())
        np.testing.assert_array_equal(result['position'], plain['position'])
        raised = self.solve(TUBE, view, table_self_correct_max_deg=15.0)
        self.assertAlmostEqual(raised['correction']['tilt_deg'], 12.0, delta=0.15)   # 上限放宽到 15° 就修

    def test_yaw_is_not_corrected(self):
        # 绕竖直轴的转动不改变桌面的倾角：桌面照样转平、高度仍对，xy 留着这点偏差（水平偏差用 base_offset_m 补）
        view = camera_pose_error(self.view, 0.0, 0.0, SINK, yaw_deg=YAW)
        result = self.solve(TUBE, view)
        err = report('tube, camera yaw 1 deg + 24 mm sink, self-corrected', result, TUBE['center'])
        self.assertLess(abs(result['plane_offset_m']), 0.0005)
        self.assertLess(abs(err[2]), 1.0)
        plain = self.solve(TUBE, self.view)                         # 同一场景、没有误差时的结果
        angle = np.deg2rad(YAW)
        turned = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        np.testing.assert_allclose(result['position'][:2], turned @ plain['position'][:2], atol=0.0015)
        self.assertGreater(np.linalg.norm(err[:2]), 5.0)           # 没修掉的部分：0.48 m 处转 1° ≈ 8 mm

    def test_correction_skipped_without_a_plane(self):
        depth, (mask,) = render(self.view, [BALL])
        depth[~mask] = 0
        camera, rect = camera_measure(depth, mask, self.g['depth_quantile'])
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            result = grasp.object_from_cloud(camera['mask'], depth, K, self.view, self.g,
                                             camera['center_px'], camera['ends_px'], rect)
        self.assertEqual(result['table_source'], 'config')
        self.assertIsNone(result['correction'])
        self.assertIn('table self-correction skipped (no usable table plane)', out.getvalue())

    def test_table_correction_levels_the_plane(self):
        table = np.array([np.tan(np.deg2rad(4.0)), -np.tan(np.deg2rad(3.0)), -0.021])   # z = a·x + b·y + c
        corr, tilt, dz = grasp.table_correction(table, self.view, 0.0)
        self.assertAlmostEqual(tilt, np.degrees(np.arctan(np.hypot(*table[:2]))), places=9)
        xy = np.random.default_rng(3).uniform(-0.6, 0.6, (500, 2))
        plane = np.column_stack((xy, xy @ table[:2] + table[2]))
        np.testing.assert_allclose((plane @ corr[:3, :3].T + corr[:3, 3])[:, 2], 0.0, atol=1e-9)   # 转平、落到 support_z_m
        center = self.view[:3, 3]
        np.testing.assert_allclose(corr[:3, :3] @ center + corr[:3, 3], center + [0.0, 0.0, dz])   # 光心只沿 z 挪
        self.assertAlmostEqual(dz, -0.4 - np.cos(np.deg2rad(tilt)) * (table[2] - 0.4), places=12)
        corr, tilt, dz = grasp.table_correction(np.array([0.0, 0.0, -0.024]), self.view, 0.0)       # 只下沉、没歪
        np.testing.assert_allclose(corr, np.diag([1.0, 1.0, 1.0, 1.0]) + np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0.024], [0, 0, 0, 0]]), atol=1e-12)
        self.assertEqual((tilt, dz), (0.0, 0.024))

    def test_fit_table_plane_on_synthetic_points(self):
        rng = np.random.default_rng(7)
        xy = rng.uniform(-0.3, 0.3, (2000, 2))
        truth = np.array([0.05, -0.03, 0.2])
        points = np.column_stack((xy, xy @ truth[:2] + truth[2] + rng.normal(0, 0.001, len(xy))))
        table, kept, gap = grasp.fit_table_plane(points, 0.0)
        np.testing.assert_allclose(table, truth, atol=0.002)
        self.assertEqual(kept, 1.0)
        self.assertLess(gap, 0.005)
        outliers = points[:800].copy()                               # 四成离群点落在低 3 cm 的另一层
        outliers[:, 2] -= 0.03
        table, kept, gap = grasp.fit_table_plane(np.vstack((points[800:], outliers)), 0.0)
        np.testing.assert_allclose(table, truth, atol=0.002)
        self.assertAlmostEqual(kept, 0.6, delta=1e-6)
        self.assertLess(gap, 0.005)
        self.assertIsNone(grasp.fit_table_plane(                   # 没有密集的一层：每米才一个点
            np.array([[0.0, 0, 0], [1, 0, 1], [2, 0, 2], [3, 0, 3]]), 0.0))
        print(f'\\n[fit_table_plane] 40% outliers on another level: table {table.round(4).tolist()} '
              f'(truth {truth.tolist()}), kept {kept:.0%}, gap {gap*1000:.1f} mm')


class DetectionFallbackTests(unittest.TestCase):
    """分割没给出掩码时 detections() 退回轴对齐框：要打日志、框画橙色。"""

    class Tensor:
        def __init__(self, v):
            self.v = v

        def cpu(self):
            return self

        def numpy(self):
            return np.array(self.v)

    def result(self, masks=None, box=(10, 20, 60, 90), confidence=0.52):
        class Boxes:
            xyxy, conf, cls = [self.Tensor(list(box))], [confidence], [0]

            def __len__(self):
                return 1

        fields = dict(names={0: 'squeeze ball'}, boxes=Boxes())
        if masks is not None:
            fields['masks'] = masks
        return types.SimpleNamespace(**fields)

    def test_no_mask_logs_reason_once_per_class_and_keeps_axis_aligned_box(self):
        grasp._NO_MASK_LOGGED.clear()
        square = np.array([[20, 30], [50, 30], [50, 60], [20, 60], [20, 30]], np.float32)
        cases = (('masks missing', None),
                 ('0 masks for 1 boxes', types.SimpleNamespace(xy=[])),
                 ('polygon has fewer than 3 points', types.SimpleNamespace(xy=[square[:2]])),
                 ('polygon has non-finite points', types.SimpleNamespace(xy=[np.array([[np.nan, 30], [50, 30], [50, 60], [20, 60], [20, 30]], np.float32)])),
                 ('polygon area below 8 px', types.SimpleNamespace(xy=[np.array([[20, 30], [21, 30], [21, 31], [20, 31], [20, 30]], np.float32)])))
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            for _, masks in cases:
                hits = grasp.detections([self.result(masks)], (480, 640, 3))
                self.assertEqual(len(hits), 1)
                np.testing.assert_array_equal(hits[0][3], [[10, 20], [60, 20], [60, 90], [10, 90]])  # 轴对齐
                self.assertIsNone(hits[0][4])
                self.assertEqual(grasp.detections([self.result(masks)], (480, 640, 3))[0][0], 'squeeze ball')
            text = out.getvalue()                        # 5 种原因 + 1 次重复调用，2 秒内同一类别只打 1 次
        self.assertEqual(text.count('no mask for'), 1)
        self.assertIn('no mask for squeeze ball (conf 0.52), using axis-aligned box: masks missing', text)
        grasp._NO_MASK_LOGGED['squeeze ball'] -= grasp.NO_MASK_LOG_INTERVAL_S   # 2 秒后可以再打
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            grasp.detections([self.result(types.SimpleNamespace(xy=[]))], (480, 640, 3))
        self.assertIn('using axis-aligned box: 0 masks for 1 boxes', out.getvalue())
        for reason, masks in cases[1:]:                  # 其余原因逐个确认日志里写明了
            grasp._NO_MASK_LOGGED.clear()
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                grasp.detections([self.result(masks)], (480, 640, 3))
            self.assertIn(f'using axis-aligned box: {reason}', out.getvalue())
        turned = np.array([[20, 30], [50, 40], [47, 60], [17, 50], [20, 30]], np.float32)   # 斜放的框
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            hits = grasp.detections([self.result(types.SimpleNamespace(xy=[turned]))], (480, 640, 3))
        self.assertEqual(out.getvalue(), '')             # 有掩码：不打日志
        np.testing.assert_array_equal(hits[0][3], cv2.boxPoints(cv2.minAreaRect(turned)))   # 跟着转的框
        np.testing.assert_array_equal(hits[0][4], turned)

    def test_draw_colors_axis_aligned_box_orange(self):
        green, orange = (0, 255, 0), (0, 165, 255)
        rect = np.array([[100, 100], [160, 100], [160, 150], [100, 150]], float)
        depth = np.full((480, 640), 400.0)
        valid = grasp.grasp_camera(rect, depth, K, 0.5, None)
        self.assertIsNotNone(valid)
        colors = []
        with patch.object(grasp.cv2, 'polylines', side_effect=lambda *a, **kw: colors.append(a[3])):
            image = np.zeros((480, 640, 3), np.uint8)
            overlay = grasp.Overlay(*image.shape[:2])
            grasp.draw(image, 'cup', 0.9, rect, valid, True, False, overlay)   # 掩码旋转框 + 有效：绿
            grasp.draw(image, 'cup', 0.9, rect, valid, True, True, overlay)    # 轴对齐框 + 有效：橙
            grasp.draw(image, 'cup', 0.9, rect, None, False, False, overlay)   # 无有效抓取几何：橙
        self.assertEqual(colors, [green, orange, orange])


class CloudDryRunTests(unittest.TestCase):
    """--dry-run 下按 g：假相机回放合成深度、假 YOLO 给出掩码、假臂只答 fk/ik，不产生运动。"""

    class FakeArm:
        """假臂。默认对任何逆解都答零、正解答一个固定位姿，只用来核对规划日志。
        kinematic=True：用仓库 SDK 的真实运动学答 fk/ik（纯计算），流式轨迹才有意义。
        带 grip（夹爪模拟）时记录运动与夹爪命令，并推进模拟时钟 now（流式每帧 50 ms、轮询每拍 100 ms、
        夹爪按行程算），执行段里窗口泵的调用间隔就拿它量。
        in_sequence：一趟抓取执行段进行中；pumping：正处在假臂的某个阻塞等待里。这两种时候 waitKey 是
        执行段的窗口泵，只处理事件，不取按键脚本。"""

        def __init__(self, t_base_tcp, grip=None, kinematic=False, ready=None, cfg=None):
            self.t_base_tcp, self.calls, self.events, self.grip = t_base_tcp, [], [], grip
            self.kinematic, self.ready = kinematic, ready
            self.cfg = dict(acc_dps2=100.0, approach_vel_dps=40.0) if cfg is None else cfg   # 同 RpcArm.cfg 的用法
            self.hold = [0.0] * 6             # 命令角（保持目标）；state() 把它当实测角报回
            self.now = 0.0                    # 模拟时钟（秒）
            self.streamed = []                # 每条流式轨迹：[(关节角, J6 目标), ...]
            self.waits = []                   # wait_still / wait_arrived 的调用（不算命令）
            self.in_sequence = False
            self.pumping = False              # 正处在假臂的某个阻塞等待里（这时 waitKey 是泵，不取按键）
            self.key_times = []               # 每次 waitKey 调用时的模拟时刻

        def wait(self, seconds, pump):
            """模拟一次阻塞调用：期间每 50 ms 泵一次窗口。"""
            end = self.now + seconds
            while self.now < end - 1e-9:
                if pump is not None:
                    self.pumping = True
                    try:
                        pump()
                    finally:
                        self.pumping = False
                self.now = min(end, self.now + 0.05)

        def wait_still(self, target=None, pump=None):
            self.waits.append('wait_still')
            self.wait(0.3, pump)              # 到位 + 稳定窗
            self.in_sequence = False          # 执行段最后一步：回准备位确认到位
            return self.state()

        def wait_arrived(self, target, pump=None):
            self.waits.append('wait_arrived')
            self.wait(0.1, pump)              # 只判到位容差
            return self.state()

        def state(self):
            state = dict(q_deg=list(self.hold), mode='IDLE')
            return dict(state, gripper_mm=self.grip.reading()) if self.grip else state

        def fk(self, q):
            return solve_fk(q) if self.kinematic else as_pose(self.t_base_tcp)

        def call(self, cmd, **kw):
            self.calls.append(cmd)
            self.events.append((cmd, kw.get('mm') if cmd == 'gripper' else None))
            if cmd == 'state':
                return dict(state=self.state())
            if cmd == 'fk':
                return dict(pose=self.fk(kw['q']))
            if cmd == 'ik':
                return dict(q_deg=[0.0] * 6 if not self.kinematic else list(solve_ik(kw['pose'], kw['seed'])))
            if cmd == 'gripper' and self.grip is not None:
                self.wait(self.grip.command(kw['mm']), kw.get('pump'))
            return dict(ok=True)

        def command_q(self):
            return list(self.hold)

        def stream(self, samples, j6=None, pump=None):
            self.calls.append('stream')
            self.events.append(('stream', len(self.streamed)))
            self.in_sequence = True
            rows = [(list(np.asarray(samples, float)[k]), None if j6 is None else j6[k]) for k in range(len(samples))]
            self.streamed.append(rows)
            marks = {}
            for k, (q, j6_deg) in enumerate(rows):
                if pump is not None:
                    pump()
                self.now += 0.05
                self.hold = list(q)
                if j6_deg is not None and (k == 1 or j6_deg != rows[k - 1][1]):
                    marks.setdefault('j6_start_s', self.now)
                    marks['j6_end_s'] = self.now
            return dict(frames=len(rows), **marks)

        def move(self, pose, vel=None, pump=None):
            self.calls.append('move_joints')
            self.events.append(('move_joints', [float(v) for v in pose]))
            pose = [float(v) for v in pose]
            self.in_sequence = self.in_sequence or pose != self.ready   # 启动时去准备位不算执行段
            self.wait(max(0.1, 1.875 * max(abs(a - b) for a, b in zip(pose, self.hold)) / (vel or 90.0)), pump)
            self.hold = list(pose)
            if pose == self.ready:
                self.in_sequence = False       # 回准备位：一趟执行段结束
            return self.state()

        def close(self):
            pass

    def setUp(self):
        self.cfg = app.load_config(_ROOT / 'config' / 'default.yaml', motion=False)
        self.cfg['robot'].update(TIMING_ROBOT)                     # 时序按固定参数算，不随 yaml 调参变
        self.cfg['grasp'].update(TIMING_GRASP)
        self.grasp = dict(grasp.DEFAULTS, **self.cfg['grasp'])     # 同 main 的合并顺序
        grasp._TABLE_CACHE.clear()                                 # 桌面缓存是模块级的：用例之间不串
        self.t_base_camera = camera_pose(40.0)
        self.t_tcp_camera = np.eye(4)                               # 眼在手上：相机相对 TCP 的固定变换
        self.t_tcp_camera[:3, :3] = cv2.Rodrigues(np.array([0.2, -0.1, 0.3]))[0]
        self.t_tcp_camera[:3, 3] = [-0.03, 0.034, 0.05]

    def direct(self, depth_u16, mask, view=None, **overrides):
        """主循环对选中目标的同一串计算，直接调用作为日志数值的对照。"""
        g = dict(self.grasp, **overrides)
        camera, rect = camera_measure(depth_u16, mask, g['depth_quantile'])
        return grasp.object_from_cloud(camera['mask'], np.asarray(depth_u16, float), K,
                                       self.t_base_camera if view is None else view, g,
                                       camera['center_px'], camera['ends_px'], rect)

    def run_main(self, keys, depth_u16, mask, label, segmented=True, view=None, live=False, held_at=None,
                 jitter_mm=0.0, kinematic=False, q_at=None, **overrides):
        """跑一遍 main。live=True 走实机流程（流式去程 → 闭爪 → 流式回程），默认 --dry-run。
        kinematic=True：假臂用 SDK 真实运动学答 fk/ik，手眼矩阵按「臂在准备位时相机正好在该合成视角」反推。
        q_at=n：执行段里第 n 次泵窗口时模拟按下 q（检验它被延后到回准备位之后）。
        按键脚本只在主循环层面的 waitKey 上消耗（' ' = 空按一次）；执行段里泵窗口的调用不取按键。
        返回 (退出码, 日志, 假臂)。"""
        contour = contour_of(mask)
        rows, cols = np.nonzero(mask)
        box = [int(cols.min()), int(rows.min()), int(cols.max()) + 1, int(rows.max()) + 1]

        class Tensor:
            def __init__(self, v):
                self.v = v

            def cpu(self):
                return self

            def numpy(self):
                return np.array(self.v)

        class Boxes:
            xyxy = [Tensor(box)]
            conf, cls = [0.9], [0]

            def __len__(self):
                return 1

        class YOLO:
            names = {0: label}

            def __init__(self, path):
                pass

            def predict(self, image, **kw):
                result = types.SimpleNamespace(names={0: label}, boxes=Boxes())
                if segmented:                              # 没有分割掩码时结果里没有 masks
                    result.masks = types.SimpleNamespace(xy=[contour])
                return [result]

        class Frame:
            def __init__(self, data):
                self.data = data

            def get_data(self):
                return self.data

        class Frames:
            def get_color_frame(self):
                return Frame(np.zeros(depth_u16.shape + (3,), np.uint8))

            def get_depth_frame(self):
                return Frame(depth_u16)

        class Pipeline:
            def start(self, config):
                intr = types.SimpleNamespace(fx=K[0, 0], fy=K[1, 1], ppx=K[0, 2], ppy=K[1, 2])
                stream = types.SimpleNamespace(as_video_stream_profile=lambda: types.SimpleNamespace(
                    get_intrinsics=lambda: intr))
                sensor = types.SimpleNamespace(get_depth_scale=lambda: 0.001)   # 1 raw = 1 mm
                return types.SimpleNamespace(get_stream=lambda s: stream,
                                             get_device=lambda: types.SimpleNamespace(
                                                 first_depth_sensor=lambda: sensor))

            def wait_for_frames(self, timeout_ms=None):
                return Frames()

            def stop(self):
                pass

        ultralytics = types.ModuleType('ultralytics')
        ultralytics.YOLO = YOLO
        rs = types.ModuleType('pyrealsense2')
        rs.pipeline, rs.align = Pipeline, lambda stream: types.SimpleNamespace(process=lambda f: Frames())
        rs.config = type('Config', (), {'enable_device': lambda self, serial: None,
                                        'enable_stream': lambda self, *a: None})
        rs.stream = types.SimpleNamespace(color='color', depth='depth')
        rs.format = types.SimpleNamespace(bgr8='bgr8', z16='z16')
        camera = self.t_base_camera if view is None else view
        ready = [float(v) for v in self.cfg['robot']['ready_pose_deg']]
        # live：夹爪由 FakeGripper 模拟（held_at = 被物体挡住的开度；不给就是抓空）
        grip = (FakeGripper(overrides.get('gripper_open_mm', self.grasp['gripper_open_mm']), held_at, jitter_mm)
                if live else None)
        eye = self.t_tcp_camera
        if kinematic:
            eye = np.linalg.inv(app.pose_matrix(solve_fk(ready))) @ camera
        arm = self.FakeArm(camera @ np.linalg.inv(eye), grip, kinematic, ready, self.cfg['robot'])
        if kinematic:
            arm.hold = list(ready)          # 真实运动学下臂一开始就在准备位（相机视角也是按它反推的）
        script = iter([NO_KEY] + [ord(k) for k in keys])
        armed = []

        def wait_key(ms=1):
            arm.key_times.append(arm.now)
            if arm.in_sequence or arm.pumping:                # 执行段里 / 阻塞等待里：这是泵，不取按键
                if q_at is not None and len(armed) < q_at:
                    armed.append(1)
                    return ord('q') if len(armed) == q_at else NO_KEY
                return NO_KEY
            return next(script)

        with tempfile.TemporaryDirectory() as td:
            config = Path(td) / 'default.yaml'
            cfg = copy.deepcopy(self.cfg)
            cfg['grasp'] = {**cfg['grasp'], 'device': 'cpu', 'classes': [label], 'model': 'models/x.pt',
                            'approach': 'cloud_top_down', **overrides}
            if live:
                cfg['grasp']['model'] = str(config)          # 实机流程要求权重文件存在：拿配置文件顶替
            config.write_text(yaml.safe_dump(cfg), encoding='utf-8')
            np.savez(Path(td) / cfg['output']['filename'], T_tcp_camera=eye)
            out = io.StringIO()
            with contextlib.redirect_stdout(out), \
                 patch.dict('sys.modules', {'ultralytics': ultralytics, 'pyrealsense2': rs}), \
                 patch.object(grasp, 'RpcArm', return_value=arm), \
                 patch.object(grasp.cv2, 'namedWindow'), patch.object(grasp.cv2, 'resizeWindow'), \
                 patch.object(grasp.cv2, 'imshow'), \
                 patch.object(grasp.cv2, 'waitKey', side_effect=wait_key), \
                 patch.object(grasp.cv2, 'getWindowProperty', return_value=1), \
                 patch.object(grasp.cv2, 'destroyAllWindows'):
                code = grasp.main(['--config', str(config)] + ([] if live else ['--dry-run']))
        return code, out.getvalue(), arm

    @staticmethod
    def xyz(text):
        return np.array([float(v) for v in text.split(',')])

    def test_g_plans_on_cloud_center_without_motion(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        grasp._NO_MASK_LOGGED.clear()
        code, out, arm = self.run_main(['g', 'q'], depth_u16, ball, 'ball')
        self.assertEqual(code, 0)
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])        # dry-run：没有任何运动命令
        expected = self.direct(depth_u16, ball)
        line = re.search(CLOUD_LINE.format(label='ball'), out)
        self.assertIsNotNone(line, out)
        center = self.xyz(line['center'])
        np.testing.assert_allclose(center, expected['position'], atol=0.0006)   # 日志保留 3 位小数
        self.assertEqual(line['source'], 'plane')
        self.assertEqual(int(line['points']), expected['n_points'])
        self.assertAlmostEqual(float(line['jaw']) / 100, expected['jaw_m'], delta=0.0006)
        self.assertEqual(line['jaw_source'], 'radius')
        close = grasp.close_margin_mm(expected['jaw_m'], self.grasp['gripper_close_margin_mm'],
                                      self.grasp['gripper_close_min_mm'], self.grasp['gripper_open_mm'])
        self.assertAlmostEqual(float(line['close']), close, delta=0.051)
        self.assertAlmostEqual(float(line['shift']) / 100, abs(expected['along_shift_m']), delta=0.0051)
        self.assertIsNone(line['box'])                               # 有掩码：不是轴对齐框
        self.assertEqual(line['kept'], '100')                        # 干净环带：离群剔除没剔掉点
        self.assertNotIn('no mask for', out)
        self.assertNotIn('raised', out)
        grasp_line = re.search(r'Grasp ball: base xyz \(([^)]*)\)', out)
        offset = self.grasp['grasp_offset_m']                         # TCP 停在抓取点上方 grasp_offset
        np.testing.assert_allclose(self.xyz(grasp_line.group(1)),
                                   expected['position'] + [0, 0, offset], atol=0.0006)
        self.assertIn(f'-> close {close:.1f}mm', out)
        self.assertNotIn('Pre-closing', out)                    # 球宽 6 cm：爪宽 + 余量超过张口，不用预合拢
        print(f'\n[dry-run g on ball] center {center.tolist()} m, TCP {self.xyz(grasp_line.group(1)).tolist()} m, '
              f'close {close:.1f} mm')

    def test_g_dry_run_prints_the_planned_timing(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        code, out, arm = self.run_main(['g', 'q'], depth_u16, ball, 'ball', kinematic=True)
        self.assertEqual(code, 0)
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])       # dry-run：没有任何运动、夹爪、流式命令
        self.assertNotIn('timing: approach', out)                    # 实测计时只在实机流程打
        timing = re.search(r'timing \(planned\): approach (\d+\.\d+)s, retreat (\d+\.\d+)s', out)
        self.assertIsNotNone(timing, out)
        self.assertAlmostEqual(float(timing.group(1)), 4.10, delta=0.15)
        self.assertAlmostEqual(float(timing.group(2)), 3.42, delta=0.15)
        plan = re.search(r'stream plan: approach (\d+\.\d+)s \((\d+) frames, pregrasp point passed at '
                         r'(\d+\.\d+)s with the TCP at (\d+\.\d+) m/s\), retreat (\d+\.\d+)s; '
                         r'corner blend (\d+) mm, at most (\d+\.\d) mm off', out)
        self.assertIsNotNone(plan, out)
        self.assertGreater(int(plan.group(2)), 40)
        self.assertTrue(0.0 < float(plan.group(4)) <= self.grasp['linear_vel_mps'] + 1e-9)   # 不停，且不超过直线段上限
        self.assertLessEqual(float(plan.group(7)), 1.0)              # 预抓取点高度处偏离竖直线 < 1 mm
        self.assertNotIn('j6 preclose', out)                          # 默认余量 0：不预合拢
        print(f'\n[dry-run g on ball, real kinematics] {timing.group(0)} | {plan.group(0)}')

    def test_g_dry_run_j6_preclose_window(self):
        thin = tube_at(90.0, y=-0.10, x=0.40, radius=0.010)          # 爪宽约 2 cm：预合拢到 2 cm + 15 mm
        depth_u16, (tube,) = render(self.t_base_camera, [thin])
        expected = self.direct(depth_u16, tube)
        target = grasp.preclose_target_mm(expected['jaw_m'], 15.0, self.grasp['gripper_open_mm'])
        code, out, arm = self.run_main(['g', 'q'], depth_u16, tube, 'tube', kinematic=True,
                                       gripper_preclose_margin_mm=15)
        self.assertEqual(code, 0)
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])
        self.assertNotIn('Pre-closing', out)                         # 预合拢并在去程里，不再单独一步
        j6 = re.search(r'j6 preclose: (\d+\.\d+)s-(\d+\.\d+)s after the approach begins(.*?)\n', out)
        self.assertIsNotNone(j6, out)
        self.assertEqual(j6.group(1), '0.00')
        travel = abs(gripper_mm_to_deg(target) - gripper_mm_to_deg(self.grasp['gripper_open_mm']))
        self.assertAlmostEqual(float(j6.group(2)), travel / grasp.STREAM_J6_SPEED_DPS + grasp.STREAM_J6_SPEED_DPS
                               / grasp.STREAM_J6_ACC_DPS2, delta=0.051)
        self.assertNotIn('not finished before the pregrasp point', j6.group(3))
        passed = float(re.search(r'pregrasp point passed at (\d+\.\d+)s', out).group(1))
        self.assertLess(float(j6.group(2)), passed)                  # 到预抓取点之前合完
        print(f'\n[dry-run g on a 2 cm tube] jaw {expected["jaw_m"]*1000:.1f} mm -> pre-close {target:.1f} mm: '
              f'{j6.group(0).strip()}; pregrasp at {passed:.2f}s')

    def test_g_raises_low_target_off_the_table(self):
        depth_u16, (tube,) = render(self.t_base_camera, [TUBE])
        code, out, arm = self.run_main(['g', 'q'], depth_u16, tube, 'tube', grasp_offset_m=0.03)
        self.assertEqual(code, 0)
        g = dict(self.grasp, grasp_offset_m=0.03)
        expected = self.direct(depth_u16, tube, grasp_offset_m=0.03)
        floor = grasp.finger_floor_z(g['support_z_m'], g)
        self.assertEqual(floor, 0.005 + 0.050 - 0.03)
        self.assertLess(expected['position'][2], floor)               # 圆柱中线低于下限：要抬高
        self.assertIn(f'Grasp point raised from z {expected["position"][2]:.3f} to {floor:.3f} m', out)
        self.assertRegex(out, rf'Grasp tube: base xyz \([^)]*,[+-]?{floor + 0.03:.3f}\)')
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])
        print(f'\n[dry-run g on tube] center z {expected["position"][2]:.4f} m raised to {floor:.3f} m')

    def test_g_keeps_heights_when_camera_chain_is_offset(self):
        depth_u16, (tube,) = render(self.t_base_camera, [TUBE])
        view = misled(self.t_base_camera, SINK, TILT, TUBE['center'][:2])
        _, plain, _ = self.run_main(['g', 'q'], depth_u16, tube, 'tube')
        # 读数是「不修正」时的坐标链偏差；自校正见 test_g_self_corrects_the_camera_pose
        code, out, arm = self.run_main(['g', 'q'], depth_u16, tube, 'tube', view=view,
                                       table_self_correct=False)
        self.assertEqual(code, 0)
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])
        line = re.search(CLOUD_LINE.format(label='tube'), out)
        self.assertEqual(line['source'], 'plane')
        self.assertAlmostEqual(float(line['offset']), -SINK * 1000, delta=3.0)   # 坐标链偏差读数
        self.assertAlmostEqual(float(line['tilt']), TILT, delta=0.3)
        tcp = lambda text: self.xyz(re.search(r'Grasp tube: base xyz \(([^)]*)\)', text).group(1))
        self.assertAlmostEqual(tcp(out)[2], tcp(plain)[2], delta=0.0006)     # 目标高度不跟着桌面往下走
        self.assertAlmostEqual(float(self.xyz(line['center'])[2]), tcp(plain)[2] - self.grasp['grasp_offset_m'],
                               delta=0.0006)
        self.assertNotIn('raised', out)
        print(f'\n[dry-run g on tube, chain {SINK*1000:.0f} mm low / {TILT} deg, no self-correction] '
              f'center z {self.xyz(line["center"])[2]:.3f} m, TCP z {tcp(out)[2]:.3f} m '
              f'(no error: {tcp(plain)[2]:.3f} m), plane offset {line["offset"]} mm, tilt {line["tilt"]} deg')

    def test_g_self_corrects_the_camera_pose(self):
        depth_u16, (tube,) = render(self.t_base_camera, [TUBE])
        view = camera_pose_error(self.t_base_camera, PITCH, ROLL, SINK)
        tcp = lambda text: self.xyz(re.search(r'Grasp tube: base xyz \(([^)]*)\)', text).group(1))
        _, clean, _ = self.run_main(['g', 'q'], depth_u16, tube, 'tube')
        _, plain, _ = self.run_main(['g', 'q'], depth_u16, tube, 'tube', view=view,
                                    table_self_correct=False)
        code, out, arm = self.run_main(['g', 'q'], depth_u16, tube, 'tube', view=view)
        self.assertEqual(code, 0)
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])
        line = re.search(CLOUD_LINE.format(label='tube'), out)
        self.assertEqual(line['source'], 'plane')
        self.assertLess(abs(float(line['offset'])), 0.5)                   # 修正后：桌面平了、高度对上 support_z_m
        self.assertLess(float(line['tilt']), 0.2)
        self.assertNotIn('self-correction skipped', out)
        correction = re.search(r'correction tilt ([\d.]+) deg, dz ([+-]?[\d.]+) mm; plane offset '
                               r'([+-]?[\d.]+) mm tilt ([\d.]+) deg before -> ([+-]?[\d.]+) mm '
                               r'([\d.]+) deg after', out)
        self.assertIsNotNone(correction, out)
        self.assertAlmostEqual(float(correction.group(1)), np.degrees(np.hypot(np.deg2rad(PITCH), np.deg2rad(ROLL))), delta=0.15)
        self.assertLess(abs(float(correction.group(5))), 0.6)
        self.assertLess(float(correction.group(6)), 0.2)
        self.assertGreater(abs(tcp(plain)[0] - tcp(clean)[0]), 0.020)      # 对照：不修正时 x 偏 2 cm 以上
        np.testing.assert_allclose(tcp(out), tcp(clean), atol=0.005)       # 修正后与没有误差那次一致
        print(f'\n[dry-run g on tube, camera pitch {PITCH} deg / roll {ROLL} deg / {SINK*1000:.0f} mm sink] '
              f'TCP {tcp(clean).round(4).tolist()} (no error) / {tcp(plain).round(4).tolist()} (no self-correction) / '
              f'{tcp(out).round(4).tolist()} (self-corrected); {correction.group(0)}')

    def test_g_uses_axis_aligned_box_when_mask_is_missing(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        grasp._NO_MASK_LOGGED.clear()
        code, out, arm = self.run_main(['g', 'q'], depth_u16, ball, 'ball', segmented=False)
        self.assertEqual(code, 0)
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])        # 不拒绝该帧，照常规划
        self.assertIn('no mask for ball (conf 0.90), using axis-aligned box: masks missing', out)
        self.assertIn('long axis unavailable (ends_px missing); keeping the image-center point', out)
        line = re.search(CLOUD_LINE.format(label='ball'), out)
        self.assertIsNotNone(line, out)
        self.assertEqual(line['box'], ', box=aabb')
        self.assertIn('Pregrasp', out)

    def test_g_skips_frame_when_cloud_is_unusable(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        table, _ = render(self.t_base_camera, [])
        depth_u16 = depth_u16.copy()
        depth_u16[ball] = table[ball]                        # 掩码内深度被抹平成桌面：物体点为 0
        code, out, arm = self.run_main(['g', 'q'], depth_u16, ball, 'ball')
        self.assertEqual(code, 0)
        self.assertIn('cloud_top_down rejected: only 0 object points above the table (need 50); '
                      'frame skipped', out)
        self.assertNotIn('Pregrasp', out)
        self.assertEqual(arm.calls, ['fk'])                   # 没往下规划

    def test_approach_validation_and_older_modes_unchanged(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        grasp._NO_MASK_LOGGED.clear()
        with self.assertRaisesRegex(ValueError,
                                    'grasp.approach must be top_down, camera_ray or cloud_top_down'):
            self.run_main(['q'], depth_u16, ball, 'ball', approach='cloud')
        for approach in ('top_down', 'camera_ray'):
            with self.subTest(approach):
                code, out, arm = self.run_main(['g', 'q'], depth_u16, ball, 'ball', approach=approach)
                self.assertEqual(code, 0)
                self.assertIn('Pregrasp', out)
                self.assertIn('final motion=linear', out)
                self.assertNotIn('cloud_top_down', out)
                self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])
                # 旧模式闭口仍是 jaw×系数（cloud_top_down 才是 jaw 减余量）；日志里爪宽只留一位小数
                jaw_cm, close = re.search(r'jaw ([\d.]+)cm -> close ([\d.]+)mm', out).groups()
                self.assertAlmostEqual(float(close), grasp.close_target_mm(
                    float(jaw_cm) / 100, self.grasp['gripper_close_factor'],
                    self.grasp['gripper_close_min_mm'], self.grasp['gripper_open_mm']), delta=0.4)
                self.assertNotIn('jaw (radius)', out)
                self.assertNotIn('Pre-closing', out)                # 预合拢只有 cloud_top_down 做

    def test_dry_run_r_reports_both_states(self):
        depth_u16, (tube,) = render(self.t_base_camera, [TUBE])
        code, out, arm = self.run_main(['r', 'g', 'r', 'q'], depth_u16, tube, 'tube')
        self.assertEqual(code, 0)
        self.assertEqual(out.count('Nothing held'), 2)              # dry-run 不会真的夹起来：r 只提示
        self.assertNotIn('Releasing', out)
        self.assertIn('Pregrasp', out)                              # 中间那次 g 照常规划
        self.assertEqual(sorted(set(arm.calls)), ['fk', 'ik'])      # 没有任何夹爪命令
        # 夹着的状态在 dry-run 下到不了（不执行抓取序列）：release 的两种模式直接核对
        arm = self.FakeArm(np.eye(4))
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            grasp.release(arm, 65.0, dry_run=True)
            grasp.release(arm, 65.0)
        self.assertEqual(arm.events, [('gripper', 65.0)])
        self.assertEqual(out.getvalue().count('Releasing at the ready pose: opening the gripper to 65.0mm'), 2)

    def ball_plan(self, depth_u16, ball):
        """同一场景下 plan_grasp 会给出的规划（真实运动学），用来核对流式轨迹的端点。"""
        expected = self.direct(depth_u16, ball)
        nominal = grasp._tcp_frame(np.array([0.0, 0.0, -1.0]), expected['open_dir'], self.grasp['open_axis'])
        ready = [float(v) for v in self.cfg['robot']['ready_pose_deg']]
        return grasp.plan_grasp(self.FakeArm(np.eye(4), None, True, ready), expected['position'], nominal, ready,
                                self.grasp)

    def test_live_cycle_streams_holds_releases(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        code, out, arm = self.run_main(['g', 'g', 'r', 'q'], depth_u16, ball, 'ball', live=True, kinematic=True,
                                       held_at=58.0)
        self.assertEqual(code, 0)
        ready = [float(v) for v in self.cfg['robot']['ready_pose_deg']]
        plan = self.ball_plan(depth_u16, ball)
        self.assertIsNotNone(plan)
        # 张口 → 去准备位 →（g）流式去程 → 只判到位 → 闭爪 → 锁停住角 → 流式回程 → 确认准备位到位 →（g 被拒）→（r）开爪 →（q）开爪回安全位
        order = [cmd for cmd, _ in arm.events if cmd not in ('fk', 'ik', 'state')]
        self.assertEqual(order, ['gripper', 'move_joints', 'stream', 'gripper', 'gripper', 'stream', 'gripper',
                                 'gripper', 'move_joints'])
        gripper = [value for cmd, value in arm.events if cmd == 'gripper']
        expected = self.direct(depth_u16, ball)
        close = grasp.close_margin_mm(expected['jaw_m'], self.grasp['gripper_close_margin_mm'],
                                      self.grasp['gripper_close_min_mm'], self.grasp['gripper_open_mm'])
        np.testing.assert_allclose(gripper, [65.0, close, 58.0, 65.0, 65.0], atol=1e-6)
        self.assertEqual(arm.calls.count('stream'), 2)
        self.assertEqual(arm.waits.count('wait_arrived'), 1)         # 抓取点：只判一次到位容差
        self.assertEqual(arm.waits.count('wait_still'), 2)           # 启动时一次 + 回准备位一次
        approach, retreat = [[q for q, _ in rows] for rows in arm.streamed]
        np.testing.assert_allclose(approach[0], ready, atol=1e-9)     # 从准备位（保持目标）起步
        np.testing.assert_allclose(approach[-1], plan['end_q'], atol=1e-9)
        np.testing.assert_allclose(retreat[0], plan['end_q'], atol=1e-9)
        np.testing.assert_allclose(retreat[-1], ready, atol=1e-9)
        self.assertTrue(all(j6 is None for rows in arm.streamed for _, j6 in rows))   # 余量 0：J6 全程不动
        self.assertRegex(out, r'timing: approach \d+\.\d\ds, grip \d+\.\d\ds, retreat \d+\.\d\ds, '
                              r'total \d+\.\d\ds')
        self.assertNotIn('j6 preclose', out)
        self.assertIn('Holding at the stopped angle: 58.0mm', out)
        self.assertIn('Holding the target; press r to release, then g for the next grasp', out)
        self.assertIn('Gripper is holding the previous target; r releases it here, then g for the next grasp', out)
        self.assertIn('Releasing at the ready pose: opening the gripper to 65.0mm', out)
        self.assertIn('Finished (1 grasped, 0 missed)', out)
        timing = re.search(r'timing: (approach [\d.]+s, grip [\d.]+s, retreat [\d.]+s, total [\d.]+s)', out)
        print(f'\n[live cycle, real kinematics] approach {len(approach)} frames, retreat {len(retreat)} frames; '
              f'{timing.group(1)} (simulated clock)')

    def test_window_is_pumped_through_the_whole_sequence(self):
        # 执行段里每个阻塞等待每拍都调 waitKey：模拟时钟上相邻两次调用的间隔不超过 0.5 s（GNOME 5 s 判未响应）
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        code, out, arm = self.run_main([' ', 'g', 'q'], depth_u16, ball, 'ball', live=True, kinematic=True,
                                       held_at=58.0)
        self.assertEqual(code, 0)
        self.assertIn('timing: approach', out)
        self.assertGreater(arm.key_times[-1], 10.0)                 # 整趟（去准备位 + 抓取）10 s 以上
        gaps = np.diff(arm.key_times)
        self.assertLessEqual(gaps.max(), 0.5, f'longest gap without a waitKey call {gaps.max():.2f} s')
        during = np.array(arm.key_times)
        self.assertGreater(len(during), 200)                        # 每帧 / 每次轮询都泵了
        print(f'\n[window pumping] {len(during)} waitKey calls over {during[-1]:.1f} s of simulated time; '
              f'longest gap {gaps.max():.2f} s')

    def test_q_during_the_sequence_runs_after_the_ready_pose(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        code, out, arm = self.run_main(['g', ' '], depth_u16, ball, 'ball', live=True, kinematic=True, held_at=58.0,
                                       q_at=30)                     # 执行段里第 30 次泵窗口时按 q（去程流式中）
        self.assertEqual(code, 0)
        self.assertEqual([cmd for cmd, _ in arm.events if cmd not in ('fk', 'ik', 'state')][2:],
                         ['stream', 'gripper', 'gripper', 'stream', 'gripper', 'move_joints'])   # 这趟抓取照常走完，再开爪回安全位
        self.assertLess(out.index('timing: approach'), out.index('Opening gripper, returning to safe pose (q pressed '
                                                                 'during the grasp sequence)'))
        self.assertNotIn('FAILED', out)
        self.assertIn('Finished (1 grasped, 0 missed)', out)

    def test_live_miss_keeps_the_session_going(self):
        thin = tube_at(90.0, y=-0.10, x=0.40, radius=0.010)          # 2 cm 细管：闭口上限落到下限 15 mm，抓空时停在它上面
        depth_u16, (tube,) = render(self.t_base_camera, [thin])
        expected = self.direct(depth_u16, tube)
        close = grasp.close_margin_mm(expected['jaw_m'], self.grasp['gripper_close_margin_mm'],
                                      self.grasp['gripper_close_min_mm'], self.grasp['gripper_open_mm'])
        self.assertEqual(close, self.grasp['gripper_close_min_mm'])
        code, out, arm = self.run_main(['g', 'g', 'q'], depth_u16, tube, 'tube', live=True, kinematic=True)
        self.assertEqual(code, 0)                                   # 抓空不退出，第二次 g 照常再抓
        self.assertEqual(out.count(f'MISS: Gripper closed to {close:.1f}mm (<= minimum 15+2mm): nothing held; '
                                   'jaws opened'), 2)
        self.assertEqual(out.count('Missed; press g for the next grasp'), 2)
        self.assertEqual(arm.calls.count('stream'), 4)              # 每次都是去程 + 回程
        self.assertNotIn('FAILED', out)
        self.assertIn('Finished (0 grasped, 2 missed)', out)
        np.testing.assert_allclose([value for cmd, value in arm.events if cmd == 'gripper'],
                                   [65.0] + [close, 65.0] * 2 + [65.0], atol=1e-6)

    def test_live_unsettled_grip_still_exits(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        code, out, arm = self.run_main(['g', ' '], depth_u16, ball, 'ball', live=True, kinematic=True, held_at=58.0,
                                       jitter_mm=0.5)
        self.assertEqual(code, 2)                                   # 闭爪一直不稳：仍是故障退出，不是抓空
        self.assertIn('FAILED: Gripper did not settle within 2s; jaws opened, no hold', out)
        self.assertNotIn('MISS', out)
        self.assertEqual(arm.calls.count('stream'), 1)              # 回程没走
        gripper = [value for cmd, value in arm.events if cmd == 'gripper']
        self.assertEqual(gripper[-1], 65.0)                         # 松爪了

    def test_live_joint_mode_runs_the_old_segments(self):
        depth_u16, (ball,) = render(self.t_base_camera, [BALL])
        code, out, arm = self.run_main(['g', 'q'], depth_u16, ball, 'ball', live=True, final_motion='joint')
        self.assertEqual(code, 0)
        self.assertNotIn('stream', arm.calls)
        self.assertEqual(arm.waits, ['wait_still'])                  # 只有启动时那一次；分段走的 move 自带到位确认
        self.assertEqual([cmd for cmd, _ in arm.events if cmd not in ('fk', 'ik', 'state')][:6],
                         ['gripper', 'move_joints', 'move_joints', 'move_joints', 'gripper', 'gripper'])
        self.assertIn('Retreating to the ready pose via the pregrasp point', out)
        self.assertRegex(out, r'timing: approach \d+\.\d\ds, grip \d+\.\d\ds, retreat \d+\.\d\ds')



class KinArm:
    """用仓库 SDK 真实运动学答 fk/ik 的假臂（纯计算，不连臂），给 plan_grasp / plan_stream 用。"""

    def __init__(self, hold, acc_dps2=100.0, approach_vel_dps=40.0):
        self.hold, self.cfg = list(hold), dict(acc_dps2=acc_dps2, approach_vel_dps=approach_vel_dps)

    def call(self, cmd, **kw):
        if cmd == 'fk':
            return dict(pose=solve_fk(kw['q']))
        if cmd == 'ik':
            return dict(q_deg=list(solve_ik(kw['pose'], kw['seed'])))
        raise app.RobotFault(f'unsupported command: {cmd}')

    def command_q(self):
        return list(self.hold)


class StreamTrajectoryTests(unittest.TestCase):
    """流式轨迹（真实运动学）：过预抓取点不停、下降段是竖直直线、速度加速度不超限、J6 预合拢。"""

    READY = [0.0, -40.0, 40.0, 0.0, 40.0, 0.0]

    def setUp(self):
        self.g = {**grasp.DEFAULTS, **app.load_config(_ROOT / 'config' / 'default.yaml', motion=False)['grasp'],
                  **TIMING_GRASP}                                     # 时序按固定参数算，不随 yaml 调参变

    def plan(self, center, open_dir=(1.0, 0.0, 0.0), start=None):
        arm = KinArm(self.READY if start is None else start)
        nominal = grasp._tcp_frame(np.array([0.0, 0.0, -1.0]), np.array(open_dir, float), self.g['open_axis'])
        plan = grasp.plan_grasp(arm, np.asarray(center, float), nominal, self.READY if start is None else start, self.g)
        self.assertIsNotNone(plan, 'the scenario must be reachable with a straight vertical grasp')
        return arm, plan

    def tcp(self, frames):
        return np.array([_KIN.fk(np.deg2rad(q))[:3, 3] for q in frames])

    def check(self, label, arm, plan, motion, v_home):
        stamps = motion['stamps']
        v, a = grasp.discrete_limits(motion['approach'], stamps)
        self.assertLessEqual(v.max(), self.g['grasp_vel_dps'] + 1e-6, f'{label}: joint speed {v.tolist()}')
        self.assertLessEqual(a.max(), arm.cfg['acc_dps2'] + 1e-6, f'{label}: joint acceleration {a.tolist()}')
        v, a = grasp.discrete_limits(motion['retreat'], motion['retreat_stamps'])
        self.assertLessEqual(v.max(), v_home + 1e-6, f'{label}: retreat joint speed {v.tolist()}')
        self.assertLessEqual(a.max(), arm.cfg['acc_dps2'] + 1e-6, f'{label}: retreat joint acceleration {a.tolist()}')
        np.testing.assert_allclose(motion['approach'][0], motion['retreat'][-1], atol=1e-9)   # 准备位
        np.testing.assert_allclose(motion['approach'][-1], plan['end_q'], atol=1e-9)
        np.testing.assert_allclose(motion['retreat'][0], plan['end_q'], atol=1e-9)
        line = (plan['pre'] - plan['end']) / np.linalg.norm(plan['pre'] - plan['end'])       # 竖直向上
        report = []
        for name, frames, times in (('approach', motion['approach'], stamps),
                                    ('retreat', motion['retreat'], motion['retreat_stamps'])):
            tcp = self.tcp(frames)
            below = np.nonzero((tcp - plan['pre']) @ line <= 0)[0]       # 没过预抓取点高度的帧
            k = int(below[0] if name == 'approach' else below[-1])       # 跨过预抓取点高度的那一帧附近
            self.assertTrue(5 < k < len(frames) - 5, f'{label}: the {name} must cross the pregrasp height')
            on_line = tcp[k:] if name == 'approach' else tcp[:k + 1]     # 预抓取点高度及以下的那一段
            off = np.linalg.norm(np.cross(on_line - plan['end'], line), axis=1)
            self.assertLess(off.max(), 0.001, f'{label}: {name} strays {off.max() * 1000:.2f} mm off the vertical line')
            speed = np.linalg.norm(np.diff(tcp, axis=0), axis=1) / np.diff(times)
            straight = speed[k:] if name == 'approach' else speed[:k + 1]
            self.assertLessEqual(straight.max(), self.g['linear_vel_mps'] * 1.001,
                                 f'{label}: TCP speed on the straight segment {straight.max():.3f} m/s')
            at_pre = speed[k - 1] if name == 'approach' else speed[k]    # 跨过预抓取点高度那一帧的末端速度
            self.assertGreater(at_pre, 0.02, f'{label}: {name} stops at the pregrasp point')
            report.append(f'{name}: {len(frames)} frames {times[-1]:.2f}s, TCP at pre {at_pre:.3f} m/s, '
                          f'max TCP {speed.max():.3f} m/s, {off.max() * 1000:.2f} mm off the line')
        print(f'\n[stream trajectory: {label}] ' + ' | '.join(report)
              + f" | pre at {motion['t_pre']:.2f}s, blend {motion['blend_m'] * 1000:.1f} mm, "
                f"deviation {motion['deviation_m'] * 1000:.2f} mm")

    def test_ball_approach_and_retreat(self):
        arm, plan = self.plan((0.40, -0.10, 0.03))
        motion = grasp.plan_stream(arm, plan, self.g, None, self.g['gripper_open_mm'], home_q=self.READY)
        self.check('ball', arm, plan, motion, arm.cfg['approach_vel_dps'])
        self.assertIsNone(motion['j6'])
        v, a = grasp.discrete_limits(motion['approach'], motion['stamps'])
        self.assertAlmostEqual(v[1], self.g['grasp_vel_dps'], delta=0.5)   # 关节直线段跑到了配置速度
        self.assertAlmostEqual(motion['approach_s'], 4.10, delta=0.15)
        self.assertAlmostEqual(motion['retreat_s'], 3.42, delta=0.15)

    def test_other_placements(self):
        for label, center, open_dir in (('tube along y, 13 cm off', (0.40, 0.13, 0.02), (1.0, 0.0, 0.0)),
                                        ('tube along y, on axis', (0.40, 0.0, 0.02), (1.0, 0.0, 0.0)),
                                        ('tube turned 45 deg', (0.43, 0.0, 0.02), (0.7, 0.7, 0.0))):
            with self.subTest(label):
                arm, plan = self.plan(center, open_dir)
                motion = grasp.plan_stream(arm, plan, self.g, None, self.g['gripper_open_mm'], home_q=self.READY)
                self.check(label, arm, plan, motion, arm.cfg['approach_vel_dps'])

    def test_j6_preclose_rides_on_the_approach(self):
        arm, plan = self.plan((0.40, -0.10, 0.02))
        target = grasp.preclose_target_mm(0.020, 15.0, self.g['gripper_open_mm'])
        motion = grasp.plan_stream(arm, plan, self.g, target, self.g['gripper_open_mm'], home_q=self.READY)
        j6 = np.array(motion['j6'])
        self.assertEqual(j6.shape, (len(motion['approach']),))
        self.assertAlmostEqual(j6[0], gripper_mm_to_deg(self.g['gripper_open_mm']), places=9)
        self.assertAlmostEqual(j6[-1], gripper_mm_to_deg(target), places=9)
        self.assertTrue((np.diff(j6) <= 0).all())                        # 一路合拢
        self.assertLessEqual(np.abs(np.diff(j6)).max(), 20.0 * grasp.STREAM_DT_S + 1e-6)   # 每帧 ≤ 20°/s 对应的 1°
        self.assertLess(motion['j6_window'][1], motion['t_pre'])          # 到预抓取点之前合完
        self.assertIsNone(grasp.plan_stream(arm, plan, self.g, None, 65.0)['j6'])   # 不预合拢：J6 没有目标
        print(f'\n[j6 preclose] {self.g["gripper_open_mm"]:.0f} -> {target:.1f} mm = {j6[0]:.1f} -> {j6[-1]:.1f} deg, '
              f'done at {motion["j6_window"][1]:.2f}s, pregrasp at {motion["t_pre"]:.2f}s, '
              f'largest step {np.abs(np.diff(j6)).max():.3f} deg/frame')

    def test_retreat_ends_at_a_different_ready_pose(self):
        arm, plan = self.plan((0.40, -0.10, 0.03))
        home = [10.0, -35.0, 35.0, 0.0, 40.0, 0.0]
        motion = grasp.plan_stream(arm, plan, self.g, None, self.g['gripper_open_mm'], home_q=home)
        np.testing.assert_allclose(motion['retreat'][-1], home, atol=1e-9)
        np.testing.assert_allclose(motion['approach'][0], self.READY, atol=1e-9)

    def test_degenerate_path_is_refused(self):
        arm, plan = self.plan((0.40, -0.10, 0.03))
        flat = dict(plan, pre_q=np.zeros(6), end_q=np.zeros(6), line_q=np.zeros((6, 6)))
        with self.assertRaisesRegex(ValueError, 'degenerate path'):
            grasp.plan_stream(arm, flat, self.g, None, self.g['gripper_open_mm'])


class FakeD1Arm:
    """假 D1Arm（给 RpcArm.stream 用）：记命令、翻控制器状态；可设成迟迟不进 STREAMING / 中途掉出。"""

    def __init__(self, online=None):
        self.log, self.seq, self.ctrl, self.streaming = [], 0, 1, False
        self.online = [True] * 7 if online is None else online
        self.enter_after, self.drop_at, self.frames, self.sent = 0, None, 0, 0
        self.stop_fails, self.gripper_fails = False, False
        self.q = [0.0] * 6                                            # 实测角（假臂不动的轴）

    def state(self, timeout_s=5.0):
        return dict(q_deg=list(self.q), online=list(self.online), stamp_ms=self.seq * 50,
                    mode={1: 'IDLE', 2: 'STREAMING', 3: 'HOLD', 6: 'ESTOP'}[self.ctrl], ctrl_state=self.ctrl,
                    gripper_mm=65.0)

    def stream_start(self):
        self.log.append('stream_start')
        self.ctrl, self.streaming = 3, True

    def stream_joints(self, q, j6_deg=None):
        if not self.streaming:
            raise D1ArmError('未在流式（先 stream_start()）')
        self.log.append(('q', list(q), j6_deg))

    def wait_frame(self, timeout=1.0):
        self.log.append('frame')
        self.seq += 1
        self.frames += 1
        if self.streaming:                       # SDK 发送线程：每收到一帧就把最新流式目标再发一次
            self.sent += 1
            if self.sent > self.enter_after:     # 控制器收到第 enter_after+1 条 STREAM 才进 STREAMING
                self.ctrl = 2
        if self.drop_at is not None and self.frames == self.drop_at:
            self.ctrl = 3

    def stream_stop(self):
        self.log.append('stream_stop')
        self.streaming = False
        if self.stop_fails:
            raise D1ArmError('stream_stop 失败：5s 内 ctrl_state 没进入 HOLD')
        self.ctrl = 3

    def estop(self):
        self.log.append('estop')
        self.streaming, self.ctrl = False, 6

    def gripper(self, mm):
        if self.gripper_fails:
            raise D1ArmError('gripper 失败')
        time.sleep(0.15)
        return {'ok': True}

    def close(self):
        pass


class RpcArmStreamTests(unittest.TestCase):
    """适配层的流式通道：一帧一个目标、先等帧再发、异常必收尾、降级不进流式、带 pump 的等待。"""

    CFG = dict(request_timeout_s=5.0, offline_grace_s=5.0, move_timeout_s=0.5, arrival_tolerance_deg=1.8,
               poll_s=0.02, settle_s=0.3, stable_tolerance_deg=0.3)

    def setUp(self):
        self.sdk = FakeD1Arm()
        self.arm = app_robot.RpcArm(dict(self.CFG), arm=self.sdk)
        self.pumps = 0

    def pump(self):
        self.pumps += 1

    @staticmethod
    def names(log):
        return [x if isinstance(x, str) else 'q' for x in log]

    def test_one_target_per_frame(self):
        samples = np.arange(30.0).reshape(5, 6) / 100
        result = self.arm.stream(samples, pump=self.pump)
        self.assertEqual(self.names(self.sdk.log), ['stream_start', 'q', 'frame', 'frame', 'q', 'frame', 'q', 'frame',
                                                    'q', 'frame', 'q', 'frame', 'stream_stop'])
        sent = [x for x in self.sdk.log if isinstance(x, tuple)]
        np.testing.assert_allclose([q for _, q, _ in sent], samples)
        self.assertTrue(all(j6 is None for _, _, j6 in sent))          # 不给 J6：保持上次命令角
        self.assertEqual(result['frames'], 5)
        self.assertNotIn('j6_start_s', result)
        self.assertGreaterEqual(self.pumps, 4)                         # 之后每帧各泵一次

    def test_j6_targets_and_marks(self):
        samples = np.zeros((4, 6))
        result = self.arm.stream(samples, j6=[None, 50.0, 49.0, 49.0])
        self.assertEqual([item[2] for item in self.sdk.log if isinstance(item, tuple)],
                         [None, 50.0, 49.0, 49.0])
        self.assertIn('j6_start_s', result)
        self.assertIn('j6_end_s', result)
        with self.assertRaisesRegex(ValueError, 'one entry per sample'):
            self.arm.stream(samples, j6=[50.0])
        with self.assertRaisesRegex(ValueError, 'N>=2'):
            self.arm.stream(samples[:1])

    def test_waits_for_streaming_before_moving(self):
        self.sdk.enter_after = 3                                      # 第 4 个目标之后控制器才进 STREAMING
        self.arm.stream(np.zeros((3, 6)))
        self.assertEqual(self.names(self.sdk.log)[:7], ['stream_start', 'q', 'frame', 'frame', 'frame', 'frame',
                                                         'frame'])                  # 首样本由发送线程重发到进入 STREAMING
        self.assertEqual(self.names(self.sdk.log).index('q', 2), 7)   # 进入之后才发第 2 个目标
        self.sdk = FakeD1Arm()
        self.sdk.enter_after = 10 ** 6                                # 永远不进：60 帧后放弃，转急停
        arm = app_robot.RpcArm(dict(self.CFG), arm=self.sdk)
        with self.assertRaisesRegex(app.RobotFault, 'did not enter STREAMING within 60 frames'):
            arm.stream(np.zeros((3, 6)))
        self.assertEqual(self.sdk.log[-1], 'estop')

    def test_fault_midway_ends_with_estop(self):
        self.sdk.drop_at = 3                                          # 第 3 帧后控制器掉出 STREAMING
        with self.assertRaisesRegex(app.RobotFault, 'left STREAMING'):
            self.arm.stream(np.zeros((5, 6)), pump=self.pump)
        self.assertEqual(self.sdk.log[-1], 'estop')
        self.assertFalse(self.sdk.streaming)

    def test_pump_failure_and_ctrl_c(self):
        # 两种都直接 estop：清掉 SDK 流式状态、控制器立刻停，同以前分段走时 Ctrl-C / 故障的软急停终态
        def boom():
            raise RuntimeError('window gone')

        with self.assertRaisesRegex(RuntimeError, 'window gone'):
            self.arm.stream(np.zeros((5, 6)), pump=boom)
        self.assertEqual(self.sdk.log[-1], 'estop')

        def ctrl_c():
            raise KeyboardInterrupt

        self.sdk = FakeD1Arm()
        arm = app_robot.RpcArm(dict(self.CFG), arm=self.sdk)
        with self.assertRaises(KeyboardInterrupt):
            arm.stream(np.zeros((5, 6)), pump=ctrl_c)
        self.assertEqual(self.sdk.log[-1], 'estop')                   # Ctrl-C：同以前，落到软急停
        self.assertTrue(arm.self_estop)

    def test_cleanup_failure_is_logged_not_raised(self):
        self.sdk.stop_fails = True
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.arm.stream(np.zeros((3, 6)))
        self.assertIn('WARN: stream_stop after the streamed trajectory failed', out.getvalue())

    def test_degraded_telemetry_refuses_to_stream(self):
        self.sdk.online[2] = False
        with self.assertRaisesRegex(app.RobotFault, 'Servo feedback degraded'):
            self.arm.stream(np.zeros((3, 6)))
        self.assertEqual(self.sdk.log, [])

    def test_wait_arrived_checks_tolerance_only(self):
        self.sdk.q = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]                     # J1 差 1°（容差 1.8°）
        started = time.monotonic()
        self.arm.wait_arrived([0.0] * 6, pump=self.pump)
        quick = time.monotonic() - started
        started = time.monotonic()
        with contextlib.redirect_stdout(io.StringIO()):
            self.arm.wait_still([0.0] * 6, pump=self.pump)             # 对照：稳定窗至少要 settle_s
            settled = time.monotonic() - started
        self.assertLess(quick, 0.1)
        self.assertGreaterEqual(settled, self.CFG['settle_s'])
        self.sdk.q = [4.0, 0.0, 0.0, 0.0, 0.0, 0.0]                     # 差 4°：超时
        self.pumps = 0
        with self.assertRaisesRegex(app.RobotFault, 'Not at the end of the streamed trajectory within 0.5s; '
                                                     'max error J1=4.0 deg'):
            self.arm.wait_arrived([0.0] * 6, pump=self.pump)
        self.assertGreater(self.pumps, 3)                              # 等待期间一直在泵

    def test_pumped_sdk_call(self):
        self.arm.call('gripper', mm=30.0, pump=self.pump)              # SDK 里阻塞 0.15 s：期间泵了 ≥ 2 次
        self.assertGreaterEqual(self.pumps, 2)
        self.sdk.gripper_fails = True
        with self.assertRaisesRegex(app.RobotFault, 'gripper 失败'):
            self.arm.call('gripper', mm=30.0, pump=self.pump)

    def test_command_q(self):
        self.assertEqual(self.arm.command_q(), [0.0] * 6)              # 假臂没有 _start_q_urdf：退回实测角
        self.sdk._wait_state = lambda: None
        self.sdk._start_q_urdf = lambda st: [5.0] * 6                  # SDK 的轨迹起点（命令角）
        self.assertEqual(self.arm.command_q(), [5.0] * 6)


class OverlayTests(unittest.TestCase):
    """画面中文化：有能画中文的字体就用 PIL 画，没有就退回英文、不报错；版面不越界。"""

    # 测试机上可能有的字体；找不到对应用例就跳过
    LATIN_FONTS = ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
                   '/System/Library/Fonts/Supplemental/Verdana.ttf')
    CJK_FONTS = ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                 '/System/Library/Fonts/STHeiti Light.ttc', '/System/Library/Fonts/Hiragino Sans GB.ttc')

    def setUp(self):
        grasp._CJK.clear()
        patcher = patch.object(grasp, '_fc_match_zh', return_value=None)   # 不去问系统的 fc-match
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(grasp._CJK.clear)

    @classmethod
    def first_existing(cls, paths):
        return next((path for path in paths if Path(path).is_file()), None)

    def use_font(self, *paths):
        """只把这些字体文件当候选（_CJK_FONT_FILES），并清掉缓存。"""
        grasp._CJK.clear()
        return patch.object(grasp, '_CJK_FONT_FILES', tuple(path for path in paths if path))

    @staticmethod
    def annotated(phase, shape=(720, 1280), long_name=False):
        """一帧假画面，返回 (图像, overlay)：一个正常物体、一个贴左上角（大半在画面外）、
        一个贴右下角（超出画面），再加状态横幅和底部当前目标。long_name 时名字 300 个字符。"""
        image = np.zeros(shape + (3,), np.uint8)
        height, width = shape
        scale = height / 720.0
        overlay = grasp.Overlay(height, width)
        box = np.array([[560, 300], [720, 300], [720, 420], [560, 420]], float) * [width / 1280, height / 720]
        unit = dict(line_vector=np.array([40.0, 0.0]), center=(int(640 * width / 1280), int(360 * height / 720)),
                    position=np.array([0.41, -0.10, 0.40]), jaw_m=0.0607)
        names = ('squeeze ball', 'x' * 300) if long_name else ('squeeze ball', 'water bottle')
        grasp.draw(image, names[0], 0.91, box, unit, True, False, overlay, scale)
        corner = box * [0.1, 0.1] - [30 * scale, 40 * scale]        # 左上角：上沿在画面外
        grasp.draw(image, names[1], 0.48, corner, None, False, True, overlay, scale)
        grasp.draw(image, 'banana', 0.33, box * [2.4, 2.2], unit, True, False, overlay, scale)
        grasp.draw_banner(overlay, phase, scale)
        grasp.draw_target(overlay, phase, (0.91, 'squeeze ball', box, unit), scale)
        overlay.render(image)
        return image, overlay

    def assert_inside(self, overlay):
        height, width = overlay.height, overlay.width
        self.assertGreaterEqual(len(overlay.placed), 8)              # 至少 4 块底色 + 每块一行
        for x0, y0, x1, y1 in overlay.placed:
            self.assertTrue(0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height, (x0, y0, x1, y1))

    def test_no_cjk_font_falls_back_to_english(self):
        drawn = []
        with self.use_font(), patch.object(grasp.cv2, 'putText', side_effect=lambda *a, **kw: drawn.append(a[1])):
            image, overlay = self.annotated('live')
        self.assertEqual((grasp.text_renderer()[1], image.shape, image.dtype), (False, (720, 1280, 3), np.uint8))
        text = '\n'.join(drawn)
        for expected in ('LIVE', 'g grasp', 'BEST squeeze ball 0.91', 'jaw=6.1cm', 'cam(+0.41,-0.10,+0.40)m',
                         'no valid grasp geometry'):
            self.assertIn(expected, text)
        self.assertNotIn('\u6293', text)                            # 退路里没有汉字
        with self.use_font(__file__), patch.object(grasp.cv2, 'putText'):   # 文件在、但不是字体
            grasp._CJK.clear()
            self.assertIsNone(grasp.cjk_font())
            self.annotated('holding')
        with self.use_font(__file__), patch.dict(sys.modules, {'PIL': None}), patch.object(grasp.cv2, 'putText'):
            grasp._CJK.clear()                                       # 没装 PIL
            self.assertIsNone(grasp.cjk_font())
            self.assertEqual(grasp.text_renderer()[1], False)
            self.annotated('missed')

    def test_font_without_chinese_glyphs_is_rejected(self):
        # fc-match 在没有中文字体的机器上也会随便回一个字体（例如 Verdana）：画出来全是方框，不能要
        latin = self.first_existing(self.LATIN_FONTS)
        if latin is None:
            self.skipTest('no Latin font on this machine')
        self.assertFalse(grasp._renders_cjk(latin, 0))
        # 候选文件也一起清掉：装了 Noto 的机器上 cjk_font() 会先从 _CJK_FONT_FILES 取到能画中文的字体
        with self.use_font(), patch.object(grasp, '_fc_match_zh', return_value=(latin, 0)), \
                patch.object(grasp.cv2, 'putText'):
            grasp._CJK.clear()
            self.assertIsNone(grasp.cjk_font())
            self.assertEqual(grasp.text_renderer()[1], False)
            self.annotated('live')

    def test_chinese_font_order_and_rendering(self):
        cjk = self.first_existing(self.CJK_FONTS)
        if cjk is None:
            self.skipTest('no CJK font on this machine')
        self.assertTrue(grasp._renders_cjk(cjk, grasp._sc_face(cjk)))
        latin = self.first_existing(self.LATIN_FONTS)
        if latin:                                                    # 候选按顺序：前一个画不出中文才看下一个
            with self.use_font(latin, cjk):
                grasp._CJK.clear()
                self.assertEqual(grasp.cjk_font(), (cjk, grasp._sc_face(cjk)))
        with self.use_font(cjk):
            grasp._CJK.clear()
            image, overlay = self.annotated('holding')
        self.assertTrue(grasp.text_renderer()[1])
        self.assert_inside(overlay)
        self.assertEqual(image.shape, (720, 1280, 3))
        banner = overlay.blocks[-2]                                  # 横幅：橙色底、横贯整幅
        self.assertEqual(banner['bg'], (0, 80, 200, 160))
        self.assertIn((0, 0, 1280, grasp.banner_height(1.0)), overlay.placed)
        # 黑底铺上 (0, 80, 200)、不透明度 160/255 → (0, 50, 125)：横幅里没被字盖住的地方都是这个橙色
        self.assertGreater(int(np.all(image[:grasp.banner_height(1.0)] == (0, 50, 125), axis=2).sum()), 1000)
        self.assertIn('夹持中', overlay.texts)
        self.assertIn('R 松开夹爪　　Q 松爪回安全位并退出', overlay.texts)

    def test_layout_stays_inside_the_frame(self):
        """英文退路和 PIL 中文两条路都查：物体框贴着边角、名字很长、画面从 160x90 到 4K。"""
        latin = self.first_existing(self.LATIN_FONTS)
        renderers = [('english', self.use_font(), None)]
        if latin:                                                    # 用拉丁字体顶替中文字体走 PIL 那条路
            renderers.append(('PIL', self.use_font(latin), patch.object(grasp, '_renders_cjk', return_value=True)))
        for label, fonts, forced in renderers:
            with fonts, forced or contextlib.nullcontext():
                for shape in ((720, 1280), (480, 640), (90, 160), (2160, 3840)):
                    with self.subTest(renderer=label, shape=shape):
                        grasp._CJK.clear()
                        image, overlay = self.annotated('missed', shape, long_name=True)
                        self.assertEqual(grasp.text_renderer()[1], label == 'PIL')
                        self.assert_inside(overlay)
                        self.assertEqual((image.shape, image.dtype), (shape + (3,), np.uint8))
                        # 名字 300 个字符：标签那一行被截断成「xxx…」，截断后不超出那一块底色
                        self.assertTrue(overlay.texts[3].startswith('xxx') and overlay.texts[3].endswith('…'))
                        self.assertLessEqual(overlay.placed[5][2], overlay.placed[4][2])
                        self.assertLess(overlay.placed[5][2] - overlay.placed[5][0], shape[1])

    def test_texts_follow_the_phase(self):
        zh = lambda lines: [line[0] for line in lines]
        en = lambda lines: [line[1] for line in lines]
        live = grasp.banner_lines('live', 1.0)
        self.assertEqual(zh(live)[0], '实时识别')
        self.assertEqual(zh(live)[1], 'G 抓取　　Q 松爪回安全位并退出')
        for word in ('绿框', '橙框', '白线', '红点', '仅示意'):
            self.assertIn(word, zh(live)[2])
        self.assertEqual(en(live)[0], 'LIVE')
        self.assertIn('g grasp', en(live)[1])
        holding = grasp.banner_lines('holding', 1.0)
        self.assertEqual((zh(holding)[0], zh(holding)[1]), ('夹持中', 'R 松开夹爪　　Q 松爪回安全位并退出'))
        self.assertEqual(en(holding)[0], 'HOLDING')
        self.assertEqual(zh(grasp.banner_lines('missed', 1.0))[0], '抓空')
        self.assertEqual(zh(grasp.banner_lines('live', 1.0, warming=True))[0], '预热中')
        self.assertEqual([line[2] for line in grasp.banner_lines('live', 0.5)], [17, 12, 8])
        target = grasp.Overlay(720, 1280)
        best = (0.91, 'squeeze ball', None, dict(position=np.array([0.41, -0.10, 0.40]), jaw_m=0.0607))
        for phase, chosen in (('live', best), ('live', None), ('holding', None), ('missed', None)):
            grasp.draw_target(target, phase, chosen, 1.0)
        self.assertEqual(zh(target.blocks[0]['lines']), ['当前目标：squeeze ball 置信度 0.91　爪宽 6.1 cm　按 G 抓取'])
        self.assertEqual([zh(block['lines'])[0] for block in target.blocks[1:]],
                         ['当前目标：暂无有效目标', '夹着物体：按 R 松开，再按 G 抓下一个', '抓空：按 G 再抓'])
        self.assertIn('[g] grasp', en(target.blocks[0]['lines'])[0])
        image = np.zeros((720, 1280, 3), np.uint8)
        box = np.array([[560, 300], [720, 300], [720, 420], [560, 420]], float)
        unit = dict(line_vector=np.array([40.0, 0.0]), center=(640, 360), position=np.array([0.41, -0.10, 0.40]),
                    jaw_m=0.0607)
        labels = grasp.Overlay(720, 1280)
        grasp.draw(image, 'squeeze ball', 0.91, box, unit, True, False, labels, 1.0)
        self.assertEqual(zh(labels.blocks[0]['lines']),
                         ['squeeze ball 置信度 0.91', '爪宽 6.1 cm', '相机系 (+0.41, -0.10, +0.40) m'])
        grasp.draw(image, 'banana', 0.33, box, None, False, True, labels, 1.0)
        self.assertEqual(zh(labels.blocks[1]['lines']), ['banana 置信度 0.33', '无有效抓取几何'])
        # 框贴着画面上沿：标签上方放不下，挪到横幅下面
        top = grasp.Overlay(720, 1280)
        grasp.draw(image, 'banana', 0.33, box * [1, 0.2] - [0, 40], unit, True, False, top, 1.0)
        self.assertLess(labels.blocks[0]['y'], 300)                  # 对照：正常框的标签在框上方
        self.assertGreaterEqual(top.blocks[0]['y'], grasp.banner_height(1.0))


