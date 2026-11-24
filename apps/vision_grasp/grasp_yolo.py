#!/usr/bin/env python3
"""YOLO min-area-rect/short-axis grasping on top of the hand-eye result.

Flow (mirrors reBot-DevArm-Grasp): move to the ready pose, open the gripper,
then show live YOLO detections in an OpenCV window. Key g selects the best
instance, preflights symmetric grasp poses against D1 IK and J5 limits, then
moves to pregrasp. Approach and retreat use the configured final_motion.
With final_motion: linear the whole grasp runs as two streamed joint
trajectories (ready -> pregrasp without stopping -> straight line to the grasp
point, then back up to ready); see plan_stream. With joint it runs the old
segments, stopping at each waypoint.
Back at the ready pose the gripper holds the object: key r releases it there
and key g grasps the next one, so a session can run many grasps in a row. A
missed grasp (nothing between the jaws) is reported and does not end the
session; any other robot fault still does. Key q opens the gripper, returns
to the safe pose and exits (pressed during a grasp, it is deferred until the
arm is back at the ready pose). The live window is annotated in Chinese when
a CJK font is available (drawn with Pillow) and falls back to English
otherwise; the terminal log stays English either way.

SDK TCP convention used here (kinematics.hpp default tool): +x = tool forward
(approach into the object), and the gripper opens along `open_axis` (y by
default; flip to z if the first dry run shows a 90-degree jaw rotation).
Install ultralytics first; see README section 10.
"""
import argparse
from datetime import datetime
import math
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handeye import RobotFault, RpcArm, load_config, log, pose_matrix  # noqa: E402
from d1t_sdk._joint_model import gripper_mm_to_deg  # noqa: E402  (utils.robot 已把 sdk/ 加进 sys.path)
from utils.device import resolve_device  # noqa: E402
from utils.paths import data_root, ensure_text_encoder, resolve_model  # noqa: E402

WINDOW = 'YOLO grasp  g:grasp  r:release  q:open+safe+quit'
DEFAULTS = dict(conf=0.5, iou=0.45, device='auto', approach='camera_ray', classes=[],
                depth_quantile=0.5, top_depth_quantile=0.15, support_z_m=0.0,
                cloud_erode_px=4, table_ring_inner_px=8, table_ring_outer_px=25,
                min_height_above_table_m=0.005, table_self_correct=True, table_self_correct_max_deg=10.0,
                open_axis='y', final_motion='linear',
                linear_vel_mps=0.04, linear_acc_mps2=0.25, j5_margin_deg=5.0,
                pregrasp_offset_m=0.05, grasp_offset_m=0.015,
                grasp_vel_dps=10.0, wait_before_grip_s=0.0,
                gripper_open_mm=65.0, gripper_close_factor=1.0, gripper_close_margin_mm=8.0,
                gripper_close_min_mm=15.0, gripper_preclose_margin_mm=0.0,
                min_base_z_m=0.02, base_offset_m=(0.0, 0.0, 0.0))

# 闭爪后锁停住角的判据（顶物体保持会堵转发热：实测目标比停住角多闭约 3°
# 电流即超 300 mA、82 s 内 J6 从 40 ℃ 升到 45 ℃，20 mm 的差更甚）
GRIP_SETTLE_TIMEOUT_S = 2.0  # 闭爪命令走完后等稳定的超时
GRIP_STABLE_FRAMES = 3       # 连续稳定帧数（每帧变化 < GRIP_STABLE_MM）
GRIP_STABLE_MM = 0.2         # 每帧开度变化阈值，≈0.3°（夹爪标定表斜率 0.55–0.92 mm/°）
GRIP_POLL_S = 0.05           # 轮询间隔（一帧 20 Hz 遥测）

# cloud_top_down：掩码点云 + 桌面平面求物体中心（竖直下抓）
CLOUD_MIN_POINTS = 50                  # 物体点数下限，更少视为深度不可用
CLOUD_HEIGHT_RANGE_M = (0.01, 0.12)    # 物高合理范围（米）
CLOUD_RING_MIN_POINTS = 50             # 桌面环带点数下限，留下的点不足则退回缓存平面或 support_z_m
CLOUD_RING_MAX_SPREAD_M = 0.01         # 留下的环带点到拟合平面残差的 10–90 分位差上限（米）
CLOUD_RING_OUTLIER_M = 0.008           # 离群阈值：到平面（起手时到水平层）的残差绝对值超过它的环带点剔掉
CLOUD_RING_REFITS = 2                  # 剔除后最多重拟合次数
CLOUD_CUT_FRACTION = 0.05              # 掩码点里落在桌面以下的占比达到它 = 平面切进了物体（见 cut_fraction）
CLOUD_CUT_DEPTH_M = 0.10               # 落在桌面以下多深还算「被平面切到」；更深的（透视到的地面）不算
CLOUD_CACHE_MAX_JOINT_DEG = 1.0        # 复用缓存平面要求拍照位姿各关节角之差不超过它（度）
CLOUD_MIN_AXIS_M = 0.010               # 长轴两端视线交点的最小间距，更近视为没有长轴
CLOUD_JAW_BAND_M = 0.010               # 量爪宽用的中段带：长轴中段两侧各 1 cm
CLOUD_JAW_MIN_PIXELS = 8               # 中段带内最少物体点数，更少退回点云跨度
FINGER_LENGTH_M = 0.050                # 指尖从 TCP（两指根部）伸出的长度
FINGER_CLEARANCE_M = 0.005             # 指尖离桌面的最小余量

_TABLE_CACHE = {}                      # 本次运行最近一次拟合成功的桌面：q_deg（拍照位姿）→ table (a, b, c)

# 夹爪预合拢（cloud_top_down）：到预抓取点后先把开口收到「爪宽 + 余量」，缩短下降后的闭爪行程
PRECLOSE_MIN_CLEARANCE_MM = 8.0       # 预合拢后两指至少比物体宽这么多：下降时指尖不压物体

# 抓取执行段的流式连续轨迹：去程一条（当前位 → 预抓取点不停 → 直线到抓取点停），回程一条（倒着走）
STREAM_DT_S = 0.05                # 轨迹每帧一行，帧周期 = 控制器命令插补周期（SDK DT_MS）
STREAM_PATH_STEP_DEG = 0.05       # 时间规划用的路径步长（关节空间弧长，度）
STREAM_BLEND_M = 0.012            # 预抓取点处圆角的半宽（沿直线段量，米）
STREAM_BLEND_MIN_M = 0.002        # 圆角最小半宽
STREAM_BLEND_MAX_DEV_M = 0.0008   # 圆角在预抓取点高度处偏离下降直线的上限（米）：指尖贴着物体下去，不能横扫
STREAM_ACCEL_MARGIN = 0.9         # 规划用的加速度上限 = 配置上限 × 它（按帧采样后二阶差分略有出入，留余量）
STREAM_J6_SPEED_DPS = 19.0        # 流式里 J6 自己限速：控制器给 J6 的流式上限是 100°/s，不能依赖（SDK gripper 是 20°/s）
STREAM_J6_ACC_DPS2 = 200.0        # J6 预合拢的加减速（°/s²；行程短走不到巡航速度就是三角形）

# 检测没有分割掩码、退回轴对齐框时的提示：同一类别每 NO_MASK_LOG_INTERVAL_S 最多打一次
NO_MASK_LOG_INTERVAL_S = 2.0
_NO_MASK_LOGGED = {}                   # 类别名 → 上次打印的时刻（monotonic）


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
                jaw_m=jaw_m, ends_px=ends_px if contour is not None else None, line_vector=direction * span_px, center=(int(round(center[0])), int(round(center[1]))),
                center_px=(float(center[0]), float(center[1])), mask=mask)


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


def ray_hits_level(uv, k, t_base_camera, z):
    """像素 uv 的视线与基座系水平面 z 的交点（基座系 xyz）。
    视线不朝下，或交点在相机后方时返回 None。"""
    rotation, origin = t_base_camera[:3, :3], t_base_camera[:3, 3]
    ray = rotation @ np.array([(uv[0] - k[0, 2]) / k[0, 0], (uv[1] - k[1, 2]) / k[1, 1], 1.0])
    if abs(ray[2]) < 1e-9:
        return None
    scale = (z - origin[2]) / ray[2]
    return origin + scale * ray if scale > 0 else None


def axis_midpoint_xy(ends_px, k, t_base_camera, z):
    """物体长轴两端像素各沿视线与基座系水平面 z 求交，返回两交点中点的 (x, y)。
    纠正透视：顺视线躺放的物体远端在图里被压短，二维中点会偏向近端。
    视线不朝下（与平面无前向交点）时返回 None。"""
    hits = [ray_hits_level(uv, k, t_base_camera, z) for uv in ends_px]
    if any(hit is None for hit in hits):
        return None
    return (hits[0][:2] + hits[1][:2]) / 2


def disc_kernel(radius_px):
    """半径 radius_px 的圆形结构元；半径 0 返回 None（不做形态学）。"""
    radius = int(radius_px)
    if radius <= 0:
        return None
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))


def backproject(xs, ys, z, k, t_base_camera):
    """像素坐标 + 相机系深度 z（米）→ 基座系点 (n, 3)。针孔模型，同 grasp_camera 不计畸变。"""
    points = np.column_stack(((xs - k[0, 2]) * z / k[0, 0], (ys - k[1, 2]) * z / k[1, 1], z))
    return points @ t_base_camera[:3, :3].T + t_base_camera[:3, 3]


def canonical_sign(vector):
    """符号固定：绝对值最大的分量为正，便于复现与比对。"""
    return -vector if vector[np.argmax(np.abs(vector))] < 0 else vector


def horizontal_direction(image_dir, k, t_base_camera):
    """图像面内的方向 → 基座系水平面内的单位向量；水平投影过短返回 None。"""
    vec = t_base_camera[:3, :3] @ np.array([image_dir[0] / k[0, 0], image_dir[1] / k[1, 1], 0.0])
    vec[2] = 0.0
    norm = float(np.linalg.norm(vec))
    return vec / norm if norm > 1e-6 else None


def cloud_jaw(obj, center, along):
    """截面宽（米）：物体点里沿 along 方向距 center 不超过 CLOUD_JAW_BAND_M 的中段带，
    到「过 center、方向 along」的长轴线的距离（去掉沿轴分量）的中位数 × 2。

    可见面只有半圈，直接取跨度会偏小；但截面上每一点到轴线的距离都是半径，
    所以不需要看不到的另一半。注意不能把掩码像素沿视线投到中线高度平面去量：
    那是物体在这个平面上的影子，沿视线方向被拉长 1/sin(俯角) 倍。
    带内点太少返回 None。"""
    offsets = obj - center
    in_band = np.abs(offsets @ along) <= CLOUD_JAW_BAND_M
    if in_band.sum() < CLOUD_JAW_MIN_PIXELS:
        return None
    radial = offsets[in_band] - (offsets[in_band] @ along)[:, None] * along
    return float(2.0 * np.median(np.linalg.norm(radial, axis=1)))


def plane_tilt_deg(table):
    """平面 z = a·x + b·y + c 相对水平面的倾角（度）。"""
    return float(np.degrees(np.arctan(np.hypot(table[0], table[1]))))


def fit_table_plane(ring_pts, support_z_m):
    """环带点的桌面平面 z = a·x + b·y + c，带离群剔除（混进环带的桌边、桌外、台沿）。

    起手不用最小二乘，用点最密的一层水平面：按 z 找宽度 2·CLOUD_RING_OUTLIER_M 的窗口里点最多的
    一段，取其中位数当初始高度；窗口并列时取离 support_z_m 最近的一层。环带一半在桌外时最小二乘
    落在桌面与桌外两层中间，两层的残差都超限，会被一起剔掉；离群点占到三成时它还会收敛到一个
    倾斜的错平面，分位差判据也拦不住。之后每轮剔掉残差绝对值 > CLOUD_RING_OUTLIER_M 的点再
    最小二乘，最多 CLOUD_RING_REFITS 轮。
    返回 (table, kept, gap)：kept = 留下的点占环带点的比例，gap = 留下点残差的 10–90 分位差（米）；
    剩下的点不足 3 个或拟合不出时返回 None。"""
    z = np.sort(ring_pts[:, 2])
    counts = np.searchsorted(z, z + 2 * CLOUD_RING_OUTLIER_M, side='right') - np.arange(len(z))
    densest = np.nonzero(counts == counts.max())[0]
    start = densest[np.argmin(np.abs(z[densest] - support_z_m))]
    level = np.median(z[start:start + counts[start]])
    design = np.column_stack((ring_pts[:, :2], np.ones(len(ring_pts))))
    keep = np.abs(ring_pts[:, 2] - level) <= CLOUD_RING_OUTLIER_M
    for _ in range(CLOUD_RING_REFITS + 1):
        if keep.sum() < 3:
            return None
        try:
            table, *_ = np.linalg.lstsq(design[keep], ring_pts[keep, 2], rcond=None)
        except np.linalg.LinAlgError:
            return None
        residual = ring_pts[:, 2] - design @ table
        trimmed = np.abs(residual) <= CLOUD_RING_OUTLIER_M
        if (trimmed == keep).all():
            break
        keep = trimmed
    return table, float(keep.mean()), float(np.ptp(np.percentile(residual[keep], [10, 90])))


def cut_fraction(mask_pts, table, min_height_m):
    """掩码（不腐蚀）的点里落在桌面以下 min_height_m ~ CLOUD_CUT_DEPTH_M 的占比。
    平面选到挨着物体的台沿那一层时，物体下半截就落在它下面：占比高 = 平面切进了物体。
    用不腐蚀的掩码点：物体贴着桌面的最低一圈在腐蚀后的核心区里已经看不到了。更深的不算
    （例如透过透明物体看到桌外的地面）：那是掩码的事，不是平面的事。"""
    relative = mask_pts[:, 2] - (mask_pts[:, :2] @ table[:2] + table[2])
    return float(np.mean((relative <= -min_height_m) & (relative >= -CLOUD_CUT_DEPTH_M)))


def table_plane(ring_pts, mask_pts, g, q_deg=None, fallback=None):
    """定桌面，返回 dict(table, source, why, kept)；table = (a, b, c) 即 z = a·x + b·y + c。

    候选按顺序取第一个「物体站在它上面」的：
    - 环带拟合（'plane'）：留下的点 ≥ CLOUD_RING_MIN_POINTS 且残差 10–90 分位差 ≤
      CLOUD_RING_MAX_SPREAD_M，见 fit_table_plane；
    - fallback（'plane' / 'cached'）：只在桌面自校正之后重算的那一遍传，是上一遍定下的平面，
      转平后就是 z = support_z_m；
    - 本次运行的缓存（'cached'）：上一次拟合成功的平面，拍照位姿各关节角之差 ≤
      CLOUD_CACHE_MAX_JOINT_DEG 才复用（位姿变了，相机链的误差也变了）。
    「物体站在它上面」= 掩码点里落在平面以下的占比 < CLOUD_CUT_FRACTION（见 cut_fraction）。
    都不行才退回水平面 z = support_z_m（'config'，不检：相机链有偏差时物体本来就在它以下）。
    why = 前面的候选被否掉的原因，日志用；kept = 拟合的环带保留比例（没用上拟合时为 None）。
    q_deg 给出时才读写缓存；直接调用的场合不带它，结果只取决于入参。"""
    support = float(g['support_z_m'])
    candidates, why = [], []
    flat = np.array([0.0, 0.0, support])
    if len(ring_pts) < CLOUD_RING_MIN_POINTS:
        why.append(f'{len(ring_pts)} points')
    else:
        fit = fit_table_plane(ring_pts, support)
        if fit is None:
            why.append('degenerate ring points')
        else:
            table, kept, gap = fit
            kept_n = int(round(kept * len(ring_pts)))
            if kept_n < CLOUD_RING_MIN_POINTS:
                why.append(f'{kept_n} inlier points after outlier removal')
            elif gap > CLOUD_RING_MAX_SPREAD_M:
                why.append(f'10-90% residual spread {gap*1000:.0f} mm')
            else:
                candidates.append((table, 'plane', kept))
    if fallback is not None:
        candidates.append((flat, fallback, None))
    elif q_deg is not None and _TABLE_CACHE:
        if np.max(np.abs(np.asarray(q_deg, float) - _TABLE_CACHE['q_deg'])) <= CLOUD_CACHE_MAX_JOINT_DEG:
            candidates.append((_TABLE_CACHE['table'].copy(), 'cached', None))
    for table, source, kept in candidates:
        cut = cut_fraction(mask_pts, table, g['min_height_above_table_m'])
        if cut < CLOUD_CUT_FRACTION:
            if source == 'plane' and fallback is None and q_deg is not None:
                _TABLE_CACHE.update(q_deg=np.asarray(q_deg, float).copy(), table=table.copy())
            return dict(table=table, source=source, kept=kept, why='; '.join(why) or None)
        why.append(f"{'fitted' if source == 'plane' else source} plane rejected: "
                   f'{cut:.0%} of the instance lies below it')
    return dict(table=flat, source='config', kept=None, why='; '.join(why) or None)


def table_correction(table, t_base_camera, support_z_m):
    """桌面自校正：把测得的桌面变成「高度 support_z_m 的水平面」的刚体修正（基座系 4x4）。

    底座与桌面同面，真实桌面就是水平面 z = support_z_m；测出来的桌面歪了、矮了，说明相机位姿
    （手眼旋转 + FK）有误差。绕相机光心把平面法向转到 +z（最小旋转），再沿 z 平移 dz 让平面落到
    support_z_m：俯仰、横滚误差正好是绕光心的旋转，在 0.4 m 视距上折合二三十毫米，这样转回去
    就抵消了。t_base_camera_corr = 返回的 T_corr @ t_base_camera。
    只能校正俯仰、横滚和竖直平移：绕竖直轴的转动不改变桌面的倾角，水平平移不改变桌面的高度，
    单凭桌面平面看不出这两项（水平偏差用 grasp.base_offset_m 补）。
    返回 (T_corr, 修正角 deg, dz 米)；修正角就是平面的倾角，由调用方拿它跟上限比。"""
    normal = np.array([-table[0], -table[1], 1.0])
    normal /= np.linalg.norm(normal)
    axis = np.cross(normal, [0.0, 0.0, 1.0])
    tilt = float(np.arctan2(np.linalg.norm(axis), normal[2]))
    out = np.eye(4)
    if tilt > 1e-12:
        out[:3, :3] = cv2.Rodrigues(axis / np.linalg.norm(axis) * tilt)[0]
    center = np.asarray(t_base_camera, float)[:3, 3]
    out[:3, 3] = center - out[:3, :3] @ center        # 绕光心转
    dz = support_z_m - (out[2, 2] * table[2] + out[2, 3])   # 点 (0, 0, c) 转后的高度 → 平移到 support
    out[2, 3] += dz
    return out, float(np.degrees(tilt)), float(dz)


def object_from_cloud(mask, depth_mm, k, t_base_camera, g, center=None, ends_px=None, rect=None,
                      q_deg=None):
    """掩码点云 + 桌面平面求物体中心（基座系），供竖直下抓；不可用时打印原因、返回 None。

    桌面：掩码外环带（table_ring_inner_px ~ table_ring_outer_px）的点带离群剔除拟合 z = a·x + b·y + c；
    拟合不出来就退回本次运行的缓存平面，再不行退回水平面 z = support_z_m（table_source =
    'plane' / 'cached' / 'config'，见 table_plane）。物体点与物高 h 都按到这个平面的相对高度算。
    桌面自校正（grasp.table_self_correct，默认开）：有可用的平面时绕相机光心把它转平、沿 z 挪到
    support_z_m（见 table_correction），再用修正后的相机位姿把点云、平面、物高、中心、长轴、爪宽
    全部重算一遍；修正角超过 table_self_correct_max_deg 或没有可用平面时不修正，打一行原因。
    输出的绝对高度以配置的真实桌面为基准，中心 z = support_z_m + h/2。
    plane_offset_m / plane_tilt_deg = 最终平面在物体处的下沉量与倾角，自校正后应接近 0；
    raw_plane_offset_m / raw_plane_tilt_deg = 自校正前的读数，即坐标链的偏差。
    中心 xy：过 center 像素（grasp_camera 的 center_px）的视线与「平面在物体处的高度 + h/2」
    水平面的交点 c；再沿长轴方向 u 挪到 m，m 是长轴两端像素 ends_px 各沿视线与同一水平面
    求交后的中点——透视把远端压短，二维中段对应的三维位置偏向近端。横向分量仍取 c，
    弯曲物体也落在身上。ends_px 缺失、任一端视线无交点或两交点相距不足 CLOUD_MIN_AXIS_M
    时只用 c 并打印一行。
    开合方向 = 水平面内垂直于 u 的方向（无 u 时退回物体点 xy 的 PCA 次轴）。爪宽 = 长轴中段
    ±CLOUD_JAW_BAND_M 带内的物体点到长轴线的距离中位数 × 2（见 cloud_jaw；无 u 时带的方向
    退回图像长轴）；带内点太少时退回点云跨度，jaw_source='fallback'，否则 'radius'。
    q_deg = 拍照时的关节角，只用来判断缓存平面能不能复用。
    返回 dict(position, open_dir, jaw_m, jaw_source, height_m, table_z, table_source, ring_kept,
    plane_offset_m, plane_tilt_deg, raw_plane_offset_m, raw_plane_tilt_deg, correction,
    along_shift_m, aabb, n_points)；correction = None 或 dict(tilt_deg, dz_m)。"""
    def reject(reason):
        log(f'cloud_top_down rejected: {reason}; frame skipped')
        return None

    if center is None:
        return reject('image center not given')
    depth_mm = np.asarray(depth_mm)
    valid = np.isfinite(depth_mm) & (depth_mm > 0)
    whole = (np.asarray(mask) > 0).astype(np.uint8)
    kernel = disc_kernel(g['cloud_erode_px'])
    core = cv2.erode(whole, kernel) if kernel is not None else whole
    ys, xs = np.nonzero(core.astype(bool) & valid)
    if not len(xs):
        return reject('no valid depth inside the instance')
    outer, inner = disc_kernel(g['table_ring_outer_px']), disc_kernel(g['table_ring_inner_px'])
    ring = (cv2.dilate(whole, outer) if outer is not None else whole).astype(bool)
    ring &= ~(cv2.dilate(whole, inner) if inner is not None else whole).astype(bool)
    ry, rx = np.nonzero(ring & valid)
    ay, ax = np.nonzero(whole.astype(bool) & valid)        # 不腐蚀的掩码点：只用来查平面有没有切进物体
    depth_m = depth_mm / 1000.0

    def clouds(view):
        """该相机位姿下的物体点云（腐蚀后）、环带点云、掩码点云（基座系）。"""
        return (backproject(xs, ys, depth_m[ys, xs], k, view),
                backproject(rx, ry, depth_m[ry, rx], k, view) if len(rx) else np.empty((0, 3)),
                backproject(ax, ay, depth_m[ay, ax], k, view))

    support = float(g['support_z_m'])
    view = np.asarray(t_base_camera, float)
    points, ring_pts, mask_pts = clouds(view)
    fit = table_plane(ring_pts, mask_pts, g, q_deg=q_deg)
    before, correction, skipped = fit, None, None
    if g['table_self_correct']:
        tilt = plane_tilt_deg(fit['table'])
        if fit['source'] == 'config':
            skipped = 'no usable table plane'
        elif tilt > g['table_self_correct_max_deg']:
            skipped = (f'tilt {tilt:.1f} deg above table_self_correct_max_deg '
                       f"{g['table_self_correct_max_deg']:g}")
        else:
            correction = table_correction(fit['table'], view, support)
            view = correction[0] @ view
            points, ring_pts, mask_pts = clouds(view)
            fit = table_plane(ring_pts, mask_pts, g, fallback=before['source'])
    if fit['why']:
        log(f"cloud_top_down: table ring unusable ({fit['why']}); using "
            + ('the cached plane from this run' if fit['source'] == 'cached'
               else f'support_z_m = {support:.3f} m'))
    table, table_source = fit['table'], fit['source']

    relative = points[:, 2] - (points[:, :2] @ table[:2] + table[2])   # 到桌面的相对高度
    above = relative > g['min_height_above_table_m']
    obj = points[above]
    if len(obj) < CLOUD_MIN_POINTS:
        return reject(f'only {len(obj)} object points above the table (need {CLOUD_MIN_POINTS})')
    low, high = CLOUD_HEIGHT_RANGE_M
    height = float(np.percentile(relative[above], 95))
    if not low <= height <= high:
        return reject(f'height {height*1000:.0f} mm outside [{low*1000:.0f}, {high*1000:.0f}] mm')

    level = float(obj[:, :2].mean(axis=0) @ table[:2] + table[2]) + height / 2.0   # 中线高度
    hit = ray_hits_level(center, k, view, level)
    if hit is None:
        return reject('the ray through the image center does not reach the mid-height plane '
                      'ahead of the camera')
    position = np.array([hit[0], hit[1], support + height / 2.0])

    ends = [ray_hits_level(uv, k, view, level) for uv in ends_px] if ends_px is not None else []
    axis, shift, why = None, 0.0, None
    if len(ends) != 2:
        why = 'ends_px missing'
    elif any(end is None for end in ends):
        why = 'an end ray does not reach the mid-height plane'
    else:
        span = ends[1][:2] - ends[0][:2]
        length = float(np.linalg.norm(span))
        if length < CLOUD_MIN_AXIS_M:
            why = f'end hits only {length*1000:.0f} mm apart'
        else:
            axis = np.array([span[0], span[1], 0.0]) / length
            shift = float(((ends[0][:2] + ends[1][:2]) / 2 - position[:2]) @ axis[:2])
            position[:2] = position[:2] + shift * axis[:2]
    if axis is None:
        log(f'cloud_top_down: long axis unavailable ({why}); keeping the image-center point')

    if axis is None:
        _, axes = np.linalg.eigh(np.cov(obj[:, :2].T))   # eigh 特征值升序：第 0 列 = 次轴
        open_dir = canonical_sign(np.array([axes[0, 0], axes[1, 0], 0.0]))
    else:
        open_dir = canonical_sign(np.array([-axis[1], axis[0], 0.0]))   # 水平面内垂直于长轴
    along = axis
    if along is None and rect is not None:            # 无 u：长轴方向退回图像长轴（垂直于 rect 短边）
        short, span_px = short_edge(np.asarray(rect, float))
        across = horizontal_direction(short / span_px, k, view) if span_px > 0 else None
        if across is not None:
            along = np.array([-across[1], across[0], 0.0])
    jaw = (cloud_jaw(obj, np.array([position[0], position[1], level]), along)
           if along is not None else None)
    jaw_source = 'radius'
    if jaw is None:
        spread = obj[:, :2] @ open_dir[:2]            # 点云只看得到朝相机的一面，会偏小
        jaw, jaw_source = float(np.percentile(spread, 95) - np.percentile(spread, 5)), 'fallback'
    table_z = float(position[:2] @ table[:2] + table[2])   # 桌面在物体处的高度
    if not (np.isfinite(position).all() and np.isfinite(open_dir).all() and np.isfinite(jaw)):
        return reject('degenerate point cloud')
    raw_offset = table_z - support
    if correction is not None:
        raw_tilt = plane_tilt_deg(before['table'])
        raw_offset = float(position[:2] @ before['table'][:2] + before['table'][2]) - support
        log(f'cloud_top_down: correction tilt {correction[1]:.1f} deg, dz {correction[2]*1000:+.1f} mm; '
            f'plane offset {raw_offset*1000:+.1f} mm tilt {raw_tilt:.1f} deg before -> '
            f'{(table_z - support)*1000:+.1f} mm {plane_tilt_deg(table):.1f} deg after')
    elif skipped:
        log(f'cloud_top_down: table self-correction skipped ({skipped})')
    return dict(position=position, open_dir=open_dir, jaw_m=jaw, jaw_source=jaw_source,
                height_m=height, table_z=table_z, table_source=table_source, ring_kept=fit['kept'],
                plane_offset_m=table_z - support,
                plane_tilt_deg=plane_tilt_deg(table),
                raw_plane_offset_m=raw_offset, raw_plane_tilt_deg=plane_tilt_deg(before['table']),
                correction=None if correction is None else dict(tilt_deg=correction[1], dz_m=correction[2]),
                along_shift_m=shift, aabb=ends_px is None, n_points=int(len(obj)))


def finger_floor_z(table_z, g):
    """cloud_top_down 目标点 z 下限：指尖离桌面至少 FINGER_CLEARANCE_M。
    TCP 在两指根部，指尖再伸出 FINGER_LENGTH_M；TCP 停在抓取点外 grasp_offset_m，
    所以指尖越过抓取点 FINGER_LENGTH_M − grasp_offset_m。
    table_z 传配置的真实桌面 support_z_m：测出的桌面可能整体下沉，拿它当基准会撞桌。"""
    return float(table_z) + FINGER_CLEARANCE_M + FINGER_LENGTH_M - g['grasp_offset_m']


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


def close_margin_mm(jaw_m, margin_mm, minimum_mm, maximum_mm):
    """cloud_top_down 的闭口上限：实测爪宽减去余量，钳到 [minimum, fully open]。
    爪宽取自整个轮廓截面（不是只看得到半边的点云跨度），所以不再乘系数：
    目标比实物小很多时舵机到接触的位置误差大，会夹得过紧。"""
    return float(np.clip(float(jaw_m) * 1000.0 - float(margin_mm), minimum_mm, maximum_mm))


class GraspMiss(RobotFault):
    """闭爪闭合到了头、两指之间没有物体：不是故障，主循环退回准备位后可以再抓。"""


def preclose_target_mm(jaw_m, margin_mm, open_mm):
    """cloud_top_down 的预合拢开口（mm）：实测爪宽加余量 gripper_preclose_margin_mm，
    但至少比爪宽宽 PRECLOSE_MIN_CLEARANCE_MM（下降时两指不压着物体），也不超过张口。
    不小于张口时调用方不必合（已经是张口）。"""
    jaw_mm = float(jaw_m) * 1000.0
    return float(min(max(jaw_mm + float(margin_mm), jaw_mm + PRECLOSE_MIN_CLEARANCE_MM), open_mm))


def preclose(arm, mm, dry_run=False, pump=None):
    """预合拢（final_motion=joint 的分段走法用）：到预抓取点后、直线下降前先把夹爪合到 mm。
    流式轨迹里预合拢并在去程中同步做，见 plan_stream。dry_run 只打印。"""
    log(f'Pre-closing gripper to {mm:.1f}mm')
    if not dry_run:
        arm.call('gripper', mm=mm, pump=pump)


def release(arm, open_mm, dry_run=False, pump=None):
    """连续测试的 r 键：夹着物体回到准备位后开爪（dry_run 只打印）。"""
    log(f'Releasing at the ready pose: opening the gripper to {open_mm:.1f}mm')
    if not dry_run:
        arm.call('gripper', mm=open_mm, pump=pump)


def close_and_lock(arm, close_mm, g, pump=None):
    """Close onto the target, then re-target the stopped angle (no stall hold).

    After the closing move, poll gripper_mm until GRIP_STABLE_FRAMES
    consecutive frames each change less than GRIP_STABLE_MM (jaws blocked by
    the object, or fully closed), then re-send that measured opening so the
    servo holds at the stopped angle instead of pushing against the object.
    No stabilization within GRIP_SETTLE_TIMEOUT_S, or an opening at/below the
    minimum (nothing between the jaws), opens the gripper and raises: a
    stalled grip is never held. The empty case raises GraspMiss (a RobotFault:
    the caller may continue with the next grasp); a grip that does not settle
    raises RobotFault. Returns the locked opening in mm. pump：每个阻塞等待每拍调一次
    （保持 GUI 窗口响应），见 main 里的 pump。"""
    log(f'Closing gripper to {close_mm:.1f}mm (cap only; locks at the stopped angle)')
    arm.call('gripper', mm=close_mm, pump=pump)
    readings, stable = [], 0
    deadline = time.monotonic() + GRIP_SETTLE_TIMEOUT_S
    while time.monotonic() < deadline:
        if pump is not None:
            pump()  # 等稳定期间保持窗口响应
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
        raise GraspMiss(f'Gripper closed to {mm_now:.1f}mm (<= minimum '
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


def _log_no_mask(name, conf, reason):
    """没有分割掩码、退回轴对齐框时提示；同一类别每 NO_MASK_LOG_INTERVAL_S 最多打一次，免得刷屏。"""
    now = time.monotonic()
    if now - _NO_MASK_LOGGED.get(name, float('-inf')) >= NO_MASK_LOG_INTERVAL_S:
        _NO_MASK_LOGGED[name] = now
        log(f'no mask for {name} (conf {conf:.2f}), using axis-aligned box: {reason}')


def detections(results, shape, allowed=()):
    """Return label, confidence, image box, rotated rectangle and original-pixel contour."""
    out = []
    for result in results:
        boxes = getattr(result, 'boxes', None)
        if boxes is None or len(boxes) == 0:
            continue
        masks = getattr(result, 'masks', None)
        polygons = getattr(masks, 'xy', None)
        for i in range(len(boxes)):
            name = result.names.get(int(boxes.cls[i]), str(int(boxes.cls[i])))
            if allowed and name not in allowed:
                continue
            xyxy = boxes.xyxy[i].cpu().numpy().astype(float)
            conf = float(boxes.conf[i])
            contour, why = None, None
            if masks is None:
                why = 'masks missing'
            elif polygons is None or len(polygons) != len(boxes):
                why = f'{0 if polygons is None else len(polygons)} masks for {len(boxes)} boxes'
            else:
                polygon = np.asarray(polygons[i], np.float32)
                if len(polygon) < 3:
                    why = 'polygon has fewer than 3 points'
                elif not np.isfinite(polygon).all():
                    why = 'polygon has non-finite points'
                elif cv2.contourArea(polygon) < 8:
                    why = 'polygon area below 8 px'
                else:
                    contour = polygon
            if contour is None:
                rect = np.array([[xyxy[0], xyxy[1]], [xyxy[2], xyxy[1]],
                                 [xyxy[2], xyxy[3]], [xyxy[0], xyxy[3]]], float)
                _log_no_mask(name, conf, why)
            else:
                rect = cv2.boxPoints(cv2.minAreaRect(contour))
            out.append((name, conf, xyxy.astype(int), rect, contour))
    return out


# 画面中文字：cv2.putText 画不了中文，用 PIL + 系统中文字体；找不到能画中文的字体就退回英文
# （终端日志始终是英文，便于复制排查）
_CJK_FONT_FILES = ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                   '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc')
_CJK = {}        # 缓存：font → (字体文件, .ttc 字面序号) 或 None；text → 画字对象；chinese → 画的是不是中文


def _sc_face(path):
    """Noto CJK 的 .ttc 里简体中文字面（名字以 SC 结尾）的序号；没有这样的字面返回 0。"""
    try:
        from PIL import ImageFont
    except ImportError:
        return 0
    for index in range(16):
        try:
            family = ImageFont.truetype(path, 16, index=index).getname()[0]
        except Exception:  # noqa: BLE001 — 序号超出这个 .ttc 的字面数
            break
        if family.endswith(' SC'):
            return index
    return 0


def _renders_cjk(path, index):
    """这个字体画得出中文吗：拿「抓」字跟一个没赋值的码位比，缺字时两者是同一个占位方框。"""
    try:
        from PIL import ImageFont
        font = ImageFont.truetype(path, 24, index=index)
        glyph, blank = np.asarray(font.getmask('抓')), np.asarray(font.getmask('\u0378'))
        return glyph.shape != blank.shape or not np.array_equal(glyph, blank)
    except Exception:  # noqa: BLE001 — 没装 PIL、字体文件读不了：都算画不出
        return False


def _fc_match_zh():
    """问 fontconfig 要中文（:lang=zh）用的字体，返回 (文件, 字面序号)；问不出来返回 None。"""
    try:
        reply = subprocess.run(['fc-match', '-f', '%{file}\n%{index}', ':lang=zh'], capture_output=True,
                               text=True, errors='replace', timeout=5).stdout.split()
    except (OSError, subprocess.SubprocessError, ValueError):
        return None
    if not reply or not Path(reply[0]).is_file():
        return None
    return reply[0], int(reply[1]) if len(reply) > 1 and reply[1].isdigit() else 0


def cjk_font():
    """能画中文的字体 (文件, 字面序号)，找不到返回 None；整个进程只找一次。
    顺序：_CJK_FONT_FILES → fc-match。fc-match 在没有中文字体的机器上也会随便回一个字体，
    画出来全是方框，所以每个候选都先用 _renders_cjk 验一下。"""
    if 'font' not in _CJK:
        candidates = [(path, _sc_face(path)) for path in _CJK_FONT_FILES if Path(path).is_file()]
        found = _fc_match_zh()
        if found:
            candidates.append(found)
        _CJK['font'] = next((font for font in candidates if _renders_cjk(*font)), None)
    return _CJK['font']


class _PilText:
    """用 PIL 画中文。图像是 BGR，这里不转通道：颜色也按 BGR 给，画完原样写回。"""

    def __init__(self, path, index):
        from PIL import Image, ImageDraw, ImageFont    # 延迟导入：没装 PIL 时由 text_renderer 退回英文
        self._image, self._draw = Image, ImageDraw
        self._truetype, self.path, self.index, self.fonts = ImageFont.truetype, path, index, {}
        self.probe = ImageDraw.Draw(Image.new('RGB', (1, 1)))

    def font(self, size):
        if size not in self.fonts:
            self.fonts[size] = self._truetype(self.path, size, index=self.index)
        return self.fonts[size]

    def width(self, text, size):
        return float(self.probe.textlength(text, font=self.font(size)))

    def render(self, image, items):
        """items = [(文字, x, y, 字号, BGR 颜色)]，(x, y) 是字的左上角。"""
        base = self._image.fromarray(image).convert('RGBA')
        layer = self._image.new('RGBA', base.size, (0, 0, 0, 0))
        draw = self._draw.Draw(layer)
        for text, x, y, size, color in items:
            draw.text((x, y), text, font=self.font(size), fill=(*color, 255))
        image[:] = np.asarray(self._image.alpha_composite(base, layer))[..., :3]


class _CvText:
    """没有中文字体时的英文退路：cv2.putText。字号（像素）按「Hershey 字号 = 像素 / 33」换算。"""

    PER_PX = 33.0

    def width(self, text, size):
        return float(cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size / self.PER_PX, 2)[0][0])

    def height(self, text, size):
        """字顶到基线的高度（putText 的 y 坐标是基线）。"""
        return int(cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size / self.PER_PX, 2)[0][1])

    def render(self, image, items):
        for text, x, y, size, color in items:
            cv2.putText(image, text, (x, y + self.height(text, size)), cv2.FONT_HERSHEY_SIMPLEX,
                        size / self.PER_PX, color, 2, cv2.LINE_AA)


def text_renderer():
    """(画字对象, 是不是中文)，只建一次。中文字体用不了（没装 PIL、字体加载失败）就退回英文，不报错。"""
    if 'text' not in _CJK:
        _CJK['chinese'] = False
        try:
            font = cjk_font()
            if font:
                _CJK['text'], _CJK['chinese'] = _PilText(*font), True
        except Exception:  # noqa: BLE001
            pass
        _CJK.setdefault('text', _CvText())
    return _CJK['text'], _CJK['chinese']


def _px(size, scale):
    """720p 下的字号 → 当前画面高度的字号（像素），最小 8。"""
    return max(8, int(round(size * scale)))


def _shade(image, rect, color):
    """给 rect 铺一层半透明底色（原地）；color = (B, G, R, 不透明度 0~255)。"""
    x0, y0, x1, y1 = rect
    alpha = color[3] / 255.0
    region = image[y0:y1, x0:x1]
    region[:] = region * (1.0 - alpha) + np.array(color[:3]) * alpha


def _fit(text, string, size, max_width):
    """太长就截掉末尾加省略号，画出来不超过 max_width 像素。"""
    while len(string) > 1 and text.width(string, size) > max_width:
        string = string[:-2] + '…'
    return string


class Overlay:
    """一帧画面上的说明文字，一块一块地摆（状态横幅、每个物体的标签、底部的当前目标）。

    每行字给中文和英文退路两种写法：有中文字体画中文，没有画英文。版面在这里算：底色矩形和每行字
    的位置都裁在画面以内，太长的字截断加省略号，所以物体框贴着画面边角时标签也不会越界。"""

    LINE_HEIGHT = 1.3     # 行高 = 字号 × 它

    def __init__(self, height, width):
        self.height, self.width, self.blocks, self.placed, self.texts = int(height), int(width), [], [], []

    def block(self, lines, x, y, pad=6, bg=(0, 0, 0, 170), full_width=False):
        """登记一块文字，画在 render 时才做。lines = [(中文, 英文退路, 字号 px, BGR 颜色)]；
        (x, y) = 块的左上角，y 传 'bottom' 贴着画面底边；bg = (B, G, R, 不透明度 0~255)，None 不画底；
        full_width = 底色横贯整幅（状态横幅）。"""
        self.blocks.append(dict(lines=lines, x=x, y=y, pad=pad, bg=bg, full_width=full_width))

    @staticmethod
    def block_height(lines, pad=6):
        """一块文字的高度（像素），调用方据此把它摆在物体框的上方或下方。"""
        return 2 * pad + sum(int(round(line[2] * Overlay.LINE_HEIGHT)) for line in lines)

    def render(self, image):
        """把登记的文字块画到 image（原地修改）。
        placed 记下每块底色、每行字最终的像素范围 (x0, y0, x1, y1)，texts 是实际画出来的字（截断后的）；
        测试拿它们查有没有越界、截断得对不对。"""
        text, chinese = text_renderer()
        items = []
        for block in self.blocks:
            lines, pad = block['lines'], block['pad']
            height = self.block_height(lines, pad)
            if block['full_width']:
                x0, x1, want = 0, self.width, self.width
            else:
                want = 2 * pad + max(text.width(line[0 if chinese else 1], line[2]) for line in lines)
            # 右边 / 下边放不下就往左 / 往上挪，整块留在画面里；块比画面还大才贴着左上角截断
            x0 = int(max(0, min(block['x'], self.width - want)))
            y0 = self.height - height if block['y'] == 'bottom' else block['y']
            y0 = int(max(0, min(y0, self.height - height)))
            x1, y1 = int(min(self.width, x0 + want)), min(self.height, y0 + height)
            self.placed.append((x0, y0, x1, y1))
            if block['bg'] is not None:
                _shade(image, (x0, y0, x1, y1), block['bg'])
            top = y0 + pad
            for line in lines:
                line_h = int(round(line[2] * self.LINE_HEIGHT))
                if top + line_h > y1:
                    break                               # 画面底边放不下剩下的行了
                tx, ty = x0 + pad, top + (line_h - line[2]) // 2
                shown = _fit(text, line[0 if chinese else 1], line[2], max(x1 - tx - pad, 1))
                items.append((shown, tx, ty, line[2], line[3]))
                self.texts.append(shown)
                self.placed.append((tx, top, tx + int(text.width(shown, line[2])) + 1, top + line_h))
                top += line_h
        text.render(image, items)


def banner_lines(phase, scale, warming=False):
    """状态横幅的三行字 (中文, 英文退路, 字号, BGR 颜色)：状态大字、按键、一行小字图例。"""
    status = (('预热中', 'warming up', (255, 255, 0)) if warming else
              ('夹持中', 'HOLDING', (255, 255, 255)) if phase == 'holding' else
              ('抓空', 'MISSED', (0, 0, 255)) if phase == 'missed' else
              ('实时识别', 'LIVE', (0, 255, 0)))
    keys = (('R 松开夹爪　　Q 松爪回安全位并退出', 'r release    q open gripper, safe pose and quit')
            if phase == 'holding' else
            ('G 抓取　　Q 松爪回安全位并退出', 'g grasp    q open gripper, safe pose and quit'))
    # 夹持中横幅是橙色底：上面的字都用白色
    on_banner = (255, 255, 255) if phase == 'holding' else (255, 255, 0)
    legend = ('图例：绿框 有掩码　橙框 无掩码（轴对齐框）或无有效抓取　白线 爪子开合方向（仅示意）　红点 抓取中心',
              'green box: mask    orange: no mask / no valid grasp    '
              'white line: jaw direction (schematic only)    red dot: grasp center')
    return [(status[0], status[1], _px(34, scale), status[2]),
            (keys[0], keys[1], _px(24, scale), on_banner),
            (legend[0], legend[1], _px(16, scale), on_banner if phase == 'holding' else (200, 200, 200))]


def draw_banner(overlay, phase, scale, warming=False):
    """顶部状态横幅：半透明底横贯整幅。夹持中换成橙色底，一眼看得出爪里有东西。"""
    overlay.block(banner_lines(phase, scale, warming), 0, 0, pad=_px(10, scale),
                  bg=(0, 80, 200, 160) if phase == 'holding' and not warming else (0, 0, 0, 150),
                  full_width=True)


def banner_height(scale):
    """状态横幅占的高度（像素）：物体标签要避开它。"""
    return Overlay.block_height(banner_lines('live', scale), _px(10, scale))


def draw_target(overlay, phase, best, scale):
    """画面底部「当前目标」一行：名称、置信度、爪宽、按 G 抓取；夹着 / 抓空时改成下一步的提示。"""
    size, green, orange, red = _px(22, scale), (120, 255, 140), (0, 165, 255), (0, 0, 255)
    if phase == 'holding':
        lines = [('夹着物体：按 R 松开，再按 G 抓下一个', 'Holding: r releases, then g for the next grasp',
                  size, (0, 128, 255))]
    elif phase == 'missed':
        lines = [('抓空：按 G 再抓', 'Missed: press g for the next grasp', size, red)]
    elif best is None:
        lines = [('当前目标：暂无有效目标', 'no valid grasp target', size, orange)]
    else:
        conf, name, rect, grasp = best
        x, y, z = grasp['position']
        lines = [(f'当前目标：{name} 置信度 {conf:.2f}　爪宽 {grasp["jaw_m"]*100:.1f} cm　按 G 抓取',
                  f'BEST {name} {conf:.2f}  jaw={grasp["jaw_m"]*100:.1f}cm  '
                  f'cam({x:+.2f},{y:+.2f},{z:+.2f})m  [g] grasp', size, green)]
    overlay.block(lines, _px(10, scale), 'bottom', pad=_px(6, scale))


def draw(image, name, conf, rect, grasp, valid, aabb, overlay, scale=1.0):
    """Annotate one detection: box, jaw line, grasp dot and a label block
    (above the box, or below it when the status banner leaves no room above).
    Boxes without a segmentation mask (axis-aligned) are orange even when valid."""
    color = (0, 255, 0) if valid and not aabb else (0, 165, 255)
    cv2.polylines(image, [np.round(rect).astype(np.int32)], True, color, 2)
    if grasp is not None:
        vec = grasp['line_vector']
        center = np.array(grasp['center'], float)
        p0 = tuple(np.round(center - vec / 2).astype(int))
        p1 = tuple(np.round(center + vec / 2).astype(int))
        cv2.line(image, p0, p1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(image, grasp['center'], 4, (0, 0, 255), -1)
    size, yellow = _px(18, scale), (0, 255, 255)
    head = (f'{name} 置信度 {conf:.2f}', f'{name} {conf:.2f}')
    if grasp is None:
        lines = [(*head, size, color), ('无有效抓取几何', 'no valid grasp geometry', size, yellow)]
    else:
        x, y, z = grasp['position']
        lines = [(*head, size, color),
                 (f"爪宽 {grasp['jaw_m']*100:.1f} cm", f"jaw={grasp['jaw_m']*100:.1f}cm", size, yellow),
                 (f'相机系 ({x:+.2f}, {y:+.2f}, {z:+.2f}) m', f'cam({x:+.2f},{y:+.2f},{z:+.2f})m', size, yellow)]
    pad = _px(6, scale)
    y = int(round(float(rect[:, 1].min()))) - Overlay.block_height(lines, pad) - 4
    if y < banner_height(scale):                # 横幅占着上方：标签放到框下面，也不压横幅
        y = max(int(round(float(rect[:, 1].max()))) + 4, banner_height(scale))
    overlay.block(lines, int(round(float(rect[:, 0].min()))), y, pad=pad)


def plan_grasp(arm, object_pos, nominal, current_q, g):
    """Read-only IK preflight. Tilt about the jaw line so its direction is preserved.
    Returns None, or pre/end poses, the joint angles pre_q/end_q and line_q (the IK
    samples every 1 cm from pre to end, what plan_stream streams along)."""
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
                                line_q=np.array(joints), tilt=tilt, flipped=flipped, max_j5=score[0])
                    candidates.append((score, plan))
        if candidates:
            return min(candidates, key=lambda item: item[0])[1]
    return None


def quintic(u):
    """0→1 的五次多项式（两端速度、加速度为 0；峰值速度 1.875，峰值加速度 5.77）。"""
    u = min(max(float(u), 0.0), 1.0)
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def _by_arclength(points, step):
    """折线按关节空间弧长从起点到终点等步长重采样。返回 (样本, 实际步长)。"""
    points = np.asarray(points, float)
    s = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
    count = max(2, int(np.ceil(s[-1] / step)) + 1)
    grid = np.linspace(0.0, s[-1], count)
    return np.column_stack([np.interp(grid, s, points[:, j]) for j in range(points.shape[1])]), s[-1] / (count - 1)


def _hermite(knots):
    """过 knots（直线段的逆解采样点，每 1 cm 一个）的三次 Hermite 样条，参数取关节空间弦长，密采样。
    直接把逆解点连成折线的话节点处有折角，离散二阶导很大，转弯限速会被压没。"""
    P = np.asarray(knots, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    tangent = np.empty_like(P)
    tangent[0], tangent[-1] = (P[1] - P[0]) / seg[0], (P[-1] - P[-2]) / seg[-1]
    tangent[1:-1] = (P[2:] - P[:-2]) / (s[2:] - s[:-2])[:, None]
    u = np.linspace(0.0, 1.0, 50, endpoint=False)[1:, None]
    dense = [P[:1]]
    for i in range(len(P) - 1):
        chord = (P[i + 1] - P[i]) / seg[i]
        m0, m1 = tangent[i], tangent[i + 1]
        dense.append(P[i] + seg[i] * (u * m0 + u * u * (3 * chord - 2 * m0 - m1)
                                      + u ** 3 * (m0 + m1 - 2 * chord)))
    dense.append(P[-1:])
    return np.vstack(dense)


def stream_path(q_start, pre_q, line_q, pre, end, blend_m, fk, step=STREAM_PATH_STEP_DEG):
    """去程的几何路径（纯几何，不带时间），全程在关节空间按弧长等步长采样：
    ① 关节直线：当前位 q_start → 预抓取点 pre_q；② 预抓取点处的圆角；③ 直线段：line_q（plan_grasp 的
    逆解采样点，预抓取点 → 抓取点）。

    圆角：①的末尾和③的开头各取同样的关节空间长度，用五次多项式权重从①过渡到③。两端切向就是两段
    各自的方向，所以预抓取点处连续转弯、不停。为什么必须有：从准备位过来的关节直线，到预抓取点要转的
    方向在关节空间里很大（腕部转到位、J3/J5 反向），按加速度上限做完要零点几秒、几厘米路程——
    做成尖角就只能停住了再走。圆角在预抓取点高度处会少许偏离下降直线，plan_stream 按上限收窄它。
    fk：关节角（URDF，度）→ 末端位置（米）。
    返回 dict(points (N×6), kind (N), step, tcp (N×3，关节直线段为 NaN), pre_index, deviation)：
    kind = 0/1/2 对应上面三段；pre_index = 末端过了预抓取点高度的第一个下标（在圆角里）；
    deviation = 预抓取点高度及其以下，末端到下降直线（过抓取点 end、沿 pre→end）的最大距离（米）。"""
    q_start, pre_q = np.asarray(q_start, float), np.asarray(pre_q, float)
    pre, end = np.asarray(pre, float), np.asarray(end, float)
    if (np.linalg.norm(np.diff(np.asarray(line_q, float), axis=0), axis=1).sum() < 2 * step
            or np.linalg.norm(pre_q - q_start) < 2 * step):
        raise ValueError('degenerate path: the joints barely move between the current pose, '
                         'the pregrasp point and the grasp point')
    line, line_step = _by_arclength(_hermite(line_q), step)
    along = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(
        [fk(q) for q in line], axis=0), axis=1))))                          # 直线段末端的累计路程
    last = int(np.clip(np.searchsorted(along, blend_m), 1, len(line) - 1))   # 圆角在直线段上取到的下标
    reach = last * line_step                                                 # 这一段的关节空间长度
    leg = pre_q - q_start
    if reach < 2 * step or np.linalg.norm(leg) < reach + 2 * step:
        raise ValueError('degenerate path: the joints barely move between the current pose, '
                         'the pregrasp point and the grasp point')
    first = pre_q - reach / np.linalg.norm(leg) * leg                        # 圆角在关节直线上的起点
    corner = [first]
    for w in np.linspace(0.0, 1.0, max(6, int(np.ceil(2 * reach / step))), endpoint=False)[1:]:
        position = w * last                                                   # 直线段一侧按弧长等比取
        lo = int(position)
        on_line = line[lo] + (position - lo) * (line[min(lo + 1, last)] - line[lo])
        on_leg = pre_q - (1 - w) * reach / np.linalg.norm(leg) * leg
        corner.append((1 - quintic(w)) * on_leg + quintic(w) * on_line)
    corner = np.array(corner)
    turn = float(np.linalg.norm(np.diff(corner, axis=0), axis=1).sum())       # 圆角的关节空间长度
    points, step = _by_arclength(np.vstack((q_start, corner, line[last:])), step)
    s = np.arange(len(points)) * step
    to_first = float(np.linalg.norm(leg) - reach)
    kind = np.where(s < to_first - 1e-9, 0, np.where(s < to_first + turn, 1, 2)).astype(int)
    tcp = np.full((len(points), 3), np.nan)
    curved = np.nonzero(kind >= 1)[0]
    tcp[curved] = [fk(q) for q in points[curved]]
    axis = (end - pre) / np.linalg.norm(end - pre)                           # 下降直线的方向
    below = curved[(tcp[curved] - pre) @ axis >= -1e-9]                      # 过了预抓取点高度的点
    deviation = (float(np.linalg.norm(np.cross(tcp[below] - end, axis), axis=1).max())
                 if len(below) else 0.0)
    return dict(points=points, kind=kind, step=step, tcp=tcp, blend_m=float(blend_m),
                pre_index=int(below[0]) if len(below) else int(curved[0]), deviation=deviation)


def _time_law(points, tcp, kind, step, v_joint, acc, v_tcp, a_tcp):
    """路径 → 时间律（两端静止）。每个路径点先算路径速度上限：逐轴关节速度、圆角和直线段的末端速度、
    向心加速度（曲率）；再从起点加速到上限、从终点倒推减速，两头取小。加速度约束逐轴
    |q'·α + q''·v²| ≤ acc（q'、q'' 是对路径弧长的导数，v、α 是路径速度、加速度），直线段另加
    末端切向加速度 ≤ a_tcp。v_joint 按段给（关节直线 / 圆角 / 直线段）。
    返回 (每点时刻 t, 每点路径速度)。中途速度降到 0 说明这条路在这些上限下走不动，报 ValueError。"""
    q1 = np.gradient(points, step, axis=0)
    q2 = np.gradient(q1, step, axis=0)
    acc = acc * STREAM_ACCEL_MARGIN
    abs_q1 = np.maximum(np.abs(q1), 1e-9)
    vmax = np.min(np.asarray(v_joint)[kind][:, None] / abs_q1, axis=1)
    vmax = np.minimum(vmax, np.sqrt(acc / np.maximum(np.abs(q2).max(axis=1), 1e-12)))
    hi_tcp = np.full(len(points), np.inf)
    lo_tcp = np.full(len(points), -np.inf)
    curved = np.nonzero(kind >= 1)[0]
    if len(curved):
        t1 = np.gradient(tcp[curved], step, axis=0)
        speed = np.maximum(np.linalg.norm(t1, axis=1), 1e-9)
        vmax[curved] = np.minimum(vmax[curved], v_tcp / speed)
        straight = kind[curved] == 2
        tangential = np.einsum('ij,ij->i', np.gradient(t1, step, axis=0), t1) / speed ** 2
        hi_tcp[curved[straight]] = (a_tcp * STREAM_ACCEL_MARGIN - tangential[straight]) / speed[straight]
        lo_tcp[curved[straight]] = (-a_tcp * STREAM_ACCEL_MARGIN - tangential[straight]) / speed[straight]
    hi0, ratio = acc / abs_q1, q2 / abs_q1     # 逐轴：可加速 α ≤ hi0 − ratio·v²，可减速 α ≥ −hi0 − ratio·v²
    n = len(points)
    vel = np.zeros(n)
    for i in range(n - 1):
        push = min((hi0[i] - ratio[i] * vel[i] ** 2).min(), hi_tcp[i])
        vel[i + 1] = min(vmax[i + 1], np.sqrt(vel[i] ** 2 + 2.0 * max(push, 0.0) * step))
    for i in range(n - 1, 0, -1):
        brake = min((hi0[i - 1] + ratio[i - 1] * vel[i - 1] ** 2).min(), -lo_tcp[i - 1])
        vel[i - 1] = min(vel[i - 1], np.sqrt(vel[i] ** 2 + 2.0 * max(brake, 0.0) * step))
    if vel[1:-1].min() <= 1e-6:
        raise ValueError('the path cannot be followed without stopping under the configured '
                         'speed and acceleration limits')
    return np.concatenate(([0.0], np.cumsum(2.0 * step / (vel[:-1] + vel[1:])))), vel


def j6_profile(stamps, j6_from, j6_to, speed=STREAM_J6_SPEED_DPS, accel=STREAM_J6_ACC_DPS2):
    """J6 预合拢：从 j6_from（舵机角，度）合到 j6_to 的梯形速度曲线，按每帧时刻取样。
    峰值速度 ≤ speed（每帧步长 ≤ speed·0.05 s < 1°），加减速各 accel；行程太短走不到 speed 就是三角形。
    返回 (每帧 J6 目标, (开始时刻, 合完时刻))。"""
    delta = float(j6_to) - float(j6_from)
    distance, ramp = abs(delta), speed / accel
    if distance <= speed * ramp:
        duration, cruise = 2.0 * math.sqrt(distance / accel), 0.0
    else:
        duration, cruise = distance / speed + ramp, speed

    def travelled(t):
        t = min(max(t, 0.0), duration)
        if t < ramp or cruise == 0.0 and t <= duration / 2:
            return 0.5 * accel * t * t
        if t > duration - ramp:
            return distance - 0.5 * accel * (duration - t) ** 2
        return cruise * t - 0.5 * cruise * ramp if cruise else distance - 0.5 * accel * (duration - t) ** 2

    return ([j6_from + math.copysign(travelled(t), delta) for t in stamps], (0.0, duration))


def _frames(points, times, dt=STREAM_DT_S):
    """按时间律每 dt 取一帧（线性插值），末尾补上轨迹终点。返回 (每帧关节角 (K×6), 每帧时刻)。"""
    end = float(times[-1])
    stamps = np.arange(0.0, end, dt)
    if end - stamps[-1] < 1e-6:
        stamps[-1] = end
    else:
        stamps = np.append(stamps, end)
    index = np.interp(stamps, times, np.arange(len(times)))
    lo = np.minimum(np.floor(index).astype(int), len(times) - 2)
    frac = (index - lo)[:, None]
    return points[lo] * (1 - frac) + points[lo + 1] * frac, stamps


def discrete_limits(points, stamps):
    """按帧采样后的逐轴最大速度（°/s）、最大加速度（°/s²）：相邻帧差分、二阶差分（帧距不等时也按实际帧距算）。"""
    points = np.asarray(points, float)
    h1, h2 = np.diff(stamps)[:-1], np.diff(stamps)[1:]
    first = np.diff(points, axis=0)
    speed = np.abs(first) / np.diff(stamps)[:, None]
    accel = 2.0 * (first[1:] / h2[:, None] - first[:-1] / h1[:, None]) / (h1 + h2)[:, None]
    return speed.max(axis=0), np.abs(accel).max(axis=0)


def plan_stream(arm, plan, g, preclose_mm=None, open_mm=None, home_q=None):
    """抓取执行段的两条流式轨迹和 J6 预合拢。几何是同一条路径（当前位 → 预抓取点 → 抓取点），
    去程正着走、回程倒着走，各配各的时间律：
    - 去程：全程各轴关节速度 ≤ grasp_vel_dps、加速度 ≤ acc_dps2；圆角起末端速度 ≤ linear_vel_mps，
      直线段末端切向加速度 ≤ linear_acc_mps2；在抓取点减速到停。
    - 回程：抓取点由静止起步，预抓取点不停；直线段同上；回准备位的关节直线段各轴 ≤ approach_vel_dps；
      在准备位减速到停。home_q（准备位）与去程起点不是同一姿态时，回程的关节直线段接到 home_q。
    - J6 预合拢（preclose_mm 给了才做）：去程一开始就从张口 open_mm 合到 preclose_mm，梯形速度，
      峰值速度 ≤ STREAM_J6_SPEED_DPS（每帧步长 < 1°）。行程大、20°/s 来不及在到预抓取点之前合完的，
      也不超速，照常合完为止（迟合只会让下降时两指更宽，不会压物体）。
    圆角在预抓取点高度处偏离下降直线超过 STREAM_BLEND_MAX_DEV_M 时自动收窄（最多到 STREAM_BLEND_MIN_M）。
    返回 dict(approach, retreat, j6, t_pre, j6_window, blend_m, deviation_m, pre_tcp_mps,
    approach_s, retreat_s, stamps, retreat_stamps)：approach / retreat 是每帧一行的关节角（URDF，度），
    stamps / retreat_stamps 是每帧的时刻；j6 与 approach 等长
    （J6 舵机角，度），不预合拢时为 None；t_pre = 去程末端过预抓取点高度的时刻；j6_window = J6 开始、
    合完的时刻（都相对去程起点）。"""
    fk = lambda q: np.asarray(arm.call('fk', q=[float(v) for v in q])['pose']['pos'], float)
    start = np.asarray(arm.command_q(), float)
    home = start if home_q is None else np.asarray(home_q, float)

    def fit(begin):                      # 圆角收到预抓取点高度处的偏离不超上限为止
        blend, path = STREAM_BLEND_M, None
        for _ in range(6):
            path = stream_path(begin, plan['pre_q'], plan['line_q'], plan['pre'], plan['end'], blend, fk)
            if path['deviation'] <= STREAM_BLEND_MAX_DEV_M or blend <= STREAM_BLEND_MIN_M:
                break
            blend = max(STREAM_BLEND_MIN_M, blend * STREAM_BLEND_MAX_DEV_M / path['deviation'])
        return path

    def timed(path, v_joint, forward):
        points, tcp, kind = (path['points'], path['tcp'], path['kind']) if forward else (
            path['points'][::-1], path['tcp'][::-1], path['kind'][::-1])
        times, speed = _time_law(points, tcp, kind, path['step'], v_joint, float(arm.cfg['acc_dps2']),
                                 float(g['linear_vel_mps']), float(g['linear_acc_mps2']))
        return _frames(points, times), (points, tcp, kind, times, speed)

    grasp_vel, home_vel = float(g['grasp_vel_dps']), float(arm.cfg['approach_vel_dps'])
    path = fit(start)
    (approach, stamps), (points, tcp, kind, times, speed) = timed(path, (grasp_vel,) * 3, True)
    away = path if np.max(np.abs(home - start)) <= 0.5 else fit(home)
    (retreat, retreat_stamps), _ = timed(away, (home_vel, grasp_vel, grasp_vel), False)
    tcp_speed = np.zeros(len(points))
    curved = np.nonzero(kind >= 1)[0]
    tcp_speed[curved] = np.linalg.norm(np.gradient(tcp[curved], path['step'], axis=0), axis=1) * speed[curved]
    j6, j6_window = None, None
    if preclose_mm is not None:
        j6, j6_window = j6_profile(stamps, gripper_mm_to_deg(open_mm), gripper_mm_to_deg(preclose_mm))
    pre = path['pre_index']
    return dict(approach=approach, retreat=retreat, j6=j6, j6_window=j6_window, t_pre=float(times[pre]),
                blend_m=path['blend_m'], deviation_m=path['deviation'], pre_tcp_mps=float(tcp_speed[pre]),
                approach_s=float(stamps[-1]), retreat_s=float(retreat_stamps[-1]),
                stamps=stamps, retreat_stamps=retreat_stamps)


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
    if g['approach'] not in ('top_down', 'camera_ray', 'cloud_top_down'):
        raise ValueError('grasp.approach must be top_down, camera_ray or cloud_top_down')
    if g['final_motion'] not in ('linear', 'joint'):
        raise ValueError('grasp.final_motion must be linear or joint')
    for key in ('linear_vel_mps', 'linear_acc_mps2', 'pregrasp_offset_m', 'grasp_vel_dps'):
        if not np.isfinite(g[key]) or g[key] <= 0:
            raise ValueError(f'grasp.{key} must be finite and positive')
    if not 0 <= g['j5_margin_deg'] < 90:
        raise ValueError('grasp.j5_margin_deg must be in [0, 90)')
    if not np.isfinite(g['gripper_preclose_margin_mm']) or g['gripper_preclose_margin_mm'] < 0:
        raise ValueError('grasp.gripper_preclose_margin_mm must be finite and not negative')
    if not isinstance(g['table_self_correct'], bool):
        raise ValueError('grasp.table_self_correct must be true or false')
    if not np.isfinite(g['table_self_correct_max_deg']) or not 0 < g['table_self_correct_max_deg'] < 90:
        raise ValueError('grasp.table_self_correct_max_deg must be in (0, 90)')
    g['base_offset_m'] = np.asarray(g['base_offset_m'], dtype=float)
    if g['base_offset_m'].shape != (3,) or not np.all(np.isfinite(g['base_offset_m'])):
        raise ValueError('grasp.base_offset_m must be three finite numbers [dx, dy, dz] in metres')
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
    motion_sent = False
    grasp_count, miss_count = 0, 0   # 本次运行夹住 / 抓空的次数（grasps 是每帧的检测结果列表，别重名）
    phase = 'live'                   # live = 可以按 g 抓；holding = 夹着物体在准备位；missed = 上一次抓空
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, cfg['camera']['width'], cfg['camera']['height'])
    quit_requested = False

    def pump():
        """抓取执行段里每个阻塞等待（流式每帧、到位等待、闭爪轮询、回准备位）每拍调一次：处理窗口事件。
        不处理的话 GNOME 认为窗口未响应（check-alive-timeout 5 s），弹「强制退出」，点了进程就被 SIGKILL。
        执行期间按的 q / Esc 只记下来，等这一趟抓取回到准备位再执行，不在运动中途中断。"""
        nonlocal quit_requested
        try:
            pressed = cv2.waitKey(1) & 0xFF
        except cv2.error:               # 窗口已经没了
            return
        if pressed in (ord('q'), ord('Q'), 27):
            quit_requested = True

    try:
        arm.wait_still(pump=pump)
        if not args.dry_run:
            motion_sent = True
            log('Opening gripper and moving to ready pose')
            arm.call('gripper', mm=g['gripper_open_mm'], pump=pump)
            arm.move(robot['ready_pose_deg'], robot['approach_vel_dps'], pump)
        log('Live view starting; the first YOLO inference can take tens of seconds on CPU')
        warm = np.zeros((cfg['camera']['height'], cfg['camera']['width'], 3), np.uint8)
        overlay = Overlay(*warm.shape[:2])     # 第一次推理可能很慢：先摆一帧「预热中」
        draw_banner(overlay, phase, warm.shape[0] / 720.0, warming=True)   # 字号随画面高度缩放
        overlay.render(warm)
        cv2.imshow(WINDOW, warm)
        cv2.waitKey(1)
        frame_index, latest, best = 0, None, None
        frame_depth = None  # 最新标注帧的深度（与 best 的掩码同一帧，cloud_top_down 用）
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
                text_scale = color.shape[0] / 720.0   # 画面文字的字号按 720p 标，随画面高度缩放（scale 是深度比例，别重名）
                overlay = Overlay(*color.shape[:2])
                # 竖直下抓取物体区域里最近的一小撮深度=顶面：透明物体/托盘的红外穿透读数偏远，被自然排除
                quantile = g['top_depth_quantile'] if g['approach'] == 'top_down' else g['depth_quantile']
                for name, conf, _box, rect, contour in detections(results, color.shape, classes):
                    # 没有分割掩码的轴对齐框：cloud_top_down 照常接（日志标 box=aabb），其余方式不认
                    grasp = (grasp_camera(rect, depth_mm, k, quantile, contour)
                             if contour is not None or g['approach'] == 'cloud_top_down' else None)
                    grasps.append((conf, name, rect, grasp))
                    draw(color, name, conf, rect, grasp, grasp is not None, contour is None, overlay, text_scale)
                valid = [item for item in grasps if item[3] is not None]
                best = max(valid, key=lambda item: item[0]) if valid else None
                # 说明文字只在标注新帧时画一次：按键状态一变 latest 就被清掉，下一帧自然换成新的横幅
                draw_banner(overlay, phase, text_scale)
                draw_target(overlay, phase, best, text_scale)
                overlay.render(color)
                latest, frame_depth = color, depth_mm
            cv2.imshow(WINDOW, latest)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27) or quit_requested:
                log('Opening gripper, returning to safe pose' + (' (q pressed during the grasp sequence)'
                                                                  if quit_requested else ''))
                if not args.dry_run:
                    arm.call('gripper', mm=g['gripper_open_mm'], pump=pump)
                    arm.move(robot['safe_pose_deg'], robot['approach_vel_dps'], pump)
                break
            if key in (ord('r'), ord('R')):
                if phase != 'holding':
                    log('Nothing held')
                else:
                    release(arm, g['gripper_open_mm'], args.dry_run, pump)
                    phase = 'live'
                    latest, best = None, None  # Re-annotate from the new scene.
            if key in (ord('g'), ord('G')) and best is not None:
                if phase == 'holding':
                    log('Gripper is holding the previous target; r releases it here, '
                        'then g for the next grasp')
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
                jaw_m = grasp['jaw_m']
                preclose_mm = None             # cloud_top_down：下降前先把夹爪预合拢；其余方式不合
                close_mm = close_target_mm(jaw_m, g['gripper_close_factor'],
                                           g['gripper_close_min_mm'], g['gripper_open_mm'])
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
                elif g['approach'] == 'cloud_top_down':
                    # 检测点改取点云物体中心（基座系）：分位深度点落在朝相机的表面上，会把抓点带向底座
                    cloud = object_from_cloud(grasp['mask'], frame_depth, k, t_base_camera, g,
                                              grasp['center_px'], grasp['ends_px'], rect, q_deg=q_meas)
                    if cloud is None:
                        continue  # 原因已打印；等下一次选择
                    object_pos = cloud['position']
                    jaw_m = cloud['jaw_m']
                    nominal = _tcp_frame(np.array([0.0, 0.0, -1.0]), cloud['open_dir'], g['open_axis'])
                    # 指尖下限以配置的真实桌面为基准：测出的桌面可能整体下沉，拿它算会撞桌
                    floor = finger_floor_z(g['support_z_m'], g)
                    if object_pos[2] < floor:
                        log(f'Grasp point raised from z {object_pos[2]:.3f} to {floor:.3f} m: '
                            f'fingertips stay {FINGER_CLEARANCE_M*1000:.0f} mm off the table')
                        object_pos[2] = floor
                    close_mm = close_margin_mm(jaw_m, g['gripper_close_margin_mm'],
                                               g['gripper_close_min_mm'], g['gripper_open_mm'])
                    preclose_mm = None             # 余量 0 = 不预合拢，下降时全开
                    if g['gripper_preclose_margin_mm'] > 0:
                        preclose_mm = preclose_target_mm(jaw_m, g['gripper_preclose_margin_mm'],
                                                         g['gripper_open_mm'])
                    if preclose_mm is not None and preclose_mm >= g['gripper_open_mm']:
                        preclose_mm = None         # 爪宽加余量已不小于张口：没什么可先合的
                    ring_kept = '' if cloud['ring_kept'] is None else f", ring kept {cloud['ring_kept']*100:.0f}%"
                    log(f"cloud_top_down {name}: center ({object_pos[0]:+.3f},{object_pos[1]:+.3f},"
                        f"{object_pos[2]:.3f}) m, height {cloud['height_m']*1000:.0f}mm, "
                        f"table_z {cloud['table_z']:+.3f}m ({cloud['table_source']}{ring_kept}), "
                        f"plane offset {cloud['plane_offset_m']*1000:+.1f}mm "
                        f"tilt {cloud['plane_tilt_deg']:.1f}deg, "
                        f"jaw {jaw_m*100:.1f}cm ({cloud['jaw_source']}), close {close_mm:.1f}mm, "
                        f"along-axis shift {abs(cloud['along_shift_m'])*100:.1f}cm"
                        f"{', box=aabb' if cloud['aabb'] else ''}, {cloud['n_points']} points")
                else:
                    nominal = tcp_rotation(grasp, t_base_camera, g['open_axis'])
                if np.any(g['base_offset_m']):  # 复刻时的固定偏差补偿（基座系，米）
                    object_pos = object_pos + g['base_offset_m']
                    log(f"Applied base_offset_m {g['base_offset_m'].round(3).tolist()}")
                plan = plan_grasp(arm, object_pos, nominal, q_meas, g)
                if plan is None:
                    log('No feasible grasp for this frame; waiting for another selection')
                    continue
                pos, quat = plan['end'], plan['quat']
                log(f"Pregrasp {plan['pre'].round(3)}, tilt={plan['tilt']} deg, "
                    f"flip={plan['flipped']}, max sampled |J5|={plan['max_j5']:.1f} deg, "
                    f"final motion={g['final_motion']}")
                log(f'Grasp {name}: base xyz ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) '
                    f'jaw {jaw_m*100:.1f}cm -> close {close_mm:.1f}mm '
                    f'(offset {g["grasp_offset_m"]*100:.1f}cm) '
                    f'quat {np.round(quat, 3).tolist()}')
                motion = None
                if g['final_motion'] == 'linear':
                    # 流式连续轨迹：去程一条（不停预抓取点）、回程一条，J6 预合拢并在去程里同步做
                    try:
                        motion = plan_stream(arm, plan, g, preclose_mm, g['gripper_open_mm'],
                                             home_q=robot['ready_pose_deg'])
                    except (RobotFault, ValueError) as exc:
                        log(f'No streamed trajectory for this grasp ({exc}); waiting for another selection')
                        continue
                    log(f"stream plan: approach {motion['approach_s']:.2f}s ({len(motion['approach'])} frames, "
                        f"pregrasp point passed at {motion['t_pre']:.2f}s with the TCP at "
                        f"{motion['pre_tcp_mps']:.2f} m/s), retreat {motion['retreat_s']:.2f}s; "
                        f"corner blend {motion['blend_m'] * 1000:.0f} mm, at most "
                        f"{motion['deviation_m'] * 1000:.1f} mm off the straight line at the pregrasp height")
                    if motion['j6'] is not None:
                        log(f"j6 preclose: {motion['j6_window'][0]:.2f}s-{motion['j6_window'][1]:.2f}s after the "
                            'approach begins' + (' (not finished before the pregrasp point: '
                                                 'capped at 20 deg/s)' if motion['j6_window'][1] > motion['t_pre'] else ''))
                if args.dry_run:
                    log((f"timing (planned): approach {motion['approach_s']:.2f}s, "
                         f"retreat {motion['retreat_s']:.2f}s") if motion else
                        'timing (planned): joint mode, the segments run back to back and stop at each waypoint')
                    continue
                # The blocking sequence below naturally freezes the window on
                # the frame captured at key-press time; every wait in it pumps
                # window events (see pump).
                clock = time.monotonic()
                sent = None
                if motion is None:                   # final_motion=joint：分段走，每段到位停稳（同以前）
                    arm.move(plan['pre_q'].tolist(), g['grasp_vel_dps'], pump)
                    if preclose_mm is not None:
                        preclose(arm, preclose_mm, pump=pump)
                    arm.move(plan['end_q'].tolist(), g['grasp_vel_dps'], pump)
                else:
                    sent = arm.stream(motion['approach'], j6=motion['j6'], pump=pump)
                    arm.wait_arrived(plan['end_q'], pump)   # 轨迹末尾已减速到停：只判到位容差，不等稳定窗
                approach_s = time.monotonic() - clock
                for _ in range(int(g['wait_before_grip_s'] * 10)):
                    pump()
                    time.sleep(0.1)  # Keep the window alive during the settle wait.
                grip_started = time.monotonic()
                missed = False
                try:
                    close_and_lock(arm, close_mm, g, pump)
                except GraspMiss as exc:
                    # 抓空不是故障：爪已松开，照常原路退回准备位，可以直接再按 g
                    missed, miss_count = True, miss_count + 1
                    log(f'MISS: {exc}')
                grip_s = time.monotonic() - grip_started
                log('Retreating to the ready pose' + (' through the pregrasp point without stopping'
                                                       if motion else ' via the pregrasp point'))
                if motion is None:
                    arm.move(plan['pre_q'].tolist(), g['grasp_vel_dps'], pump)
                    arm.move(robot['ready_pose_deg'], robot['approach_vel_dps'], pump)
                else:
                    arm.stream(motion['retreat'], pump=pump)
                    arm.wait_still(robot['ready_pose_deg'], pump)   # 只在准备位确认一次到位
                retreat_s = time.monotonic() - clock - approach_s - grip_s
                log(f"timing: approach {approach_s:.2f}s, grip {grip_s:.2f}s, retreat {retreat_s:.2f}s, "
                    f"total {approach_s + grip_s + retreat_s:.2f}s")
                if motion is not None and motion['j6'] is not None:
                    log(f"j6 preclose: {sent.get('j6_start_s', 0.0):.2f}s-{sent.get('j6_end_s', 0.0):.2f}s "
                        'after the approach began (as streamed)')
                if missed:
                    phase = 'missed'
                    log('Missed; press g for the next grasp')
                else:
                    grasp_count, phase = grasp_count + 1, 'holding'
                    log('Holding the target; press r to release, then g for the next grasp')
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
    log('Finished' + (f' ({grasp_count} grasped, {miss_count} missed)' if grasp_count + miss_count else ''))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
