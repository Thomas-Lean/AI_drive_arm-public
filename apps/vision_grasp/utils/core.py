"""手眼标定几何与数学核心（纯函数，无 IO）。

内容：四元数/位姿矩阵转换、刚体变换校验、旋转平均与散布、采样多样性
（旋转/平移跨度、轴比=相对旋转奇异值之比）、PARK/TSAI/HORAUD 手眼求解、
全样本残差+固定 1/4 留出验证的质量评估、早停判据、ArUco 检测器
（IPPE_SQUARE + 双解歧义拒收）。
"""
from itertools import combinations
import cv2
import numpy as np


def transform(rotation, translation):
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = np.asarray(translation).reshape(3)
    return out


def pose_matrix(pose):
    position = np.asarray(pose['pos'], dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('Invalid SDK position')
    w, x, y, z = np.asarray(pose['quat'], dtype=float)
    norm = np.linalg.norm([w, x, y, z])
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid SDK quaternion')
    w, x, y, z = np.array([w, x, y, z]) / norm
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return transform(r, pose['pos'])


def validate_transforms(ts):
    ts = np.asarray(ts)
    if ts.ndim != 3 or ts.shape[1:] != (4,4) or not np.isfinite(ts).all():
        raise ValueError('Malformed transform dataset')
    for t in ts:
        r = t[:3,:3]
        if (not np.allclose(t[3], [0,0,0,1], atol=1e-8) or
                not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or
                abs(np.linalg.det(r)-1) > 1e-5):
            raise ValueError('Dataset contains a non-rigid transform')


def angle_deg(r):
    return float(np.degrees(np.linalg.norm(cv2.Rodrigues(r)[0])))


def mean_transform(ts):
    u, _, vt = np.linalg.svd(np.mean(ts[:, :3, :3], axis=0))
    r = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return transform(r, np.mean(ts[:, :3, 3], axis=0))


def spread(ts, reference):
    translation = np.linalg.norm(ts[:, :3, 3] - reference[:3, 3], axis=1)
    rotation = np.array([angle_deg(reference[:3, :3].T @ t[:3, :3]) for t in ts])
    return translation, rotation


def diversity(ts):
    vectors = [cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0].ravel()
               for a, b in combinations(ts, 2)]
    if not vectors:
        return dict(rotation_span_deg=0., translation_span_m=0., axis_ratio=0.)
    singular = np.linalg.svd(np.array(vectors), compute_uv=False)
    return dict(rotation_span_deg=float(np.degrees(np.max(np.linalg.norm(vectors, axis=1)))),
                translation_span_m=max(float(np.linalg.norm(a[:3, 3]-b[:3, 3]))
                                       for a, b in combinations(ts, 2)),
                axis_ratio=float(singular[1]/max(singular[0], 1e-12)) if len(singular)>1 else 0.)


def solve(bg, ct, method):
    methods = {'PARK': cv2.CALIB_HAND_EYE_PARK, 'TSAI': cv2.CALIB_HAND_EYE_TSAI,
               'HORAUD': cv2.CALIB_HAND_EYE_HORAUD}
    r, t = cv2.calibrateHandEye(list(bg[:, :3, :3]), list(bg[:, :3, 3]),
                              list(ct[:, :3, :3]), list(ct[:, :3, 3]),
                              method=methods[method])
    x = transform(r, t)
    if (not np.isfinite(x).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-5)
            or abs(np.linalg.det(r)-1) > 1e-5):
        raise ValueError('Hand-eye solution is degenerate')
    return x


def assess(bg, ct, cfg):
    """All-sample residual + deterministic held-out validation; no silent outlier removal."""
    bg, ct = np.asarray(bg, float), np.asarray(ct, float)
    validate_transforms(bg)
    validate_transforms(ct)
    if bg.shape != ct.shape:
        raise ValueError('Unpaired transform dataset')
    metrics = {'samples': len(bg)}
    if len(bg) < cfg['min_samples']:
        return None, metrics, 'insufficient samples'
    metrics.update(diversity(bg))
    checks = [('rotation_span_deg', 'min_rotation_span_deg'),
              ('translation_span_m', 'min_translation_span_m'), ('axis_ratio', 'min_axis_ratio')]
    for key, limit in checks:
        if metrics[key] < cfg[limit]:
            return None, metrics, 'insufficient diversity: ' + key
    try:
        x = solve(bg, ct, cfg['method'])
        fixed = bg @ x @ ct
        center = mean_transform(fixed)
        dt, dr = spread(fixed, center)
        metrics.update(translation_rms_m=float(np.sqrt(np.mean(dt**2))),
                       rotation_rms_deg=float(np.sqrt(np.mean(dr**2))),
                       translation_max_m=float(max(dt)), rotation_max_deg=float(max(dr)))
        # A fixed 1-in-4 holdout prevents acceptance from training residual alone.
        hold = np.arange(len(bg)) % 4 == 0
        train_diversity = diversity(bg[~hold])
        if any(train_diversity[key] < cfg[limit] for key, limit in checks):
            return None, metrics, 'training subset lacks diversity'
        trial = solve(bg[~hold], ct[~hold], cfg['method'])
        reference = mean_transform(bg[~hold] @ trial @ ct[~hold])
        ht, hr = spread(bg[hold] @ trial @ ct[hold], reference)
        metrics.update(holdout_translation_max_m=float(max(ht)),
                       holdout_rotation_max_deg=float(max(hr)))
        for key, limit in [('translation_rms_m', 'max_translation_rms_m'),
                           ('rotation_rms_deg', 'max_rotation_rms_deg'),
                           ('translation_max_m', 'max_translation_error_m'),
                           ('rotation_max_deg', 'max_rotation_error_deg'),
                           ('holdout_translation_max_m', 'max_translation_error_m'),
                           ('holdout_rotation_max_deg', 'max_rotation_error_deg')]:
            if metrics[key] > cfg[limit]:
                return None, metrics, 'residual too large: ' + key
        return x, metrics, 'qualified'
    except (cv2.error, ValueError, np.linalg.LinAlgError) as exc:
        return None, metrics, str(exc)


class EarlyStop:
    def __init__(self, cfg):
        self.cfg, self.previous, self.streak = cfg, None, 0

    def update(self, x):
        if x is None:
            self.previous, self.streak = None, 0
            return False
        stable = (self.previous is not None and
                  np.linalg.norm(x[:3, 3]-self.previous[:3, 3]) <= self.cfg['solution_change_m'] and
                  angle_deg(self.previous[:3, :3].T @ x[:3, :3]) <= self.cfg['solution_change_deg'])
        self.streak = self.streak + 1 if stable else 1
        self.previous = x.copy()
        return self.streak >= self.cfg['consecutive_passes']


class MarkerDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg['dictionary']))
        params = cv2.aruco.DetectorParameters()
        params.markerBorderBits = cfg['border_bits']
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        h = cfg['marker_length_m']/2
        # OpenCV IPPE_SQUARE order: top-left, top-right, bottom-right, bottom-left.
        self.points = np.array([[-h,h,0], [h,h,0], [h,-h,0], [-h,-h,0]], np.float64)

    def detect(self, image, k, dist):
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None or np.count_nonzero(ids == self.cfg['marker_id']) != 1:
            raise ValueError('Configured marker missing or duplicated')
        uv = corners[int(np.flatnonzero(ids.ravel() == self.cfg['marker_id'])[0])].reshape(4,2)
        if min(np.linalg.norm(uv-np.roll(uv, 1, axis=0), axis=1)) < self.cfg['min_side_px']:
            raise ValueError('Marker too small in image')
        margin = self.cfg['edge_margin_px']
        if (np.any(uv < margin) or np.any(uv[:,0] >= image.shape[1]-margin)
                or np.any(uv[:,1] >= image.shape[0]-margin)):
            raise ValueError('Marker too close to image edge')
        result = cv2.solvePnPGeneric(self.points, uv, k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        for rv, tv in zip(result[1], result[2]):
            r = cv2.Rodrigues(rv)[0]
            if not np.isfinite(r).all() or not np.isfinite(tv).all():
                continue
            if np.any((self.points @ r.T + tv.reshape(3))[:,2] <= 0):
                continue
            projected = cv2.projectPoints(self.points, rv, tv, k, dist)[0].reshape(4,2)
            error = float(np.sqrt(np.mean(np.sum((projected-uv)**2, axis=1))))
            if not np.isfinite(error):
                continue
            candidates.append((error, transform(r, tv)))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > self.cfg['max_reprojection_px']:
            raise ValueError('PnP reprojection error too large')
        if len(candidates)>1 and candidates[1][0]-candidates[0][0] < self.cfg['min_pnp_error_gap_px']:
            raise ValueError('Ambiguous planar PnP; use a more oblique view')
        return candidates[0][1], candidates[0][0], uv
