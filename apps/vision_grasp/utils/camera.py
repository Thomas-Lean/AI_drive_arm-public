"""RealSense 彩色流封装：内参/畸变读取、预热、带元数据的取帧。

pyrealsense2 延迟导入（离线求解与无设备测试不依赖设备 SDK）。
畸变策略：只接受 none / 系数全零 / Brown-Conrady；非零 modified/inverse
Brown 在运动前直接报错，绝不当作 Brown 系数静默使用。
"""
from datetime import datetime, timezone
import time

import numpy as np


class Camera:
    def __init__(self, cfg):
        import pyrealsense2 as rs  # 延迟导入
        self.rs, self.cfg = rs, cfg
        self.pipeline = rs.pipeline()
        config = rs.config()
        if cfg['serial']:
            config.enable_device(cfg['serial'])
        config.enable_stream(rs.stream.color, cfg['width'], cfg['height'], rs.format.bgr8, cfg['fps'])
        self.started = False
        try:
            profile = self.pipeline.start(config)
            self.started = True
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.k = np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
            coeff = np.array(intr.coeffs, dtype=float)
            if intr.model == rs.distortion.none or np.allclose(coeff, 0, atol=1e-12):
                self.dist = np.zeros(5)
            elif intr.model == rs.distortion.brown_conrady:
                self.dist = coeff
            else:
                raise ValueError(f'Unsupported nonzero distortion model: {intr.model}; rectify/calibrate color first')
            device = profile.get_device()
            self.metadata = dict(serial=device.get_info(rs.camera_info.serial_number),
                                 model=device.get_info(rs.camera_info.name),
                                 distortion_model=str(intr.model), raw_coefficients=coeff.tolist(),
                                 width=intr.width, height=intr.height, fps=cfg['fps'])
            for _ in range(cfg['warmup_frames']):  # 预热：曝光稳定 + 丢弃过渡帧
                self.frame()
        except BaseException:
            self.close()
            raise

    def frame(self):
        """取一帧彩色图；返回 (BGR 图像, 帧元数据)。设备时间与主机时间
        不共时钟，只分别记录、不做差。"""
        f = self.pipeline.wait_for_frames(self.cfg['frame_timeout_ms']).get_color_frame()
        if not f:
            raise RuntimeError('No RealSense color frame')
        return (np.asanyarray(f.get_data()).copy(),
                dict(device_timestamp_ms=f.get_timestamp(), timestamp_domain=str(f.get_frame_timestamp_domain()),
                     frame_number=f.get_frame_number(), host_monotonic_s=time.monotonic(),
                     host_utc=datetime.now(timezone.utc).isoformat()))

    def close(self):
        if self.started:
            self.pipeline.stop()
            self.started = False
