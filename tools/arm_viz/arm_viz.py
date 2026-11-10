#!/usr/bin/env python3
"""D1-T ArmState 只读运动学显示；不创建命令 writer，不执行动力学步进。"""
import argparse
from contextlib import contextmanager
import os
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sdk"))


def dds_uri(iface, peer, index):
    root = ET.Element("CycloneDDS")
    domain = ET.SubElement(root, "Domain", Id="0")
    general = ET.SubElement(domain, "General")
    interfaces = ET.SubElement(general, "Interfaces")
    ET.SubElement(interfaces, "NetworkInterface", name=iface, autodetermine="false")
    ET.SubElement(general, "AllowMulticast").text = "false"
    discovery = ET.SubElement(domain, "Discovery")
    ET.SubElement(discovery, "ParticipantIndex").text = str(index)
    peers = ET.SubElement(discovery, "Peers")
    ET.SubElement(peers, "Peer", address=peer)
    return ET.tostring(root, encoding="unicode")


def add_scene(mujoco, spec):
    """浅色渐变背景 + 地面网格（base_link 所在平面，每格 10 cm），只影响显示。"""
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.92, 0.94, 0.97], rgb2=[0.75, 0.80, 0.86], width=256, height=256)
    spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, mark=mujoco.mjtMark.mjMARK_EDGE,
                     rgb1=[0.85, 0.85, 0.85], rgb2=[0.85, 0.85, 0.85], markrgb=[0.55, 0.55, 0.55],
                     width=200, height=200)
    material = spec.add_material(name="grid", texrepeat=[5, 5], texuniform=True, reflectance=0)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01],
                            material="grid", contype=0, conaffinity=0)


class ArmView:
    """URDF 模型和状态映射；update 输入七个舵机角（度）。"""

    def __init__(self):
        import mujoco
        import numpy as np
        from d1t_sdk._common import ARM_MODEL
        from d1t_sdk._kinematics import Kinematics, _xyzrpy

        self.mj, self.np = mujoco, np
        self.fk = Kinematics()
        model_dir = ROOT / "sdk/arm_model/d1_description"
        xml = (model_dir / "urdf/d1_description.urdf").read_text()
        xml = xml.replace("package://d1_description/", str(model_dir) + "/")
        root = ET.fromstring(xml)
        # 只保留外观，避免 visual / collision 两层重叠；原 URDF 不落盘改写。
        for link in root.findall("link"):
            for collision in link.findall("collision"):
                link.remove(collision)
        extension = ET.SubElement(root, "mujoco")
        ET.SubElement(extension, "compiler", discardvisual="false", fusestatic="false",
                      strippath="false")
        spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"))
        tcp = ARM_MODEL["tcp"]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, _xyzrpy(0, 0, 0, *tcp["rpy"])[:3, :3].flatten())
        spec.body("Link6").add_site(name="TCP", pos=tcp["xyz"], quat=quat,
                                    size=[0.004, 0, 0], rgba=[1, 0.8, 0, 1])
        spec.body("base_link").add_site(name="base_link", pos=[0, 0, 0],
                                        size=[0.004, 0, 0], rgba=[1, 1, 1, 1])
        add_scene(mujoco, spec)
        self.model = spec.compile()
        self.model.vis.headlight.ambient = [0.4, 0.4, 0.4]
        self.model.vis.headlight.diffuse = [0.6, 0.6, 0.6]
        self.model.vis.scale.framelength = 0.14 / self.model.stat.meansize
        self.model.vis.scale.framewidth = 0.003 / self.model.stat.meansize
        self.data = mujoco.MjData(self.model)
        self.arm_adr = [self.model.joint(f"Joint{i+1}").qposadr[0] for i in range(6)]
        self.finger_adr = [self.model.joint(name).qposadr[0]
                           for name in ("Joint7_1", "Joint7_2")]
        self.tcp_id = self.model.site("TCP").id
        self.base_id = self.model.site("base_link").id
        # 用实际 mesh 内侧表面定义开度，不能把两指 joint 原点距离当指间距。
        mujoco.mj_forward(self.model, self.data)
        lateral = self.data.xmat[self.model.body("Link6").id].reshape(3, 3)[:, 1]
        inner, self.finger_axis = [], []
        for i, name in enumerate(("Link7_1", "Link7_2")):
            body = self.model.body(name)
            geom_id = int(body.geomadr[0])
            mesh_id = self.model.geom_dataid[geom_id]
            adr, count = self.model.mesh_vertadr[mesh_id], self.model.mesh_vertnum[mesh_id]
            vertices = self.model.mesh_vert[adr:adr + count]
            rotation = self.data.geom_xmat[geom_id].reshape(3, 3)
            world = vertices @ rotation.T + self.data.geom_xpos[geom_id]
            projection = world @ lateral
            inner.append(float(projection.max() if i == 0 else projection.min()))
            joint_id = self.model.joint(("Joint7_1", "Joint7_2")[i]).id
            self.finger_axis.append(float(self.data.xaxis[joint_id] @ lateral))
        self.finger_zero_gap = inner[1] - inner[0]
        self.q_deg = np.zeros(6)
        self.gap_mm = 0.0
        self.error_mm = 0.0
        mujoco.mj_forward(self.model, self.data)

    def update(self, servo_deg):
        from d1t_sdk._joint_model import servo_to_joint, gripper_deg_to_mm

        np = self.np
        raw = np.asarray(servo_deg, dtype=float)
        if raw.shape != (7,) or not np.isfinite(raw).all():
            raise ValueError("ArmState.q 必须是七个有限舵机角")
        self.q_deg = np.array([servo_to_joint(i, raw[i]) for i in range(6)])
        q = np.deg2rad(self.q_deg)
        self.data.qpos[self.arm_adr] = q
        self.gap_mm = gripper_deg_to_mm(raw[6])
        opening = (self.gap_mm / 1000 - self.finger_zero_gap) / 2
        self.data.qpos[self.finger_adr] = [-opening / self.finger_axis[0],
                                          opening / self.finger_axis[1]]
        self.mj.mj_forward(self.model, self.data)
        self.error_mm = float(np.linalg.norm(self.data.site_xpos[self.tcp_id]
                                             - self.fk.fk(q)[:3, 3]) * 1000)



@contextmanager
def passive_viewer(model, data):
    import mujoco.viewer

    # Linux passive viewer 的 close 只提交退出请求；等渲染线程释放 GL
    # 后再退出 Python，避免 glfw.terminate 与渲染线程争用导致段错误。
    before = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data)
    render_threads = set(threading.enumerate()) - before
    try:
        yield viewer
    finally:
        viewer.close()
        for thread in render_threads:
            thread.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iface", default="lo", help="DDS 网卡，默认 lo")
    parser.add_argument("--peer", default="127.0.0.1", help="DDS 单播对端")
    parser.add_argument("--index", type=int, default=6, help="独占 ParticipantIndex，默认 6")
    args = parser.parse_args()
    if not 0 <= args.index <= 119:
        parser.error("--index 必须在 0..119 内")
    # 必须在导入 DDS / SDK 前设置；覆盖终端可能遗留的真臂 URI。
    os.environ["CYCLONEDDS_URI"] = dds_uri(args.iface, args.peer, args.index)
    import mujoco
    import mujoco.viewer
    from cyclonedds.sub import DataReader
    from d1t_sdk._common import CTRL_STATE_NAMES, dds_qos, make_participant, make_state_topic

    view = ArmView()
    participant = make_participant()
    reader = DataReader(participant, make_state_topic(participant), dds_qos())
    print(f"只读 ArmState：iface={args.iface} peer={args.peer} index={args.index}", flush=True)
    last_state, received_at, printed_at = None, None, 0.0
    with passive_viewer(view.model, view.data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.22]
        viewer.cam.distance = 1.1
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_SITE
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        while viewer.is_running():
            now = time.monotonic()
            samples = reader.take(32)
            with viewer.lock():
                for sample in samples:
                    if hasattr(sample, "q"):
                        view.update(sample.q)
                        last_state, received_at = sample, now
            viewer.sync()
            if now - printed_at >= 1:
                printed_at = now
                if last_state is None:
                    print("WAITING：尚未收到 ArmState（窗口初始姿态不是反馈）", flush=True)
                else:
                    age = now - received_at
                    status = "STALE" if age > 1 else "LIVE"
                    mode = CTRL_STATE_NAMES.get(last_state.ctrl_state, "UNKNOWN")
                    tcp_mm = view.data.site_xpos[view.tcp_id] * 1000
                    print(f"{status} age={age:.2f}s mode={mode} ctrl_state={last_state.ctrl_state} "
                          f"q_deg={view.q_deg.round(2).tolist()} "
                          f"gripper={view.gap_mm:.2f}mm TCP_mm={tcp_mm.round(2).tolist()} "
                          f"FK_error={view.error_mm:.4f}mm valid=0x{last_state.valid_mask:02x}"
                          + (" MODEL_MISMATCH" if view.error_mm > 1 else ""), flush=True)
            time.sleep(1 / 60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
