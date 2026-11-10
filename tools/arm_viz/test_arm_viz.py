"""离线模型检查：在 MuJoCo venv 内运行，不创建 DDS participant。"""
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from arm_viz import ArmView, dds_uri
from d1t_sdk._common import SERVO_LIMITS


class ModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.view = ArmView()

    def test_tcp_matches_sdk_across_servo_limits(self):
        rng = np.random.default_rng(7)
        limits = np.asarray(SERVO_LIMITS)
        worst = 0
        for raw in rng.uniform(limits[:, 0], limits[:, 1], size=(500, 7)):
            self.view.update(raw)
            worst = max(worst, self.view.error_mm)
        print(f"500 poses: max TCP error = {worst:.6f} mm")
        self.assertLess(worst, 1)

    def test_mesh_finger_gap(self):
        v = self.view
        for servo, expected in ((-29.6, 4), (0.3, 20), (29.2, 45), (64.2, 70)):
            v.update([20, -40, 60, 30, 20, -10, servo])
            lateral = v.data.xmat[v.model.body("Link6").id].reshape(3, 3)[:, 1]
            inner = []
            for i, name in enumerate(("Link7_1", "Link7_2")):
                geom = int(v.model.body(name).geomadr[0])
                mesh = v.model.geom_dataid[geom]
                start, count = v.model.mesh_vertadr[mesh], v.model.mesh_vertnum[mesh]
                world = (v.model.mesh_vert[start:start + count]
                         @ v.data.geom_xmat[geom].reshape(3, 3).T + v.data.geom_xpos[geom])
                projections = world @ lateral
                inner.append(projections.max() if i == 0 else projections.min())
            self.assertAlmostEqual((inner[1] - inner[0]) * 1000, expected, places=4)

    def test_reject_nonfinite_state(self):
        before = self.view.data.qpos.copy()
        for raw in ([0] * 6, [float("nan")] * 7, [float("inf")] * 7):
            with self.assertRaises(ValueError):
                self.view.update(raw)
            np.testing.assert_array_equal(before, self.view.data.qpos)

    def test_dds_uri(self):
        root = ET.fromstring(dds_uri("lo", "127.0.0.1", 6))
        self.assertEqual(root.find(".//NetworkInterface").get("name"), "lo")
        self.assertEqual(root.find(".//ParticipantIndex").text, "6")
        self.assertEqual(root.find(".//AllowMulticast").text, "false")
        self.assertEqual(root.find(".//Peer").get("address"), "127.0.0.1")


if __name__ == "__main__":
    unittest.main()
