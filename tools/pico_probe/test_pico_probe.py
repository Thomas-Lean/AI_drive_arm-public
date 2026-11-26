#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pico_probe 离线测试（无 PICO、无 TeleRobot 查看器、不连臂）。

FakeXrPublisher 起真实 ZMQ PUB（随机端口）→ TeleRobot WebXRController 订阅 →
RelativePoseMapper → PicoProbe，走与接 PICO 时完全相同的代码路径。覆盖：扳机→J6 端点、
接合不跳变、相对映射方向与比例、脱开目标保持、滞回、再接合重捕参考、epoch 变化须先松握把、
断流 STALE、非新鲜帧被忽略、接收频率统计、状态行字段。

运行（需 numpy/scipy/pyzmq/cyclonedds 与 TeleRobot 目录；TeleRobot 不在 ~/TeleRobot 时设
TELEROBOT_DIR）：
    python3 tools/pico_probe/test_pico_probe.py -v
"""
import sys
import time
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fake_xr_pub  # noqa: E402
import pico_probe  # noqa: E402

P0 = (0.30, 1.20, -0.30)  # 手柄起点（XR 系，米）
FRAME_S = 0.02            # 测试里的发帧间隔（50 Hz）


class ProbeRig:
    """端到端装置：假发布器 + 真实订阅/映射链。"""

    def __init__(self, side="right"):
        self.mods = pico_probe.load_modules(pico_probe.DEFAULT_TELEROBOT_DIR,
                                            pico_probe.DEFAULT_SDK_DIR)
        self.ee_pos, self.ee_rot = pico_probe.fake_tcp_pose(pico_probe.DEFAULT_HOME_Q, self.mods)
        self.pub = fake_xr_pub.FakeXrPublisher("tcp://127.0.0.1:0", side=side)
        mapper = self.mods["RelativePoseMapper"](side=side,
                                                 grip_press_threshold=pico_probe.GRIP_PRESS,
                                                 grip_release_threshold=pico_probe.GRIP_RELEASE,
                                                 stale_timeout_s=pico_probe.MAPPER_STALE_S)
        self.probe = pico_probe.PicoProbe(mapper, self.ee_pos, self.ee_rot)
        config = self.mods["WebXRConfig"](hand_side=side, stale_timeout=pico_probe.CTRL_STALE_S,
                                          endpoint=self.pub.endpoint)
        self.controller = self.probe.attach(
            self.mods["WebXRController"](config, on_tracking=self.probe.on_tracking))
        self.controller.connect()
        self._wait_subscribed()

    def _wait_subscribed(self):
        """PUB/SUB 慢加入：确认订阅端真收到过帧，后面的断言才可靠。"""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.pub.send(P0, 0.0, 0.0)
            time.sleep(FRAME_S)
            if self.controller.latest_sample() is not None:
                return
        raise AssertionError("3 s 内订阅端没收到帧")

    def frame(self, position=P0, grip=0.0, trigger=0.0, fresh=True):
        """发一帧、等一个帧间隔、步进探针一次。返回 status。"""
        self.pub.send(position, grip, trigger, fresh=fresh)
        time.sleep(FRAME_S)
        return self.probe.step()[0]

    def until(self, predicate, timeout=2.0, **frame_kw):
        """持续发同一帧直到 predicate(status) 为真；超时抛 AssertionError。"""
        status = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.frame(**frame_kw)
            if predicate(status):
                return status
        raise AssertionError(f"{timeout} s 内条件未满足：state={status.get('state')} "
                             f"live={status.get('live')} rx={status.get('rx_hz')}")

    def hold(self, duration, **frame_kw):
        """持续发同一帧 duration 秒，返回全程每一步的 status。"""
        out = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            out.append(self.frame(**frame_kw))
        return out

    def engage(self, position=P0):
        """先松（清 require_release）再握到 0.95，返回进入 ACTIVE 那一帧的 status。"""
        self.until(lambda s: s["state"].value == "idle" and not s["require_release"],
                   position=position, grip=0.0)
        return self.until(lambda s: s["state"].value == "active", position=position, grip=0.95)

    def close(self):
        self.controller.disconnect()
        self.pub.close()


def state_of(status):
    return status["state"].value


class TestTriggerMapping(unittest.TestCase):
    def test_endpoints_and_linearity(self):
        """扳机 0→64.5°（全开）、1→−30°（全闭）、中间线性；越界截断。"""
        self.assertAlmostEqual(pico_probe.trigger_to_j6(0.0), 64.5, places=9)
        self.assertAlmostEqual(pico_probe.trigger_to_j6(1.0), -30.0, places=9)
        self.assertAlmostEqual(pico_probe.trigger_to_j6(0.5), 17.25, places=9)
        self.assertAlmostEqual(pico_probe.trigger_to_j6(0.2), 64.5 - 94.5 * 0.2, places=9)
        self.assertAlmostEqual(pico_probe.trigger_to_j6(-0.2), 64.5, places=9)
        self.assertAlmostEqual(pico_probe.trigger_to_j6(1.3), -30.0, places=9)


class TestProbeChain(unittest.TestCase):
    def setUp(self):
        self.rig = ProbeRig()

    def tearDown(self):
        self.rig.close()

    def test_engage_target_equals_tcp_no_jump(self):
        """握把过 0.85：进入 ACTIVE 那一帧捕获参考，目标 = 当前 TCP（位置、姿态都不跳）。"""
        st = self.rig.engage()
        self.assertTrue(st["reference_captured"])
        np.testing.assert_allclose(st["target"].position, self.rig.ee_pos, atol=1e-9)
        np.testing.assert_allclose(st["target"].rotation, self.rig.ee_rot, atol=1e-9)
        for st in self.rig.hold(0.2, grip=0.95):  # 手柄不动：目标一直等于 TCP
            self.assertEqual(state_of(st), "active")
            np.testing.assert_allclose(st["target"].position, self.rig.ee_pos, atol=1e-9)

    def test_relative_mapping_direction_and_scale(self):
        """手柄 XR 位移 Δ → 目标 = TCP + S·Δ（S = DEFAULT_XR_TO_WORLD），比例 1：
        XR 右 10 cm → 基座 −Y 10 cm；XR 上 → +Z；XR 前（−Z）→ +X。"""
        from teleop.vendor.pose_mapping import DEFAULT_XR_TO_WORLD
        self.rig.engage()
        for dxr, want_dir in (((0.10, 0.0, 0.0), (0.0, -0.10, 0.0)),
                              ((0.0, 0.10, 0.0), (0.0, 0.0, 0.10)),
                              ((0.0, 0.0, -0.10), (0.10, 0.0, 0.0))):
            p1 = tuple(P0[i] + dxr[i] for i in range(3))
            st = self.rig.until(lambda s: s["target"] is not None and np.linalg.norm(
                s["target"].position - self.rig.ee_pos) > 0.05, position=p1, grip=0.95)
            delta = st["target"].position - self.rig.ee_pos
            np.testing.assert_allclose(delta, DEFAULT_XR_TO_WORLD @ np.array(dxr), atol=1e-9)
            np.testing.assert_allclose(delta, want_dir, atol=1e-9)
            self.rig.until(lambda s: s["target"] is not None and np.linalg.norm(
                s["target"].position - self.rig.ee_pos) < 1e-9, position=P0, grip=0.95)

    def test_release_target_holds_then_reengage_recaptures(self):
        """握把 <0.75：脱开，手柄怎么动都没有新目标（保持不动）；再握：重捕参考、不跳。"""
        self.rig.engage()
        st = self.rig.until(lambda s: state_of(s) == "idle", grip=0.5)
        self.assertIsNone(st["target"])
        p_far = (P0[0] + 0.20, P0[1] - 0.10, P0[2] + 0.05)
        for st in self.rig.hold(0.3, position=p_far, grip=0.5):
            self.assertEqual(state_of(st), "idle")
            self.assertIsNone(st["target"])
        st = self.rig.until(lambda s: state_of(s) == "active", position=p_far, grip=0.95)
        self.assertTrue(st["reference_captured"])
        np.testing.assert_allclose(st["target"].position, self.rig.ee_pos, atol=1e-9)

    def test_grip_hysteresis_band(self):
        """0.75 < 握把 < 0.85：脱开时不接合；接合时不脱开。"""
        self.rig.until(lambda s: state_of(s) == "idle" and not s["require_release"], grip=0.0)
        for st in self.rig.hold(0.3, grip=0.80):
            self.assertEqual(state_of(st), "idle")
        self.rig.until(lambda s: state_of(s) == "active", grip=0.95)
        for st in self.rig.hold(0.3, grip=0.80):
            self.assertEqual(state_of(st), "active")

    def test_epoch_change_requires_release(self):
        """换 epoch（=查看器重启）：清参考；握把一直 >0.85 也不自动接合，须先松开再握。"""
        self.rig.engage()
        self.rig.pub.new_epoch()
        st = self.rig.until(lambda s: state_of(s) == "idle", grip=0.95)
        self.assertTrue(st["require_release"])
        self.assertIsNone(st["target"])
        for st in self.rig.hold(0.4, grip=0.95):
            self.assertEqual(state_of(st), "idle")
            self.assertIsNone(st["target"])
        self.rig.until(lambda s: not s["require_release"], grip=0.0)
        st = self.rig.until(lambda s: state_of(s) == "active", grip=0.95)
        self.assertTrue(st["reference_captured"])

    def test_stale_when_stream_stops_then_requires_release(self):
        """停发：0.2 s 后 STALE（LIVE→STALE）、0.3 s 后映射器断流清参考；恢复后须先松握把。"""
        self.rig.engage()
        t_stop = time.monotonic()
        t_not_live = t_stale = None
        while time.monotonic() - t_stop < 1.0 and t_stale is None:
            st = self.rig.probe.step()[0]
            now = time.monotonic()
            if t_not_live is None and not st["live"]:
                t_not_live = now - t_stop
            if state_of(st) == "stale":
                t_stale = now - t_stop
                self.assertIsNone(st["target"])
            time.sleep(0.005)
        self.assertIsNotNone(t_not_live, "停发后没变 STALE")
        self.assertIsNotNone(t_stale, "停发后映射器没进 stale")
        self.assertGreaterEqual(t_not_live, pico_probe.CTRL_STALE_S - 0.03)
        self.assertGreaterEqual(t_stale, pico_probe.MAPPER_STALE_S - 0.03)
        st = self.rig.until(lambda s: s["live"], grip=0.95)       # 恢复发送、握把仍握着
        for st in self.rig.hold(0.3, grip=0.95):
            self.assertEqual(state_of(st), "idle")
            self.assertTrue(st["require_release"])

    def test_nonfresh_frames_ignored(self):
        """fresh=False 的帧（查看器判输入过期）不更新样本：照样断流 STALE。"""
        self.rig.engage()
        st = self.rig.until(lambda s: not s["live"] and state_of(s) == "stale",
                            grip=0.95, fresh=False)
        self.assertIsNone(st["target"])

    def test_rx_rate_and_j6_follow_trigger(self):
        """接收频率按手柄有效样本计（约等于发帧率）；j6 目标随扳机到两个端点。"""
        self.rig.engage()
        st = self.rig.until(lambda s: abs(s["j6_deg"] + 30.0) < 1e-9, grip=0.95, trigger=1.0)
        rates = [s["rx_hz"] for s in self.rig.hold(1.2, grip=0.95, trigger=1.0)]
        self.assertGreater(rates[-1], 20.0, f"接收频率 {rates[-1]:.1f} Hz")
        self.assertLess(rates[-1], 1.0 / FRAME_S + 5.0)
        st = self.rig.until(lambda s: abs(s["j6_deg"] - 64.5) < 1e-9, grip=0.95, trigger=0.0)
        self.assertEqual(state_of(st), "active")  # 扳机与离合互不影响

    def test_format_line_fields(self):
        """状态行含：接收频率、LIVE/STALE、离合状态、目标 TCP（mm）、j6。"""
        self.rig.engage()
        st = self.rig.frame(grip=0.95, trigger=0.5)
        line = pico_probe.format_line(st)
        for kw in ("Hz", "LIVE", "接合", "mm", "j6", "+17.2"):
            self.assertIn(kw, line)
        self.assertIn(f"{self.rig.ee_pos[0] * 1000:+7.1f}", line)
        st = self.rig.until(lambda s: state_of(s) == "idle", grip=0.0)
        self.assertIn("保持不动", pico_probe.format_line(st))


if __name__ == "__main__":
    unittest.main(verbosity=2)
