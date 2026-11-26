#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GripLock 离线测试（纯函数，喂合成 j6 实测序列，无臂无网络）。

覆盖：夹住、空夹到底、松开解锁、抖动不误判；另测闭合限速、
解锁滞回带、空夹每段一次、首帧从实测起步、freeze。角度均为 J6 舵机角（度）。

运行：python3 tools/pico_teleop/test_griplock.py -v
"""
import unittest

from griplock import CLOSE_SPEED_DPS, DEFAULT_DT_S, GripLock  # 同目录

STEP = CLOSE_SPEED_DPS * DEFAULT_DT_S   # 默认闭合每帧步长（°）


def run(g, cmds, meass):
    """逐帧喂 (cmd, meas) 序列，返回 [(out, event)]。"""
    return [g.update(c, m) for c, m in zip(cmds, meass)]


class TestGrasp(unittest.TestCase):
    def test_grasp_locks_at_stationary_measured(self):
        """实测停在 31°、目标更闭 > 2°：连续 3 帧停住后锁定在 31°，目标再深也不动。"""
        g = GripLock()
        # 预热 1 帧建立 _out/_prev_meas（首帧不计停住）
        run(g, [40.0], [40.0])
        outs = run(g, [20.0] * 6, [31.0] * 6)
        events = [e for _, e in outs]
        self.assertEqual(events[:3], [None, None, None])       # streak 1,2,3 的前三帧
        self.assertEqual(events[3], "grasp")                    # 第 4 帧触发
        self.assertTrue(g.locked and g.lock_deg == 31.0)
        self.assertEqual(outs[3][0], 31.0)
        # 扳机继续压深、实测被顶在 31°：输出恒 31°，不再有事件
        outs2 = run(g, [-30.0] * 10, [31.0] * 10)
        self.assertTrue(all(o == 31.0 and e is None for o, e in outs2))

    def test_fast_close_no_false_grasp(self):
        """空手快闭（实测每帧变 2° > 0.3°）：目标深闭也不判夹住；限速 40°/s（显式给）。"""
        g = GripLock(close_speed_dps=40.0)
        outs = []
        meas = 64.5
        for _ in range(5):
            o, e = g.update(-30.0, meas)     # 实测完全跟随（无物体）
            outs.append((o, e))
            meas = o
        self.assertTrue(all(e is None for _, e in outs), "快闭过程不得误判夹住")
        self.assertEqual([round(o, 6) for o, _ in outs],
                         [62.5, 60.5, 58.5, 56.5, 54.5])        # 每帧 −2°（40°/s × 0.05 s）

    def test_close_speed_limit_from_jump(self):
        """目标从全开一步跳到全闭：输出每帧只走 2°（40°/s 显式给），不跳变。"""
        g = GripLock(close_speed_dps=40.0)
        o1, e1 = g.update(-30.0, 64.5)
        self.assertAlmostEqual(o1, 62.5)
        self.assertIsNone(e1)
        o2, _ = g.update(-30.0, o1)
        self.assertAlmostEqual(o2, 60.5)

    def test_default_close_speed(self):
        """默认闭合限速 CLOSE_SPEED_DPS：每帧 −STEP。"""
        g = GripLock()
        o1, _ = g.update(-30.0, 64.5)
        self.assertAlmostEqual(o1, 64.5 - STEP)
        o2, _ = g.update(-30.0, o1)
        self.assertAlmostEqual(o2, 64.5 - 2 * STEP)

    def test_release_unlocks_and_relock(self):
        """锁定后扳机松开 > 锁角 + 3°：解锁（release），输出跟开；再闭可重新锁。"""
        g = GripLock()
        run(g, [40.0], [40.0])
        run(g, [20.0] * 4, [31.0] * 4)                          # 锁在 31°
        self.assertTrue(g.locked)
        outs = run(g, [33.0] * 3, [31.0] * 3)                   # 滞回带内（33 ≤ 31+3）
        self.assertTrue(all(o == 31.0 and e is None for o, e in outs))
        o, e = g.update(40.0, 31.0)                             # 40 > 34 → 解锁
        self.assertEqual((o, e), (40.0, "release"))
        self.assertFalse(g.locked)
        run(g, [40.0] * 3, [40.0] * 3)                          # 实测跟到 40° 并停住（累计 3 帧停住）
        _, e = g.update(20.0, 40.0)                             # 又深闭：重新锁在 40°
        self.assertEqual(e, "grasp")
        self.assertEqual(g.lock_deg, 40.0)


class TestEmpty(unittest.TestCase):
    def close_to_bottom(self, g):
        """空手把扳机压到底（实测完全跟随），走到实测 ≤ −28°；返回逐帧结果。"""
        outs = []
        meas = 64.5
        while True:
            o, e = g.update(-30.0, meas)
            outs.append((o, e))
            if meas <= -30.0:            # 到底那一帧的实测也喂进去后再停
                return outs
            meas = o

    def test_empty_fires_once_per_episode(self):
        """空夹到底：实测 ≤ −28° 发一次 empty；停在最底不再发；回弹后再夹再发。"""
        g = GripLock()
        outs = self.close_to_bottom(g)
        self.assertEqual([e for _, e in outs].count("empty"), 1)
        self.assertEqual(outs[-1][0], -30.0)
        pinned = run(g, [-30.0] * 10, [-30.0] * 10)             # 停在最底
        self.assertTrue(all(e is None for _, e in pinned))
        run(g, [64.5] * 5, [0.0] * 5)                           # 张开清段（实测 > −27°）
        outs2 = self.close_to_bottom(g)
        self.assertEqual([e for _, e in outs2].count("empty"), 1)

    def test_empty_not_fired_when_locked(self):
        """锁定中的实测不会触发 empty（未锁条件）。"""
        g = GripLock()
        run(g, [40.0], [40.0])
        run(g, [20.0] * 4, [31.0] * 4)                          # 锁在 31°
        outs = run(g, [-30.0] * 10, [31.0] * 10)
        self.assertTrue(all(e is None for _, e in outs))


class TestMisc(unittest.TestCase):
    def test_no_grasp_at_open_stop(self):
        """实测停在开端 64.2°（开到头）、目标更闭：不判夹住——否则锁在开端，
        解锁条件（锁角+3° > 64.5 限位）永不满足（全链路 A 实测出过，已修）。"""
        g = GripLock()
        run(g, [40.0], [40.0])
        outs = run(g, [40.0] * 6, [64.2] * 6)
        self.assertTrue(all(e is None for _, e in outs))
        self.assertFalse(g.locked)

    def test_streak_resets_across_freeze(self):
        """断流恢复后停住帧计数清零：恢复后前 3 帧不判夹住，第 4 帧才可能。"""
        g = GripLock()
        run(g, [40.0], [40.0])
        run(g, [20.0] * 3, [31.0] * 3)          # streak 到 2（差一帧就够）
        g.freeze()
        outs = [g.update(20.0, 31.0) for _ in range(4)]
        self.assertEqual([e for _, e in outs], [None, None, None, "grasp"])

    def test_jitter_no_false_grasp(self):
        """实测抖动 ±0.25°（每帧变化 0.5° > 0.3°）：目标深闭也不判夹住。"""
        g = GripLock()
        run(g, [40.0], [40.0])
        events = []
        for k in range(40):
            meas = 31.0 + (0.25 if k % 2 == 0 else -0.25)
            _, e = g.update(20.0, meas)
            events.append(e)
        self.assertNotIn("grasp", events)
        self.assertNotIn("empty", events)                       # 31° 远离 −28°
        self.assertFalse(g.locked)

    def test_slow_drift_counts_as_stationary(self):
        """慢漂移（每帧 0.2° < 0.3°）算停住：3 帧后照样锁（按定义，非误判）。"""
        g = GripLock()
        run(g, [40.0], [40.0])
        meas = 31.0
        events = []
        for _ in range(4):
            _, e = g.update(20.0, meas)
            events.append(e)
            meas -= 0.2
        self.assertIn("grasp", events)
        self.assertAlmostEqual(g.lock_deg, 30.4)                # 锁在触发帧（第 4 帧）的实测角

    def test_first_frame_output_starts_at_measured(self):
        """首帧输出以实测为起点：开方向直接跟目标，闭方向只走一帧限速步长。"""
        g = GripLock()
        self.assertEqual(g.update(60.0, 10.0)[0], 60.0)         # 开不限速
        g2 = GripLock()
        self.assertAlmostEqual(g2.update(-30.0, 10.0)[0], 10.0 - STEP)  # 闭每帧一个步长

    def test_freeze_returns_last_output(self):
        """freeze：断流期间返回冻结输出，状态不变。"""
        g = GripLock()
        run(g, [40.0], [40.0])
        run(g, [20.0] * 4, [31.0] * 4)
        before = (g.locked, g.lock_deg)
        self.assertEqual(g.freeze(), 31.0)
        self.assertEqual((g.locked, g.lock_deg), before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
