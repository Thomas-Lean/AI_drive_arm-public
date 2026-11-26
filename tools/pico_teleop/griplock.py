# -*- coding: utf-8 -*-
"""GripLock — 扳机目标 → 实际下发的 J6 角（度，舵机角空间），纯函数式小类。

行为（暂定值，实机调）：
- 判夹住：目标比实测更闭 > CLOSE_MARGIN_DEG 且实测连续 STATIONARY_FRAMES 帧每帧变化
  < STATIONARY_DEG，且实测角 < GRASP_MAX_DEG（开到头的停住不算夹住——否则锁在
  开端、解锁条件永不满足）→ 锁在当时实测角（locked，event="grasp"），此后输出恒为
  锁定角。断流（freeze）恢复后停住帧计数清零，不跨冻结累计；
- 解锁：扳机松回到比锁定角更开 > RELEASE_MARGIN_DEG → event="release"（无振动），
  开方向不限速（控制器另有 100°/s 帧步限）；
- 空夹：实测 ≤ EMPTY_DEG 且未锁 → event="empty"（每段只发一次，实测回弹 EMPTY_CLEAR_DEG
  以上才允许下一段）；
- 闭合方向限速 CLOSE_SPEED_DPS（100°/s × dt）；首帧输出从实测角起步，不跳。

角度约定：开 = 大角（J6 全开 64.5°），闭 = 小角（全闭 −30°）；「更闭」= 数值更小。
"""
from __future__ import annotations

CLOSE_MARGIN_DEG = 2.0     # 目标比实测更闭超过此值才可能是夹到东西
STATIONARY_DEG = 0.3       # 实测每帧变化小于此值算「停住」
STATIONARY_FRAMES = 3      # 连续停住帧数
RELEASE_MARGIN_DEG = 3.0   # 扳机松到比锁定角更开此值以上才解锁
EMPTY_DEG = -28.0          # 未锁而实测到此处 = 空夹到底
EMPTY_CLEAR_DEG = 1.0      # 实测回弹此值以上，空夹段才重新武装
CLOSE_SPEED_DPS = 100.0    # 闭合方向限速（°/s；= 控制器帧步上限，全行程 94.5° ≈ 0.95 s，--grip-vel 可改）
GRASP_MAX_DEG = 62.0       # 锁角上限：实测停在此角以上视为开到头（≈67 mm），不判夹住
DEFAULT_DT_S = 0.05        # 控制周期 20 Hz


class GripLock:
    """update(j6_cmd, j6_meas) → (j6_out, event)。event ∈ {None, "grasp", "release", "empty"}。"""

    def __init__(self, dt_s: float = DEFAULT_DT_S,
                 close_margin_deg: float = CLOSE_MARGIN_DEG,
                 stationary_deg: float = STATIONARY_DEG,
                 stationary_frames: int = STATIONARY_FRAMES,
                 release_margin_deg: float = RELEASE_MARGIN_DEG,
                 empty_deg: float = EMPTY_DEG,
                 close_speed_dps: float = CLOSE_SPEED_DPS,
                 grasp_max_deg: float = GRASP_MAX_DEG):
        self.dt_s = float(dt_s)
        self.close_margin_deg = float(close_margin_deg)
        self.stationary_deg = float(stationary_deg)
        self.stationary_frames = int(stationary_frames)
        self.release_margin_deg = float(release_margin_deg)
        self.empty_deg = float(empty_deg)
        self.close_speed_dps = float(close_speed_dps)
        self.grasp_max_deg = float(grasp_max_deg)

        self.locked = False          # 夹住锁定中
        self.lock_deg: float | None = None  # 锁定的实测角
        self._out: float | None = None      # 上次输出（stale 冻结用）
        self._prev_meas: float | None = None
        self._streak = 0             # 实测连续「停住」帧数
        self._empty_latched = False  # 空夹段已发过事件
        self._frozen = False         # 上次调用是 freeze（断流/首接合前）

    # -- 内部：实测停住帧计数 ------------------------------------------------
    def _tick_stationary(self, j6_meas: float) -> None:
        if self._prev_meas is not None and abs(j6_meas - self._prev_meas) < self.stationary_deg:
            self._streak += 1
        else:
            self._streak = 0
        self._prev_meas = j6_meas

    def update(self, j6_cmd: float, j6_meas: float) -> tuple[float, str | None]:
        """喂一帧：扳机目标角与 J6 实测角（度）。返回 (实际下发角, 事件或 None)。"""
        event: str | None = None
        if self._out is None:            # 首帧：从实测起步，不跳
            self._out = float(j6_meas)
        if self._frozen:                 # 断流恢复：停住帧计数清零，不跨冻结累计
            self._frozen = False         # （冻结期间实测不动，旧计数会一恢复就误判夹住）
            self._prev_meas = None
            self._streak = 0

        self._tick_stationary(j6_meas)
        closing_harder = j6_cmd < j6_meas - self.close_margin_deg  # 目标比实测更闭 > margin

        # 夹住判定（未锁时；开到头的停住不算——锁在开端将永远无法解锁）
        if (not self.locked and closing_harder and self._streak >= self.stationary_frames
                and j6_meas < self.grasp_max_deg):
            self.locked = True
            self.lock_deg = float(j6_meas)
            event = "grasp"

        # 输出角
        if self.locked:
            if j6_cmd > self.lock_deg + self.release_margin_deg:   # 松扳机 → 解锁张开
                self.locked = False
                self.lock_deg = None
                event = "release"
                self._out = float(j6_cmd)   # 开方向不限速
            else:
                self._out = float(self.lock_deg)
        else:
            # 闭合方向限速：每帧至多 close_speed_dps × dt（目标在步长内则直接到目标）
            floor = self._out - self.close_speed_dps * self.dt_s
            self._out = float(max(j6_cmd, floor)) if j6_cmd < self._out else float(j6_cmd)

        # 空夹到底（未锁）：每段一次
        if j6_meas > self.empty_deg + EMPTY_CLEAR_DEG:
            self._empty_latched = False
        elif not self.locked and not self._empty_latched and event is None \
                and self._prev_meas is not None:
            event = "empty"
            self._empty_latched = True
        return self._out, event

    def freeze(self) -> float | None:
        """断流（STALE）/首帧接合前不喂帧：返回冻结的输出角（从未 update 过则 None），
        不产生事件、不推进限速；下次 update 时停住帧计数清零。"""
        self._frozen = True
        return self._out
