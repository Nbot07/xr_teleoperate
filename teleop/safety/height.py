"""Hands-only stand-height ("waist height") control for Quest 3 hand tracking.

Interaction spec (see docs/SAFETY_FRAMEWORK.md §6):
  * LEFT FIST held 0.8 s (or key 'h')  -> toggle HEIGHT MODE. Arms freeze.
  * In height mode:
      - ONE hand pinched: vertical wrist drag nudges height
        (dh = pinch_gain * dz, re-anchor on each pinch — like scrolling).
      - BOTH hands pinched: robot height follows headset dz (embodied squat).
  * Auto-exit on 10 s idle or 2 s tracking staleness; left fist exits too.
  * All targets are clamped/slewed and battery-gated by the supervisor.

The controller never talks to the SDK; it only calls supervisor.nudge_height /
request_height_target, which enforce gates, ceiling, and slew.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from .config import SafetyConfig
from .gestures import HandGestureTracker
from .supervisor import SafetySupervisor


@dataclass
class HeightDirectives:
    height_mode: bool
    arms_frozen: bool
    status: str


class HeightGestureController:
    def __init__(self, cfg: SafetyConfig, supervisor: SafetySupervisor):
        self.cfg = cfg
        self.sup = supervisor
        g = cfg.gesture
        self.left = HandGestureTracker(g.fist_hold_s, g.tracking_stale_exit_s)
        self.right = HandGestureTracker(g.fist_hold_s, g.tracking_stale_exit_s)
        self._mode = False
        self._t_activity = 0.0
        self._anchor_z: float | None = None       # pinched-wrist anchor
        self._anchor_h: float | None = None
        self._head_anchor_z: float | None = None  # both-pinch head anchor
        self._pending_toggle = False              # keyboard fallback

    # ------------------------------------------------------------- interface
    def request_toggle(self) -> None:
        """Keyboard 'h' fallback."""
        self._pending_toggle = True

    def update(self, tele_data, suppress_toggle: bool = False) -> HeightDirectives:
        now = time.monotonic()
        g = self.cfg.gesture
        lh = getattr(tele_data, "left_hand_pos", None)
        rh = getattr(tele_data, "right_hand_pos", None)
        self.left.update(lh, now)
        self.right.update(rh, now)

        # left-fist toggle only counts with the RIGHT hand open — both fists
        # is the pause/safe-stop deadman (handled by the shim), never height.
        fist_toggle = self.left.fist_held_toggle(now) and not self.right.fist
        toggle = self._pending_toggle or (fist_toggle and not suppress_toggle)
        self._pending_toggle = False
        if toggle:
            self._set_mode(not self._mode, now)
        if self._mode and self.left.fist and self.right.fist:
            # deadman engaged while in height mode: bail out immediately
            self._set_mode(False, now)
            return HeightDirectives(False, True, "height mode exit: both-fist hold")

        if not self._mode:
            return HeightDirectives(False, False, "")

        # (mode active from here)

        # --- inside height mode -------------------------------------------
        if not self.sup.height_available():
            self._set_mode(False, now)
            return HeightDirectives(False, True,
                                    "height unavailable (no stand-height readback)")
        if self.left.stale(now) and self.right.stale(now):
            if now - self._t_activity > g.tracking_stale_exit_s:
                self._set_mode(False, now)
                return HeightDirectives(False, True, "height mode exit: tracking lost")
            return HeightDirectives(True, True, "height: tracking…")
        if now - self._t_activity > g.idle_exit_s:
            self._set_mode(False, now)
            return HeightDirectives(False, True, "height mode exit: idle")

        l_pinch = bool(getattr(tele_data, "left_hand_pinch", False))
        r_pinch = bool(getattr(tele_data, "right_hand_pinch", False))
        head_z = self._pose_z(getattr(tele_data, "head_pose", None))
        wrist_z = None
        if r_pinch:
            wrist_z = self._pose_z(getattr(tele_data, "right_wrist_pose", None))
        elif l_pinch:
            wrist_z = self._pose_z(getattr(tele_data, "left_wrist_pose", None))

        status = f"HEIGHT {self._h():.2f} m  [{self._gauge()}]"
        if l_pinch and r_pinch and head_z is not None:
            # embodied head-follow
            if self._head_anchor_z is None:
                self._head_anchor_z = head_z
                self._anchor_h = self._h()
            dz = head_z - self._head_anchor_z
            if abs(dz) > g.head_deadband_m:
                target = self._anchor_h + g.head_follow_gain * dz
                self.sup.request_height_target(target, from_user=True)
                self._t_activity = now
            self._anchor_z = None
            status += "  ⇕ head-follow"
        elif wrist_z is not None:
            # pinch elevator (relative, re-anchor each pinch)
            self._head_anchor_z = None
            if self._anchor_z is None:
                self._anchor_z = wrist_z
                self._anchor_h = self._h()
            target = self._anchor_h + g.pinch_gain * (wrist_z - self._anchor_z)
            self.sup.request_height_target(target, from_user=True)
            self._t_activity = now
            status += "  ✊ pinch-drag"
        else:
            self._anchor_z = None
            self._head_anchor_z = None
            status += "  (pinch to adjust, both pinch = follow, left fist = exit)"

        return HeightDirectives(True, True, status)

    # --------------------------------------------------------------- helpers
    def _set_mode(self, on: bool, now: float) -> None:
        if on == self._mode:
            return
        self._mode = on
        self._t_activity = now
        self._anchor_z = None
        self._head_anchor_z = None
        self.sup.notifier.status("HEIGHT MODE ON — arms frozen" if on
                                 else "height mode off — arms stay frozen until restart gesture")

    def _h(self) -> float:
        s = self.sup.snapshot()
        return (self.sup._h_cmd if self.sup._h_cmd is not None
                else (s.stand_height or 0.0))

    def _gauge(self, width: int = 12) -> str:
        lo, hi = self.sup.h_min(), self.sup.h_max()
        span = max(1e-6, hi - lo)
        k = int(round((self._h() - lo) / span * (width - 1)))
        k = max(0, min(width - 1, k))
        return "▁" * k + "█" + "▁" * (width - 1 - k)

    @staticmethod
    def _pose_z(pose_4x4) -> float | None:
        try:
            return float(pose_4x4[2, 3])
        except Exception:  # noqa: BLE001
            return None
