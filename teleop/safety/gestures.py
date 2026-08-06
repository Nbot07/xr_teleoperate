"""Gesture primitives over televuer's 25-point hand landmarks.

Landmark convention (WebXR / Vuer): 0 wrist; thumb 1-4; index 5-9; middle
10-14; ring 15-19; pinky 20-24 (tips at 4, 9, 14, 19, 24). Pinch is NOT
computed here — televuer already provides `left/right_hand_pinch` booleans and
distances; we add the fist (mode toggle) and a staleness detector, since Quest
hand tracking silently repeats the last pose when hands leave the cameras' FOV.
"""
from __future__ import annotations

import time

import numpy as np

WRIST = 0
MIDDLE_PROXIMAL = 10
FINGERTIPS = (9, 14, 19, 24)   # index/middle/ring/pinky tips (thumb excluded)

_CLOSE_ON = 1.15    # fist engaged below this curl ratio
_CLOSE_OFF = 1.45   # fist released above this (hysteresis)


class HandGestureTracker:
    """Per-hand fist detection with hold timing, plus staleness detection."""

    def __init__(self, fist_hold_s: float, stale_s: float):
        self._hold_s = fist_hold_s
        self._stale_s = stale_s
        self._fist = False
        self._fist_since: float | None = None
        self._fired = False
        self._last_pos: np.ndarray | None = None
        self._last_change_t = 0.0

    # ---------------------------------------------------------------- update
    def update(self, hand_pos: np.ndarray | None, now: float | None = None) -> None:
        now = now if now is not None else time.monotonic()
        if hand_pos is None:
            self._fist = False
            self._fist_since = None
            return
        p = np.asarray(hand_pos).reshape(25, 3)
        if self._last_pos is None or not np.allclose(p, self._last_pos, atol=1e-6):
            self._last_change_t = now
            self._last_pos = p.copy()

        scale = float(np.linalg.norm(p[MIDDLE_PROXIMAL] - p[WRIST]))
        if scale < 1e-4:
            return
        curl = float(np.mean([np.linalg.norm(p[i] - p[WRIST]) for i in FINGERTIPS])) / scale
        if self._fist:
            if curl > _CLOSE_OFF:
                self._fist = False
                self._fist_since = None
                self._fired = False
        else:
            if curl < _CLOSE_ON:
                self._fist = True
                self._fist_since = now
                self._fired = False

    # --------------------------------------------------------------- queries
    def stale(self, now: float | None = None) -> bool:
        now = now if now is not None else time.monotonic()
        return self._last_pos is None or (now - self._last_change_t) > self._stale_s

    @property
    def fist(self) -> bool:
        return self._fist

    def fist_duration(self, now: float | None = None) -> float:
        """Seconds the current fist has been held (0.0 if not a fist)."""
        now = now if now is not None else time.monotonic()
        if self._fist and self._fist_since is not None and not self.stale(now):
            return now - self._fist_since
        return 0.0

    def fist_held_toggle(self, now: float | None = None) -> bool:
        """True exactly once per continuous fist held longer than fist_hold_s."""
        now = now if now is not None else time.monotonic()
        if (self._fist and not self._fired and self._fist_since is not None
                and now - self._fist_since >= self._hold_s and not self.stale(now)):
            self._fired = True
            return True
        return False
