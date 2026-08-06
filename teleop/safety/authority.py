"""Single source of truth for user authority over the robot.

Precedence (highest wins):
    emergency damp  >  safe stop  >  supervisor safety actions  >  pause/resume
    >  teleop  >  AI-proposed actions

Invariants:
  * Energy-reducing requests (pause, safe stop, damp) are ALWAYS accepted,
    from any state, at any time.
  * Energy-increasing requests (resume, stand, raise height) are gated by the
    supervisor and may be denied; an informed override exists but requires a
    fresh two-step confirmation and is logged.
  * Emergency damp from a tall stand means a collapse. The API forces a
    confirming second call within a short window so a single stray keypress
    cannot drop the robot; safe_stop() is the recommended everyday stop.
"""
from __future__ import annotations

import threading
import time
from typing import Optional


class UserAuthority:
    def __init__(self, edamp_confirm_window_s: float = 2.0):
        self._lock = threading.Lock()
        self._paused = False
        self._safe_stop = False
        self._edamp_armed_t: float | None = None
        self._edamp = False
        self._answers: dict[str, str] = {}
        self._override_armed_t: float | None = None
        self._edamp_window = edamp_confirm_window_s
        self.log: list[tuple[float, str]] = []

    # ------------------------------------------------------------- primitives
    def _note(self, s: str) -> None:
        self.log.append((time.monotonic(), s))

    # ------------------------------------------------------------------ pause
    def toggle_pause(self) -> bool:
        with self._lock:
            self._paused = not self._paused
            self._note(f"pause -> {self._paused}")
            return self._paused

    def pause(self) -> None:
        with self._lock:
            self._paused = True
            self._note("pause")

    def resume_requested(self) -> None:
        """Resume is a *request*; the supervisor grants it via its gates."""
        with self._lock:
            self._paused = False
            self._note("resume requested")

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._paused

    # -------------------------------------------------------------- safe stop
    def safe_stop(self) -> None:
        """Controlled descent to the safe pose, then damp. Always accepted."""
        with self._lock:
            self._safe_stop = True
            self._note("safe_stop")

    def consume_safe_stop(self) -> bool:
        with self._lock:
            v = self._safe_stop
            self._safe_stop = False
            return v

    # --------------------------------------------------------- emergency damp
    def emergency_damp(self) -> str:
        """Two-step: first call arms, second call within the window fires.

        Returns 'armed', 'fired', or 're-armed'.
        """
        with self._lock:
            now = time.monotonic()
            if (self._edamp_armed_t is not None
                    and now - self._edamp_armed_t <= self._edamp_window):
                self._edamp = True
                self._edamp_armed_t = None
                self._note("EMERGENCY DAMP fired")
                return "fired"
            self._edamp_armed_t = now
            self._note("emergency damp armed")
            return "armed"

    def consume_emergency_damp(self) -> bool:
        with self._lock:
            v = self._edamp
            self._edamp = False
            return v

    # ------------------------------------------------- risk-increasing override
    def arm_override(self) -> None:
        with self._lock:
            self._override_armed_t = time.monotonic()
            self._note("override armed")

    def take_override(self, window_s: float = 5.0) -> bool:
        with self._lock:
            ok = (self._override_armed_t is not None
                  and time.monotonic() - self._override_armed_t <= window_s)
            self._override_armed_t = None
            if ok:
                self._note("override TAKEN")
            return ok

    # -------------------------------------------------------- decision answers
    def answer(self, decision_id_or_key: str, key: Optional[str] = None) -> None:
        """answer(key) applies to the currently open decision ('*')."""
        with self._lock:
            if key is None:
                self._answers["*"] = decision_id_or_key
            else:
                self._answers[decision_id_or_key] = key
            self._note(f"answer {decision_id_or_key} {key or ''}".strip())

    def pop_answer(self, decision_id: str) -> Optional[str]:
        with self._lock:
            return self._answers.pop(decision_id, self._answers.pop("*", None))
