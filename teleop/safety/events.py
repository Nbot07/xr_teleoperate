"""Hazards, timed decisions, and the notifier seam (console now, XR overlay later)."""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .states import SafetyLevel


@dataclass
class Decision:
    """A user-facing choice with a deadline and a default.

    options: mapping of single-key answer -> label, e.g.
        {"c": "continue", "s": "squat now", "z": "snooze"}
    default_key is executed when the deadline passes unanswered.
    """
    decision_id: str
    message: str
    options: dict
    default_key: str
    deadline_s: float
    created: float = field(default_factory=time.monotonic)
    answered: Optional[str] = None

    def remaining(self) -> float:
        return max(0.0, self.deadline_s - (time.monotonic() - self.created))

    def expired(self) -> bool:
        return self.remaining() <= 0.0


@dataclass
class Hazard:
    hazard_id: str
    level: SafetyLevel
    message: str
    decision: Optional[Decision] = None


class Notifier:
    """Protocol. Implementations must be non-blocking."""

    def toast(self, level: SafetyLevel, message: str) -> None:
        raise NotImplementedError

    def open_decision(self, decision: Decision) -> None:
        raise NotImplementedError

    def close_decision(self, decision_id: str, outcome: str) -> None:
        raise NotImplementedError

    def status(self, line: str) -> None:
        """High-rate one-line status (height gauge, mode). May be dropped."""


class ConsoleNotifier(Notifier):
    def __init__(self, log: Callable[[str], None] | None = None):
        self._log = log or (lambda s: print(s, flush=True))
        self._last_status = ""
        self._lock = threading.Lock()

    def toast(self, level: SafetyLevel, message: str) -> None:
        icon = {SafetyLevel.NOMINAL: "·", SafetyLevel.ADVISORY: "🔵",
                SafetyLevel.WARNING: "🟡", SafetyLevel.CRITICAL: "🔴",
                SafetyLevel.REFLEX: "⚡"}.get(level, "·")
        self._log(f"[safety]{icon} {message}")

    def open_decision(self, decision: Decision) -> None:
        opts = "  ".join(f"[{k}]={v}" for k, v in decision.options.items())
        self._log(
            f"[safety]❓ {decision.message}\n"
            f"         {opts}   (default '[{decision.default_key}]' in "
            f"{decision.deadline_s:.0f}s — answer with the key)"
        )

    def close_decision(self, decision_id: str, outcome: str) -> None:
        self._log(f"[safety]✔ decision '{decision_id}' -> {outcome}")

    def status(self, line: str) -> None:
        with self._lock:
            if line != self._last_status:
                self._last_status = line
                self._log(f"[safety] {line}")
