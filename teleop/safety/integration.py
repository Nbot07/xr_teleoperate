"""SafetyShim — the one object teleop_hand_and_arm.py needs to touch.

TARGET BASE: the merge of xr_teleoperate PRs #322 (hand-arm-walk-nod) and
#324 (onboard-deployment). A ready-made commit patch that performs every
integration edit below ships alongside this package
(`safety_framework_on_merged.patch`, apply with `git am`). The edits it makes:

  * builds the shim right after the loco/motion-switcher block and starts it
  * `LocoClientWrapper(auto_stand=False)` — standing is no longer a side
    effect of launching: press 'b' and confirm, and the supervisor runs the
    same verified damp→4→retry-501 sequence as an interruptible SafeAction
  * `sd = safety.update(tele_data, walk_mode=walk_mode)` each loop
  * PR #322's nod/steer block runs only when `sd.walk_input_allowed`, and
    every `loco_wrapper.Move(...)` (hand AND controller paths) is wrapped in
    `safety.filter_move(...)`
  * both-thumbstick damp routes through `safety.user_damp("thumbsticks")`
  * the arm command gains one branch: walk-freeze (theirs) → safety hold
    (skip the call; the arm controller's internal publisher keeps streaming
    the last targets) → normal tracking
  * `safety.controlled_shutdown()` first in `finally:` — 'q', Ctrl-C and
    crashes all de-energize through the safe pose
  * `on_press` forwards unclaimed keys to `safety.on_press(key)`; 'r' also
    calls `safety.notify_start()`

Keyboard map added (existing r/q/s untouched; works over SSH via sshkeyboard,
so it survives onboard/headless deployment):
    p  pause / resume            x  SAFE STOP (squat → damp)
    e  emergency damp (press twice within 2 s — from a tall stand this is a
       controlled COLLAPSE; prefer 'x')
    u  get up (lying/low → stand, asks for clearance confirmation)
    b  enter balance / stand for teleop (confirmed; verified 1→4→501 path)
    h  height mode toggle        1/2/3, y/n  answer the open safety decision

Hands-only safety gesture (no keyboard, no controllers needed):
    RAISE BOTH FISTS to head height and hold —
        ≥ 2 s : PAUSE (arms hold, legs stop; release to resume)
        ≥ 4 s : SAFE STOP (squat → damp; latched)
    The head-height requirement means a two-handed carry at torso height can
    never false-trigger it. Gesture arbitration: WALK mode (double nod,
    PR #322) blocks height-mode entry; height mode blocks walk input; the
    deadman overrides both.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .config import SafetyConfig
from .events import ConsoleNotifier
from .height import HeightGestureController
from .supervisor import SafetySupervisor


@dataclass
class LoopDirectives:
    arms_enabled: bool
    walk_input_allowed: bool
    height_active: bool
    reason: str
    status: str


class SafetyShim:
    def __init__(self, simulation_mode: bool = False, motion_mode: bool = True,
                 cfg: SafetyConfig | None = None, notifier=None):
        self.cfg = cfg or SafetyConfig.load()
        self.supervisor = SafetySupervisor(
            cfg=self.cfg,
            notifier=notifier or ConsoleNotifier(),
            simulation_mode=simulation_mode,
        )
        self.height = HeightGestureController(self.cfg, self.supervisor)
        self.motion_mode = motion_mode
        self._arms_latched_off = False
        self._deadman_paused = False
        self._deadman_stopped = False
        self._n = 0
        self._warned: dict[str, float] = {}

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        self.supervisor.start()

    def controlled_shutdown(self) -> None:
        if self.motion_mode:
            self.supervisor.controlled_shutdown()
        self.supervisor.stop()

    # ------------------------------------------------------------------ keys
    def on_press(self, key: str) -> bool:
        a = self.supervisor.authority
        if key == "p":
            paused = a.toggle_pause()
            self.supervisor.notifier.status("PAUSED" if paused else "resume requested")
        elif key == "x":
            a.safe_stop()
        elif key == "e":
            if a.emergency_damp() == "armed":
                self.supervisor.notifier.status(
                    "e-damp ARMED — press 'e' again within 2 s to drop torque")
        elif key == "u":
            self.supervisor.user_get_up()
        elif key == "b":
            self.supervisor.user_enter_balance()
        elif key == "h":
            self.height.request_toggle()
        elif key in ("1", "2", "3", "y", "n"):
            a.answer(key)
        else:
            return False
        return True

    def notify_start(self) -> None:
        """User pressed the existing 'r' start key: re-enable arms if allowed."""
        self._arms_latched_off = False
        self.supervisor.authority.resume_requested()

    # ------------------------------------------------ loco passthroughs
    def filter_move(self, vx: float, vy: float, vyaw: float):
        return self.supervisor.filter_move(vx, vy, vyaw)

    def user_damp(self, source: str = "operator") -> None:
        self.supervisor.user_damp(source)

    def height_active(self) -> bool:
        return self.height._mode  # noqa: SLF001

    # ------------------------------------------------------------------ loop
    def update(self, tele_data, walk_mode: bool = False) -> LoopDirectives:
        now = time.monotonic()
        if getattr(tele_data, "motion_data_ready", False):
            if not (self.height.left.stale(now) and self.height.right.stale(now)):
                self.supervisor.feed_xr()

        hd = self.height.update(tele_data, suppress_toggle=walk_mode)
        if hd.height_mode or hd.arms_frozen:
            self._arms_latched_off = True

        deadman_engaged = self._tick_deadman(tele_data, now)

        ok, why = self.supervisor.allow_arm_teleop()
        arms = (ok and not self._arms_latched_off and not hd.arms_frozen
                and not deadman_engaged)
        walk_ok = not hd.height_mode and not deadman_engaged

        self._n += 1
        if self._n % 15 == 0:  # ~2 Hz at 30 Hz loop
            line = hd.status or self.supervisor.status_line()
            if not arms and (why or hd.height_mode):
                line += f"  | arms held: {why or 'height mode'}"
            self.supervisor.notifier.status(line)
        return LoopDirectives(arms_enabled=arms,
                              walk_input_allowed=walk_ok,
                              height_active=hd.height_mode,
                              reason=why if not arms else "",
                              status=hd.status)

    # --------------------------------------------------------------- deadman
    def _warn_every(self, key: str, period_s: float, now: float, msg: str) -> None:
        """Rate-limited loud warning. The deadman must never fail quietly."""
        last = self._warned.get(key, -1e9)
        if now - last >= period_s:
            self._warned[key] = now
            self.supervisor.notifier.status(msg)

    def _tick_deadman(self, tele_data, now: float) -> bool:
        """Both fists raised to head height: hold ≥2 s → pause, ≥4 s → safe stop.

        Every way this gesture can be unavailable is announced. A silently
        disabled e-stop is worse than no e-stop, because the operator is
        relying on it.
        """
        g = self.cfg.gesture
        a = self.supervisor.authority
        L, R = self.height.left, self.height.right
        dm_stale = getattr(g, "deadman_stale_s", g.tracking_stale_exit_s)

        raised, readable = self._both_raised(tele_data, g.deadman_raise_below_head_m)
        if not readable:
            self._warn_every(
                "dm_head", 5.0, now,
                "⚠ DEADMAN UNAVAILABLE: head pose unreadable, raised-fists safe stop "
                "will NOT fire. Use 'x' (safe stop), 'e','e' (damp), or the RC.")

        fists_up = L.fist and R.fist and raised
        stale = L.stale(now, dm_stale) or R.stale(now, dm_stale)
        if fists_up and stale:
            self._warn_every(
                "dm_stale", 5.0, now,
                f"⚠ DEADMAN SUPPRESSED: hand tracking frozen >{dm_stale:.0f}s while both "
                "fists held. Gesture will NOT fire — use 'x' or the RC.")

        both = fists_up and not stale
        dur = min(L.fist_duration(now, dm_stale),
                  R.fist_duration(now, dm_stale)) if both else 0.0

        if both and dur >= g.both_fist_stop_s:
            if not self._deadman_stopped:
                self._deadman_stopped = True
                self.supervisor.notifier.status("DEADMAN HELD — SAFE STOP")
                a.safe_stop()
        elif both and dur >= g.both_fist_pause_s:
            if not self._deadman_paused:
                self._deadman_paused = True
                a.pause()
                self.supervisor.notifier.status(
                    "deadman: PAUSED — release to resume, keep holding for safe stop")
        elif not both:
            if self._deadman_paused and not self._deadman_stopped:
                a.resume_requested()
                self.supervisor.notifier.status("deadman released — resumed")
            self._deadman_paused = False
            self._deadman_stopped = False
        return both and dur >= g.both_fist_pause_s

    @staticmethod
    def _both_raised(tele_data, below_head_m: float):
        """Returns (raised, readable).

        `readable` is False when the poses cannot be evaluated at all. The
        caller announces that case rather than treating it as "not raised" --
        otherwise an unreadable head pose disables the e-stop in silence.
        """
        try:
            hz = float(tele_data.head_pose[2, 3])
            lz = float(tele_data.left_wrist_pose[2, 3])
            rz = float(tele_data.right_wrist_pose[2, 3])
        except Exception:  # noqa: BLE001
            return False, False
        if not all(map(math.isfinite, (hz, lz, rz))):
            return False, False
        return (lz > hz - below_head_m and rz > hz - below_head_m), True
