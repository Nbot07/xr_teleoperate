"""Tick-based safe actions and their executor.

Every behavior — supervisor-initiated safety moves, user-initiated get-ups,
and (later) AI-proposed skills like "sit in chair" — is a SafeAction run by the
same executor, so preconditions, pause checkpoints, invariant monitoring, and
abort-to-safest are uniform. Actions never block: tick() is called at the
supervisor rate and must return quickly.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional

from .states import ControlMode, Posture, Priority, RobotSnapshot


class Status(Enum):
    RUNNING = auto()
    DONE = auto()
    FAILED = auto()


@dataclass
class ActionContext:
    """Capabilities handed to actions by the supervisor (no direct SDK access)."""
    snapshot: Callable[[], RobotSnapshot]
    set_fsm: Callable[[int], bool]
    damp: Callable[[], bool]
    zero_torque: Callable[[], bool]
    enter_balance: Callable[[], bool]          # tries main_balance_candidates
    slew_height_to: Callable[[float], None]    # non-blocking; supervisor slews
    height_target_reached: Callable[[], bool]
    h_min: Callable[[], float]
    notify: Callable[[str], None]
    fsm: object                                # config.FsmTable
    confirm: Callable[[str, float], Optional[bool]]  # opens y/n decision; None=pending


class SafeAction:
    name: str = "action"
    priority: Priority = Priority.USER

    def __init__(self):
        self._phase = 0
        self._t_phase = time.monotonic()
        self.pausable_now = True   # phases flip this off during non-interruptible motion
        self.paused = False
        self.pause_latched = False

    # ----- lifecycle ------------------------------------------------------
    def preconditions(self, snap: RobotSnapshot) -> tuple[bool, str]:
        return True, ""

    def tick(self, ctx: ActionContext) -> Status:
        raise NotImplementedError

    def abort(self, ctx: ActionContext) -> None:
        """Must leave the robot stable. Default: damp if already low, else
        route to the safe squat (the executor swaps in SafeSquatThenDamp)."""

    # ----- helpers --------------------------------------------------------
    def _goto(self, phase: int) -> None:
        self._phase = phase
        self._t_phase = time.monotonic()

    def _phase_elapsed(self) -> float:
        return time.monotonic() - self._t_phase

    def request_pause(self) -> None:
        self.pause_latched = True

    def maybe_pause_checkpoint(self, ctx: ActionContext) -> bool:
        """Call at safe checkpoints; honors a latched pause. Returns True if
        currently paused (caller should return Status.RUNNING immediately)."""
        if self.pause_latched and self.pausable_now:
            if not self.paused:
                self.paused = True
                ctx.notify(f"{self.name}: paused at checkpoint")
            return True
        if self.paused and not self.pause_latched:
            self.paused = False
            ctx.notify(f"{self.name}: resumed")
        return self.paused


# ---------------------------------------------------------------- concrete
class SafeSquatThenDamp(SafeAction):
    """Controlled descent: balance to h_min, settle, damp. The safe stop."""
    name = "safe_squat_then_damp"
    priority = Priority.SAFETY

    def __init__(self, then_zero_torque: bool = False):
        super().__init__()
        self._zt = then_zero_torque

    def tick(self, ctx: ActionContext) -> Status:
        snap = ctx.snapshot()
        if self._phase == 0:
            if snap.is_low() and snap.control_mode != ControlMode.BALANCING:
                self._goto(3)  # already low: damp directly
                return Status.RUNNING
            if snap.control_mode != ControlMode.BALANCING:
                ctx.notify("safe stop: robot not balancing; damping in place")
                self._goto(3)
                return Status.RUNNING
            ctx.notify("safe stop: descending to safe squat")
            ctx.slew_height_to(ctx.h_min())
            self._goto(1)
        elif self._phase == 1:
            if self.maybe_pause_checkpoint(ctx):
                return Status.RUNNING          # pausing mid-descent is fine: balance holds
            if ctx.height_target_reached() or self._phase_elapsed() > 12.0:
                self._goto(2)
        elif self._phase == 2:
            self.pausable_now = False          # settle+damp is atomic
            if self._phase_elapsed() > 0.6:
                self._goto(3)
        elif self._phase == 3:
            ctx.damp()
            self._goto(4)
        elif self._phase == 4:
            if self._phase_elapsed() < 1.0:
                return Status.RUNNING
            if not self._zt:
                ctx.notify("safe stop complete: damped in low pose")
                return Status.DONE
            snap = ctx.snapshot()
            if snap.is_low() and snap.gyro_mag < 0.5:
                ctx.zero_torque()
                ctx.notify("✅ de-energized (zero torque). Safe to power off.")
                return Status.DONE
            if self._phase_elapsed() > 5.0:
                ctx.notify("⚠ settle check failed; staying in DAMP (not zero-torque)")
                return Status.DONE
        return Status.RUNNING


class ControlledDeenergize(SafeSquatThenDamp):
    """Any energized state → safest reachable pose → damp → zero torque."""
    name = "controlled_deenergize"
    priority = Priority.SAFETY

    def __init__(self):
        super().__init__(then_zero_torque=True)


class EmergencyDamp(SafeAction):
    """Immediate torque-to-damping. Correct when already low or already falling;
    from a tall stand it is a collapse — UserAuthority makes it two-step."""
    name = "emergency_damp"
    priority = Priority.REFLEX

    def tick(self, ctx: ActionContext) -> Status:
        self.pausable_now = False
        ctx.damp()
        return Status.DONE


class GetUp(SafeAction):
    """User-initiated recovery from lying (FSM lie_to_stand) or low squat
    (FSM squat_to_stand). Requires explicit clearance confirmation."""
    name = "get_up"
    priority = Priority.USER

    def preconditions(self, snap: RobotSnapshot) -> tuple[bool, str]:
        if snap.posture == Posture.FALLING:
            return False, "robot is falling"
        if not snap.is_low():
            return False, "robot is not in a low pose"
        if snap.lowstate_age_s > 0.5:
            return False, "no fresh robot state"
        return True, ""

    def tick(self, ctx: ActionContext) -> Status:
        snap = ctx.snapshot()
        if self._phase == 0:
            ans = ctx.confirm(
                "GET UP: confirm ≥1.5 m clearance on all sides and a spotter?",
                20.0)
            if ans is None:
                return Status.RUNNING
            if ans is False:
                ctx.notify("get up cancelled")
                return Status.FAILED
            ctx.damp()  # documented prerequisite before stand transitions
            self._goto(1)
        elif self._phase == 1:
            if self._phase_elapsed() < 0.8:
                return Status.RUNNING
            target = (ctx.fsm.lie_to_stand if snap.is_lying()
                      else ctx.fsm.squat_to_stand)
            ctx.notify(f"get up: FSM {target} — do not touch the robot")
            self.pausable_now = False   # mid-get-up is not a stable stop point;
            ok = ctx.set_fsm(target)    # pause requests latch until upright
            if not ok:
                ctx.notify("get up: FSM command rejected")
                return Status.FAILED
            self._goto(2)
        elif self._phase == 2:
            if snap.posture in (Posture.STANDING, Posture.SQUAT) and snap.gyro_mag < 0.6:
                self.pausable_now = True
                if self.maybe_pause_checkpoint(ctx):
                    return Status.RUNNING
                ctx.notify("get up complete (position hold). Use start-teleop to balance.")
                return Status.DONE
            if self._phase_elapsed() > 20.0:
                ctx.notify("get up timed out; damping")
                ctx.damp()
                return Status.FAILED
        return Status.RUNNING


class EnterTeleopBalance(SafeAction):
    """Into main operation control via the ON-ROBOT VERIFIED sw 1.5.3 path
    (xr_teleoperate PR #322): damp(1) → stand(4, physical, several seconds) →
    retry SetFsmId(main_balance) until GetFsmId echoes it, because the balance
    id is silently ignored while the stand-up is still executing.

    This replaces LocoClientWrapper's blocking constructor stand-up: standing
    is a large motion, so it is a *confirmed* action here, not a side effect
    of launching the program. Phases 0-1 are pausable; once FSM 4 is sent the
    trajectory runs to completion (pause latches and is honored after).
    Abort semantics: before stand → damp; once standing → STAY standing
    (aborting a stand by damping means falling).
    """
    name = "enter_teleop_balance"
    priority = Priority.USER

    def preconditions(self, snap: RobotSnapshot) -> tuple[bool, str]:
        if snap.posture == Posture.FALLING or snap.fallen_latched:
            return False, "fallen latch set — run get-up/recovery first"
        return True, ""

    def tick(self, ctx: ActionContext) -> Status:
        snap = ctx.snapshot()
        fsm = ctx.fsm
        if self._phase == 0:                       # confirm clearance
            if snap.control_mode == ControlMode.BALANCING:
                return Status.DONE
            ans = ctx.confirm("robot will STAND UP — ≥2 m clearance and "
                              "spotter ready?", 20.0)
            if ans is None:
                return Status.RUNNING
            if not ans:
                ctx.notify("stand cancelled")
                return Status.FAILED
            ctx.damp()
            self._goto(1)
        elif self._phase == 1:                     # settle in damp, last pausable point
            if self.maybe_pause_checkpoint(ctx):
                return Status.RUNNING
            if self._phase_elapsed() >= 1.0:
                self.pausable_now = False
                ctx.set_fsm(fsm.stand_lock)        # physical stand-up begins
                ctx.notify("standing up (FSM 4)…")
                self._goto(2)
        elif self._phase == 2:                     # wait out the stand-up
            if self._phase_elapsed() >= fsm.stand_wait_s:
                self._retries = 0
                self._goto(3)
        elif self._phase == 3:                     # retry balance id until it takes
            target = fsm.main_balance
            if snap.fsm_id == target:
                ctx.notify(f"main operation control reached (FSM {target}) — "
                           "teleop enabled")
                return Status.DONE
            if self._phase_elapsed() >= fsm.balance_retry_period_s:
                self._retries += 1
                if self._retries > fsm.balance_retries:
                    ctx.notify(f"enter balance: FSM stuck at {snap.fsm_id}, "
                               f"wanted {target} — staying position-locked; "
                               "check clearance/battery and retry with 'b'")
                    return Status.FAILED           # do NOT damp from a stand
                ctx.set_fsm(target)
                self._goto(3)                      # restart phase timer
        return Status.RUNNING

    def abort(self, ctx: ActionContext) -> None:
        snap = ctx.snapshot()
        if self._phase <= 1 and snap.posture in (Posture.LYING_FRONT,
                                                 Posture.LYING_BACK,
                                                 Posture.LYING_SIDE,
                                                 Posture.SQUAT_LOW,
                                                 Posture.UNKNOWN):
            ctx.damp()
        # from phase 2 on the robot is standing or nearly so: damping would
        # drop it — leave the position-hold stand in place and let the user
        # decide (safe stop 'x' still works and descends first).


# ---------------------------------------------------------------- executor
@dataclass
class _Slot:
    action: SafeAction
    started: float = field(default_factory=time.monotonic)


class ActionExecutor:
    def __init__(self, ctx: ActionContext, notify: Callable[[str], None]):
        self._ctx = ctx
        self._notify = notify
        self._slot: Optional[_Slot] = None

    @property
    def active(self) -> Optional[SafeAction]:
        return self._slot.action if self._slot else None

    def submit(self, action: SafeAction) -> bool:
        cur = self.active
        if cur is not None:
            if action.priority.value <= cur.priority.value:
                self._notify(f"'{action.name}' rejected: '{cur.name}' is active")
                return False
            self._notify(f"'{cur.name}' preempted by '{action.name}'")
            try:
                cur.abort(self._ctx)
            except Exception:  # noqa: BLE001
                pass
        ok, why = action.preconditions(self._ctx.snapshot())
        if not ok:
            self._notify(f"'{action.name}' preconditions failed: {why}")
            return False
        self._slot = _Slot(action)
        return True

    def request_pause(self) -> None:
        if self.active:
            self.active.request_pause()

    def clear_pause(self) -> None:
        if self.active:
            self.active.pause_latched = False

    def tick(self) -> None:
        if not self._slot:
            return
        try:
            st = self._slot.action.tick(self._ctx)
        except Exception as e:  # noqa: BLE001
            self._notify(f"action '{self._slot.action.name}' crashed: {e}; damping")
            self._ctx.damp()
            st = Status.FAILED
        if st in (Status.DONE, Status.FAILED):
            self._slot = None
