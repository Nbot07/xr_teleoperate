"""SafetySupervisor — the always-on layer between teleop/AI and the G1.

Runs at cfg.supervisor_hz in its own thread:
  watchdogs (XR, lowstate) → falling reflex → battery ladder → decisions →
  height slewing → active-action tick.

All LocoClient RPCs go through this object (one lock), all hazards surface
through the Notifier, and all user input arrives via UserAuthority.
"""
from __future__ import annotations

import os
import threading
import time
from typing import Callable, Optional

from .actions import (ActionContext, ActionExecutor, ControlledDeenergize,
                      EnterTeleopBalance, GetUp, SafeAction, SafeSquatThenDamp)
from .authority import UserAuthority
from .config import SafetyConfig
from .estimator import RobotStateEstimator
from .events import ConsoleNotifier, Decision, Notifier
from .states import ControlMode, Posture, RobotSnapshot, SafetyLevel


class SafetySupervisor:
    def __init__(self,
                 cfg: Optional[SafetyConfig] = None,
                 notifier: Optional[Notifier] = None,
                 authority: Optional[UserAuthority] = None,
                 simulation_mode: bool = False,
                 on_arm_release: Optional[Callable[[], None]] = None):
        self.cfg = cfg or SafetyConfig.load()
        # honor the same override LocoClientWrapper uses (PR #322)
        env_fsm = os.environ.get("G1_CTRL_FSM")
        if env_fsm:
            try:
                t = int(env_fsm)
                self.cfg.fsm.main_balance = t
                self.cfg.fsm.main_balance_candidates = tuple(
                    [t] + [c for c in self.cfg.fsm.main_balance_candidates if c != t])
            except ValueError:
                pass
        self.notifier = notifier or ConsoleNotifier()
        self.authority = authority or UserAuthority(self.cfg.edamp_confirm_window_s)
        self.simulation_mode = simulation_mode
        self._on_arm_release = on_arm_release or (lambda: None)

        self._loco = None
        self._loco_lock = threading.Lock()
        if not simulation_mode:
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            self._loco = LocoClient()
            self._loco.SetTimeout(0.3)
            self._loco.Init()

        self.estimator = RobotStateEstimator(self.cfg, self._loco, self._loco_lock)

        # height slewing state
        self._h_init: Optional[float] = None
        self._h_cmd: Optional[float] = None
        self._h_target: Optional[float] = None
        self._h_safety = False
        self._h_last_rpc = 0.0

        self.level = SafetyLevel.NOMINAL
        self._decision: Optional[Decision] = None
        self._decision_action: dict[str, Callable[[], None]] = {}
        self._battery_ack: set[str] = set()
        self._snooze_until = 0.0
        self._t_xr = 0.0
        self._last_blind_damp = 0.0
        self._pending_confirm: Optional[bool] = None

        ctx = ActionContext(
            snapshot=self.snapshot, set_fsm=self._set_fsm, damp=self._damp,
            zero_torque=self._zero_torque, enter_balance=self._enter_balance,
            slew_height_to=self._slew_height_to_safety,
            height_target_reached=self._height_target_reached,
            h_min=self.h_min, notify=self._notify_info, fsm=self.cfg.fsm,
            confirm=self._confirm,
        )
        self.executor = ActionExecutor(ctx, self._notify_info)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._snap = RobotSnapshot()

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        self.estimator.start()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="safety-supervisor")
        self._thread.start()
        self.notifier.toast(SafetyLevel.NOMINAL,
                            "supervisor online (FSM table "
                            + ("VERIFIED" if self.cfg.fsm.verified
                               else "UNVERIFIED — run tools/fsm_probe.py") + ")")

    def stop(self) -> None:
        self._running = False
        self.estimator.stop()

    # ------------------------------------------------------------------ RPCs
    def _rpc(self, fn: Callable, *a) -> bool:
        if self._loco is None:
            self._notify_info(f"[sim] {getattr(fn, '__name__', fn)}{a}")
            return True
        try:
            with self._loco_lock:
                code = fn(*a)
            return code in (0, None)
        except Exception as e:  # noqa: BLE001
            self._notify_info(f"RPC {getattr(fn, '__name__', fn)} failed: {e}")
            return False

    def _set_fsm(self, fsm_id: int) -> bool:
        return self._rpc(self._loco.SetFsmId, fsm_id) if self._loco else self._rpc(lambda i: 0, fsm_id)

    def _damp(self) -> bool:
        self._on_arm_release()
        return self._set_fsm(self.cfg.fsm.damp)

    def _zero_torque(self) -> bool:
        return self._set_fsm(self.cfg.fsm.zero_torque)

    def _enter_balance(self) -> bool:
        for cand in self.cfg.fsm.main_balance_candidates:
            if not self._set_fsm(cand):
                continue
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.5:
                if self.snapshot().fsm_id == cand:
                    self.cfg.fsm.main_balance = cand
                    return True
                time.sleep(0.05)
        return self._loco is None  # sim: pretend success

    # --------------------------------------------------------------- heights
    def h_min(self) -> float:
        c = self.cfg.height
        if c.abs_min is not None:
            return c.abs_min
        base = self._h_init if self._h_init is not None else 0.0
        return base - c.pre_probe_span

    def h_max(self) -> float:
        c = self.cfg.height
        if c.abs_max is not None:
            return c.abs_max
        base = self._h_init if self._h_init is not None else 0.0
        return base + c.pre_probe_span * 0.5   # cautious upward before probe

    def height_ceiling(self) -> float:
        """Soft ceiling that descends with the battery."""
        soc = self._snap.soc
        b = self.cfg.battery
        if soc is None or soc >= b.warning_soc:
            return self.h_max()
        if soc <= b.critical_soc:
            return self.h_min()
        f = (soc - b.critical_soc) / (b.warning_soc - b.critical_soc)
        return self.h_min() + f * (self.h_max() - self.h_min())

    def height_available(self) -> bool:
        return self._h_cmd is not None

    def request_height_target(self, h_abs: float, from_user: bool = True) -> bool:
        if from_user:
            if self.authority.paused:
                return False
            if self._snap.control_mode != ControlMode.BALANCING:
                return False
            if self.level.value >= SafetyLevel.WARNING.value and h_abs > (self._h_cmd or h_abs):
                self._notify_info("height raise denied (WARNING+)")
                return False
            h_abs = min(h_abs, self.height_ceiling())
        self._h_target = max(self.h_min(), min(self.h_max(), h_abs))
        self._h_safety = not from_user
        return True

    def nudge_height(self, dh: float) -> bool:
        if self._h_cmd is None:
            return False
        return self.request_height_target(self._h_cmd + dh, from_user=True)

    def _slew_height_to_safety(self, h_abs: float) -> None:
        self.request_height_target(h_abs, from_user=False)

    def _height_target_reached(self) -> bool:
        if self._h_cmd is None or self._h_target is None:
            return True
        return abs(self._h_cmd - self._h_target) <= self.cfg.height.settle_tol_m

    def _tick_height(self, dt: float) -> None:
        snap = self._snap
        if self._h_init is None and snap.stand_height is not None:
            self._h_init = snap.stand_height
            self._h_cmd = snap.stand_height
        if (self._h_cmd is None or self._h_target is None
                or snap.control_mode != ControlMode.BALANCING):
            return
        if self.authority.paused and not self._h_safety:
            return
        step = self.cfg.height.slew_m_s * dt
        err = self._h_target - self._h_cmd
        if abs(err) <= 1e-4:
            return
        self._h_cmd += max(-step, min(step, err))
        now = time.monotonic()
        if now - self._h_last_rpc >= self.cfg.height.rpc_period_s and self._loco is not None:
            self._h_last_rpc = now
            self._rpc(self._loco.SetStandHeight, float(self._h_cmd))

    # ----------------------------------------------------------------- gates
    def allow_arm_teleop(self) -> tuple[bool, str]:
        s = self._snap
        if self.authority.paused:
            return False, "paused"
        if self.level == SafetyLevel.CRITICAL:
            return False, "critical safety level"
        if self.executor.active and self.executor.active.priority.value >= 2:
            return False, f"safety action '{self.executor.active.name}' running"
        if s.control_mode != ControlMode.BALANCING:
            return False, f"not balancing (mode={s.control_mode.name})"
        if time.monotonic() - self._t_xr > self.cfg.watchdogs.xr_freeze_s:
            return False, "XR data stale"
        return True, ""

    def allow_locomotion(self) -> tuple[bool, str]:
        s = self._snap
        if self.authority.paused:
            return False, "paused"
        if self.level.value >= SafetyLevel.CRITICAL.value:
            return False, "critical safety level"
        if self.executor.active and self.executor.active.priority.value >= 2:
            return False, f"safety action '{self.executor.active.name}' running"
        if s.control_mode != ControlMode.BALANCING:
            return False, f"not balancing (mode={s.control_mode.name})"
        return True, ""

    def filter_move(self, vx: float, vy: float, vyaw: float) -> tuple[float, float, float]:
        """Every Move() flows through here: gated to zero when locomotion is
        not allowed, and hard-clamped to cfg.walk_vmax otherwise."""
        ok, _ = self.allow_locomotion()
        if not ok:
            return 0.0, 0.0, 0.0
        m = self.cfg.walk_vmax
        clamp = lambda v: max(-m, min(m, float(v)))   # noqa: E731
        return clamp(vx), clamp(vy), clamp(vyaw)

    def user_damp(self, source: str = "operator") -> None:
        """Immediate operator damp (e.g. both thumbsticks pressed). Energy-
        reducing, therefore always accepted; logged and announced."""
        self.authority._note(f"user damp ({source})")  # noqa: SLF001
        self.notifier.toast(SafetyLevel.REFLEX, f"OPERATOR DAMP ({source})")
        self._damp()

    # --------------------------------------------------------------- helpers
    def _notify_info(self, msg: str) -> None:
        self.notifier.toast(self.level, msg)

    def _confirm(self, message: str, deadline_s: float) -> Optional[bool]:
        """y/n decision used by actions; default = no."""
        if self._pending_confirm is not None:
            v, self._pending_confirm = self._pending_confirm, None
            return v
        if self._decision is None:
            self._open_decision(Decision("confirm", message,
                                         {"y": "yes", "n": "no"}, "n", deadline_s),
                                {"y": lambda: self._set_confirm(True),
                                 "n": lambda: self._set_confirm(False)})
        return None

    def _set_confirm(self, v: bool) -> None:
        self._pending_confirm = v

    def _open_decision(self, d: Decision, handlers: dict[str, Callable[[], None]]) -> None:
        self._decision = d
        self._decision_action = handlers
        self.notifier.open_decision(d)

    def _tick_decision(self) -> None:
        d = self._decision
        if d is None:
            return
        key = self.authority.pop_answer(d.decision_id)
        if key is None and d.expired():
            key = d.default_key
        if key is None:
            return
        if key not in d.options:
            return
        self._decision = None
        self.notifier.close_decision(d.decision_id, d.options[key])
        handler = self._decision_action.get(key)
        if handler:
            handler()

    # ------------------------------------------------------------ escalation
    def _battery_ladder(self) -> None:
        soc = self._snap.soc
        b = self.cfg.battery
        if soc is None:
            return
        if soc <= b.critical_soc:
            self.level = SafetyLevel.CRITICAL
            act = self.executor.active
            if not isinstance(act, (SafeSquatThenDamp, ControlledDeenergize)):
                self.notifier.toast(SafetyLevel.CRITICAL,
                                    f"battery {soc:.0f}% CRITICAL — descending to safe "
                                    f"pose NOW (not vetoable)")
                self.executor.submit(SafeSquatThenDamp())
            return
        if soc <= b.warning_soc:
            self.level = max(self.level, SafetyLevel.WARNING, key=lambda l: l.value)
            if (self._decision is None and "warn" not in self._battery_ack
                    and time.monotonic() >= self._snooze_until):
                self._open_decision(
                    Decision("battery_warning",
                             f"battery {soc:.0f}% — auto safe-squat when timer expires",
                             {"1": "continue (ack)", "2": "squat now",
                              "3": f"snooze {b.snooze_s:.0f}s"},
                             "2", b.warning_countdown_s),
                    {"1": lambda: self._battery_ack.add("warn"),
                     "2": lambda: self.executor.submit(SafeSquatThenDamp()),
                     "3": self._snooze})
            return
        if soc <= b.advisory_soc:
            if self.level.value < SafetyLevel.ADVISORY.value:
                self.level = SafetyLevel.ADVISORY
            if "adv" not in self._battery_ack:
                self._battery_ack.add("adv")
                self.notifier.toast(SafetyLevel.ADVISORY, f"battery {soc:.0f}%")
            return
        self._battery_ack.discard("warn")
        if self.level in (SafetyLevel.ADVISORY, SafetyLevel.WARNING):
            self.level = SafetyLevel.NOMINAL

    def _snooze(self) -> None:
        self._snooze_until = time.monotonic() + self.cfg.battery.snooze_s

    def _watchdogs(self) -> None:
        s = self._snap
        w = self.cfg.watchdogs
        if s.lowstate_age_s > w.lowstate_stale_s:
            self.level = SafetyLevel.CRITICAL
            if time.monotonic() - self._last_blind_damp > 1.0:
                self._last_blind_damp = time.monotonic()
                self.notifier.toast(SafetyLevel.CRITICAL,
                                    "lowstate LOST — attempting damp (RC is your backup)")
                self._damp()
            return
        xr_age = time.monotonic() - self._t_xr
        if (self._t_xr > 0 and xr_age > w.xr_warning_s
                and s.control_mode == ControlMode.BALANCING
                and self._decision is None and self.executor.active is None):
            self._open_decision(
                Decision("xr_lost",
                         "hand tracking lost — safe-squat when timer expires",
                         {"1": "I'm here, continue", "2": "squat now"}, "2", 15.0),
                {"1": lambda: None,
                 "2": lambda: self.executor.submit(SafeSquatThenDamp())})

    def _reflex(self) -> None:
        if self._snap.posture == Posture.FALLING:
            self.level = SafetyLevel.REFLEX
            self.notifier.toast(SafetyLevel.REFLEX, "FALLING — damping now")
            self._damp()

    # ------------------------------------------------------------------ loop
    def _loop(self) -> None:
        period = 1.0 / self.cfg.supervisor_hz
        t_prev = time.monotonic()
        while self._running:
            t = time.monotonic()
            dt, t_prev = t - t_prev, t
            self._snap = self.estimator.snapshot()
            self._reflex()
            self._watchdogs()
            self._battery_ladder()
            self._tick_decision()
            if self.authority.consume_safe_stop():
                self.executor.submit(SafeSquatThenDamp())
            if self.authority.consume_emergency_damp():
                self.notifier.toast(SafetyLevel.REFLEX, "EMERGENCY DAMP")
                self._damp()
            if self.authority.paused:
                self.executor.request_pause()
                if self._loco is not None and self._snap.control_mode == ControlMode.BALANCING:
                    self._rpc(self._loco.StopMove)
            else:
                self.executor.clear_pause()
            self._tick_height(dt)
            self.executor.tick()
            time.sleep(max(0.0, period - (time.monotonic() - t)))

    # ------------------------------------------------------------ public API
    def snapshot(self) -> RobotSnapshot:
        return self._snap

    def feed_xr(self) -> None:
        self._t_xr = time.monotonic()

    def submit(self, action: SafeAction) -> bool:
        return self.executor.submit(action)

    def user_get_up(self) -> None:
        self.estimator.clear_fallen_latch()
        self.submit(GetUp())

    def user_enter_balance(self) -> None:
        self.submit(EnterTeleopBalance())

    def controlled_shutdown(self, timeout_s: float = 25.0) -> None:
        """Blocking: safest reachable pose → damp → zero torque."""
        self.submit(ControlledDeenergize())
        t0 = time.monotonic()
        while self.executor.active is not None and time.monotonic() - t0 < timeout_s:
            time.sleep(0.1)

    def status_line(self) -> str:
        s = self._snap
        soc = f"{s.soc:.0f}%" if s.soc is not None else "--"
        h = f"{self._h_cmd:.2f}m" if self._h_cmd is not None else "--"
        act = self.executor.active.name if self.executor.active else "-"
        return (f"{self.level.name} | {s.control_mode.name}/{s.posture.name} | "
                f"soc {soc} | h {h} | act {act} | "
                f"{'PAUSED' if self.authority.paused else 'live'}")
