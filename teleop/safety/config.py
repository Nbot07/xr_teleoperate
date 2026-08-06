"""Central configuration for the gantry-free safety framework.

Values marked VERIFY are firmware-dependent assumptions. Run tools/fsm_probe.py
on your robot (on a mat, lying down) to validate them; the probe writes
fsm_verified.json next to this file and SafetyConfig.load() will prefer it.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, asdict

_VERIFIED_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fsm_verified.json")


@dataclass
class FsmTable:
    """G1 loco-service FSM ids.

    zero_torque/damp/sit/lie_to_stand/squat_to_stand and Start()->500 match the
    current unitree_sdk2_python source. On sw 1.5.3 the walkable "main
    operation control" state was verified ON-ROBOT (xr_teleoperate PR #322,
    LocoClientWrapper._enter_control_mode) to be FSM 501, reached via
    damp(1) -> stand(4) -> retry 501; SetFsmId(500) returns success from the
    R2+A Running mode (802) but the robot never leaves it. stand_lock=4 is a
    PHYSICAL stand-up taking several seconds, during which 501 is silently
    ignored — hence the retry loop in EnterTeleopBalance. G1_CTRL_FSM env
    overrides the first candidate, matching the wrapper. squat=2 semantics
    still need the probe.
    """
    zero_torque: int = 0
    damp: int = 1
    squat: int = 2                 # VERIFY semantics on your firmware
    sit: int = 3
    stand_lock: int = 4            # verified: position-hold stand-up (physical, ~5 s)
    main_balance: int = 501        # verified walkable on sw 1.5.3 (PR #322)
    main_balance_candidates: tuple = (501, 500, 200)
    lie_to_stand: int = 702
    squat_to_stand: int = 706
    verified: bool = False         # set True by fsm_probe results
    # EnterTeleopBalance sequencing (matches the on-robot verified wrapper):
    stand_wait_s: float = 5.0      # wait after SetFsmId(stand_lock) before retrying balance
    balance_retry_period_s: float = 1.0
    balance_retries: int = 12


@dataclass
class BatteryTiers:
    advisory_soc: float = 30.0
    warning_soc: float = 15.0
    critical_soc: float = 8.0
    warning_countdown_s: float = 60.0   # time user gets before auto safe-squat
    snooze_s: float = 120.0
    # Soft height ceiling shrinks linearly from full range at warning_soc
    # down to h_min at critical_soc (see Supervisor.height_ceiling()).


@dataclass
class TiltLimits:
    upright_max_rad: float = 0.35       # |roll|,|pitch| below this => upright
    lying_min_rad: float = 1.10         # above this => lying
    falling_tilt_rad: float = 0.62      # ~35 deg while balancing => REFLEX
    falling_gyro_rad_s: float = 4.0     # gyro magnitude spike => REFLEX


@dataclass
class Watchdogs:
    xr_freeze_s: float = 0.5            # stale XR data: freeze arms
    xr_warning_s: float = 2.0           # stale XR data: WARNING + countdown
    lowstate_stale_s: float = 0.5       # blind: attempt Damp immediately
    fsm_poll_period_s: float = 0.5
    bms_stale_s: float = 10.0


@dataclass
class HeightConfig:
    # Absolute accepted range in meters is firmware-specific; until the probe
    # measures it we confine motion to +/- pre_probe_span around the height
    # read at supervisor start. Probe results override abs_min/abs_max.
    abs_min: float | None = None        # VERIFY via probe
    abs_max: float | None = None        # VERIFY via probe
    pre_probe_span: float = 0.12
    slew_m_s: float = 0.05
    rpc_period_s: float = 0.2           # max SetStandHeight rate (5 Hz)
    settle_tol_m: float = 0.015


@dataclass
class GestureConfig:
    fist_hold_s: float = 0.8            # LEFT-fist hold toggles height mode (right hand open)
    fist_close_ratio: float = 0.55      # fingertip-to-wrist vs open-hand ref
    pinch_gain: float = 0.6             # robot dh per hand dz while pinched
    head_follow_gain: float = 1.0
    head_deadband_m: float = 0.03
    idle_exit_s: float = 10.0
    tracking_stale_exit_s: float = 2.0
    # BOTH-fist deadman (works in every hand-mode state, incl. WALK):
    both_fist_pause_s: float = 2.0      # hold both fists this long -> PAUSE (releases on open)
    both_fist_stop_s: float = 4.0       # keep holding -> SAFE STOP (squat -> damp, latched)
    # The deadman only counts with both fists RAISED near head height, so a
    # two-handed carry at torso height can never false-trigger it:
    deadman_raise_below_head_m: float = 0.25


@dataclass
class SafetyConfig:
    fsm: FsmTable = field(default_factory=FsmTable)
    battery: BatteryTiers = field(default_factory=BatteryTiers)
    tilt: TiltLimits = field(default_factory=TiltLimits)
    watchdogs: Watchdogs = field(default_factory=Watchdogs)
    height: HeightConfig = field(default_factory=HeightConfig)
    gesture: GestureConfig = field(default_factory=GestureConfig)
    use_fsm_squat_for_safe_pose: bool = False   # enable after probe verifies FSM 2
    bms_topic_candidates: tuple = ("rt/lf/bmsstate", "rt/bmsstate")  # VERIFY
    supervisor_hz: float = 50.0
    edamp_confirm_window_s: float = 2.0
    walk_vmax: float = 0.3              # hard clamp on |vx|,|vy|,|vyaw| (upstream limit)

    @classmethod
    def load(cls) -> "SafetyConfig":
        cfg = cls()
        if os.path.exists(_VERIFIED_JSON):
            try:
                with open(_VERIFIED_JSON) as f:
                    data = json.load(f)
                fsm = data.get("fsm", {})
                for k, v in fsm.items():
                    if hasattr(cfg.fsm, k):
                        setattr(cfg.fsm, k, v)
                h = data.get("height", {})
                if "abs_min" in h:
                    cfg.height.abs_min = h["abs_min"]
                if "abs_max" in h:
                    cfg.height.abs_max = h["abs_max"]
                cfg.fsm.verified = bool(fsm.get("verified", True))
            except Exception as e:  # noqa: BLE001
                print(f"[safety.config] failed to load {_VERIFIED_JSON}: {e}")
        return cfg

    def save_verified(self, fsm_updates: dict, height_updates: dict) -> str:
        payload = {"fsm": {**fsm_updates, "verified": True}, "height": height_updates}
        with open(_VERIFIED_JSON, "w") as f:
            json.dump(payload, f, indent=2)
        return _VERIFIED_JSON

    def to_dict(self) -> dict:
        return asdict(self)
