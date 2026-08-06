"""Robot state estimation from rt/lowstate (IMU + joints), BMS, and FSM polls.

Posture is classified coarsely — the gates only need to distinguish
lying / low / standing / falling reliably. Sign conventions for lying
front-vs-back are firmware/mounting dependent: the probe's posture test will
tell you if `_LYING_PITCH_SIGN_FRONT` must be flipped.
"""
from __future__ import annotations

import math
import threading
import time

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hgLowState

try:  # BmsState_ exists in current unitree_sdk2_python; keep optional.
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_ as hgBmsState
except Exception:  # noqa: BLE001
    hgBmsState = None

from .config import SafetyConfig
from .states import ControlMode, Posture, RobotSnapshot

# G1 29-DoF leg joint indices in motor_state (see robot_control/robot_arm.py).
_L_HIP_PITCH, _L_KNEE = 0, 3
_R_HIP_PITCH, _R_KNEE = 6, 9
_KNEE_STAND_MAX = 0.55       # rad: below => legs near-extended
_KNEE_DEEP_MIN = 1.3         # rad: above => deep flexion (squat/sit)
_LYING_PITCH_SIGN_FRONT = -1  # VERIFY with probe: sign of pitch when face-down


class RobotStateEstimator:
    def __init__(self, cfg: SafetyConfig, loco=None, loco_lock: threading.Lock | None = None):
        self._cfg = cfg
        self._loco = loco                       # shared LocoClient (may be None in sim)
        self._loco_lock = loco_lock or threading.Lock()
        self._lock = threading.Lock()
        self._running = False

        self._t_low = 0.0
        self._t_bms = 0.0
        self._t_fsm = 0.0
        self._rpy = (0.0, 0.0, 0.0)
        self._gyro_mag = 0.0
        self._knee_mean = 0.0
        self._hip_mean = 0.0
        self._soc: float | None = None
        self._fsm_id: int | None = None
        self._stand_height: float | None = None
        self.fallen_latched = False

    # ------------------------------------------------------------------ setup
    def start(self) -> None:
        self._running = True
        try:
            self._low_sub = ChannelSubscriber("rt/lowstate", hgLowState)
            self._low_sub.Init(self._on_lowstate, 10)
        except Exception as e:  # noqa: BLE001  (no DDS: degrade to no-telemetry)
            print(f"[safety.estimator] lowstate subscribe failed: {e}")
        self._bms_subs = []
        if hgBmsState is not None:
            for topic in self._cfg.bms_topic_candidates:
                try:
                    sub = ChannelSubscriber(topic, hgBmsState)
                    sub.Init(self._on_bms, 5)
                    self._bms_subs.append(sub)
                except Exception:  # noqa: BLE001
                    pass
        if self._loco is not None:
            threading.Thread(target=self._fsm_poll_loop, daemon=True,
                             name="safety-fsm-poll").start()

    def stop(self) -> None:
        self._running = False

    # -------------------------------------------------------------- callbacks
    def _on_lowstate(self, msg: hgLowState):
        rpy = msg.imu_state.rpy
        g = msg.imu_state.gyroscope
        knees = 0.5 * (msg.motor_state[_L_KNEE].q + msg.motor_state[_R_KNEE].q)
        hips = 0.5 * (msg.motor_state[_L_HIP_PITCH].q + msg.motor_state[_R_HIP_PITCH].q)
        with self._lock:
            self._rpy = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
            self._gyro_mag = math.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)
            self._knee_mean = abs(float(knees))
            self._hip_mean = float(hips)
            self._t_low = time.monotonic()

    def _on_bms(self, msg):
        with self._lock:
            try:
                self._soc = float(msg.soc)
            except Exception:  # noqa: BLE001
                return
            self._t_bms = time.monotonic()

    def _fsm_poll_loop(self):
        period = self._cfg.watchdogs.fsm_poll_period_s
        while self._running:
            try:
                with self._loco_lock:
                    code, fsm = self._loco.GetFsmId()
                    hcode, h = (1, None)
                    try:
                        hcode, h = self._loco.GetStandHeight()
                    except Exception:  # noqa: BLE001
                        pass
                with self._lock:
                    if code == 0 and fsm is not None:
                        self._fsm_id = int(fsm)
                        self._t_fsm = time.monotonic()
                    if hcode == 0 and h is not None:
                        self._stand_height = float(h)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(period)

    # ------------------------------------------------------------ classifiers
    def _control_mode(self, fsm_id: int | None) -> ControlMode:
        f = self._cfg.fsm
        if fsm_id is None:
            return ControlMode.UNKNOWN
        if fsm_id == f.zero_torque:
            return ControlMode.ZERO_TORQUE
        if fsm_id == f.damp:
            return ControlMode.DAMPED
        if fsm_id in (f.squat, f.sit, f.stand_lock):
            return ControlMode.POSITION_HOLD
        if fsm_id in f.main_balance_candidates:
            return ControlMode.BALANCING
        if fsm_id in (f.lie_to_stand, f.squat_to_stand):
            return ControlMode.TRANSITION
        return ControlMode.UNKNOWN

    def _posture(self, roll, pitch, gyro, knee, mode: ControlMode) -> Posture:
        t = self._cfg.tilt
        tilt = max(abs(roll), abs(pitch))
        if mode == ControlMode.BALANCING and (
            tilt > t.falling_tilt_rad or gyro > t.falling_gyro_rad_s
        ):
            return Posture.FALLING
        if tilt > t.lying_min_rad:
            if abs(pitch) >= abs(roll):
                sign_front = _LYING_PITCH_SIGN_FRONT
                return (Posture.LYING_FRONT
                        if math.copysign(1.0, pitch) == sign_front
                        else Posture.LYING_BACK)
            return Posture.LYING_SIDE
        if tilt <= t.upright_max_rad:
            if knee <= _KNEE_STAND_MAX:
                return Posture.STANDING
            if knee >= _KNEE_DEEP_MIN:
                return (Posture.SQUAT
                        if mode in (ControlMode.BALANCING, ControlMode.POSITION_HOLD)
                        else Posture.SITTING_OR_LOW)
            return Posture.STANDING if mode == ControlMode.BALANCING else Posture.SITTING_OR_LOW
        return Posture.SITTING_OR_LOW  # leaned / sprawled but not flat

    # --------------------------------------------------------------- snapshot
    def snapshot(self) -> RobotSnapshot:
        now = time.monotonic()
        with self._lock:
            roll, pitch, _ = self._rpy
            gyro = self._gyro_mag
            knee = self._knee_mean
            soc = self._soc
            fsm = self._fsm_id
            h = self._stand_height
            t_low, t_bms, t_fsm = self._t_low, self._t_bms, self._t_fsm
        mode = self._control_mode(fsm)
        if not t_low or (now - t_low) > self._cfg.watchdogs.lowstate_stale_s:
            posture = Posture.UNKNOWN          # blind: never claim a pose
        else:
            posture = self._posture(roll, pitch, gyro, knee, mode)
        if posture == Posture.FALLING:
            self.fallen_latched = True
        return RobotSnapshot(
            t=now, posture=posture, control_mode=mode, fsm_id=fsm,
            roll=roll, pitch=pitch, gyro_mag=gyro, knee_mean=knee,
            soc=soc, stand_height=h,
            lowstate_age_s=(now - t_low) if t_low else float("inf"),
            bms_age_s=(now - t_bms) if t_bms else float("inf"),
            fsm_age_s=(now - t_fsm) if t_fsm else float("inf"),
            fallen_latched=self.fallen_latched,
        )

    def clear_fallen_latch(self) -> None:
        self.fallen_latched = False
