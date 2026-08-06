"""State vocabulary for the gantry-free safety framework."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto


class Posture(Enum):
    UNKNOWN = auto()
    LYING_FRONT = auto()
    LYING_BACK = auto()
    LYING_SIDE = auto()
    SITTING_OR_LOW = auto()   # upright-ish torso, deep leg flexion, not balancing:
                              # floor-sit (possibly sprawled), kneel, or parked squat
    SQUAT = auto()            # balancing (or stand-lock) at low height
    STANDING = auto()         # upright, legs near-extended
    FALLING = auto()          # reflex-detected loss of balance in a balancing mode


class ControlMode(Enum):
    """Firmware control mode, derived from the polled FSM id."""
    UNKNOWN = auto()
    ZERO_TORQUE = auto()
    DAMPED = auto()
    POSITION_HOLD = auto()    # squat / sit / stand-lock style position FSMs
    BALANCING = auto()        # main operation control (teleop-capable)
    TRANSITION = auto()       # get-up or other firmware trajectory in progress


class SafetyLevel(Enum):
    NOMINAL = 0
    ADVISORY = 1
    WARNING = 2
    CRITICAL = 3
    REFLEX = 4


class Priority(Enum):
    AI_PROPOSED = 0
    USER = 1
    SAFETY = 2
    REFLEX = 3


@dataclass
class RobotSnapshot:
    """Everything the supervisor needs, sampled atomically-enough each tick."""
    t: float = field(default_factory=time.monotonic)
    posture: Posture = Posture.UNKNOWN
    control_mode: ControlMode = ControlMode.UNKNOWN
    fsm_id: int | None = None
    roll: float = 0.0
    pitch: float = 0.0
    gyro_mag: float = 0.0
    knee_mean: float = 0.0
    soc: float | None = None          # battery %, None until BMS seen
    stand_height: float | None = None # last GetStandHeight readback
    lowstate_age_s: float = float("inf")
    bms_age_s: float = float("inf")
    fsm_age_s: float = float("inf")
    fallen_latched: bool = False

    def is_low(self) -> bool:
        return self.posture in (
            Posture.LYING_FRONT, Posture.LYING_BACK, Posture.LYING_SIDE,
            Posture.SITTING_OR_LOW, Posture.SQUAT,
        )

    def is_lying(self) -> bool:
        return self.posture in (
            Posture.LYING_FRONT, Posture.LYING_BACK, Posture.LYING_SIDE,
        )
