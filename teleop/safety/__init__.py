"""Gantry-free safety framework for xr_teleoperate (G1 first, extensible).

Public surface:
    SafetyShim          — drop-in for teleop_hand_and_arm.py (see integration.py)
    SafetySupervisor    — always-on monitor/executor
    UserAuthority       — pause / safe stop / emergency damp / answers
    SafeAction & friends— shared pipeline for teleop, safety, and AI actions
    SafetyConfig        — thresholds + firmware FSM table (probe-verifiable)
"""
from .actions import (ControlledDeenergize, EnterTeleopBalance, GetUp,
                      SafeAction, SafeSquatThenDamp, Status)
from .authority import UserAuthority
from .config import SafetyConfig
from .events import ConsoleNotifier, Decision, Hazard, Notifier
from .integration import LoopDirectives, SafetyShim
from .states import ControlMode, Posture, Priority, RobotSnapshot, SafetyLevel
from .supervisor import SafetySupervisor

__all__ = [
    "SafetyShim", "LoopDirectives", "SafetySupervisor", "UserAuthority",
    "SafetyConfig", "SafeAction", "SafeSquatThenDamp", "ControlledDeenergize",
    "GetUp", "EnterTeleopBalance", "Status", "Notifier", "ConsoleNotifier",
    "Decision", "Hazard", "Posture", "ControlMode", "SafetyLevel", "Priority",
    "RobotSnapshot",
]
