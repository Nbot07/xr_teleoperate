#!/usr/bin/env python3
"""Scenario harness for the safety supervisor — no robot, no simulator.

unitree_mujoco cannot cover these: it bridges low-level DDS only and
implements no LocoClient services, and the framework stubs loco entirely when
simulation_mode=True. So every action that actually commands the robot --
'b' stand, 'x' safe stop, height slewing, shutdown descent -- is unreachable
in a mujoco pass. This harness drives them against a recording fake client
and asserts the ORDER of what reaches the robot.

The property that matters most: for an actively-balanced robot, "cut torque"
and "safe" conflict while standing. Every stop must get low FIRST and only
then de-energize. A stop that damps from a tall stand is a fall.

    python teleop/safety/tests/test_scenarios.py
"""
from __future__ import annotations

import os
import sys
import time
import types

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

# ---------------------------------------------------------------- fake robot
import unitree_sdk2py.g1.loco.g1_loco_client as loco_mod


class FakeLoco:
    """Records every RPC in order, and answers state queries consistently."""

    log: list[tuple[float, str, tuple]] = []
    fsm: int = 0
    height: float = 0.0

    def __init__(self):
        FakeLoco.log = []
        FakeLoco.fsm = 0
        FakeLoco.height = 0.0

    def _rec(self, name, *a):
        FakeLoco.log.append((time.monotonic(), name, a))

    def SetTimeout(self, *a): pass
    def Init(self, *a): pass

    def SetFsmId(self, i):
        self._rec("SetFsmId", i)
        FakeLoco.fsm = int(i)
        return 0

    def GetFsmId(self):
        return 0, FakeLoco.fsm

    def SetStandHeight(self, h):
        self._rec("SetStandHeight", round(float(h), 4))
        FakeLoco.height = float(h)
        return 0

    def GetStandHeight(self):
        return 0, FakeLoco.height

    def StopMove(self):
        self._rec("StopMove")
        return 0

    def Damp(self):
        self._rec("Damp")
        FakeLoco.fsm = 1
        return 0

    def __getattr__(self, name):           # any other RPC: record and succeed
        def f(*a, **k):
            self._rec(name, *a)
            return 0
        return f


loco_mod.LocoClient = FakeLoco

from teleop.safety.config import SafetyConfig            # noqa: E402
from teleop.safety.states import ControlMode, Posture    # noqa: E402
from teleop.safety.supervisor import SafetySupervisor    # noqa: E402


def lowstate(*, knee=0.1, pitch=0.0, gyro=0.0, hip=0.0):
    """Synthetic rt/lowstate: knee~0.1 = standing, knee>1.3 = deep squat."""
    m = types.SimpleNamespace()
    m.imu_state = types.SimpleNamespace(rpy=(0.0, pitch, 0.0),
                                        gyroscope=(gyro, 0.0, 0.0))
    m.motor_state = [types.SimpleNamespace(q=0.0) for _ in range(35)]
    for i in (0, 6):
        m.motor_state[i].q = hip
    for i in (3, 9):
        m.motor_state[i].q = knee
    return m


def build(*, knee=0.1, fsm=None):
    cfg = SafetyConfig.load()
    sup = SafetySupervisor(cfg=cfg, simulation_mode=False)   # real path, fake client
    sup.notifier.status = lambda *a, **k: None
    sup.notifier.info = lambda *a, **k: None
    if fsm is not None:
        FakeLoco.fsm = fsm
    sup.estimator._on_lowstate(lowstate(knee=knee))
    sup.estimator._t_low = time.monotonic()
    sup.start()
    return sup


def set_mode(sup, fsm_id):
    """Drive control mode via the FSM id, the way the estimator really derives it."""
    FakeLoco.fsm = int(fsm_id)
    sup.estimator._fsm_id = int(fsm_id)
    sup.estimator._t_fsm = time.monotonic()
    sup._snap = sup.estimator.snapshot()
    return sup._snap


def names(log=None):
    return [n for _, n, _ in (log if log is not None else FakeLoco.log)]


def pump(sup, secs=6.0, *, knee_after=None, after_s=1.0):
    """Run the supervisor's action executor, feeding telemetry as it goes."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        k = knee_after if (knee_after is not None and
                           time.monotonic() - t0 > after_s) else None
        sup.estimator._on_lowstate(lowstate(knee=k if k is not None else 0.1))
        sup.estimator._t_low = time.monotonic()
        if sup.executor.active is None and time.monotonic() - t0 > 0.4:
            break
        time.sleep(0.02)


PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print(f"  [{'PASS' if cond else 'FAIL'}] {label}")
    if detail and not cond:
        print(f"         {detail}")


# ============================================================ 1. safe stop
print("1. SAFE STOP from a tall stand — must get low BEFORE de-energizing")
sup = build(knee=0.1, fsm=501)
set_mode(sup, sup.cfg.fsm.main_balance)
sup.authority.safe_stop()
pump(sup, 8.0, knee_after=1.5, after_s=1.5)   # robot descends as commanded
seq = names()
sup.stop()

heights = [i for i, n in enumerate(seq) if n == "SetStandHeight"]
damps = [i for i, n in enumerate(seq) if n == "SetFsmId" and
         FakeLoco.log[i][2] and FakeLoco.log[i][2][0] == sup.cfg.fsm.damp]
check("commanded height down before any damp",
      bool(heights) and (not damps or min(heights) < min(damps)),
      f"seq={seq[:12]}")
check("damp was reached", bool(damps), f"seq={seq[:12]}")
hvals = [a[0] for _, n, a in FakeLoco.log if n == "SetStandHeight"]
check("height command is monotonically descending",
      all(b <= a + 1e-6 for a, b in zip(hvals, hvals[1:])),
      f"heights={hvals[:8]}")

# ============================================== 2. move gating / clamping
print("\n2. filter_move() — the gate every Move() passes through")
sup = build(knee=0.1, fsm=501)
set_mode(sup, sup.cfg.fsm.main_balance)
big = sup.filter_move(99.0, -99.0, 99.0)
check("clamped to walk_vmax",
      max(abs(v) for v in big) <= sup.cfg.walk_vmax + 1e-9, f"got {big}")
sup.authority.pause()
check("zeroed while paused", sup.filter_move(0.2, 0.2, 0.2) == (0.0, 0.0, 0.0))
sup.authority.resume_requested()
set_mode(sup, 999)   # unknown fsm -> not balancing
check("zeroed when not balancing", sup.filter_move(0.2, 0.2, 0.2) == (0.0, 0.0, 0.0))
sup.stop()

# ======================================================== 3. arm authority
print("\n3. arm teleop authority")
sup = build(knee=0.1, fsm=501)
set_mode(sup, sup.cfg.fsm.main_balance)
sup.authority.pause()
ok, why = sup.allow_arm_teleop()
check("arms denied while paused", not ok and "paus" in why.lower(), f"why={why!r}")
sup.authority.resume_requested()
sup.stop()

# ============================================ 4. energy-reducing always ok
print("\n4. operator damp is accepted from any state")
sup = build(knee=0.1, fsm=501)
set_mode(sup, sup.cfg.fsm.main_balance)
sup.user_damp("test")
pump(sup, 4.0, knee_after=1.5, after_s=0.8)
seq = names()
sup.stop()
check("user_damp reached the robot", any(n in ("SetFsmId", "Damp") for n in seq),
      f"seq={seq[:10]}")

# ================================================ 5. controlled shutdown
print("\n5. controlled shutdown ('q' / Ctrl-C / crash)")
sup = build(knee=0.1, fsm=501)
set_mode(sup, sup.cfg.fsm.main_balance)
import threading
threading.Thread(target=lambda: pump(sup, 12.0, knee_after=1.5, after_s=1.5),
                 daemon=True).start()
sup.controlled_shutdown(timeout_s=12.0)
seq = names()
sup.stop()
h = [i for i, n in enumerate(seq) if n == "SetStandHeight"]
z = [i for i, n in enumerate(seq) if n == "SetFsmId" and
     FakeLoco.log[i][2] and FakeLoco.log[i][2][0] == sup.cfg.fsm.zero_torque]
check("descends before de-energizing", bool(h) and (not z or min(h) < min(z)),
      f"seq={seq[:14]}")

print(f"\n{sum(PASS)}/{len(PASS)} scenarios behaved as specified")
sys.exit(0 if all(PASS) else 1)
