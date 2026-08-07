#!/usr/bin/env python3
"""Deadman gesture: the hands-only e-stop for gantry-free teleop.

Covers the normal path, the false-positive defence, and both silent-failure
modes that were fixed: an unreadable head pose, and staleness suppression.
A silently disabled e-stop is worse than none, so "it warns" is asserted, not
assumed.
"""
import sys, types
import numpy as np

sys.path.insert(0, __import__("os").path.abspath(
    __import__("os").path.join(__import__("os").path.dirname(__file__), "..", "..", "..")))
from teleop.safety.integration import SafetyShim

_RNG = np.random.default_rng(7)


def hand(origin, jitter=1e-4) -> np.ndarray:
    """25x3 landmarks shaped as a closed fist (curl ratio ~0.9)."""
    p = _RNG.normal(0.0, jitter, (25, 3)) if jitter else np.zeros((25, 3))
    p[0] += origin
    p[10] += origin + [0.0, 0.0, 0.09]
    for tip in (9, 14, 19, 24):
        p[tip] += origin + [0.0, 0.0, 0.08]
    return p


def td(*, head_z=1.5, hand_z=1.45, head_ok=True, jitter=1e-4):
    o = types.SimpleNamespace()
    o.head_pose = np.eye(4)
    if head_ok:
        o.head_pose[2, 3] = head_z
    else:
        o.head_pose = None
    o.left_wrist_pose = np.eye(4);  o.left_wrist_pose[2, 3] = hand_z
    o.right_wrist_pose = np.eye(4); o.right_wrist_pose[2, 3] = hand_z
    o.left_hand_pos = hand(np.array([0.2, 0.2, hand_z]), jitter)
    o.right_hand_pos = hand(np.array([0.2, -0.2, hand_z]), jitter)
    o.left_hand_pinch = o.right_hand_pinch = False
    o.left_hand_pinchValue = o.right_hand_pinchValue = 10.0
    o.motion_data_ready = True
    return o


class Rec:
    def __init__(self): self.calls = []
    def __getattr__(self, name):
        def f(*a, **k):
            self.calls.append(name)
            return False if name == "toggle_pause" else None
        return f


def run(label, secs, *, head_ok=True, hand_z=1.45, jitter=1e-4,
        expect_pause, expect_stop, expect_warn=None):
    shim = SafetyShim(simulation_mode=True, motion_mode=False)
    rec = Rec(); shim.supervisor.authority = rec
    msgs = []
    shim.supervisor.notifier.status = lambda m="", *a, **k: msgs.append(str(m))

    t, dt = 1000.0, 1.0 / 30.0
    for _ in range(int(secs / dt)):
        t += dt
        d = td(hand_z=hand_z, head_ok=head_ok, jitter=jitter)
        shim.height.left.update(d.left_hand_pos, t)
        shim.height.right.update(d.right_hand_pos, t)
        shim._tick_deadman(d, t)

    got = [c for c in rec.calls if c in ("pause", "safe_stop")]
    ok = (("pause" in got) == expect_pause) and (("safe_stop" in got) == expect_stop)
    warn_txt = " | ".join(m for m in msgs if "DEADMAN" in m)
    if expect_warn is not None:
        ok = ok and (expect_warn in warn_txt)
    note = f"  warns: {warn_txt[:58]}" if warn_txt else ""
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}\n        calls={got or 'none'}{note}")
    return ok


print("normal operation")
r = [
    run("held 1 s   -> nothing",       1.0, expect_pause=False, expect_stop=False),
    run("held 2.5 s -> pause",         2.5, expect_pause=True,  expect_stop=False),
    run("held 4.5 s -> pause + stop",  4.5, expect_pause=True,  expect_stop=True),
    run("torso-height carry -> never", 4.5, hand_z=0.9,
        expect_pause=False, expect_stop=False),
]

print("\nFIX 1 — unreadable head pose must ANNOUNCE, not vanish")
r.append(run("head_pose=None", 4.5, head_ok=False,
             expect_pause=False, expect_stop=False,
             expect_warn="DEADMAN UNAVAILABLE"))

print("\nFIX 2 — perfectly still hold (zero jitter) must still reach 4 s")
r.append(run("static landmarks, 4.5 s", 4.5, jitter=0.0,
             expect_pause=True, expect_stop=True))
print("        (previously suppressed: stale budget was 2.0 s == pause threshold)")

print("\nFIX 2b — truly dead tracking is still rejected, and says so")
r.append(run("static landmarks, 7 s", 7.0, jitter=0.0,
             expect_pause=True, expect_stop=True,
             expect_warn="DEADMAN SUPPRESSED"))
print("        (fires before 5 s, then warns once the data is genuinely dead)")

print(f"\n{sum(r)}/{len(r)} behaved as specified")
sys.exit(0 if all(r) else 1)
