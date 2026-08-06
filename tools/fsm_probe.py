#!/usr/bin/env python3
"""fsm_probe — verify firmware FSM ids and stand-height limits from the ground up.

The safety framework refuses to *assume* what FSM 2/4/500/501/702/706 do on
your firmware. This tool walks them one at a time, starting with the robot
LYING ON A MAT, motors on, wide clear area, spotter present, RC in hand.

Usage:
    python tools/fsm_probe.py --network-interface eth0
    python tools/fsm_probe.py --sim            # DDS domain 1, dry logic check

For each candidate it: shows what is EXPECTED, requires you to type YES,
sends SetFsmId, polls GetFsmId + the estimator's posture for 8 s, and records
what actually happened. The height sweep (run only once you confirm the robot
is balancing) steps SetStandHeight in ±2 cm increments and reads back
GetStandHeight to find the accepted range. Results are written to
teleop/safety/fsm_verified.json, which SafetyConfig.load() picks up.

Abort any step with just Enter. 'd' at any prompt = damp immediately.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from teleop.safety.config import SafetyConfig
from teleop.safety.estimator import RobotStateEstimator

BANNER = """
================================  FSM PROBE  ================================
 START CONDITION: robot LYING on a padded mat, ≥2 m clearance all sides,
 battery >50 %, spotter ready, RC in your hand (L2+B / damp is your backup).
 This tool will command real transitions, including STAND-UPS.
=============================================================================
"""

STEPS = [
    ("damp",            "Damping — joints go compliant, robot stays put"),
    ("zero_torque",     "Zero torque — fully limp (from damp)"),
    ("damp",            "Back to damp before trajectories"),
    ("lie_to_stand",    "LIE-TO-STAND get-up trajectory — ROBOT WILL MOVE A LOT"),
    ("stand_lock",      "Position-hold stand ('get ready') — VERIFYING id 4"),
    ("squat",           "Firmware squat pose — VERIFYING id 2 semantics"),
    ("squat_to_stand",  "Squat-to-stand trajectory (id 706)"),
    ("main_balance",    "Main operation control — candidates in order (501 first: verified "
                        "walkable on sw 1.5.3 by PR #322; see also teleop/fsm_explore.py)"),
]


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        return "d"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--network-interface", default=None)
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(1 if args.sim else 0,
                             networkInterface=args.network_interface)

    cfg = SafetyConfig()          # deliberately NOT .load(): probe from defaults
    loco = None
    import threading
    lock = threading.Lock()
    if not args.sim:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        loco = LocoClient()
        loco.SetTimeout(0.5)
        loco.Init()
    est = RobotStateEstimator(cfg, loco, lock)
    est.start()

    print(BANNER)
    if ask("Type YES to confirm the start condition above: ") != "yes":
        print("aborted.")
        return

    def damp_now():
        if loco:
            with lock:
                loco.SetFsmId(cfg.fsm.damp)
        print(">>> DAMP sent")

    def snap_str():
        s = est.snapshot()
        return (f"fsm={s.fsm_id} posture={s.posture.name} "
                f"tilt=({s.roll:+.2f},{s.pitch:+.2f}) knee={s.knee_mean:.2f} "
                f"soc={s.soc}")

    results: dict = {}
    for name, desc in STEPS:
        candidates = (list(cfg.fsm.main_balance_candidates)
                      if name == "main_balance" else [getattr(cfg.fsm, name)])
        for fsm_id in candidates:
            print(f"\n--- {name} (FSM {fsm_id}) ---\n    expected: {desc}")
            print(f"    before : {snap_str()}")
            a = ask("    type YES to send, Enter to skip, d to damp: ")
            if a == "d":
                damp_now()
                continue
            if a != "yes":
                continue
            if loco:
                with lock:
                    code = loco.SetFsmId(fsm_id)
                print(f"    SetFsmId -> code {code}")
            for _ in range(16):
                time.sleep(0.5)
                print(f"    …      : {snap_str()}")
            verdict = ask("    matched expectation? y/n/d: ")
            if verdict == "d":
                damp_now()
                verdict = "n"
            results.setdefault(name, {})[str(fsm_id)] = (verdict == "y")
            if name == "main_balance" and verdict == "y":
                results["main_balance_id"] = fsm_id
                break

    # ---- stand-height sweep (only while balancing) -------------------------
    h_meas = {}
    if ask("\nRobot balancing now and area clear? Run height sweep? y/n: ") == "y" and loco:
        with lock:
            _, h0 = loco.GetStandHeight()
        print(f"    current stand height readback: {h0}")
        if h0 is not None:
            h0 = float(h0)
            for direction in (-1, +1):
                h, last_ok = h0, h0
                for _ in range(20):
                    h += direction * 0.02
                    with lock:
                        loco.SetStandHeight(float(h))
                    time.sleep(1.2)
                    with lock:
                        _, hb = loco.GetStandHeight()
                    print(f"    cmd {h:+.2f} -> readback {hb}")
                    if hb is None or abs(float(hb) - h) > 0.03:
                        break
                    last_ok = float(hb)
                    if ask("    continue this direction? y/n: ") != "y":
                        break
                h_meas["abs_min" if direction < 0 else "abs_max"] = last_ok
                with lock:
                    loco.SetStandHeight(h0)
                time.sleep(2.0)
        damp_after = ask("    sweep done. damp the robot? y/n: ")
        if damp_after == "y":
            damp_now()

    # ---- write verified table ---------------------------------------------
    fsm_out = {}
    if "main_balance_id" in results:
        fsm_out["main_balance"] = results["main_balance_id"]
    for key in ("squat", "stand_lock"):
        r = results.get(key, {})
        ok_ids = [int(k) for k, v in r.items() if v]
        if ok_ids:
            fsm_out[key] = ok_ids[0]
    path = cfg.save_verified(fsm_out, h_meas)
    print(f"\nraw results: {json.dumps(results, indent=2)}")
    print(f"height     : {json.dumps(h_meas, indent=2)}")
    print(f"✅ wrote {path} — SafetyConfig.load() will use it from now on.")
    print("Re-run this probe after every firmware update.")
    est.stop()


if __name__ == "__main__":
    main()
