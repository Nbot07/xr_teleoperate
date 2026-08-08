#!/usr/bin/env python3
"""Step through FSM transitions one at a time, instrumented and abortable.

Written after a waist_pitch winding overheat (motorstate bit 9 = 512) took the
whole robot down mid-sequence and was only noticed minutes later, when a
command "succeeded" against a robot whose motors were all disabled. Everything
here exists to make that visible while it happens.

    python tools/fsm_sequence.py --network-interface eth0 1 2 4 2
    python tools/fsm_sequence.py --network-interface eth0 --watch 20     # observe only

Per step: shows the current state, waits for YES, sends SetFsmId, then polls at
5 Hz reporting joints, IMU, the hottest joint and any fault word, and finally
reports whether the FSM ACTUALLY changed and whether the robot ACTUALLY moved.

Three things this checks that a bare SetFsmId does not:

  * SetFsmId's return code is meaningless on this firmware — it returns 0 for
    transitions it silently ignores. Only GetFsmId is believed.
  * A transition can "succeed" with every motor disabled, doing nothing. Motor
    mode is checked before every step.
  * A joint can trip a thermal fault mid-trajectory. Fault words are polled
    throughout and abort the run immediately.

On abort it does NOT command anything. Damping a robot that is standing is a
fall, and after a fault the motors are already disabled — so the safe action is
to stop talking and let the operator use the RC.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

JOINTS = [
    "L_hip_p", "L_hip_r", "L_hip_y", "L_knee", "L_ank_p", "L_ank_r",
    "R_hip_p", "R_hip_r", "R_hip_y", "R_knee", "R_ank_p", "R_ank_r",
    "waist_yaw", "waist_roll", "waist_pitch",
    "L_sh_p", "L_sh_r", "L_sh_y", "L_elbow", "L_wr_r", "L_wr_p", "L_wr_y",
    "R_sh_p", "R_sh_r", "R_sh_y", "R_elbow", "R_wr_r", "R_wr_p", "R_wr_y",
]
WATCH = [0, 3, 4, 6, 9, 14]          # hips, knees, ankle, waist_pitch
WAIST_PITCH = 14

# Fault-word bits observed on this firmware. Extend as they are met.
FAULT_BITS = {512: "winding overheat"}

FSM_DESC = {
    0: "zero torque — fully limp",
    1: "damp — joints compliant, holds where it is",
    2: "squat (VERIFIED on sw 1.5.3: smooth ~5.5 s, holds)",
    3: "sit",
    4: "stand / position-hold (VERIFIED: ~5 s trajectory + active balancing)",
    500: "main control — IGNORED on sw 1.5.3 (returns 0, does nothing)",
    501: "main control / walkable (verified by PR #322)",
    702: "lie-to-stand — large motion",
    706: "squat-to-stand — IGNORED on sw 1.5.3 (returns 0, does nothing)",
}


def describe_fault(word: int) -> str:
    if word == 0:
        return ""
    named = [name for bit, name in FAULT_BITS.items() if word & bit]
    return f"{word} ({', '.join(named)})" if named else f"{word} (unknown bits)"


class Robot:
    def __init__(self, iface, sim=False):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hgLowState
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        ChannelFactoryInitialize(1 if sim else 0, networkInterface=iface)
        self._m = None
        self._n = 0
        ChannelSubscriber("rt/lowstate", hgLowState).Init(self._on, 10)
        self.loco = LocoClient()
        self.loco.SetTimeout(3.0)
        self.loco.Init()
        t0 = time.time()
        while self._m is None and time.time() - t0 < 10:
            time.sleep(0.05)
        if self._m is None:
            sys.exit("no lowstate — is DDS reaching the robot on this interface?")

    def _on(self, m):
        self._m = m
        self._n += 1

    def snap(self) -> dict:
        m = self._m
        q = [math.degrees(float(m.motor_state[i].q)) for i in range(29)]
        temps = [int(m.motor_state[i].temperature[0]) for i in range(29)]
        faults = {i: int(m.motor_state[i].motorstate) for i in range(29)
                  if m.motor_state[i].motorstate != 0}
        modes = [int(m.motor_state[i].mode) for i in range(29)]
        rpy = [math.degrees(float(x)) for x in m.imu_state.rpy]
        gyro = math.sqrt(sum(float(x) ** 2 for x in m.imu_state.gyroscope))
        return {"q": q, "temps": temps, "faults": faults, "modes": modes,
                "rpy": rpy, "gyro": gyro, "t": time.time()}

    def fsm(self):
        try:
            _, f = self.loco.GetFsmId()
            return f
        except Exception:
            return None


def line(s: dict, tag: str) -> str:
    hot = max(range(29), key=lambda i: s["temps"][i])
    return (f"{tag} " + " ".join(f"{JOINTS[i]}={s['q'][i]:+7.1f}" for i in WATCH)
            + f" | pitch={s['rpy'][1]:+6.1f} gyro={s['gyro']:4.2f}"
            + f" | waist_p={s['temps'][WAIST_PITCH]}C hot={JOINTS[hot]}:{s['temps'][hot]}C")


def preflight(r: Robot, args) -> bool:
    s = r.snap()
    ok = True
    dead = [JOINTS[i] for i in range(29) if s["modes"][i] == 0]
    if dead:
        print(f"  ⚠ {len(dead)} motors report mode 0 (DISABLED): {', '.join(dead[:6])}"
              f"{' …' if len(dead) > 6 else ''}")
        print("    A transition can 'succeed' against disabled motors and do nothing.")
        ok = False
    if s["faults"]:
        for i, w in s["faults"].items():
            print(f"  ⚠ FAULT {JOINTS[i]}: {describe_fault(w)}")
        ok = False
    hot = max(range(29), key=lambda i: s["temps"][i])
    print(f"  temps: hottest {JOINTS[hot]} {s['temps'][hot]}C, "
          f"waist_pitch {s['temps'][WAIST_PITCH]}C")
    if s["temps"][hot] >= args.temp_abort:
        print(f"  ⚠ {JOINTS[hot]} at/above abort threshold {args.temp_abort}C")
        ok = False
    elif s["temps"][hot] >= args.temp_warn:
        print(f"  ⚠ {JOINTS[hot]} above warn threshold {args.temp_warn}C")
    print(f"  FSM: {r.fsm()}   lowstate msgs: {r._n}")
    return ok


def run_step(r: Robot, fsm_id: int, args, log: list) -> bool:
    """Returns False to abort the whole run."""
    before = r.snap()
    f_before = r.fsm()
    print(f"\n--- SetFsmId({fsm_id}) — {FSM_DESC.get(fsm_id, 'UNKNOWN id')} ---")
    print(f"  {line(before, 'before')}")
    print(f"  FSM now: {f_before}")
    if not preflight(r, args):
        print("  preflight FAILED — not sending. Fix the above first.")
        return False

    a = input(f"  type YES to send {fsm_id}, Enter to skip, q to quit: ").strip().lower()
    if a == "q":
        return False
    if a != "yes":
        print("  skipped.")
        return True

    code = r.loco.SetFsmId(fsm_id)
    print(f"  SetFsmId returned {code}  (return code is NOT trustworthy on this firmware)")

    traj, aborted = [], False
    t0 = time.time()
    while time.time() - t0 < args.settle:
        time.sleep(0.2)
        s = r.snap()
        traj.append({"dt": round(time.time() - t0, 2), "q": [round(x, 2) for x in s["q"]],
                     "rpy": [round(x, 2) for x in s["rpy"]], "gyro": round(s["gyro"], 3),
                     "temps": s["temps"]})
        if s["faults"]:
            print(f"  {line(s, f'+{time.time()-t0:4.1f}s')}")
            for i, w in s["faults"].items():
                print(f"  🛑 FAULT on {JOINTS[i]}: {describe_fault(w)}")
            print("  🛑 ABORTING. Not sending anything further — use the RC if it needs damping.")
            aborted = True
            break
        if s["temps"][WAIST_PITCH] >= args.temp_abort:
            print(f"  🛑 waist_pitch {s['temps'][WAIST_PITCH]}C >= abort {args.temp_abort}C")
            aborted = True
            break
        if len(traj) % 3 == 0:
            print(f"  {line(s, f'+{time.time()-t0:4.1f}s')}")

    after = r.snap()
    f_after = r.fsm()
    moved = max(abs(after["q"][i] - before["q"][i]) for i in range(29))
    print(f"  {line(after, 'after ')}")
    print(f"  FSM {f_before} -> {f_after}   max joint change {moved:.1f}deg")
    if f_after == f_before and moved < 1.0:
        print("  ==> ACCEPTED AND IGNORED: FSM unchanged and nothing moved.")
    elif f_after != f_before and moved < 1.0:
        print("  ==> FSM CHANGED BUT NO MOTION — check motor modes (disabled?).")
    elif f_after == f_before:
        print("  ==> MOVED BUT FSM UNCHANGED — transient or rejected end state.")
    else:
        print("  ==> EXECUTED.")
    dtemp = after["temps"][WAIST_PITCH] - before["temps"][WAIST_PITCH]
    if dtemp:
        print(f"  waist_pitch temp change over this step: {dtemp:+d}C")

    log.append({"fsm_requested": fsm_id, "fsm_before": f_before, "fsm_after": f_after,
                "code": code, "max_joint_change_deg": round(moved, 2),
                "aborted": aborted,
                "before": {"q": [round(x, 2) for x in before["q"]],
                           "temps": before["temps"], "rpy": before["rpy"]},
                "after": {"q": [round(x, 2) for x in after["q"]],
                          "temps": after["temps"], "rpy": after["rpy"]},
                "trajectory": traj})
    return not aborted


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sequence", nargs="*", type=int, help="FSM ids, e.g. 1 2 4 2")
    ap.add_argument("--network-interface", default=None)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--settle", type=float, default=12.0, help="seconds to watch per step")
    ap.add_argument("--temp-warn", type=int, default=60)
    ap.add_argument("--temp-abort", type=int, default=75)
    ap.add_argument("--watch", type=float, default=0.0, help="observe only, N seconds")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    r = Robot(args.network_interface, args.sim)

    if args.watch:
        t0 = time.time()
        while time.time() - t0 < args.watch:
            s = r.snap()
            print(line(s, f"+{time.time()-t0:5.1f}s"))
            for i, w in s["faults"].items():
                print(f"   FAULT {JOINTS[i]}: {describe_fault(w)}")
            time.sleep(1.0)
        return 0

    if not args.sequence:
        print("no sequence given. Current state:")
        preflight(r, args)
        return 0

    print("=" * 74)
    print(" FSM SEQUENCE RUNNER — one step at a time, aborts on any motor fault")
    print(" RC in hand. On abort this tool sends NOTHING; damping a standing")
    print(" robot is a fall, so recovery is yours to choose.")
    print("=" * 74)

    log: list = []
    try:
        for fsm_id in args.sequence:
            if not run_step(r, fsm_id, args, log):
                print("\nrun stopped.")
                break
    except KeyboardInterrupt:
        print("\ninterrupted — sending nothing. Use the RC if the robot needs damping.")

    out = args.out or os.path.expanduser(
        f"~/fsm_sequence_{time.strftime('%Y%m%d_%H%M%S')}.json")
    with open(out, "w") as f:
        json.dump({"sequence": args.sequence, "steps": log}, f, indent=2)
    print(f"\nrecorded {len(log)} step(s) -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
