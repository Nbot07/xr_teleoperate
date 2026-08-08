#!/usr/bin/env python3
"""Record the robot's CURRENT joint angles as a named, hand-editable pose file.

Intended use: put the robot into a statically stable pose by hand (motors
damped/limp), then run this to capture it. The result becomes ground truth —
the pose the robot starts in, and the pose it returns to when something goes
wrong.

    python tools/capture_pose.py --network-interface eth0 --name safety_squat
    python tools/capture_pose.py --network-interface eth0 --name safety_squat --dry-run

Reads only. Sends nothing to the robot.

The output is deliberately one line per named joint, in degrees, so a single
joint can be nudged without disturbing the rest of the pose:

    left_knee:  118.4   # <- change just this, everything else holds

Degrees because that is what a human can reason about while looking at the
robot; the loader converts to radians.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# G1 29-DoF ordering, from teleop/robot_control/robot_arm.py
JOINTS = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]
LEGS = slice(0, 12)
WAIST = slice(12, 15)
ARMS = slice(15, 29)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--network-interface", default=None)
    ap.add_argument("--name", default="safety_squat")
    ap.add_argument("--samples", type=int, default=60,
                    help="averaged to reject encoder noise (default 60 ~ 2 s)")
    ap.add_argument("--sim", action="store_true", help="DDS domain 1")
    ap.add_argument("--dry-run", action="store_true", help="print, do not write")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as hgLowState

    ChannelFactoryInitialize(1 if args.sim else 0,
                             networkInterface=args.network_interface)

    frames: list = []
    sub = ChannelSubscriber("rt/lowstate", hgLowState)
    sub.Init(lambda m: frames.append(m), 10)

    print(f"collecting {args.samples} lowstate frames ...")
    t0 = time.time()
    while len(frames) < args.samples and time.time() - t0 < 15.0:
        time.sleep(0.05)
    if len(frames) < args.samples:
        print(f"ERROR: only {len(frames)} frames in 15 s — is DDS reaching the robot?",
              file=sys.stderr)
        return 2

    use = frames[-args.samples:]
    q_rad, spread = [], []
    for i in range(len(JOINTS)):
        vals = [float(f.motor_state[i].q) for f in use]
        q_rad.append(statistics.median(vals))
        spread.append(max(vals) - min(vals))

    rpy = [float(x) for x in use[-1].imu_state.rpy]
    temps = [int(use[-1].motor_state[i].temperature[0]) for i in range(len(JOINTS))]
    faults = [JOINTS[i] for i in range(len(JOINTS))
              if use[-1].motor_state[i].motorstate != 0]

    # --------------------------------------------------------------- report
    print(f"\n{'joint':<22} {'deg':>8} {'rad':>8} {'noise°':>7} {'°C':>4}")
    print("-" * 54)
    for i, name in enumerate(JOINTS):
        if i == 0:
            print("  # legs")
        elif i == 12:
            print("  # waist")
        elif i == 15:
            print("  # arms")
        print(f"{name:<22} {math.degrees(q_rad[i]):8.2f} {q_rad[i]:8.4f} "
              f"{math.degrees(spread[i]):7.3f} {temps[i]:4d}")

    print(f"\nIMU  roll={math.degrees(rpy[0]):+.2f}°  pitch={math.degrees(rpy[1]):+.2f}°  "
          f"yaw={math.degrees(rpy[2]):+.2f}°")
    worst = max(range(len(JOINTS)), key=lambda i: spread[i])
    print(f"noisiest joint: {JOINTS[worst]} ({math.degrees(spread[worst]):.3f}° spread)")
    if faults:
        print(f"⚠ motors reporting a fault word: {', '.join(faults)}")
    if math.degrees(max(abs(rpy[0]), abs(rpy[1]))) > 25:
        print("⚠ torso tilt >25° — expected for a folded-forward pose, noted for the record")

    # ------------------------------------------------------------- write out
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "..", "teleop", "safety", "poses",
                                   f"{args.name}.yaml")
    out = os.path.abspath(out)
    lines = [
        f"# {args.name} — captured from the robot on "
        f"{time.strftime('%Y-%m-%d %H:%M:%S')}",
        "#",
        "# Angles are DEGREES so you can reason about them while looking at the robot.",
        "# Edit ONE line to nudge ONE joint; everything else holds its captured value.",
        "# Re-run tools/capture_pose.py to re-measure from a hand-posed robot.",
        "#",
        f"# IMU at capture: roll={math.degrees(rpy[0]):+.2f} "
        f"pitch={math.degrees(rpy[1]):+.2f} yaw={math.degrees(rpy[2]):+.2f}",
        f"# encoder noise: max {math.degrees(max(spread)):.3f}° "
        f"over {args.samples} frames",
        "",
        "meta:",
        f"  name: {args.name}",
        f"  captured_utc: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
        "  units: degrees",
        "  source: measured from a hand-posed robot (motors compliant)",
        "",
        "joints:",
    ]
    for i, name in enumerate(JOINTS):
        tag = ""
        if i == 0:
            lines.append("  # ---- legs ----")
        elif i == 12:
            lines.append("  # ---- waist ----")
        elif i == 15:
            lines.append("  # ---- arms ----")
        lines.append(f"  {name + ':':<24}{math.degrees(q_rad[i]):8.2f}{tag}")
    text = "\n".join(lines) + "\n"

    if args.dry_run:
        print("\n--- would write ---\n" + text)
        return 0

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(text)
    print(f"\n✅ wrote {out}")
    print(f"   also: {out.replace('.yaml', '.json')}")
    with open(out.replace(".yaml", ".json"), "w") as f:
        json.dump({"meta": {"name": args.name, "units": "radians"},
                   "joints": dict(zip(JOINTS, q_rad)),
                   "imu_rpy": rpy}, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
