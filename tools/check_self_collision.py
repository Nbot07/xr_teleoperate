#!/usr/bin/env python3
"""Check a pose — or an interpolated path between two poses — for self-collision.

The safety squat folds the robot tightly: hips −145°, knees +166°, torso +29°
forward, arms brought down and in. The legs carry the weight, so the arms only
have to *arrive* at their target without hitting anything on the way. That makes
self-collision an offline geometry question, answerable here with no GPU and no
robot.

    python tools/check_self_collision.py --pose teleop/safety/poses/safety_squat.yaml
    python tools/check_self_collision.py --from stand.yaml --to safety_squat.yaml --steps 40

The robot's floating base is parked high above the floor so every contact
reported is robot-on-robot.

IMPORTANT: this first verifies that self-collision is actually detectable in the
model. Many robot models ship with contype/conaffinity that exclude self-pairs
for speed, which would make every pose look clean. If the self-test fails, the
results here mean nothing and it says so.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

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

# The hand-equipped model, shipped in this repo and already used by
# robot_arm_ik.py. unitree_mujoco's g1 scene has NO hands -- its chain ends at
# wrist_yaw_link -- so it cannot see hand collisions at all, which is the thing
# worth protecting.
DEFAULT_SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "assets", "g1", "g1_body29_hand14.xml")

HAND_KEYS = ("hand", "finger", "palm", "thumb", "index", "middle", "ring", "pinky")


def load_pose(path: str) -> list[float]:
    """Read the degrees-per-named-joint YAML written by tools/capture_pose.py."""
    vals: dict[str, float] = {}
    in_joints = False
    for raw in open(path):
        line = raw.split("#")[0].rstrip()
        if not line.strip():
            continue
        if line.startswith("joints:"):
            in_joints = True
            continue
        if in_joints and not line.startswith((" ", "\t")):
            in_joints = False
        if in_joints and ":" in line:
            k, v = line.split(":", 1)
            try:
                vals[k.strip()] = float(v.strip())
            except ValueError:
                pass
    missing = [j for j in JOINTS if j not in vals]
    if missing:
        sys.exit(f"{path}: missing joints {missing[:5]}")
    return [math.radians(vals[j]) for j in JOINTS]


def contacts(model, data, mujoco, floor_geoms: set) -> list[tuple[str, str, float]]:
    mujoco.mj_forward(model, data)
    out = []
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if g1 in floor_geoms or g2 in floor_geoms:
            continue
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g1) or f"geom{g1}"
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g2) or f"geom{g2}"
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g1])
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g2])
        out.append((f"{b1}/{n1}", f"{b2}/{n2}", float(c.dist)))
    return out


def build_index(model, mujoco) -> list[int]:
    """qpos address for each of our 29 body joints, matched BY NAME.

    The hand-equipped model interleaves the finger joints after each arm
    (left hand at qpos 29-35, right at 43-49), so positional indexing is wrong.
    """
    adr = []
    for j in JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j + "_joint")
        if jid < 0:
            sys.exit(f"model has no joint named {j}_joint")
        adr.append(int(model.jnt_qposadr[jid]))
    return adr


def set_pose(model, data, q: list[float], adr: list[int]):
    data.qpos[:] = 0.0
    data.qpos[2] = 3.0            # park the base well above the floor
    data.qpos[3] = 1.0            # unit quaternion
    for a, v in zip(adr, q):      # fingers stay at 0 (open hand)
        data.qpos[a] = v
    data.qvel[:] = 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--pose")
    ap.add_argument("--from", dest="src")
    ap.add_argument("--to", dest="dst")
    ap.add_argument("--steps", type=int, default=40)
    ap.add_argument("--all", action="store_true",
                    help="report every self-contact, not just hand ones")
    args = ap.parse_args()

    import mujoco
    if not os.path.exists(args.scene):
        sys.exit(f"scene not found: {args.scene}\n(clone unitree_mujoco or pass --scene)")
    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)

    floor = set()
    for g in range(model.ngeom):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if model.geom_bodyid[g] == 0 or "floor" in nm.lower() or "ground" in nm.lower():
            floor.add(g)
    adr = build_index(model, mujoco)
    hand_bodies = {b for b in range(model.nbody)
                   if any(k in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "").lower()
                          for k in HAND_KEYS)}
    print(f"model: {os.path.basename(args.scene)} — {model.ngeom} geoms, "
          f"{len(hand_bodies)} hand bodies, {len(floor)} floor/world")
    if not hand_bodies:
        print("  ⚠ this model has NO hand bodies — hand collisions cannot be detected.")
    mode = "ALL self-contacts" if args.all else "HAND contacts only"
    print(f"  reporting: {mode}\n")

    def is_hand(nm):
        return any(k in nm.lower() for k in HAND_KEYS)

    # ---- validate that self-collision is detectable at all ------------------
    probe = [0.0] * 29
    # NB sign convention: NEGATIVE left shoulder-roll adducts the arm INTO the
    # torso. Positive abducts it away and collides with nothing.
    probe[JOINTS.index("left_shoulder_roll")] = math.radians(-80)
    probe[JOINTS.index("right_shoulder_roll")] = math.radians(80)
    set_pose(model, data, probe, adr)
    probe_hits = contacts(model, data, mujoco, floor)
    if not probe_hits:
        print("\n⚠ SELF-TEST FAILED: a deliberately self-intersecting pose produced no")
        print("  contacts. This model excludes self-collision pairs, so a clean result")
        print("  below would be meaningless. Enable self-collision before trusting this.")
        return 3
    print(f"self-test OK: forced arm-into-torso pose reports {len(probe_hits)} contact(s)\n")

    def report(q, label) -> int:
        set_pose(model, data, q, adr)
        hits = contacts(model, data, mujoco, floor)
        if not args.all:
            hits = [h for h in hits if is_hand(h[0]) or is_hand(h[1])]
        if not hits:
            print(f"  {label}: clean (no hand contact)" if not args.all else f"  {label}: clean")
        else:
            print(f"  {label}: {len(hits)} self-contact(s)")
            bodies = {}
            for a, b, d in hits:
                key = tuple(sorted((a.split("/")[0], b.split("/")[0])))
                bodies[key] = min(bodies.get(key, 0.0), d)
            for (b1, b2), d in sorted(bodies.items(), key=lambda kv: kv[1]):
                print(f"      {b1:<26} <-> {b2:<26} max penetration {-d*1000:5.1f} mm")
        return len(hits)

    if args.pose:
        print(f"pose: {args.pose}")
        return 1 if report(load_pose(args.pose), "static") else 0

    if not (args.src and args.dst):
        sys.exit("give --pose, or both --from and --to")

    a, b = load_pose(args.src), load_pose(args.dst)
    print(f"path: {os.path.basename(args.src)} -> {os.path.basename(args.dst)}, "
          f"{args.steps} steps (linear joint interpolation)")
    bad = []
    for k in range(args.steps + 1):
        t = k / args.steps
        q = [a[i] + t * (b[i] - a[i]) for i in range(29)]
        set_pose(model, data, q, adr)
        hits = contacts(model, data, mujoco, floor)
        if not args.all:
            hits = [h for h in hits if is_hand(h[0]) or is_hand(h[1])]
        if hits:
            bad.append((t, hits))
    if not bad:
        print(f"  clean at all {args.steps + 1} samples")
        return 0
    print(f"  ⚠ self-collision at {len(bad)}/{args.steps + 1} samples")
    first, last = bad[0][0], bad[-1][0]
    print(f"  first at t={first:.2f}, last at t={last:.2f}")
    pairs = {}
    for t, hits in bad:
        for a_, b_, d in hits:
            pairs.setdefault((a_, b_), []).append(t)
    print("  colliding pairs:")
    for (a_, b_), ts in sorted(pairs.items(), key=lambda kv: -len(kv[1]))[:8]:
        print(f"      {a_}  <->  {b_}   t={min(ts):.2f}..{max(ts):.2f}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
