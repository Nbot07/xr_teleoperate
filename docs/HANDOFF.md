# Session handoff — gantry-free G1 teleoperation

Written 2026-08-10 to carry context to a second machine. Split:

* **dev machine (GPU)** — policy training
* **WSL laptop** — sim2real verification against the real robot (it is the one
  with the robot network, certs, and the onboard Jetson deployment)

Nothing here is speculative; every claim below was measured on the robot or in
code, and the dead ends are recorded so they are not repeated.

---

## 1. Branch map (all pushed to `origin`)

| branch | contains |
| :-- | :-- |
| `hand-arm-walk-nod` | PR #322 — locomotion, double-nod walk, inspire grasp, FSM 1→4→501 fix |
| `onboard-deployment` | PR #324 — BLAS thread cap, `max_iter` 15, `TELEOP_LOG_LEVEL`, onboard docs |
| `jetson-integration` | local merge of the two, for running on the robot |
| **`teleop-safety`** | **everything current** — the above plus the safety framework, the captured pose, tests and FSM tooling |

Work from `teleop-safety`.

---

## 2. The decision that shapes everything

**Train a policy for squat↔stand.** Reached after measuring the firmware
alternative on-robot and rejecting it:

* the **ankles are unpowered** in the FSM 4 stand
* the FSM 2 squat is inferior to the hand-made pose
* **none of FSM 2, FSM 4, or the motion between them actively balances** — the
  robot tipped during a stand, caught by the hoist

An IMU pitch swing of −38°→0° with gyro 1.19 after FSM 4 *looked* like active
balancing in the logs. It was the tip and the strap. Do not re-read those logs
as evidence of a balancing controller.

---

## 3. Firmware ground truth (G1 sw 1.5.3)

**`SetFsmId`'s return code is meaningless.** It returned `0` for: FSM 2 from
zero-torque (ignored), 706 twice (ignored), and a command issued to 29 disabled
motors. **Only ever believe `GetFsmId`.**

| transition | result |
| :-- | :-- |
| `0 → 1` damp | works |
| `1 → 2` squat | works, smooth ~5.5 s |
| `2 → 4` stand | works, ~5 s trajectory (ankles unpowered) |
| `2 → 706` | **accepted and ignored**, suspended *and* with feet loaded |
| `0 → 2` | **rejected** — needs damp first |

706 is not the squat-to-stand id on this firmware; **4** is the stand id.

**Motor faults.** `motorstate` bit 9 (`512`) = winding overheat. When
`waist_pitch` (joint 14) tripped it, **all 29 motors were disabled** while
`SetFsmId` kept returning 0. It reads only ~45 °C afterwards — the fault latches
and the joint cools, so temperature alone will not tell you. Cleared by a robot
restart. Motors report `mode 0` disabled / `mode 1` live; check before trusting
any transition.

`tools/fsm_sequence.py` encodes all of this: it believes only `GetFsmId`,
refuses to send to disabled motors, polls fault words at 5 Hz, and classifies
each step as EXECUTED / ACCEPTED AND IGNORED / FSM CHANGED BUT NO MOTION /
MOVED BUT FSM UNCHANGED.

---

## 4. Leg control: low-level is the only path

| topic | reaches | balances? | precondition |
| :-- | :-- | :-- | :-- |
| `rt/arm_sdk` | arms only (weight channel at motor index 29) | yes | none |
| `rt/lowcmd` | **all 29 joints** | **no** | `MotionSwitcherClient.ReleaseMode()` |

`ReleaseMode()` stops the high-level controller. The repo already wraps this as
`MotionSwitcher.Enter_Debug_Mode()`. After it, the robot is limp unless
`rt/lowcmd` is streamed — which is why the ground pose must be statically
stable.

SDK reference gains: legs `Kp 60,60,60,100,40,40` / `Kd 1,1,1,2,1,1`; waist
`60,40,40`; arms `40` / `Kd 1`. `LowState_` has **no foot-force sensor** — only
`imu_state`, `motor_state`, `wireless_remote` — so weight-on-feet (gantry
detection) must be inferred from `tau_est`.

---

## 5. The target pose

`teleop/safety/poses/safety_squat.yaml` — captured from the robot posed by hand,
statically stable with no active balancing.

| | |
| :-- | :-- |
| hips | −144.8° |
| knees | +166.4° |
| ankles | −49.8° |
| waist_pitch | +28.6° (IMU confirms +30.0°) |

Symmetry within 0.7°, encoder noise ≤0.005°. Captured with
`tools/capture_pose.py` (read-only; one named joint per line in degrees so a
single joint can be nudged without disturbing the rest).

**Two caveats:**

1. **The hands-down contact is incidental, not structural** — stability comes
   from feet + shins/knees. So the arms can be pinned to a fixed pose and left
   out of the action space. The saved file has the hands down, so **its arm
   angles are not the training target**. Re-pose the arms tucked/at-sides,
   confirm the kneel still holds, and re-capture before training.
2. **The pose does not hold its shape when hoisted** — it needs ground contact.
   It cannot be commanded in mid-air without low-level control.

---

## 6. Training: what is already here, and what is missing

`~/unitree_rl_gym` and `~/isaacgym` are cloned on the laptop (clone them on the
dev machine too). What ships:

* `deploy/pre_train/g1/motion.pt` — a pre-trained G1 policy
* `deploy/deploy_mujoco` and `deploy/deploy_real`
* `deploy_real.py` publishes to `rt/lowcmd` — **the same path established above**,
  so the deployment plumbing already matches what the robot requires

**What does not fit:** the G1 env is `num_actions = 12`, legs only, nominal knee
`+0.3 rad` — a flat-ground velocity-tracking walk policy. The squat↔stand
transition needs roughly **legs + `waist_pitch` (~13 DoF)**, a contact schedule
(shins lift off as the robot rises), and a reward that cares about a specific
terminal pose rather than a commanded velocity.

Because the hands are incidental, this is a **modest extension of the existing
env**, not a whole-body arm-loaded push-off. That was the single largest scoping
question and it resolved favourably.

---

## 7. Verification path (laptop side)

Already built and green:

* `teleop/safety/tests/test_deadman.py` — 7/7
* `teleop/safety/tests/test_scenarios.py` — 9/9, asserts a safe stop descends
  before de-energizing
* `teleop/safety/tests/sim_keydrive.py` — 11/11 against unitree_mujoco

Two environment workarounds are needed for the mujoco sim and are documented in
`docs/SAFETY_FRAMEWORK.md` §10: `mujoco.viewer.launch_passive` hangs under WSLg
(use `tools/mujoco_headless_sim.py`), and local DDS discovery between two
processes needs a `localhost` peer.

**A mujoco sim pass cannot validate any safety action** — unitree_mujoco bridges
low-level DDS only and implements no `LocoClient` services. Actions are covered
by `test_scenarios.py` against a fake client, and remain unvalidated on hardware.

---

## 8. Do not redo these

* **CPU affinity for latency.** Pinning the whole teleop tree made it *worse*
  (IK 60 → 95 ms). The real fix was `OMP_NUM_THREADS=1`; OpenBLAS was running 35
  threads at ~392 % CPU on a 14-DoF problem. Already in `teleop_hand_and_arm.py`.
* **`ipopt.hessian_approximation: limited-memory`.** 12–15× *slower* here.
* **The original `fsm_probe.py` plan.** It tested unknown FSM ids from a
  standing robot. `tools/fsm_sequence.py` replaces it.
