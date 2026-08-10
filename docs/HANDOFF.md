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
   from feet + shins/knees. The arms still have to *finish* in this pose, but
   they carry no load, so they can follow a deterministic trajectory and stay
   out of the policy's action space. That keeps it at **legs + `waist_pitch`,
   ~13 DoF**, and makes self-collision an offline geometry problem rather than a
   reward term. The policy still has to reject the inertial disturbance the arm
   motion causes, so the arm trajectory should be deterministic and observable.
2. **The pose does not hold its shape when hoisted** — it needs ground contact.
   It cannot be commanded in mid-air without low-level control.

### Self-collision: only the hands matter

Arms may touch the chest and the legs fold onto each other — a deep kneel puts
calf on thigh, which is real contact, not a modelling error. The only collision
worth preventing is one that could damage the hands.

`tools/check_self_collision.py` checks a pose, or a joint-space path between two
poses, and reports hand contacts by body pair with penetration depth.

**Result on the captured pose:** only `left_hand_thumb_1/2` touch `left_knee`,
by **2.7 mm and 1.0 mm**. Right hand clear. Minimum single-joint fixes:

| joint | change that clears it |
| :-- | --: |
| `left_shoulder_pitch` | **−1.0°** |
| `left_shoulder_yaw` | +2.0° |
| `left_wrist_roll` | −2.5° |
| `left_wrist_pitch` | +2.5° |

2.7 mm is at the edge of what mesh approximations resolve, and in the photos the
hands rest on the mat *beside* the knees — so this may be an artifact. Clear it
because it is nearly free, not because the pose needs redesigning.

**Two traps, both of which produced confidently wrong answers first:**

* **`unitree_mujoco`'s G1 scene has NO hands.** Its chain ends at
  `wrist_yaw_link`. It reports 11–30 mm "knee ↔ wrist" penetrations that are the
  wrist *stub*, while the actual Inspire hands are invisible to it. Use
  **`assets/g1/g1_body29_hand14.xml`** — in this repo, already used by
  `robot_arm_ik.py`, with real fingers (14 hand bodies, 43 actuators).
* **That model interleaves the finger joints**: left hand at `qpos` 29–35, right
  at 43–49, *after* each arm rather than appended. Positional indexing silently
  writes arm values into finger joints. **Map by joint name.**

The checker self-tests by forcing a known-colliding pose and refuses to report
if that produces no contacts, because many robot models exclude self-collision
pairs and would make every pose look clean. Worth keeping: the first probe used
a *positive* shoulder-roll, which abducts the arm away from the torso and hits
nothing, yielding a false "self-collision is disabled" verdict. Negative left
shoulder-roll adducts.

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

Concrete implications for the env:

* **Action space:** the 12 leg joints plus `waist_pitch`. Arms follow a scripted
  trajectory to the target pose; put its phase in the observation so the policy
  can anticipate the inertial disturbance rather than only reacting to it.
* **Contact schedule:** shins lift off as the robot rises. Simpler than a
  hands-releasing schedule, but still a contact change the reward must handle.
* **Terminal pose, not commanded velocity.** The shipped config rewards tracking
  a velocity command; this task rewards arriving at and holding a specific joint
  configuration. Expect to replace most of `rewards.scales`.
* **Collision penalties must not fight the pose.** Calf-on-thigh and arm-on-chest
  are wanted. Penalise hand contacts only — the same filter
  `tools/check_self_collision.py` applies.
* **Validate the arm trajectory offline** with `--from`/`--to` before training,
  so the policy never has to learn around a path that damages the hands.

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
