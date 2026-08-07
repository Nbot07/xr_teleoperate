# Gantry-Free Safety Framework for XR Teleoperation
### Design document — v0.1 (targets `Nbot07/xr_teleoperate`, Unitree G1, Quest 3 hand tracking)

## 1. Problem statement and scope

`xr_teleoperate` today assumes the robot is *already* standing in main operation
control (entered manually with the RC: damp → stand → start), and in hand-tracking
mode it provides **no** locomotion, height, e-stop, battery, or fall handling at
all — those exist only on the controller path (`left/right thumbstick` → Damp/Move).
Shutdown sends the arms home and exits; it never de-energizes the robot safely.

A G1 without a gantry is an **actively stabilized** machine: it requires continuous
control power to remain upright and collapses if that control is removed. This is
precisely the hazard class the in-development ISO 25785-1 standard ("industrial
mobile robots with actively controlled stability") was created for, and the reason
"cut power = safe" — the reflex inherited from arms and AMRs — is *inverted* here.

**Core principle of this framework:** for a dynamically stabilized robot,
*the safe stop is a controlled descent, not a torque cut.* Every path from any
energized state to any de-energized state must route through the lowest-energy
pose reachable from the current state. Instant torque removal is reserved for the
two cases where it is already the least-bad option: the robot is already low, or
the robot is already falling.

Scope of v0.1: G1 (29-DoF), hand-tracking input, `--motion` mode, no gantry.
The seams for other robots and for autonomous "AI actions" are defined in §8.

## 2. Robot state lattice

Physical/energetic states the framework recognizes (estimated in
`safety/estimator.py` from `rt/lowstate` IMU + joint angles, fused with the
firmware FSM id polled via `LocoClient.GetFsmId()`):

```
                    POWERED_OFF (any pose on ground)
                         │ power switch (operator SOP, outside software authority)
                         ▼
   ┌─────────────  ENERGIZED, FSM 0/1 (zero-torque / damp)  ─────────────┐
   │   LYING_FRONT   LYING_BACK   LYING_SIDE   SITTING_GROUND   SQUAT    │
   └───────┬────────────┬───────────────────────────┬────────────┬───────┘
           │ FSM 702 (lie2standup)                  │            │ FSM 706
           ▼                                        ▼            ▼ (squat2stand)
        [get-up trajectory]  ──────────────►  STAND_LOCK (FSM 4, position hold)
                                                     │  Start (FSM 500/501/200*)
                                                     ▼
                                              BALANCING (main operation ctrl)
                                              ├─ height ∈ [h_min, h_max] via
                                              │  SetStandHeight (slewed)
                                              ├─ TELEOP: arms via rt/arm_sdk,
                                              │  loco via SetVelocity
                                              └─ SAFE_SQUAT = BALANCING at h_min
                                                     │ Damp (short settle)
                                                     ▼
                                               DAMPED, low  ──► ZERO_TORQUE ──► off
   FALLING (reflex state, any time in BALANCING)  ──► DAMP immediately ──► FALLEN
   FALLEN (latched; recovery only via explicit user-confirmed get-up workflow)
```

\* Firmware-dependent — and now partly resolved on-robot: on **sw 1.5.3**,
PR #322's `LocoClientWrapper._enter_control_mode` verified that `SetFsmId(500)`
returns success from the R2+A Running mode (FSM 802) **but the robot never
leaves 802**; the walkable state is **FSM 501**, reached via
damp(1) → stand(4) → *retried* 501 (the balance id is silently ignored while
the FSM-4 stand-up is still physically executing). The framework's
`EnterTeleopBalance` implements exactly that sequence, non-blocking and
confirmed. For reference, `unitree_sdk2_python` maps `Start()` → FSM **500**
(1-DoF-waist locomotion policy); FSM **501** is the 3-DoF-waist variant; older
firmware used **200**. FSM 2 ("squat") and 4 ("stand lock") semantics, and
whether 706 is bidirectional (the SDK maps both `Squat2StandUp` and
`StandUp2Squat` to 706), **must be verified per-firmware with `tools/fsm_probe.py`**
before the framework is trusted. Verified results are written to
`teleop/safety/fsm_verified.json` and loaded by `config.py`.

**Why the safety pose is "balancing at minimum height," not a firmware squat FSM.**
Two candidate safe poses exist: (a) the position-controlled squat FSM, (b) main
operation control slewed to its minimum stand height. We default to (b) → then
Damp, because (b) is reachable by a *continuous, interruptible* action from any
balancing state, needs no unverified FSM semantics, keeps the balance controller
active until the last moment, and leaves only centimeters of settle when torque
is finally removed. If the probe verifies the squat FSM behaves well on your
firmware, flip `SafetyConfig.use_fsm_squat_for_safe_pose = True` to add it as the
terminal step. The safe pose doubles as the **critical-battery destination**: the
robot never rides a dying battery in a tall stand.

## 3. Architecture

```
                                    ┌──────────────────────────────┐
 Quest 3 ── televuer ── TeleData ──►│  SafetyShim (integration.py) │──► arms_enabled /
   (head pose, 25-pt hands,         │   • feeds XR watchdog        │    height status →
    pinch bool/value)               │   • HeightGestureController  │    existing teleop loop
                                    └──────────────┬───────────────┘
 keyboard / ipc  ──► UserAuthority ◄───────────────┤
   (pause, stop, e-damp,            ┌──────────────▼───────────────┐
    decision answers)               │  SafetySupervisor (50 Hz)    │──► LocoClient
                                    │   • watchdogs (XR, lowstate) │    (SetStandHeight,
 rt/lowstate ──► RobotState        │   • battery ladder           │     SetFsmId, Damp,
 rt/*bmsstate ──► Estimator ──────►│   • falling reflex           │     ZeroTorque, ...)
 GetFsmId poll ──┘                  │   • gates (teleop / height)  │
                                    │   • ActionExecutor (ticked)  │──► Notifier (console
                                    └──────────────────────────────┘     now, XR overlay next)
```

Design rules:

1. **One authority object, checked everywhere.** `UserAuthority` is the single
   source of pause/stop/e-damp truth. The teleop loop, the height controller,
   the supervisor, and every `SafeAction.tick()` consult it. Nothing in the
   system can run while `paused` except the balance controller itself and
   safety-escalating actions.
2. **One action pipeline for everything.** Teleoperated behaviors, supervisor-
   initiated safety behaviors, and (later) AI-proposed behaviors are all
   `SafeAction`s executed by the same `ActionExecutor` with the same
   precondition checks, invariant monitoring, pause checkpoints, and
   abort-to-safest path. Priority: `REFLEX > SAFETY > USER > AI_PROPOSED`.
   A lower-priority action is preempted via its `abort()` which must leave the
   robot in a stable state.
3. **Actions are tick-based state machines, not blocking scripts.** The
   supervisor ticks the active action at 50 Hz, so pause/override/preemption
   latency is one tick, and a hung RPC can never freeze safety monitoring.
4. **Pause has honest semantics.** Some motions cannot stop mid-way safely
   (e.g., mid get-up). Each action phase declares `pausable`; a pause request
   during a non-pausable phase is latched and honored at the next checkpoint,
   and the user is told "pausing at next safe point." A pause request never
   silently disappears.
5. **The supervisor may only be overridden downward.** User commands that
   reduce energy (pause, squat, damp) are always accepted, from any state.
   User commands that *increase* risk (stand up, raise height, resume teleop)
   pass through gates and can be denied during WARNING/CRITICAL. There is a
   two-step informed-override for edge cases, and it is logged.

## 4. Escalation ladder

| Level | Example triggers (defaults in `config.py`) | Behavior |
|---|---|---|
| NOMINAL | — | Everything permitted. |
| ADVISORY | SoC < 30 %; hand tracking dropped < 2 s; IK error spike | Toast notification. No restrictions. |
| WARNING | SoC < 15 %; XR data stale > 2 s; repeated FSM RPC failures; tilt excursions | Modal decision with **deadline + default**: "Battery 14 %. Descending to safe squat in 60 s. [continue / squat now / snooze 2 min]". Height-raise and get-up gates close. Teleop continues. |
| CRITICAL | SoC < 8 %; lowstate stale > 0.5 s; supply sag; temp limit | Auto-executes `SafeSquat` → `Damp` **now**, announced, not vetoable. User may still choose *how* (e.g., "hold 5 s" grace once) but not *whether*. |
| REFLEX | tilt > 35–40° or gyro spike while BALANCING (falling) | Immediate `Damp()` (torque cut mid-fall reduces impact/gear damage; the balance controller has already lost). Latch **FALLEN**; recovery requires explicit user-initiated, user-confirmed get-up. |

Notification-before-action is the default; the ladder is exactly the user-facing
contract requested: *notify → user decides → deadline default → auto-act only
when the physics no longer waits.*

## 5. De-energize pipeline

Every software path that ends in torque removal goes through
`ControlledDeenergize`:

```
BALANCING ──slew height→h_min──► SAFE_SQUAT ──Damp──► settle ──ZeroTorque──►
   "✅ Safe to power off" prompt
LYING/SITTING/SQUAT (already low) ──Damp──► ZeroTorque directly
FALLING ──Damp──► (skip everything; already the safest available act)
```

`ZeroTorque` is only ever issued when posture is low **and** angular rate is
small. The `finally:` block of the teleop entrypoint calls
`shim.controlled_shutdown()` *before* the arms-go-home routine, so Ctrl-C, `q`,
and crashes all route through the pipeline. The physical power switch and the RC
remain the true Category-0 stop and are outside software authority — the
framework's job is to make sure you never *need* them from a tall stand.

## 6. Waist/stand-height control, hands only

Requirement: intuitive height adjustment with the Quest 3 using hand tracking
only, without colliding with manipulation gestures. Design (in
`safety/height.py` + `safety/gestures.py`):

* **Mode gate (deliberate, low false-positive):** hold a **left-hand fist for
  0.8 s** (all four non-thumb fingertips near the palm — computed from the
  25-point landmark set televuer already streams) to toggle **height mode**.
  Keyboard `h` is the fallback. On entry, **arm teleop freezes** (targets
  latched — the arm controller's internal publisher keeps streaming the last
  setpoint), which both prevents gesture/manipulation cross-talk and makes the
  mode unmistakable in the headset.
* **Fine adjust — pinch elevator:** in height mode, **right-hand pinch** (the
  `pinch` boolean televuer computes) anchors the wrist height; vertical wrist
  displacement while pinched commands `Δh = 0.6 × Δz_wrist`, clamped and
  slew-limited (≤ 5 cm/s default). Release, re-pinch to re-anchor — like
  scrolling. Left pinch works identically.
* **Embodied adjust — head follow:** pinch **both** hands in height mode and
  the robot's stand height tracks your **headset Δheight** (deadband 3 cm,
  gain 1.0, same slew/clamps): squat and the robot squats with you. Release to
  hold.
* **Exit:** left fist again, or auto-exit after 10 s idle / 2 s tracking loss.
  Arms stay frozen until you re-engage with the existing start mechanism, so
  the robot never jumps to wherever your hands drifted.
* **Gating:** every height command passes `supervisor.allow_height_change()`.
  Raising is denied at WARNING+, and the permissible ceiling shrinks as SoC
  drops, so the robot trends downward as the battery does.
* **Limits:** until `fsm_probe.py` has measured the firmware's accepted
  `SetStandHeight` range (it sweeps and reads back `GetStandHeight`), commands
  are confined to ±12 cm around the height observed at startup.

## 7. Watchdogs and comms honesty

* **XR watchdog:** `motion_data_ready` + data-freshness; stale > 0.5 s freezes
  arms (targets hold), > 2 s raises WARNING with the safe-squat countdown.
  Quest hand tracking drops whenever hands leave the camera FOV — this is the
  most frequent real-world event and gets first-class treatment.
* **lowstate watchdog:** if state telemetry stops, we are blind; supervisor
  immediately attempts `Damp()` (best effort — if DDS is truly dead nothing
  software-side can act, which is exactly why the RC stays on your belt).
* **Control-loop watchdog:** supervisor monitors its own tick jitter and the
  teleop loop's feed rate; sustained overruns escalate.

## 8. Extensibility seams

* **Robot adapter boundary.** Everything G1-specific lives behind three
  surfaces: the FSM verb table (`config.FsmTable`), the posture classifier
  (`estimator.py` thresholds + joint indices), and the loco RPC wrapper inside
  the supervisor. A new robot supplies those three; the supervisor, authority,
  ladder, executor, gestures, and height logic are robot-agnostic. Capability
  flags (`has_get_up_front/back`, `continuous_height`, `has_firmware_squat`)
  let the planner degrade gracefully — a robot with no prone get-up simply
  latches FALLEN→"await human assist" instead.
* **AI actions ride the same rails.** "Sit in chair," "pick up dry-erase
  marker," etc. are implemented as `SafeAction` subclasses submitted at
  `AI_PROPOSED` priority: they declare preconditions (posture, battery floor,
  clearance confirmation), run under the same invariant monitors, pause at
  declared checkpoints, and inherit abort-to-safest. The user's pause/stop
  gestures work identically on an autonomous action and a teleoperated one —
  which is the property that makes gradually mixing autonomy in safe.
* **XR notifier.** `Notifier` is a protocol; `ConsoleNotifier` ships now, and
  an overlay implementation can render the same toasts/modals/height gauge in
  the headset via televuer/Vuer scene UI without touching supervisor logic.

## 9. Relationship to emerging functional-safety work

The framework is deliberately shaped around the concepts the humanoid safety
community is converging on: ISO 25785-1 (draft; "actively controlled stability,"
loss-of-power collapse as a first-class hazard, behavior-on-instability
requirements, fall zones), the stability-strategy/fall-management literature it
draws on, and the bridge posture practitioners use today (ISO 10218:2025 +
ISO 13849-1 risk assessment applied with documented engineering judgment).
Concretely: the safe-squat-first stop is a "controlled stop maintaining
stability"; the escalation ladder is a documented risk-reduction hierarchy; the
FALLEN latch + confirmed recovery matches post-fall protection guidance; and the
verified-vs-assumed FSM table is the kind of validation evidence a conformity
argument eventually wants. **Honesty note:** this is an engineering-quality
supervisory layer, not a certified safety function — there is no safety-rated
channel to the actuators. Keep the RC within reach, mark a fall zone (≈ robot
height + reach in all directions) during standing tests, and treat the software
ladder as risk *reduction*, not risk *elimination*.

## 10. Bring-up and validation plan

1. **Sim first:** run the framework against `unitree_mujoco` / your Isaac
   `--sim` path (DDS domain 1). Verify: gesture toggling, slewed height,
   ladder timing with a mocked BMS, pause/abort at every phase of every action.

   > ⚠️ **What a mujoco pass can and cannot cover.** `unitree_mujoco` bridges
   > low-level DDS only — its README states it "only supports low-level
   > development", and it publishes `LowCmd`/`LowState`/`SportModeState`/
   > `IMUState` with **no `LocoClient` services**. Independently, this
   > framework sets `_loco = None` whenever `simulation_mode=True`. So a sim
   > pass validates DDS plumbing, arm/IK, loop integration, key routing and
   > gesture arbitration — but **executes no FSM transition, no
   > `SetStandHeight`, and no damp**. Every action that commands the robot is
   > covered instead by `teleop/safety/tests/test_scenarios.py`, which drives
   > them against a recording fake client and asserts command *order*.
   >
   > Two practical notes from running this (Ubuntu 20.04 under WSL2):
   > `mujoco.viewer.launch_passive` hangs under WSLg, and `unitree_mujoco.py`
   > gates its whole loop on `while viewer.is_running():` — so nothing is ever
   > published. Model loading and physics are fine headless, so run the bridge
   > without a viewer. And DDS discovery between two local processes needs a
   > `<Peer address="localhost"/>`: `lo` is not multicast-capable, and on this
   > WSL setup even locally-originated multicast is not delivered back.

   Verified this way (11/11): teleop reaches the main loop against the
   simulator, the arm controller subscribes to simulated lowstate, the
   supervisor comes online announcing `FSM table UNVERIFIED`, and
   `r`/`p`/`h`/`e`/`x`/`q` all route through the real `sshkeyboard` path —
   including `e` arming rather than executing on a single press, and `q`
   running the controlled de-energize.
2. **Probe on hardware, robot on a mat, lying down:** `python tools/fsm_probe.py
   --network-interface <if>`. It walks candidate FSM ids one at a time with
   typed confirmation, records posture before/after from the estimator, sweeps
   the accepted stand-height range, and writes `fsm_verified.json`. Nothing
   else should be trusted before this exists.
3. **Ground-level rehearsal:** damp/zero-torque transitions, get-up (702) with
   a spotter, safe-squat→damp settle distance measurement.
4. **First stands:** fall zone marked, mats, spotter, battery > 50 %. Exercise
   the WARNING countdown by lowering thresholds temporarily. Only then teleop.
5. **Regression habit:** re-run the probe after every firmware update — FSM id
   semantics have already changed across firmware versions in the wild.

## 11. Verified vs. assumed (as shipped)

| Item | Status |
|---|---|
| FSM 0=zero-torque, 1=damp, 3=sit, 702=lie2standup, 706=squat2standup; `SetStandHeight`/`GetStandHeight`; `Start()`→500 | **Verified against `unitree_sdk2_python` source** |
| Walkable state on sw 1.5.3 is **FSM 501** via damp(1)→stand(4)→retry 501; FSM 500 accepted-but-ignored from 802; FSM 4 = physical multi-second stand-up during which 501 is silently dropped | **Verified on-robot** (PR #322, `_enter_control_mode`) |
| Damp→StandUp→Start sequencing required; arm_sdk active only after stand; 500 vs 501 waist-DoF split | Community-documented; matches the on-robot finding above |
| FSM 2 (squat), 4 (stand-lock) semantics; 706 bidirectionality; stand-height range in meters; BMS topic name (`rt/lf/bmsstate` vs `rt/bmsstate`); IMU lying-pose sign conventions | **Assumed — probe verifies**, estimator/config carry `VERIFY` markers |
| Quest hand-landmark ordering (WebXR 25-pt, tips at 4/9/14/19/24) | Standard televuer/Vuer convention; sanity-checked live by the probe's gesture test |


## 12. Integration with the merged base (PR #322 + PR #324)

The framework now targets the merge of **hand-arm-walk-nod** (#322) and
**onboard-deployment** (#324). What each brings, and how the framework
composes with it:

**From #322 (nod-toggled walking).** Hand mode owns a loco path: a double
head-nod toggles WALK mode, arm displacement drives `Move(vx, vy, vyaw)`
(≤0.3 m/s), and the arms freeze at the pose captured on entry. Composition
rules the shim enforces:

* Every `Move()` — hand *and* controller paths — flows through
  `filter_move()`: zeroed when paused, non-balancing, CRITICAL, or a
  safety-priority action is running; hard-clamped to `walk_vmax` otherwise.
* **Gesture arbitration.** WALK mode blocks height-mode entry (a left fist
  while steering must not hijack the arms); height mode blocks walk input
  (`walk_input_allowed=False` suppresses the nod detector and steering); the
  **raised-both-fists deadman** overrides both. The head-height requirement
  on the deadman exists so a two-handed carry at torso height can never
  false-trigger a pause mid-manipulation.
* The both-thumbstick soft e-stop is preserved with its original semantics
  (operator-initiated, energy-reducing → always accepted) but routes through
  `user_damp()` so it is logged and the state machine knows.

**Startup behavior change (deliberate).** #322's `LocoClientWrapper`
constructor damps and **stands the robot as a side effect of launching the
program**, blocking ~10–18 s. The patch adds `auto_stand=False` and moves the
identical verified sequence into `EnterTeleopBalance` ('b'), where it is
clearance-confirmed, tick-based, pausable before the stand begins, and —
critically — **refuses to damp on failure once standing** (aborting a stand
by damping means falling; it stays position-locked and tells you instead).

**Two LocoClients, on purpose.** #322's wrapper needs `SetTimeout(1e-4)` for
its 30 Hz fire-and-forget `Move()` loop; the supervisor needs ~0.3 s timeouts
for `GetFsmId`/`SetStandHeight`/`SetFsmId` round-trips. One client cannot
serve both. Two DDS requesters coexist fine; the supervisor's client is
shared with the estimator under one lock.

**From #324 (onboard/cable-free).** The control loop runs on the robot's
dual-homed onboard computer; only video and XR poses cross Wi-Fi. Safety
implications, all favorable: the keyboard map works unchanged because the
base uses `sshkeyboard` (raw stdin over SSH, no X needed); the supervisor is
a single mostly-sleeping thread and is unaffected by the BLAS `threads=1`
caps (do not re-add `taskset` — #324 measured it making IK *worse*); and a
Wi-Fi dropout now degrades the *operator's view and XR feed*, not the wired
DDS loop — which is exactly the failure the XR-staleness ladder (freeze →
WARNING countdown → safe squat) was designed for. The `--headless` run mode
changes nothing: every safety surface is reachable by key over SSH or by
gesture.

**Tooling.** #322's `fsm_explore.py` (gantry, free-form) and this package's
`tools/fsm_probe.py` (ground-start, estimator-backed, writes
`fsm_verified.json`) are complementary: explore first, then probe to persist
what your firmware actually does.
