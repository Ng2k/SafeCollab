# SafeCollab — Knowledge Base

> **Purpose.** This document is the single, exhaustive reference for SafeCollab —
> every module, configuration key, architectural decision, formula, and piece of
> software used. It is written to be the **baseline for a written manual (.docx) and
> a presentation (PowerPoint)**: each section is self-contained and can be lifted
> into slides or chapters. For a guided overview, read the [`README.md`](../README.md)
> first; this document goes deeper and wider.

**Contents**

1. [Project overview](#1-project-overview)
2. [Requirements & acceptance criteria](#2-requirements--acceptance-criteria)
3. [System architecture](#3-system-architecture)
4. [Architectural decisions (ADRs)](#4-architectural-decisions-adrs)
5. [The ISO/TS 15066 SSM safety model](#5-the-isots-15066-ssm-safety-model)
6. [Node reference](#6-node-reference)
7. [Pure-logic module reference](#7-pure-logic-module-reference)
8. [Configuration reference](#8-configuration-reference)
9. [Interface contract (topics, types, QoS)](#9-interface-contract-topics-types-qos)
10. [The robot cell & simulation](#10-the-robot-cell--simulation)
11. [Launch system](#11-launch-system)
12. [Software & libraries](#12-software--libraries)
13. [Testing strategy](#13-testing-strategy)
14. [CI/CD & release process](#14-cicd--release-process)
15. [Docker image](#15-docker-image)
16. [Demo & operations scripts](#16-demo--operations-scripts)
17. [Directory & file map](#17-directory--file-map)
18. [Glossary](#18-glossary)

---

## 1. Project overview

SafeCollab is a **perception-driven, human-aware collaborative kitting cell** that
implements **ISO/TS 15066 Speed-and-Separation Monitoring (SSM)** in simulation. It
was built for the *Smart Robotics* course of the MSc in Artificial Intelligence
Engineering.

**The scenario.** A 6-DOF **UR5e** arm runs a continuous *kitting loop*: it picks a
part from a feeder bin and drops it into a **kitting tray it shares with a human
operator**. The operator reaches into the same tray, so human and robot genuinely
co-occupy one workspace. A simulated overhead **camera** watches the cell; a
**perception** node detects the operator with classical computer vision (no ML, CPU
only) and estimates position with an explicit uncertainty **σ**. A **safety monitor**
computes the minimum human–robot separation and scales the robot's speed by zone —
**full (green) → ramped (yellow) → protective stop (red)** — with a **fail-safe
`lost` state** that stops the arm when perception is lost or stale.

**The one invariant.** Zone thresholds are **derived from an ISO/TS 15066 risk model
at start-up**, never hand-tuned. The non-negotiable core (the "never cut" line) is:
`green → yellow → red`, protective stop, clean resume on retreat, and
fail-safe-on-`lost`.

**Why it matters.** SSM is one of the four collaborative-operation modes in
ISO/TS 15066. It lets a robot and a human share space *without* a physical cage by
continuously maintaining a computed *protective separation distance* and slowing or
stopping as that distance shrinks. SafeCollab is a faithful, testable, end-to-end
demonstration of that principle, from camera pixels to a scaled joint trajectory.

---

## 2. Requirements & acceptance criteria

### Functional requirements (referenced throughout the code)

The code traces to functional requirements `FR-*` (from the engineering report). The
ones the implementation and tests reference directly:

| FR | Intent |
|---|---|
| FR-2 / FR-7 | Full speed and green zone when the operator is clear. |
| FR-6 | Speed decreases smoothly as the operator approaches (yellow ramp). |
| FR-8 | Protective stop on breach (red) and clean resume on withdrawal. |
| FR-9 | Fail-safe on lost/stale perception; **never dead-reckon** the last position. |
| FR-11 | Behaviour generalises across randomised operator paths (no path-specific tuning). |
| FR-12 | *(Cut-scope / "Could")* speed-dependent separation — fixed risk-derived thresholds suffice. |

### Acceptance tests (the definition of "works")

| ID | Scenario | Expected | Traces |
|---|---|---|---|
| AT-1 | Operator away during a full kit | Full speed; zone green | FR-2, FR-7 |
| AT-2 | Operator reaches slowly toward tray | Speed decreases smoothly; marker yellow | FR-6, FR-7 |
| AT-3 | Hand enters the shared tray (red) | Protective stop within one tick; HUD logs STOP | FR-8 |
| AT-4 | Operator withdraws | Clean resume from current state, no jerk | FR-8 |
| AT-5 | Perception loses operator (occlusion) | Enter LOST and stop (fail-safe); resume on re-acquire | FR-9 |
| AT-6 | Randomised operator paths | Behaviour generalises; no path-specific tuning | FR-11 |

The core `AT-1…AT-5` are proven for one seeded path by
`test/scenario/test_at1_at5.py`; `AT-6` and the robustness variants (`AT-3r/4r/5r`)
by `test/scenario/test_robustness.py` and the unit suite. See
[§13 Testing strategy](#13-testing-strategy).

---

## 3. System architecture

### Two loops

SafeCollab is best understood as **two loops** that meet at `motion_node`:

- **Safety loop (closed):** `perception_node → safety_monitor → /safety/scale →
  motion_node → arm → /joint_states + robot TF → safety_monitor`. This loop *is* the
  SSM guarantee.
- **Task loop (open):** `planner_node → /motion/nominal_trajectory → motion_node`. It
  proposes *what path* the arm should follow; it knows nothing about safety.

The design principle: **planning proposes the path; safety governs its speed.** Speed
scaling lives entirely in `motion_node`, so the safety behaviour is independent of how
the path was generated.

### Node graph (data flow)

```
             gz camera ──/camera/image──► perception_node
                                              │  broadcasts TF world→human (perceived)
                                              │  publishes /human/uncertainty (σ)
                                              ▼
 planner_node ──/motion/nominal_trajectory──► safety_monitor ──/safety/scale──► motion_node
   (MoveIt/Pilz)          │                    (20 Hz: min-dist                    │
   publishes /task/state  │                     → risk zone → scale)               │
                          │                        ▲                               ▼
                          │        robot TF + /joint_states               /arm_controller/
                          │                        │                       joint_trajectory
                          └────────────────────────┴───────────────► gz_ros2_control (UR5e JTC)

 human_node ── /world/empty/set_pose ──► moves the yellow operator body in gz
            └─ broadcasts TF world→human_gt (sim-internal ground truth, never used by safety)

 hud_node ◄── /safety/zone, /safety/scale, /safety/min_distance (view only)
```

### Layered structure

Each functional area is split into a **pure-logic layer** (no ROS, unit-tested,
covered by the ≥ 90 % gate) and a **thin ROS adapter** (`# pragma: no cover`) that only
wires topics/TF and delegates every decision to the logic:

| Area | Pure logic | ROS adapter |
|---|---|---|
| Safety | `safety/{risk,zone,logic,marker}.py` | `safety/node.py` |
| Motion | `motion/{retime,logic}.py` | `motion/node.py` |
| Perception | `perception/{detection,geometry,uncertainty,state}.py` | `perception/node.py` |
| Human | `human/{path,model}.py` | `human/node.py` |
| Planning | `planning/plan.py` | `planning/node.py` |
| HUD | `hud/format.py` | `hud/node.py` |
| Shared glue | — | `_ros_runtime.py` |

---

## 4. Architectural decisions (ADRs)

Each decision below states the choice, the rationale, and the consequence.

### ADR-1 — Clean architecture: pure logic vs. ROS adapters
**Choice.** Every safety-relevant decision is a pure function/class with no `rclpy`
import; ROS nodes are thin adapters (`# pragma: no cover`).
**Why.** Safety maths must be unit-testable without a simulator, so CI can enforce a
**≥ 90 % coverage gate** on the logic. `rclpy` is an apt package, not pip-installable,
so keeping it out of the logic lets unit tests run in a plain-Python venv.
**Consequence.** Fast, deterministic unit tests for the core; the live suites only
need to cover the (thin) wiring. `.coveragerc` omits the `*node.py`/`_ros_runtime.py`
adapters.

### ADR-2 — Config, not constants
**Choice.** All tunable safety/risk/motion/HUD numbers live in `config/*.yaml`, loaded
at start-up (ground rule 5). Thresholds are *computed*, never pasted into code.
**Why.** Safety parameters must be auditable and adjustable without code changes; the
worked thresholds (0.43 / 0.84 m) are a *result* of `risk.yaml`, not a magic number.
**Consequence.** Changing the cell's safety behaviour is a YAML edit; tests read the
same config so they can never drift from what runs.

### ADR-3 — Perceived distance, never dead-reckoned
**Choice.** The safety loop consumes only the perceived `world → human` TF. The
simulator's exact pose (`world → human_gt`) is sim-internal. On loss the last position
is never reused.
**Why.** A cage-free SSM cell must earn its distance from real sensing; reusing a
stale position would silently defeat the fail-safe (FR-9).
**Consequence.** Perception failure degrades to a protective stop by *omission*
(the node stops broadcasting), which is the safe default. `safety_source:=ground_truth`
exists only as a documented fallback (cut-scope #4) when perception is GPU-flaky.

### ADR-4 — Scaling lives in exactly one node
**Choice.** `motion_node` is the only node that publishes `/arm_controller/joint_trajectory`
and the only place speed is scaled.
**Why.** Auditability ("who can move the robot?") and a single, testable SSM chokepoint.
**Consequence.** The SSM guarantee is independent of the path source; swapping the arm
model changed only the trajectory *source*, not the governor.

### ADR-5 — UR5e + MoveIt/Pilz, deterministic PTP
**Choice.** A real, kinematically solved **UR5e** planned by MoveIt's **Pilz**
industrial planner (deterministic PTP), replacing an earlier hand-solved cylinder arm.
**Why.** Hand-solving poses for a custom arm was brittle and looked mechanical; a real
manipulator with a deterministic planner gives repeatable, natural motion and removes
per-geometry pose maths.
**Consequence.** Runs are repeatable (no sampling stutter). Deterministic IK is cached
per Cartesian target so the arm doesn't re-solve to different configs each cycle.

### ADR-6 — Retime, don't re-plan
**Choice.** SSM scaling stretches each waypoint's `time_from_start` by `1/scale`
(`retime`); it does not replan velocities.
**Why.** Pilz emits a dense position trajectory the controller interpolates, so
"slow down / stop" works by time-stretching alone — no velocity/acceleration
bookkeeping, and it works on any planned path.
**Consequence.** `scale == 0` → `retime` returns `None` → command nothing (a hold).
Resume re-plans from the current joint state (no splice jerk), keeping only waypoints
still ahead of the goal.

### ADR-7 — motion_node, not the UR native scaled controller
**Choice.** Route SSM through the tested `motion_node` retiming rather than UR's
`scaled_joint_trajectory_controller`.
**Why.** The native UR speed-scaling interface is driver-fed and not cleanly
commandable in gz simulation; the existing retimer preserves the AT-verified behaviour
with no dependency on a sim speed-scaling hook.
**Consequence.** SSM behaviour is identical regardless of the controller stack.

### ADR-8 — Fail-safe by omission
**Choice.** Perception signals loss by *not broadcasting*; the monitor treats a
missing/stale TF (> `loss_timeout`) as `lost` → `("lost", 0.0)`.
**Why.** Making safety the default (rather than an explicit code path that could be
skipped) is more robust.
**Consequence.** Any perception fault — crash, occlusion, dropped frames — degrades to
a protective stop.

### ADR-9 — Deploy = the container image
**Choice.** No server or registry; CI builds the runnable image and, on a version tag,
publishes it as a GitHub Release asset (`.tar.gz`) with the changelog slice.
**Why.** Reproducibility: the same image runs the integration/scenario suites and is
what ships, so "tested" and "shipped" are byte-identical.
**Consequence.** Consume with `gh release download vX.Y.Z && docker load`.

### ADR-10 — Clean shutdown via `os._exit(0)` and a benign-error predicate
**Choice.** Nodes that spin their own loop swallow one specific benign teardown error
and exit via `os._exit(0)`.
**Why.** On SIGINT, rclpy's executor can raise a pybind "Unable to convert call
argument" `RuntimeError` mid-take, and MoveItPy's C++ static-destructor teardown can
intermittently exit 1 on slow runners. `is_benign_shutdown_error()` recognises the
former; `os._exit(0)` skips the latter.
**Consequence.** Deterministic exit codes so the integration exit-code check is stable.

### ADR-11 — Frozen interface contract for parallel work
**Choice.** The topic/type/QoS interface (§9) was frozen first; streams coded against
it with stubs.
**Why.** Multiple work-streams (safety, sim, perception, motion, task, CI) proceeded in
parallel without blocking each other.
**Consequence.** The contract in AGENTS.md §3 is the coordination point; QoS is matched
per topic by `_ros_runtime`.

---

## 5. The ISO/TS 15066 SSM safety model

### 5.1 Protective separation distance `S_p`

SSM maintains a **protective separation distance** `S_p` between operator and robot.
If the measured separation drops below `S_p`, the robot must slow or stop. SafeCollab
implements the standard's sum of contributions (`safety/risk.py`):

```
S_p = S_H + S_R + S_S + C + Z_d + Z_r
    = v_h·(t_r + t_s)  +  v_r·t_r  +  s_s  +  c  +  z_d  +  z_r
```

| Term | Formula | Meaning |
|---|---|---|
| `S_H` | `v_h·(t_r + t_s)` | how far the human travels during system reaction + robot stop |
| `S_R` | `v_r·t_r` | how far the robot travels during its reaction time |
| `S_S` | `s_s` | robot stopping-distance contribution |
| `C` | `c` | intrusion distance (how far a hand reaches in) |
| `Z_d` | `z_d` | **operator position uncertainty — live from perception (σ)** |
| `Z_r` | `z_r` | robot position uncertainty |

```python
def protective_distance(v_h, t_r, t_s, v_r, s_s, c, z_d, z_r):
    s_h = v_h * (t_r + t_s)
    s_r = v_r * t_r
    return s_h + s_r + s_s + c + z_d + z_r        # ISO/TS 15066 S_p
```

### 5.2 Two threshold-generating scenarios

`safety/risk.py::thresholds(cfg, z_d)` evaluates `S_p` **twice**, once per scenario in
`risk.yaml`, sharing the cell/sensor properties but differing in robot speed `v_r` and
stop time `t_s`:

- **`d_yellow`** = `S_p` with the **full-speed** scenario (`v_r=0.50`, `t_s=0.25`) —
  the distance at which the cell **begins slowing**.
- **`d_red`** = `S_p` with the **reduced** scenario (`v_r=0.10`, `t_s=0.02`) — the
  distance at which the robot **must already be stopped**.

The two scenarios are **not** two free parameters: a robot already moving slowly has
less kinetic energy and genuinely stops faster, so `t_s` shrinks *with* `v_r`. This is
the physically correct reading of the standard.

```python
def thresholds(cfg, z_d):
    d_yellow = protective_distance(v_r=cfg.full_speed.v_r, t_s=cfg.full_speed.t_s, …, z_d=z_d, …)
    d_red    = protective_distance(v_r=cfg.reduced.v_r,   t_s=cfg.reduced.t_s,   …, z_d=z_d, …)
    return round(d_red, 2), round(d_yellow, 2)
```

### 5.3 Worked example & monotonicity property

With the `risk.yaml` inputs and `z_d = 0.05`:

```
d_yellow = 1.6·(0.10+0.25) + 0.50·0.10 + 0.06 + 0.10 + 0.05 + 0.02 = 0.84 m
d_red    = 1.6·(0.10+0.02) + 0.10·0.10 + 0.06 + 0.10 + 0.05 + 0.02 = 0.43 m
```

| `z_d` (m) | `d_red` (m) | `d_yellow` (m) |
|---|---|---|
| 0.05 (nominal) | 0.43 | 0.84 |
| 0.10 (noisier detection) | 0.48 | 0.89 |

**Property (asserted in `test_risk.py`):** raising `z_d` **widens both thresholds** —
a less certain operator position makes the cell more cautious. Because `z_d` is fed
live from perception σ, the zones "breathe" with detection quality.

### 5.4 Distance → zone → speed scale

`safety/zone.py::classify(d, d_red, d_yellow, s_min)` maps the live minimum separation
`d` to a `(zone, scale)`:

```python
def classify(d, *, d_red, d_yellow, s_min):
    if d is None:      return "lost", 0.0                  # fail-safe (FR-9)
    if d >= d_yellow:  return "green", 1.0                 # yellow edge EXCLUSIVE
    if d <= d_red:     return "red", 0.0                   # red edge INCLUSIVE
    s = s_min + (1 - s_min) * (d - d_red) / (d_yellow - d_red)
    return "yellow", s
```

| Condition | Zone | Scale |
|---|---|---|
| `d is None` | `lost` | `0.0` (fail-safe) |
| `d ≥ d_yellow` | `green` | `1.0` |
| `d_red < d < d_yellow` | `yellow` | linear ramp `s_min…1.0` |
| `d ≤ d_red` | `red` | `0.0` (protective stop) |

**Boundary semantics (locked with tests):** yellow edge exclusive (`d == d_yellow →
green`), red edge inclusive (`d == d_red → red`), `d is None → ("lost", 0.0)`. The ramp
floor `s_min = 0.10` keeps the arm creeping rather than stalling mid-band.

### 5.5 The minimum-distance sweep

`safety/logic.py::SafetyMonitorLogic._min_distance` takes the **minimum** Euclidean
distance from the perceived operator to *any* configured robot frame (`tool0`,
`wrist_3_link`, `forearm_link`). It compares squared distances and takes `sqrt` once on
the nearest (a micro-optimisation for the 20 Hz hot path). If the operator TF is
missing/stale **or** no robot frames resolve, `d` is `None` → fail-safe.

### 5.6 Speed knob: `retime`

`motion/retime.py` converts a scale into motion: it stretches each waypoint time by
`1/scale` (slower → later arrivals). `scale == 0.0` returns `None` (protective stop,
avoids div-by-zero); out-of-range scales raise `ValueError`.

---

## 6. Node reference

Six ROS 2 nodes (all `console_scripts` entry points in `setup.py`). Each is a thin
adapter delegating to pure logic.

### 6.1 `planner_node` (`planning/node.py`)
- **Role.** Owns the kitting cycle; plans each leg with MoveIt/Pilz and publishes the
  dense nominal trajectory.
- **Publishes.** `/motion/nominal_trajectory` (JointTrajectory), `/task/state` (String).
- **Subscribes.** `/joint_states` (for closed-loop pacing).
- **Key behaviour.**
  - Uses `MoveItPy` with the Pilz pipeline (config assembled by `cell.launch.py`'s
    `_moveit_params()` via `MoveItConfigsBuilder`, reshaped so MoveItCpp loads Pilz).
  - Plans each leg as a **PTP** to a cached joint goal; deterministic IK is solved
    **once** per Cartesian target and cached (KDL is non-deterministic, so re-solving
    would swing the arm and corrupt the SSM zone).
  - **Closed-loop pacing:** after publishing a leg it waits (`wait_until_reached`) until
    the arm is within tolerance of the goal, so an SSM slow-down only *delays* a pick,
    never abandons it.
  - Exits via `os._exit(0)` after swallowing the benign SIGINT teardown race (ADR-10).
- **Kitting cycle** (`planning/plan.py::kitting_legs`, alternating left/right bin):
  `GO_TO_BIN → PICK → LIFT_BIN → GO_TO_TRAY → DROP → LIFT_TRAY`, as Cartesian `tool0`
  targets: bins at world `(0.15, ±0.35)`, tray at `(0.35, 0.0)`, pick/place height
  `0.78 m`, raised height `0.98 m`. The DROP leg dwells longer so an operator tray reach
  co-occurs with the arm (drives AT-6 red escalations).

### 6.2 `human_node` (`human/node.py`)
- **Role.** Drives the simulated operator; provides ground truth (sim-internal).
- **Publishes/broadcasts.** Moves the operator body via the gz service
  `/world/empty/set_pose` (~30 Hz); broadcasts TF `world → human_gt` (~50 Hz).
- **Key behaviour.** Advances an `OperatorModel` along a randomised `OperatorPath`
  (with a guaranteed tray reach); holds the body at the constant marker height
  `_MARKER_Z = 0.95 m` (== perception's `plane_z_m`) so the recovered planar pose is
  parallax-free. `path_seed>0` makes the path reproducible for tests. The gz set_pose
  runs on a worker thread so a slow service call can't stall path advancement or TF.

### 6.3 `perception_node` (`perception/node.py`)
- **Role.** Classical CV → perceived operator pose + uncertainty.
- **Subscribes.** `/camera/image` (best-effort, depth 1).
- **Publishes/broadcasts.** `/human/uncertainty` (Float32 σ), TF `world → human`
  (perceived). Runs a **25 Hz** timer for loss-timeout + TF/σ publication.
- **Pipeline per frame.** `detect_human` (HSV blob) → `project_pixel_to_plane` (ray→
  plane at `z=0.95`) → depth = ‖world_pt − camera‖ → `estimate_uncertainty` →
  `PerceptionLogic.update`. Camera intrinsics derive from `hfov=1.5`, `640×480`; the
  camera→world transform is built from the true mast pose `(-0.45, 0, 2.40)`, pitch
  `1.2 rad`. All are ROS parameters (defaults from `cell.xacro`).
- **Fail-safe.** While `is_lost`, it stops broadcasting/publishing → the monitor sees a
  stale TF and enters `lost` (FR-9).

### 6.4 `safety_monitor` (`safety/node.py`)
- **Role.** Closes the SSM loop.
- **Subscribes.** `/human/uncertainty`; TF `world → human` (or `human_gt` if
  `safety_source:=ground_truth`); TF `world → {robot_frames}`.
- **Publishes.** `/safety/scale` (reliable), `/safety/zone` (transient_local),
  `/safety/min_distance` (best-effort), `/viz/safety_marker` (Marker: sphere id=0 +
  text label id=1).
- **Key behaviour.** 20 Hz tick: look up robot-frame and human positions → delegate to
  `SafetyMonitorLogic.compute` → publish. Staleness is checked manually against
  `loss_timeout` (rather than a blocking TF timeout) to keep the fail-safe path fast.

### 6.5 `motion_node` (`motion/node.py`)
- **Role.** The **only** arm commander; fuses nominal trajectory × scale.
- **Subscribes.** `/motion/nominal_trajectory`, `/safety/scale`, `/joint_states`.
- **Publishes.** `/arm_controller/joint_trajectory`.
- **Key behaviour.** Delegates to `MotionLogic`. A new leg → publish retimed at current
  scale. A new scale → dead-band (`republish_scale_epsilon`) drops unchanged 20 Hz
  ticks; `scale==0` → active hold; resume re-plans from current state (no backtrack).

### 6.6 `hud_node` (`hud/node.py`)
- **Role.** View-only console readout (never affects the loop).
- **Subscribes.** `/safety/zone`, `/safety/scale`, `/safety/min_distance`.
- **Behaviour.** Redraws one aligned, colour-coded line (`hud/format.format_status`) at
  `refresh_hz` (5 Hz). Shows `--` for distance when `lost` (never a stale value).

---

## 7. Pure-logic module reference

### 7.1 Safety
- **`safety/risk.py`** — `protective_distance(...)` (`S_p`), `thresholds(cfg, z_d)` →
  `(d_red, d_yellow)`, `load_config(path)` → namespace from `risk.yaml`. §5.
- **`safety/zone.py`** — `classify(d, *, d_red, d_yellow, s_min)` → `(zone, scale)`. §5.4.
- **`safety/logic.py`** — `SafetyMonitorLogic(risk_cfg, safety_cfg)`; `.compute(
  robot_frames_xyz, human_xyz_or_none, uncertainty)` → `(zone, scale, dist, marker)`;
  `_min_distance` sweep; exposes `loss_timeout`, `marker_radius`, label geometry.
- **`safety/marker.py`** — `_ZONE_RGBA` (green/yellow/red/lost colours; alpha 0.8 active,
  0.4 for lost) and `_marker_params(zone, xyz, radius, label_offset)` — single source of
  truth for zone colour, reused by sphere and label.

### 7.2 Motion
- **`motion/retime.py`** — `retime(times, scale)` (§5.6).
- **`motion/logic.py`** — `MotionLogic(republish_scale_epsilon, hold_time_s)`. State:
  scale (default **1.0**), nominal trajectory, joint positions+names, stop/resume edge
  flags, last-commanded scale. Key methods:
  - `command_for_new_leg()` — publish a fresh leg retimed at current scale (holds if
    stopped). **Because scale defaults to 1.0, a leg runs at full speed with no
    `/safety/scale` ever received** — this is what enables the "robot only" demo mode.
  - `command_for_scale(scale)` — dead-band + stop/resume; `("move"|"hold", …)` or `None`.
  - `compute_command(resuming=…)` — retimes; on resume, prepends the current state at
    `t=0` and keeps only waypoints still **ahead** of the goal (`_ahead_positions`,
    "no backtrack"). Realigns `/joint_states` (alphabetical) into the trajectory's
    group order before using it as a waypoint.
  - `_hold_command()` — single-point HOLD at current state over `hold_time_s`.

### 7.3 Perception
- **`perception/detection.py`** — `detect_human(bgr, …)` → `(u, v, area, confidence)` or
  `None`. BGR→HSV, threshold hue 20–40 / sat,val ≥100, largest contour, centroid;
  confidence = `min(1, area/conf_area_ref_px)`.
- **`perception/geometry.py`** — `_derive_intrinsics(w, h, hfov)` → `(fx, fy, cx, cy)`
  (`fx = (w/2)/tan(hfov/2)`); `cam_to_world_transform(...)` (4×4 from the mast chain:
  `R = Ry(pitch)·Rz(−π/2)·Rx(−π/2)`); `project_pixel_to_plane(u, v, intr, cam2world,
  plane_z)` → world point on `z=plane_z` (ray/plane intersection; `None` if parallel or
  behind camera). Legacy `back_project`/`transform_point` retained for the pinhole tests.
- **`perception/uncertainty.py`** — `estimate_uncertainty(area, depth, confidence, …)` =
  `base + depth·k_d + (1−conf)·k_n`, floored at `min_sigma`.
- **`perception/state.py`** — `PerceptionLogic(loss_timeout_s)`: `update(position, sigma,
  timestamp)` / `check_timeout(current_time)`; `is_lost`, `position` (None while lost),
  `sigma`. Starts `lost=True`.

### 7.4 Human
- **`human/path.py`** — cell geometry constants (tray centre `(0.35, 0, 0.76)`, footprint
  ±0.15/±0.20, `REACH_Z=0.82`, `STANDING_Z=1.10`, `APPROACH_Z=0.95`, `STANDING_DWELL_S=5`);
  `Waypoint`; `_interpolate_waypoints`; `OperatorPath` with `has_tray_reach()` and
  `generate_random(rng)` (randomises approach side ±0.65, reach position, ±40 % speed,
  and dwell — so the operator never phase-locks to the arm's period).
- **`human/model.py`** — `OperatorModel(path)`: `position_at(t)`, `is_complete(t)`,
  `distance_to_point`.

### 7.5 HUD
- **`hud/format.py`** — `format_status(zone, scale, min_distance, colours)` →
  `"SSM | RED    | speed   0% | min-dist 0.38 m"`; `build_colour_map`,
  `load_hud_config`, `sgr`.

### 7.6 Shared glue
- **`_ros_runtime.py`** — imported only inside a node's `try: import rclpy` guard.
  `reliable_qos` (RELIABLE/KEEP_LAST), `transient_qos` (RELIABLE/TRANSIENT_LOCAL,
  latched), `best_effort_qos`; `config_path(name)` (resolves `config/` under both the
  `--symlink-install` and plain-build layouts); `is_benign_shutdown_error(exc)`;
  `spin_and_shutdown(node, on_shutdown)`.

---

## 8. Configuration reference

All under `src/safecollab/config/`. Ground rule 5: **no safety-relevant constants in
code** — everything below is loaded at start-up.

### 8.1 `risk.yaml` — ISO/TS 15066 inputs
| Key | Value | Meaning |
|---|---|---|
| `v_h` | 1.6 | operator approach speed (m/s) |
| `t_r` | 0.10 | system reaction time (s) |
| `s_s` | 0.06 | robot stopping-distance contribution (m) |
| `c` | 0.10 | intrusion distance (m) |
| `z_r` | 0.02 | robot position uncertainty (m) |
| `full_speed.v_r` / `.t_s` | 0.50 / 0.25 | robot speed/stop-time at full speed → `d_yellow` |
| `reduced.v_r` / `.t_s` | 0.10 / 0.02 | robot speed/stop-time when reduced → `d_red` |
| `z_d` | — | *not stored* — supplied live by perception σ |

### 8.2 `safety.yaml` — monitor config
| Key | Value | Meaning |
|---|---|---|
| `s_min` | 0.10 | minimum speed scale in the yellow band (ramp floor) |
| `loss_timeout` | 0.5 | seconds of stale/absent human TF before `lost` |
| `robot_frames` | `tool0`, `wrist_3_link`, `forearm_link` | frames swept for min-distance |
| `marker_radius` | 0.15 | RViz zone-sphere radius (m) |
| `marker_label_offset` | 0.25 | gap sphere-top → text label (m) |
| `marker_label_height` | 0.20 | label glyph height (m) |
| `safety_source` | `perceived` | `perceived` (world→human) or `ground_truth` (world→human_gt) |

### 8.3 `motion.yaml` — motion tuning
| Key | Value | Meaning |
|---|---|---|
| `republish_scale_epsilon` | 0.02 | dead-band before re-issuing the arm command (avoids 20 Hz re-publish that resets the controller clock) |
| `hold_time_s` | 0.2 | protective-stop hold horizon (active hold, not "hold by inaction") |

### 8.4 `hud.yaml` — console HUD (view only)
| Key | Value | Meaning |
|---|---|---|
| `refresh_hz` | 5.0 | redraw rate |
| `use_colour` | true | ANSI colour the zone word |
| `colours` | green 32 / yellow 33 / red 31 / lost 90 | zone → SGR code |

### 8.5 `controllers.yaml` — ros2_control
`controller_manager` at 100 Hz; `joint_state_broadcaster` + `arm_controller`
(`joint_trajectory_controller` over the six UR joints: `shoulder_pan/lift`, `elbow`,
`wrist_1/2/3`; command interface `position`, state `position`+`velocity`;
`open_loop_control: true`, `state_publish_rate 50`, `action_monitor_rate 20`).

### 8.6 MoveIt configs
- `kinematics.yaml` — IK solver for the `ur_manipulator` group.
- `joint_limits.yaml` — MoveIt joint limits (velocity/accel bounds for planning).
- `pilz_cartesian_limits.yaml` — Pilz Cartesian velocity/accel limits.
- `pilz_industrial_motion_planner_planning.yaml` — Pilz pipeline params.
- `view.rviz` — RViz layout: RobotModel (topic `/robot_description`, transient_local),
  TF, Camera, and the `SafetyZone` Marker (`/viz/safety_marker`); Fixed Frame `world`.

---

## 9. Interface contract (topics, types, QoS)

Frozen first (AGENTS.md §3). QoS matched per topic by `_ros_runtime`.

| Topic / channel | Type | QoS | Producer → Consumer |
|---|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | best-effort | gz camera → `perception_node` |
| TF `world → human_gt` | `tf2` | — | `human_node` → (sim-internal only) |
| TF `world → human` | `tf2` | — | `perception_node` → `safety_monitor` (**perceived**) |
| `/human/uncertainty` | `std_msgs/Float32` (σ, m) | reliable | `perception_node` → `safety_monitor` |
| `/safety/scale` | `std_msgs/Float32` (0–1) | reliable | `safety_monitor` → `motion_node` |
| `/safety/zone` | `std_msgs/String` | reliable, transient_local | `safety_monitor` → HUD/viz |
| `/safety/min_distance` | `std_msgs/Float32` (m) | best-effort | `safety_monitor` → HUD/viz |
| `/viz/safety_marker` | `visualization_msgs/Marker` | best-effort | `safety_monitor` → RViz |
| `/motion/nominal_trajectory` | `trajectory_msgs/JointTrajectory` | reliable | `planner_node` → `motion_node` |
| `/task/state` | `std_msgs/String` | reliable, transient_local | `planner_node` → HUD/viz |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | controller default | `motion_node` → JTC |
| `/joint_states` | `sensor_msgs/JointState` | default | `joint_state_broadcaster` → motion/planner/safety |
| `/clock` | `rosgraph_msgs/Clock` | — | gz → all `use_sim_time` nodes (via `ros_gz_bridge`) |

---

## 10. The robot cell & simulation

### 10.1 Description — `urdf/cell.xacro`
Top-level robot description (`robot_state_publisher` + `ros_gz_sim create`). Holds:
`world` root; a **table** (0.74 m high); two **feeder bins** at world `(0.15, ±0.35)`;
a shared **kitting tray** at `(0.35, 0)`; an overhead **camera** on a 1.70 m mast
(camera_link at world `(-0.45, 0, 2.40)`, pitched 1.2 rad, 640×480, hfov 1.5,
`camera/image` @ 15 Hz); and the **UR5e** (`ur_description` `ur_robot` macro with the
`ur5e` config, mounted on `table_top` at base offset `(-0.10, 0, 0)`, `force_abs_paths`
so meshes resolve). The **`gz_ros2_control`** plugin loads `config/controllers.yaml`.
Furniture links double as MoveIt collision geometry.

### 10.2 World — `worlds/cell.sdf`
`empty.sdf` **plus the gz `Sensors` system** — without it the camera never renders and
the safety loop fail-safes to `lost`. Headless uses `--headless-rendering` (EGL
offscreen) so the camera produces frames with no X display.

### 10.3 Operator — `urdf/operator.sdf`
A static, collision-free **flat yellow puck** (0.35×0.35×0.08 m, RGB `(1,1,0)` → HSV hue
~30, inside the detector's 20–40 window). Flat + held at a constant height so the camera's
off-vertical view doesn't bias the recovered centroid. Spawned by `cell.launch.py`;
`human_node` moves it via `/world/empty/set_pose`.

### 10.4 MoveIt semantic model — `srdf/cell.srdf.xacro`
The SRDF (planning group `ur_manipulator`, `tool0` EEF, disabled collision pairs) read by
`planner_node`'s `MoveItConfigsBuilder`, together with `config/kinematics.yaml`,
`joint_limits.yaml`, and the Pilz limit files.

### 10.5 Control chain
`gz_ros2_control` starts an in-process `controller_manager`; `cell.launch.py` spawns
`joint_state_broadcaster` then `arm_controller` (ordered via `OnProcessExit`).
`motion_node` commands `/arm_controller/joint_trajectory`; the JTC interpolates the dense
waypoints and drives the UR5e in gz.

---

## 11. Launch system

`launch/cell.launch.py` is the single entry point. It brings up, in order: the gz sim
(headless or GUI), the `/clock` bridge, `robot_state_publisher` (xacro `Command`), the
model spawn, the operator spawn, the controller spawners (ordered via `OnProcessExit`),
the `ros_gz_image` camera bridge, and the application nodes. `planner_node` receives the
assembled MoveIt params from `_moveit_params()` (which reshapes `planning_pipelines` so
MoveItCpp loads Pilz).

### Launch arguments
| Arg | Default | Effect |
|---|---|---|
| `headless` | `false` | gz server-only + offscreen rendering (CI/Docker) |
| `rviz` | `false` | open RViz with `config/view.rviz` |
| `hud` | `false` | run the console HUD node |
| `human` | `true` | spawn operator body + `human_node` (`IfCondition`) |
| `safety` | `true` | run `perception_node` + `safety_monitor` (`IfCondition`) |
| `safety_source` | `perceived` | `perceived` (world→human) or `ground_truth` (world→human_gt) |
| `path_seed` | `0` | operator-path RNG seed (`0` non-deterministic; `>0` reproducible) |

**Mode combinations** used by the demo scripts:
- `human:=false safety:=false` → robot-only: no operator, no SSM; `motion_node` runs
  each leg at scale 1.0 (default). Full pick-and-place.
- `human:=true safety:=true` (defaults) → full SSM cell.

---

## 12. Software & libraries

| Layer | Component | Role |
|---|---|---|
| OS / base | Ubuntu 24.04 (via `ros:jazzy-*` images) | runtime base |
| Middleware | **ROS 2 Jazzy** — `rclpy`, `tf2_ros`, `launch`, `launch_ros` | nodes, TF, launch |
| Simulator | **Gazebo Harmonic** — `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image` | physics, sensors, ROS↔gz bridges |
| Control | `ros2_control`, `ros2_controllers`, `gz_ros2_control`, `joint_trajectory_controller` | controller manager + arm JTC in gz |
| Planning | **MoveIt 2** (`moveit`, `moveit_py`), **Pilz** industrial motion planner, `moveit_configs_utils` | deterministic PTP planning |
| Robot model | `ur_description`, `ur_simulation_gz`, `ur_moveit_config` | UR5e model, gz wiring, MoveIt config |
| Perception | **OpenCV** (`python3-opencv`, `cv2`), `cv_bridge`, **NumPy** | blob detection, geometry |
| Config | **PyYAML** | load `config/*.yaml` |
| Testing | `pytest`, `pytest-cov`, `launch_testing`, `launch_testing_ros` | unit / integration / scenario |
| Lint | `ruff` (0.8.6), `yamllint` (1.35.1) | Python + YAML lint/format |
| Packaging | **Docker** (multi-stage, BuildKit cache mounts) | the deploy artifact |
| CI/CD | **GitHub Actions** (`.github/workflows/ci.yml`) | gated pipeline + release |

Design constraints on the stack: **no ML, CPU-only perception**; `rclpy` is apt-only, so
pure logic stays importable without ROS; every added Python dep goes in the root
`requirements.txt`.

---

## 13. Testing strategy

Three layers under `src/safecollab/test/`, each verifying a property where it is cheapest
and most deterministic.

### 13.1 Unit (`test/unit/`) — the TDD core + coverage gate
Pure functions, no ROS graph, run in a plain-Python venv. Enforces **≥ 90 % coverage** on
the safety/risk modules (`.coveragerc` omits the ROS adapters). Suites include:
`test_risk.py` (S_p worked example + monotonicity), `test_safety.py` /
`test_safety_monitor.py` (classify boundaries, min-distance, fail-safe), `test_retime.py`,
`test_motion_node.py` (dead-band, hold, **no-backtrack resume = AT-4r**),
`test_fast_crossing.py` (**AT-3r** tunnel invariant), `test_perception_geom.py` /
`test_perception_calibration.py`, `test_human_operator.py`, `test_hud_node.py`.

**Running (matches CI):**
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -p pytest_cov src/safecollab/test/unit \
  --cov=safecollab --cov-fail-under=90
```
(`PYTEST_DISABLE_PLUGIN_AUTOLOAD` stops the globally-installed `launch_testing` pytest
plugin — built for an older pluggy — from auto-loading; `pytest-cov` is re-added with `-p`.)

### 13.2 Integration (`test/integration/test_bringup.py`) — headless launch_testing
Brings the cell up headless and asserts: controllers active, TF chain resolves,
`/safety/scale` ≥ 20 Hz, and **clean process exit codes** (the check ADR-10 stabilises).

### 13.3 Scenario (`test/scenario/`) — the acceptance gate
Runs **inside the deploy image**. `test_at1_at5.py` records SSM behaviour over one seeded
operator cycle and asserts AT-1…AT-5 (`test_at1_through_at5`) plus a clean-shutdown
exit-code test. `test_robustness.py` covers **AT-6** (randomised paths generalise),
**AT-5r** (transient loss recovery), and exit codes. A shared `_ssm_harness.py` records
zone/scale over the run.

### 13.4 The `# pragma: no cover` convention
Node adapters, `main()`s, and `_ros_runtime` are marked no-cover and validated *only* by
the live integration/scenario suites — so a refactor in covered logic is caught by the
fast unit gate, and a refactor in adapters is caught by the (slower) live suites.

---

## 14. CI/CD & release process

### 14.1 Pipeline (`.github/workflows/ci.yml`)
Triggers: **push to `main`** and **pull_request into `main`**. Pushing a feature branch
alone runs nothing; opening the PR runs the pipeline. Gated stages:

1. **lint** — `ruff check .` + `ruff format --check .` + `yamllint`.
2. **build** — `colcon build --symlink-install` from scratch.
3. **unit** — pytest + coverage gate (≥ 90 % on safety/risk).
4. **integration** — `launch_test test_bringup.py` inside the built image.
5. **scenario** — `launch_test test_at1_at5.py` (+ `test_robustness.py`) inside the image.
6. **package** — build the image; smoke-test that ROS env sources inside it.
7. **deliver** (tags only) — save the image `.tar.gz` and publish a GitHub Release with
   `CHANGELOG-vX.Y.Z.md` as notes.

The integration and scenario stages run inside `safecollab:ci` — the exact deploy image.

### 14.2 Versioning (SemVer, state-based)
`vMAJOR.MINOR.PATCH`: MAJOR = breaking interface/behaviour change; MINOR = new capability
/ completed phase (`feat`); PATCH = fix. Tagging is **state-based** (tag when `main` is
release-worthy), not calendar-based. Phase ladder: `v0.1.0` P0 … `v0.7.0` P7, `v1.0.0`
final. Release = the container image published as a GitHub Release asset (no external
registry). `CHANGELOG.md` follows Keep a Changelog; `feat:`/`fix:`/`docs:` prefixes let
the sections assemble from the log.

### 14.3 Hard constraint — no AI-attribution trailers
Commit messages, tag messages, PR bodies, and changelog entries must contain **no**
`Co-Authored-By:` line or any trailer referencing Claude/Anthropic/an AI assistant. This
overrides any tooling default and applies to every commit.

---

## 15. Docker image

Multi-stage, optimised for size and rebuild speed (final image **3.52 GB**, down from
4.07 GB):

- **Builder stage** — `FROM ros:jazzy-ros-base`; a plain `colcon build` (not
  `--symlink-install`) so `install/` is **self-contained** (config lands in
  `share/safecollab`, resolved by `config_path()`), and only `install/` crosses into the
  runtime.
- **Runtime stage** — `FROM ros:jazzy-ros-core` (drops the build toolchain, ~80 MB); apt
  installs the gz/ros2_control/MoveIt/UR/perception stack. Each apt `RUN` prunes
  `/usr/share/{doc,man,info}` **in-layer** (~156 MB, mostly duplicated `copyright` files)
  and uses **BuildKit cache mounts** on `/var/cache/apt` + the apt lists so ~2.8 GB of
  `.debs` stay warm across rebuilds (cached rebuild ~15 s vs. ~10 min cold).
- **`.dockerignore`** excludes `.venv/` (291 MB), `.git/`, docs/media, caches, and build
  artefacts (`COPY` layer 316 MB → ~2 MB).
- **Entrypoint** (`entrypoint.sh`) sources `/opt/ros/jazzy` + the overlay; CRLF stripped
  so a Windows checkout can't break the shebang. `CMD` launches the cell headless.

---

## 16. Demo & operations scripts

All in `scripts/`; the three launchers share `scripts/demo-common.sh` (build, X11 + GPU
passthrough, run, readiness gates, teardown). Each `demo-*.sh` accepts `--no-build`.

| Script | Purpose |
|---|---|
| `demo-robot.sh` | Robot-only: `human:=false safety:=false`. Waits for the arm TF **and** the first `/task/state` (so the planner has actually started) before streaming leg names. Full pick-and-place at full speed. |
| `demo-ssm.sh` | Full SSM cell (`path_seed:=42`); waits for `/safety/zone` + arm TF; foreground HUD. The green→yellow→red story. |
| `demo-failsafe.sh` | SSM cell + a background loop that freezes `perception_node` for 5–8 s at random 15–30 s intervals (`pkill -STOP/-CONT`) → LOST fail-safe → recover; foreground HUD. |
| `record-demo.sh` | The richer recording harness: gz headless + RViz + HUD, `failsafe`/`down` subcommands, and flags (`--no-build`, `--source`, `--seed`, `--no-rviz`, `--gz-gui`, `--auto-failsafe`). Two-gate readiness (`/safety/zone` **and** `world→tool0` TF) so "READY TO RECORD" means the arm is on screen. |
| `validate-perceived.sh` | Headless live check: `/camera/image` alive, perceived `world→human` exists and agrees with `world→human_gt` within 0.15 m, and the SSM loop reacts. Exits 0 only if all four pass. |
| `demo.Dockerfile` | Layers `rviz2` onto the lean app image for the GUI demos. |

**Common readiness pattern.** `dc_wait` polls a probe inside the container until it exits
0 (or dies with the container's logs on crash). The arm-TF gate exists because
`ros2 topic echo` on a not-yet-advertised topic exits immediately — which would tear a
demo down before the arm appears.

---

## 17. Directory & file map

```
SafeCollab/
├─ AGENTS.md                     # build handoff: ground rules, contract, streams, ATs
├─ CHANGELOG.md                  # Keep a Changelog / SemVer
├─ Dockerfile                    # multi-stage build (builder ros-base → runtime ros-core)
├─ entrypoint.sh                 # sources ROS + overlay
├─ requirements.txt              # shared pip deps for the .venv
├─ .github/workflows/ci.yml      # lint→build→unit→integration→scenario→package→deliver
├─ docs/
│  ├─ KNOWLEDGE_BASE.md          # this document
│  ├─ ROADMAP-P7.md              # P7 phase plan (shipped v0.7.0)
│  └─ media/                     # demo gif + mp4
├─ scripts/
│  ├─ demo-common.sh  demo-robot.sh  demo-ssm.sh  demo-failsafe.sh
│  ├─ record-demo.sh  validate-perceived.sh  demo.Dockerfile
└─ src/safecollab/
   ├─ package.xml  setup.py  resource/safecollab
   ├─ config/    risk.yaml safety.yaml motion.yaml hud.yaml controllers.yaml
   │             kinematics.yaml joint_limits.yaml pilz_cartesian_limits.yaml
   │             pilz_industrial_motion_planner_planning.yaml view.rviz
   ├─ launch/    cell.launch.py  _stream_a_smoke_test.launch.py
   ├─ urdf/      cell.xacro  operator.sdf
   ├─ srdf/      cell.srdf.xacro
   ├─ worlds/    cell.sdf
   ├─ safecollab/
   │  ├─ _ros_runtime.py
   │  ├─ safety/       risk.py zone.py logic.py marker.py node.py config.py
   │  ├─ motion/       retime.py logic.py node.py config.py
   │  ├─ perception/   detection.py geometry.py uncertainty.py state.py node.py
   │  ├─ planning/     plan.py node.py
   │  ├─ human/        path.py model.py node.py
   │  └─ hud/          format.py node.py
   └─ test/
      ├─ unit/         test_{risk,safety,safety_monitor,retime,motion_node,
      │                fast_crossing,perception_geom,perception_calibration,
      │                human_operator,hud_node}.py
      ├─ integration/  test_bringup.py
      └─ scenario/     test_at1_at5.py  test_robustness.py  _ssm_harness.py
```

---

## 18. Glossary

| Term | Meaning |
|---|---|
| **SSM** | Speed-and-Separation Monitoring — an ISO/TS 15066 collaborative mode; keep a computed separation, slow/stop as it shrinks. |
| **`S_p`** | Protective separation distance — the minimum allowed human–robot separation. |
| **`d_yellow` / `d_red`** | Thresholds where the cell begins slowing / must be stopped (`S_p` at full-speed / reduced scenarios). |
| **`z_d` (σ)** | Operator position uncertainty from perception; live input to `S_p`. |
| **zone** | `green` (full) / `yellow` (ramp) / `red` (stop) / `lost` (fail-safe). |
| **scale** | Speed multiplier in `[0, 1]`; `retime` stretches trajectory times by `1/scale`. |
| **`s_min`** | Ramp floor — minimum non-zero speed inside the yellow band (0.10). |
| **retime** | The "speed knob": scale motion by stretching `time_from_start`. |
| **PTP** | Point-to-point (Pilz) — deterministic joint-space motion between two configs. |
| **JTC** | `joint_trajectory_controller` — the ros2_control controller driving the arm. |
| **perceived vs. ground truth** | `world→human` (camera, used by safety) vs. `world→human_gt` (sim, never used by safety). |
| **fail-safe / `lost`** | Missing/stale perception → `("lost", 0.0)` protective stop; last position never reused. |
| **pure logic / adapter** | No-ROS unit-tested module vs. thin `# pragma: no cover` ROS node. |
| **AT-x / FR-x** | Acceptance test / functional requirement (§2). |

---

*This knowledge base is derived directly from the SafeCollab source tree and
`AGENTS.md`. When code changes, update the relevant section here so it remains the
authoritative baseline for the manual and presentation.*
