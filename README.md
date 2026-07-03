# SafeCollab

Project for the course of Smart Robotics of the Master's degree in Artificial
Intelligence Engineering.

A **ROS 2 (Jazzy) + Gazebo (Harmonic)** demonstration of an **ISO/TS 15066**
Speed-and-Separation Monitoring (SSM) safety loop: a **UR5e** runs a
MoveIt/Pilz-planned pick-and-place while a `safety_monitor` scales its speed —
**green (full) → yellow (ramp) → red (protective stop)** — from the *perceived*
distance to a human operator, and **fails safe** (`lost`) when perception is lost
or stale. Zone thresholds are **derived from an ISO/TS 15066 risk model, not
hand-tuned**.

## Demo

![SafeCollab SSM demo — UR5e kitting under ISO/TS 15066 speed scaling, with a protective stop and a fail-safe re-acquire](docs/media/safecollab-demo.gif)

One `GREEN → YELLOW → RED` protective-stop → resume cycle, then a detection-loss
fail-safe (`LOST` → re-acquire). On screen you see the UR5e running its
MoveIt/Pilz kitting motion in RViz, a coloured safety-zone sphere and label at the
operator, and a one-line console HUD:

```
SSM | RED    | speed   0% | min-dist 0.38 m
```

## Quickstart

```bash
scripts/quickstart.sh
```

## Architecture & data flow

A **UR5e** runs a continuous kitting loop (bin → shared tray) planned by
**MoveIt/Pilz**. A simulated overhead **camera** watches the cell; `perception_node`
detects the human operator with classical OpenCV and estimates position with an
explicit uncertainty **σ**. `safety_monitor` computes the minimum separation
between the perceived operator and the robot, maps it through a **risk-derived**
zone model to a speed **scale** (`green` 1.0 → `yellow` ramp → `red` 0.0, plus a
fail-safe `lost`), and `motion_node` — the only node that commands the arm —
**re-times** the planned trajectory by that scale. Perception drives safety;
**planning only proposes the path, safety governs its speed.**

```mermaid
flowchart LR
    subgraph sim["Gazebo (gz) simulation"]
        CAM["overhead camera sensor"]
        ARM["UR5e + gz_ros2_control<br/>joint_trajectory_controller"]
    end

    HUMAN["human_node<br/>operator model + tray reaches"]
    PERC["perception_node<br/>classical CV → position + σ"]
    PLAN["planner_node<br/>MoveIt/Pilz kitting legs"]
    SAFE["safety_monitor<br/>min-distance → risk zone → scale"]
    MOT["motion_node<br/>retime by scale, stop/resume"]
    HUD["hud_node<br/>console safety readout"]

    HUMAN -->|drives operator body + TF world→human_gt sim-only| CAM
    CAM -->|/camera/image| PERC
    PERC -->|TF world→human perceived| SAFE
    PERC -->|/human/uncertainty| SAFE
    ARM -->|/joint_states + robot TF| SAFE

    PLAN -->|/motion/nominal_trajectory| MOT
    SAFE -->|/safety/scale| MOT
    MOT -->|/arm_controller/joint_trajectory| ARM

    SAFE -->|/safety/zone| HUD
    SAFE -->|/safety/min_distance| HUD

    SAFE -. uses .-> LOGIC["risk.py + safety_logic.py<br/>pure, no ROS"]
    MOT -. uses .-> RETIME["retime.py<br/>pure, no ROS"]
```

Solid arrows are the live ROS graph; dotted arrows are in-process calls into the
pure (no-ROS) safety maths. The **safety loop** is the closed cycle `perception →
safety_monitor → /safety/scale → motion_node → arm → /joint_states →
safety_monitor`; the **task loop** (`planner_node → /motion/nominal_trajectory →
motion_node`) only feeds it a path to slow down or stop.

### Nodes

- **`planner_node`** — owns the kitting cycle. Plans each leg (bin → tray) with
  MoveIt's deterministic Pilz PTP/LIN planner and publishes the dense
  `/motion/nominal_trajectory`. Paces legs closed-loop (waits until the arm
  reaches each goal). **Never reasons about safety zones.**
- **`human_node`** — drives the simulated operator along a (randomisable) path
  with tray reaches, moving the operator body in gz and broadcasting ground truth
  as TF `world → human_gt`. That ground truth is **sim-internal only** — the safety
  loop never consumes it (it must earn its distance from perception).
- **`perception_node`** — classical CV over `/camera/image`: detect the operator,
  back-project the centroid to a world point, broadcast the **perceived** TF
  `world → human`, and publish the position uncertainty on `/human/uncertainty`
  (σ). On loss/stale detection it stops publishing, which the monitor treats as
  `lost`.
- **`safety_monitor`** — the heart of the SSM loop. Computes the minimum distance
  between the perceived operator and the robot frames (`tool0`, `wrist_3_link`,
  `forearm_link`), feeds it and the live σ (as `z_d`) through the risk model to get
  the zone thresholds, and publishes `/safety/scale`, `/safety/zone`,
  `/safety/min_distance`, and the RViz marker. **Fail-safe:** no fresh perception →
  `lost`, scale `0.0`, and the last known position is never reused.
- **`motion_node`** — the **only** node that commands the arm. Fuses the nominal
  trajectory with `/safety/scale` and **re-times** each point's `time_from_start`
  (the speed knob), honours a protective stop at `scale == 0.0`, and resumes
  cleanly from the current joint state without a splice jerk.
- **`hud_node`** — a view-only console HUD subscribing `/safety/zone`,
  `/safety/scale`, and `/safety/min_distance`; renders one aligned, colour-coded
  line for the 5-second legibility check.

### Pure logic modules (no ROS — the TDD core)

- **`risk.py`** — the ISO/TS 15066 protective-separation model `S_p` and the
  zone-threshold derivation from `config/risk.yaml` (see the risk model below).
- **`safety_logic.py`** — `classify(d, …)` maps a distance to `(zone, scale)` with
  the ramp formula and the `d is None → lost` fail-safe.
- **`retime.py`** — stretches trajectory timestamps by `1/scale`; the maths
  `motion_node` applies to turn a scale into a slower/stopped motion.

### The speed-scaling loop, step by step

1. `perception_node` publishes the perceived operator TF `world → human` and σ.
2. `safety_monitor` computes min-distance to the robot frames; `risk.py` turns the
   live σ (`z_d`) into `(d_red, d_yellow)`; `safety_logic.classify()` yields a
   `zone` and a `scale` in `[0, 1]`.
3. `/safety/scale` reaches `motion_node`, which re-times the current
   `/motion/nominal_trajectory` (from `planner_node`) and commands
   `/arm_controller/joint_trajectory`.
4. The controller moves the UR5e; `/joint_states` and the robot TF close the loop
   back into `safety_monitor`.
5. On lost/stale perception the monitor emits `lost` / scale `0.0` — a protective
   stop — until re-acquisition.

Because scaling lives entirely in `motion_node`, the SSM guarantee is independent
of *how* the path was produced: swapping a hand-solved arm for the MoveIt-planned
UR5e changed only the trajectory **source**, not the safety governor.

---

## Safety model (ISO/TS 15066 SSM)

The zones are **derived from an ISO/TS 15066 risk model at start-up, never
hand-tuned**: the model lives in `src/safecollab/safecollab/risk.py`, its inputs in
`src/safecollab/config/risk.yaml`, and the numbers below are what
`risk.thresholds()` reproduces (unit-tested in `test/unit/test_risk.py`).

### The protective-separation model

SSM keeps a **protective separation distance** `S_p` between human and robot; if
the measured separation drops below `S_p`, the robot must slow or stop. SafeCollab
uses the standard's sum of contributions:

```
S_p = S_H + S_R + S_S + C + Z_d + Z_r
```

| Term  | Meaning                                             | In SafeCollab |
|-------|-----------------------------------------------------|---------------|
| `S_H` | distance the human travels during reaction + stop   | `v_h · (t_r + t_s)` |
| `S_R` | distance the robot travels during its reaction time | `v_r · t_r` |
| `S_S` | robot stopping-distance contribution                | `s_s` |
| `C`   | intrusion distance (how far a hand reaches in)      | `c` |
| `Z_d` | operator **position uncertainty**                   | `z_d` — **live from perception (σ)** |
| `Z_r` | robot position uncertainty                           | `z_r` |

`Z_d` is the important one: it is **not** a constant — perception supplies it live
as the detection uncertainty σ, so the zones breathe with how well the camera
currently sees the operator.

### Inputs (`config/risk.yaml`)

Shared cell/sensor properties, plus two robot scenarios that differ **only** in
`v_r` and `t_s` (a robot already moving slowly has less kinetic energy and
genuinely stops faster, so `t_s` shrinks *with* `v_r`):

| Input | Value | |
|---|---|---|
| `v_h` | 1.6 m/s | operator approach speed |
| `t_r` | 0.10 s | system reaction time |
| `s_s` | 0.06 m | robot stopping-distance contribution |
| `c`   | 0.10 m | intrusion distance |
| `z_r` | 0.02 m | robot position uncertainty |
| **full speed** | `v_r` 0.50 m/s · `t_s` 0.25 s | governs the **yellow** edge |
| **reduced**    | `v_r` 0.10 m/s · `t_s` 0.02 s | governs the **red** edge |

### Derived thresholds

`d_yellow` is `S_p` computed with the **full-speed** scenario (begin slowing);
`d_red` is `S_p` with the **reduced** scenario (must be stopped):

| `z_d` (m) | `d_red` (m) | `d_yellow` (m) |
|---|---|---|
| **0.05** (nominal) | **0.43** | **0.84** |
| 0.10 (noisier detection) | 0.48 | 0.89 |

**Property:** raising `z_d` **widens both thresholds** — a less certain operator
position makes the cell more cautious, exactly as SSM intends. This monotonicity is
asserted in `test_risk.py`.

### From distance to speed scale

`safety_logic.classify(d, d_red, d_yellow, s_min)` maps the live minimum separation
`d` to a zone and a speed `scale ∈ [0, 1]`:

| Condition | Zone | Scale |
|---|---|---|
| `d is None` (perception lost/stale) | `lost` | `0.0` — fail-safe |
| `d ≥ d_yellow` | `green` | `1.0` — full speed |
| `d_red < d < d_yellow` | `yellow` | ramps `s_min … 1.0` linearly |
| `d ≤ d_red` | `red` | `0.0` — protective stop |

The yellow ramp is `scale = s_min + (1 − s_min)·(d − d_red)/(d_yellow − d_red)`.
The **yellow edge is exclusive** (`d == d_yellow → green`) and the **red edge
inclusive** (`d == d_red → red`). A `lost` or `red` scale of `0.0` is a protective
stop, and the last known operator position is **never** reused to fabricate a
distance.

---

## Running & recording the demo

`scripts/record-demo.sh` brings up Gazebo + RViz + the HUD, driven by the
*perceived* operator, in a `safecollab:demo` image (the lean app image plus
`rviz2`, built automatically). Gazebo runs **headless** (offscreen rendering); RViz
shows the robot, camera feed, and zones.

```bash
# Build (if needed) and bring up Gazebo + RViz + the HUD:
scripts/record-demo.sh
```

Wait for `cell is up`; this terminal becomes the HUD. **Cue the fail-safe** on
camera from a second terminal:

```bash
scripts/record-demo.sh failsafe            # freezes perception ~6 s, then resumes
scripts/record-demo.sh failsafe --hold 8   # hold the loss longer
```

The zone goes grey `LOST`, the HUD shows `LOST / 0%`, then it re-acquires and the
arm resumes. `Ctrl-C` (or `scripts/record-demo.sh down`) tears the whole demo down.

A good ~30-second take covers the acceptance story end to end: **approach**
(`GREEN → YELLOW → RED`, speed → `0%`), **retreat** (arm resumes, back to `GREEN`),
then the **fail-safe** cue (grey `LOST` → re-acquire).

| Flag | Effect |
|------|--------|
| `--no-build` | reuse the existing `safecollab:dev` image |
| `--source ground_truth` | drive from ground-truth TF if perception is GPU-flaky |
| `--seed N` | a different deterministic operator path (default `42`) |
| `--no-rviz` | HUD only (no RViz window; gz stays headless) |
| `--gz-gui` | also open the raw Gazebo window (needs good GL passthrough) |
| `--auto-failsafe` | fire the loss→recover cue automatically (~35 s in) |

---

## Validating the perceived path live

CI proves the AT-1…AT-5 acceptance behaviour with `safety_source:=ground_truth`,
which feeds the monitor the simulator's exact operator pose and **bypasses
perception**. `scripts/validate-perceived.sh` closes that gap: it runs the live
camera → detector → SSM loop headless and compares the **perceived** `world→human`
against the simulator's **ground-truth** `world→human_gt`.

```bash
scripts/validate-perceived.sh          # builds, runs headless, prints PASS/FAIL
# flags: --no-build  --window N  --seed N  --keep
```

It exits `0` only if all four checks pass:

1. **`/camera/image` is publishing** — the gz camera sensor + `ros_gz_image`
   bridge are alive (perception's input).
2. **Perceived TF `world→human` exists** — `perception_node` detected the operator
   and is broadcasting its estimated pose.
3. **Perceived ≈ ground truth** — the two agree within `0.15 m`; this is what
   proves the camera calibration *and* the detector are correct live, not just in
   unit tests.
4. **The SSM loop reacts** — as the operator's path nears the arm, `/safety/zone`
   leaves `green` and `/safety/scale` drops below `1.0`.

A green run means the **perceived** human — not the ground-truth shortcut — drives
the SSM loop, satisfying the "perceived human in the headline demo" requirement.

---

## Robustness guarantees

Four guarantees harden the SSM loop under adverse conditions, each verified at the
layer where it is cheapest and most deterministic — a pure-logic property as a unit
test, an emergent behaviour as a live scenario:

| # | Guarantee | Layer | Test |
|---|-----------|-------|------|
| AT-6 | Behaviour generalises across randomised operator paths (no path-specific tuning) | scenario (live) | `test/scenario/test_robustness.py::test_at6_randomised_paths_generalise` |
| AT-4r | A protective stop part-way through a leg resumes without backtracking | unit | `test/unit/test_motion_node.py` |
| AT-3r | A fast operator crossing cannot skip the protective stop between ticks | unit | `test/unit/test_fast_crossing.py` |
| AT-5r | A transient detection loss fails safe, then re-acquires and resumes | scenario (live) | `test/scenario/test_robustness.py::test_recovery_after_transient_detection_loss` |

These complement the AT-1…AT-5 acceptance harness (`test/scenario/test_at1_at5.py`),
which proves the core green→yellow→red / stop / resume / fail-safe behaviour for a
single seeded path.

- **AT-6** — with a fixed `path_seed`, `human_node` regenerates a fresh random
  operator path each cycle, so one run is driven by several distinct paths. Using
  the same `risk.yaml` thresholds, the escalation and clean resume must recur, and
  the zone must never spuriously enter `lost`.
- **AT-4r** — `motion_node` drops the nominal waypoints the robot has already
  passed and resumes from the current state, so progress toward the goal is
  monotonic (no backtrack jerk); the goal is always retained.
- **AT-3r** — a sampling invariant: even at the maximum modelled operator speed
  (`risk.yaml` `v_h`), the displacement per safety-monitor tick (`v_h / _TICK_HZ`)
  is well below the width of the red band `[0, d_red]`, so a fast head-on approach
  cannot "tunnel" past the stop between two ticks. Inputs are read from the real
  config, never hard-coded.
- **AT-5r** — a transient loss (occlusion, dropped detection) must fail safe and
  recover: after `loss_timeout` the monitor passes `d=None` to `classify()`, which
  returns `("lost", 0.0)` — the last position is never reused — and on recovery the
  loop re-acquires and resumes with no node faulting.

Reproduce the unit guarantees (fast, no simulator) with
`pytest test/unit/test_motion_node.py test/unit/test_fast_crossing.py`; the live
guarantees run inside the deploy image via
`launch_test src/safecollab/test/scenario/test_robustness.py` (the same path CI's
`scenario` stage runs).

---

## Testing & CI

Three test layers under `src/safecollab/test/`: **unit** (pure logic, ≥ 90 %
coverage gate on safety/risk), **integration** (headless `launch_testing`
bring-up), and **scenario** (the AT-1…AT-6 acceptance tests). CI runs seven gated
stages — **lint → build → unit → integration → scenario → package → deliver** — and
on a `vX.Y.Z` tag the `deliver` stage publishes the image as a GitHub Release asset
with the matching changelog slice.

Release history follows SemVer in [`CHANGELOG.md`](CHANGELOG.md).
