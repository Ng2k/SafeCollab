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
fail-safe (`LOST` → re-acquire). See [`docs/DEMO.md`](docs/DEMO.md) for how to run
and record it.

## Quickstart (one-command run)

```bash
# Build the image (the deploy artifact)
docker build -t safecollab:latest .

# Run with the GUI (Gazebo + RViz), X11 passthrough
xhost +local:root
docker run --rm -it --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix safecollab:latest
```

Headless (CI / no display) — the cell launches with a single argument:

```bash
ros2 launch safecollab cell.launch.py headless:=true
```

The one-command GUI demo (Gazebo + RViz zones + console HUD, perceived-operator
driven, with an on-demand fail-safe cue) is scripted in
[`scripts/record-demo.sh`](scripts/record-demo.sh); see [`docs/DEMO.md`](docs/DEMO.md).

## How it works

A camera observes the cell → `perception_node` detects the operator (classical
OpenCV) with an uncertainty σ → `safety_monitor` turns min-distance + σ into a
speed **scale** via the risk model → `motion_node` **re-times** the planned
trajectory by that scale. Planning proposes the path; **safety governs its
speed.**

- **Architecture & data flow:** [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) —
  the node graph, the topics, and the SSM loop end to end.
- **Safety model:** [`docs/RISK.md`](docs/RISK.md) — the ISO/TS 15066 `S_p` model,
  the config inputs, and the derived `green/yellow/red` thresholds.

## Testing & CI

Three test layers (`src/safecollab/test/`): **unit** (pure logic + the
doc-consistency checks, ≥ 90 % coverage gate on safety/risk), **integration**
(headless `launch_testing` bring-up), and **scenario** (the AT-1…AT-6 acceptance
tests). CI runs six gated stages — **lint → build → unit → integration → package
→ deliver** — and on a `vX.Y.Z` tag the deliver stage publishes the image as a
GitHub Release asset with the matching changelog. See
[`docs/VALIDATE.md`](docs/VALIDATE.md) and [`docs/ROBUSTNESS.md`](docs/ROBUSTNESS.md).

## Documentation

| Doc | What |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Node graph + SSM data flow |
| [`docs/RISK.md`](docs/RISK.md) | ISO/TS 15066 risk model & thresholds |
| [`docs/DEMO.md`](docs/DEMO.md) | Run & record the headline demo |
| [`docs/VALIDATE.md`](docs/VALIDATE.md) | Validation / acceptance walkthrough |
| [`docs/ROBUSTNESS.md`](docs/ROBUSTNESS.md) | Robustness behaviour (P4) |
| [`CHANGELOG.md`](CHANGELOG.md) | Release history (SemVer) |

The phase roadmap lives in `docs/ROADMAP-P*.md`.
