# Changelog

All notable changes to **SafeCollab** are recorded here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html) (`vMAJOR.MINOR.PATCH`).

> **How releases work.** The CD "deploy" is the **container image**. On a `vX.Y.Z` git tag, CI
> builds the image, saves it as `safecollab-vX.Y.Z.tar.gz`, and **publishes it as a GitHub Release
> asset on this repository** (no external registry such as Docker Hub). The release notes for each
> tag are the matching section below, sliced into `CHANGELOG-vX.Y.Z.md` by the pipeline.
>
> **Categories:** `Added` (feat) · `Changed` · `Fixed` (fix) · `Removed` · `Deprecated` ·
> `Security`. Commits use `feat:` / `fix:` / `docs:` prefixes so these sections assemble from the
> commit log.

---

## [Unreleased]

### Fixed

- fix(build): install the gz Harmonic Python bindings (`python3-gz-transport13`,
  `python3-gz-msgs10`) from the OSRF apt repo — they are absent from the ROS apt repo,
  which ships gz only as C++ vendor packages — so `human_node` can drive the operator
  body via the `/world/empty/set_pose` service; install the `worlds/` directory so
  `cell.launch.py` can resolve the project world after `colcon build`.
- fix(sim): load `worlds/cell.sdf` (stock `empty.sdf` + the gz Sensors system) so the
  overhead camera actually renders frames, and raise/centre/widen the camera mast
  (world z 2.40 m, pitch 1.2 rad, hfov 1.5 rad) so the FOV covers the whole operator
  working volume instead of only the tray (the standing/approach path was off-frame).
- fix(perception): match `perception_node`'s camera pose + intrinsics to the new mast and
  back-project detections onto the operator's constant working-height plane (0.95 m);
  represent the operator with a flat marker held at that plane so the recovered planar
  pose is parallax-free — perceived vs ground-truth now agree to ~1 cm live.
- fix(sim): correct `human_node`'s gz `set_pose` request (it omitted the required
  `request_type` argument, so the call always failed) and move the synchronous request to
  a background worker thread, so a slow service can no longer stall the 50 Hz
  ground-truth broadcast or freeze the operator body.
- fix(validate): in `validate-perceived.sh`, sample perceived-vs-ground-truth, and
  zone-vs-scale, concurrently — the previous sequential sampling measured the operator's
  motion between reads (and cross-window phase skew) rather than perception accuracy / the
  co-occurring SSM response, producing spurious failures on a correct system.

### Added

- _(work in progress toward the next tag)_

---

## [0.2.0] - 2026-06-26 — P1: kitting loop + motion layer

### Added

- feat(motion): `motion_node` — fuses the nominal trajectory with `/safety/scale` via the
  existing `retime()`, split into `MotionLogic` (pure Python, fully unit-testable) and
  `MotionNode` (the ROS wrapper). Protective stop on `scale == 0.0` (no command published, no
  division-by-zero). Clean resume re-plans from the current joint state as the first waypoint at
  `t=0`, avoiding a jerk on resume.
- feat(task): `task_node` — `KittingStateMachine` driving the
  `GO_TO_BIN → PICK → GO_TO_TRAY → DROP` cycle, alternating between the two feeder bins, with
  per-leg trajectories respecting joint limits. Never reasons about safety zones (kept
  orthogonal to the safety loop, per §3/§5.9).
- feat(sim): `human_node` — `OperatorModel` with randomised operator paths; every generated path
  is guaranteed to include a tray-reach (FR-11, AT-6).
- feat(packaging): `motion_node`, `task_node`, and `human_node` registered as console-script
  entry points in `setup.py`.

### Changed

- The ground-truth operator TF is broadcast as **`human_gt`**, kept distinct from the perceived
  `human` TF that `perception_node`/`safety_monitor` will use — `human_gt` is sim-internal only
  and is never consumed by the safety loop. This convention is now fixed in `AGENTS.md` §3.

**Verification:** 132/132 unit tests passing (27 safety/risk core + 29 motion + 36 task +
40 human), 100% coverage on safety/risk/motion modules (gate: ≥ 90%); full CI pipeline (lint,
build, unit+coverage, integration, package, deliver) green on GitHub Actions.

**Known follow-ups (tracked, not blocking):** the post-resume waypoint timing is conservative
(lands at `t=2T` rather than `t=T`) and may be tuned once live behaviour is observed in
simulation; the kitting state machine's joint configurations are geometric approximations
pending IK verification once Stream A's cell is fully integrated with the application nodes.

**Artifact:** `safecollab-v0.2.0.tar.gz` (published on the repo Releases page).

---

## [0.1.0] - 2026-06-25 — P0: foundation + CI

### Added

- feat(safety): `safety_logic.classify()` with green/yellow/red/lost zones and the
  fail-safe-on-`None` contract, written test-first; boundary semantics locked by test
  (yellow edge exclusive, red edge **inclusive** — `d ≤ d_red`).
- feat(risk): `risk.py` ISO/TS 15066 `S_p` protective-separation model; thresholds computed at
  start-up from `config/risk.yaml` (`full_speed` / `reduced` scenarios), not hard-coded.
- feat(motion): `retime()` trajectory speed-scaling with an explicit `scale == 0.0` guard
  (protective stop, no division-by-zero).
- feat(sim): 6-DOF arm (`arm.xacro`) and the kitting-cell world (`cell.xacro`) — work table, two
  feeder bins, the shared kitting tray, and a down-looking camera sensor — wired to
  `gz_ros2_control` / `GazeboSimROS2ControlPlugin`.
- feat(sim): `controllers.yaml` (`joint_state_broadcaster` + `arm_controller` /
  `joint_trajectory_controller`) and a debug `view.rviz`.
- feat(ci): Dockerfile (`ros:jazzy-ros-base` + the project's apt/pip dependency set),
  `entrypoint.sh`, and the GitHub Actions pipeline — six gated stages: lint → build →
  unit (+coverage) → integration → package → deliver.
- feat(ci): pre-commit hooks — ruff (lint + format), yamllint, Conventional Commits, and a
  block on AI-attribution trailers (ground rule 7) — installed and verified end to end.
- feat(packaging): minimal `ament_python` `setup.py` / `package.xml` / `resource/safecollab`,
  making `safecollab` an installable package (`pip install -e src/safecollab`) and resolving the
  `safecollab.*` imports used by every unit test.

### Changed

- The `risk.yaml` config model was restructured mid-stream: each speed scenario now carries its
  own `v_r` **and** `t_s` (a slower robot also stops faster), replacing an earlier draft that
  scaled `v_r` alone — this is what makes `d_yellow ≈ 0.84 m` / `d_red ≈ 0.43 m` physically
  reachable from the ISO/TS 15066 model rather than an arbitrary pair of constants.
- Stream B's modules were relocated from a root-level `safecollab/` to `src/safecollab/safecollab/`
  to match the package layout, alongside its config and tests.

### Fixed

- fix(risk): correct the ISO/TS 15066 worked example — the original `d_red ≈ 0.35 m` target was
  mathematically unreachable with `v_h = 1.6 m/s` (the human term `S_H` alone already exceeds it).
  Thresholds recomputed by varying both `v_R` and `T_s` together between the full-speed and
  reduced-speed scenarios, yielding the physically consistent `d_yellow ≈ 0.84 m` /
  `d_red ≈ 0.43 m`.
- style: applied `ruff format` and fixed `yamllint` colon-spacing / comment-indentation issues in
  modules and config written before the lint hooks existed (Stream B's first pass predated
  Stream G's `.pre-commit-config.yaml`).

**Verification:** 27/27 unit tests passing; 100% coverage on `safety_logic.py`, `risk.py`,
`retime.py` (gate: ≥ 90%); `gz sim` spawn, `arm_controller` + `joint_state_broadcaster` active,
and a clean `world → … → tcp` / `world → … → camera_optical_frame` TF tree all verified live in
the `ros:jazzy-ros-base` container.

**Artifact:** `safecollab-v0.1.0.tar.gz` (published on the repo Releases page).

---

## Planned releases

| Tag | Phase | Theme |
|---|---|---|
| ~~`v0.1.0`~~ | P0 | ~~Foundation + CI; pure safety/risk core (TDD)~~ — **released above** |
| ~~`v0.2.0`~~ | P1 | ~~Collaborative kitting loop~~ — **released above (+ motion layer)** |
| `v0.3.0` | P2 | Perception-driven human (TF + uncertainty) |
| `v0.4.0` | P3 | Safety loop closed (zones, stop, resume, fail-safe) |
| `v0.5.0` | P4 | Robustness (randomised paths, recovery, edge cases) |
| `v0.6.0` | P5 | Polish + demo video / GIF |
| `v0.7.0` | P6 | Documentation (README, diagrams, risk note) |
| **`v1.0.0`** | **submission** | **Acceptance AT-1…AT-5 passing; final image artifact + notes** |

[Unreleased]: https://github.com/<owner>/safecollab/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/<owner>/safecollab/releases/tag/v0.2.0
[0.1.0]: https://github.com/<owner>/safecollab/releases/tag/v0.1.0
