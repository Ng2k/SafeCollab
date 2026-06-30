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

P5 — polish & demo (toward `v0.5.0`); see `docs/ROADMAP-P5.md`.

### Added

- feat(viz): **RViz zone visualisation** (P5 sprint 1). `safety_monitor` now also
  publishes a floating `TEXT_VIEW_FACING` zone label (`/viz/safety_marker` id 1,
  e.g. `RED` / `LOST`) above the existing zone sphere (id 0), sharing the single
  `_ZONE_RGBA` source so the two colours can never drift. `cell.launch.py` gains
  an `rviz:=true` argument (default `false`, so CI/headless and the scenario
  harness are unchanged) that starts `rviz2 -d config/view.rviz`; `view.rviz`
  gains a Marker display on `/viz/safety_marker`. Label gap above the sphere and
  glyph height are configurable (`marker_label_offset`, `marker_label_height` in
  `config/safety.yaml`; ground rule 5).
- test(viz): unit-cover the label geometry (`_marker_params` label text/position)
  and assert the marker contract live (AT-V in the AT-1..AT-5 harness): the sphere
  is zone-coloured and the text label names the zone in the matching colour, on
  `/viz/safety_marker` — folded into the existing bring-up (no extra launch).

---

## [0.4.0] - 2026-06-28 — P4: robustness (generalisation, recovery, fail-safe re-acquire)

Hardens the closed Speed-and-Separation-Monitoring loop so it behaves correctly
under adverse conditions, proven by acceptance tests with **no path-specific
tuning and no new hand-tuned constants**. The four P4 guarantees — randomised
operator-path generalisation (AT-6), no-backtrack resume from a mid-trajectory
protective stop (AT-4r), a fast crossing that cannot skip the protective stop
(AT-3r), and transient detection-loss → fail-safe → re-acquire → resume (AT-5r)
— are each verified at the cheapest reliable layer (a pure-logic unit test or a
live scenario). See `docs/ROBUSTNESS.md`.

### Added

- test(scenario): **AT-6** — randomised operator paths generalise. With a fixed
  seed, `human_node` regenerates a fresh random path each cycle, so one long
  ground-truth run is driven by several distinct paths; the harness asserts the
  `green→yellow→red` escalation and clean resume recur across them (no
  path-specific tuning) and that the zone never spuriously enters `lost`.
- test(scenario): **AT-5 hardening** — transient detection loss recovery. In the
  same robustness bring-up, `human_node` is SIGSTOP'd so the human TF goes stale:
  the monitor must fail-safe (zone `lost`, scale 0); on SIGCONT the zone must
  leave `lost` (re-acquire, no permanent latch), with no node faulting (asserted
  by the post-shutdown exit-code check). The recovery check is position-independent
  — zone-left-`lost` is the proof, since where the operator is when re-acquired
  (and thus the exact scale) depends on freeze-duration × real-time factor; scale
  is asserted to lift only when a tracked out-of-red zone is actually observed
  (catching a scale-only latch without a timing dependency). Runs after AT-6 so it
  cannot perturb it.
- test(scenario): P4 robustness acceptance harness
  (`test/scenario/test_robustness.py`) wired into the CI `scenario` stage,
  holding the live robustness cases (AT-6, AT-5r); the pure-logic cases (AT-4r,
  AT-3r) are unit tests (see `docs/ROADMAP-P4.md` / `docs/ROBUSTNESS.md`).
- docs(roadmap): `docs/ROADMAP-P4.md` — TDD/agile task breakdown for the P4
  robustness phase (→ v0.4.0).
- docs(robustness): `docs/ROBUSTNESS.md` — the four P4 robustness guarantees
  (AT-6, AT-4r, AT-3r, AT-5r), the layer each is verified at, and how to
  reproduce them.

### Changed

- test(scenario): factor the shared launch_testing helpers (topic collection,
  zone-stream analysis, controller readiness) out of the AT-1..AT-5 harness into
  `test/scenario/_ssm_harness.py`, reused by both scenario harnesses (no
  duplication).
- ci(deliver): build, save and publish the deploy image tarball **as the GitHub
  Release asset, only on version tags** — there is no `actions/upload-artifact`
  copy (it was redundant with the Release asset and consumed the Actions storage
  quota, ~1 GB per run). The `package` job still verifies the image build on
  every run, so regular pushes/PRs build no tarball at all.

### Fixed

- fix(motion): **AT-4 hardening** — a protective stop part-way through a leg now
  resumes without backtracking. A leg is published as `[leg_start, leg_end]`, so
  the old resume (`[current, leg_start, leg_end]`) drove the arm back to the leg
  start before going forward. `MotionLogic` now drops nominal waypoints the robot
  has already passed (those no closer to the goal than the current state),
  resuming as `[current, leg_end]` — monotonic progress toward the goal, no jerk.
- test(safety): **AT-3 hardening** — guard the fast-crossing sampling invariant
  (`test_fast_crossing.py`): even at the maximum modelled operator speed
  (`risk.yaml` `v_h`), the per-monitor-tick step (`v_h / SafetyMonitorNode._TICK_HZ`)
  is well below the red band `[0, d_red]`, so several samples land in red before
  contact — a fast crossing cannot tunnel through the protective stop between
  ticks. Inputs are read from the real config/node (no hard-coded thresholds).
- fix(scenario): restore `import threading` in the AT-1..AT-5 harness — the
  helper extraction dropped it while the test body still uses `threading.Thread`
  for its parallel collectors (a runtime `NameError` the import-time check missed).
- fix(scenario): record the cell topics (`/safety/scale`, `/safety/zone`,
  `/safety/min_distance`, and the `/arm_controller/joint_trajectory` presence
  check) with a single in-process rclpy subscriber (`record_safety_topics`)
  instead of `ros2 topic echo` subprocesses. The CLI echo lost the DDS discovery
  race under load and captured nothing — first the reliable+transient_local
  `/safety/zone` (failing AT-1/AT-2 with an empty zone stream while
  `/safety/scale` on the same tick was fine), then the arm-trajectory `--once`
  check (failing AT-4). One subscriber with QoS matched to each topic's contract
  is deterministic.
- fix(scenario): make the AT-5r resume check position-independent. The old check
  asserted scale rose above the stop floor unconditionally after re-acquire, but
  scale is 0 in both `red` and `lost`, so it required the re-acquired operator to
  also leave the red band within the window — a freeze-duration × RTF artifact
  (observed `max 0.000`). Scale is now asserted to lift only when a tracked
  out-of-red (`yellow`/`green`) zone is actually observed, which still catches a
  scale-only latch but never flakes on an operator legitimately still in red; the
  no-latch proof rests on the zone leaving `lost`.
- test(integration,scenario): tolerate the gz stack's `SIGABRT` (-6) during
  SIGINT teardown in the post-shutdown exit-code checks (gz sim and the
  `ros_gz_bridge` `/clock` `parameter_bridge` intermittently abort in
  gz-transport cleanup), alongside the existing `-SIGTERM` tolerance — a
  teardown-only artifact, not a bring-up crash.

**Verification:** 264 unit tests passing, total coverage 97.4% (≥ 90 % gate on
safety/risk); `ruff check` + `ruff format --check` clean; the AT-1..AT-5 and the
P4 robustness scenario harnesses both green headless; full CI pipeline (lint,
build, unit + coverage, integration, package, deliver) green on GitHub Actions.

**Artifact:** `safecollab-v0.4.0.tar.gz` (published on the repo Releases page).

---

## [0.3.0] - 2026-06-28 — P2–P3: perception-driven human + closed safety loop

The **perceived** Speed-and-Separation-Monitoring path now runs end-to-end live: a
simulated overhead camera detects the operator, `perception_node` estimates its pose with
an explicit uncertainty σ, and `safety_monitor` scales robot speed by zone
(green → yellow → red) with a fail-safe on lost detection. `scripts/validate-perceived.sh`
passes all checks live (perceived vs ground-truth agree to ~1 cm; zones reach yellow/red as
the operator approaches; `/safety/scale` drops to 0 on breach). This closes the headline
demo's Definition of Done #5 — the perceived human, not the ground-truth shortcut, drives
the SSM loop.

### Added

- feat(perception): `perception_node` (Stream C) — classical-CV (HSV colour-blob) operator
  detection over `/camera/image`, pinhole back-projection geometry (pixel → ray → world-plane
  intersection, unit-tested sub-mm), broadcasts the perceived `world→human` TF and publishes
  `/human/uncertainty` (σ, m). A loss-timeout stops the broadcast so downstream sees the
  fail-safe.
- feat(safety): `safety_monitor` + `SafetyMonitorLogic` + `config/safety.yaml` (Stream F) —
  sweeps minimum separation over the robot frames (TCP / link_6 / link_3) against the
  perceived human, classifies the ISO/TS 15066 risk zone, and publishes `/safety/scale`,
  `/safety/zone`, `/safety/min_distance`. Fail-safe to (`lost`, 0.0) on stale/absent
  perceived TF.
- feat(launch): `cell.launch.py` (Stream G) assembles the full closed loop — gz sim + world
  + `robot_state_publisher` + spawn + controllers + camera/clock bridges + the five
  application nodes — with `headless:=true` for CI; bridges gz `/clock` → ROS so every
  `use_sim_time` node shares sim time.
- feat(safety): `safety_source` launch toggle — `perceived` (headline demo) or
  `ground_truth` fallback (AGENTS §11 cut-scope #4), so the ramp / stop / resume / fail-safe
  can be demonstrated even if perception is unavailable.
- feat(sim): a camera-visible yellow operator body, moved in gz by `human_node` via the
  `/world/empty/set_pose` service so the live camera path can detect the operator without a
  physical person; the full 3-D ground truth is still broadcast on `world→human_gt`.
- feat(sim): `worlds/cell.sdf` (stock `empty.sdf` + the gz Sensors system) so the overhead
  camera renders, installed via `setup.py`.
- test(scenario): AT-1..AT-5 ground-truth scenario harness (`test/scenario`) and headless
  integration bring-up assertions (controllers active, `/safety/scale` liveness, TF chain).
- docs(validate): `docs/VALIDATE.md` + `scripts/validate-perceived.sh` — live end-to-end
  validation of the perceived path (camera → perception → SSM).

### Changed

- The overhead camera is mounted higher, centred, and wider (world z 2.40 m, pitch 1.2 rad,
  hfov 1.5, 15 Hz) so its FOV covers the whole operator working volume; `perception_node`'s
  calibration mirrors the mast pose and back-projects onto the operator's constant
  working-height plane (0.95 m), with a flat marker held at that plane so the recovered
  planar pose is parallax-free.
- deps: `opencv-python` → `opencv-python-headless` for CI / Docker compatibility.
- ci: the integration `launch_test` runs inside the deploy image; the `/safety/scale` check
  asserts a wall-clock liveness floor (the 20 Hz design rate is sim-time), recalibrated to
  5 Hz now that the camera renders under software GL.

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
- fix(ci): keep the safety loop above the integration liveness floor now that the camera
  renders. The headless CI sim renders the camera under software GL, which lowers the
  real-time factor; lower the camera rate to 15 Hz (urdf/cell.xacro) to recover RTF and
  recalibrate the `/safety/scale` wall-clock floor (8 → 5 Hz) in `test_bringup.py`. The
  20 Hz sim-time design rate of the safety loop is unchanged.
- fix: make every node's `main()` survive the rclpy SIGINT teardown race — the executor
  can raise `RuntimeError: Unable to convert call argument` from `_take_subscription`
  while the context is torn down, and `rclpy.ok()` is an unreliable discriminator. Treat
  that specific take-time error as benign so launch/CI shutdown exit codes stay clean,
  while still re-raising genuine `RuntimeError`s.
- fix(task): solve the kitting poses so the arm reaches the workspace at table height.
- fix(scenario): match the `/safety/zone` `transient_local` QoS in the recorder and assert
  zone ordering as a subsequence (not a fixed first-index).
- fix(packaging): install node executables to `lib/safecollab` (`setup.cfg`) and restore the
  urdf/config `data_files` lost in an earlier `setup.py` merge.
- chore(ci): bring the headless build / integration / package stages green — install the
  ros-gz/control deps, run the headless launch under `docker --init` for clean teardown,
  resolve a pytest plugin-autoload conflict, and run the integration test inside the deploy
  image.

**Verification:** 258/258 unit tests passing; full CI pipeline (lint, build, unit + coverage
≥ 90 % on safety/risk, integration, package, deliver) green on GitHub Actions;
`scripts/validate-perceived.sh` → 7/7 checks passing live (camera → perception → SSM).

**Artifact:** `safecollab-v0.3.0.tar.gz` (published on the repo Releases page).

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
| ~~`v0.3.0`~~ | P2–P3 | ~~Perception-driven human + safety loop closed (zones, stop, resume, fail-safe)~~ — **released above** |
| ~~`v0.4.0`~~ | P4 | ~~Robustness (randomised paths, recovery, edge cases)~~ — **released above** |
| `v0.5.0` | P5 | Polish + demo video / GIF |
| `v0.6.0` | P6 | Documentation (README, diagrams, risk note) |
| **`v1.0.0`** | **submission** | **Acceptance AT-1…AT-5 passing; final image artifact + notes** |

[Unreleased]: https://github.com/<owner>/safecollab/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/<owner>/safecollab/releases/tag/v0.4.0
[0.3.0]: https://github.com/<owner>/safecollab/releases/tag/v0.3.0
[0.2.0]: https://github.com/<owner>/safecollab/releases/tag/v0.2.0
[0.1.0]: https://github.com/<owner>/safecollab/releases/tag/v0.1.0
