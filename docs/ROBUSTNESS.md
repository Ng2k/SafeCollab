# Robustness guarantees (P4)

The P4 phase hardens the already-working Speed-and-Separation-Monitoring (SSM)
loop so it behaves correctly under adverse conditions, proven by tests rather
than asserted by hand. There are four guarantees; each is covered at the layer
where it is cheapest and most deterministic to verify (a pure-logic property as a
unit test, an emergent behaviour as a live scenario).

| # | Guarantee | Layer | Test |
|---|-----------|-------|------|
| AT-6 | Behaviour generalises across randomised operator paths (no path-specific tuning) | scenario (live) | `test/scenario/test_robustness.py::test_at6_randomised_paths_generalise` |
| AT-4r | A protective stop part-way through a leg resumes without backtracking | unit | `test/unit/test_motion_node.py` (`test_resume_*backtrack*`, `*monotonic*`) |
| AT-3r | A fast operator crossing cannot skip the protective stop between ticks | unit | `test/unit/test_fast_crossing.py` |
| AT-5r | A transient detection loss fails safe, then re-acquires and resumes | scenario (live) | `test/scenario/test_robustness.py::test_recovery_after_transient_detection_loss` |

These complement the AT-1…AT-5 acceptance harness
(`test/scenario/test_at1_at5.py`), which proves the core green→yellow→red /
stop / resume / fail-safe behaviour for a single seeded path.

## What each guarantee means

### AT-6 — generalisation across randomised paths
With a fixed `path_seed`, `human_node` regenerates a *fresh random* operator path
each cycle, so one long ground-truth run is driven by several **distinct** paths.
Using the **same** `risk.yaml` thresholds — no per-path tuning — the SSM
escalation `green→yellow→red` and the clean resume must recur across them, and the
zone must never spuriously enter `lost`. The harness requires several complete
escalations and recoveries within the recording window.

### AT-4r — no backtrack on mid-trajectory resume
`task_node` publishes each leg as a 2-waypoint trajectory `[leg_start, leg_end]`.
If a protective stop halts the arm part-way through the leg, the resume must drive
on toward `leg_end`, **not** back to `leg_start` and then forward again (a jerk,
and motion the operator would not expect in a shared workspace). `MotionLogic`
drops the nominal waypoints the robot has already passed (those no closer to the
goal than the current state) and resumes from the current state, so progress
toward the goal is monotonic. The goal is always retained.

### AT-3r — a fast crossing cannot skip the stop
Expressed as a sampling invariant. Even at the maximum modelled operator speed
(`risk.yaml` `v_h`), the displacement per safety-monitor tick
(`v_h / SafetyMonitorNode._TICK_HZ`) is well below the width of the red band
`[0, d_red]`. So as a fast operator approaches head-on, several monitor samples
fall inside the red band before contact — the monitor cannot "tunnel" from
outside the band to contact between two ticks without classifying `red` and
commanding the protective stop. The inputs are read from the real configuration
and node design (`risk.thresholds`, `_TICK_HZ`), never hard-coded.

### AT-5r — transient detection loss recovery
A transient loss of the operator's pose (occlusion, a dropped detection) must
fail safe and then recover. The harness freezes `human_node` (SIGSTOP) so the
human TF goes stale: after `loss_timeout` the monitor passes `d=None` to
`classify()`, which returns `("lost", 0.0)` — a protective stop; the last known
position is never reused (FR-9). On SIGCONT the TF returns and the loop must
re-acquire and resume (the scale recovers), with no node faulting through the
cycle (asserted by the post-shutdown exit-code check).

## How to reproduce

Unit guarantees (AT-4r, AT-3r) — fast, no simulator:

```bash
# inside the deploy image (or any env with the safecollab package installed)
python -m pytest test/unit/test_motion_node.py test/unit/test_fast_crossing.py -v
```

Live guarantees (AT-6, AT-5r) — headless cell bring-up, inside the deploy image
(the same path CI's `scenario` stage runs):

```bash
docker build -t safecollab:ci .
docker run --rm --init safecollab:ci \
  launch_test src/safecollab/test/scenario/test_robustness.py
```

The full perceived-path validation (camera → perception → SSM) is separate; see
[`VALIDATE.md`](VALIDATE.md).
