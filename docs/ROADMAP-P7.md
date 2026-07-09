# P7 — Refactoring & Optimization → `v0.7.0`

> Phase plan for the SafeCollab **refactoring & optimization** sprint. Goal:
> reduce technical debt — readability, maintainability, and comment quality —
> **without changing any behaviour or adding any feature**. Follows the ground
> rules throughout (≥ 90 % coverage on safety/risk, config-not-constants, no
> AI-attribution trailers, the release procedure).

> **Status: ✅ complete — shipped as `v0.7.0` (2026-07-02).** All seven sprints
> below landed and the single PR (`refactor/v0.7.0`) merged. A follow-up
> `refactor/docker` pass (image 4.07 → 3.52 GB, cached rebuilds, self-contained
> plain-build overlay, plus the `config_path` and planner-teardown fixes) merged
> as **PR #11**. This document is retained as the phase record; the plan below is
> historical. **Next up:** demo & docs polish (`docs/demo-polish`), then the
> clean-machine dry-run and `v1.0.0` submission noted under *Version note* below.

## Working agreement (this phase)

- **One branch:** `refactor/v0.7.0`, **one commit per sprint** (each independently
  revertible).
- **One final pull request**, opened only **after every test is green AND the
  demo has been run and looks correct**. No intermediate PRs.
- **No behavioural change, no new features.** Pure structure / readability / debt.
- **Green at every step** — a sprint isn't done until the gate below passes.
- **No new tests to police style or comments** (existing suites are the net).

## Baseline (measured 2026-07-02)

- Unit: **288 passed**, **99.12 %** coverage (gate ≥ 90 %).
- The coverage is entirely the **pure-logic layer**. Every ROS/MoveIt wrapper is
  `# pragma: no cover` (`MotionNode`, `SafetyMonitorNode`, `PerceptionNode`, all
  six `main()`s, `human_node.HumanNode`, `planner_node.main`). Those are guarded
  **only** by the live `launch_test` integration + scenario suites.
- **Consequence that drives sprint ordering:** refactors in pure-logic code are
  caught by the fast unit gate; refactors in `pragma:no-cover` code are caught
  **only** by the slower, occasionally-flaky live suites. Front-load the former;
  keep the latter small, late, and individually shippable.

## The verification gate (run before considering any sprint done)

```bash
# Fast inner loop (pure logic) — run on every change:
ruff check . && ruff format --check .
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -p pytest_cov \
  -p no:cacheprovider src/safecollab/test/unit \
  --cov=safecollab --cov-report=term-missing --cov-fail-under=90

# Live gate (required before finishing any sprint that touches pragma:no-cover code):
docker build -t safecollab:ci .
docker run --rm safecollab:ci launch_test src/safecollab/test/integration/test_bringup.py
docker run --rm safecollab:ci launch_test src/safecollab/test/scenario/test_at1_at5.py
docker run --rm safecollab:ci launch_test src/safecollab/test/scenario/test_robustness.py
```

---

## Debt inventory

| # | Theme | Severity | Where |
|---|---|---|---|
| D1 | Stale/false comments describing a build state that no longer exists | **High** | `perception_node`, `safety_monitor` "Stream C/G not yet merged" |
| D2 | Copy-pasted lifecycle boilerplate: SIGINT teardown guard ×5, QoS profiles ×3, rclpy import-guard ×4, config-path idiom ×3 | **High** | all nodes |
| D3 | Comment over-narration: historical bug post-mortems inline ("the old X", "this was the original bug") | Medium | `perception`, `planner`, `human`, `motion` |
| D4 | Structural inconsistency: `human_node` nests its class in `main()`; `planner_node.main` is a 200-line monolith of nested closures | Medium | `human_node`, `planner_node` |
| D5 | Metadata debt: `setup.py` `version="0.1.0"` (repo is 0.6.0), `maintainer_email="TODO@example.com"`, `license="TODO"` | Medium | `setup.py` |
| D6 | Local imports that belong at module top (`import math as _math` ×2, `import time`, `import rclpy as _rclpy`) | Low | `perception`, `planner`, `human`, `motion` |
| D7 | Typing-style drift: `safety_monitor` uses `List/Optional/Tuple`; rest use PEP 585 `list[…]` + `X \| None` | Low | `safety_monitor` |
| D8 | Dead/decorative params: `estimate_uncertainty(area_px=…)` accepted but unused | Low | `perception_node` |

### Comment audit taxonomy

- **KEEP** (non-recoverable "why"): joint-order realignment (`motion_node`),
  gz-transport-on-a-worker-thread rationale (`human_node`), deterministic-IK-cache
  rationale (`planner_node`), FR-9 "never dead-reckon" notes.
- **TRIM** (past, not present): confidence post-mortem (`perception_node:261-269`),
  timeout post-mortem (`planner_node:303-311`), dwell phase-lock paragraphs
  (`human_node`). Reduce to one-line "what"; the history belongs in CHANGELOG/PR.
- **DELETE** (stale & now false): the "Stream C/G not yet merged / until that file
  exists" blocks — both `perception_node.py` and `cell.launch.py` exist.

---

## Sprints (lowest-risk first)

### Sprint 0 — Baseline & net (done)
Record the gate output (288 / 99.12 %); confirm the live suites are green on `main`
before touching anything. **Risk: none.**

### Sprint 1 — Delete the lies + fix metadata *(docs/config only)*
Kill D1 + D5: remove the stale "Stream C/G" comment blocks; fix `setup.py`
`version`/`maintainer_email`/`license`/`TODO`s. **Net:** unit gate + lint.
**Risk: negligible.**

### Sprint 2 — Extract lifecycle boilerplate *(highest leverage)*
Kill D2. New `safecollab/_ros_runtime.py`: shared QoS profiles, a single
`spin_and_shutdown(node)` (the teardown-race guard, written once), and
`config_path(name)`. Apply to the 4 module-level nodes. The integration suite's
exit-code assertions are what prove this. **Net:** unit + **live gate mandatory**.
**Risk: medium, fully covered by integration.**

### Sprint 3 — Comment & readability pass on pure logic *(unit-gated)*
D3 + D6 + D7, confined to unit-covered code: trim post-mortems to one-liners;
hoist local imports; normalise `safety_monitor` typing to PEP 585. **Net:** unit +
lint. **Risk: low.**

### Sprint 4 — Unify `human_node` onto the standard node shape
D4 part 1: lift `HumanNode` + its gz-transport guard out of `main()` to module
scope, behind the same import guard the other nodes use; reuse `spin_and_shutdown`.
**Net:** unit + **live gate mandatory** (class body is `pragma:no-cover`).
**Risk: medium.**

### Sprint 5 — Tame `planner_node.main` *(riskiest; cut-scope valve)*
D4 part 2: extract the nested closures (`resolve_joints`, `plan_and_publish`,
`wait_until_reached`) into named methods on a `PlannerRuntime` class —
**behaviour-preserving extraction only**. **Do NOT** touch IK seeding, tolerances,
timeouts, or dwell values (the scenario suite asserts their emergent effects).
**Net:** scenario suite only → run it 2–3× to rule out flakiness. **Risk: high —
ship alone, revert-friendly.**

### Sprint 6 — Micro-optimization + release prep
Honest, small (this is a 20 Hz loop, not a hot path): drop dead `area_px` (D8) iff
tests don't pin the signature; compare squared distances in `_min_distance` to skip
a per-frame `sqrt`. Finalise CHANGELOG `[0.7.0]`. **Net:** unit gate. **Risk: low.**

### Release
After all sprints are green **and the demo has been run and verified by hand**,
open the single PR, let the six-stage CI pass, merge, tag `v0.7.0`, and confirm the
`deliver` job publishes the Release.

## Do NOT touch (guardrails)

- Tuning constants: `risk.yaml`, IK tolerances (`_REACH_TOL`, `err<0.02`),
  dwell/timeout values, `STANDING_DWELL_S`, Pilz params — these are *behaviour*.
- The `*Logic` public method signatures the unit tests rely on — refactor
  internals freely, keep the seams.
- The load-bearing "why" comments in the KEEP bucket.

## Version note

State-based tagging (tracks capability, not the planned index): the sequence ran
`v0.4.0`=P4 → `v0.5.0`=P5 → `v0.6.0`=P6, so this phase is **`v0.7.0`**. The
clean-machine dry-run and `v1.0.0` final submission move to the phase after.
