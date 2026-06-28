# P4 — Robustness → `v0.4.0`

> Phase plan for the SafeCollab **robustness** sprint. Written around TDD + agile
> increments, clean/readable code, and a single-branch / merge-when-green workflow.
> Follows the `AGENTS.md` ground rules throughout (TDD §2.1, ≥90 % coverage §2.2,
> config-not-constants §2.5, no AI-attribution trailers §2.7, shippable per sprint
> §2.8, testing layers §9, acceptance gate §12, cut-scope ladder §11, release §7b).

## Goal (AGENTS §10, P4 rows)

Harden the **already-working** SSM loop so it generalizes and recovers under adverse
conditions — proven by acceptance tests, with **no path-specific tuning and no new
hand-tuned constants**:

1. **AT-6** — randomised operator paths (×5) all produce correct
   `green → yellow → red` / resume behaviour.
2. **AT-4 hardening** — recover cleanly from a protective stop triggered
   **mid-trajectory** (not only at a waypoint boundary).
3. **AT-3 hardening** — a **fast crossing** drives the zone to `red` and
   `/safety/scale → 0` within one control tick.
4. **AT-5 hardening** — a **transient detection loss** (occlusion) →
   `lost` / protective-stop → **re-acquire → clean resume**, with no node faulting.

### Definition of Done (gates the merge)

- AT-1…AT-6 green (unit + scenario).
- ≥ 90 % coverage on the safety/risk modules (ground rule 2).
- Every robustness knob lives in `config/*.yaml`, not in code (ground rule 5).
- Full six-stage CI green on the branch.
- Clean, readable diffs (the "refactor" step is not optional).

## Branch & CI workflow

- One branch off `v0.3.0` `main`: **`feat/p4-robustness`**.
- Open a **draft PR immediately** so the **full CI pipeline runs on every push** —
  that is the per-sprint "visible result" and keeps the branch continuously green.
- **Merge to `main` only at the end**, once all six CI stages are green
  (lint → build → unit + coverage → integration → package → deliver). Then tag
  `v0.4.0` per §7b.
- Conventional Commits (`test:` / `feat:` / `fix:` / `refactor:` / `docs:`).
  **Never** add a `Co-Authored-By` or any AI-attribution trailer (ground rule 7).

> Note: AGENTS prefers short-lived branches; we deliberately use one longer-lived
> P4 branch here, but a draft PR with CI-on-every-push keeps it green and visible,
> satisfying ground rule 8 at sprint granularity.

## Method applied to *every* sprint (ground rules 1 & 8, §9)

**Red → Green → Refactor**, each sprint ending as a **shippable, demoable increment**
with the branch green:

1. **Red** — write the failing test first (unit for pure logic; `test/scenario/`
   for emergent AT behaviour).
2. **Green** — the minimum code to pass.
3. **Refactor** — clean/readable pass (names, small functions, no duplication);
   tests still green.
4. **Visible result** — a passing AT with printed evidence, or a
   `scripts/validate-perceived.sh` run, recorded in the PR.

---

## Sprints

### Sprint 0 — Scaffold & baseline (thin slice, proves the pipeline)

- Create `feat/p4-robustness` from `main`; open the draft PR.
- Audit the existing `test/scenario/` AT harness so the new "Red" tests build on
  what is there rather than duplicating it.
- Add `test/scenario/test_robustness.py` skeleton with the four new cases marked
  `pytest.mark.skip("P4: pending")`, plus an `[Unreleased]` CHANGELOG stub.
- **Deliverable / visible:** draft PR with **green CI** and the new cases collected
  (skips shown).
- Commit: `test(scenario): scaffold P4 robustness acceptance cases`.

### Sprint 1 — AT-6: randomised paths generalize (×5)

- **Red:** parametrize a scenario test over ≥ 5 `path_seed` values; assert each run
  exhibits `green → yellow → red` then resume using the **same** `risk.yaml`
  thresholds (no per-seed tuning).
- **Green / Refactor:** fix whatever a seed exposes (likely in `human_node` path
  generation or timing); keep `task_node` zone-agnostic (§3).
- **Visible:** test output listing all 5 seeds passing.
- Commits: `test(scenario): AT-6 randomised paths across 5 seeds` → `fix(...)` as needed.

### Sprint 2 — AT-4 hardening: mid-trajectory stop → clean resume

- **Red:** unit test on `MotionLogic` injecting `scale = 0` **between** waypoints;
  assert resume **re-plans from the current state at `t = 0`** (no splice/jerk, no
  backtrack); plus a scenario check.
- **Green / Refactor:** harden `motion_node` resume; any timing knob → config, not
  literals.
- **Visible:** resume trajectory asserted smooth; scenario shows stop-then-continue.
- Commits: `test(motion): resume from arbitrary mid-trajectory point`
  → `fix(motion): re-plan resume from current state`.

### Sprint 3 — AT-3 hardening: fast crossing reacts within one tick

- **Red:** scenario/integration test with a high-speed operator crossing; assert
  `/safety/zone → red` and `/safety/scale → 0` within one control period (ties to
  perception ≤ 100 ms + safety reaction).
- **Green / Refactor:** address any latency/ordering gap; express the reaction-time
  bound via config.
- **Visible:** measured stop latency under the bound, printed.
- Commits: `test(scenario): AT-3 fast-crossing stops within one tick` → `fix(...)`.

### Sprint 4 — AT-5 hardening: transient loss → re-acquire → resume

- **Red:** scenario test that forces detection to drop for a window (occlude/hide the
  marker); assert zone → `lost`, `scale → 0`, **no node exits/faults**, then on
  re-acquire zone returns and scale recovers.
- **Green / Refactor:** confirm `perception_node` loss-timeout, `safety_monitor`
  fail-safe, and `motion_node` resume compose cleanly; keep the loss-timeout in
  `config/safety.yaml`.
- **Visible:** lost → recover cycle in the scenario log; a `validate-perceived.sh`
  run still 7/7.
- Commits: `test(scenario): AT-5 transient detection loss and recovery` → `fix(...)`.

### Sprint 5 — Clean-code pass + coverage + config + docs

- Refactor for readability across the touched nodes; dedupe; ensure **≥ 90 %
  coverage on safety/risk**; confirm **all robustness params live in
  `config/*.yaml`** (ground rule 5).
- Add a short **robustness note** (docs) describing the four guarantees and how to
  reproduce them.
- **Visible:** full unit + scenario suite green locally; coverage report ≥ 90 %.
- Commits: `refactor(...)`, `docs(...)`.

### Sprint 6 — Release `v0.4.0` (§7b)

- Push; confirm **all six CI stages green on the branch**, then **merge to `main`**.
- Move `[Unreleased]` → `## [0.4.0]`, tag `v0.4.0`, verify the **deliver** job
  published the Release + sliced changelog.

---

## Cut-scope ladder if time-pressured (§11, shed from the top)

1. FR-12 speed-dependent separation (already out) — keep fixed risk-derived thresholds.
2. Trim AT-6 from 5 seeds to 3 (still proves "no path-specific tuning").
3. Defer the docs robustness note to P6.
4. **Never cut:** AT-1…AT-5 behaviour and fail-safe-on-`lost`.

---

## Per-sprint Definition of Done checklist

- [ ] Failing test written first (Red), then minimal code (Green), then cleanup (Refactor).
- [ ] Branch CI green after the push (all six stages).
- [ ] Visible result captured in the draft PR (test output / validation run).
- [ ] No new hand-tuned constants in code; tunables in `config/*.yaml`.
- [ ] Coverage gate (≥ 90 % on safety/risk) still satisfied.
- [ ] Conventional Commit message, no AI-attribution trailer.
