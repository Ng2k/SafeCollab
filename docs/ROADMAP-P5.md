# P5 — Polish + Demo → `v0.5.0`

> Phase plan for the SafeCollab **polish & demo** sprint. Same discipline as P4:
> TDD + agile increments, clean/readable code, single-branch / merge-when-green.
> Follows the `AGENTS.md` ground rules throughout (TDD §2.1, ≥90 % coverage §2.2,
> config-not-constants §2.5, no AI-attribution trailers §2.7, shippable per sprint
> §2.8, testing layers §9, acceptance gate §12, cut-scope ladder §11, release §7b).

## Goal (AGENTS §10, P5 row)

Make the **already-working, already-robust** SSM loop *legible* — a person glancing
at the screen for five seconds must understand the safety state — and capture a
**reproducible demo** that shows the headline behaviour end to end:

1. **RViz zone visualisation** — the live safety state (`green / yellow / red /
   lost`) is visible in RViz: the existing `/viz/safety_marker` sphere renders, the
   robot model + camera image are framed, and a text label shows the current zone.
2. **Console HUD** — a compact, always-on terminal readout of `zone`, `scale`, and
   `min_distance` that passes a **5-second legibility check** (a viewer names the
   current safety state within five seconds).
3. **Demo capture** — a recorded **video / GIF**, driven by the **perceived** human
   (not ground truth, §12.5), showing one full `green → yellow → red →` protective
   stop `→ resume` cycle **and** a fail-safe (transient detection loss → `lost` →
   re-acquire). Committed as a repo asset and linked from the docs.

### What already exists (build on it, don't duplicate)

- `safety_monitor` already publishes `/viz/safety_marker`
  (`visualization_msgs/Marker`) with the per-zone RGBA in `_ZONE_RGBA`, radius from
  `config/safety.yaml` (`marker_radius`). **Reuse it** — P5 makes it *visible*, it
  does not re-implement it.
- `config/view.rviz` exists (RobotModel + Camera on `/camera/image`).
- **Gaps:** `cell.launch.py` launches **no `rviz2` node** (headless-only), the
  `view.rviz` has **no Marker/zone-label display**, and there is **no console HUD**.

### Definition of Done (gates the merge)

- `rviz:=true` brings up RViz showing the robot, camera, the zone sphere, and a zone
  text label — verified live (screenshot in the PR). Headless stays the CI default.
- Console HUD node ships, unit-tested on its **pure formatting logic**, and passes
  the 5-second legibility check (recorded in the PR).
- A demo video/GIF (perceived-driven, incl. fail-safe) is committed and linked.
- ≥ 90 % coverage on the safety/risk modules still holds (ground rule 2); any new
  pure logic (HUD formatter) is covered.
- Every new knob (HUD colours/precision, marker label offset, rviz toggle default)
  lives in `config/*.yaml` or a launch arg, **not** in code literals (ground rule 5).
- Full six-stage CI green on the branch.
- Clean, readable diffs (the "refactor" step is not optional).

## Branch & CI workflow

- One branch off `main`: **`feat/p5-polish`** (created from the v0.4.0 `main`).
- Open a **draft PR immediately** so the **full CI pipeline runs on every push** —
  the per-sprint "visible result" that keeps the branch continuously green.
- **Merge to `main` only at the end**, once all six CI stages are green
  (lint → build → unit + coverage → integration → package → deliver). Then tag
  `v0.5.0` per §7b.
- Conventional Commits (`feat:` / `fix:` / `test:` / `refactor:` / `docs:`).
  **Never** add a `Co-Authored-By` or any AI-attribution trailer (ground rule 7).

> The visual deliverables (RViz, HUD, demo) are inherently non-headless; CI still
> only runs the **headless** path, so every visual sprint also lands a *headless,
> testable* slice (a pure formatter unit test, a marker-publish assertion) so the
> increment is provable in CI, not only by eyeballing.

## Method applied to *every* sprint (ground rules 1 & 8, §9)

**Red → Green → Refactor**, each sprint ending as a **shippable, demoable increment**
with the branch green:

1. **Red** — write the failing test first (unit for pure logic — e.g. the HUD
   formatter, the marker colour/label mapping; `test/integration` for a
   marker-published assertion).
2. **Green** — the minimum code to pass.
3. **Refactor** — clean/readable pass (names, small functions, no duplication);
   tests still green.
4. **Visible result** — a screenshot, a HUD capture, or the demo asset, recorded in
   the PR (this is the phase where "visible" is literal).

---

## Sprints

### Sprint 0 — Scaffold & baseline (thin slice, proves the pipeline)

- Create `feat/p5-polish` from `main`; open the draft PR.
- Land this `docs/ROADMAP-P5.md` and an `[Unreleased]` CHANGELOG stub.
- Audit the existing marker (`_ZONE_RGBA`, `/viz/safety_marker`) and `view.rviz` so
  later sprints extend them rather than duplicate.
- **Deliverable / visible:** draft PR with **green CI** and the P5 plan in place.
- Commit: `docs: P5 polish & demo roadmap`.

### Sprint 1 — RViz zone visualisation (`rviz:=true`)

- **Red:** integration test asserting `safety_monitor` publishes `/viz/safety_marker`
  with the **zone-correct colour** (reuse the rclpy recorder; assert the marker
  colour matches `_ZONE_RGBA` for the observed zone). Headless, CI-provable.
- **Green:** add an `rviz:=true` launch arg to `cell.launch.py` (default `false`, so
  CI/headless is unchanged) that starts `rviz2 -d config/view.rviz`; extend
  `view.rviz` with a **Marker display** on `/viz/safety_marker` and a zone **text
  label** (a `Marker`/`MarkerArray` `TEXT_VIEW_FACING` from `safety_monitor`, colour
  + string from the zone — radius/offset/scale from `config/safety.yaml`).
- **Refactor:** factor the marker construction in `safety_monitor` so the sphere and
  the text label share the zone→RGBA mapping (no duplicated colour table).
- **Visible:** screenshot of RViz showing robot + camera + zone sphere + label
  during an escalation.
- Commits: `test(integration): assert the safety marker is zone-coloured`
  → `feat(viz): rviz:=true with zone sphere + text label`.

### Sprint 2 — Console HUD (5-second legibility)

- **Red:** unit-test a **pure formatter** (`hud.py`, no ROS) mapping
  `(zone, scale, min_distance)` → a single, aligned status line (zone word, colour
  code, `scale` as a percent, `min_distance` in metres). Test edge cases:
  `lost`/`None` distance, `scale` 0 and 1, width/alignment stable.
- **Green:** a small `hud_node` subscribing `/safety/{zone,scale,min_distance}`
  (QoS matched to the §3 contract) that prints the formatted line on each update;
  colours/precision/refresh from `config/hud.yaml` (ground rule 5). Register the
  console entry point in `setup.py`. Add `hud:=true` to `cell.launch.py`
  (default `false`).
- **Refactor:** keep the node a thin I/O shell over the tested formatter.
- **Visible:** a captured terminal HUD across a `green→red→green` cycle; run the
  **5-second legibility check** and record the result in the PR.
- Commits: `test(hud): pure status-line formatter` → `feat(hud): console safety HUD`.

### Sprint 3 — Demo capture (perceived-driven, incl. fail-safe)

- **Red/Setup:** add `scripts/record-demo.sh` that brings up the cell with
  `safety_source:=perception rviz:=true hud:=true`, drives the perceived human, and
  records the screen to `docs/media/`. Keep it deterministic (fixed `path_seed`).
- **Green:** capture one full `green → yellow → red →` stop `→ resume` cycle **and**
  a transient detection loss (`lost` → re-acquire), with the HUD + RViz visible.
  Produce a compressed **GIF** (and/or short MP4) committed under `docs/media/`.
- **Visible:** the committed GIF; embed it in `docs/DEMO.md`.
- Commits: `feat(demo): record-demo script` → `docs(demo): capture GIF incl. fail-safe`.

### Sprint 4 — Clean-code pass + docs + coverage

- Refactor the touched code for readability; dedupe the zone→colour mapping shared
  by the marker, the label, and the HUD into one source of truth.
- Confirm **≥ 90 % coverage on safety/risk** and that the HUD formatter is covered.
- Confirm **all polish params live in config / launch args** (ground rule 5).
- Add `docs/DEMO.md` (how to reproduce the demo + the legibility rationale) and a
  README pointer to the GIF.
- **Visible:** full unit + scenario suite green locally; coverage report ≥ 90 %.
- Commits: `refactor(viz,hud): single zone→colour source`, `docs: demo & HUD notes`.

### Sprint 5 — Release `v0.5.0` (§7b)

- Push; confirm **all six CI stages green on the branch**, then **merge to `main`**.
- Move `[Unreleased]` → `## [0.5.0]`, tag `v0.5.0`, verify the **deliver** job
  published the Release + sliced changelog.

---

## Cut-scope ladder if time-pressured (§11, shed from the top)

1. Ship a **GIF only** (drop the MP4) — still demonstrates the loop + fail-safe.
2. HUD without ANSI colour (plain aligned text) if a terminal mangles colours — the
   word `RED`/`LOST` still passes the legibility check.
3. Defer `docs/DEMO.md` prose to P6 (keep the committed GIF + a one-line pointer).
4. **Never cut:** the perceived-driven demo showing a protective stop **and** a
   fail-safe re-acquire (§12.5) — that is the headline result.

---

## Per-sprint Definition of Done checklist

- [ ] Failing test written first (Red), then minimal code (Green), then cleanup (Refactor).
- [ ] Branch CI green after the push (all six stages).
- [ ] Visible result captured in the draft PR (screenshot / HUD capture / GIF).
- [ ] No new hand-tuned constants in code; tunables in `config/*.yaml` or launch args.
- [ ] Coverage gate (≥ 90 % on safety/risk) still satisfied.
- [ ] Conventional Commit message, no AI-attribution trailer.
