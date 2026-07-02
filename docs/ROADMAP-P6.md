# P6 — Documentation & Finalisation → `v0.6.0`

> Phase plan for the SafeCollab **documentation & finalisation** sprint. Same
> discipline as P4/P5: TDD + agile increments, clean/readable docs,
> single-branch / merge-when-green. Follows the `AGENTS.md` ground rules
> throughout (TDD §2.1, ≥90 % coverage §2.2, config-not-constants §2.5, no
> AI-attribution trailers §2.7, shippable per sprint §2.8, testing layers §9,
> definition of done §12, cut-scope ladder §11, release §7b).

## Goal (AGENTS §10 P6 row, §12.6)

Produce the **final documentation set** the Definition of Done requires —
**README, architecture image, risk note** — and **finalise the CHANGELOG**, each
provably **consistent with the build**. This is the sprint that makes SafeCollab
understandable and submission-ready, immediately ahead of P7's clean-machine
dry-run and the `v1.0.0` submission tag.

DoD §12.6 verbatim: *"README, demo video, architecture image, and risk note
committed and consistent with the build."* The demo video/GIF already landed in
`v0.5.1`; P6 delivers the remaining three and ties them together.

### What already exists (build on it, don't duplicate)

- `README.md` — project summary + embedded demo GIF (from `v0.5.1`); **thin**, no
  architecture/quickstart depth yet.
- `docs/DEMO.md`, `docs/VALIDATE.md`, `docs/ROBUSTNESS.md`, `docs/ROADMAP-P4.md`,
  `docs/ROADMAP-P5.md` — reuse and cross-link, don't restate.
- `AGENTS.md §3` (interface contract) and `§4` (the ISO/TS 15066 model) — the
  **source of truth** the architecture diagram and risk note must agree with.
- `src/safecollab/safecollab/risk.py` + `config/risk.yaml` — the risk note's
  numbers must equal what these compute (`thresholds(cfg, z_d=0.05)` →
  `(d_red, d_yellow) = (0.43, 0.84)`).
- **Gaps:** no architecture diagram/image; no risk-note document; no
  doc-consistency tests; the CHANGELOG is not yet finalised for the phase.

### TDD for a documentation phase — *doc-consistency tests*

Docs drift from code silently — the failure mode P6 must prevent. The **testable
slice** for every visual/prose deliverable is a **pure-Python unit test** in
`test/unit/` (so it runs in the CI *unit* stage, needs no new dependency, and
counts toward "main is always green"):

- **Architecture:** the diagram references **every ROS node** and the **§3
  contract topics** — it cannot silently omit or misname a node/topic.
- **Risk note:** the stated `d_yellow` / `d_red` **equal** `risk.thresholds()`
  computed from `config/risk.yaml`; the ISO `S_p` terms and the *"raising `z_d`
  widens both thresholds"* property are documented.
- **README:** the one-command run matches `AGENTS.md §8`; the demo GIF and every
  linked doc resolve.
- **Cross-links:** every relative Markdown link in `README.md` / `docs/*.md`
  points to a file that exists.

**Red → Green → Refactor** applies literally: the test states the doc↔build
contract and **fails** (doc missing/wrong), the doc is written to satisfy it,
then a readability pass — tests still green. These tests add **no** source to the
`safecollab` package, so the §2.2 coverage gate is unaffected.

### Definition of Done (gates the merge)

- `docs/ARCHITECTURE.md` with a **GitHub-native (Mermaid) rendered diagram**
  covering the five application nodes, the two pure modules, and the SSM data
  flow (`perception → safety_monitor → /safety/scale → motion_node`); its
  consistency test is green.
- `docs/RISK.md` explaining the ISO/TS 15066 `S_p` model with numbers that
  **match `risk.py`**; its test is green.
- `README.md` expanded: what/why, **quickstart (one-command run)**, links to the
  architecture + risk + demo docs, the demo GIF, and a test/CI overview; its
  consistency test is green.
- **All internal doc links resolve** (link-check test green).
- **CHANGELOG finalised** for the release; a tidy `[Unreleased]`.
- ≥ 90 % coverage on the safety/risk modules **still holds** (ground rule 2).
- Every new knob (none expected) would live in `config/*.yaml`, not code
  (ground rule 5).
- **Full six-stage CI green** on the branch; clean, readable diffs.
- Conventional Commits, **no AI-attribution trailer** (ground rule 7).

## Branch & CI workflow

- One branch off `main`: **`docs/p6-docs`**.
- Open a **draft PR early** so the full CI pipeline runs on every push.
- **Merge to `main` only** once all six stages are green
  (lint → build → unit + coverage → integration → package → deliver), then tag
  **`v0.6.0`** per §7b.
- **Version note (§7b Step 3):** the §7a *planned* ladder pencils `v0.7.0` for P6,
  but tagging is **state-based** and tracks capability, not the calendar. The
  actual sequence ran `v0.4.0`=P4 → `v0.5.0`=P5, so the next MINOR is **`v0.6.0`**
  (do not skip a number to "catch up" to the planned index).
- Conventional Commits (`test:` / `docs:` / `refactor:`). **Never** add a
  `Co-Authored-By` or any AI-attribution trailer (ground rule 7).

## Method applied to *every* sprint (ground rules 1 & 8, §9)

**Red → Green → Refactor**, each sprint ending as a **shippable, reviewable
increment** with the branch green:

1. **Red** — write the failing doc-consistency test first.
2. **Green** — write the minimum doc content to pass.
3. **Refactor** — clean/readable pass (structure, cross-links, no duplication);
   tests still green.
4. **Visible result** — the rendered doc/diagram in the PR.

---

## Sprints

### Sprint 0 — Scaffold & baseline (this document)

- Create `docs/p6-docs` from `main`; open the draft PR.
- Land this `docs/ROADMAP-P6.md` and an `[Unreleased]` CHANGELOG note.
- **Deliverable / visible:** draft PR with **green CI** and the P6 plan in place.
- Commit: `docs: P6 documentation & finalisation roadmap`.

### Sprint 1 — Architecture diagram (`docs/ARCHITECTURE.md`)

- **Red:** `test_architecture_doc.py` — assert `docs/ARCHITECTURE.md` exists,
  contains a `mermaid` diagram, and references **every** application node
  (`task_node`/`planner_node`, `human_node`, `perception_node`, `safety_monitor`,
  `motion_node`, plus `hud_node`), the two pure modules (`risk`, `safety_logic`),
  and the key **§3 topics** (`/safety/scale`, `/safety/zone`,
  `/human/uncertainty`, `/motion/nominal_trajectory`,
  `/arm_controller/joint_trajectory`, TF `human`).
- **Green:** write `docs/ARCHITECTURE.md`: the SSM data-flow diagram + a prose
  walk of each node's responsibility and the speed-scaling loop.
- **Refactor:** link it from the README stub; dedupe against `AGENTS.md §3`
  (reference, don't copy).
- **Visible:** the rendered Mermaid diagram on the PR.
- Commits: `test(docs): architecture doc references every node and topic`
  → `docs: architecture diagram and SSM data flow`.

### Sprint 2 — Risk note (`docs/RISK.md`)

- **Red:** `test_risk_note.py` — compute `(d_red, d_yellow)` from
  `config/risk.yaml` via `risk.py`; assert `docs/RISK.md` states those exact
  values, lists the six `S_p` terms, and documents that raising `z_d` widens both
  thresholds.
- **Green:** write `docs/RISK.md`: the ISO/TS 15066 `S_p` model, the two
  scenarios (`full_speed` / `reduced`), the worked example, and the `z_d`
  property — all sourced from the config, no hand-typed constants.
- **Refactor:** cross-link from README + ARCHITECTURE.
- **Visible:** the risk note with the derived threshold table.
- Commits: `test(docs): risk note numbers match risk.py` → `docs: ISO/TS 15066
  risk note`.

### Sprint 3 — README build-out

- **Red:** `test_readme.py` — assert the README has the **one-command run**
  (`docker build` + `docker run`) matching `AGENTS.md §8`, the headless launch
  line, links to `docs/DEMO.md` / `docs/ARCHITECTURE.md` / `docs/RISK.md`, and the
  embedded demo GIF.
- **Green:** expand the README: what/why, quickstart, architecture + risk +
  demo links, a short test/CI overview, and the SSM one-liner.
- **Refactor:** tighten prose; ensure every claim traces to a doc.
- **Visible:** the rendered README on the PR.
- Commits: `test(docs): README documents the one-command run and doc links`
  → `docs: build out the README (quickstart, architecture, risk, demo)`.

### Sprint 4 — Cross-link check, tidy & coverage confirm

- **Red:** `test_doc_links.py` — every relative Markdown link in `README.md` and
  `docs/*.md` resolves to an existing file.
- **Green:** fix any dangling links; tidy headings/cross-references.
- **Refactor:** final readability pass across the doc set.
- **Visible:** full unit suite green locally incl. the four new doc tests;
  coverage report ≥ 90 % on safety/risk unchanged.
- Commits: `test(docs): internal Markdown links resolve` → `docs: cross-link and
  tidy the documentation set`.

### Sprint 5 — Release `v0.6.0` (§7b)

- Push; confirm **all six CI stages green on the branch**, then **merge to
  `main`**.
- Move `[Unreleased]` → `## [0.6.0]`, tag `v0.6.0`, verify the **deliver** job
  published the Release + sliced changelog.

---

## Cut-scope ladder if time-pressured (§11, shed from the top)

1. Drop any secondary/ASCII diagrams — keep the **one** Mermaid architecture
   diagram (it renders on GitHub, needs no toolchain).
2. Risk note as a **concise table** (derived values + term list); defer the long
   prose derivation to the engineering report.
3. Defer the README test/CI-overview section — keep quickstart + the three doc
   links (they carry §12.6).
4. **Never cut:** an **architecture image**, a **risk note**, and a **README that
   actually runs** (DoD §12.6) — these are the phase's reason to exist.

---

## Per-sprint Definition of Done checklist

- [ ] Failing doc-consistency test written first (Red), then minimal doc (Green),
      then cleanup (Refactor).
- [ ] Branch CI green after the push (all six stages).
- [ ] Visible result in the draft PR (rendered diagram / doc / README).
- [ ] No new hand-typed constants that duplicate config/code (numbers are derived
      or asserted against the source).
- [ ] Coverage gate (≥ 90 % on safety/risk) still satisfied.
- [ ] Conventional Commit message, no AI-attribution trailer.
