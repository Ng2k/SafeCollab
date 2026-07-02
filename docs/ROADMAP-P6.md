# P6 — Documentation & Finalisation → `v0.6.0`

> Phase plan for the SafeCollab **documentation & finalisation** sprint. Same
> discipline as P4/P5: agile increments, clean/readable docs, single-branch /
> merge-when-green. Follows the ground rules throughout (≥90 % coverage on
> safety/risk, config-not-constants, no AI-attribution trailers, shippable per
> sprint, the release procedure).

## Goal (§10 P6 row, §12.6)

Produce the **final documentation set** the Definition of Done requires —
**README, architecture image, risk note** — and **finalise the CHANGELOG**, each
consistent with the build. This is the sprint that makes SafeCollab
understandable and submission-ready, immediately ahead of P7's clean-machine
dry-run and the `v1.0.0` submission tag.

DoD §12.6 verbatim: *"README, demo video, architecture image, and risk note
committed and consistent with the build."* The demo GIF already landed in
`v0.5.1`; P6 delivers the remaining three and ties them together.

### What already exists (build on it, don't duplicate)

- `README.md` — project summary + embedded demo GIF (from `v0.5.1`); **thin**, no
  architecture/quickstart depth yet.
- `docs/DEMO.md`, `docs/VALIDATE.md`, `docs/ROBUSTNESS.md`, `docs/ROADMAP-P4.md`,
  `docs/ROADMAP-P5.md` — reuse and cross-link, don't restate.
- `src/safecollab/safecollab/risk.py` + `config/risk.yaml` — the risk note's
  numbers are the ones these compute (`thresholds(cfg, z_d=0.05)` →
  `(d_red, d_yellow) = (0.43, 0.84)`), already covered by `test_risk.py`.
- **Gaps:** no architecture diagram/image; no risk-note document; the CHANGELOG
  is not yet finalised for the phase.

### How this is verified (no new test code)

P6 is documentation — the deliverables are prose and a diagram, not new logic, so
it does **not** add test code. Consistency with the build is kept by:

- **The existing CI** — the *package* stage already verifies the README
  one-command run (§7 stage 5); the full unit suite and the ≥90 % safety/risk
  coverage gate keep `main` green.
- **Sourcing numbers from the code** — the risk note quotes the values
  `risk.py`/`risk.yaml` produce (reproduced by `test_risk.py`), so it is written
  against the model, not hand-invented.
- **Review** — each doc is proofed against the actual node/topic names and the
  run commands before merge.

### Definition of Done (gates the merge)

- `docs/ARCHITECTURE.md` with a **GitHub-native (Mermaid) rendered diagram**
  covering the five application nodes, the two pure modules, and the SSM data
  flow (`perception → safety_monitor → /safety/scale → motion_node`).
- `docs/RISK.md` explaining the ISO/TS 15066 `S_p` model with numbers that match
  `risk.py`.
- `README.md` expanded: what/why, **quickstart (one-command run)**, links to the
  architecture + risk + demo docs, the demo GIF, and a test/CI overview.
- All internal doc links resolve (no dead relative links in a fresh clone —
  note `AGENTS.md` is gitignored, so docs reference it by name, not by link).
- **CHANGELOG finalised** for the release; a tidy `[Unreleased]`.
- ≥ 90 % coverage on safety/risk **still holds**; full six-stage CI green.
- Conventional Commits, **no AI-attribution trailer**.

## Branch & CI workflow

- One branch off `main`: **`docs/p6-docs`**; draft PR early so CI runs each push.
- **Merge to `main` only** once all six stages are green, then tag **`v0.6.0`**.
- **Version note:** the planned ladder pencils `v0.7.0` for P6, but tagging is
  **state-based** and tracks capability. The actual sequence ran `v0.4.0`=P4 →
  `v0.5.0`=P5, so the next MINOR is **`v0.6.0`** (don't skip a number to catch up).

## Method per sprint

Each sprint is a **reviewable increment** with the branch green: draft the doc →
proof it against the code (node/topic names, run commands, derived numbers) →
readability pass → the rendered doc/diagram is the visible result in the PR.

---

## Sprints

### Sprint 0 — Scaffold & baseline (this document)

- Create `docs/p6-docs`; open the draft PR; land this roadmap + an `[Unreleased]`
  note. Commit: `docs: P6 documentation & finalisation roadmap`.

### Sprint 1 — Architecture diagram (`docs/ARCHITECTURE.md`)

- The SSM data-flow Mermaid diagram + a prose walk of each node's responsibility
  and the speed-scaling loop; linked from the README.
- Commit: `docs: architecture diagram and SSM data flow`.

### Sprint 2 — Risk note (`docs/RISK.md`)

- The ISO/TS 15066 `S_p` model, the config inputs, the two scenarios
  (`full_speed` / `reduced`), the worked example, the `z_d`-widens property, and
  the distance → zone → scale mapping — numbers taken from `risk.py`/`risk.yaml`.
- Commit: `docs: ISO/TS 15066 risk note`.

### Sprint 3 — README build-out

- What/why, the one-command run + headless launch, "how it works" links to the
  architecture/risk docs, a testing/CI overview, and a documentation index.
- Commit: `docs: build out the README`.

### Sprint 4 — Cross-link & tidy

- Proof every relative link resolves in a fresh clone; final readability pass.
- Commit: `docs: cross-link and tidy the documentation set`.

### Sprint 5 — Release `v0.6.0`

- Push; confirm **all six CI stages green**, then **merge to `main`**. Move
  `[Unreleased]` → `## [0.6.0]`, tag `v0.6.0`, verify the **deliver** job
  published the Release + sliced changelog.

---

## Cut-scope ladder if time-pressured (shed from the top)

1. Drop any secondary/ASCII diagrams — keep the **one** Mermaid architecture
   diagram (it renders on GitHub, needs no toolchain).
2. Risk note as a **concise table** (derived values + term list); defer the long
   prose derivation to the engineering report.
3. Defer the README test/CI-overview section — keep quickstart + the three doc
   links (they carry §12.6).
4. **Never cut:** an **architecture image**, a **risk note**, and a **README that
   actually runs** (DoD §12.6) — these are the phase's reason to exist.
