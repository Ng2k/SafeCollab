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

### Added
- _(work in progress toward the next tag)_

### Fixed
- _(none yet)_

---

## [0.4.0] - 2026-07-04 — P3: safety loop closed

### Added
- feat(safety): close the loop — perceived pose → minimum separation → risk zone → speed scale.
- feat(safety): protective stop on red-zone breach and clean resume on retreat (re-plan from current state).
- feat(safety): fail-safe — enter `lost` and stop when perception is lost/stale; resume on re-acquire.

### Fixed
- fix(motion): guard `scale == 0.0` to avoid a division-by-zero in `retime()`.

**Artifact:** `safecollab-v0.4.0.tar.gz` (published on the repo Releases page).

---

## [0.3.0] - 2026-07-02 — P2: perception

### Added
- feat(perception): `perception_node` detects the operator from the simulated camera and
  broadcasts the perceived `human` TF.
- feat(perception): publish `/human/uncertainty` (σ); larger σ widens the risk thresholds (`Z_d`).
- feat(perception): loss-timeout detection feeding the safety layer's `lost` state.

### Changed
- The safety loop now consumes the **perceived** operator pose instead of sim ground truth.

**Artifact:** `safecollab-v0.3.0.tar.gz`.

---

## [0.2.0] - 2026-06-28 — P1: kitting loop

### Added
- feat(task): kitting state machine `GO_TO_BIN → PICK → GO_TO_TRAY → DROP` driving the arm under
  `joint_trajectory_controller` at full speed.
- feat(sim): `human_node` operator model moves through the workspace and reaches into the shared tray.

**Artifact:** `safecollab-v0.2.0.tar.gz`.

---

## [0.1.0] - 2026-06-25 — P0: foundation + CI

### Added
- feat(safety): `safety_logic.classify()` with green/yellow/red zones and the fail-safe-on-`None`
  contract (written test-first).
- feat(risk): `risk.py` ISO/TS 15066 `S_p` model; thresholds computed from `config/risk.yaml`.
- feat(motion): `retime()` trajectory speed-scaling with a `scale == 0` guard.
- feat(ci): Dockerfile, `entrypoint.sh`, GitHub Actions pipeline (lint → build → unit+coverage →
  integration → package → deliver), pre-commit hooks, **≥ 90 % coverage gate** on safety & risk.

**Artifact:** `safecollab-v0.1.0.tar.gz`.

---

## Planned releases

| Tag | Phase | Theme |
|---|---|---|
| `v0.1.0` | P0 | Foundation + CI; pure safety/risk core (TDD) |
| `v0.2.0` | P1 | Collaborative kitting loop |
| `v0.3.0` | P2 | Perception-driven human (TF + uncertainty) |
| `v0.4.0` | P3 | Safety loop closed (zones, stop, resume, fail-safe) |
| `v0.5.0` | P4 | Robustness (randomised paths, recovery, edge cases) |
| `v0.6.0` | P5 | Polish + demo video / GIF |
| `v0.7.0` | P6 | Documentation (README, diagrams, risk note) |
| **`v1.0.0`** | **submission** | **Acceptance AT-1…AT-5 passing; final image artifact + notes** |

Patch releases (`v0.x.Y`) carry bug-fixes between the minor milestones above.

[Unreleased]: https://github.com/<owner>/safecollab/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/<owner>/safecollab/releases/tag/v0.4.0
[0.3.0]: https://github.com/<owner>/safecollab/releases/tag/v0.3.0
[0.2.0]: https://github.com/<owner>/safecollab/releases/tag/v0.2.0
[0.1.0]: https://github.com/<owner>/safecollab/releases/tag/v0.1.0
