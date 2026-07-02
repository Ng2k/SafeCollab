# SafeCollab — Risk Note (ISO/TS 15066 SSM)

Why the safety zones are the size they are. The thresholds are **derived from an
ISO/TS 15066 risk model at start-up, never hand-tuned** (ground rule 5): the model
lives in [`risk.py`](../src/safecollab/safecollab/risk.py), its inputs in
[`config/risk.yaml`](../src/safecollab/config/risk.yaml), and the numbers below
are reproduced by `risk.thresholds()` — a unit test
([`test_risk_note.py`](../src/safecollab/test/unit/test_risk_note.py)) fails if
this note ever disagrees with the config. See [`ARCHITECTURE.md`](ARCHITECTURE.md)
for where this sits in the loop and [`AGENTS.md §4`](../AGENTS.md) for the
signatures.

## The protective-separation model

ISO/TS 15066 Speed-and-Separation Monitoring keeps a **protective separation
distance** `S_p` between human and robot. If the measured separation drops below
`S_p`, the robot must slow or stop. SafeCollab uses the standard's sum of
contributions:

```
S_p = S_H + S_R + S_S + C + Z_d + Z_r
```

| Term  | Meaning                                             | In SafeCollab |
|-------|-----------------------------------------------------|---------------|
| `S_H` | distance the human travels during reaction + stop   | `v_h · (t_r + t_s)` |
| `S_R` | distance the robot travels during its reaction time | `v_r · t_r` |
| `S_S` | robot stopping-distance contribution                | `s_s` |
| `C`   | intrusion distance (how far a hand reaches in)      | `c` |
| `Z_d` | operator **position uncertainty**                   | `z_d` — **live from perception (σ)** |
| `Z_r` | robot position uncertainty                           | `z_r` |

`Z_d` is the important one: it is **not** a constant — perception supplies it live
as the detection uncertainty σ, so the zones breathe with how well the camera
currently sees the operator.

## Inputs (`config/risk.yaml`)

Shared cell/sensor properties, plus two robot scenarios that differ **only** in
`v_r` and `t_s` (a robot already moving slowly has less kinetic energy and
genuinely stops faster, so `t_s` shrinks *with* `v_r` — one physical fact, not two
free knobs):

| Input | Value | |
|---|---|---|
| `v_h` | 1.6 m/s | operator approach speed |
| `t_r` | 0.10 s | system reaction time |
| `s_s` | 0.06 m | robot stopping-distance contribution |
| `c`   | 0.10 m | intrusion distance |
| `z_r` | 0.02 m | robot position uncertainty |
| **full speed** | `v_r` 0.50 m/s · `t_s` 0.25 s | governs the **yellow** edge |
| **reduced**    | `v_r` 0.10 m/s · `t_s` 0.02 s | governs the **red** edge |

## Derived thresholds

`d_yellow` is `S_p` computed with the **full-speed** scenario (the distance at
which the robot must *begin* slowing); `d_red` is `S_p` with the **reduced**
scenario (the distance at which it must be *stopped*). With the measured operator
uncertainty `z_d ≈ 0.05 m`:

| `z_d` (m) | `d_red` (m) | `d_yellow` (m) |
|---|---|---|
| **0.05** (nominal) | **0.43** | **0.84** |
| 0.10 (noisier detection) | 0.48 | 0.89 |

**Property:** raising `z_d` **widens both thresholds** (0.05 → 0.10 moves red
0.43 → 0.48 and yellow 0.84 → 0.89) — a less certain operator position makes the
cell more cautious, exactly as SSM intends. This monotonicity is asserted in
`test_risk.py` and `test_risk_note.py`.

## From distance to speed scale

`safety_logic.classify(d, d_red, d_yellow, s_min)` maps the live minimum
separation `d` to a zone and a speed `scale ∈ [0, 1]`:

| Condition | Zone | Scale |
|---|---|---|
| `d is None` (perception lost/stale) | `lost` | `0.0` — fail-safe (FR-9) |
| `d ≥ d_yellow` | `green` | `1.0` — full speed |
| `d_red < d < d_yellow` | `yellow` | ramps `s_min … 1.0` linearly |
| `d ≤ d_red` | `red` | `0.0` — protective stop |

The yellow ramp is `scale = s_min + (1 − s_min)·(d − d_red)/(d_yellow − d_red)`.
The **yellow edge is exclusive** (`d == d_yellow → green`) and the **red edge
inclusive** (`d == d_red → red`). `motion_node` turns this scale into motion by
re-timing the planned trajectory (see [`ARCHITECTURE.md`](ARCHITECTURE.md)); a
`lost` or `red` scale of `0.0` is a protective stop, and the last known operator
position is **never** reused to fabricate a distance.
