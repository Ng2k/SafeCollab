"""AT-3 hardening (P4): a fast operator crossing cannot skip the protective stop.

The live AT-3 (``test/scenario/test_at1_at5.py``) already proves that an operator
inside the red zone stops the arm. This module proves the *fast-crossing*
robustness as a sampling invariant, with no ROS/sim bring-up:

Even at the maximum modelled operator speed (``risk.yaml`` ``v_h``), the
operator's displacement per safety-monitor tick (``v_h / monitor_rate``) is small
relative to the width of the red band ``[0, d_red]``. So as a fast operator
approaches head-on, several monitor samples fall inside the red band before
contact — the monitor cannot "tunnel" from outside the band to contact between
two ticks without ever classifying 'red' and commanding the protective stop.

The inputs are taken from the real configuration / node design (no hard-coded
duplicates): ``v_h`` and ``d_red`` come from ``config/risk.yaml`` via
``risk.thresholds`` (ground rule 5: thresholds are computed, never pasted), and
the monitor rate from ``SafetyMonitorNode._TICK_HZ``.
"""

from pathlib import Path

from safecollab.safety.risk import load_config, thresholds
from safecollab.safety.node import SafetyMonitorNode

RISK_YAML = Path(__file__).resolve().parents[2] / "config" / "risk.yaml"

#: Operator-uncertainty z_d for the worked-example thresholds (matches test_risk).
_Z_D = 0.05

#: Minimum monitor samples that must fall inside the red band during a worst-case
#: (max-speed) head-on approach, so a fast crossing reliably triggers the stop.
_MIN_RED_SAMPLES = 2


def _d_red() -> float:
    cfg = load_config(RISK_YAML)
    d_red, _ = thresholds(cfg, z_d=_Z_D)
    return d_red


def test_max_speed_step_per_tick_is_below_red_band():
    """v_h / monitor_rate must be < d_red, so no single tick can cross the band."""
    cfg = load_config(RISK_YAML)
    d_red = _d_red()
    step = cfg.v_h / SafetyMonitorNode._TICK_HZ
    assert step < d_red, (
        f"per-tick approach step {step:.3f} m (v_h={cfg.v_h} m/s / "
        f"{SafetyMonitorNode._TICK_HZ:.0f} Hz) must be below d_red={d_red:.3f} m, "
        "or a fast crossing could step from outside the red band to contact "
        "between two monitor ticks without ever sampling 'red'."
    )


def test_fast_crossing_yields_multiple_red_samples():
    """At max operator speed, several samples land inside [0, d_red] before contact."""
    cfg = load_config(RISK_YAML)
    d_red = _d_red()
    step = cfg.v_h / SafetyMonitorNode._TICK_HZ
    red_samples = int(d_red // step)
    assert red_samples >= _MIN_RED_SAMPLES, (
        f"only ~{red_samples} monitor sample(s) fall inside the red band at max "
        f"speed (d_red={d_red:.3f} m / step={step:.3f} m); need "
        f">= {_MIN_RED_SAMPLES} so a fast crossing reliably triggers the "
        "protective stop within a tick of entering the band."
    )
