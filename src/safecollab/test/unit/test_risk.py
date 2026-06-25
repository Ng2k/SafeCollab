"""TDD spec for safecollab.risk -- ISO/TS 15066 protective-separation model.

Thresholds must be DERIVED from config/risk.yaml at runtime, never hard-coded.
"""

from pathlib import Path

import pytest

from safecollab.risk import load_config, protective_distance, thresholds

RISK_YAML = Path(__file__).resolve().parents[2] / "config" / "risk.yaml"


# --- the S_p formula itself ------------------------------------------------


def test_protective_distance_sums_iso_terms():
    # S_p = S_H + S_R + S_S + C + Z_d + Z_r
    #     = v_h*(t_r+t_s) + v_r*t_r + s_s + c + z_d + z_r
    s_p = protective_distance(
        v_h=1.0, t_r=0.1, t_s=0.2, v_r=0.5, s_s=0.0, c=0.0, z_d=0.0, z_r=0.0
    )
    assert s_p == pytest.approx(1.0 * 0.3 + 0.5 * 0.1)


def test_protective_distance_each_term_adds_distance():
    base = protective_distance(
        v_h=1.0, t_r=0.1, t_s=0.2, v_r=0.5, s_s=0.0, c=0.0, z_d=0.0, z_r=0.0
    )
    with_margins = protective_distance(
        v_h=1.0, t_r=0.1, t_s=0.2, v_r=0.5, s_s=0.05, c=0.1, z_d=0.05, z_r=0.02
    )
    assert with_margins == pytest.approx(base + 0.05 + 0.1 + 0.05 + 0.02)


# --- the worked example (AGENTS.md  4) -------------------------------------


def test_worked_example_thresholds_from_yaml():
    cfg = load_config(RISK_YAML)
    d_red, d_yellow = thresholds(cfg, z_d=0.05)
    assert d_yellow == pytest.approx(0.84, abs=1e-9)
    assert d_red == pytest.approx(0.43, abs=1e-9)


def test_thresholds_are_rounded_to_centimetres():
    cfg = load_config(RISK_YAML)
    d_red, d_yellow = thresholds(cfg, z_d=0.05)
    assert d_red == round(d_red, 2)
    assert d_yellow == round(d_yellow, 2)


def test_red_threshold_is_below_yellow():
    cfg = load_config(RISK_YAML)
    d_red, d_yellow = thresholds(cfg, z_d=0.05)
    assert d_red < d_yellow


# --- property: more operator uncertainty widens BOTH zones ------------------


def test_raising_z_d_widens_both_thresholds():
    cfg = load_config(RISK_YAML)
    red_lo, yellow_lo = thresholds(cfg, z_d=0.05)
    red_hi, yellow_hi = thresholds(cfg, z_d=0.20)
    assert red_hi > red_lo
    assert yellow_hi > yellow_lo


def test_z_d_widens_thresholds_monotonically():
    cfg = load_config(RISK_YAML)
    reds, yellows = [], []
    for z_d in (0.0, 0.05, 0.10, 0.20, 0.40):
        d_red, d_yellow = thresholds(cfg, z_d=z_d)
        reds.append(d_red)
        yellows.append(d_yellow)
    assert reds == sorted(reds)
    assert yellows == sorted(yellows)
