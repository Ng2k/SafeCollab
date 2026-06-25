"""TDD spec for safecollab.safety_logic.classify (zone classifier, FR-9 fail-safe)."""
import pytest

from safecollab.safety_logic import classify

# Reference thresholds for the table-driven tests (mirror the risk worked example).
D_RED = 0.35
D_YELLOW = 0.85
S_MIN = 0.1


def _classify(d):
    return classify(d, d_red=D_RED, d_yellow=D_YELLOW, s_min=S_MIN)


# --- fail-safe -------------------------------------------------------------

def test_none_distance_is_lost_and_stops():
    # FR-9: perception lost/stale -> fail-safe stop.
    assert _classify(None) == ("lost", 0.0)


# --- green zone ------------------------------------------------------------

def test_far_is_green_full_speed():
    zone, scale = _classify(1.5)
    assert zone == "green"
    assert scale == 1.0


def test_yellow_edge_is_exclusive_equal_yellow_is_green():
    # Boundary semantics: d == d_yellow -> green (yellow edge EXCLUSIVE).
    zone, scale = _classify(D_YELLOW)
    assert zone == "green"
    assert scale == 1.0


# --- red zone --------------------------------------------------------------

def test_close_is_red_protective_stop():
    zone, scale = _classify(0.1)
    assert zone == "red"
    assert scale == 0.0


def test_red_edge_is_inclusive_equal_red_is_red():
    # Boundary semantics: d == d_red -> red (red edge INCLUSIVE).
    zone, scale = _classify(D_RED)
    assert zone == "red"
    assert scale == 0.0


# --- yellow zone / ramp ----------------------------------------------------

def test_yellow_midpoint_ramps_linearly():
    # Midway between the thresholds -> midway on the [s_min, 1.0] ramp.
    midpoint = (D_RED + D_YELLOW) / 2.0
    zone, scale = _classify(midpoint)
    assert zone == "yellow"
    assert scale == pytest.approx(S_MIN + (1 - S_MIN) * 0.5)


def test_yellow_just_inside_red_edge_is_near_s_min():
    zone, scale = _classify(D_RED + 1e-6)
    assert zone == "yellow"
    assert scale == pytest.approx(S_MIN, abs=1e-4)


def test_yellow_just_inside_yellow_edge_is_near_full_speed():
    zone, scale = _classify(D_YELLOW - 1e-6)
    assert zone == "yellow"
    assert scale == pytest.approx(1.0, abs=1e-4)


def test_yellow_scale_is_monotonic_increasing_with_distance():
    samples = [D_RED + step * (D_YELLOW - D_RED) / 10 for step in range(1, 10)]
    scales = [_classify(d)[1] for d in samples]
    assert scales == sorted(scales)
    assert all(S_MIN <= s <= 1.0 for s in scales)
