"""TDD spec for safecollab.retime -- the "speed knob" (pure math, no ROS).

retime() rescales speed by STRETCHING the time_from_start of each trajectory
point by 1/scale. scale == 0.0 is a protective stop and must be guarded (no
division by zero).
"""
import pytest

from safecollab.retime import retime


def test_full_speed_leaves_times_unchanged():
    times = [0.0, 1.0, 2.0, 3.0]
    assert retime(times, 1.0) == pytest.approx(times)


def test_half_speed_doubles_times():
    times = [0.0, 1.0, 2.0]
    assert retime(times, 0.5) == pytest.approx([0.0, 2.0, 4.0])


def test_quarter_speed_quadruples_times():
    assert retime([0.0, 1.0], 0.25) == pytest.approx([0.0, 4.0])


def test_protective_stop_returns_none_no_div_by_zero():
    # scale == 0.0 -> protective stop; guarded, must not raise ZeroDivisionError.
    assert retime([0.0, 1.0, 2.0], 0.0) is None


def test_preserves_length_and_monotonicity():
    times = [0.0, 0.5, 1.0, 2.5, 5.0]
    out = retime(times, 0.3)
    assert len(out) == len(times)
    assert out == sorted(out)


def test_relative_spacing_scales_proportionally():
    out = retime([0.0, 1.0, 4.0], 0.5)
    assert out[2] / out[1] == pytest.approx(4.0)


def test_empty_trajectory_returns_empty():
    assert retime([], 0.7) == []


@pytest.mark.parametrize("bad_scale", [-0.1, 1.1, 2.0, -1.0])
def test_scale_out_of_range_raises(bad_scale):
    with pytest.raises(ValueError):
        retime([0.0, 1.0], bad_scale)
