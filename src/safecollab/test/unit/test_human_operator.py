"""TDD spec for the operator model (Stream E, human_node.py).

All tests exercise the pure-Python ``OperatorPath``, ``OperatorModel``, and
``Waypoint`` types.  No ROS graph is required.

Operator model contract under test
------------------------------------
* ``OperatorPath.generate_random()`` returns a path whose :meth:`has_tray_reach`
  is always ``True`` (AT-6 — every randomised path covers the shared tray).
* Two calls with different seeds produce *different* paths (randomness works).
* ``OperatorModel.position_at(0)`` returns the starting waypoint position.
* ``position_at`` returns interpolated positions between waypoints.
* ``position_at`` clamps at the last waypoint after path end.
* ``is_complete`` is ``False`` before the path ends and ``True`` after.
* Cell geometry:  tray footprint is x ∈ [0.20, 0.50], y ∈ [-0.20, +0.20].
"""

import random

import pytest

from safecollab.human_node import (
    TRAY_CENTRE,
    TRAY_HALF_X,
    TRAY_HALF_Y,
    OperatorModel,
    OperatorPath,
    Waypoint,
    _interpolate_waypoints,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _path(seed: int = 42) -> OperatorPath:
    """Return a deterministic random path."""
    return OperatorPath.generate_random(rng=random.Random(seed))


def _model(seed: int = 42) -> OperatorModel:
    return OperatorModel(_path(seed))


# ---------------------------------------------------------------------------
# Waypoint
# ---------------------------------------------------------------------------


def test_waypoint_position_property():
    wp = Waypoint(x=1.0, y=2.0, z=3.0, t=5.0)
    assert wp.position == (1.0, 2.0, 3.0)


def test_waypoint_fields_accessible():
    wp = Waypoint(x=0.35, y=0.1, z=0.82, t=3.0)
    assert wp.x == pytest.approx(0.35)
    assert wp.y == pytest.approx(0.1)
    assert wp.z == pytest.approx(0.82)
    assert wp.t == pytest.approx(3.0)


# ---------------------------------------------------------------------------
# OperatorPath construction
# ---------------------------------------------------------------------------


def test_operator_path_requires_nonempty_waypoints():
    with pytest.raises(ValueError, match="requires at least one waypoint"):
        OperatorPath([])


def test_operator_path_stores_waypoints():
    wps = [Waypoint(0.9, 0.6, 1.1, 0.0), Waypoint(0.5, 0.1, 0.9, 3.0)]
    path = OperatorPath(wps)
    assert len(path.waypoints) == 2


def test_operator_path_waypoints_returns_defensive_copy():
    wps = [Waypoint(0.9, 0.6, 1.1, 0.0)]
    path = OperatorPath(wps)
    copy = path.waypoints
    copy.append(Waypoint(0.0, 0.0, 0.0, 99.0))
    assert len(path.waypoints) == 1  # original unchanged


def test_operator_path_duration_is_last_waypoint_time():
    wps = [
        Waypoint(0.9, 0.6, 1.1, 0.0),
        Waypoint(0.5, 0.1, 0.9, 3.0),
        Waypoint(0.35, 0.0, 0.82, 7.5),
    ]
    path = OperatorPath(wps)
    assert path.duration_s == pytest.approx(7.5)


# ---------------------------------------------------------------------------
# OperatorPath.has_tray_reach
# ---------------------------------------------------------------------------


def test_has_tray_reach_true_when_waypoint_inside_footprint():
    cx, cy, _ = TRAY_CENTRE
    wps = [
        Waypoint(0.9, 0.6, 1.1, 0.0),
        Waypoint(cx, cy, 0.9, 5.0),  # inside footprint
    ]
    path = OperatorPath(wps)
    assert path.has_tray_reach() is True


def test_has_tray_reach_false_when_no_waypoint_inside_footprint():
    wps = [
        Waypoint(0.9, 0.6, 1.1, 0.0),
        Waypoint(2.0, 2.0, 1.1, 5.0),  # well outside tray
    ]
    path = OperatorPath(wps)
    assert path.has_tray_reach() is False


def test_has_tray_reach_boundary_inside():
    cx, cy, _ = TRAY_CENTRE
    # Use 99 % of the half-extent so we are clearly inside without hitting
    # floating-point representation issues at the exact boundary.
    wps = [Waypoint(cx + TRAY_HALF_X * 0.99, cy, 0.9, 1.0)]
    path = OperatorPath(wps)
    assert path.has_tray_reach() is True


def test_has_tray_reach_boundary_outside():
    cx, cy, _ = TRAY_CENTRE
    wps = [Waypoint(cx + TRAY_HALF_X + 0.01, cy, 0.9, 1.0)]  # just outside
    path = OperatorPath(wps)
    assert path.has_tray_reach() is False


# ---------------------------------------------------------------------------
# OperatorPath.tray_reach_waypoints
# ---------------------------------------------------------------------------


def test_tray_reach_waypoints_returns_only_inside_footprint():
    cx, cy, _ = TRAY_CENTRE
    inside = Waypoint(cx, cy, 0.82, 3.0)
    outside = Waypoint(2.0, 2.0, 1.1, 0.0)
    path = OperatorPath([outside, inside])
    reaches = path.tray_reach_waypoints()
    assert len(reaches) == 1
    assert reaches[0] is inside


# ---------------------------------------------------------------------------
# OperatorPath.generate_random
# ---------------------------------------------------------------------------


def test_generate_random_returns_operator_path():
    path = _path()
    assert isinstance(path, OperatorPath)


def test_generate_random_path_has_tray_reach():
    """Every randomly generated path must include a tray reach (FR-11 / AT-6)."""
    for seed in range(20):
        path = _path(seed)
        assert (
            path.has_tray_reach()
        ), f"Seed {seed}: path has no waypoint inside the tray footprint"


def test_generate_random_tray_reach_within_tray_bounds():
    """Tray-reach waypoints must be geometrically inside the tray footprint."""
    cx, cy, _ = TRAY_CENTRE
    for seed in range(20):
        path = _path(seed)
        for wp in path.tray_reach_waypoints():
            assert (
                abs(wp.x - cx) <= TRAY_HALF_X + 1e-9
            ), f"Seed {seed}: reach x={wp.x} outside tray x bounds"
            assert (
                abs(wp.y - cy) <= TRAY_HALF_Y + 1e-9
            ), f"Seed {seed}: reach y={wp.y} outside tray y bounds"


def test_generate_random_produces_different_paths_with_different_seeds():
    path_a = _path(seed=1)
    path_b = _path(seed=2)
    # At least one waypoint position should differ
    positions_a = [wp.position for wp in path_a.waypoints]
    positions_b = [wp.position for wp in path_b.waypoints]
    assert positions_a != positions_b


def test_generate_random_is_reproducible_with_same_seed():
    path_a = _path(seed=99)
    path_b = _path(seed=99)
    positions_a = [wp.position for wp in path_a.waypoints]
    positions_b = [wp.position for wp in path_b.waypoints]
    assert positions_a == positions_b


def test_generate_random_has_multiple_waypoints():
    for seed in range(5):
        path = _path(seed)
        assert (
            len(path.waypoints) >= 4
        ), f"Seed {seed}: path has only {len(path.waypoints)} waypoints"


def test_generate_random_approaches_from_both_sides():
    """Across many seeds the operator approaches from both +y and −y sides."""
    start_ys = set()
    for seed in range(40):
        path = _path(seed)
        first_y = path.waypoints[0].y
        start_ys.add("pos" if first_y > 0 else "neg")
    assert "pos" in start_ys, "No seed produced a +y approach"
    assert "neg" in start_ys, "No seed produced a −y approach"


def test_generate_random_duration_is_positive():
    for seed in range(10):
        assert _path(seed).duration_s > 0.0


def test_generate_random_waypoints_have_strictly_increasing_times():
    for seed in range(10):
        times = [wp.t for wp in _path(seed).waypoints]
        assert times == sorted(times), f"Seed {seed}: waypoint times not sorted"
        # First waypoint at t=0
        assert times[0] == pytest.approx(0.0)


def test_generate_random_no_rng_argument_works():
    """generate_random() with no argument must not raise."""
    path = OperatorPath.generate_random()
    assert isinstance(path, OperatorPath)
    assert path.has_tray_reach()


# ---------------------------------------------------------------------------
# _interpolate_waypoints (module-level helper)
# ---------------------------------------------------------------------------


def test_interpolate_at_zero_returns_first_waypoint():
    wps = [Waypoint(1.0, 2.0, 3.0, 0.0), Waypoint(4.0, 5.0, 6.0, 10.0)]
    pos = _interpolate_waypoints(wps, 0.0)
    assert pos == pytest.approx((1.0, 2.0, 3.0))


def test_interpolate_at_end_returns_last_waypoint():
    wps = [Waypoint(1.0, 2.0, 3.0, 0.0), Waypoint(4.0, 5.0, 6.0, 10.0)]
    pos = _interpolate_waypoints(wps, 10.0)
    assert pos == pytest.approx((4.0, 5.0, 6.0))


def test_interpolate_at_midpoint_returns_midpoint():
    wps = [Waypoint(0.0, 0.0, 0.0, 0.0), Waypoint(2.0, 4.0, 6.0, 10.0)]
    pos = _interpolate_waypoints(wps, 5.0)
    assert pos == pytest.approx((1.0, 2.0, 3.0))


def test_interpolate_past_end_returns_last_waypoint():
    wps = [Waypoint(1.0, 2.0, 3.0, 0.0), Waypoint(4.0, 5.0, 6.0, 10.0)]
    pos = _interpolate_waypoints(wps, 999.0)
    assert pos == pytest.approx((4.0, 5.0, 6.0))


def test_interpolate_before_start_clamps_to_first_waypoint():
    wps = [Waypoint(1.0, 2.0, 3.0, 0.0), Waypoint(4.0, 5.0, 6.0, 10.0)]
    pos = _interpolate_waypoints(wps, -5.0)
    assert pos == pytest.approx((1.0, 2.0, 3.0))


def test_interpolate_empty_list_returns_origin():
    pos = _interpolate_waypoints([], 5.0)
    assert pos == (0.0, 0.0, 0.0)


def test_interpolate_across_multiple_segments():
    wps = [
        Waypoint(0.0, 0.0, 0.0, 0.0),
        Waypoint(1.0, 0.0, 0.0, 1.0),
        Waypoint(1.0, 2.0, 0.0, 3.0),
        Waypoint(1.0, 2.0, 4.0, 7.0),
    ]
    # t=0.5 is in segment [0,1]
    assert _interpolate_waypoints(wps, 0.5) == pytest.approx((0.5, 0.0, 0.0))
    # t=2.0 is in segment [1,3]
    assert _interpolate_waypoints(wps, 2.0) == pytest.approx((1.0, 1.0, 0.0))
    # t=5.0 is in segment [3,7]
    assert _interpolate_waypoints(wps, 5.0) == pytest.approx((1.0, 2.0, 2.0))


# ---------------------------------------------------------------------------
# OperatorModel
# ---------------------------------------------------------------------------


def test_operator_model_position_at_start():
    """position_at(0) must match the first waypoint of the path."""
    path = _path()
    model = OperatorModel(path)
    expected = path.waypoints[0].position
    assert model.position_at(0.0) == pytest.approx(expected)


def test_operator_model_position_at_end():
    """position_at(duration) must match the last waypoint."""
    path = _path()
    model = OperatorModel(path)
    expected = path.waypoints[-1].position
    assert model.position_at(path.duration_s) == pytest.approx(expected)


def test_operator_model_position_returns_three_tuple():
    model = _model()
    pos = model.position_at(0.0)
    assert len(pos) == 3


def test_operator_model_position_at_intermediate_is_between_waypoints():
    """Interpolated position is between consecutive waypoints (monotone test)."""
    path = _path()
    model = OperatorModel(path)
    w0 = path.waypoints[0]
    w1 = path.waypoints[1]
    t_mid = (w0.t + w1.t) / 2.0
    pos_start = model.position_at(w0.t)
    pos_mid = model.position_at(t_mid)
    pos_end = model.position_at(w1.t)
    # x-coordinate at midpoint must be between start and end
    x_min = min(pos_start[0], pos_end[0])
    x_max = max(pos_start[0], pos_end[0])
    assert x_min - 1e-9 <= pos_mid[0] <= x_max + 1e-9


def test_operator_model_is_complete_false_before_end():
    path = _path()
    model = OperatorModel(path)
    assert model.is_complete(0.0) is False
    assert model.is_complete(path.duration_s - 0.1) is False


def test_operator_model_is_complete_true_at_end():
    path = _path()
    model = OperatorModel(path)
    assert model.is_complete(path.duration_s) is True


def test_operator_model_is_complete_true_after_end():
    path = _path()
    model = OperatorModel(path)
    assert model.is_complete(path.duration_s + 999.0) is True


def test_operator_model_path_property_returns_current_path():
    path = _path()
    model = OperatorModel(path)
    assert model.path is path


def test_operator_model_path_can_be_replaced():
    path1 = _path(seed=1)
    path2 = _path(seed=2)
    model = OperatorModel(path1)
    model.path = path2
    assert model.path is path2


def test_operator_model_distance_to_point_zero_for_same_point():
    pos = (1.0, 2.0, 3.0)
    assert OperatorModel.distance_to_point(pos, pos) == pytest.approx(0.0)


def test_operator_model_distance_to_point_correct():
    pos = (0.0, 0.0, 0.0)
    target = (3.0, 4.0, 0.0)
    assert OperatorModel.distance_to_point(pos, target) == pytest.approx(5.0)


# ---------------------------------------------------------------------------
# End-to-end: operator follows the path into the tray
# ---------------------------------------------------------------------------


def test_operator_reaches_tray_area_during_path():
    """At the tray-reach time the operator's hand must be inside the tray area."""
    for seed in range(10):
        path = _path(seed)
        model = OperatorModel(path)
        # Find the tray-reach waypoints and check the model position at those times
        for wp in path.tray_reach_waypoints():
            pos = model.position_at(wp.t)
            cx, cy, _ = TRAY_CENTRE
            assert (
                abs(pos[0] - cx) <= TRAY_HALF_X + 1e-9
            ), f"Seed {seed}: x={pos[0]:.3f} outside tray at t={wp.t}"
            assert (
                abs(pos[1] - cy) <= TRAY_HALF_Y + 1e-9
            ), f"Seed {seed}: y={pos[1]:.3f} outside tray at t={wp.t}"
