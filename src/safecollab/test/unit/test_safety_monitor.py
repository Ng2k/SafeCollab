"""TDD spec for safecollab.safety_monitor — SafetyMonitorLogic (Stream F).

Tests cover the logic ADDED by safety_monitor.py on top of classify() (which
is already tested by Stream B in test_safety.py and is NOT re-tested here):

  - ``load_safety_config()``: reads s_min, loss_timeout, robot_frames,
    marker_radius correctly from safety.yaml.
  - ``SafetyMonitorLogic`` construction: config values stored correctly.
  - ``SafetyMonitorLogic._min_distance()``: computes Euclidean minimum over
    one or more robot frames.
  - ``SafetyMonitorLogic._marker_params()``: correct colour per zone; position
    from human_xyz or world-origin fallback when None; zone field present.
  - ``SafetyMonitorLogic.compute()``:
      * FAIL-SAFE (AT-5): ``human_xyz=None`` → ``d=None`` → ``zone="lost"``,
        ``scale=0.0``.  Called out explicitly as required by the agent spec.
      * Fail-safe: empty robot_frames → same result as human_xyz=None.
      * Green / yellow / red paths with stub robot-frame and human positions.
      * Min-distance picks the closest frame, not just the first.
      * Uncertainty widens thresholds (qualitative: same position changes zone
        when σ grows enough to push d_yellow above the measured distance).

ROS (rclpy, tf2, visualization_msgs) is NOT installed in the pure-Python venv.
Stream C's perception_node is NOT yet merged — all TF data is stub/mocked here.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from safecollab.risk import load_config as load_risk_config
from safecollab.safety_monitor import SafetyMonitorLogic, load_safety_config

# ---------------------------------------------------------------------------
# Paths to the real YAML files (same pattern as test_risk.py)
# ---------------------------------------------------------------------------

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"
_RISK_YAML = _CONFIG_DIR / "risk.yaml"
_SAFETY_YAML = _CONFIG_DIR / "safety.yaml"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def risk_cfg():
    """Load the real risk.yaml config (no side-effects)."""
    return load_risk_config(_RISK_YAML)


@pytest.fixture()
def safety_cfg():
    """Load the real safety.yaml config."""
    return load_safety_config(_SAFETY_YAML)


@pytest.fixture()
def logic(risk_cfg, safety_cfg):
    """A SafetyMonitorLogic instance backed by the real YAML files."""
    return SafetyMonitorLogic(risk_cfg, safety_cfg)


@pytest.fixture()
def minimal_safety_cfg():
    """A minimal in-memory safety config for tests that don't need the file."""
    return SimpleNamespace(
        s_min=0.10, loss_timeout=0.5, robot_frames=["tcp"], marker_radius=0.15
    )


@pytest.fixture()
def minimal_logic(risk_cfg, minimal_safety_cfg):
    """A SafetyMonitorLogic backed by the real risk config + a minimal safety config."""
    return SafetyMonitorLogic(risk_cfg, minimal_safety_cfg)


# ---------------------------------------------------------------------------
# load_safety_config
# ---------------------------------------------------------------------------


class TestLoadSafetyConfig:
    """load_safety_config reads all expected fields from safety.yaml."""

    def test_s_min_is_positive_fraction(self, safety_cfg):
        assert 0.0 < safety_cfg.s_min < 1.0

    def test_loss_timeout_is_positive(self, safety_cfg):
        assert safety_cfg.loss_timeout > 0.0

    def test_robot_frames_is_nonempty_list(self, safety_cfg):
        assert isinstance(safety_cfg.robot_frames, list)
        assert len(safety_cfg.robot_frames) > 0

    def test_robot_frames_contains_expected_names(self, safety_cfg):
        for expected in ("tcp", "link_6", "link_3"):
            assert expected in safety_cfg.robot_frames

    def test_marker_radius_is_positive(self, safety_cfg):
        assert safety_cfg.marker_radius > 0.0

    def test_s_min_value_matches_yaml(self, safety_cfg):
        # safety.yaml sets s_min: 0.10
        assert safety_cfg.s_min == pytest.approx(0.10)

    def test_loss_timeout_value_matches_yaml(self, safety_cfg):
        # safety.yaml sets loss_timeout: 0.5
        assert safety_cfg.loss_timeout == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# SafetyMonitorLogic — construction and properties
# ---------------------------------------------------------------------------


class TestSafetyMonitorLogicInit:
    """SafetyMonitorLogic exposes the right config values."""

    def test_loss_timeout_property(self, logic, safety_cfg):
        assert logic.loss_timeout == pytest.approx(safety_cfg.loss_timeout)

    def test_marker_radius_property(self, logic, safety_cfg):
        assert logic.marker_radius == pytest.approx(safety_cfg.marker_radius)

    def test_loss_timeout_from_minimal_cfg(self, minimal_logic):
        assert minimal_logic.loss_timeout == pytest.approx(0.5)

    def test_marker_radius_from_minimal_cfg(self, minimal_logic):
        assert minimal_logic.marker_radius == pytest.approx(0.15)


# ---------------------------------------------------------------------------
# SafetyMonitorLogic._min_distance  (static method)
# ---------------------------------------------------------------------------


class TestMinDistance:
    """_min_distance returns the Euclidean minimum over all robot frames."""

    def test_single_frame_distance(self):
        # robot at origin, human at (3, 4, 0) => distance = 5
        d = SafetyMonitorLogic._min_distance([(0.0, 0.0, 0.0)], (3.0, 4.0, 0.0))
        assert d == pytest.approx(5.0)

    def test_two_frames_picks_closer_one(self):
        # Frame A at (0, 0, 0), frame B at (1, 0, 0); human at (0.8, 0, 0)
        # d(A) = 0.8, d(B) = 0.2 → min = 0.2
        d = SafetyMonitorLogic._min_distance(
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], (0.8, 0.0, 0.0)
        )
        assert d == pytest.approx(0.2)

    def test_three_frames_picks_minimum(self):
        # TCP at (0,0,0), wrist at (0.5,0,0), elbow at (1.0,0,0)
        # Human at (0.6, 0, 0)
        # distances: 0.6, 0.1, 0.4 → min = 0.1
        d = SafetyMonitorLogic._min_distance(
            [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.0, 0.0)],
            (0.6, 0.0, 0.0),
        )
        assert d == pytest.approx(0.1)

    def test_3d_euclidean_distance(self):
        # Robot frame at (1, 2, 3), human at (4, 6, 3)
        # distance = sqrt((3)^2 + (4)^2 + 0^2) = 5
        d = SafetyMonitorLogic._min_distance([(1.0, 2.0, 3.0)], (4.0, 6.0, 3.0))
        assert d == pytest.approx(5.0)

    def test_zero_distance_when_coincident(self):
        d = SafetyMonitorLogic._min_distance([(1.0, 2.0, 3.0)], (1.0, 2.0, 3.0))
        assert d == pytest.approx(0.0)

    def test_equidistant_frames_returns_that_distance(self):
        # Both frames equidistant from human → still returns that value
        d = SafetyMonitorLogic._min_distance(
            [(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)], (0.0, 0.0, 0.0)
        )
        assert d == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# SafetyMonitorLogic._marker_params  (static method)
# ---------------------------------------------------------------------------


class TestMarkerParams:
    """_marker_params returns correctly coloured markers per zone."""

    def test_green_zone_is_green(self):
        p = SafetyMonitorLogic._marker_params("green", (0.0, 0.0, 0.0), 0.15)
        assert p["r"] == pytest.approx(0.0)
        assert p["g"] == pytest.approx(1.0)
        assert p["b"] == pytest.approx(0.0)

    def test_yellow_zone_is_yellow(self):
        p = SafetyMonitorLogic._marker_params("yellow", (0.0, 0.0, 0.0), 0.15)
        assert p["r"] == pytest.approx(1.0)
        assert p["g"] == pytest.approx(1.0)
        assert p["b"] == pytest.approx(0.0)

    def test_red_zone_is_red(self):
        p = SafetyMonitorLogic._marker_params("red", (0.0, 0.0, 0.0), 0.15)
        assert p["r"] == pytest.approx(1.0)
        assert p["g"] == pytest.approx(0.0)
        assert p["b"] == pytest.approx(0.0)

    def test_lost_zone_is_grey(self):
        p = SafetyMonitorLogic._marker_params("lost", None, 0.15)
        # Grey: r==g==b and all equal 0.5
        assert p["r"] == pytest.approx(0.5)
        assert p["g"] == pytest.approx(0.5)
        assert p["b"] == pytest.approx(0.5)

    def test_active_zones_have_high_alpha(self):
        for zone in ("green", "yellow", "red"):
            p = SafetyMonitorLogic._marker_params(zone, (0.0, 0.0, 0.0), 0.15)
            assert p["a"] > 0.5, f"expected alpha > 0.5 for {zone}"

    def test_lost_zone_has_low_alpha(self):
        p = SafetyMonitorLogic._marker_params("lost", None, 0.15)
        assert p["a"] < 0.6

    def test_position_set_from_human_xyz(self):
        p = SafetyMonitorLogic._marker_params("green", (1.5, -0.3, 0.8), 0.15)
        assert p["x"] == pytest.approx(1.5)
        assert p["y"] == pytest.approx(-0.3)
        assert p["z"] == pytest.approx(0.8)

    def test_position_defaults_to_origin_when_none(self):
        p = SafetyMonitorLogic._marker_params("lost", None, 0.15)
        assert p["x"] == pytest.approx(0.0)
        assert p["y"] == pytest.approx(0.0)
        assert p["z"] == pytest.approx(0.0)

    def test_zone_field_is_correct(self):
        for zone in ("green", "yellow", "red", "lost"):
            p = SafetyMonitorLogic._marker_params(zone, None, 0.15)
            assert p["zone"] == zone

    def test_radius_field_stored(self):
        p = SafetyMonitorLogic._marker_params("green", (0.0, 0.0, 0.0), 0.25)
        assert p["radius"] == pytest.approx(0.25)

    def test_unknown_zone_falls_back_to_grey(self):
        # An unexpected zone string should not raise; grey is the safe fallback
        p = SafetyMonitorLogic._marker_params("unknown_zone", None, 0.15)
        assert p["r"] == pytest.approx(0.5)
        assert p["g"] == pytest.approx(0.5)
        assert p["b"] == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# SafetyMonitorLogic.compute — fail-safe paths (AT-5 / FR-9)
#
# These are the most safety-critical tests.  They verify that when perception
# is lost or robot frame data is absent, the system immediately enters the
# "lost" zone with scale=0.0 — a protective stop.  No grace period, no last-
# known-position assumption.
# ---------------------------------------------------------------------------


class TestComputeFailSafe:
    """FAIL-SAFE: human_xyz=None or empty robot_frames → zone='lost', scale=0.0.

    This directly corresponds to AT-5 and FR-9 ("fail-safe on lost/stale
    detection") from AGENTS.md.  Explicitly called out as required.
    """

    def test_none_human_xyz_gives_lost_zone(self, minimal_logic):
        """AT-5 / FR-9: perception lost → zone='lost', scale=0.0, dist=None."""
        zone, scale, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=None,
            uncertainty=0.05,
        )
        assert zone == "lost", "zone must be 'lost' when human_xyz is None"
        assert scale == pytest.approx(
            0.0
        ), "scale must be 0.0 (protective stop) when lost"
        assert dist is None, "min_dist must be None when human position is unknown"

    def test_none_human_xyz_scale_is_exactly_zero(self, minimal_logic):
        """Protective stop: scale is exactly 0.0, not a small positive value."""
        _, scale, _, _ = minimal_logic.compute(
            robot_frames_xyz=[(0.5, 0.0, 0.0)],
            human_xyz_or_none=None,
            uncertainty=0.0,
        )
        assert scale == 0.0

    def test_empty_robot_frames_gives_lost_zone(self, minimal_logic):
        """No robot frame data → fail-safe stop (cannot compute separation)."""
        zone, scale, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[],
            human_xyz_or_none=(1.0, 0.0, 0.0),
            uncertainty=0.05,
        )
        assert zone == "lost"
        assert scale == pytest.approx(0.0)
        assert dist is None

    def test_both_none_and_empty_frames_gives_lost(self, minimal_logic):
        """Neither perception nor robot frames: definitely lost."""
        zone, scale, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[],
            human_xyz_or_none=None,
            uncertainty=0.05,
        )
        assert zone == "lost"
        assert scale == 0.0
        assert dist is None

    def test_fail_safe_marker_zone_is_lost(self, minimal_logic):
        """Marker params reflect the lost zone when in fail-safe."""
        _, _, _, marker = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=None,
            uncertainty=0.05,
        )
        assert marker["zone"] == "lost"

    def test_fail_safe_marker_position_is_origin(self, minimal_logic):
        """When lost, marker position defaults to world origin (no stale pose)."""
        _, _, _, marker = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=None,
            uncertainty=0.05,
        )
        assert marker["x"] == pytest.approx(0.0)
        assert marker["y"] == pytest.approx(0.0)
        assert marker["z"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# SafetyMonitorLogic.compute — normal operation paths
# ---------------------------------------------------------------------------


class TestComputeNormal:
    """Normal operation: correct zone/scale/dist for green, yellow, red."""

    # Human very far from robot — must be green at full speed
    def test_far_human_is_green_full_speed(self, minimal_logic):
        # Robot TCP at origin; human at (5, 0, 0); d=5 m >> d_yellow (~0.84)
        zone, scale, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(5.0, 0.0, 0.0),
            uncertainty=0.05,
        )
        assert zone == "green"
        assert scale == pytest.approx(1.0)
        assert dist == pytest.approx(5.0)

    # Human very close to robot — must be red, protective stop
    def test_very_close_human_is_red_stop(self, minimal_logic):
        # Robot TCP at origin; human at (0.1, 0, 0); d=0.1 m << d_red (~0.43)
        zone, scale, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(0.1, 0.0, 0.0),
            uncertainty=0.05,
        )
        assert zone == "red"
        assert scale == pytest.approx(0.0)
        assert dist == pytest.approx(0.1)

    # Human at medium distance — must be yellow with ramped scale
    def test_medium_distance_human_is_yellow(self, minimal_logic):
        # With z_d=0.05: d_red=0.43, d_yellow=0.84
        # Mid-point: (0.43 + 0.84) / 2 = 0.635
        zone, scale, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(0.635, 0.0, 0.0),
            uncertainty=0.05,
        )
        assert zone == "yellow"
        assert 0.0 < scale < 1.0

    def test_yellow_scale_respects_s_min_floor(self, minimal_logic):
        # Very close to red edge: scale should be near s_min (0.10) but >= it
        # d = d_red + epsilon  =>  scale ~ s_min
        # With z_d=0.05, d_red=0.43
        zone, scale, _, _ = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(0.44, 0.0, 0.0),  # just above d_red
            uncertainty=0.05,
        )
        assert zone == "yellow"
        assert scale >= 0.10 - 1e-4  # at least s_min

    def test_dist_is_returned_correctly(self, minimal_logic):
        # Robot at (1, 0, 0); human at (1, 3, 4) => d = sqrt(0+9+16) = 5
        _, _, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[(1.0, 0.0, 0.0)],
            human_xyz_or_none=(1.0, 3.0, 4.0),
            uncertainty=0.0,
        )
        assert dist == pytest.approx(5.0)

    def test_min_over_multiple_frames_picks_closest(self, minimal_logic):
        # TCP far, wrist close; min distance should come from the wrist
        # TCP at (0,0,0), wrist at (0.6,0,0)
        # Human at (0.7, 0, 0)
        # d(tcp)=0.7, d(wrist)=0.1 → min=0.1 → red zone
        zone, scale, dist, _ = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0), (0.6, 0.0, 0.0)],
            human_xyz_or_none=(0.7, 0.0, 0.0),
            uncertainty=0.05,
        )
        assert dist == pytest.approx(0.1)
        assert zone == "red"  # 0.1 m << d_red (0.43 m)
        assert scale == pytest.approx(0.0)

    def test_marker_zone_matches_returned_zone(self, minimal_logic):
        for human_pos, expected_zone in [
            ((5.0, 0.0, 0.0), "green"),
            ((0.1, 0.0, 0.0), "red"),
        ]:
            zone, _, _, marker = minimal_logic.compute(
                robot_frames_xyz=[(0.0, 0.0, 0.0)],
                human_xyz_or_none=human_pos,
                uncertainty=0.05,
            )
            assert marker["zone"] == expected_zone
            assert marker["zone"] == zone

    def test_marker_position_matches_human_xyz(self, minimal_logic):
        _, _, _, marker = minimal_logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(1.2, -0.5, 0.9),
            uncertainty=0.05,
        )
        assert marker["x"] == pytest.approx(1.2)
        assert marker["y"] == pytest.approx(-0.5)
        assert marker["z"] == pytest.approx(0.9)


# ---------------------------------------------------------------------------
# SafetyMonitorLogic.compute — uncertainty effect on thresholds
# ---------------------------------------------------------------------------


class TestComputeUncertainty:
    """Higher uncertainty widens thresholds, potentially changing the zone."""

    def test_higher_uncertainty_can_push_zone_to_red(self, risk_cfg):
        # Fixed distance that is yellow at low σ but pushes into red at high σ.
        # With z_d=0.05: d_red=0.43, d_yellow=0.84 (from test_risk.py worked example)
        # Choose a distance at the low end of yellow: d=0.50 m
        # As σ grows, d_red grows too.  At some σ, d_red > 0.50 → zone becomes red.

        # Low uncertainty: d=0.50 should be yellow (d_red=0.43, d_yellow=0.84)
        low_sigma_cfg = SimpleNamespace(
            s_min=0.10, loss_timeout=0.5, robot_frames=["tcp"], marker_radius=0.15
        )
        logic_low = SafetyMonitorLogic(risk_cfg, low_sigma_cfg)
        zone_low, _, _, _ = logic_low.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(0.50, 0.0, 0.0),
            uncertainty=0.05,  # z_d=0.05 → d_red=0.43
        )
        assert (
            zone_low == "yellow"
        ), f"at d=0.50, z_d=0.05 the zone should be yellow (got {zone_low!r})"

        # High uncertainty: z_d=0.15 → d_red grows above 0.50 → zone becomes red
        # d_red(z_d=0.15) = 1.6*(0.10+0.02) + 0.10*0.10 + 0.06 + 0.10 + 0.15 + 0.02
        #                 = 0.192 + 0.01 + 0.06 + 0.10 + 0.15 + 0.02 = 0.532
        zone_high, _, _, _ = logic_low.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(0.50, 0.0, 0.0),
            uncertainty=0.15,  # z_d=0.15 → d_red=0.53 > 0.50
        )
        assert (
            zone_high == "red"
        ), f"at d=0.50, z_d=0.15 the zone should be red (got {zone_high!r})"

    def test_uncertainty_zero_gives_tightest_thresholds(self, risk_cfg):
        # σ=0 → smallest d_red and d_yellow (strictest case with known geometry)
        # σ=0.1 → wider thresholds
        cfg = SimpleNamespace(
            s_min=0.10, loss_timeout=0.5, robot_frames=["tcp"], marker_radius=0.15
        )
        logic = SafetyMonitorLogic(risk_cfg, cfg)

        # d=1.5 m — comfortably green at both σ levels
        zone0, _, _, _ = logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(1.5, 0.0, 0.0),
            uncertainty=0.0,
        )
        zone1, _, _, _ = logic.compute(
            robot_frames_xyz=[(0.0, 0.0, 0.0)],
            human_xyz_or_none=(1.5, 0.0, 0.0),
            uncertainty=0.1,
        )
        assert zone0 == "green"
        assert zone1 == "green"


# ---------------------------------------------------------------------------
# Full-pipeline scenario: robot near tray, operator reaches in
# (mirrors the human_node's TRAY_CENTRE geometry from cell.xacro)
# ---------------------------------------------------------------------------


class TestFullPipelineScenario:
    """End-to-end scenario: arm at tray, operator approaches then reaches in."""

    # Cell geometry from cell.xacro / human_node.py:
    # tray centre = (0.35, 0.0, 0.76); TCP hovers at (0.35, 0.0, 0.80)
    _TCP = (0.35, 0.0, 0.80)
    _WRIST = (0.35, 0.0, 0.55)
    _ELBOW = (0.35, 0.0, 0.30)

    def test_operator_far_away_is_green(self, logic):
        # Operator at standing position far from tray
        zone, scale, dist, _ = logic.compute(
            robot_frames_xyz=[self._TCP, self._WRIST, self._ELBOW],
            human_xyz_or_none=(0.9, 0.65, 1.10),  # from human_node Waypoint 0
            uncertainty=0.05,
        )
        assert zone == "green"
        assert scale == pytest.approx(1.0)
        assert dist is not None and dist > 0.5

    def test_operator_approaching_tray_is_yellow(self, logic):
        # Operator hovering above tray at approach height
        # human at (0.35, 0.0, 0.95), TCP at (0.35, 0.0, 0.80)
        # d ≈ |0.95 - 0.80| = 0.15? No: dist from wrist (0.35,0,0.55) = 0.40 m
        # Actually let's use a distance clearly in the yellow band
        # d_yellow ≈ 0.84, d_red ≈ 0.43 with z_d=0.05
        # Put operator at 0.6 m from TCP: human at (0.35, 0.6, 0.80)
        zone, scale, dist, _ = logic.compute(
            robot_frames_xyz=[self._TCP, self._WRIST, self._ELBOW],
            human_xyz_or_none=(0.35, 0.6, 0.80),
            uncertainty=0.05,
        )
        assert zone == "yellow"
        assert 0.0 < scale < 1.0
        assert dist is not None

    def test_operator_in_tray_is_red_stop(self, logic):
        # Operator hand inside tray very close to TCP
        # TCP at (0.35, 0.0, 0.80), human at (0.37, 0.0, 0.82) → d~0.028 m
        zone, scale, dist, _ = logic.compute(
            robot_frames_xyz=[self._TCP, self._WRIST, self._ELBOW],
            human_xyz_or_none=(0.37, 0.0, 0.82),
            uncertainty=0.05,
        )
        assert zone == "red"
        assert scale == pytest.approx(0.0)

    def test_occlusion_triggers_fail_safe(self, logic):
        """AT-5: perception loses operator → fail-safe protective stop."""
        zone, scale, dist, marker = logic.compute(
            robot_frames_xyz=[self._TCP, self._WRIST, self._ELBOW],
            human_xyz_or_none=None,  # TF stale/missing
            uncertainty=0.05,
        )
        assert zone == "lost", "AT-5: occlusion must trigger 'lost' zone"
        assert scale == 0.0, "AT-5: lost zone must produce scale=0.0 (protective stop)"
        assert dist is None
        assert marker["zone"] == "lost"

    def test_operator_retreat_resumes_normal_scale(self, logic):
        """Once operator retreats (human_xyz valid again), zone recovers."""
        # First call: perception lost
        zone_lost, scale_lost, _, _ = logic.compute(
            robot_frames_xyz=[self._TCP, self._WRIST, self._ELBOW],
            human_xyz_or_none=None,
            uncertainty=0.05,
        )
        assert zone_lost == "lost"
        assert scale_lost == 0.0

        # Second call: operator back at standing position (far)
        zone_ok, scale_ok, _, _ = logic.compute(
            robot_frames_xyz=[self._TCP, self._WRIST, self._ELBOW],
            human_xyz_or_none=(0.9, 0.65, 1.10),
            uncertainty=0.05,
        )
        assert zone_ok == "green", "clean resume: zone must be green after retreat"
        assert scale_ok == pytest.approx(1.0), "clean resume: scale must return to 1.0"


# ---------------------------------------------------------------------------
# safety_source field in safety.yaml (AGENTS.md §11 cut-scope fallback #4)
#
# The pure-Python SafetyMonitorLogic layer is NOT affected by safety_source —
# it takes human_xyz directly; frame selection is the ROS wrapper's job.
# These tests verify the YAML contract so a misconfigured file is caught early.
# ---------------------------------------------------------------------------


class TestSafetyYamlSafetySource:
    """safety.yaml must contain safety_source: perceived (the default).

    The field documents the §11 fallback toggle; the ROS wrapper reads it as
    a ROS parameter (default "perceived").  Tests here guard the YAML contract
    so a stale or corrupted config file is caught by the unit suite, not
    discovered at runtime.
    """

    def _load_raw(self) -> dict:
        with open(_SAFETY_YAML, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def test_safety_source_field_present(self):
        """safety_source key must exist under the safety: block."""
        data = self._load_raw()
        assert "safety_source" in data["safety"], (
            "safety.yaml is missing 'safety_source' under the 'safety:' block. "
            "Add it with default 'perceived' per §11 fallback #4."
        )

    def test_safety_source_default_is_perceived(self):
        """Default value must be 'perceived' — DoD #5 requires perceived as headline."""
        data = self._load_raw()
        assert data["safety"]["safety_source"] == "perceived", (
            "safety_source default must be 'perceived' (DoD #5). "
            f"Got: {data['safety']['safety_source']!r}"
        )
