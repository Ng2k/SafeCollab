"""TDD spec for the safecollab.perception package — geometry & logic.

Tests cover the pure-Python core that runs without any ROS graph:

  - back_project: pixel + depth + intrinsics → camera-frame 3-D point
    (sub-centimetre accuracy required, AGENTS.md §6 "First PR").
  - transform_point: 4x4 homogeneous transform applied to a 3-D point.
  - estimate_uncertainty: σ grows on noisier/ambiguous detections (feeds Z_d).
  - detect_human: OpenCV colour-blob detection on synthetic images.
  - PerceptionLogic: state machine — loss-timeout → is_lost, position, sigma.

ROS (rclpy, tf2, sensor_msgs, etc.) is NOT installed in the pure-Python venv
(see requirements.txt and AGENTS.md ground rule 9).  All tests here run
without a live ROS graph.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from safecollab.perception import (
    PerceptionLogic,
    _derive_intrinsics,
    back_project,
    detect_human,
    estimate_uncertainty,
    transform_point,
)


# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

# Typical 640×480 camera intrinsics (mid-range webcam / sim camera defaults).
# These values are used throughout the geometry tests.
FX = 525.0
FY = 525.0
CX = 320.0
CY = 240.0


def _identity_4x4() -> np.ndarray:
    """4×4 identity matrix."""
    return np.eye(4, dtype=float)


def _translation_4x4(tx: float, ty: float, tz: float) -> np.ndarray:
    """Pure-translation 4×4 homogeneous matrix."""
    m = np.eye(4, dtype=float)
    m[0, 3] = tx
    m[1, 3] = ty
    m[2, 3] = tz
    return m


def _rotation_z_4x4(angle_rad: float) -> np.ndarray:
    """Rotation about Z by *angle_rad*, embedded in 4×4."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    m = np.eye(4, dtype=float)
    m[0, 0] = c
    m[0, 1] = -s
    m[1, 0] = s
    m[1, 1] = c
    return m


def _synthetic_bgr_with_blob(
    width: int = 200,
    height: int = 200,
    blob_cx: int = 100,
    blob_cy: int = 100,
    blob_radius: int = 20,
    bgr_color: tuple = (0, 255, 255),  # BGR yellow
) -> np.ndarray:
    """Create a black image with a solid-colour circular blob.

    Args:
        bgr_color: BGR tuple for the blob (default: yellow, easy to detect).
    """
    img = np.zeros((height, width, 3), dtype=np.uint8)
    cv2_circle_center = (blob_cx, blob_cy)
    import cv2

    cv2.circle(img, cv2_circle_center, blob_radius, bgr_color, thickness=-1)
    return img


# ---------------------------------------------------------------------------
# back_project — pixel + depth → camera-frame 3-D point
# ---------------------------------------------------------------------------


class TestBackProject:
    """Verify the pinhole back-projection formula to sub-centimetre accuracy."""

    def test_centre_pixel_maps_to_optical_axis(self):
        """Principal point at any depth gives (0, 0, depth) — the optical axis."""
        depth = 1.5
        x, y, z = back_project(CX, CY, depth, FX, FY, CX, CY)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
        assert z == pytest.approx(depth, abs=1e-9)

    def test_offset_pixel_x_direction(self):
        """One focal-length offset in u → x = depth at depth=1."""
        depth = 1.0
        u = CX + FX  # one focal length to the right of centre
        x, y, z = back_project(u, CY, depth, FX, FY, CX, CY)
        assert x == pytest.approx(1.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
        assert z == pytest.approx(1.0, abs=1e-9)

    def test_offset_pixel_y_direction(self):
        """One focal-length offset in v → y = depth at depth=1."""
        depth = 1.0
        v = CY + FY
        x, y, z = back_project(CX, v, depth, FX, FY, CX, CY)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(1.0, abs=1e-9)
        assert z == pytest.approx(1.0, abs=1e-9)

    def test_negative_pixel_offset(self):
        """Pixels left of / above the principal point produce negative x / y."""
        depth = 2.0
        u = CX - FX / 2.0
        v = CY - FY / 2.0
        x, y, z = back_project(u, v, depth, FX, FY, CX, CY)
        assert x == pytest.approx(-1.0, abs=1e-9)
        assert y == pytest.approx(-1.0, abs=1e-9)
        assert z == pytest.approx(2.0, abs=1e-9)

    def test_sub_centimetre_accuracy_known_point(self):
        """Round-trip: project a known 3-D point to pixel and back → error < 1 mm.

        This is the AGENTS.md §6 'sub-cm accuracy' gate.
        """
        # Known camera-frame point
        x_true, y_true, z_true = 0.15, -0.10, 1.20

        # Forward-project to pixel (ideal pinhole)
        u = x_true * FX / z_true + CX
        v = y_true * FY / z_true + CY

        # Back-project with the correct depth
        x_rec, y_rec, z_rec = back_project(u, v, z_true, FX, FY, CX, CY)

        assert abs(x_rec - x_true) < 0.001  # < 1 mm
        assert abs(y_rec - y_true) < 0.001
        assert abs(z_rec - z_true) < 0.001

    def test_depth_scaling_linearity(self):
        """Doubling depth doubles all camera-frame coordinates (linearity)."""
        u, v = CX + 100, CY + 80
        x1, y1, z1 = back_project(u, v, 1.0, FX, FY, CX, CY)
        x2, y2, z2 = back_project(u, v, 2.0, FX, FY, CX, CY)
        assert x2 == pytest.approx(2.0 * x1, abs=1e-9)
        assert y2 == pytest.approx(2.0 * y1, abs=1e-9)
        assert z2 == pytest.approx(2.0 * z1, abs=1e-9)

    def test_zero_depth_is_origin(self):
        """depth=0 always returns (0, 0, 0) regardless of pixel location."""
        x, y, z = back_project(CX + 50, CY - 30, 0.0, FX, FY, CX, CY)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
        assert z == pytest.approx(0.0, abs=1e-9)

    def test_different_fx_fy(self):
        """Non-square pixels: fx ≠ fy produces different x and y scales."""
        fx, fy = 600.0, 400.0
        depth = 1.0
        u = CX + fx  # one fx to the right
        v = CY + fy  # one fy below
        x, y, z = back_project(u, v, depth, fx, fy, CX, CY)
        assert x == pytest.approx(1.0, abs=1e-9)
        assert y == pytest.approx(1.0, abs=1e-9)


# ---------------------------------------------------------------------------
# transform_point — 4×4 homogeneous transform
# ---------------------------------------------------------------------------


class TestTransformPoint:
    """Verify homogeneous-coordinate 3-D point transformation."""

    def test_identity_transform_leaves_point_unchanged(self):
        """Identity matrix must not alter any coordinate."""
        point = (1.0, 2.0, 3.0)
        result = transform_point(point, _identity_4x4())
        assert result == pytest.approx(point, abs=1e-9)

    def test_pure_translation(self):
        """Translating by (dx, dy, dz) adds exactly that offset."""
        point = (1.0, 0.0, 0.0)
        m = _translation_4x4(0.5, -0.3, 1.2)
        x, y, z = transform_point(point, m)
        assert x == pytest.approx(1.5, abs=1e-9)
        assert y == pytest.approx(-0.3, abs=1e-9)
        assert z == pytest.approx(1.2, abs=1e-9)

    def test_90_degree_rotation_around_z(self):
        """90° rotation about Z: (1,0,0) → (0,1,0)."""
        point = (1.0, 0.0, 0.0)
        m = _rotation_z_4x4(math.pi / 2.0)
        x, y, z = transform_point(point, m)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(1.0, abs=1e-9)
        assert z == pytest.approx(0.0, abs=1e-9)

    def test_180_degree_rotation_around_z(self):
        """180° rotation about Z: (1,0,0) → (-1,0,0)."""
        point = (1.0, 0.0, 0.0)
        m = _rotation_z_4x4(math.pi)
        x, y, z = transform_point(point, m)
        assert x == pytest.approx(-1.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
        assert z == pytest.approx(0.0, abs=1e-9)

    def test_rotation_plus_translation(self):
        """Rotation then translation: combined 4×4 matrix must compose correctly."""
        # Rotate 90° around Z, then translate (1, 0, 0).
        # After rotation, (1,0,0) becomes (0,1,0); then +translate(1,0,0) → (1,1,0).
        rot = _rotation_z_4x4(math.pi / 2.0)
        trans = _translation_4x4(1.0, 0.0, 0.0)
        combined = trans @ rot  # translate AFTER rotate
        x, y, z = transform_point((1.0, 0.0, 0.0), combined)
        assert x == pytest.approx(1.0, abs=1e-9)
        assert y == pytest.approx(1.0, abs=1e-9)
        assert z == pytest.approx(0.0, abs=1e-9)

    def test_z_translation_only(self):
        """Pure Z translation does not affect x or y."""
        m = _translation_4x4(0.0, 0.0, 0.74)
        x, y, z = transform_point((0.0, 0.0, 0.0), m)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)
        assert z == pytest.approx(0.74, abs=1e-9)

    def test_returns_floats(self):
        """Output elements are Python floats, not numpy scalars."""
        result = transform_point((1.0, 2.0, 3.0), _identity_4x4())
        assert all(isinstance(v, float) for v in result)

    def test_sub_centimetre_end_to_end(self):
        """Full pipeline: back_project → transform_point → world point < 1 mm error.

        Sets up a camera at (0, 0, 1.5) looking straight down (Z→−Z), so the
        optical axis hits the table surface.  A pixel at the principal point
        should map to (0, 0, 0) in the world frame.
        """
        # Camera is at z=1.5 above the origin, looking straight down.
        # In the camera frame, +Z_cam = "away from sensor" (downward toward table).
        # The camera→world transform: rotate 180° about X (flip Y and Z),
        # then translate to (0, 0, 1.5).
        # Rotation 180° about X: y' = -y, z' = -z
        rot_x_180 = np.eye(4, dtype=float)
        rot_x_180[1, 1] = -1.0
        rot_x_180[2, 2] = -1.0
        cam_to_world = _translation_4x4(0.0, 0.0, 1.5) @ rot_x_180

        # Principal pixel at depth=1.5 → camera frame (0, 0, 1.5)
        x_cam, y_cam, z_cam = back_project(CX, CY, 1.5, FX, FY, CX, CY)
        # camera frame: (0, 0, 1.5)
        # After rotation 180° about X: (0, 0, -1.5)
        # After translate (0,0,1.5): (0, 0, 0) — the table surface
        x_w, y_w, z_w = transform_point((x_cam, y_cam, z_cam), cam_to_world)
        assert abs(x_w) < 0.001
        assert abs(y_w) < 0.001
        assert abs(z_w) < 0.001


# ---------------------------------------------------------------------------
# estimate_uncertainty — σ grows on noisier/ambiguous detections
# ---------------------------------------------------------------------------


class TestEstimateUncertainty:
    """Verify that σ (fed to Z_d in the risk model) behaves correctly."""

    def test_returns_positive_float(self):
        """σ must always be strictly positive."""
        sigma = estimate_uncertainty(area_px=500.0, depth_m=1.0, confidence=0.9)
        assert sigma > 0.0

    def test_sigma_grows_with_depth(self):
        """Deeper detections are less certain → σ must increase with depth."""
        s1 = estimate_uncertainty(area_px=500.0, depth_m=1.0, confidence=0.9)
        s2 = estimate_uncertainty(area_px=500.0, depth_m=2.0, confidence=0.9)
        s3 = estimate_uncertainty(area_px=500.0, depth_m=3.0, confidence=0.9)
        assert s1 < s2 < s3, "σ must be monotonically increasing with depth"

    def test_sigma_grows_with_low_confidence(self):
        """Low confidence → noisier detection → σ must increase."""
        s_high = estimate_uncertainty(area_px=500.0, depth_m=1.0, confidence=0.95)
        s_mid = estimate_uncertainty(area_px=500.0, depth_m=1.0, confidence=0.50)
        s_low = estimate_uncertainty(area_px=500.0, depth_m=1.0, confidence=0.10)
        assert s_high < s_mid < s_low, "σ must grow as confidence decreases"

    def test_sigma_has_minimum_floor(self):
        """Even a perfect detection must have σ ≥ min_sigma."""
        sigma = estimate_uncertainty(area_px=10_000.0, depth_m=0.1, confidence=1.0)
        assert sigma >= 0.01  # min_sigma default is 0.01 m

    def test_perfect_detection_gives_small_sigma(self):
        """High confidence, close range, large area → small σ."""
        sigma = estimate_uncertainty(area_px=2000.0, depth_m=0.5, confidence=1.0)
        assert sigma < 0.10  # should be well under 10 cm

    def test_poor_detection_gives_large_sigma(self):
        """Very low confidence at long range → large σ (feeds larger Z_d threshold)."""
        sigma = estimate_uncertainty(area_px=50.0, depth_m=3.0, confidence=0.05)
        assert sigma > 0.20  # should be > 20 cm

    def test_sigma_feeds_risk_model_widening(self):
        """Raising σ must widen the risk-model thresholds (integration sanity).

        This is the Z_d property from AGENTS.md §4 / risk.py — a larger σ
        from perception produces a larger Z_d → wider d_yellow, d_red.
        We test that the sigma values are ordered correctly; the risk model
        widening is tested in test_risk.py.
        """
        s_nominal = estimate_uncertainty(area_px=500.0, depth_m=1.0, confidence=0.8)
        s_noisy = estimate_uncertainty(area_px=100.0, depth_m=1.5, confidence=0.3)
        assert s_nominal < s_noisy

    def test_confidence_zero_does_not_raise(self):
        """Confidence=0 must not raise ZeroDivisionError (guarded by eps)."""
        sigma = estimate_uncertainty(area_px=500.0, depth_m=1.0, confidence=0.0)
        assert sigma > 0.0

    def test_area_px_parameter_accepted(self):
        """area_px parameter must be accepted without error (even if not used)."""
        s1 = estimate_uncertainty(area_px=10.0, depth_m=1.0, confidence=0.5)
        s2 = estimate_uncertainty(area_px=10000.0, depth_m=1.0, confidence=0.5)
        assert isinstance(s1, float)
        assert isinstance(s2, float)


# ---------------------------------------------------------------------------
# detect_human — OpenCV colour-blob detector on synthetic images
# ---------------------------------------------------------------------------


class TestDetectHuman:
    """Verify blob detection on controlled synthetic images."""

    def test_blank_image_returns_none(self):
        """A completely black image has no human → None."""
        blank = np.zeros((100, 100, 3), dtype=np.uint8)
        assert detect_human(blank) is None

    def test_detects_blob_in_synthetic_image(self):
        """A single bright blob must be found; centroid must be near the blob centre."""
        # Yellow blob (BGR = [0, 255, 255]) at centre of 200×200 image.
        img = _synthetic_bgr_with_blob(
            width=200,
            height=200,
            blob_cx=100,
            blob_cy=100,
            blob_radius=25,
            bgr_color=(0, 255, 255),  # yellow
        )
        result = detect_human(img)
        assert result is not None, "yellow blob must be detected"
        u, v, area, confidence = result
        assert abs(u - 100.0) < 5.0, f"centroid u={u:.1f} not near 100"
        assert abs(v - 100.0) < 5.0, f"centroid v={v:.1f} not near 100"

    def test_centroid_accuracy_off_centre_blob(self):
        """Blob placed off-centre: centroid must be within 3 px of truth."""
        img = _synthetic_bgr_with_blob(
            width=320,
            height=240,
            blob_cx=80,
            blob_cy=60,
            blob_radius=20,
            bgr_color=(0, 255, 255),
        )
        result = detect_human(img)
        assert result is not None
        u, v, _, _ = result
        assert abs(u - 80.0) < 3.0
        assert abs(v - 60.0) < 3.0

    def test_area_is_positive(self):
        """Detected blob area must be positive."""
        img = _synthetic_bgr_with_blob()
        result = detect_human(img)
        assert result is not None
        _, _, area, _ = result
        assert area > 0.0

    def test_confidence_in_unit_interval(self):
        """Confidence must be in (0, 1]."""
        img = _synthetic_bgr_with_blob()
        result = detect_human(img)
        assert result is not None
        _, _, _, confidence = result
        assert 0.0 < confidence <= 1.0

    def test_small_blob_below_min_area_not_detected(self):
        """A blob smaller than min_area_px must be rejected."""
        img = _synthetic_bgr_with_blob(blob_radius=2)  # ≈ 12 px² area
        result = detect_human(img, min_area_px=500.0)
        assert result is None

    def test_large_blob_detected_with_custom_min_area(self):
        """A large blob must pass the min_area_px gate."""
        img = _synthetic_bgr_with_blob(blob_radius=40)
        result = detect_human(img, min_area_px=100.0)
        assert result is not None

    def test_returns_four_tuple(self):
        """Return value, when not None, must be a 4-tuple."""
        img = _synthetic_bgr_with_blob()
        result = detect_human(img)
        assert result is not None
        assert len(result) == 4

    def test_none_input_returns_none(self):
        """Passing None as the image must return None without raising."""
        result = detect_human(None)
        assert result is None

    def test_zero_size_array_returns_none(self):
        """A zero-sized numpy array (empty image) must return None."""
        empty = np.zeros((0, 0, 3), dtype=np.uint8)
        result = detect_human(empty)
        assert result is None


# ---------------------------------------------------------------------------
# PerceptionLogic — state machine: loss timeout, position, sigma
# ---------------------------------------------------------------------------


class TestPerceptionLogic:
    """Verify the pure-Python state machine for loss handling."""

    def test_initially_lost(self):
        """Before any update the node is in the 'lost' state."""
        logic = PerceptionLogic()
        assert logic.is_lost is True

    def test_position_is_none_when_lost(self):
        """position property returns None when is_lost is True."""
        logic = PerceptionLogic()
        assert logic.position is None

    def test_update_with_valid_detection_clears_lost(self):
        """A valid detection (position not None) transitions out of 'lost'."""
        logic = PerceptionLogic()
        logic.update(position=(0.3, 0.0, 0.8), sigma=0.05, timestamp_s=1.0)
        assert logic.is_lost is False

    def test_position_returned_after_valid_detection(self):
        """After a valid detection, position returns the supplied tuple."""
        logic = PerceptionLogic()
        pos = (0.3, 0.1, 0.78)
        logic.update(position=pos, sigma=0.05, timestamp_s=1.0)
        assert logic.position == pytest.approx(pos)

    def test_sigma_updated_on_valid_detection(self):
        """sigma property reflects the value supplied in the latest update."""
        logic = PerceptionLogic()
        logic.update(position=(0.3, 0.0, 0.8), sigma=0.07, timestamp_s=1.0)
        assert logic.sigma == pytest.approx(0.07)

    def test_update_with_none_position_does_not_clear_lost_immediately(self):
        """A None-position update within the timeout window keeps the last position.

        The node is still lost (never had a valid detection), so this should
        remain lost — but the point here is it must NOT raise.
        """
        logic = PerceptionLogic(loss_timeout_s=0.5)
        logic.update(position=None, sigma=0.10, timestamp_s=1.0)
        assert logic.is_lost is True  # was never acquired

    def test_loss_timeout_after_valid_detection(self):
        """After a valid detection, missing detections beyond the timeout → lost."""
        logic = PerceptionLogic(loss_timeout_s=0.5)
        # Valid detection at t=1.0
        logic.update(position=(0.3, 0.0, 0.8), sigma=0.05, timestamp_s=1.0)
        assert logic.is_lost is False

        # No detection for 0.6 s (> 0.5 s timeout)
        logic.check_timeout(current_time_s=1.6)
        assert logic.is_lost is True, "must enter 'lost' after timeout"

    def test_loss_within_timeout_does_not_trigger_lost(self):
        """Missing detections WITHIN the timeout do not trigger 'lost'."""
        logic = PerceptionLogic(loss_timeout_s=0.5)
        logic.update(position=(0.3, 0.0, 0.8), sigma=0.05, timestamp_s=1.0)
        # Only 0.3 s have passed — still within timeout
        logic.check_timeout(current_time_s=1.3)
        assert logic.is_lost is False

    def test_position_becomes_none_after_timeout(self):
        """After loss timeout, position must return None (not stale data)."""
        logic = PerceptionLogic(loss_timeout_s=0.3)
        logic.update(position=(0.5, 0.1, 0.8), sigma=0.05, timestamp_s=0.0)
        logic.check_timeout(current_time_s=0.5)
        assert logic.position is None

    def test_re_acquisition_clears_lost(self):
        """A new valid detection after loss re-acquires the human."""
        logic = PerceptionLogic(loss_timeout_s=0.3)
        logic.update(position=(0.3, 0.0, 0.8), sigma=0.05, timestamp_s=0.0)
        logic.check_timeout(current_time_s=0.5)
        assert logic.is_lost is True

        # New detection
        logic.update(position=(0.31, 0.01, 0.8), sigma=0.06, timestamp_s=0.6)
        assert logic.is_lost is False
        assert logic.position == pytest.approx((0.31, 0.01, 0.8))

    def test_check_timeout_no_prior_detection_stays_lost(self):
        """check_timeout with no prior detection must not raise and stays lost."""
        logic = PerceptionLogic(loss_timeout_s=0.5)
        logic.check_timeout(current_time_s=10.0)
        assert logic.is_lost is True

    def test_sigma_default_is_positive(self):
        """Initial sigma must be positive (no division-by-zero in risk model)."""
        logic = PerceptionLogic()
        assert logic.sigma > 0.0

    def test_custom_loss_timeout(self):
        """loss_timeout_s parameter is respected."""
        logic_short = PerceptionLogic(loss_timeout_s=0.1)
        logic_long = PerceptionLogic(loss_timeout_s=2.0)

        for lg in (logic_short, logic_long):
            lg.update(position=(0.3, 0.0, 0.8), sigma=0.05, timestamp_s=0.0)

        logic_short.check_timeout(current_time_s=0.2)
        logic_long.check_timeout(current_time_s=0.2)

        assert logic_short.is_lost is True, "short timeout (0.1s) must have expired"
        assert logic_long.is_lost is False, "long timeout (2.0s) must not have expired"


# ---------------------------------------------------------------------------
# Full pipeline integration — pixel → camera → world (sub-cm gate)
# ---------------------------------------------------------------------------


class TestFullPipeline:
    """End-to-end geometry: back_project + transform_point → world point."""

    def test_known_world_point_recovered_to_sub_centimetre(self):
        """Project a known world point forward and verify the round-trip.

        Setup: camera at (0, 0, 2.0), looking straight down (Z_cam → -Z_world).
        The human hand is at world (0.35, 0.0, 0.76) — the kitting tray centre
        from AGENTS.md / cell.xacro.
        """
        # Camera-to-world transform:
        # Camera sits at (0, 0, 2.0) and points downward.
        # In OpenCV convention, Z_cam points "away from lens" (downward here).
        # Rotation: 180° about X maps (X_cam, Y_cam, Z_cam) to (X_w, -Y_w, -Z_w)
        # plus the camera position offset.
        rot_x_180 = np.eye(4, dtype=float)
        rot_x_180[1, 1] = -1.0
        rot_x_180[2, 2] = -1.0
        cam_to_world = _translation_4x4(0.0, 0.0, 2.0) @ rot_x_180

        # True world position of the hand
        x_w_true, y_w_true, z_w_true = 0.35, 0.0, 0.76

        # Compute camera-frame point manually:
        # P_cam = R_world_to_cam * (P_world - t_cam)
        # camera is at (0,0,2): P_relative = (0.35, 0.0, 0.76 - 2.0) = (0.35, 0.0, -1.24)
        # rotation 180° about X (world→cam inverse of cam→world):
        # R_wc = R_cw^T = same rotation since R is symmetric for 180° about X
        # P_cam = (0.35, 0.0 * -1, -1.24 * -1) = (0.35, 0.0, 1.24)
        x_cam_true, y_cam_true, z_cam_true = 0.35, 0.0, 1.24

        # Forward-project to pixel
        u = x_cam_true * FX / z_cam_true + CX
        v = y_cam_true * FY / z_cam_true + CY

        # Back-project to camera frame using the correct depth
        x_cam, y_cam, z_cam = back_project(u, v, z_cam_true, FX, FY, CX, CY)

        # Transform to world
        x_w, y_w, z_w = transform_point((x_cam, y_cam, z_cam), cam_to_world)

        assert abs(x_w - x_w_true) < 0.001, f"x error: {abs(x_w - x_w_true):.4f} m"
        assert abs(y_w - y_w_true) < 0.001, f"y error: {abs(y_w - y_w_true):.4f} m"
        assert abs(z_w - z_w_true) < 0.001, f"z error: {abs(z_w - z_w_true):.4f} m"

    def test_multiple_world_points_all_sub_centimetre(self):
        """Verify sub-cm accuracy for several different world positions.

        This ensures the geometry is not accidentally tuned for one special case.
        """
        rot_x_180 = np.eye(4, dtype=float)
        rot_x_180[1, 1] = -1.0
        rot_x_180[2, 2] = -1.0
        cam_to_world = _translation_4x4(0.0, 0.0, 2.0) @ rot_x_180

        # Several hand positions within the cell workspace
        world_points = [
            (0.20, -0.15, 0.78),
            (0.35, 0.00, 0.76),
            (0.50, 0.20, 0.82),
            (-0.10, 0.10, 0.50),
        ]

        for x_w_true, y_w_true, z_w_true in world_points:
            # Convert to camera frame (inverse of cam_to_world)
            # cam_to_world = T(0,0,2) @ Rx180
            # world_to_cam = Rx180^-1 @ T(0,0,-2) = Rx180 @ T(0,0,-2)
            # (since Rx180 is its own inverse)
            dx, dy, dz = x_w_true - 0.0, y_w_true - 0.0, z_w_true - 2.0
            # Apply Rx180 inverse (= Rx180) to get camera frame coords
            x_cam_t = dx
            y_cam_t = -dy
            z_cam_t = -dz

            if z_cam_t <= 0:
                continue  # skip: point is behind camera

            u = x_cam_t * FX / z_cam_t + CX
            v = y_cam_t * FY / z_cam_t + CY

            x_cam, y_cam, z_cam = back_project(u, v, z_cam_t, FX, FY, CX, CY)
            x_w, y_w, z_w = transform_point((x_cam, y_cam, z_cam), cam_to_world)

            assert (
                abs(x_w - x_w_true) < 0.001
            ), f"Point {(x_w_true, y_w_true, z_w_true)}: x error {abs(x_w - x_w_true):.4f} m"
            assert (
                abs(y_w - y_w_true) < 0.001
            ), f"Point {(x_w_true, y_w_true, z_w_true)}: y error {abs(y_w - y_w_true):.4f} m"
            assert (
                abs(z_w - z_w_true) < 0.001
            ), f"Point {(x_w_true, y_w_true, z_w_true)}: z error {abs(z_w - z_w_true):.4f} m"


# ---------------------------------------------------------------------------
# _derive_intrinsics — Gazebo camera intrinsics from FOV + image size
# ---------------------------------------------------------------------------


class TestDeriveIntrinsics:
    """Verify the pinhole intrinsic derivation matches Gazebo's formula."""

    def test_square_image_fx_equals_fy(self):
        """Square pixels assumed: fx must equal fy."""
        fx, fy, cx, cy = _derive_intrinsics(width=640, height=480, hfov_rad=1.0472)
        assert fx == pytest.approx(fy, abs=1e-9)

    def test_principal_point_at_image_centre(self):
        """cx and cy must be exactly half the image dimensions."""
        fx, fy, cx, cy = _derive_intrinsics(width=640, height=480, hfov_rad=1.0472)
        assert cx == pytest.approx(320.0, abs=1e-9)
        assert cy == pytest.approx(240.0, abs=1e-9)

    def test_60_degree_hfov_gives_reasonable_focal_length(self):
        """60° HFOV on 640 px width: fx ≈ 554 px (standard value)."""
        fx, fy, cx, cy = _derive_intrinsics(
            width=640, height=480, hfov_rad=math.radians(60.0)
        )
        # fx = (320) / tan(30°) = 320 / 0.5774 ≈ 554.3 px
        expected_fx = 320.0 / math.tan(math.radians(30.0))
        assert fx == pytest.approx(expected_fx, abs=0.01)

    def test_focal_length_scales_with_image_width(self):
        """Doubling image width (same FOV) doubles fx."""
        fx1, _, _, _ = _derive_intrinsics(width=320, height=240, hfov_rad=1.0)
        fx2, _, _, _ = _derive_intrinsics(width=640, height=480, hfov_rad=1.0)
        assert fx2 == pytest.approx(2.0 * fx1, abs=1e-9)

    def test_narrower_fov_gives_larger_focal_length(self):
        """Narrower FOV = more telephoto = larger fx."""
        fx_wide, _, _, _ = _derive_intrinsics(width=640, height=480, hfov_rad=1.5708)
        fx_narrow, _, _, _ = _derive_intrinsics(width=640, height=480, hfov_rad=0.5236)
        assert fx_narrow > fx_wide

    def test_returns_four_floats(self):
        """Return value must be a 4-tuple of floats."""
        result = _derive_intrinsics()
        assert len(result) == 4
        assert all(isinstance(v, float) for v in result)
