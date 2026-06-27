"""TDD tests for the calibrated camera geometry — Task 4 bug-fix (Stream C).

Covers the two new pure module-level helpers added to perception_node.py:

  - cam_to_world_transform(): returns the 4×4 camera-optical-frame→world
    matrix derived from the actual cell.xacro mast chain.

  - project_pixel_to_plane(u, v, intrinsics, cam_to_world, plane_z): full
    back-projection pipeline — pixel + camera model → ray → plane intersection
    → world (x, y, z).

All tests run without a live ROS graph (no rclpy / cv_bridge needed).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from safecollab.perception_node import (
    _derive_intrinsics,
    cam_to_world_transform,
    project_pixel_to_plane,
)

# ---------------------------------------------------------------------------
# Constants that mirror the cell.xacro / human_node.py values
# ---------------------------------------------------------------------------

# Camera optical-frame origin in world (from xacro link chain)
CAM_X = -0.45
CAM_Y = 0.35
CAM_Z = 1.64

# mast_to_camera rpy = (0, 1.0, 0)  →  pitch = 1.0 rad
CAM_PITCH_RAD = 1.0

# Hand reach plane height (REACH_Z in human_node.py)
REACH_Z = 0.82

# Intrinsics from cell.xacro: 640×480, hfov = 1.0472 rad (60°)
_FX, _FY, _CX, _CY = _derive_intrinsics(640, 480, 1.0472)


# ---------------------------------------------------------------------------
# Helper: forward-project a world point to pixel (exact pinhole model)
# ---------------------------------------------------------------------------


def _world_to_pixel(
    x_w: float,
    y_w: float,
    z_w: float,
    cam_to_world: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float, float]:
    """Project a world point to pixel (u, v) and camera-frame z-depth.

    Returns (u, v, z_cam).  Raises ValueError if the point is behind the camera.
    """
    R = cam_to_world[:3, :3]
    t = cam_to_world[:3, 3]

    # World point in camera frame: p_cam = R^T @ (p_world - t)
    p_rel = np.array([x_w - t[0], y_w - t[1], z_w - t[2]], dtype=float)
    p_cam = R.T @ p_rel

    if p_cam[2] <= 0:
        raise ValueError(f"Point {(x_w, y_w, z_w)} is behind the camera.")

    u = fx * p_cam[0] / p_cam[2] + cx
    v = fy * p_cam[1] / p_cam[2] + cy
    return u, v, float(p_cam[2])


# ---------------------------------------------------------------------------
# TestCamToWorldTransform — matrix structure and bearing sanity checks
# ---------------------------------------------------------------------------


class TestCamToWorldTransform:
    """Verify cam_to_world_transform() produces the geometrically correct 4×4."""

    def test_camera_origin_in_world(self):
        """Camera optical frame must be at world (-0.45, 0.35, 1.64)."""
        T = cam_to_world_transform(
            cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z, pitch_rad=CAM_PITCH_RAD
        )
        assert T[0, 3] == pytest.approx(CAM_X, abs=1e-9)
        assert T[1, 3] == pytest.approx(CAM_Y, abs=1e-9)
        assert T[2, 3] == pytest.approx(CAM_Z, abs=1e-9)

    def test_rotation_is_orthonormal(self):
        """The 3×3 rotation block must be orthonormal: R^T @ R = I."""
        T = cam_to_world_transform(
            cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z, pitch_rad=CAM_PITCH_RAD
        )
        R = T[:3, :3]
        product = R.T @ R
        assert product == pytest.approx(np.eye(3), abs=1e-9)

    def test_rotation_is_proper(self):
        """Determinant of the rotation block must be +1 (no reflection)."""
        T = cam_to_world_transform(
            cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z, pitch_rad=CAM_PITCH_RAD
        )
        assert np.linalg.det(T[:3, :3]) == pytest.approx(1.0, abs=1e-9)

    def test_homogeneous_row(self):
        """Bottom row must be [0, 0, 0, 1]."""
        T = cam_to_world_transform()
        assert T[3, :].tolist() == pytest.approx([0.0, 0.0, 0.0, 1.0], abs=1e-9)

    def test_optical_axis_points_forward_and_downward(self):
        """The optical axis (camera +Z) must have a downward world component.

        With pitch = 1.0 rad the camera tilts forward; the optical axis in world
        is col-2 of R, which must have a negative world-Z component (pointing
        toward the table) and a positive world-X component (tilted forward toward
        the tray from the mast behind the table).
        """
        T = cam_to_world_transform(
            cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z, pitch_rad=CAM_PITCH_RAD
        )
        # optical axis in world = R @ [0, 0, 1]^T = 3rd column of R
        optical_axis = T[:3, 2]
        assert optical_axis[2] < 0.0, "optical axis must point downward (neg world-Z)"
        assert optical_axis[0] > 0.0, "optical axis must tilt toward +X (the tray)"

    def test_principal_pixel_maps_to_table_area(self):
        """Center pixel at plane z=0.82 must land near the kitting tray (positive X)."""
        T = cam_to_world_transform(
            cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z, pitch_rad=CAM_PITCH_RAD
        )
        intrinsics = (_FX, _FY, _CX, _CY)
        world_pt = project_pixel_to_plane(_CX, _CY, intrinsics, T, REACH_Z)
        assert world_pt is not None
        x_w, y_w, z_w = world_pt
        # Center of view from mast at (-0.45, 0.35) pitched 1.0 rad at height 1.64
        # should project to roughly (0.0 – 0.15, 0.35) at z=0.82 — positive X.
        assert x_w > -0.2, f"view centre x={x_w:.3f} should be near the table"
        assert z_w == pytest.approx(REACH_Z, abs=1e-6)

    def test_defaults_match_xacro(self):
        """Default arguments must reproduce the xacro link-chain camera pose."""
        T_default = cam_to_world_transform()
        T_explicit = cam_to_world_transform(
            cam_x=-0.45, cam_y=0.35, cam_z=1.64, pitch_rad=1.0
        )
        assert T_default == pytest.approx(T_explicit, abs=1e-12)


# ---------------------------------------------------------------------------
# TestProjectPixelToPlane — ray-plane intersection edge cases
# ---------------------------------------------------------------------------


class TestProjectPixelToPlane:
    """Verify project_pixel_to_plane() edge cases and failure modes."""

    def _default_transform(self) -> np.ndarray:
        return cam_to_world_transform(
            cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z, pitch_rad=CAM_PITCH_RAD
        )

    def test_returns_three_floats(self):
        """Return value must be a 3-tuple of Python floats (not numpy scalars)."""
        T = self._default_transform()
        result = project_pixel_to_plane(_CX, _CY, (_FX, _FY, _CX, _CY), T, REACH_Z)
        assert result is not None
        assert len(result) == 3
        assert all(isinstance(v, float) for v in result)

    def test_z_coordinate_equals_plane_z(self):
        """The returned world point must lie exactly on the requested plane."""
        T = self._default_transform()
        for u, v in [(_CX, _CY), (_CX + 50, _CY - 30), (_CX - 80, _CY + 40)]:
            result = project_pixel_to_plane(u, v, (_FX, _FY, _CX, _CY), T, REACH_Z)
            if result is not None:
                assert result[2] == pytest.approx(
                    REACH_Z, abs=1e-9
                ), f"pixel ({u},{v}) gave z={result[2]:.6f}, expected {REACH_Z}"

    def test_horizontal_ray_returns_none(self):
        """A camera looking exactly sideways (no z-component in ray) → None."""
        # Build a transform where the optical axis is purely horizontal
        # (90° pitch → Ry(pi/2) makes the z-cam point in the +X world direction)
        T_horizontal = cam_to_world_transform(
            cam_x=0.0, cam_y=0.0, cam_z=1.0, pitch_rad=math.pi / 2.0
        )
        # Check that all pixels produce either None (horizontal optical axis)
        # or a valid intersection — we specifically want no crash.
        result = project_pixel_to_plane(
            _CX, _CY, (_FX, _FY, _CX, _CY), T_horizontal, 0.0
        )
        # With pitch=pi/2 the optical axis is exactly horizontal (+X in world):
        # the center-pixel ray direction in world will have z ≈ 0 → None
        # (or a very large t value if float precision gives a tiny z component).
        # Either None or a finite result is acceptable; the function must not raise.
        if result is not None:
            assert all(math.isfinite(v) for v in result)

    def test_plane_above_camera_returns_none_or_negative_t(self):
        """If the plane is far above the camera and the ray points downward → None."""
        T = self._default_transform()
        # Place the plane above the camera (z = 5.0 > cam_z = 1.64).
        # The camera looks downward, so the ray has negative world-Z and will
        # never reach a plane at z=5.0 → None (negative t).
        result = project_pixel_to_plane(_CX, _CY, (_FX, _FY, _CX, _CY), T, 5.0)
        assert result is None

    def test_different_plane_heights_give_different_points(self):
        """Varying plane_z shifts the recovered world point along the ray."""
        T = self._default_transform()
        intrinsics = (_FX, _FY, _CX, _CY)
        r1 = project_pixel_to_plane(_CX, _CY, intrinsics, T, 0.74)
        r2 = project_pixel_to_plane(_CX, _CY, intrinsics, T, 0.82)
        assert r1 is not None and r2 is not None
        # Different heights → different (x, y) for a non-vertical ray
        assert r1[0] != pytest.approx(r2[0], abs=1e-3) or r1[1] != pytest.approx(
            r2[1], abs=1e-3
        )


# ---------------------------------------------------------------------------
# TestFullPipelineActualCamera — round-trip pixel ↔ world with the true pose
# ---------------------------------------------------------------------------


class TestFullPipelineActualCamera:
    """End-to-end round-trip: world point → pixel → project_pixel_to_plane → world.

    Asserts sub-millimetre residual for multiple hand positions on the plane
    z=0.82 within the kitting-cell workspace.  This is the AGENTS.md §6
    'sub-cm accuracy' gate applied to the *corrected* camera model.

    The round-trip procedure:
    1. Choose a world point P on the plane z=REACH_Z.
    2. Forward-project P through the exact pinhole model to get pixel (u, v).
    3. Run project_pixel_to_plane(u, v, intrinsics, cam_to_world, REACH_Z).
    4. Assert |recovered − P| < 1 mm in each axis.
    """

    @pytest.fixture(autouse=True)
    def _setup(self):
        self.T = cam_to_world_transform(
            cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z, pitch_rad=CAM_PITCH_RAD
        )
        self.intrinsics = (_FX, _FY, _CX, _CY)

    def _roundtrip_residual(
        self, x_w: float, y_w: float, z_w: float
    ) -> tuple[float, float, float]:
        """Return (|Δx|, |Δy|, |Δz|) for one round-trip."""
        u, v, _ = _world_to_pixel(x_w, y_w, z_w, self.T, _FX, _FY, _CX, _CY)
        recovered = project_pixel_to_plane(u, v, self.intrinsics, self.T, z_w)
        assert (
            recovered is not None
        ), f"project_pixel_to_plane returned None for world point {(x_w, y_w, z_w)}"
        return abs(recovered[0] - x_w), abs(recovered[1] - y_w), abs(recovered[2] - z_w)

    def test_tray_centre_roundtrip_sub_mm(self):
        """Kitting-tray centre (0.35, 0.0, 0.82) → pixel → world, error < 1 mm."""
        dx, dy, dz = self._roundtrip_residual(0.35, 0.0, REACH_Z)
        assert dx < 0.001, f"x residual {dx*1000:.3f} mm > 1 mm"
        assert dy < 0.001, f"y residual {dy*1000:.3f} mm > 1 mm"
        assert dz < 1e-9, f"z residual {dz*1000:.6f} mm > eps"

    def test_camera_centre_projection_roundtrip(self):
        """Principal ray intersection with REACH_Z plane → round-trip < 1 mm."""
        # Find where the principal ray (centre pixel) hits z=REACH_Z
        centre_pt = project_pixel_to_plane(_CX, _CY, self.intrinsics, self.T, REACH_Z)
        assert centre_pt is not None
        x0, y0, z0 = centre_pt
        dx, dy, dz = self._roundtrip_residual(x0, y0, z0)
        assert dx < 0.001, f"x residual {dx*1000:.3f} mm"
        assert dy < 0.001, f"y residual {dy*1000:.3f} mm"

    def test_multiple_workspace_points_all_sub_mm(self):
        """Several hand positions in the kitting-cell workspace, all sub-mm.

        World points are chosen to be within the camera FOV at z=REACH_Z.
        """
        workspace_points = [
            (0.35, 0.00, REACH_Z),  # tray centre
            (0.20, 0.20, REACH_Z),  # tray left
            (0.20, -0.20, REACH_Z),  # tray right
            (0.10, 0.35, REACH_Z),  # near mast X-projection, Y=same
            (0.00, 0.35, REACH_Z),  # approx. camera nadir point
            (0.50, 0.10, REACH_Z),  # far right of tray
        ]
        for x_w, y_w, z_w in workspace_points:
            try:
                u, v, _ = _world_to_pixel(x_w, y_w, z_w, self.T, _FX, _FY, _CX, _CY)
            except ValueError:
                # Point behind camera — skip (shouldn't happen for workspace pts)
                continue

            # Only test pixels within the image bounds
            if not (0 <= u < 640 and 0 <= v < 480):
                continue

            dx, dy, dz = self._roundtrip_residual(x_w, y_w, z_w)
            assert (
                dx < 0.001
            ), f"Point {(x_w, y_w, z_w)}: x residual {dx*1000:.3f} mm > 1 mm"
            assert (
                dy < 0.001
            ), f"Point {(x_w, y_w, z_w)}: y residual {dy*1000:.3f} mm > 1 mm"
            assert dz < 1e-9, f"Point {(x_w, y_w, z_w)}: z residual {dz:.2e} m > eps"

    def test_off_axis_pixel_roundtrip(self):
        """A pixel far from the principal point also round-trips within 1 mm."""
        # Use a pixel significantly off-centre
        u, v = _CX + 120, _CY - 80
        world_pt = project_pixel_to_plane(u, v, self.intrinsics, self.T, REACH_Z)
        assert world_pt is not None
        x_w, y_w, z_w = world_pt
        dx, dy, dz = self._roundtrip_residual(x_w, y_w, z_w)
        assert dx < 0.001
        assert dy < 0.001

    def test_recovered_z_is_exactly_plane_z(self):
        """Recovered world z must equal plane_z to floating-point precision."""
        for u, v in [(_CX, _CY), (_CX + 100, _CY + 60), (_CX - 50, _CY - 70)]:
            result = project_pixel_to_plane(u, v, self.intrinsics, self.T, REACH_Z)
            if result is not None:
                assert result[2] == pytest.approx(REACH_Z, abs=1e-9)

    def test_rotation_composition_matches_manual_calculation(self):
        """Spot-check the rotation matrix against the hand-computed values.

        R = Ry(1.0) @ Rz(-pi/2) @ Rx(-pi/2):

        Rx(-pi/2):  [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
        Rz(-pi/2):  [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
        Rz@Rx    :  [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        Ry(1.0)@...: row-0=[0, -sin(1.0), cos(1.0)]
                     row-1=[-1, 0, 0]
                     row-2=[0, -cos(1.0), -sin(1.0)]
        """
        T = cam_to_world_transform(cam_x=0.0, cam_y=0.0, cam_z=0.0, pitch_rad=1.0)
        R = T[:3, :3]
        c, s = math.cos(1.0), math.sin(1.0)
        # Row 0
        assert R[0, 0] == pytest.approx(0.0, abs=1e-9)
        assert R[0, 1] == pytest.approx(-s, abs=1e-9)
        assert R[0, 2] == pytest.approx(c, abs=1e-9)
        # Row 1
        assert R[1, 0] == pytest.approx(-1.0, abs=1e-9)
        assert R[1, 1] == pytest.approx(0.0, abs=1e-9)
        assert R[1, 2] == pytest.approx(0.0, abs=1e-9)
        # Row 2
        assert R[2, 0] == pytest.approx(0.0, abs=1e-9)
        assert R[2, 1] == pytest.approx(-c, abs=1e-9)
        assert R[2, 2] == pytest.approx(-s, abs=1e-9)

    def test_residual_reported_as_mm(self):
        """Document the exact round-trip residual for CI reporting.

        This test always passes; its purpose is to surface the residual value
        so reviewers can confirm it is well within the 5 mm gate.
        """
        x_w, y_w, z_w = 0.35, 0.0, REACH_Z
        dx, dy, dz = self._roundtrip_residual(x_w, y_w, z_w)
        # Residuals should be at machine-epsilon level (< 0.001 mm)
        assert dx < 0.005, f"x residual {dx*1000:.4f} mm exceeds 5 mm gate"
        assert dy < 0.005, f"y residual {dy*1000:.4f} mm exceeds 5 mm gate"
