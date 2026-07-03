"""Camera geometry for perception — pure Python, no ROS.

Pinhole back-projection and the camera-optical-frame → world transform used to
recover the operator's world position from a detected pixel. Fully unit-testable
without a live ROS graph (AGENTS.md ground rule 3 / §9).

``project_pixel_to_plane`` is the geometry the node uses live (ray → plane
intersection for the angled mast camera). ``back_project`` + ``transform_point``
are the older fixed-depth primitives it superseded, kept for the unit tests that
document the pinhole model.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


def back_project(
    u: float,
    v: float,
    depth: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float, float]:
    """Back-project a pixel ``(u, v)`` and depth to a camera-frame 3-D point.

    Uses the standard pinhole camera model:

    .. code-block::

        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth

    Args:
        u: Pixel column coordinate.
        v: Pixel row coordinate.
        depth: Distance along the optical axis (metres).
        fx: Focal length in pixels (x direction).
        fy: Focal length in pixels (y direction).
        cx: Principal point column (pixels).
        cy: Principal point row (pixels).

    Returns:
        ``(x, y, z)`` in the camera frame (metres).
    """
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    return x, y, z


def transform_point(
    point: Tuple[float, float, float],
    matrix: np.ndarray,
) -> Tuple[float, float, float]:
    """Apply a 4×4 homogeneous transform matrix to a 3-D point.

    Args:
        point: ``(x, y, z)`` in the source frame (metres).
        matrix: 4×4 ``numpy.ndarray`` encoding rotation + translation
                (e.g. a camera-to-world pose transform).

    Returns:
        ``(x, y, z)`` in the target frame (metres), as plain Python
        ``float`` values (not NumPy scalars).
    """
    p = np.array([point[0], point[1], point[2], 1.0], dtype=float)
    result = matrix @ p
    return float(result[0]), float(result[1]), float(result[2])


# ---------------------------------------------------------------------------
# Default camera intrinsics (URDF camera — from cell.xacro)
# ---------------------------------------------------------------------------

#: Default image width for the simulated camera (pixels).
_CAM_WIDTH: int = 640

#: Default image height for the simulated camera (pixels).
_CAM_HEIGHT: int = 480

#: Default horizontal FOV for the gz camera (radians).
_CAM_HFOV: float = 1.5  # ~86 degrees — mirrors cell.xacro <horizontal_fov>


def _derive_intrinsics(
    width: int = _CAM_WIDTH,
    height: int = _CAM_HEIGHT,
    hfov_rad: float = _CAM_HFOV,
) -> Tuple[float, float, float, float]:
    """Derive pinhole intrinsics from image size and horizontal FOV.

    This is the formula Gazebo uses internally when generating the
    ``camera_info`` message for a ``<camera>`` sensor specified by
    ``<horizontal_fov>``.

    Returns:
        ``(fx, fy, cx, cy)`` in pixels.
    """
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx  # square pixels assumed
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


def cam_to_world_transform(
    *,
    cam_x: float = -0.45,
    cam_y: float = 0.0,
    cam_z: float = 2.40,
    pitch_rad: float = 1.2,
) -> np.ndarray:
    """Build the camera-optical-frame → world 4×4 homogeneous transform.

    Derived from the cell.xacro link chain (all joints are fixed):

    .. code-block::

        world
         └─ world_to_table  xyz=(0, 0, 0.37)
             └─ table_to_top  xyz=(0, 0, 0.37)   → table_top at z=0.74 m
                 └─ top_to_mast  xyz=(-0.55, 0, 0.85)
                     └─ mast_to_camera  xyz=(0.10, 0, 0.81) rpy=(0, 1.2, 0)
                         └─ camera_to_optical  rpy=(-π/2, 0, -π/2)

    This places ``camera_link`` at world ``(-0.45, 0, 2.40)`` with
    orientation ``Ry(1.2 rad)``.  The optical frame then adds
    ``Rz(−π/2) · Rx(−π/2)`` (URDF static/extrinsic RPY convention:
    ``rpy=(r, p, y)`` → ``Rz(y) · Ry(p) · Rx(r)``).

    Full rotation of the optical frame in world:

    .. code-block::

        R = Ry(pitch_rad) · Rz(−π/2) · Rx(−π/2)

    Args:
        cam_x: Camera-link x in world frame (m).  Default from xacro: −0.45.
        cam_y: Camera-link y in world frame (m).  Default from xacro:  0.00.
        cam_z: Camera-link z in world frame (m).  Default from xacro:  2.40.
        pitch_rad: Camera-body pitch in radians (``mast_to_camera`` rpy y).
                   Default 1.2 rad.

    Returns:
        4×4 ``numpy.ndarray`` (camera optical frame → world).
    """
    # Rx(−π/2): [1 0 0 / 0 0 1 / 0 −1 0]
    Rx_neg90 = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        dtype=float,
    )

    # Rz(−π/2): [0 1 0 / −1 0 0 / 0 0 1]
    Rz_neg90 = np.array(
        [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )

    # Ry(pitch_rad): camera-body pitch from mast_to_camera joint
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    Ry_pitch = np.array(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
        dtype=float,
    )

    # Full rotation: Ry(pitch) · Rz(−π/2) · Rx(−π/2)
    R = Ry_pitch @ Rz_neg90 @ Rx_neg90

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[0, 3] = cam_x
    T[1, 3] = cam_y
    T[2, 3] = cam_z
    return T


def project_pixel_to_plane(
    u: float,
    v: float,
    intrinsics: Tuple[float, float, float, float],
    cam_to_world: np.ndarray,
    plane_z: float,
) -> Optional[Tuple[float, float, float]]:
    """Back-project a pixel to a 3-D world point on a horizontal plane.

    Uses pinhole back-projection to form a ray in camera space, transforms
    the ray into the world frame via ``cam_to_world``, then solves for the
    intersection with the horizontal plane ``z = plane_z``.

    This supersedes the old fixed-depth approach (``back_project`` +
    ``transform_point`` with a constant depth) and correctly handles the
    angled mast-mounted camera geometry.

    The ray in camera frame is:

    .. code-block::

        d_cam = [(u − cx)/fx,  (v − cy)/fy,  1.0]

    The world-frame parametric ray is:

    .. code-block::

        P(s) = t_cam  +  s · (R · d_cam)

    Intersection with ``z = plane_z``:

    .. code-block::

        s = (plane_z − t_cam[2]) / (R · d_cam)[2]

    Args:
        u: Pixel column coordinate.
        v: Pixel row coordinate.
        intrinsics: ``(fx, fy, cx, cy)`` in pixels.
        cam_to_world: 4×4 camera-optical-frame → world transform
                      (e.g., from :func:`cam_to_world_transform`).
        plane_z: Height of the horizontal plane in world frame (metres).
                 Use ``REACH_Z = 0.82 m`` for the operator hand plane.

    Returns:
        ``(x, y, z)`` world-frame position as plain Python floats, or
        ``None`` if the ray is parallel to the plane (``d_world[z] ≈ 0``)
        or the intersection is behind the camera (``s < 0``).
    """
    fx, fy, cx, cy = intrinsics

    # Normalised direction in camera frame (unit z_cam = 1)
    d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)

    R = cam_to_world[:3, :3]
    t = cam_to_world[:3, 3]

    d_world = R @ d_cam

    # Solve P(s).z = plane_z
    if abs(d_world[2]) < 1e-10:
        return None  # ray is horizontal — no intersection

    s = (plane_z - t[2]) / d_world[2]
    if s < 0.0:
        return None  # intersection is behind the camera

    P = t + s * d_world
    return float(P[0]), float(P[1]), float(P[2])
