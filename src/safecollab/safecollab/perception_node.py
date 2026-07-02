"""Perception node — classical CV operator detection (Stream C).

Architecture
------------
``PerceptionLogic``  -- pure-Python state + computation; no ROS.  Unit-testable
                        without a live ROS graph (AGENTS.md ground rule 3 / §9).

``PerceptionNode``   -- thin ROS 2 wrapper.  Wires the subscriber, publisher,
                        TF broadcaster, and loss-timeout timer; every real
                        decision delegates to ``PerceptionLogic``.

Interface contract (AGENTS.md §3):
  Subscribes:  /camera/image              (sensor_msgs/Image)
  Publishes:   /human/uncertainty         (std_msgs/Float32, σ in metres)
  Broadcasts:  TF world → human           (perceived, NOT ground truth)

Requirements (AGENTS.md §6):
  - Detection latency ≤ 100 ms (node processes one frame per callback).
  - σ grows on noisier / ambiguous detections (feeds Z_d in the risk model).
  - Loss timeout → downstream sees ``lost`` (TF goes stale; safety monitor
    treats a stale TF the same as ``d is None → "lost", 0.0``).

``/camera/image`` is fed by the ``ros_gz_image`` bridge wired in
``launch/cell.launch.py``. The pure-Python geometry (``back_project``,
``transform_point``, ``estimate_uncertainty``) and the state machine
(``PerceptionLogic``) are exercised by ``test/unit/test_perception_geom.py``.
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

# ROS 2 is not available in the pure-Python unit-test environment (see
# requirements.txt — rclpy is an apt package, not a pip package).
# Guard the imports so PerceptionLogic can be imported and tested without ROS.
try:
    import rclpy  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from cv_bridge import CvBridge  # pragma: no cover
    from geometry_msgs.msg import TransformStamped  # pragma: no cover
    from sensor_msgs.msg import Image  # pragma: no cover
    from std_msgs.msg import Float32  # pragma: no cover
    from tf2_ros import TransformBroadcaster  # pragma: no cover

    from safecollab._ros_runtime import (  # pragma: no cover
        best_effort_qos,
        reliable_qos,
        spin_and_shutdown,
    )

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]


# ---------------------------------------------------------------------------
# Camera geometry — pure Python, no ROS
# ---------------------------------------------------------------------------


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
# Uncertainty estimation — feeds Z_d in the risk model
# ---------------------------------------------------------------------------


def estimate_uncertainty(
    area_px: float,
    depth_m: float,
    confidence: float,
    *,
    base_sigma: float = 0.02,
    depth_coeff: float = 0.05,
    noise_coeff: float = 0.15,
    min_sigma: float = 0.01,
) -> float:
    """Estimate detection uncertainty σ (metres).

    σ grows with depth (farther objects are less precisely localised) and
    grows as confidence drops (noisier or smaller blobs → more uncertainty).

    This value is fed to ``Z_d`` in the ISO/TS 15066 risk model (``risk.py``
    ``thresholds()``), so a genuinely non-constant σ is **required** — the
    safety thresholds widen and narrow with actual perception quality.

    Model:

    .. code-block::

        σ = base_sigma
            + depth_coeff * depth_m               # grows with range
            + noise_coeff * (1 − confidence)      # grows as quality falls
        σ = max(σ, min_sigma)                     # always positive

    Args:
        area_px: Detected blob area in pixels² (accepted but primarily
                 ``confidence`` and ``depth_m`` drive σ; ``area_px`` is
                 available for callers that wish to pass it explicitly as an
                 additional quality proxy).
        depth_m: Estimated distance to the human (metres).
        confidence: Detection confidence in ``[0, 1]``.  ``1.0`` = very
                    certain; ``0.0`` = effectively a guess.
        base_sigma: Fixed floor of uncertainty even for a perfect detection.
        depth_coeff: Rate at which σ grows per metre of depth.
        noise_coeff: Maximum σ contribution from zero-confidence detection.
        min_sigma: Hard lower bound on the returned σ (metres).

    Returns:
        σ in metres (always ≥ ``min_sigma``).
    """
    depth_term = depth_coeff * depth_m
    confidence_term = noise_coeff * (1.0 - max(0.0, min(1.0, confidence)))
    sigma = base_sigma + depth_term + confidence_term
    return max(sigma, min_sigma)


# ---------------------------------------------------------------------------
# Human detection — classical OpenCV colour-blob pipeline
# ---------------------------------------------------------------------------


def detect_human(
    bgr_image: np.ndarray,
    *,
    min_area_px: float = 100.0,
    conf_area_ref_px: float = 2000.0,
    hue_low: int = 20,
    hue_high: int = 40,
    sat_low: int = 100,
    sat_high: int = 255,
    val_low: int = 100,
    val_high: int = 255,
) -> Optional[Tuple[float, float, float, float]]:
    """Detect a human in a BGR image using colour-based blob detection.

    Classical OpenCV pipeline: BGR → HSV → colour-range threshold →
    largest contour → centroid.

    In the Gazebo simulation the human marker uses a **yellow** (BGR
    ``[0, 255, 255]``, HSV hue ≈ 30) colour that falls within the default
    HSV range ``hue_low=20, hue_high=40``.  For real-camera deployment the
    HSV parameters should be tuned or the detector replaced with a skin-tone /
    motion-based approach.

    Args:
        bgr_image: OpenCV-style BGR image (H × W × 3 ``uint8``).
        min_area_px: Minimum blob area (pixels²) to accept as a valid
                     detection.  Smaller blobs are ignored.
        conf_area_ref_px: Reference blob area (pixels²) representing a full,
                     clean operator detection.  ``confidence`` saturates to 1.0
                     at or above this area and scales down for smaller/partial
                     blobs.  Sized to the operator marker under the overhead
                     camera so a normal detection is confident.
        hue_low, hue_high: HSV hue range (OpenCV ``[0, 180]`` convention).
        sat_low, sat_high: HSV saturation range.
        val_low, val_high: HSV value (brightness) range.

    Returns:
        ``(u_px, v_px, area_px2, confidence)`` or ``None`` if no human
        is found.

        - ``u_px``, ``v_px``: centroid pixel coordinates.
        - ``area_px2``: blob area in pixels².
        - ``confidence``: blob area relative to the expected operator size
          (``conf_area_ref_px``), clipped to ``(0, 1]`` — a full, solid blob is
          confident; a partial/occluded blob scores lower.
    """
    if bgr_image is None or bgr_image.size == 0:
        return None

    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([hue_low, sat_low, val_low], dtype=np.uint8),
        np.array([hue_high, sat_high, val_high], dtype=np.uint8),
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Take the single largest contour — assumes one dominant human blob.
    largest = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(largest))

    if area < min_area_px:
        return None

    moments = cv2.moments(largest)
    if moments["m00"] < 1e-6:  # pragma: no cover  # degenerate contour guard
        return None

    u = moments["m10"] / moments["m00"]
    v = moments["m01"] / moments["m00"]

    # Confidence = how completely the blob fills the EXPECTED operator size,
    # not its fraction of the whole frame. The old "area / image_area" made a
    # clearly-visible overhead marker (a few thousand px in a 640×480 frame)
    # score ~0.02 confidence, which maxed the σ noise term and inflated the
    # ISO/TS 15066 thresholds so far (d_yellow ≈ 1.0 m) that the operator was
    # NEVER far enough to register green in this cell. Normalising by a
    # reference blob area (conf_area_ref_px) makes a full, solid detection
    # confident (σ small → realistic thresholds → a real green window), while a
    # partial/occluded blob still scores low (σ grows → more conservative).
    confidence = min(1.0, area / conf_area_ref_px) if conf_area_ref_px > 0 else 0.0

    return u, v, area, confidence


# ---------------------------------------------------------------------------
# Pure-Python state machine — loss timeout & perceived pose
# ---------------------------------------------------------------------------


class PerceptionLogic:
    """Pure-Python perception state machine.

    Tracks the latest human detection and enforces a **loss timeout**: if no
    valid detection has been received within ``loss_timeout_s`` seconds the
    node transitions to ``is_lost=True``, which causes the ROS wrapper to
    stop broadcasting the ``world → human`` TF.  A stale / absent TF is how
    the safety monitor detects a lost operator (AGENTS.md §3, FR-9).

    Typical usage (inside ``PerceptionNode`` callbacks)::

        # image callback:
        detection = detect_human(frame)
        if detection is not None:
            u, v, area, conf = detection
            x, y, z = ...  # back-project + transform
            sigma = estimate_uncertainty(area, depth, conf)
            self._logic.update(position=(x, y, z), sigma=sigma,
                               timestamp_s=now_s)

        # timer callback (≥ 20 Hz):
        self._logic.check_timeout(current_time_s=now_s)
        if not self._logic.is_lost:
            self._broadcast_tf(*self._logic.position)
            self._publish_uncertainty(self._logic.sigma)
    """

    #: Default loss timeout (seconds).  Configurable via the constructor.
    DEFAULT_LOSS_TIMEOUT_S: float = 0.5

    def __init__(self, loss_timeout_s: float = DEFAULT_LOSS_TIMEOUT_S) -> None:
        """Initialise with an optional custom loss-timeout duration.

        Args:
            loss_timeout_s: Seconds without a valid detection before the
                            node declares the operator lost.
        """
        self._loss_timeout_s: float = loss_timeout_s
        self._last_detection_s: Optional[float] = None
        self._last_position: Optional[Tuple[float, float, float]] = None
        self._last_sigma: float = 0.10  # initial σ (unknown) — positive for risk model
        self._lost: bool = True

    # ------------------------------------------------------------------
    # State updates
    # ------------------------------------------------------------------

    def update(
        self,
        position: Optional[Tuple[float, float, float]],
        sigma: float,
        timestamp_s: float,
    ) -> None:
        """Update state with the result of processing one camera frame.

        Args:
            position: Estimated world-frame ``(x, y, z)`` of the operator,
                      or ``None`` when detection failed on this frame.
            sigma: Uncertainty estimate σ (metres) for this detection.
            timestamp_s: Timestamp of the processed frame (seconds, any
                         monotonic clock — just needs to be consistent with
                         the clock passed to :meth:`check_timeout`).
        """
        if position is not None:
            self._last_position = position
            self._last_sigma = sigma
            self._last_detection_s = timestamp_s
            self._lost = False
        # If position is None, we do NOT update _last_detection_s;
        # the timeout check is left to check_timeout() so the caller
        # controls when to enforce the loss (e.g., on a separate timer).

    def check_timeout(self, current_time_s: float) -> None:
        """Enforce the loss timeout.

        Should be called on each timer tick (≥ 20 Hz in the ROS node).
        If ``loss_timeout_s`` seconds have elapsed since the last valid
        detection (or if there has never been a detection), set ``is_lost``.

        Args:
            current_time_s: Current time in seconds (same clock as
                            timestamps passed to :meth:`update`).
        """
        if self._last_detection_s is None:
            self._lost = True
            return
        elapsed = current_time_s - self._last_detection_s
        if elapsed > self._loss_timeout_s:
            self._lost = True

    # ------------------------------------------------------------------
    # Read-only state accessors
    # ------------------------------------------------------------------

    @property
    def is_lost(self) -> bool:
        """``True`` when the operator is lost (no recent valid detection)."""
        return self._lost

    @property
    def position(self) -> Optional[Tuple[float, float, float]]:
        """Last known world-frame position, or ``None`` when ``is_lost``."""
        if self._lost:
            return None
        return self._last_position

    @property
    def sigma(self) -> float:
        """Latest uncertainty estimate σ (metres, always > 0)."""
        return self._last_sigma


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
    import math as _math

    fx = (width / 2.0) / _math.tan(hfov_rad / 2.0)
    fy = fx  # square pixels assumed
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy


# ---------------------------------------------------------------------------
# Camera-to-world geometry — pure Python, no ROS, fully unit-testable
# ---------------------------------------------------------------------------


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
    import math as _math

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
    cp = _math.cos(pitch_rad)
    sp = _math.sin(pitch_rad)
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


# ---------------------------------------------------------------------------
# ROS 2 node wrapper (requires rclpy — not available in the pure-Python venv)
# ---------------------------------------------------------------------------


class PerceptionNode(Node):  # type: ignore[misc]  # pragma: no cover
    """ROS 2 node that detects the operator and publishes perceived pose.

    Subscribes:
        /camera/image              (sensor_msgs/Image)

    Publishes:
        /human/uncertainty         (std_msgs/Float32, σ in metres, ≥ 20 Hz)

    Broadcasts:
        TF world → human           (perceived position; safety_monitor uses this)

    If no images arrive, the loss-timeout timer sets ``is_lost=True``, which
    drives the safety monitor into its fail-safe ``lost`` state — the correct
    and safe behaviour.
    """

    _WORLD_FRAME: str = "world"
    _HUMAN_FRAME: str = "human"
    _TIMER_HZ: float = 25.0  # tick rate for loss-timeout check + TF broadcast
    _LOSS_TIMEOUT_S: float = 0.5

    def __init__(self) -> None:
        super().__init__("perception_node")  # type: ignore[call-arg]

        # Load camera intrinsics from parameters (defaults from cell.xacro).
        self.declare_parameter("cam_width", _CAM_WIDTH)
        self.declare_parameter("cam_height", _CAM_HEIGHT)
        self.declare_parameter("cam_hfov_rad", _CAM_HFOV)
        self.declare_parameter("loss_timeout_s", self._LOSS_TIMEOUT_S)
        # Camera pose (world frame) from cell.xacro mast chain.
        # Defaults match xacro: camera_link at (-0.45, 0.0, 2.40),
        # mast_to_camera pitch = 1.2 rad.
        self.declare_parameter("cam_x", -0.45)
        self.declare_parameter("cam_y", 0.0)
        self.declare_parameter("cam_z", 2.40)
        self.declare_parameter("cam_pitch_rad", 1.2)
        # Plane height (world z, m) for the ray–plane back-projection. The
        # operator is represented to the camera by a flat yellow marker held at
        # this exact height (human_node._MARKER_Z), so the recovered (x, y) is
        # parallax-free and the perceived 'human' TF tracks the operator's
        # planar position across the whole path. MUST equal human_node._MARKER_Z.
        self.declare_parameter("plane_z_m", 0.95)

        width = int(self.get_parameter("cam_width").value)
        height = int(self.get_parameter("cam_height").value)
        hfov = float(self.get_parameter("cam_hfov_rad").value)
        loss_timeout = float(self.get_parameter("loss_timeout_s").value)

        self._fx, self._fy, self._cx, self._cy = _derive_intrinsics(width, height, hfov)
        self._plane_z = float(self.get_parameter("plane_z_m").value)
        self._logic = PerceptionLogic(loss_timeout_s=loss_timeout)

        # Build the calibrated camera→world transform from the true mast pose.
        cam_x = float(self.get_parameter("cam_x").value)
        cam_y = float(self.get_parameter("cam_y").value)
        cam_z = float(self.get_parameter("cam_z").value)
        cam_pitch = float(self.get_parameter("cam_pitch_rad").value)
        self._cam_to_world = cam_to_world_transform(
            cam_x=cam_x, cam_y=cam_y, cam_z=cam_z, pitch_rad=cam_pitch
        )

        self._bridge = CvBridge()
        self._br = TransformBroadcaster(self)

        self._pub_uncertainty = self.create_publisher(
            Float32,
            "/human/uncertainty",
            reliable_qos(),
        )

        self.create_subscription(
            Image,
            "/camera/image",
            self._on_image,
            best_effort_qos(depth=1),
        )

        self._timer = self.create_timer(1.0 / self._TIMER_HZ, self._tick)

        self.get_logger().info(
            "[perception_node] started; fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
            % (self._fx, self._fy, self._cx, self._cy)
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_image(self, msg: "Image") -> None:
        """Process one camera frame: detect human → update logic → broadcast."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"[perception_node] cv_bridge error: {exc}")
            return

        detection = detect_human(bgr)
        now_s = self.get_clock().now().nanoseconds * 1e-9

        if detection is not None:
            u, v, area, conf = detection
            intrinsics = (self._fx, self._fy, self._cx, self._cy)
            world_pt = project_pixel_to_plane(
                u, v, intrinsics, self._cam_to_world, self._plane_z
            )
            if world_pt is not None:
                x_w, y_w, z_w = world_pt
                # Depth = Euclidean distance camera → detected point.
                cam_pos = self._cam_to_world[:3, 3]
                depth_m = float(np.linalg.norm(np.array([x_w, y_w, z_w]) - cam_pos))
                sigma = estimate_uncertainty(area, depth_m, conf)
                self._logic.update(
                    position=(x_w, y_w, z_w), sigma=sigma, timestamp_s=now_s
                )
            else:
                # Ray doesn't intersect the plane (degenerate/oblique detection).
                self._logic.update(
                    position=None, sigma=self._logic.sigma, timestamp_s=now_s
                )
        else:
            # No detection this frame — let check_timeout handle the timeout.
            self._logic.update(
                position=None, sigma=self._logic.sigma, timestamp_s=now_s
            )

    # ------------------------------------------------------------------
    # Timer callback — loss timeout + TF + uncertainty publication
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        """25 Hz tick: enforce loss timeout, broadcast TF, publish uncertainty."""
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        self._logic.check_timeout(current_time_s=now_s)

        if not self._logic.is_lost:
            pos = self._logic.position
            if pos is not None:
                self._broadcast_human_tf(pos, now)
            self._pub_uncertainty.publish(Float32(data=float(self._logic.sigma)))
        # When lost: stop publishing / broadcasting so the safety monitor
        # detects the stale TF and enters fail-safe (FR-9).

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _broadcast_human_tf(
        self, position: Tuple[float, float, float], stamp: "rclpy.time.Time"
    ) -> None:
        """Broadcast the perceived ``world → human`` TF."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp.to_msg()
        tf_msg.header.frame_id = self._WORLD_FRAME
        tf_msg.child_frame_id = self._HUMAN_FRAME
        tf_msg.transform.translation.x = position[0]
        tf_msg.transform.translation.y = position[1]
        tf_msg.transform.translation.z = position[2]
        tf_msg.transform.rotation.w = 1.0  # point transform — no orientation
        self._br.sendTransform(tf_msg)


# ---------------------------------------------------------------------------
# Entry point for `ros2 run safecollab perception_node`
# ---------------------------------------------------------------------------


def main(args: list | None = None) -> None:  # pragma: no cover
    """Entry point for the ``perception_node`` ROS 2 executable."""
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — perception_node requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = PerceptionNode()
    spin_and_shutdown(node)
