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

Stream C "Must" requirements (AGENTS.md §6):
  - Detection latency ≤ 100 ms (node processes one frame per callback).
  - σ grows on noisier / ambiguous detections (feeds Z_d in the risk model).
  - Loss timeout → downstream sees ``lost`` (TF goes stale; safety monitor
    treats a stale TF the same as ``d is None → "lost", 0.0``).

Live-camera dependency note
---------------------------
The ``ros_gz_image`` bridge that feeds ``/camera/image`` with real Gazebo
frames is **BLOCKED on Stream G's ``cell.launch.py``**, which does not yet
exist (as of this PR).  The ``PerceptionNode`` ROS wrapper is implemented
against the documented topic/message contract (§3) and will wire up
automatically once Stream G provides the bridge.  Until then, the pure-Python
geometry (``back_project``, ``transform_point``, ``estimate_uncertainty``) and
the state machine (``PerceptionLogic``) are fully exercised by the unit tests
in ``test/unit/test_perception_geom.py``.
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
    from rclpy.qos import (  # pragma: no cover
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from cv_bridge import CvBridge  # pragma: no cover
    from geometry_msgs.msg import TransformStamped  # pragma: no cover
    from sensor_msgs.msg import Image  # pragma: no cover
    from std_msgs.msg import Float32  # pragma: no cover
    from tf2_ros import TransformBroadcaster  # pragma: no cover

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
        hue_low, hue_high: HSV hue range (OpenCV ``[0, 180]`` convention).
        sat_low, sat_high: HSV saturation range.
        val_low, val_high: HSV value (brightness) range.

    Returns:
        ``(u_px, v_px, area_px2, confidence)`` or ``None`` if no human
        is found.

        - ``u_px``, ``v_px``: centroid pixel coordinates.
        - ``area_px2``: blob area in pixels².
        - ``confidence``: ratio of blob area to image area, clipped to
          ``(0, 1]`` — a small blob in a large image gives low confidence.
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

    h, w = bgr_image.shape[:2]
    image_area = float(h * w)
    confidence = min(1.0, area / image_area) if image_area > 0 else 0.0

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
_CAM_HFOV: float = 1.0472  # 60 degrees

#: Derived focal length (assuming square pixels and horizontal FOV).
_CAM_FX: float = _CAM_WIDTH / (2.0 * 1.0472 / (2.0**0.5))  # approximate
_CAM_FX = (_CAM_WIDTH / 2.0) / ((_CAM_HFOV / 2.0) ** 0.5)  # corrected below


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

    Live-camera note:
        The ``ros_gz_image`` bridge for ``/camera/image`` is provided by
        ``launch/cell.launch.py`` (Stream G).  Until that file exists, this
        node will start but receive no images.  The loss-timeout timer will
        immediately set ``is_lost=True``, which causes the safety monitor to
        enter the fail-safe ``lost`` state — the correct and safe behaviour.
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
        # Approximate camera pose (world frame) — overridden by TF at runtime.
        # The camera is mounted above the table looking straight down.
        self.declare_parameter("cam_x", 0.0)
        self.declare_parameter("cam_y", 0.0)
        self.declare_parameter("cam_z", 2.0)  # 2 m above table surface
        # Known depth: distance from camera to table surface (metres).
        # Used for depth estimation from colour-only image.
        self.declare_parameter("table_depth_m", 1.26)  # 2.0 - 0.74 (table top)

        width = int(self.get_parameter("cam_width").value)
        height = int(self.get_parameter("cam_height").value)
        hfov = float(self.get_parameter("cam_hfov_rad").value)
        loss_timeout = float(self.get_parameter("loss_timeout_s").value)

        self._fx, self._fy, self._cx, self._cy = _derive_intrinsics(width, height, hfov)
        self._depth_m = float(self.get_parameter("table_depth_m").value)
        self._logic = PerceptionLogic(loss_timeout_s=loss_timeout)

        # Build an approximate camera→world transform from parameters.
        # The camera looks straight down: flip Y and Z (180° about X).
        cam_x = float(self.get_parameter("cam_x").value)
        cam_y = float(self.get_parameter("cam_y").value)
        cam_z = float(self.get_parameter("cam_z").value)
        self._cam_to_world = self._make_cam_to_world(cam_x, cam_y, cam_z)

        self._bridge = CvBridge()
        self._br = TransformBroadcaster(self)

        _reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        _sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_uncertainty = self.create_publisher(
            Float32,
            "/human/uncertainty",
            _reliable,
        )

        self.create_subscription(
            Image,
            "/camera/image",
            self._on_image,
            _sensor,
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
            x_cam, y_cam, z_cam = back_project(
                u, v, self._depth_m, self._fx, self._fy, self._cx, self._cy
            )
            x_w, y_w, z_w = transform_point((x_cam, y_cam, z_cam), self._cam_to_world)
            sigma = estimate_uncertainty(area, self._depth_m, conf)
            self._logic.update(position=(x_w, y_w, z_w), sigma=sigma, timestamp_s=now_s)
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

    @staticmethod
    def _make_cam_to_world(cam_x: float, cam_y: float, cam_z: float) -> np.ndarray:
        """Build a camera→world 4×4 for a top-down camera (looking straight down).

        The camera's +Z axis (optical axis) points downward in the world
        frame.  To bring camera coordinates into world coordinates, flip
        Y and Z (180° rotation about X), then translate by the camera pose.
        """
        rot_x_180 = np.eye(4, dtype=float)
        rot_x_180[1, 1] = -1.0
        rot_x_180[2, 2] = -1.0
        trans = np.eye(4, dtype=float)
        trans[0, 3] = cam_x
        trans[1, 3] = cam_y
        trans[2, 3] = cam_z
        return trans @ rot_x_180

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # On SIGINT, rclpy's default signal handler already shuts the context
        # down; calling rclpy.shutdown() again raises RCLError. Guard with ok().
        if rclpy.ok():
            rclpy.shutdown()
