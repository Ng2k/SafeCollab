"""Perception ROS 2 adapter — wires the camera in and the perceived pose out.

Thin wrapper (``# pragma: no cover``): subscribes ``/camera/image``, delegates
every real decision to the pure modules (``detection``, ``geometry``,
``uncertainty``, ``state``), and broadcasts the perceived ``world → human`` TF plus
``/human/uncertainty``.

Interface contract (AGENTS.md §3):
  Subscribes:  /camera/image              (sensor_msgs/Image)
  Publishes:   /human/uncertainty         (std_msgs/Float32, σ in metres)
  Broadcasts:  TF world → human           (perceived, NOT ground truth)

``/camera/image`` is fed by the ``ros_gz_image`` bridge wired in
``launch/cell.launch.py``.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from safecollab.perception.detection import detect_human
from safecollab.perception.geometry import (
    _CAM_HFOV,
    _CAM_HEIGHT,
    _CAM_WIDTH,
    _derive_intrinsics,
    cam_to_world_transform,
    project_pixel_to_plane,
)
from safecollab.perception.state import PerceptionLogic
from safecollab.perception.uncertainty import estimate_uncertainty

# ROS 2 is not available in the pure-Python unit-test environment (rclpy is an
# apt package, not pip). Guard the imports so the pure modules above can still be
# imported and tested without ROS.
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
