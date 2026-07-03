"""Safety-monitor ROS adapter — closes the SSM loop (Stream F).

Thin wrapper (``# pragma: no cover``): TF lookups, the ``/human/uncertainty``
subscriber, and four publishers; every decision delegates to
:class:`safecollab.safety.logic.SafetyMonitorLogic`.

Subscribes: /human/uncertainty (Float32, σ m) · TF world→human (perception) ·
            TF world→<robot_frames> (robot_state_publisher).
Publishes:  /safety/scale (Float32 0–1) → motion_node · /safety/zone (String) ·
            /safety/min_distance (Float32 m) · /viz/safety_marker (Marker).

Fail-safe (FR-9): a missing or stale (> ``loss_timeout``) human TF yields
``human_xyz=None`` → ``classify()`` returns ``("lost", 0.0)``. The last known
position is NEVER reused ("no dead-reckoning").
"""

from __future__ import annotations

from safecollab.safety.config import load_safety_config
from safecollab.safety.logic import SafetyMonitorLogic
from safecollab.safety.risk import load_config as load_risk_config

try:
    import rclpy  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from std_msgs.msg import Float32, String  # pragma: no cover
    from tf2_ros import Buffer, TransformListener  # pragma: no cover
    from visualization_msgs.msg import Marker  # pragma: no cover

    from safecollab._ros_runtime import (  # pragma: no cover
        best_effort_qos,
        config_path,
        reliable_qos,
        spin_and_shutdown,
        transient_qos,
    )

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]


class SafetyMonitorNode(Node):  # type: ignore[misc]  # pragma: no cover
    """ROS 2 node that closes the safety loop; publishes ``lost`` while the
    ``world → human`` TF (from perception_node) is missing or stale."""

    _WORLD_FRAME = "world"
    _HUMAN_FRAME = "human"
    _TICK_HZ = 20.0

    def __init__(self) -> None:
        super().__init__("safety_monitor")  # type: ignore[call-arg]

        # safety_source (§11 fallback #4): "perceived" (default, world→human from
        # perception_node — headline demo path) or "ground_truth" (world→human_gt
        # from human_node — fallback when perception is flaky). Unknown -> warn +
        # fall back to "perceived".
        _source = self.declare_parameter("safety_source", "perceived").value
        if _source == "ground_truth":
            self._HUMAN_FRAME = "human_gt"
        elif _source == "perceived":
            self._HUMAN_FRAME = "human"
        else:
            self.get_logger().warn(
                f"[safety_monitor] Unknown safety_source '{_source}'; "
                "defaulting to 'perceived' (world → human)."
            )
            self._HUMAN_FRAME = "human"
        self.get_logger().info(
            f"[safety_monitor] safety_source={_source!r} "
            f"→ TF frame '{self._HUMAN_FRAME}'"
        )

        risk_cfg = load_risk_config(config_path("risk.yaml"))
        safety_cfg = load_safety_config(config_path("safety.yaml"))
        self._logic = SafetyMonitorLogic(risk_cfg, safety_cfg)
        self._robot_frames: list[str] = safety_cfg.robot_frames

        self._uncertainty: float = 0.0  # updated by the subscriber callback

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers (§3 contract) — QoS matched per topic.
        self._scale_pub = self.create_publisher(
            Float32, "/safety/scale", reliable_qos()
        )
        self._zone_pub = self.create_publisher(String, "/safety/zone", transient_qos())
        self._dist_pub = self.create_publisher(
            Float32, "/safety/min_distance", best_effort_qos()
        )
        self._marker_pub = self.create_publisher(
            Marker, "/viz/safety_marker", best_effort_qos()
        )

        self.create_subscription(
            Float32,
            "/human/uncertainty",
            self._on_uncertainty,
            reliable_qos(),
        )

        self.create_timer(1.0 / self._TICK_HZ, self._tick)

        self.get_logger().info(
            f"[safety_monitor] started at {self._TICK_HZ:.0f} Hz; "
            f"loss_timeout={self._logic.loss_timeout:.2f} s"
        )

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _on_uncertainty(self, msg: "Float32") -> None:
        """Update perception σ from ``/human/uncertainty``."""
        self._uncertainty = float(msg.data)

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        """20 Hz: look up TF poses → classify → publish."""
        robot_frames_xyz = self._lookup_robot_frames()
        human_xyz = self._lookup_human_tf()

        zone, scale, dist, marker_p = self._logic.compute(
            robot_frames_xyz=robot_frames_xyz,
            human_xyz_or_none=human_xyz,
            uncertainty=self._uncertainty,
        )

        self._publish(zone, scale, dist, marker_p)

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _lookup_robot_frames(self) -> list[tuple[float, float, float]]:
        """World-frame positions for each configured robot frame; frames that
        can't be looked up are skipped (an empty list is the caller's fail-safe)."""
        positions = []
        for frame in self._robot_frames:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._WORLD_FRAME, frame, rclpy.time.Time()
                )
                positions.append(
                    (
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                    )
                )
            except Exception:  # noqa: BLE001
                pass
        return positions

    def _lookup_human_tf(self) -> tuple[float, float, float] | None:
        """Perceived human world-frame position, or ``None`` if missing/stale.

        ``rclpy.time.Time()`` requests the latest transform (TF2's
        ``ros::Time(0)``); we then check the stamp age manually rather than using
        TF2's blocking timeout, which would add latency on the fail-safe path.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._WORLD_FRAME,
                self._HUMAN_FRAME,
                rclpy.time.Time(),
            )
            now = self.get_clock().now()
            stamp = rclpy.time.Time.from_msg(t.header.stamp)
            age_s = (now - stamp).nanoseconds * 1e-9
            if age_s > self._logic.loss_timeout:
                return None  # stale — fail-safe
            return (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
        except Exception:  # noqa: BLE001
            return None  # missing — fail-safe

    # ------------------------------------------------------------------
    # Publication helper
    # ------------------------------------------------------------------

    def _publish(
        self,
        zone: str,
        scale: float,
        dist: float | None,
        marker_p: dict,
    ) -> None:
        """Publish zone, scale, distance, and the RViz sphere + text label."""
        scale_msg = Float32()
        scale_msg.data = scale
        self._scale_pub.publish(scale_msg)

        zone_msg = String()
        zone_msg.data = zone
        self._zone_pub.publish(zone_msg)

        # Publish 0.0 when lost — not a real distance, but keeps the topic live.
        dist_msg = Float32()
        dist_msg.data = dist if dist is not None else 0.0
        self._dist_pub.publish(dist_msg)

        # Zone sphere (id=0).
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._WORLD_FRAME
        marker.ns = "safety_monitor"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = marker_p["x"]
        marker.pose.position.y = marker_p["y"]
        marker.pose.position.z = marker_p["z"]
        marker.pose.orientation.w = 1.0
        d = marker_p["radius"] * 2.0
        marker.scale.x = marker.scale.y = marker.scale.z = d
        marker.color.r = marker_p["r"]
        marker.color.g = marker_p["g"]
        marker.color.b = marker_p["b"]
        marker.color.a = marker_p["a"]
        self._marker_pub.publish(marker)

        # Floating zone text label (id=1, distinct from the sphere so RViz shows
        # both). Shares the sphere colour but stays fully opaque (a=1.0) so the
        # word is legible even in the half-transparent 'lost' state.
        label = Marker()
        label.header.stamp = marker.header.stamp
        label.header.frame_id = self._WORLD_FRAME
        label.ns = "safety_monitor"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.text = marker_p["label"]
        label.pose.position.x = marker_p["x"]
        label.pose.position.y = marker_p["y"]
        label.pose.position.z = marker_p["label_z"]
        label.pose.orientation.w = 1.0
        label.scale.z = self._logic.marker_label_height
        label.color.r = marker_p["r"]
        label.color.g = marker_p["g"]
        label.color.b = marker_p["b"]
        label.color.a = 1.0
        self._marker_pub.publish(label)


def main(args: list | None = None) -> None:  # pragma: no cover
    """Entry point for ``ros2 run safecollab safety_monitor``."""
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — safety_monitor requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    spin_and_shutdown(node)
