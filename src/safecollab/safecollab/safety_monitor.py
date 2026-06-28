"""Safety monitor — min-distance over robot frames → risk zone → scale (Stream F).

Architecture
------------
``SafetyMonitorLogic``  -- pure-Python; no ROS; unit-testable without a live
                           ROS graph (AGENTS.md ground rule 3 / §9).

``SafetyMonitorNode``   -- thin ROS 2 wrapper (``# pragma: no cover``).  Wires TF
                           lookups, the ``/human/uncertainty`` subscriber, and the
                           four publishers; every real decision delegates to Logic.

Interface contract (AGENTS.md §3):
  Subscribes:  /human/uncertainty   (std_msgs/Float32, σ m)  — perception_node
               TF world → human                              — perception_node
               TF world → <robot_frames>                     — robot_state_publisher
  Publishes:   /safety/scale        (std_msgs/Float32, 0.0–1.0)  → motion_node
               /safety/zone         (std_msgs/String, green|yellow|red|lost)
               /safety/min_distance (std_msgs/Float32, m)
               /viz/safety_marker   (visualization_msgs/Marker)

Fail-safe (FR-9, AGENTS.md §1 / §6 Stream F card):
  If the perceived human TF is missing or stale beyond ``loss_timeout`` seconds
  (from ``config/safety.yaml``), the ROS wrapper passes ``human_xyz=None`` to
  ``SafetyMonitorLogic.compute()``.  ``classify(None, …)`` returns
  ``("lost", 0.0)`` — a protective stop.  The last known position is NEVER
  assumed still valid ("no dead-reckoning" property).

Live integration note:
  ``perception_node`` (Stream C) is NOT yet merged.  The ROS wrapper's TF lookup
  for ``world → human`` and the ``/human/uncertainty`` subscriber are written
  against the §3 contract but cannot be verified end-to-end until Stream C lands.
  The pure-Python ``SafetyMonitorLogic`` is fully unit-tested with stub data.
"""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional, Tuple

import yaml

from safecollab.risk import load_config as load_risk_config
from safecollab.risk import thresholds
from safecollab.safety_logic import classify

# ---------------------------------------------------------------------------
# ROS 2 guard — same pattern as motion_node.py
# ---------------------------------------------------------------------------

try:
    import rclpy  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from rclpy.qos import (  # pragma: no cover
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Float32, String  # pragma: no cover
    from tf2_ros import Buffer, TransformListener  # pragma: no cover
    from visualization_msgs.msg import Marker  # pragma: no cover

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------


def load_safety_config(path) -> SimpleNamespace:
    """Load the ``safety:`` block of ``config/safety.yaml`` into a namespace.

    Returns a ``SimpleNamespace`` with attributes:
        ``s_min``       — minimum speed scale in the yellow band.
        ``loss_timeout`` — stale-TF grace period in seconds.
        ``robot_frames`` — list of TF frame names for the min-distance sweep.
        ``marker_radius`` — RViz sphere radius in metres.
    """
    with open(Path(path), "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    safety = data["safety"]
    return SimpleNamespace(
        s_min=float(safety["s_min"]),
        loss_timeout=float(safety["loss_timeout"]),
        robot_frames=list(safety.get("robot_frames", ["tcp", "wrist", "elbow"])),
        marker_radius=float(safety.get("marker_radius", 0.15)),
    )


# ---------------------------------------------------------------------------
# Zone → RGBA colour mapping for RViz markers
# ---------------------------------------------------------------------------

#: RGBA colours per zone: (r, g, b, alpha).  Alpha 0.8 for active zones; 0.4
#: for the ``lost`` state (greyed-out to signal perception failure).
_ZONE_RGBA: dict[str, tuple[float, float, float, float]] = {
    "green": (0.0, 1.0, 0.0, 0.8),
    "yellow": (1.0, 1.0, 0.0, 0.8),
    "red": (1.0, 0.0, 0.0, 0.8),
    "lost": (0.5, 0.5, 0.5, 0.4),
}


# ---------------------------------------------------------------------------
# Pure-Python logic (unit-testable without ROS)
# ---------------------------------------------------------------------------


class SafetyMonitorLogic:
    """Pure-Python safety monitor logic.

    Computes minimum separation between robot frames and the perceived human
    position, calls ``safety_logic.classify()`` (Stream B), and returns zone,
    speed scale, distance, and RViz marker parameters.

    No ROS dependencies; fully testable with stub data (AGENTS.md ground rule 3).

    Typical usage inside ``SafetyMonitorNode._tick()``::

        zone, scale, dist, marker_p = self._logic.compute(
            robot_frames_xyz=[(tcp_x, tcp_y, tcp_z), ...],
            human_xyz_or_none=(hx, hy, hz),   # None when TF is stale/missing
            uncertainty=sigma,
        )
    """

    def __init__(
        self,
        risk_cfg: SimpleNamespace,
        safety_cfg: SimpleNamespace,
    ) -> None:
        """Initialise with pre-loaded config namespaces.

        Args:
            risk_cfg:   Loaded from ``config/risk.yaml`` via ``risk.load_config()``.
            safety_cfg: Loaded from ``config/safety.yaml`` via
                        :func:`load_safety_config`.
        """
        self._risk_cfg = risk_cfg
        self._s_min = safety_cfg.s_min
        self._loss_timeout = safety_cfg.loss_timeout
        self._marker_radius = safety_cfg.marker_radius

    # ------------------------------------------------------------------
    # Public read-only properties (consumed by the ROS wrapper)
    # ------------------------------------------------------------------

    @property
    def loss_timeout(self) -> float:
        """Stale-TF timeout in seconds (read by the ROS wrapper).

        The ROS wrapper uses this value to decide when a ``world → human``
        transform is too old to trust, and passes ``human_xyz=None`` to
        :meth:`compute` accordingly.
        """
        return self._loss_timeout

    @property
    def marker_radius(self) -> float:
        """Radius of the RViz safety-zone sphere in metres."""
        return self._marker_radius

    # ------------------------------------------------------------------
    # Static helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _min_distance(
        robot_frames_xyz: List[Tuple[float, float, float]],
        human_xyz: Tuple[float, float, float],
    ) -> float:
        """Euclidean minimum distance from any robot frame to the human.

        Args:
            robot_frames_xyz: List of ``(x, y, z)`` world-frame positions for
                              TCP, wrist, elbow (or whatever the caller provides).
            human_xyz:        Perceived human position ``(x, y, z)`` in the
                              world frame.

        Returns:
            The minimum Euclidean distance (metres) over all robot frames.
        """
        hx, hy, hz = human_xyz
        min_d = math.inf
        for rx, ry, rz in robot_frames_xyz:
            d = math.sqrt((rx - hx) ** 2 + (ry - hy) ** 2 + (rz - hz) ** 2)
            if d < min_d:
                min_d = d
        return min_d

    @staticmethod
    def _marker_params(
        zone: str,
        human_xyz_or_none: Optional[Tuple[float, float, float]],
        radius: float,
    ) -> dict:
        """Build RViz marker parameters for the safety zone.

        Returns a plain ``dict``; the ROS wrapper converts it into a
        ``visualization_msgs/Marker`` message.

        Keys:
            ``r``, ``g``, ``b``, ``a`` — RGBA colour components (0.0–1.0).
            ``x``, ``y``, ``z``       — centre of the sphere (human position,
                                        or world origin when ``None``).
            ``radius``                 — sphere radius in metres.
            ``zone``                   — zone string (for callers that need it).
        """
        r, g, b, a = _ZONE_RGBA.get(zone, (0.5, 0.5, 0.5, 0.4))
        if human_xyz_or_none is not None:
            x, y, z = human_xyz_or_none
        else:
            x, y, z = 0.0, 0.0, 0.0
        return {
            "r": r,
            "g": g,
            "b": b,
            "a": a,
            "x": x,
            "y": y,
            "z": z,
            "radius": radius,
            "zone": zone,
        }

    # ------------------------------------------------------------------
    # Main computation
    # ------------------------------------------------------------------

    def compute(
        self,
        robot_frames_xyz: List[Tuple[float, float, float]],
        human_xyz_or_none: Optional[Tuple[float, float, float]],
        uncertainty: float,
    ) -> Tuple[str, float, Optional[float], dict]:
        """Compute zone, speed scale, minimum distance, and RViz marker params.

        This is the hot path called on every tick.

        Args:
            robot_frames_xyz:  World-frame positions of TCP, wrist, elbow (or
                               whichever frames are available).  An empty list
                               is treated as a fail-safe (no robot knowledge).
            human_xyz_or_none: Perceived human position, or ``None`` when the
                               ``world → human`` TF is missing or stale beyond
                               ``loss_timeout`` (see :attr:`loss_timeout`).
            uncertainty:       Perception uncertainty σ (metres) from
                               ``/human/uncertainty``.  Widens the ISO/TS 15066
                               thresholds via ``risk.thresholds()``.

        Returns:
            A four-tuple ``(zone, scale, min_dist_or_none, marker_params)``:

            * ``zone``           — ``"green" | "yellow" | "red" | "lost"``
            * ``scale``          — float in ``[0.0, 1.0]``
            * ``min_dist_or_none`` — minimum separation (m), or ``None`` when
                                     ``d`` was not computable (fail-safe path).
            * ``marker_params``  — ``dict`` ready for the RViz Marker message.

        Fail-safe guarantee:
            If ``human_xyz_or_none is None`` or ``robot_frames_xyz`` is empty,
            ``d`` is set to ``None`` and ``classify()`` returns
            ``("lost", 0.0)`` — a protective stop (FR-9).
        """
        # Determine effective separation distance -------------------------
        if human_xyz_or_none is None or not robot_frames_xyz:
            # Fail-safe: perception lost/stale, or no robot frame data.
            # Pass d=None to classify() — it returns ("lost", 0.0) per FR-9.
            d: Optional[float] = None
        else:
            d = self._min_distance(robot_frames_xyz, human_xyz_or_none)

        # Risk-model thresholds (widened by live perception uncertainty) ---
        d_red, d_yellow = thresholds(self._risk_cfg, uncertainty)

        # Zone classification (Stream B's function — not a local copy) ----
        zone, scale = classify(d, d_red=d_red, d_yellow=d_yellow, s_min=self._s_min)

        # RViz marker parameters ------------------------------------------
        marker = self._marker_params(zone, human_xyz_or_none, self._marker_radius)

        return zone, scale, d, marker


# ---------------------------------------------------------------------------
# ROS 2 node wrapper (requires rclpy — not available in the pure-Python venv)
# ---------------------------------------------------------------------------


class SafetyMonitorNode(Node):  # type: ignore[misc]  # pragma: no cover
    """ROS 2 node that closes the safety loop.

    Subscribes:
        ``/human/uncertainty``  (std_msgs/Float32) — perception uncertainty σ
        TF ``world → human``                       — perceived human pose
        TF ``world → <robot_frames>``              — robot frame poses

    Publishes:
        ``/safety/scale``        (std_msgs/Float32)
        ``/safety/zone``         (std_msgs/String)
        ``/safety/min_distance`` (std_msgs/Float32)
        ``/viz/safety_marker``   (visualization_msgs/Marker)

    Live integration with ``perception_node`` (Stream C) is required for the
    ``world → human`` TF to be available.  Until Stream C is merged, the node
    starts but publishes ``lost`` on every tick (fail-safe default).
    """

    _WORLD_FRAME = "world"
    _HUMAN_FRAME = "human"
    _TICK_HZ = 20.0

    def __init__(self) -> None:
        super().__init__("safety_monitor")  # type: ignore[call-arg]

        # ------------------------------------------------------------------
        # safety_source parameter — §11 cut-scope fallback #4
        #
        # "perceived"    (default): world → human TF from perception_node.
        #                           Headline demo path; required by DoD #5.
        # "ground_truth"          : world → human_gt TF from human_node.
        #                           Demo fallback when perception is flaky/absent.
        #
        # Any other value is rejected with a warning and falls back to "perceived".
        # ------------------------------------------------------------------
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

        # Resolve config paths relative to this file (colcon install layout
        # and local editable install both put config/ next to safecollab/).
        _here = Path(__file__).resolve().parent
        _risk_yaml = _here.parent / "config" / "risk.yaml"
        _safety_yaml = _here.parent / "config" / "safety.yaml"

        risk_cfg = load_risk_config(_risk_yaml)
        safety_cfg = load_safety_config(_safety_yaml)
        self._logic = SafetyMonitorLogic(risk_cfg, safety_cfg)
        self._robot_frames: List[str] = safety_cfg.robot_frames

        # Perception uncertainty (updated by subscriber callback)
        self._uncertainty: float = 0.0

        # TF infrastructure
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # QoS profiles
        _reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        _transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        _best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers (§3 contract)
        self._scale_pub = self.create_publisher(Float32, "/safety/scale", _reliable)
        self._zone_pub = self.create_publisher(String, "/safety/zone", _transient)
        self._dist_pub = self.create_publisher(
            Float32, "/safety/min_distance", _best_effort
        )
        self._marker_pub = self.create_publisher(
            Marker, "/viz/safety_marker", _best_effort
        )

        # Subscriber — perception uncertainty
        self.create_subscription(
            Float32,
            "/human/uncertainty",
            self._on_uncertainty,
            _reliable,
        )

        # 20 Hz timer — main computation tick
        self.create_timer(1.0 / self._TICK_HZ, self._tick)

        self.get_logger().info(
            f"[safety_monitor] started at {self._TICK_HZ:.0f} Hz; "
            f"loss_timeout={self._logic.loss_timeout:.2f} s"
        )

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _on_uncertainty(self, msg: "Float32") -> None:
        """Update the perception uncertainty σ from ``/human/uncertainty``."""
        self._uncertainty = float(msg.data)

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        """20 Hz computation: look up TF poses → classify → publish."""
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

    def _lookup_robot_frames(self) -> List[Tuple[float, float, float]]:
        """Return world-frame positions for each configured robot frame.

        Frames that cannot be looked up (e.g. not yet available) are silently
        skipped — the caller handles an empty list as a fail-safe.
        """
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

    def _lookup_human_tf(self) -> Optional[Tuple[float, float, float]]:
        """Return the perceived human world-frame position, or ``None``.

        A transform older than ``loss_timeout`` is treated as stale — the
        function returns ``None``, which triggers the fail-safe in
        :meth:`SafetyMonitorLogic.compute`.

        Notes:
            ``rclpy.time.Time()`` requests the *latest available* transform
            (equivalent to TF2's ``ros::Time(0)``).  We then manually check the
            stamp age rather than relying on TF2's timeout, because the TF2
            timeout blocks the calling thread until the transform arrives —
            which would introduce latency on the fail-safe path.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._WORLD_FRAME,
                self._HUMAN_FRAME,
                rclpy.time.Time(),
            )
            # Staleness check: reject transforms older than loss_timeout
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
        dist: Optional[float],
        marker_p: dict,
    ) -> None:
        """Publish zone, scale, distance, and RViz marker."""
        # /safety/scale
        scale_msg = Float32()
        scale_msg.data = scale
        self._scale_pub.publish(scale_msg)

        # /safety/zone
        zone_msg = String()
        zone_msg.data = zone
        self._zone_pub.publish(zone_msg)

        # /safety/min_distance (publish 0.0 when lost — not a real distance,
        # but avoids silently dropping the topic during the fail-safe state)
        dist_msg = Float32()
        dist_msg.data = dist if dist is not None else 0.0
        self._dist_pub.publish(dist_msg)

        # /viz/safety_marker
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


# ---------------------------------------------------------------------------
# Entry point for `ros2 run safecollab safety_monitor`
# ---------------------------------------------------------------------------


def main(args: list | None = None) -> None:  # pragma: no cover
    """ROS 2 entry point."""
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — safety_monitor requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # Teardown race: rclpy's executor can raise from take_message (e.g. on
        # the /clock subscription that use_sim_time creates) if it is mid-take
        # when the SIGINT handler shuts the context down. It surfaces as a pybind
        # "Unable to convert call argument" error from _take_subscription.
        # rclpy.ok() is an unreliable discriminator (it can still report True for
        # a tick during teardown), so also treat that specific take-time error as
        # benign; re-raise anything else so real bugs still surface.
        if rclpy.ok() and "convert call argument" not in str(exc):
            raise
    finally:
        node.destroy_node()
        # On SIGINT, rclpy's default signal handler already shuts the context
        # down; calling rclpy.shutdown() again raises RCLError. Guard with ok().
        if rclpy.ok():
            rclpy.shutdown()
