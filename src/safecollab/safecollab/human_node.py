"""Simulated operator (human) node for the SafeCollab kitting cell.

Architecture note (AGENTS.md §3, §6 FR-11)
-------------------------------------------
``human_node`` drives the *ground-truth* operator model.  It:

* generates **randomisable** operator paths that include tray-reach waypoints
  (FR-11 / AT-6);
* broadcasts the ground-truth TF ``world → human_gt`` for simulation
  diagnostics;
* publishes **no safety output** — the safety monitor consumes the *perceived*
  ``world → human`` TF broadcast by ``perception_node`` (Stream C), not the
  ground truth.

Module layout
-------------
Pure-Python classes (``Waypoint``, ``OperatorPath``, ``OperatorModel``) are at
the top with **no ROS imports** — unit-testable without a live ROS graph.

The ROS 2 node wrapper lives inside ``main()`` (``# pragma: no cover``) so that
importing this module in a plain-Python test environment never fails due to a
missing ``rclpy`` installation.

Cell geometry (world frame, metres) — from ``urdf/cell.xacro``
---------------------------------------------------------------
* ``table_top`` : (0, 0, 0.74)
* ``kitting_tray`` centre: table_top + (0.35, 0, 0.02) → world (0.35, 0, 0.76)
* Tray footprint: x ∈ [0.20, 0.50], y ∈ [-0.20, +0.20], top surface z ≈ 0.78
* Operator stands at the front (+x) long side of the table.
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Cell geometry constants (from cell.xacro)
# ---------------------------------------------------------------------------

#: World-frame centre of the kitting tray (x, y, z).
TRAY_CENTRE: tuple[float, float, float] = (0.35, 0.0, 0.76)

#: Half-extents of the tray footprint (x and y).
TRAY_HALF_X: float = 0.15
TRAY_HALF_Y: float = 0.20

#: Top surface of the tray (z).
TRAY_TOP_Z: float = TRAY_CENTRE[2] + 0.02  # 0.78 m

#: Hand height of the operator when performing a tray reach.
REACH_Z: float = 0.82

#: Typical hand heights for standing / leaning poses.
STANDING_Z: float = 1.10
APPROACH_Z: float = 0.95

#: Seconds the operator stands clear of the cell at the start of each traversal
#: before approaching the tray — a contiguous GREEN window (operator ~0.9 m from
#: the arm's tray pose, beyond d_yellow) so the arm can run its MoveIt-planned
#: kitting between reaches. Kept short enough that the operator reaches the shared
#: tray often over a long run: with the UR5e's ~15 s planned cycle, too long a
#: dwell means too few tray reaches co-occur with the arm at the tray, so the
#: red protective stop (and AT-6's ≥3 escalations) become rare. Held fixed (not
#: scaled by the ±40 % approach-speed randomisation).
STANDING_DWELL_S: float = 5.0


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass
class Waypoint:
    """A 3-D world-frame position with an arrival time.

    Attributes:
        x: World-frame x coordinate (metres).
        y: World-frame y coordinate (metres).
        z: World-frame z coordinate (metres).
        t: Time to reach this waypoint from the path start (seconds).
    """

    x: float
    y: float
    z: float
    t: float

    @property
    def position(self) -> Tuple[float, float, float]:
        """(x, y, z) position tuple."""
        return (self.x, self.y, self.z)


def _lerp(a: float, b: float, alpha: float) -> float:
    """Linear interpolation between ``a`` and ``b`` by factor ``alpha``."""
    return a + alpha * (b - a)


def _interpolate_waypoints(
    waypoints: Sequence[Waypoint],
    t: float,
) -> Tuple[float, float, float]:
    """Linearly interpolate between consecutive waypoints at time ``t``.

    If ``t`` is before the first waypoint, the first position is returned.
    If ``t`` is after the last waypoint, the last position is returned.

    Args:
        waypoints: Ordered sequence of ``Waypoint`` objects.
        t: Query time in seconds.

    Returns:
        ``(x, y, z)`` world-frame position.
    """
    if not waypoints:
        return (0.0, 0.0, 0.0)

    t = max(0.0, t)

    # Walk segment-by-segment
    for i in range(len(waypoints) - 1):
        w0 = waypoints[i]
        w1 = waypoints[i + 1]
        if t <= w1.t:
            dt = w1.t - w0.t
            alpha = (t - w0.t) / dt if dt > 1e-12 else 0.0
            alpha = max(0.0, min(1.0, alpha))
            return (
                _lerp(w0.x, w1.x, alpha),
                _lerp(w0.y, w1.y, alpha),
                _lerp(w0.z, w1.z, alpha),
            )

    # Past the last waypoint
    last = waypoints[-1]
    return last.position


# ---------------------------------------------------------------------------
# Operator path
# ---------------------------------------------------------------------------


class OperatorPath:
    """A timed waypoint sequence for one operator traversal.

    The path **must** include at least one waypoint inside the kitting-tray
    horizontal footprint (a "tray reach") to satisfy FR-11.

    The path is generated by :meth:`generate_random`, which randomises:

    * which side of the table (+y or −y) the operator approaches from;
    * the exact tray-reach position within the tray footprint;
    * approach timing (±40 % speed variation).
    """

    def __init__(self, waypoints: List[Waypoint]) -> None:
        """Initialise with an ordered list of waypoints.

        Raises:
            ValueError: If ``waypoints`` is empty.
        """
        if not waypoints:
            raise ValueError("OperatorPath requires at least one waypoint")
        self._waypoints: List[Waypoint] = list(waypoints)

    @property
    def waypoints(self) -> List[Waypoint]:
        """Ordered list of waypoints (defensive copy)."""
        return list(self._waypoints)

    @property
    def duration_s(self) -> float:
        """Total duration of the path (arrival time of the last waypoint)."""
        return self._waypoints[-1].t

    def has_tray_reach(self) -> bool:
        """Return ``True`` if any waypoint lies within the tray footprint.

        "Within the tray footprint" means the x and y coordinates are within
        the tray's horizontal bounds (ignoring z so that above-tray and
        inside-tray positions both count).
        """
        cx, cy, _ = TRAY_CENTRE
        return any(
            abs(wp.x - cx) <= TRAY_HALF_X and abs(wp.y - cy) <= TRAY_HALF_Y
            for wp in self._waypoints
        )

    def tray_reach_waypoints(self) -> List[Waypoint]:
        """Return waypoints that lie inside the tray footprint."""
        cx, cy, _ = TRAY_CENTRE
        return [
            wp
            for wp in self._waypoints
            if abs(wp.x - cx) <= TRAY_HALF_X and abs(wp.y - cy) <= TRAY_HALF_Y
        ]

    @classmethod
    def generate_random(cls, rng: Optional[random.Random] = None) -> "OperatorPath":
        """Generate a random operator path that includes a tray reach.

        Randomises approach side, reach position within the tray footprint, and
        approach speed, satisfying FR-11 (AT-6 randomised operator paths).

        Args:
            rng: Optional seeded ``random.Random`` instance for reproducibility.
                 If ``None``, a fresh (unseeded) instance is used.

        Returns:
            A new ``OperatorPath`` whose :meth:`has_tray_reach` is ``True``.
        """
        if rng is None:
            rng = random.Random()

        # Operator approaches from the +y or −y side of the table
        side_y = rng.choice([0.65, -0.65])

        # Randomise the tray-reach position within the tray footprint
        reach_x = TRAY_CENTRE[0] + rng.uniform(-TRAY_HALF_X * 0.8, TRAY_HALF_X * 0.8)
        reach_y = rng.uniform(-TRAY_HALF_Y * 0.6, TRAY_HALF_Y * 0.6)

        # Timing scale (±40 % speed variation) applied to the approach/reach/
        # withdraw phase. The standing dwell is ALSO randomised per cycle: a fixed
        # dwell makes the operator's period near-commensurate with the arm's kitting
        # period, so their phases lock and a tray reach rarely coincides with the arm
        # at the tray (few red escalations). Varying the dwell breaks that lock so
        # the green→yellow→red escalation recurs across cycles (AT-6).
        t_scale = rng.uniform(0.7, 1.4)
        d = STANDING_DWELL_S * rng.uniform(0.5, 1.6)  # randomised GREEN window

        waypoints = [
            # 0 — standing well clear of the table (start of the dwell). Pushed back
            #     to ~1 m so the operator is unambiguously GREEN even while the arm
            #     is parked at the tray (see the DROP dwell in planner_node).
            Waypoint(x=1.0, y=side_y * 1.1, z=STANDING_Z, t=0.0),
            # 1 — still standing back (end of the dwell): a contiguous green window
            Waypoint(x=1.0, y=side_y * 1.1, z=STANDING_Z, t=round(d, 3)),
            # 2 — table edge, leaning toward tray area
            Waypoint(x=0.7, y=side_y * 0.4, z=APPROACH_Z, t=round(d + 2.0 * t_scale, 3)),
            # 3 — hover above tray
            Waypoint(x=reach_x, y=reach_y, z=APPROACH_Z, t=round(d + 4.0 * t_scale, 3)),
            # 4 — tray reach (hand inside tray) → drives the zone to red
            Waypoint(x=reach_x, y=reach_y, z=REACH_Z, t=round(d + 5.5 * t_scale, 3)),
            # 4b — HOLD the reach: the operator works in the shared tray for a few
            #      seconds. This lengthens the co-occupancy window so a reach reliably
            #      overlaps the arm's time at the tray, giving a red protective stop.
            Waypoint(x=reach_x, y=reach_y, z=REACH_Z, t=round(d + 9.5 * t_scale, 3)),
            # 5 — withdraw (hand back above tray)
            Waypoint(x=reach_x, y=reach_y, z=APPROACH_Z, t=round(d + 11.0 * t_scale, 3)),
            # 6 — step back from table
            Waypoint(x=0.7, y=side_y * 0.4, z=APPROACH_Z, t=round(d + 13.0 * t_scale, 3)),
            # 7 — return to standing (well clear again)
            Waypoint(x=1.0, y=side_y * 1.1, z=STANDING_Z, t=round(d + 15.0 * t_scale, 3)),
        ]

        return cls(waypoints)


# ---------------------------------------------------------------------------
# Operator model
# ---------------------------------------------------------------------------


class OperatorModel:
    """Simulated operator model: advances along an ``OperatorPath``.

    ``position_at(t)`` returns the world-frame ``(x, y, z)`` hand position at
    time ``t`` by linearly interpolating between consecutive waypoints.  When
    the path ends, :meth:`is_complete` returns ``True``; the caller (HumanNode)
    is responsible for generating and switching to a fresh path.

    This class contains no ROS imports and is fully unit-testable.
    """

    def __init__(self, path: OperatorPath) -> None:
        self._path = path

    @property
    def path(self) -> OperatorPath:
        """The current operator path."""
        return self._path

    @path.setter
    def path(self, new_path: OperatorPath) -> None:
        """Replace the current path (e.g., when a cycle ends)."""
        self._path = new_path

    def position_at(self, t: float) -> Tuple[float, float, float]:
        """World-frame ``(x, y, z)`` hand position at time ``t``.

        Clamps ``t`` to ``[0, duration_s]`` — does not loop; call
        :meth:`is_complete` to detect end-of-path.
        """
        return _interpolate_waypoints(self._path.waypoints, t)

    def is_complete(self, t: float) -> bool:
        """Return ``True`` when the path has ended at time ``t``."""
        return t >= self._path.duration_s

    @staticmethod
    def distance_to_point(
        pos: Tuple[float, float, float],
        target: Tuple[float, float, float],
    ) -> float:
        """Euclidean distance between two 3-D world-frame points."""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos, target)))


# ---------------------------------------------------------------------------
# ROS 2 node wrapper  (requires rclpy — excluded from unit-test coverage)
# ---------------------------------------------------------------------------


def main(args=None):  # pragma: no cover
    """Entry point for the ``human_node`` ROS 2 executable.

    All ROS imports are deferred to here so that importing this module in a
    plain-Python test environment (no ROS) does not raise ``ImportError``.

    The node broadcasts the **ground-truth** TF ``world → human_gt`` at 50 Hz.
    The frame is named ``human_gt`` (not ``human``) to distinguish it from the
    *perceived* ``world → human`` TF published by ``perception_node``; the safety
    monitor consumes only the perceived TF.

    Gz-transport pose following
    ---------------------------
    In addition to broadcasting the TF, this node moves the yellow operator
    visual body (spawned from ``urdf/operator.sdf`` by ``cell.launch.py``) by
    calling the gz sim service ``/world/empty/set_pose`` via the
    ``gz.transport13`` Python bindings.  This closes the perception loop:
    the overhead camera sees the yellow body and ``perception_node`` can
    detect the operator without requiring a physical person in the scene.

    The X/Y of the marker tracks the operator path; its z is held constant at
    ``_MARKER_Z`` (== ``perception_node`` ``plane_z_m``) so the camera-recovered
    planar pose is parallax-free (see ``_MARKER_Z``).

    The ``set_pose`` call runs on a **background worker thread**, never on the
    ROS timer.  ``gz.transport13.Node.request()`` is synchronous and, under sim
    load, can block for far longer than its timeout; calling it from the 50 Hz
    timer stalled the node and froze both the ``human_gt`` broadcast and the
    marker for seconds.  The timer now only stores the latest target pose; the
    worker drains it best-effort, so a slow ``set_pose`` can never stall path
    advancement or the TF broadcast.

    The gz-transport import is guarded: if ``python3-gz-transport13`` /
    ``python3-gz-msgs10`` are absent the node still functions correctly —
    the TF broadcast is unaffected, and only the visual body tracking is
    disabled (body stays at its spawn-time position).  This makes the node
    safe to start in environments where gz Python bindings are not installed.

    Why gz transport (not a ROS bridge)?
    * The gz ``/world/*/set_pose`` service is a gz-transport-only endpoint;
      no standard ``ros_gz_bridge`` service bridge exists for it, so a
      ROS-service approach would need custom bridge configuration.
    * Works headless: gz transport is pure middleware with no display
      dependency.
    """
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    # ------------------------------------------------------------------
    # Optional gz transport bindings for operator body pose following.
    # gz Harmonic ships gz-transport 13 and gz-msgs 10.
    # Package names (Ubuntu 24.04 OSRF apt repo):
    #   python3-gz-transport13  →  gz.transport13
    #   python3-gz-msgs10       →  gz.msgs10.pose_pb2 / gz.msgs10.boolean_pb2
    # These are listed as explicit apt deps in the Dockerfile.  If they are
    # absent the except branch runs and pose following is silently disabled.
    # ------------------------------------------------------------------
    _GzTransportNode = None
    _GzPose = None
    _GzBoolean = None
    try:
        from gz.transport13 import Node as _GzTransportNode  # type: ignore[import]
        from gz.msgs10.pose_pb2 import Pose as _GzPose  # type: ignore[import]
        from gz.msgs10.boolean_pb2 import Boolean as _GzBoolean  # type: ignore[import]
    except ImportError:
        pass  # gz Python bindings not installed; visual body tracking disabled

    class HumanNode(Node):
        """ROS 2 wrapper around ``OperatorModel``.

        Generates a random ``OperatorPath`` at start-up, then replaces it with a
        fresh random path each time the current path completes.  Broadcasts the
        ground-truth ``world → human_gt`` TF at 50 Hz and moves the yellow gz
        operator entity at 10 Hz so perception_node can detect it.
        """

        _GT_FRAME = "human_gt"
        _WORLD_FRAME = "world"
        _TIMER_HZ = 50.0

        # gz transport pose-following constants
        _GZ_SET_POSE_SVC: str = "/world/empty/set_pose"
        _GZ_ENTITY_NAME: str = "operator"

        # Height (world z, m) at which the yellow detection marker is held.
        # The marker tracks the operator in X/Y but stays at a constant working
        # height instead of following the hand's vertical reach (0.82..1.10).
        # Why: the single overhead camera recovers the operator's *planar* (x, y)
        # position by back-projecting the blob centroid onto one plane
        # (perception_node plane_z_m). Keeping the marker at exactly that plane
        # height makes the recovered X/Y match the true X/Y with no parallax —
        # so the perceived 'human' TF tracks the operator across the whole path,
        # not just during a tray reach. 0.95 m == APPROACH_Z, the operator's
        # dominant working height, which also keeps the 3-D residual against the
        # (vertically reaching) human_gt small everywhere. The true 3-D hand
        # pose is still published faithfully on the world->human_gt TF.
        _MARKER_Z: float = APPROACH_Z  # 0.95 m — must equal perception plane_z_m

        def __init__(self) -> None:
            super().__init__("human_node")

            # path_seed: integer ROS param for reproducible operator paths.
            # seed <= 0  → unseeded Random (non-deterministic, default for live demo).
            # seed > 0   → seeded Random(seed) — deterministic for scenario/AT tests.
            # The parameter may arrive as a string from launch args (LaunchConfiguration
            # substitution), so we coerce it to int defensively.
            _seed_val = self.declare_parameter("path_seed", 0).value
            if not isinstance(_seed_val, int):
                try:
                    _seed_val = int(_seed_val)
                except (TypeError, ValueError):
                    _seed_val = 0
            # random.Random(None) seeds from OS entropy — identical to Random().
            self._rng = random.Random(_seed_val if _seed_val > 0 else None)

            path = OperatorPath.generate_random(self._rng)
            self._model = OperatorModel(path)
            self._br = TransformBroadcaster(self)
            self._start_time: Optional[float] = None

            # Initialise gz transport node for pose following if bindings available.
            self._gz_node = _GzTransportNode() if _GzTransportNode is not None else None

            # Operator-body pose following runs on a BACKGROUND THREAD, never on
            # the ROS timer. gz.transport13's request() is synchronous and, under
            # sim load, can block for far longer than its timeout; calling it from
            # _tick() stalled the whole node — freezing the 50 Hz world->human_gt
            # broadcast AND the marker for seconds at a time (perception then saw a
            # frozen operator). The timer now only publishes the latest target
            # pose into _gz_target; the worker drains it best-effort, so a slow
            # set_pose can never stall path advancement or the TF broadcast.
            self._gz_target: Optional[Tuple[float, float, float]] = None
            self._gz_target_lock = threading.Lock()
            self._gz_worker: Optional[threading.Thread] = None
            if self._gz_node is not None:
                self._gz_worker = threading.Thread(
                    target=self._gz_pose_worker, daemon=True
                )
                self._gz_worker.start()

            self._timer = self.create_timer(1.0 / self._TIMER_HZ, self._tick)
            self.get_logger().info(
                "[human_node] started; broadcasting world -> human_gt at "
                f"{self._TIMER_HZ:.0f} Hz"
            )
            if self._gz_node is not None:
                self.get_logger().info(
                    "[human_node] gz transport available; operator body will "
                    "track the path at 10 Hz via /world/empty/set_pose"
                )
            else:
                self.get_logger().warning(
                    "[human_node] gz.transport13 not available "
                    "(python3-gz-transport13 / python3-gz-msgs10 not installed?); "
                    "operator body stays at spawn position — camera detection may fail"
                )

        def _tick(self) -> None:
            """Timer callback: advance the operator, broadcast TF, update gz pose."""
            now = self.get_clock().now()
            now_s = now.nanoseconds * 1e-9

            if self._start_time is None:
                self._start_time = now_s

            t = now_s - self._start_time

            # Regenerate path when current one is complete
            if self._model.is_complete(t):
                new_path = OperatorPath.generate_random(self._rng)
                self._model.path = new_path
                self._start_time = now_s
                t = 0.0
                self.get_logger().debug("[human_node] new operator path generated")

            x, y, z = self._model.position_at(t)

            # Broadcast the ground-truth TF (50 Hz — unchanged from before)
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self._WORLD_FRAME
            tf_msg.child_frame_id = self._GT_FRAME
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z
            tf_msg.transform.rotation.w = 1.0  # no rotation for the hand point
            self._br.sendTransform(tf_msg)

            # Publish the latest target for the background pose-following worker.
            # Track the operator in X/Y but hold the marker at the constant
            # perception plane height (see _MARKER_Z) — not the reaching hand's
            # z — so the camera-recovered planar pose is parallax-free. This is a
            # cheap, non-blocking store; the actual (potentially slow) set_pose
            # call happens on the worker thread, off the timer.
            if self._gz_node is not None:
                with self._gz_target_lock:
                    self._gz_target = (x, y, self._MARKER_Z)

        def _gz_pose_worker(self) -> None:
            """Background loop: push the latest target pose to gz, off the timer.

            Drains ``_gz_target`` at ~20 Hz and calls the synchronous (possibly
            slow) gz ``set_pose`` service here so it can never stall the ROS timer
            that advances the path and broadcasts the ground-truth TF.
            """
            import rclpy as _rclpy  # local: same guarded ROS dependency as main()

            while _rclpy.ok():
                target = None
                with self._gz_target_lock:
                    target = self._gz_target
                if target is not None:
                    self._set_gz_pose(*target)
                time.sleep(0.1)  # ~10 Hz best-effort — enough for smooth visual
                # tracking while keeping gz-service load (and headless CPU) low.

        def _set_gz_pose(self, x: float, y: float, z: float) -> None:
            """Move the yellow operator gz entity to ``(x, y, z)`` via set_pose.

            Runs on the background worker thread (see _gz_pose_worker). Calls the
            gz sim service ``/world/empty/set_pose``.  Failures are logged at
            DEBUG level only — the entity may not have been spawned yet in the
            first few ticks, and that is expected and harmless.
            """
            try:
                pose_msg = _GzPose()
                pose_msg.name = self._GZ_ENTITY_NAME
                pose_msg.position.x = x
                pose_msg.position.y = y
                pose_msg.position.z = z
                pose_msg.orientation.w = 1.0
                # gz.transport13 signature:
                #   request(service, request, request_type, response_type, timeout_ms)
                # The request_type (_GzPose) is REQUIRED — omitting it raises
                # TypeError, which the except below would silently swallow, so the
                # operator body would never move. (This was the original bug.)
                self._gz_node.request(
                    self._GZ_SET_POSE_SVC,
                    pose_msg,
                    _GzPose,
                    _GzBoolean,
                    100,  # ms; on the worker thread, so a slow call only delays
                    # the next marker update, never the ROS timer / TF broadcast.
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(
                    f"[human_node] gz set_pose failed (entity not yet spawned?): {exc}"
                )

    rclpy.init(args=args)
    node = HumanNode()
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


if __name__ == "__main__":  # pragma: no cover
    main()
