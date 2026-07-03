"""Human (simulated operator) ROS adapter.

Thin wrapper (``# pragma: no cover``): drives the ground-truth ``OperatorModel``,
broadcasts ``world → human_gt`` at 50 Hz, and moves the yellow operator visual
body in gz so the overhead camera can see it.

Publishes no safety output — the safety monitor consumes the *perceived*
``world → human`` TF from perception_node, not this ground truth (named
``human_gt`` precisely so the two are never confused).
"""

from __future__ import annotations

import random
import threading
import time
from typing import Optional, Tuple

from safecollab.human.model import OperatorModel
from safecollab.human.path import APPROACH_Z, OperatorPath

try:
    import rclpy  # pragma: no cover
    from geometry_msgs.msg import TransformStamped  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from tf2_ros import TransformBroadcaster  # pragma: no cover

    from safecollab._ros_runtime import spin_and_shutdown  # pragma: no cover

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]

# Optional gz transport bindings for operator body pose following (gz Harmonic:
# python3-gz-transport13 / python3-gz-msgs10, listed in the Dockerfile). If
# absent, visual body tracking is silently disabled; the TF broadcast is
# unaffected.
_GzTransportNode = None
_GzPose = None
_GzBoolean = None
try:
    from gz.transport13 import Node as _GzTransportNode  # type: ignore[import] # pragma: no cover
    from gz.msgs10.pose_pb2 import Pose as _GzPose  # type: ignore[import] # pragma: no cover
    from gz.msgs10.boolean_pb2 import Boolean as _GzBoolean  # type: ignore[import] # pragma: no cover
except ImportError:
    pass  # gz Python bindings not installed; visual body tracking disabled


class HumanNode(Node):  # type: ignore[misc]  # pragma: no cover
    """ROS 2 wrapper around ``OperatorModel``.

    Generates a random ``OperatorPath`` at start-up and a fresh one each time the
    current path completes, broadcasting ``world → human_gt`` at 50 Hz.

    Gz-transport pose following: the node also moves the yellow operator visual
    body (``urdf/operator.sdf``) via the gz service ``/world/empty/set_pose`` so
    ``perception_node`` can detect it. That call runs on a **background worker**,
    never on the ROS timer: ``gz.transport13.Node.request()`` is synchronous and
    under sim load can block far past its timeout, which would freeze the
    ``human_gt`` broadcast. The timer only stores the latest target; the worker
    drains it best-effort.
    """

    _GT_FRAME = "human_gt"
    _WORLD_FRAME = "world"
    _TIMER_HZ = 50.0

    _GZ_SET_POSE_SVC: str = "/world/empty/set_pose"
    _GZ_ENTITY_NAME: str = "operator"

    # Height (world z, m) the yellow detection marker is held at. It tracks the
    # operator in X/Y but stays at this constant plane (== perception plane_z_m)
    # rather than following the hand's vertical reach, so the single overhead
    # camera recovers planar (x, y) with no parallax. The true 3-D hand pose still
    # goes out on world→human_gt.
    _MARKER_Z: float = APPROACH_Z  # 0.95 m — must equal perception plane_z_m

    def __init__(self) -> None:
        super().__init__("human_node")

        # path_seed: int ROS param for reproducible paths. seed <= 0 → unseeded
        # (non-deterministic, live-demo default); seed > 0 → Random(seed) for
        # scenario/AT tests. May arrive as a string from launch args, so coerce.
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

        self._gz_node = _GzTransportNode() if _GzTransportNode is not None else None

        # Pose following runs on a BACKGROUND THREAD, never on the ROS timer:
        # gz.transport13's request() is synchronous and under sim load can block
        # far past its timeout, which from _tick() froze the 50 Hz human_gt
        # broadcast AND the marker. The timer only stores the latest target; the
        # worker drains it best-effort.
        self._gz_target: Optional[Tuple[float, float, float]] = None
        self._gz_target_lock = threading.Lock()
        self._gz_worker: Optional[threading.Thread] = None
        if self._gz_node is not None:
            self._gz_worker = threading.Thread(target=self._gz_pose_worker, daemon=True)
            self._gz_worker.start()

        self._timer = self.create_timer(1.0 / self._TIMER_HZ, self._tick)
        self.get_logger().info(
            "[human_node] started; broadcasting world -> human_gt at "
            f"{self._TIMER_HZ:.0f} Hz"
        )
        if self._gz_node is not None:
            self.get_logger().info(
                "[human_node] gz transport available; operator body will "
                "track the path at 30 Hz via /world/empty/set_pose"
            )
        else:
            self.get_logger().warning(
                "[human_node] gz.transport13 not available "
                "(python3-gz-transport13 / python3-gz-msgs10 not installed?); "
                "operator body stays at spawn position — camera detection may fail"
            )

    def _tick(self) -> None:
        """Timer callback: advance the operator, broadcast TF, queue the gz pose."""
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        if self._start_time is None:
            self._start_time = now_s

        t = now_s - self._start_time

        # Regenerate the path once the current one completes.
        if self._model.is_complete(t):
            self._model.path = OperatorPath.generate_random(self._rng)
            self._start_time = now_s
            t = 0.0
            self.get_logger().debug("[human_node] new operator path generated")

        x, y, z = self._model.position_at(t)

        # Ground-truth TF (50 Hz).
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self._WORLD_FRAME
        tf_msg.child_frame_id = self._GT_FRAME
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.w = 1.0  # no rotation for the hand point
        self._br.sendTransform(tf_msg)

        # Queue the latest target for the worker, held at the constant perception
        # plane height (see _MARKER_Z) so the recovered planar pose is parallax-free.
        if self._gz_node is not None:
            with self._gz_target_lock:
                self._gz_target = (x, y, self._MARKER_Z)

    def _gz_pose_worker(self) -> None:
        """Background loop: push the latest target pose to gz at ~30 Hz, off the
        timer, so a slow set_pose can never stall path advancement or the TF."""
        while rclpy.ok():
            with self._gz_target_lock:
                target = self._gz_target
            if target is not None:
                self._set_gz_pose(*target)
            time.sleep(0.033)  # ~30 Hz best-effort

    def _set_gz_pose(self, x: float, y: float, z: float) -> None:
        """Move the yellow operator gz entity to ``(x, y, z)`` via set_pose.

        Runs on the worker thread. Failures log at DEBUG only — the entity may
        not have spawned in the first few ticks, which is expected.
        """
        try:
            pose_msg = _GzPose()
            pose_msg.name = self._GZ_ENTITY_NAME
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            pose_msg.orientation.w = 1.0
            # request(service, request, request_type, response_type, timeout_ms).
            # request_type (_GzPose) is REQUIRED — omitting it raises TypeError,
            # which the except would swallow, leaving the body frozen (the orig bug).
            self._gz_node.request(
                self._GZ_SET_POSE_SVC,
                pose_msg,
                _GzPose,
                _GzBoolean,
                100,  # ms; on the worker thread, so a slow call only delays the
                # next marker update, never the ROS timer / TF broadcast.
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(
                f"[human_node] gz set_pose failed (entity not yet spawned?): {exc}"
            )


def main(args=None):  # pragma: no cover
    """Entry point for ``ros2 run safecollab human_node``."""
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — human_node requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = HumanNode()
    spin_and_shutdown(node)


if __name__ == "__main__":  # pragma: no cover
    main()
