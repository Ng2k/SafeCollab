"""Motion ROS adapter — fuses nominal trajectory × safety scale.

Thin wrapper (``# pragma: no cover``): wires three subscribers and one publisher;
every decision delegates to :class:`safecollab.motion.logic.MotionLogic`.

Subscribes: /motion/nominal_trajectory (JointTrajectory) · /safety/scale (Float32) ·
/joint_states (JointState).
Publishes:  /arm_controller/joint_trajectory (JointTrajectory).

Stop/resume (§4/§6): scale 0.0 → retime() None → no command (hold via inaction);
resume re-plans from the current joint state, not by splicing the old trajectory.
"""

from __future__ import annotations

from safecollab.motion.config import load_motion_config
from safecollab.motion.logic import MotionLogic

try:
    import rclpy  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from builtin_interfaces.msg import Duration  # pragma: no cover
    from sensor_msgs.msg import JointState  # pragma: no cover
    from std_msgs.msg import Float32  # pragma: no cover
    from trajectory_msgs.msg import (  # pragma: no cover
        JointTrajectory,
        JointTrajectoryPoint,
    )

    from safecollab._ros_runtime import (  # pragma: no cover
        config_path,
        reliable_qos,
        spin_and_shutdown,
    )

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]  # fallback base class


class MotionNode(Node):  # type: ignore[misc]  # pragma: no cover
    """ROS 2 node that fuses nominal trajectory × safety scale."""

    def __init__(self) -> None:
        super().__init__("motion_node")  # type: ignore[call-arg]

        # Config knobs live next to the package (ground rule 5).
        motion_cfg = load_motion_config(config_path("motion.yaml"))
        self._logic = MotionLogic(
            republish_scale_epsilon=motion_cfg.republish_scale_epsilon,
            hold_time_s=motion_cfg.hold_time_s,
        )

        self._pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            reliable_qos(),
        )

        self.create_subscription(
            JointTrajectory,
            "/motion/nominal_trajectory",
            self._on_nominal_trajectory,
            reliable_qos(),
        )

        self.create_subscription(
            Float32,
            "/safety/scale",
            self._on_scale,
            reliable_qos(),
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_nominal_trajectory(self, msg: "JointTrajectory") -> None:
        """Store the new nominal trajectory and publish a re-timed version."""
        times = [
            p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in msg.points
        ]
        positions = [list(p.positions) for p in msg.points]
        self._logic.set_nominal_trajectory(list(msg.joint_names), times, positions)

        cmd = self._logic.command_for_new_leg()
        if cmd is not None:
            _kind, names, new_times, new_positions = cmd
            self._publish(names, new_times, new_positions, frame_id=msg.header.frame_id)

    def _on_scale(self, msg: "Float32") -> None:
        """React to a new safety scale: re-time, hold, or (usually) do nothing.

        The dead-band and active-hold decisions live in
        MotionLogic.command_for_scale; most 20 Hz ticks carry an unchanged scale and
        return None, so the controller is not re-commanded every tick.
        """
        cmd = self._logic.command_for_scale(float(msg.data))
        if cmd is not None:
            _kind, names, new_times, new_positions = cmd
            self._publish(names, new_times, new_positions)

    def _on_joint_states(self, msg: "JointState") -> None:
        """Track current joint positions (and names) for the resume re-plan."""
        self._logic.set_joint_positions(list(msg.position), list(msg.name))

    # ------------------------------------------------------------------
    # Publication helper
    # ------------------------------------------------------------------

    def _publish(
        self,
        joint_names: list[str],
        times: list[float],
        positions: list[list[float]],
        *,
        frame_id: str = "",
    ) -> None:
        """Convert logic output to a JointTrajectory message and publish it."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.header.frame_id = frame_id
        traj.joint_names = joint_names

        for t, pos in zip(times, positions):
            pt = JointTrajectoryPoint()
            pt.positions = list(pos)
            sec = int(t)
            nanosec = int(round((t - sec) * 1e9))
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj.points.append(pt)

        self._pub.publish(traj)


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — motion_node requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = MotionNode()
    spin_and_shutdown(node)
