"""Motion node — fuses nominal trajectory * safety scale (Stream D).

Architecture
------------
``MotionLogic``  -- pure-Python state + computation; no ROS.  Unit-testable
                    without a live ROS graph (AGENTS.md ground rule 3 / §9).

``MotionNode``   -- thin ROS 2 wrapper.  Wires the three subscribers and one
                    publisher; every real decision delegates to ``MotionLogic``.

Interface contract (AGENTS.md §3):
  Subscribes:  /motion/nominal_trajectory  (trajectory_msgs/JointTrajectory)
               /safety/scale               (std_msgs/Float32, 0.0-1.0)
               /joint_states               (sensor_msgs/JointState)
  Publishes:   /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

Stop/resume contract (AGENTS.md §4, §6):
  scale == 0.0  -> retime() returns None -> no motion command sent (no
                   div-by-zero, holds position via inaction).
  scale > 0 after stop -> resume re-plans from current joint state, NOT by
                   splicing back into the old trajectory (avoids jerk).
"""

from __future__ import annotations

from safecollab.retime import retime

# ROS 2 is not available in the pure-Python unit-test environment (see
# requirements.txt — rclpy is an apt package, not a pip package).
# Guard the imports so MotionLogic can be imported and tested without ROS.
try:
    import rclpy  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        HistoryPolicy,
    )  # pragma: no cover
    from builtin_interfaces.msg import Duration  # pragma: no cover
    from sensor_msgs.msg import JointState  # pragma: no cover
    from std_msgs.msg import Float32  # pragma: no cover
    from trajectory_msgs.msg import (
        JointTrajectory,
        JointTrajectoryPoint,
    )  # pragma: no cover

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]  # fallback base class


# ---------------------------------------------------------------------------
# Pure-Python logic (unit-testable without ROS)
# ---------------------------------------------------------------------------


class MotionLogic:
    """Pure-Python fusion logic for the motion node.

    Holds state (current scale, nominal trajectory, joint positions) and
    computes the re-timed trajectory command.  No ROS dependencies.

    Typical usage inside MotionNode callbacks::

        # scale callback:
        is_resume = self._logic.set_scale(msg.data)
        cmd = self._logic.compute_command(resuming=is_resume)
        if cmd is not None:
            self._publish(*cmd)

        # trajectory callback:
        self._logic.set_nominal_trajectory(names, times, positions)
        cmd = self._logic.compute_command()
        if cmd is not None:
            self._publish(*cmd)
    """

    def __init__(self) -> None:
        self._scale: float = 1.0
        # (joint_names, times, positions) — None until a trajectory arrives.
        self._nominal: tuple[list[str], list[float], list[list[float]]] | None = None
        self._joint_positions: list[float] | None = None
        self._stopped: bool = False

    # ------------------------------------------------------------------
    # State updates (called from ROS callbacks or from tests directly)
    # ------------------------------------------------------------------

    def set_scale(self, scale: float) -> bool:
        """Update the speed scale.

        Returns:
            True on a *resume* event (transition from scale==0 to scale>0).
            The caller should pass ``resuming=True`` to ``compute_command()``
            on the same tick so that the resume trajectory starts from the
            current joint state (§4 / §6 contract).

        Raises:
            ValueError: if *scale* is outside [0.0, 1.0].
        """
        if not 0.0 <= scale <= 1.0:
            raise ValueError(f"scale must be in [0.0, 1.0], got {scale!r}")
        was_stopped = self._stopped
        self._scale = scale
        self._stopped = scale == 0.0
        return was_stopped and not self._stopped  # True only on resume

    def set_nominal_trajectory(
        self,
        joint_names: list[str],
        times: list[float],
        positions: list[list[float]],
    ) -> None:
        """Store a new nominal (full-speed) trajectory.

        Replaces any previously stored trajectory.
        """
        self._nominal = (list(joint_names), list(times), list(positions))

    def set_joint_positions(self, positions: list[float]) -> None:
        """Update current joint positions (e.g. from /joint_states).

        Used only on the resume path to seed the first waypoint of the
        freshly-built trajectory.
        """
        self._joint_positions = list(positions)

    # ------------------------------------------------------------------
    # Command computation
    # ------------------------------------------------------------------

    def compute_command(
        self, *, resuming: bool = False
    ) -> tuple[list[str], list[float], list[list[float]]] | None:
        """Compute the re-timed trajectory ready for publication.

        Args:
            resuming: When True, prepend the current joint positions as
                      waypoint 0 at t=0 so the resumed trajectory starts
                      from the actual robot state — the §4/§6 "no jerk"
                      contract.  Falls back to the nominal trajectory if
                      no joint state has been received yet.

        Returns:
            ``(joint_names, new_times, positions)`` or ``None`` when:
              - no nominal trajectory has been set, or
              - scale == 0.0 (protective stop; ``retime()`` returns None).
        """
        if self._nominal is None:
            return None

        joint_names, times, positions = self._nominal

        if resuming and self._joint_positions is not None and positions:
            # Build a fresh trajectory starting from the current robot state.
            #
            # Prepend the halted position as waypoint 0 at t=0.  Shift the
            # nominal times forward by one inter-waypoint interval (dt) so
            # the controller has a non-zero time-budget to move from the
            # current state to the first nominal waypoint.
            #
            # Example (nominal times [0.0, 1.0, 2.0], dt=1.0):
            #   resume_times = [0.0, 1.0, 2.0, 3.0]
            #   After retime(scale=0.5): [0.0, 2.0, 4.0, 6.0]
            if len(times) >= 2:
                dt = times[1] - times[0]
            elif len(times) == 1:
                dt = times[0] if times[0] > 0.0 else 1.0
            else:
                dt = 1.0

            times_to_use = [0.0] + [t + dt for t in times]
            positions_to_use = [self._joint_positions] + list(positions)
        else:
            times_to_use = times
            positions_to_use = positions

        new_times = retime(times_to_use, self._scale)
        if new_times is None:
            # scale == 0.0: protective stop — do not emit a motion command.
            return None

        return (joint_names, new_times, positions_to_use)


# ---------------------------------------------------------------------------
# ROS 2 node wrapper (requires rclpy — not available in the pure-Python venv)
# ---------------------------------------------------------------------------


class MotionNode(Node):  # type: ignore[misc]  # pragma: no cover
    """ROS 2 node that fuses nominal trajectory * safety scale.

    Subscribes:
        /motion/nominal_trajectory  (trajectory_msgs/JointTrajectory)
        /safety/scale               (std_msgs/Float32)
        /joint_states               (sensor_msgs/JointState)

    Publishes:
        /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
    """

    def __init__(self) -> None:
        super().__init__("motion_node")  # type: ignore[call-arg]

        self._logic = MotionLogic()

        _reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            _reliable,
        )

        self.create_subscription(
            JointTrajectory,
            "/motion/nominal_trajectory",
            self._on_nominal_trajectory,
            _reliable,
        )

        self.create_subscription(
            Float32,
            "/safety/scale",
            self._on_scale,
            _reliable,
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

        cmd = self._logic.compute_command(resuming=False)
        if cmd is not None:
            self._publish(*cmd, frame_id=msg.header.frame_id)

    def _on_scale(self, msg: "Float32") -> None:
        """Update the safety scale and publish a re-timed command (or hold)."""
        is_resume = self._logic.set_scale(float(msg.data))
        cmd = self._logic.compute_command(resuming=is_resume)
        if cmd is not None:
            self._publish(*cmd)

    def _on_joint_states(self, msg: "JointState") -> None:
        """Track current joint positions for the resume re-plan."""
        self._logic.set_joint_positions(list(msg.position))

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


# ---------------------------------------------------------------------------
# Entry point for `ros2 run safecollab motion_node`
# ---------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:  # pragma: no cover
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — motion_node requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = MotionNode()
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
