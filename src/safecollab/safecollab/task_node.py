"""Kitting-cell task node — owns the state machine and nominal trajectory.

Architecture note (AGENTS.md §3, §5.9)
---------------------------------------
``task_node`` **never** reasons about safety zones or speed scaling.  It emits
the *nominal* (full-speed) ``JointTrajectory`` for each leg of the kitting loop.
Speed adaptation is the sole responsibility of ``motion_node`` (Stream D), which
fuses the nominal trajectory with ``/safety/scale``.

Module layout
-------------
The pure-Python state machine classes (``KittingState``, ``KittingStateMachine``,
``TrajectoryLeg``) sit at the top of this file with **no ROS imports** so they are
directly importable by unit tests without a live ROS graph.

The ROS 2 node wrapper lives entirely inside ``main()`` (marked
``# pragma: no cover`` because it requires ``rclpy``) so that running
``python -m pytest`` on the unit-test suite never fails due to a missing ROS
installation.

Cell geometry (world frame, metres) — from ``urdf/cell.xacro``
---------------------------------------------------------------
* ``table_top`` : (0, 0, 0.74)
* ``bin_left``  : table_top + (-0.30, +0.25, 0.04) → world (-0.30, +0.25, 0.78)
* ``bin_right`` : table_top + (-0.30, -0.25, 0.04) → world (-0.30, -0.25, 0.78)
* ``kitting_tray`` centre: table_top + (0.35, 0, 0.02) → world (0.35, 0, 0.76)
* Arm base     : table_top + (-0.10, 0, 0) → world (-0.10, 0, 0.74)

Joint-space configurations are approximate (planning-level waypoints); exact
IK solutions require a live robot or an offline solver not available here.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Sequence

# ---------------------------------------------------------------------------
# Joint configuration constants
# ---------------------------------------------------------------------------

#: Names of the six revolute joints, in controller order (arm.xacro).
JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Approximate joint-space configurations for the kitting cell.
# Values: (pan, lift, elbow, wrist1, wrist2, wrist3) in radians.
# Geometric rationale:
#   bin_left  is ~128° CCW from arm +x → shoulder_pan ≈ +2.2 rad
#   bin_right is ~128° CW  from arm +x → shoulder_pan ≈ -2.2 rad
#   kitting_tray is directly forward (+x) → shoulder_pan ≈ 0.0 rad
#   All joint values are within the [-π, π] limits declared in arm.xacro.
_Q: dict[str, tuple[float, ...]] = {
    "home": (0.0, -1.0, 1.5, -0.5, 0.0, 0.0),
    "bin_left_above": (2.2, -0.5, 1.4, -0.8, 0.0, 0.0),
    "bin_left_pick": (2.2, -0.3, 1.1, -0.8, 0.0, 0.0),
    "bin_right_above": (-2.2, -0.5, 1.4, -0.8, 0.0, 0.0),
    "bin_right_pick": (-2.2, -0.3, 1.1, -0.8, 0.0, 0.0),
    "tray_above": (0.0, -0.6, 1.3, -0.7, 0.0, 0.0),
    "tray_drop": (0.0, -0.4, 1.0, -0.5, 0.0, 0.0),
}

# Nominal (full-speed) duration for each leg, in seconds.
# motion_node stretches these via retime() when the safety scale is < 1.0.
_LEG_DURATION: dict["KittingState", float] = {}  # populated after KittingState


# ---------------------------------------------------------------------------
# State-machine data types
# ---------------------------------------------------------------------------


class KittingState(enum.Enum):
    """States of the kitting loop state machine."""

    GO_TO_BIN = "GO_TO_BIN"
    PICK = "PICK"
    GO_TO_TRAY = "GO_TO_TRAY"
    DROP = "DROP"


# Populate leg durations (nominal, before safety scaling).
_LEG_DURATION = {
    KittingState.GO_TO_BIN: 3.0,
    KittingState.PICK: 1.5,
    KittingState.GO_TO_TRAY: 3.0,
    KittingState.DROP: 1.5,
}

# Deterministic, linear state transition.
_NEXT_STATE: dict[KittingState, KittingState] = {
    KittingState.GO_TO_BIN: KittingState.PICK,
    KittingState.PICK: KittingState.GO_TO_TRAY,
    KittingState.GO_TO_TRAY: KittingState.DROP,
    KittingState.DROP: KittingState.GO_TO_BIN,
}


@dataclass
class TrajectoryLeg:
    """A nominal joint-space trajectory for one leg of the kitting cycle.

    This is a pure-Python data structure; the ROS ``JointTrajectory`` message
    is built from it only when the node is running inside a live ROS graph.

    Attributes:
        joint_names: Ordered joint names (matches arm controller).
        waypoints: List of joint-position tuples (radians), one per waypoint.
        time_per_waypoint_s: Uniform time step between consecutive waypoints.
    """

    joint_names: tuple[str, ...]
    waypoints: list[tuple[float, ...]]
    time_per_waypoint_s: float

    @property
    def total_duration_s(self) -> float:
        """Total planned duration for the leg."""
        return self.time_per_waypoint_s * len(self.waypoints)

    @property
    def n_joints(self) -> int:
        """Number of joints in each waypoint."""
        return len(self.joint_names)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------


class KittingStateMachine:
    """Pure-Python kitting loop state machine.

    Cycle: ``GO_TO_BIN → PICK → GO_TO_TRAY → DROP → GO_TO_BIN → …``

    The machine alternates between ``bin_left`` and ``bin_right`` across kitting
    cycles so both bins are used.  The bin sequence is configurable so tests can
    exercise specific scenarios.

    No ROS imports — fully unit-testable without a live ROS graph.
    """

    def __init__(
        self,
        initial_state: KittingState = KittingState.GO_TO_BIN,
        bin_sequence: Sequence[str] = ("left", "right"),
    ) -> None:
        """Initialise the state machine.

        Args:
            initial_state: The state to start in (default ``GO_TO_BIN``).
            bin_sequence: Cycling bin labels.  Must be a non-empty sequence of
                ``"left"`` and/or ``"right"`` strings.

        Raises:
            ValueError: If ``bin_sequence`` is empty or contains invalid labels.
        """
        if not bin_sequence:
            raise ValueError("bin_sequence must not be empty")
        for label in bin_sequence:
            if label not in ("left", "right"):
                raise ValueError(
                    f"Invalid bin label {label!r}; expected 'left' or 'right'"
                )
        self._state = initial_state
        self._bin_sequence = tuple(bin_sequence)
        # _cycle_count increments each time the machine wraps back to GO_TO_BIN.
        self._cycle_count: int = 0

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def state(self) -> KittingState:
        """The current state."""
        return self._state

    @property
    def cycle_count(self) -> int:
        """Number of completed full cycles (DROP → GO_TO_BIN transitions)."""
        return self._cycle_count

    def current_bin(self) -> str:
        """Return ``'left'`` or ``'right'`` — the bin active for this cycle."""
        return self._bin_sequence[self._cycle_count % len(self._bin_sequence)]

    def advance(self) -> KittingState:
        """Transition to the next state and return it.

        Increments ``cycle_count`` each time the machine wraps back to
        ``GO_TO_BIN``.
        """
        self._state = _NEXT_STATE[self._state]
        if self._state == KittingState.GO_TO_BIN:
            self._cycle_count += 1
        return self._state

    def leg_duration_s(self) -> float:
        """Nominal (full-speed) duration for the current leg, in seconds."""
        return _LEG_DURATION[self._state]

    def current_leg(self) -> TrajectoryLeg:
        """Return the nominal ``TrajectoryLeg`` for the current state.

        The leg is a 2-waypoint trajectory: approach → terminal configuration.
        ``motion_node`` will re-time this with the live safety scale before
        commanding the arm controller.
        """
        bin_side = self.current_bin()

        if self._state == KittingState.GO_TO_BIN:
            waypoints = [_Q["home"], _Q[f"bin_{bin_side}_above"]]
        elif self._state == KittingState.PICK:
            waypoints = [_Q[f"bin_{bin_side}_above"], _Q[f"bin_{bin_side}_pick"]]
        elif self._state == KittingState.GO_TO_TRAY:
            waypoints = [_Q[f"bin_{bin_side}_pick"], _Q["tray_above"]]
        else:  # KittingState.DROP
            waypoints = [_Q["tray_above"], _Q["tray_drop"]]

        duration = _LEG_DURATION[self._state]
        return TrajectoryLeg(
            joint_names=JOINT_NAMES,
            waypoints=waypoints,
            time_per_waypoint_s=duration / len(waypoints),
        )


# ---------------------------------------------------------------------------
# ROS 2 node wrapper  (requires rclpy — excluded from unit-test coverage)
# ---------------------------------------------------------------------------


def main(args=None):  # pragma: no cover
    """Entry point for the ``task_node`` ROS 2 executable.

    All ROS imports are deferred to here so that importing this module in a
    plain-Python test environment (no ROS) does not raise ``ImportError``.
    """
    import rclpy
    from builtin_interfaces.msg import Duration as RosDuration
    from rclpy.node import Node
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    def _leg_to_ros_trajectory(leg: TrajectoryLeg) -> JointTrajectory:
        """Convert a ``TrajectoryLeg`` to a ``JointTrajectory`` ROS message."""
        traj = JointTrajectory()
        traj.joint_names = list(leg.joint_names)
        for i, positions in enumerate(leg.waypoints):
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            t = (i + 1) * leg.time_per_waypoint_s
            sec = int(t)
            nanosec = int((t - sec) * 1_000_000_000)
            pt.time_from_start = RosDuration(sec=sec, nanosec=nanosec)
            traj.points.append(pt)
        return traj

    class TaskNode(Node):
        """ROS 2 wrapper around ``KittingStateMachine``.

        Publishes the nominal ``JointTrajectory`` on ``/motion/nominal_trajectory``
        and the current state string on ``/task/state``.  Advances the state
        machine on a one-shot timer after each leg completes.
        """

        def __init__(self):
            super().__init__("task_node")
            self._sm = KittingStateMachine()
            self._traj_pub = self.create_publisher(
                JointTrajectory, "/motion/nominal_trajectory", 10
            )
            self._state_pub = self.create_publisher(String, "/task/state", 10)
            self._leg_timer = None
            self._start_leg()

        def _start_leg(self) -> None:
            """Publish the nominal trajectory and start the leg timer."""
            leg = self._sm.current_leg()
            traj = _leg_to_ros_trajectory(leg)
            traj.header.stamp = self.get_clock().now().to_msg()
            self._traj_pub.publish(traj)

            state_msg = String()
            state_msg.data = self._sm.state.value
            self._state_pub.publish(state_msg)

            self.get_logger().info(
                f"[task_node] state={self._sm.state.value}  "
                f"bin={self._sm.current_bin()}  "
                f"duration={self._sm.leg_duration_s():.1f}s"
            )

            duration = self._sm.leg_duration_s()
            self._leg_timer = self.create_timer(duration, self._on_leg_complete)

        def _on_leg_complete(self) -> None:
            """Timer callback: cancel current timer, advance SM, start next leg."""
            if self._leg_timer is not None:
                self._leg_timer.cancel()
                self._leg_timer = None
            self._sm.advance()
            self._start_leg()

    rclpy.init(args=args)
    node = TaskNode()
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


if __name__ == "__main__":  # pragma: no cover
    main()
