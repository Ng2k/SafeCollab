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

Joint-space configurations are FK-verified to place the TCP at the workspace
targets (tray / bins at table height, z ≈ 0.80 m), solved offline against the
arm.xacro kinematic chain.  This matters for safety as well as fidelity: if the
arm never descends into the shared tray, no robot frame ever comes within the
red-zone separation of an operator reaching that tray, and the cell could never
demonstrate a protective stop (AT-3, a §9 "never cut" acceptance test).
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

# Joint-space configurations for the kitting cell, IK-solved against the
# arm.xacro chain (see module docstring).  Values: (pan, lift, elbow, wrist1,
# wrist2, wrist3) in radians.
# Geometric rationale:
#   bin_left  is ~129° CCW from arm +x → shoulder_pan ≈ +2.25 rad
#   bin_right is ~129° CW  from arm +x → shoulder_pan ≈ -2.25 rad
#   kitting_tray is directly forward (+x) → shoulder_pan ≈ 0.0 rad
#   lift/elbow/wrist_1 form a planar sub-chain (all about y) solved with a
#   closed-form 2R + tool-down constraint on the SHORT wrist (l_4..l_6): the
#   elbow-up branch reaches each TCP target with the tool pointing straight down,
#   so from the elbow the chain descends monotonically in both height and radius
#   to the target — a clean inverted-V reach, NOT the forearm folding back over
#   the upper arm (the earlier long-link/long-wrist chain forced that fold).
#   Targets: *_pick / *_drop → TCP z ≈ 0.80 m (table height); *_above → z ≈ 0.95 m
#   hover; home → a compact hover (0.20, 0, 1.00) over the work area, tool down.
#   All joint values are within the [-π, π] limits declared in arm.xacro.
#   NOTE: these are solved for the l_2=l_3=0.25, wrist=0.16 m chain in arm.xacro —
#   re-solve (see the IK in the arm.xacro link-length comment) if a length changes.
_Q: dict[str, tuple[float, ...]] = {
    "home": (0.000, 0.245, 1.620, 1.277, 0.000, 0.000),
    "bin_left_above": (2.246, 0.394, 1.636, 1.112, 0.000, 0.000),
    "bin_left_pick": (2.246, 0.792, 1.744, 0.605, 0.000, 0.000),
    "bin_right_above": (-2.246, 0.394, 1.636, 1.112, 0.000, 0.000),
    "bin_right_pick": (-2.246, 0.792, 1.744, 0.605, 0.000, 0.000),
    "tray_above": (0.000, 0.938, 0.745, 1.459, 0.000, 0.000),
    "tray_drop": (0.000, 1.191, 0.893, 1.058, 0.000, 0.000),
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
