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

from types import SimpleNamespace

import yaml

from safecollab.retime import retime

# Defaults mirror config/motion.yaml so MotionLogic is usable (and unit-testable)
# without a config file; the ROS node overrides them from the YAML (ground rule 5).
_DEFAULT_REPUBLISH_EPSILON = 0.02
_DEFAULT_HOLD_TIME_S = 0.2


def load_motion_config(path) -> SimpleNamespace:
    """Load the ``motion:`` block of ``config/motion.yaml`` into a namespace.

    Missing keys fall back to the module defaults so a partial file never crashes
    the node. Returns a namespace with ``republish_scale_epsilon`` and
    ``hold_time_s`` attributes.
    """
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    motion = data.get("motion") or {}
    return SimpleNamespace(
        republish_scale_epsilon=float(
            motion.get("republish_scale_epsilon", _DEFAULT_REPUBLISH_EPSILON)
        ),
        hold_time_s=float(motion.get("hold_time_s", _DEFAULT_HOLD_TIME_S)),
    )

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

    def __init__(
        self,
        *,
        republish_scale_epsilon: float = _DEFAULT_REPUBLISH_EPSILON,
        hold_time_s: float = _DEFAULT_HOLD_TIME_S,
    ) -> None:
        self._scale: float = 1.0
        # (joint_names, times, positions) — None until a trajectory arrives.
        self._nominal: tuple[list[str], list[float], list[list[float]]] | None = None
        self._joint_positions: list[float] | None = None
        self._stopped: bool = False
        # Stop/resume edge flags, refreshed by every set_scale (see command_for_scale).
        self._just_stopped: bool = False
        self._just_resumed: bool = False
        # The scale at which we last actually issued a command; None until the
        # first command. Drives the dead-band so unchanged 20 Hz ticks are dropped.
        self._last_cmd_scale: float | None = None
        self._republish_epsilon: float = republish_scale_epsilon
        self._hold_time_s: float = hold_time_s

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
        self._just_stopped = (not was_stopped) and self._stopped
        self._just_resumed = was_stopped and not self._stopped
        return self._just_resumed  # True only on resume

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
            # Prepend the halted position as waypoint 0 at t=0, then continue with
            # only the nominal waypoints that are still AHEAD of the current
            # position (see _ahead_positions).  Dropping already-passed waypoints
            # is the "no backtrack" half of the §6 clean-resume contract: a leg is
            # published as [leg_start, leg_end], so a protective stop part-way
            # through it must resume by driving on to leg_end — NOT back to
            # leg_start and then forward again (that backtrack is a jerk and, in a
            # shared workspace, motion the operator would not expect).
            #
            # The kept waypoints are evenly spaced by one inter-waypoint interval
            # (dt) so the controller has a non-zero time budget for each segment.
            # Example (nominal [w0, w1, w2], dt=1.0, robot stopped past w1):
            #   ahead = [w2]; resume_times = [0.0, 1.0]
            #   After retime(scale=0.5): [0.0, 2.0]
            if len(times) >= 2:
                dt = times[1] - times[0]
            elif len(times) == 1:
                dt = times[0] if times[0] > 0.0 else 1.0
            else:
                dt = 1.0

            ahead = self._ahead_positions(self._joint_positions, positions)
            times_to_use = [0.0] + [(i + 1) * dt for i in range(len(ahead))]
            positions_to_use = [self._joint_positions] + ahead
        else:
            times_to_use = times
            positions_to_use = positions

        new_times = retime(times_to_use, self._scale)
        if new_times is None:
            # scale == 0.0: protective stop — do not emit a motion command.
            return None

        return (joint_names, new_times, positions_to_use)

    # ------------------------------------------------------------------
    # Command orchestration (what the ROS callbacks actually call)
    # ------------------------------------------------------------------

    def command_for_scale(
        self, scale: float
    ) -> tuple[str, list[str], list[float], list[list[float]]] | None:
        """Decide what to command in response to a new ``/safety/scale``.

        This is the single entry the node's scale callback uses. It applies the
        dead-band (``config/motion.yaml``) so that the ~20 Hz stream of *unchanged*
        scale values does not re-command the controller on every tick (which resets
        the trajectory clock and produces the erratic motion seen in the demo), and
        it turns a protective stop into an ACTIVE hold at the current joint state.

        Returns:
            ``(kind, joint_names, times, positions)`` where *kind* is:
              * ``"move"`` — a re-timed trajectory to the leg goal, anchored at the
                current joint state (no backtrack), or
              * ``"hold"`` — a single-point trajectory pinning the current joint
                state (protective stop).
            or ``None`` when nothing new should be sent (scale materially unchanged,
            or a stop with no known joint state to hold).

        Raises:
            ValueError: if *scale* is outside [0.0, 1.0] (via :meth:`set_scale`).
        """
        self.set_scale(scale)
        if not self._is_material_change(scale):
            return None
        if self._stopped:
            return self._hold_command()
        cmd = self.compute_command(resuming=True)
        if cmd is None:
            return None
        self._last_cmd_scale = scale
        return ("move", *cmd)

    def command_for_new_leg(
        self,
    ) -> tuple[str, list[str], list[float], list[list[float]]] | None:
        """Decide what to command when a fresh nominal leg arrives.

        Issues the leg re-timed at the current scale, anchored at the current joint
        state. While a protective stop is in effect it keeps holding instead of
        starting the new leg (task_node advances legs open-loop, even during a stop).

        Returns the same ``(kind, joint_names, times, positions)`` shape as
        :meth:`command_for_scale`, or ``None`` when there is nothing to command.
        """
        if self._stopped:
            return self._hold_command()
        cmd = self.compute_command(resuming=True)
        if cmd is None:
            return None
        self._last_cmd_scale = self._scale
        return ("move", *cmd)

    def _is_material_change(self, scale: float) -> bool:
        """True when a scale value warrants (re)issuing a command.

        Stop and resume edges always qualify; otherwise the change must clear the
        configured dead-band. The very first command (no prior baseline) qualifies.
        """
        return (
            self._just_stopped
            or self._just_resumed
            or self._last_cmd_scale is None
            or abs(scale - self._last_cmd_scale) >= self._republish_epsilon
        )

    def _hold_command(
        self,
    ) -> tuple[str, list[str], list[float], list[list[float]]] | None:
        """A single-point HOLD at the current joint state (the active stop).

        Returns ``None`` if the current joint state or joint names are unknown —
        there is nothing meaningful to pin, so we command nothing and let the
        controller hold its last state.
        """
        if self._joint_positions is None or self._nominal is None:
            return None
        joint_names = self._nominal[0]
        self._last_cmd_scale = 0.0
        return (
            "hold",
            list(joint_names),
            [self._hold_time_s],
            [list(self._joint_positions)],
        )

    # ------------------------------------------------------------------
    # Resume helpers (no-backtrack re-planning)
    # ------------------------------------------------------------------

    @staticmethod
    def _distance(a: list[float], b: list[float]) -> float:
        """Euclidean distance between two joint-space configurations."""
        return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5

    def _ahead_positions(
        self, current: list[float], positions: list[list[float]]
    ) -> list[list[float]]:
        """Return the nominal waypoints still ahead of *current* toward the goal.

        A waypoint is "ahead" when it is strictly closer to the goal (the last
        nominal waypoint) than *current* is — i.e. the robot has not yet passed
        it.  Already-passed leading waypoints are dropped so a resume after a
        mid-trajectory stop continues toward the goal instead of backtracking to
        the start of the leg.  The goal is always kept, so the result is never
        empty (resume = [current, goal] in the worst case).
        """
        goal = positions[-1]
        d_current = self._distance(current, goal)
        ahead = [p for p in positions if self._distance(p, goal) < d_current]
        if not ahead or ahead[-1] != goal:
            ahead.append(goal)
        return ahead


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

        # Config knobs live next to the package (colcon install + editable layout
        # both put config/ next to safecollab/), same idiom as safety_monitor
        # (ground rule 5). _here is the package dir; config/ is beside it.
        from pathlib import Path

        _here = Path(__file__).resolve().parent
        motion_cfg = load_motion_config(_here.parent / "config" / "motion.yaml")
        self._logic = MotionLogic(
            republish_scale_epsilon=motion_cfg.republish_scale_epsilon,
            hold_time_s=motion_cfg.hold_time_s,
        )

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

        cmd = self._logic.command_for_new_leg()
        if cmd is not None:
            _kind, names, new_times, new_positions = cmd
            self._publish(names, new_times, new_positions, frame_id=msg.header.frame_id)

    def _on_scale(self, msg: "Float32") -> None:
        """React to a new safety scale: re-time, hold, or (usually) do nothing.

        The dead-band and active-hold decisions live in MotionLogic.command_for_scale;
        most 20 Hz ticks carry an unchanged scale and return None here, so the
        controller is not re-commanded every tick (which reset its trajectory clock).
        """
        cmd = self._logic.command_for_scale(float(msg.data))
        if cmd is not None:
            _kind, names, new_times, new_positions = cmd
            self._publish(names, new_times, new_positions)

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
