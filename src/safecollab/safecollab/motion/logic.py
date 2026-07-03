"""Pure fusion logic for the motion node — no ROS, unit-tested directly.

Holds state (scale, nominal trajectory, joint positions) and computes the
re-timed trajectory command, including the §4/§6 stop/resume contract.
"""

from __future__ import annotations

from safecollab.motion.retime import retime

# Defaults mirror config/motion.yaml so MotionLogic is usable without a config
# file; the ROS node overrides them from YAML (ground rule 5).
_DEFAULT_REPUBLISH_EPSILON = 0.02
_DEFAULT_HOLD_TIME_S = 0.2


class MotionLogic:
    """Fusion logic: nominal trajectory × safety scale → re-timed command.

    Usage inside MotionNode callbacks: ``set_scale`` / ``set_nominal_trajectory`` /
    ``set_joint_positions`` feed state; ``command_for_scale`` / ``command_for_new_leg``
    return ``(kind, names, times, positions)`` or ``None``.
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
        # /joint_states is ordered ALPHABETICALLY while the planned trajectory uses
        # the ur_manipulator GROUP order; keeping the names lets us realign the
        # current state to the trajectory order before using it as a waypoint —
        # otherwise the resume/hold waypoint is scrambled (barely-moving motion).
        self._joint_names: list[str] | None = None
        self._stopped: bool = False
        # Stop/resume edge flags, refreshed by every set_scale.
        self._just_stopped: bool = False
        self._just_resumed: bool = False
        # Scale at which we last issued a command (None until the first); drives the
        # dead-band so unchanged 20 Hz ticks are dropped.
        self._last_cmd_scale: float | None = None
        self._republish_epsilon: float = republish_scale_epsilon
        self._hold_time_s: float = hold_time_s

    # ------------------------------------------------------------------
    # State updates (called from ROS callbacks or from tests directly)
    # ------------------------------------------------------------------

    def set_scale(self, scale: float) -> bool:
        """Update the speed scale.

        Returns True on a *resume* (scale==0 → scale>0); the caller then passes
        ``resuming=True`` to ``compute_command`` so the resume trajectory starts
        from the current joint state (§4/§6). Raises ``ValueError`` if out of range.
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
        """Store a new nominal (full-speed) trajectory, replacing any previous one."""
        self._nominal = (list(joint_names), list(times), list(positions))

    def set_joint_positions(
        self, positions: list[float], names: list[str] | None = None
    ) -> None:
        """Update current joint positions (from /joint_states).

        ``names`` is stored so the state can be realigned to the trajectory's joint
        order in :meth:`_current_in_nominal_order`; when omitted the positions are
        used as-is (the historical behaviour the unit tests rely on).
        """
        self._joint_positions = list(positions)
        self._joint_names = list(names) if names is not None else None

    def _current_in_nominal_order(self) -> list[float] | None:
        """Current joint positions permuted into the nominal trajectory's order.

        Any waypoint built from the current state must match the trajectory's joint
        order or the controller drives the wrong joints. Falls back to the raw list
        when names are unknown, already match, or a name is unmappable (never crashes).
        """
        if self._joint_positions is None:
            return None
        if self._joint_names is None or self._nominal is None:
            return self._joint_positions
        nominal_names = self._nominal[0]
        if self._joint_names == nominal_names:
            return self._joint_positions
        lut = dict(zip(self._joint_names, self._joint_positions))
        try:
            return [lut[name] for name in nominal_names]
        except KeyError:
            return self._joint_positions

    # ------------------------------------------------------------------
    # Command computation
    # ------------------------------------------------------------------

    def compute_command(
        self, *, resuming: bool = False
    ) -> tuple[list[str], list[float], list[list[float]]] | None:
        """Compute the re-timed trajectory ready for publication.

        When ``resuming``, prepend the current joint state as waypoint 0 (the §4/§6
        no-jerk contract). Returns ``None`` when no nominal is set or scale == 0.0.
        """
        if self._nominal is None:
            return None

        joint_names, times, positions = self._nominal
        current = self._current_in_nominal_order()

        if resuming and current is not None and positions:
            # Fresh trajectory from the current state: prepend the halted position at
            # t=0, then keep only the waypoints still AHEAD of it (see
            # _ahead_positions). Dropping passed waypoints is the "no backtrack" half
            # of the §6 resume contract — a stop part-way through a leg must drive on
            # to leg_end, not back to leg_start. Kept waypoints are spaced by one dt
            # so the controller has a non-zero time budget per segment.
            if len(times) >= 2:
                dt = times[1] - times[0]
            elif len(times) == 1:
                dt = times[0] if times[0] > 0.0 else 1.0
            else:
                dt = 1.0

            ahead = self._ahead_positions(current, positions)
            times_to_use = [0.0] + [(i + 1) * dt for i in range(len(ahead))]
            positions_to_use = [current] + ahead
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
        """Decide what to command for a new ``/safety/scale``.

        Applies the dead-band so the ~20 Hz stream of *unchanged* scales does not
        re-command the controller every tick (which resets the trajectory clock),
        and turns a protective stop into an ACTIVE hold. Returns ``(kind, names,
        times, positions)`` with *kind* ``"move"`` or ``"hold"``, or ``None`` when
        nothing new should be sent. Raises ``ValueError`` if scale is out of range.
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
        """Command a freshly-arrived nominal leg, re-timed at the current scale.

        While a protective stop is in effect it keeps holding instead of starting
        the new leg. Same return shape as :meth:`command_for_scale`.
        """
        if self._stopped:
            return self._hold_command()
        cmd = self.compute_command(resuming=True)
        if cmd is None:
            return None
        self._last_cmd_scale = self._scale
        return ("move", *cmd)

    def _is_material_change(self, scale: float) -> bool:
        """True when a scale warrants (re)issuing a command.

        Stop/resume edges always qualify; otherwise the change must clear the
        dead-band. The first command (no baseline) qualifies.
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

        Returns ``None`` if the current state or joint names are unknown — nothing
        to pin, so command nothing and let the controller hold.
        """
        current = self._current_in_nominal_order()
        if current is None or self._nominal is None:
            return None
        joint_names = self._nominal[0]
        self._last_cmd_scale = 0.0
        return (
            "hold",
            list(joint_names),
            [self._hold_time_s],
            [list(current)],
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
        """Nominal waypoints still ahead of *current* toward the goal.

        "Ahead" = strictly closer to the goal (last waypoint) than *current*, so a
        resume after a mid-leg stop drives on instead of backtracking. The goal is
        always kept, so the result is never empty ([current, goal] worst case).
        """
        goal = positions[-1]
        d_current = self._distance(current, goal)
        ahead = [p for p in positions if self._distance(p, goal) < d_current]
        if not ahead or ahead[-1] != goal:
            ahead.append(goal)
        return ahead
