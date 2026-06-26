"""TDD spec for safecollab.motion_node -- Stream D fusion & control logic.

Tests cover what motion_node.py *adds on top* of retime() (which is already
tested by Stream B in test_retime.py and is not re-tested here):

  - MotionLogic state management: nominal trajectory, safety scale, joint state.
  - Fusing: nominal trajectory * scale => re-timed command.
  - Publishing gate: scale==0 => no motion command (retime returns None, no
    div-by-zero).
  - Stop/resume state machine: transitions, flag return value.
  - Resume re-plans from current joint state (not by splicing old trajectory),
    avoiding jerk on resume (AGENTS.md §4 + §6, Stream D contract).

ROS (rclpy, trajectory_msgs, etc.) is NOT installed in the pure-Python venv
(see requirements.txt and AGENTS.md ground rule 9).  MotionLogic is a pure-
Python class so all tests here run without a live ROS graph.
"""

import pytest

from safecollab.motion_node import MotionLogic


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _simple_logic(scale: float = 1.0) -> MotionLogic:
    """Return a MotionLogic with a two-waypoint nominal trajectory and given scale."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(
        joint_names=["j1", "j2"],
        times=[0.0, 1.0],
        positions=[[0.0, 0.0], [1.0, 1.0]],
    )
    logic.set_scale(scale)
    return logic


# ---------------------------------------------------------------------------
# No trajectory
# ---------------------------------------------------------------------------


def test_no_trajectory_returns_none():
    """No nominal trajectory => compute_command returns None."""
    logic = MotionLogic()
    assert logic.compute_command() is None


def test_no_trajectory_with_scale_still_returns_none():
    """Scale set but no trajectory => still None."""
    logic = MotionLogic()
    logic.set_scale(0.8)
    assert logic.compute_command() is None


# ---------------------------------------------------------------------------
# Normal (non-stopped) operation: fusing & re-timing
# ---------------------------------------------------------------------------


def test_full_speed_times_unchanged():
    """scale=1.0 leaves time_from_start values identical to nominal."""
    logic = _simple_logic(scale=1.0)
    _, times, _ = logic.compute_command()
    assert times == pytest.approx([0.0, 1.0])


def test_half_speed_doubles_times():
    """scale=0.5 stretches each time_from_start by 2x."""
    logic = _simple_logic(scale=0.5)
    _, times, _ = logic.compute_command()
    assert times == pytest.approx([0.0, 2.0])


def test_quarter_speed_quadruples_times():
    """scale=0.25 stretches each time_from_start by 4x."""
    logic = _simple_logic(scale=0.25)
    _, times, _ = logic.compute_command()
    assert times == pytest.approx([0.0, 4.0])


def test_joint_names_preserved():
    """Joint names from the nominal trajectory pass through unchanged."""
    logic = _simple_logic()
    names, _, _ = logic.compute_command()
    assert names == ["j1", "j2"]


def test_positions_preserved_in_normal_command():
    """Waypoint positions from the nominal trajectory pass through unchanged."""
    logic = _simple_logic()
    _, _, positions = logic.compute_command()
    assert positions == [[0.0, 0.0], [1.0, 1.0]]


def test_three_waypoint_trajectory_retimed():
    """All three waypoints get re-timed consistently."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0, 3.0], [[0.0], [1.0], [3.0]])
    logic.set_scale(0.5)
    _, times, _ = logic.compute_command()
    assert times == pytest.approx([0.0, 2.0, 6.0])


def test_new_trajectory_replaces_old():
    """Setting a second nominal trajectory supersedes the first."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_nominal_trajectory(["j1"], [0.0, 2.0], [[0.0], [2.0]])
    logic.set_scale(1.0)
    _, times, positions = logic.compute_command()
    assert times == pytest.approx([0.0, 2.0])
    assert positions == [[0.0], [2.0]]


# ---------------------------------------------------------------------------
# Protective stop: scale == 0.0
# ---------------------------------------------------------------------------


def test_protective_stop_returns_none():
    """scale=0.0 => protective stop => compute_command returns None (no div-by-zero)."""
    logic = _simple_logic(scale=0.0)
    assert logic.compute_command() is None


def test_protective_stop_returns_none_regardless_of_trajectory():
    """Even with a full trajectory, scale=0 must suppress the command."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(
        ["j1", "j2", "j3"],
        [0.0, 0.5, 1.0, 1.5],
        [[0.0, 0.0, 0.0], [0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [0.3, 0.3, 0.3]],
    )
    logic.set_scale(0.0)
    assert logic.compute_command() is None


# ---------------------------------------------------------------------------
# set_scale return value (stop / resume flag)
# ---------------------------------------------------------------------------


def test_set_scale_returns_false_for_normal_update():
    """Normal scale change (non-zero to non-zero) returns False."""
    logic = MotionLogic()
    logic.set_scale(1.0)
    result = logic.set_scale(0.5)
    assert result is False


def test_set_scale_returns_false_on_first_stop():
    """Transition from running to stopped returns False (it is not a resume)."""
    logic = MotionLogic()
    logic.set_scale(1.0)
    result = logic.set_scale(0.0)
    assert result is False


def test_set_scale_returns_true_on_resume():
    """Transition from stopped (0.0) to running returns True (resume event)."""
    logic = MotionLogic()
    logic.set_scale(0.0)  # stop
    result = logic.set_scale(1.0)  # resume
    assert result is True


def test_set_scale_returns_true_on_partial_resume():
    """Resume to any non-zero scale returns True."""
    logic = MotionLogic()
    logic.set_scale(0.0)
    result = logic.set_scale(0.3)
    assert result is True


def test_set_scale_returns_false_for_stop_to_stop():
    """Two consecutive stops do not count as resume."""
    logic = MotionLogic()
    logic.set_scale(0.0)
    result = logic.set_scale(0.0)
    assert result is False


def test_set_scale_raises_for_negative():
    """Scale < 0 raises ValueError."""
    logic = MotionLogic()
    with pytest.raises(ValueError):
        logic.set_scale(-0.1)


def test_set_scale_raises_for_above_one():
    """Scale > 1 raises ValueError."""
    logic = MotionLogic()
    with pytest.raises(ValueError):
        logic.set_scale(1.01)


# ---------------------------------------------------------------------------
# Resume: re-plan from current joint state (AGENTS.md §4, §6)
# ---------------------------------------------------------------------------


def test_resume_first_position_is_current_joint_state():
    """On resume, positions[0] must be the current joint state, not nominal[0].

    This is the core §6 contract: resume re-plans from the current state,
    NOT by splicing back into the old trajectory, to avoid jerk.
    """
    logic = MotionLogic()
    logic.set_nominal_trajectory(
        ["j1", "j2"],
        [0.0, 1.0, 2.0],
        [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]],
    )
    logic.set_scale(0.0)  # protective stop
    logic.set_joint_positions([0.5, 0.7])  # robot halted here

    is_resume = logic.set_scale(1.0)  # resume
    assert is_resume is True, "set_scale must signal resume"

    cmd = logic.compute_command(resuming=True)
    assert cmd is not None
    _, _, positions = cmd
    assert positions[0] == pytest.approx(
        [0.5, 0.7]
    ), "first waypoint must be the halted position, not nominal[0]"


def test_resume_first_time_is_zero():
    """On resume, times[0] must be 0.0 (the new trajectory starts now)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0, 2.0], [[0.0], [1.0], [2.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5])

    logic.set_scale(1.0)
    _, times, _ = logic.compute_command(resuming=True)
    assert times[0] == pytest.approx(0.0)


def test_resume_includes_all_nominal_waypoints_after_current_state():
    """After the current-state waypoint, all nominal waypoints must follow."""
    logic = MotionLogic()
    nominal_positions = [[0.0], [1.0], [2.0]]
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0, 2.0], nominal_positions)
    logic.set_scale(0.0)
    logic.set_joint_positions([0.4])

    logic.set_scale(1.0)
    _, _, positions = logic.compute_command(resuming=True)

    # positions[0] = current state; positions[1:] = all nominal waypoints
    assert positions[1:] == nominal_positions


def test_resume_times_are_monotonically_increasing():
    """Re-planned times must be strictly increasing (no time travel)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(
        ["j1", "j2"],
        [0.0, 1.0, 2.0, 3.0],
        [[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0]],
    )
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5, 0.5])

    logic.set_scale(1.0)
    _, times, _ = logic.compute_command(resuming=True)

    for a, b in zip(times, times[1:]):
        assert b > a, f"expected strictly increasing times, got {times}"


def test_resume_retimes_with_half_speed():
    """Resume + half speed: times stretched 2x."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5])

    logic.set_scale(0.5)
    _, times, _ = logic.compute_command(resuming=True)

    # Resume times (scale=1): [0.0, dt, 1.0+dt] where dt = 1.0 -> [0.0, 1.0, 2.0]
    # After retime(scale=0.5): [0.0, 2.0, 4.0]
    assert times == pytest.approx([0.0, 2.0, 4.0])


def test_resume_without_joint_positions_falls_back_to_nominal():
    """If no joint state is known, compute_command(resuming=True) falls back
    to the normal (nominal) trajectory rather than erroring out."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_scale(0.0)
    # deliberate: do NOT call set_joint_positions

    logic.set_scale(1.0)
    cmd = logic.compute_command(resuming=True)
    assert cmd is not None
    _, _, positions = cmd
    assert positions == [[0.0], [1.0]]  # nominal positions unchanged


# ---------------------------------------------------------------------------
# Scenario tests: simulate the callback sequence the MotionNode would drive
# ---------------------------------------------------------------------------


def test_scenario_trajectory_then_stop_then_resume():
    """Full stop/resume cycle as the safety monitor and task node would drive it.

    This exercises the same logic the MotionNode callbacks trigger — without
    needing a live ROS graph — by passing stub data directly to MotionLogic.
    """
    logic = MotionLogic()

    # Step 1: task_node publishes nominal trajectory (/motion/nominal_trajectory)
    logic.set_nominal_trajectory(
        ["j1", "j2"],
        [0.0, 1.0, 2.0],
        [[0.0, 0.0], [1.0, 0.5], [2.0, 1.0]],
    )

    # Step 2: safety_monitor publishes green scale (/safety/scale = 1.0)
    is_resume = logic.set_scale(1.0)
    assert is_resume is False
    cmd = logic.compute_command()
    assert cmd is not None, "running at full speed: command must be non-None"

    # Step 3: operator reaches into tray -> scale drops to 0 (protective stop)
    logic.set_joint_positions([0.4, 0.2])  # joint_states update
    is_resume = logic.set_scale(0.0)
    assert is_resume is False
    cmd = logic.compute_command()
    assert cmd is None, "protective stop: no motion command"

    # Step 4: operator retreats -> scale rises again (resume)
    is_resume = logic.set_scale(1.0)
    assert is_resume is True, "must signal resume"
    cmd = logic.compute_command(resuming=True)
    assert cmd is not None, "resumed: command must be non-None"
    _, _, positions = cmd
    assert positions[0] == pytest.approx(
        [0.4, 0.2]
    ), "resume must start from halted position"


def test_scenario_scale_update_without_trajectory_is_safe():
    """Safety scale arriving before any trajectory must not crash."""
    logic = MotionLogic()
    logic.set_scale(0.5)
    logic.set_scale(0.0)
    logic.set_scale(1.0)
    assert logic.compute_command() is None  # still no trajectory


def test_scenario_joint_state_update_does_not_affect_normal_command():
    """Updating joint state while running should NOT change the published positions
    (we only use joint state for the resume path)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_scale(1.0)
    logic.set_joint_positions([0.99])  # some live joint state

    _, _, positions = logic.compute_command(resuming=False)
    assert positions == [[0.0], [1.0]], "normal path must use nominal positions"


# ---------------------------------------------------------------------------
# Edge cases: single-waypoint and empty-times trajectories on resume
# (covers the elif len==1 and else dt=1.0 branches inside compute_command)
# ---------------------------------------------------------------------------


def test_resume_single_waypoint_trajectory():
    """Resume with a one-point trajectory: first output position is current state."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [2.0], [[1.0]])  # single waypoint at t=2.0
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5])

    logic.set_scale(1.0)
    cmd = logic.compute_command(resuming=True)

    assert cmd is not None
    _, times, positions = cmd
    assert times[0] == pytest.approx(0.0), "resume trajectory must start at t=0"
    assert positions[0] == pytest.approx([0.5]), "first waypoint must be current state"
    assert len(times) == 2  # [0.0, something]


def test_resume_empty_times_trajectory():
    """Resume with a trajectory that has positions but no times: must not crash.

    This is a degenerate (malformed) input; the node should handle it gracefully.
    Covers the ``else: dt = 1.0`` branch.
    """
    logic = MotionLogic()
    # Malformed: positions present, times empty.
    logic.set_nominal_trajectory(["j1"], [], [[1.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5])

    logic.set_scale(1.0)
    cmd = logic.compute_command(resuming=True)

    # Must not raise; at minimum the first time is 0.0 and the first
    # position is the current joint state.
    assert cmd is not None
    _, times, positions = cmd
    assert times[0] == pytest.approx(0.0)
    assert positions[0] == pytest.approx([0.5])
