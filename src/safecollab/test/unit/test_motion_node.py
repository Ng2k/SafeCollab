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

from safecollab.motion_node import MotionLogic, load_motion_config


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


def test_current_state_realigned_to_trajectory_joint_order():
    """Current joint state is permuted into the trajectory's joint order.

    /joint_states is published ALPHABETICALLY (elbow, shoulder_lift,
    shoulder_pan, …) while the planned trajectory uses ur_manipulator GROUP
    order (shoulder_pan, shoulder_lift, elbow, …). If the current state were
    spliced in raw, shoulder_pan and elbow would be swapped in the resume/hold
    waypoint — the scramble that made the arm jerk and barely move. The state
    must be realigned to the trajectory's joint names before use.
    """
    logic = MotionLogic()
    logic.set_nominal_trajectory(
        ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"],
        [0.0, 1.0],
        [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
    )
    logic.set_scale(0.0)  # protective stop
    # Halted state arrives in /joint_states (alphabetical) order with names.
    logic.set_joint_positions(
        [0.3, 0.2, 0.1],  # elbow=0.3, shoulder_lift=0.2, shoulder_pan=0.1
        ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint"],
    )

    logic.set_scale(1.0)  # resume
    cmd = logic.compute_command(resuming=True)
    assert cmd is not None
    _, _, positions = cmd
    # Waypoint 0 must be in trajectory order: [shoulder_pan, shoulder_lift, elbow].
    assert positions[0] == pytest.approx([0.1, 0.2, 0.3])


def test_hold_command_realigns_current_state_to_trajectory_order():
    """The protective-hold waypoint is also expressed in trajectory joint order."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(
        ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"],
        [0.0, 1.0],
        [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
    )
    logic.set_joint_positions(
        [0.3, 0.2, 0.1],
        ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint"],
    )
    cmd = logic.command_for_scale(0.0)  # protective stop => hold
    assert cmd is not None
    kind, _, _, positions = cmd
    assert kind == "hold"
    assert positions[0] == pytest.approx([0.1, 0.2, 0.3])


def test_current_state_without_names_used_as_is():
    """When names are omitted the state is used verbatim (historical behaviour)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1", "j2"], [0.0, 1.0], [[0.0, 0.0], [1.0, 1.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5, 0.7])  # no names
    logic.set_scale(1.0)
    cmd = logic.compute_command(resuming=True)
    assert cmd is not None
    _, _, positions = cmd
    assert positions[0] == pytest.approx([0.5, 0.7])


def test_resume_first_time_is_zero():
    """On resume, times[0] must be 0.0 (the new trajectory starts now)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0, 2.0], [[0.0], [1.0], [2.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5])

    logic.set_scale(1.0)
    _, times, _ = logic.compute_command(resuming=True)
    assert times[0] == pytest.approx(0.0)


def test_resume_keeps_only_ahead_nominal_waypoints():
    """After the current-state waypoint, only nominal waypoints still AHEAD of
    the robot follow — already-passed leading waypoints are dropped so the resume
    does not backtrack (AT-4 hardening)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0, 2.0], [[0.0], [1.0], [2.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.4])  # stopped past the start (w0)

    logic.set_scale(1.0)
    _, _, positions = logic.compute_command(resuming=True)

    # positions[0] = current state; the passed w0=[0.0] is dropped; w1, w2 follow.
    assert positions[0] == pytest.approx([0.4])
    assert positions[1:] == [[1.0], [2.0]]


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
    """Resume + half speed: kept times stretched 2x.

    current=[0.5] is past w0=[0.0] on a [w0, w1] leg, so resume drops w0 and
    keeps [current, w1]; times [0.0, dt] (dt=1.0) -> retime(0.5) -> [0.0, 2.0].
    """
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.5])

    logic.set_scale(0.5)
    _, times, _ = logic.compute_command(resuming=True)

    assert times == pytest.approx([0.0, 2.0])


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
# AT-4 hardening (P4): a mid-trajectory protective stop resumes without backtrack
# ---------------------------------------------------------------------------


def test_resume_mid_leg_does_not_backtrack_to_leg_start():
    """A stop part-way through a [start, end] leg resumes straight on to end.

    This exercises the resume path with a minimal 2-waypoint leg [leg_start,
    leg_end]. If the robot is halted between them, resume must NOT re-insert leg_start —
    that would drive the arm backward then forward again (a jerk, and motion the
    operator would not expect in a shared workspace). The resumed trajectory is
    [current, leg_end] only.
    """
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1", "j2"], [0.0, 1.0], [[0.0, 0.0], [2.0, 2.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([1.2, 1.2])  # halted mid-leg, past the start

    logic.set_scale(1.0)
    _, _, positions = logic.compute_command(resuming=True)

    assert positions[0] == pytest.approx([1.2, 1.2]), "resume starts at current state"
    assert [0.0, 0.0] not in positions, "leg_start must not reappear (no backtrack)"
    assert positions == [[1.2, 1.2], [2.0, 2.0]]


def test_resume_monotonic_progress_toward_goal():
    """Distance to the goal is non-increasing along the resumed trajectory.

    Generalises 'no backtrack' to a multi-waypoint leg: waypoints behind the
    current position are dropped, so progress toward the goal never reverses.
    """
    logic = MotionLogic()
    logic.set_nominal_trajectory(
        ["j1"], [0.0, 1.0, 2.0, 3.0], [[0.0], [1.0], [2.0], [3.0]]
    )
    logic.set_scale(0.0)
    logic.set_joint_positions([1.5])  # between w1 and w2

    logic.set_scale(1.0)
    _, _, positions = logic.compute_command(resuming=True)

    goal = positions[-1]
    dists = [abs(p[0] - goal[0]) for p in positions]
    for a, b in zip(dists, dists[1:]):
        assert b <= a, f"distance to goal must not increase (backtrack): {dists}"
    # w0, w1 are behind current -> dropped; current + w2 + w3 remain.
    assert positions == [[1.5], [2.0], [3.0]]


def test_resume_when_already_at_goal_keeps_goal():
    """If the robot is halted at the leg goal, resume still yields a valid
    [current, goal] trajectory — the goal is always retained, never empty."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [2.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([2.0])  # already at the goal

    logic.set_scale(1.0)
    _, _, positions = logic.compute_command(resuming=True)
    assert positions == [[2.0], [2.0]]


def test_resume_barely_moved_keeps_ahead_waypoints():
    """If the robot only just left the start, the ahead waypoints are retained."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0, 2.0], [[0.0], [1.0], [2.0]])
    logic.set_scale(0.0)
    logic.set_joint_positions([0.05])  # only just past the start

    logic.set_scale(1.0)
    _, _, positions = logic.compute_command(resuming=True)
    # The passed start w0 is dropped; both ahead waypoints w1, w2 are kept.
    assert positions == [[0.05], [1.0], [2.0]]


# ---------------------------------------------------------------------------
# Scenario tests: simulate the callback sequence the MotionNode would drive
# ---------------------------------------------------------------------------


def test_scenario_trajectory_then_stop_then_resume():
    """Full stop/resume cycle as the safety monitor and task node would drive it.

    This exercises the same logic the MotionNode callbacks trigger — without
    needing a live ROS graph — by passing stub data directly to MotionLogic.
    """
    logic = MotionLogic()

    # Step 1: planner_node publishes nominal trajectory (/motion/nominal_trajectory)
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


# ---------------------------------------------------------------------------
# Dead-band + active-hold orchestration (P5 demo fix)
#
# safety_monitor publishes /safety/scale at a fixed 20 Hz even when the value is
# unchanged. command_for_scale() is the single entry the ROS scale callback uses:
# it dedups unchanged ticks (so the joint_trajectory_controller is not re-commanded
# ~20x/s, which reset its clock and produced the erratic "random swinging"), and it
# turns a protective stop (scale==0) into an ACTIVE hold at the current joint state
# (joint_trajectory_controller does not stop on silence — it finishes its last goal).
# ---------------------------------------------------------------------------


def _running_logic() -> MotionLogic:
    """A MotionLogic with a 2-waypoint leg, a known joint state, running (scale 1)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1", "j2"], [0.0, 1.0], [[0.0, 0.0], [1.0, 1.0]])
    logic.set_joint_positions([0.3, 0.3])
    return logic


def test_command_for_scale_first_call_issues_move():
    """The first scale after start-up issues a move command."""
    logic = _running_logic()
    cmd = logic.command_for_scale(1.0)
    assert cmd is not None
    assert cmd[0] == "move"


def test_command_for_scale_dedups_identical_scale():
    """An unchanged scale tick issues nothing (no 20 Hz re-command spam)."""
    logic = _running_logic()
    assert logic.command_for_scale(1.0) is not None
    assert logic.command_for_scale(1.0) is None


def test_command_for_scale_dedups_subthreshold_change():
    """A scale change below the dead-band is ignored."""
    logic = _running_logic()
    logic.command_for_scale(0.5)
    assert logic.command_for_scale(0.51) is None  # 0.01 < epsilon (0.02)


def test_command_for_scale_reissues_on_material_change():
    """A scale change above the dead-band re-issues a move command."""
    logic = _running_logic()
    logic.command_for_scale(0.5)
    cmd = logic.command_for_scale(0.8)
    assert cmd is not None and cmd[0] == "move"


def test_command_for_scale_stop_issues_hold_at_current_state():
    """scale==0 issues a single-point HOLD at the current joint state."""
    logic = _running_logic()
    logic.command_for_scale(1.0)
    cmd = logic.command_for_scale(0.0)
    assert cmd is not None
    kind, names, times, positions = cmd
    assert kind == "hold"
    assert names == ["j1", "j2"]
    assert positions == [[0.3, 0.3]]  # holds where the arm is, not the leg end
    assert len(positions) == 1 and times[0] > 0.0  # a short, positive hold horizon


def test_command_for_scale_stop_dedups_after_first_hold():
    """Repeated scale==0 ticks do not re-issue the hold every tick."""
    logic = _running_logic()
    logic.command_for_scale(1.0)
    assert logic.command_for_scale(0.0) is not None  # first stop -> hold
    assert logic.command_for_scale(0.0) is None  # already holding


def test_command_for_scale_stop_without_joint_state_sends_nothing():
    """Cannot hold an unknown joint state -> command nothing (no crash)."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.command_for_scale(1.0)  # no set_joint_positions
    assert logic.command_for_scale(0.0) is None


def test_command_for_scale_resume_moves_from_current_state():
    """Resume (0 -> >0) issues a move that starts at the current joint state."""
    logic = _running_logic()
    logic.command_for_scale(1.0)
    logic.command_for_scale(0.0)
    logic.set_joint_positions([0.6, 0.6])  # halted here
    cmd = logic.command_for_scale(1.0)  # resume
    assert cmd is not None
    kind, _, _, positions = cmd
    assert kind == "move"
    assert positions[0] == pytest.approx([0.6, 0.6])


def test_command_for_scale_midleg_change_does_not_backtrack():
    """A mid-leg slow-down re-plans from the current state, never back to leg start."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [2.0]])
    logic.command_for_scale(1.0)
    logic.set_joint_positions([1.2])  # moved well into the leg
    cmd = logic.command_for_scale(0.5)  # material slow-down mid-leg
    assert cmd is not None
    _, _, _, positions = cmd
    assert positions[0] == pytest.approx([1.2])  # anchored at current state
    assert [0.0] not in positions  # leg start is not re-commanded


def test_command_for_new_leg_issues_move_when_running():
    """A fresh nominal leg while running issues a move command."""
    logic = MotionLogic()
    logic.set_scale(1.0)
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_joint_positions([0.0])
    cmd = logic.command_for_new_leg()
    assert cmd is not None and cmd[0] == "move"


def test_command_for_new_leg_holds_when_stopped():
    """A fresh nominal leg while STOPPED keeps holding — it does not start moving."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_joint_positions([0.4])
    logic.command_for_scale(1.0)
    logic.command_for_scale(0.0)  # protective stop in effect
    cmd = logic.command_for_new_leg()  # planner_node advances a leg during the stop
    assert cmd is not None and cmd[0] == "hold"
    assert cmd[3] == [[0.4]]  # keep holding current state, not the new leg


def test_command_for_new_leg_sets_dedup_baseline():
    """After a new leg is issued at scale 1.0, an identical scale tick dedups."""
    logic = MotionLogic()
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_joint_positions([0.0])
    logic.set_scale(1.0)
    logic.command_for_new_leg()
    assert logic.command_for_scale(1.0) is None


def test_command_for_new_leg_without_trajectory_returns_none():
    """No nominal trajectory yet -> nothing to command."""
    logic = MotionLogic()
    logic.set_scale(1.0)
    assert logic.command_for_new_leg() is None


def test_command_for_scale_without_trajectory_returns_none():
    """A running scale with no nominal leg yet commands nothing (no crash)."""
    logic = MotionLogic()
    logic.set_joint_positions([0.0])
    assert logic.command_for_scale(1.0) is None


# ---------------------------------------------------------------------------
# Config loader (ground rule 5: knobs live in config/motion.yaml)
# ---------------------------------------------------------------------------


def test_load_motion_config_reads_knobs(tmp_path):
    """load_motion_config reads the republish dead-band and hold horizon."""
    p = tmp_path / "motion.yaml"
    p.write_text("motion:\n  republish_scale_epsilon: 0.05\n  hold_time_s: 0.5\n")
    cfg = load_motion_config(p)
    assert cfg.republish_scale_epsilon == pytest.approx(0.05)
    assert cfg.hold_time_s == pytest.approx(0.5)


def test_load_motion_config_defaults_when_missing(tmp_path):
    """Missing keys fall back to sane defaults (no crash on a partial file)."""
    p = tmp_path / "motion.yaml"
    p.write_text("motion: {}\n")
    cfg = load_motion_config(p)
    assert cfg.republish_scale_epsilon > 0.0
    assert cfg.hold_time_s > 0.0


def test_motion_logic_honours_configured_epsilon():
    """A larger configured dead-band widens what counts as 'unchanged'."""
    logic = MotionLogic(republish_scale_epsilon=0.2, hold_time_s=0.2)
    logic.set_nominal_trajectory(["j1"], [0.0, 1.0], [[0.0], [1.0]])
    logic.set_joint_positions([0.0])
    logic.command_for_scale(0.5)
    assert logic.command_for_scale(0.6) is None  # 0.1 < 0.2 -> deduped
    assert logic.command_for_scale(0.75) is not None  # 0.25 >= 0.2 -> re-issued
