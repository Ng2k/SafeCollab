"""TDD spec for the kitting state machine (Stream E, task_node.py).

All tests exercise the pure-Python ``KittingStateMachine`` and related
types.  No ROS graph is required — these are fully isolated unit tests.

State-machine contract under test
----------------------------------
* GO_TO_BIN → PICK → GO_TO_TRAY → DROP → GO_TO_BIN → … (linear cycle)
* Bin alternates left / right across cycles (configurable sequence).
* ``advance()`` returns the *new* state.
* ``current_leg()`` returns a 2-waypoint ``TrajectoryLeg`` with 6-joint
  positions within the joint limits declared in arm.xacro ([-π, π]).
* ``leg_duration_s()`` is positive for every state.
* Different states must produce different joint configurations so that the
  arm actually moves.
"""

import math

import pytest

from safecollab.task_node import (
    JOINT_NAMES,
    KittingState,
    KittingStateMachine,
    TrajectoryLeg,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_JOINT_LIMIT = math.pi  # declared in arm.xacro: lower="${-pi}"  upper="${pi}"
_N_JOINTS = 6


def _fresh_sm(**kwargs) -> KittingStateMachine:
    """Return a fresh state machine with optional overrides."""
    return KittingStateMachine(**kwargs)


# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------


def test_initial_state_is_go_to_bin():
    sm = _fresh_sm()
    assert sm.state is KittingState.GO_TO_BIN


def test_initial_cycle_count_is_zero():
    sm = _fresh_sm()
    assert sm.cycle_count == 0


def test_initial_bin_is_left_in_default_sequence():
    sm = _fresh_sm()
    assert sm.current_bin() == "left"


# ---------------------------------------------------------------------------
# State transitions — single-step
# ---------------------------------------------------------------------------


def test_advance_from_go_to_bin_yields_pick():
    sm = _fresh_sm()
    new_state = sm.advance()
    assert new_state is KittingState.PICK
    assert sm.state is KittingState.PICK


def test_advance_from_pick_yields_go_to_tray():
    sm = _fresh_sm()
    sm.advance()  # → PICK
    new_state = sm.advance()
    assert new_state is KittingState.GO_TO_TRAY
    assert sm.state is KittingState.GO_TO_TRAY


def test_advance_from_go_to_tray_yields_drop():
    sm = _fresh_sm()
    sm.advance()  # → PICK
    sm.advance()  # → GO_TO_TRAY
    new_state = sm.advance()
    assert new_state is KittingState.DROP
    assert sm.state is KittingState.DROP


def test_advance_from_drop_wraps_to_go_to_bin():
    sm = _fresh_sm()
    for _ in range(3):
        sm.advance()  # GO_TO_BIN → PICK → GO_TO_TRAY → DROP
    new_state = sm.advance()
    assert new_state is KittingState.GO_TO_BIN
    assert sm.state is KittingState.GO_TO_BIN


# ---------------------------------------------------------------------------
# Full-cycle transitions
# ---------------------------------------------------------------------------


def test_full_cycle_restores_go_to_bin():
    sm = _fresh_sm()
    states = [sm.state]
    for _ in range(4):
        states.append(sm.advance())
    assert states == [
        KittingState.GO_TO_BIN,
        KittingState.PICK,
        KittingState.GO_TO_TRAY,
        KittingState.DROP,
        KittingState.GO_TO_BIN,
    ]


def test_three_full_cycles_complete_correctly():
    sm = _fresh_sm()
    for _ in range(3 * 4):
        sm.advance()
    assert sm.state is KittingState.GO_TO_BIN
    assert sm.cycle_count == 3


# ---------------------------------------------------------------------------
# Cycle count
# ---------------------------------------------------------------------------


def test_cycle_count_increments_only_on_wrap_to_go_to_bin():
    sm = _fresh_sm()
    assert sm.cycle_count == 0
    sm.advance()  # → PICK
    assert sm.cycle_count == 0
    sm.advance()  # → GO_TO_TRAY
    assert sm.cycle_count == 0
    sm.advance()  # → DROP
    assert sm.cycle_count == 0
    sm.advance()  # → GO_TO_BIN  ← cycle 1 complete
    assert sm.cycle_count == 1


def test_cycle_count_increments_each_full_cycle():
    sm = _fresh_sm()
    for c in range(1, 6):
        for _ in range(4):
            sm.advance()
        assert sm.cycle_count == c


# ---------------------------------------------------------------------------
# Bin alternation
# ---------------------------------------------------------------------------


def test_bin_is_left_during_first_cycle():
    sm = _fresh_sm()
    # All states in the first cycle should see "left"
    for _ in range(4):
        assert sm.current_bin() == "left"
        sm.advance()


def test_bin_alternates_to_right_on_second_cycle():
    sm = _fresh_sm()
    for _ in range(4):
        sm.advance()  # complete cycle 1
    assert sm.current_bin() == "right"


def test_bin_alternates_back_to_left_on_third_cycle():
    sm = _fresh_sm()
    for _ in range(8):
        sm.advance()  # complete 2 cycles
    assert sm.current_bin() == "left"


def test_custom_bin_sequence_right_then_left():
    sm = _fresh_sm(bin_sequence=("right", "left"))
    assert sm.current_bin() == "right"
    for _ in range(4):
        sm.advance()
    assert sm.current_bin() == "left"


def test_single_bin_sequence_never_alternates():
    sm = _fresh_sm(bin_sequence=("left",))
    for _ in range(12):
        sm.advance()
    assert sm.current_bin() == "left"


# ---------------------------------------------------------------------------
# Constructor validation
# ---------------------------------------------------------------------------


def test_empty_bin_sequence_raises():
    with pytest.raises(ValueError, match="bin_sequence must not be empty"):
        KittingStateMachine(bin_sequence=())


def test_invalid_bin_label_raises():
    with pytest.raises(ValueError, match="Invalid bin label"):
        KittingStateMachine(bin_sequence=("centre",))


# ---------------------------------------------------------------------------
# leg_duration_s
# ---------------------------------------------------------------------------


def test_leg_duration_positive_for_all_states():
    for initial_state in KittingState:
        sm = KittingStateMachine(initial_state=initial_state)
        assert sm.leg_duration_s() > 0.0


@pytest.mark.parametrize(
    "state,expected_s",
    [
        (KittingState.GO_TO_BIN, 3.0),
        (KittingState.PICK, 1.5),
        (KittingState.GO_TO_TRAY, 3.0),
        (KittingState.DROP, 1.5),
    ],
)
def test_leg_duration_matches_expected(state, expected_s):
    sm = KittingStateMachine(initial_state=state)
    assert sm.leg_duration_s() == pytest.approx(expected_s)


# ---------------------------------------------------------------------------
# current_leg / TrajectoryLeg
# ---------------------------------------------------------------------------


def test_current_leg_returns_trajectory_leg_instance():
    sm = _fresh_sm()
    leg = sm.current_leg()
    assert isinstance(leg, TrajectoryLeg)


def test_trajectory_leg_has_correct_joint_names():
    sm = _fresh_sm()
    leg = sm.current_leg()
    assert leg.joint_names == JOINT_NAMES


def test_trajectory_leg_has_two_waypoints():
    sm = _fresh_sm()
    for _ in range(4):
        leg = sm.current_leg()
        assert (
            len(leg.waypoints) == 2
        ), f"Expected 2 waypoints in state {sm.state.value}"
        sm.advance()


def test_each_waypoint_has_six_joint_positions():
    sm = _fresh_sm()
    for _ in range(4):
        leg = sm.current_leg()
        for wp in leg.waypoints:
            assert (
                len(wp) == _N_JOINTS
            ), f"Expected {_N_JOINTS} joint positions in state {sm.state.value}"
        sm.advance()


def test_joint_positions_within_limits():
    """All joint positions must respect the [-π, π] limits from arm.xacro."""
    sm = _fresh_sm()
    for _ in range(4):
        leg = sm.current_leg()
        for wp in leg.waypoints:
            for j, angle in enumerate(wp):
                assert -_JOINT_LIMIT <= angle <= _JOINT_LIMIT, (
                    f"Joint {j} angle {angle:.3f} rad out of limits in "
                    f"state {sm.state.value}"
                )
        sm.advance()


def test_different_states_produce_different_configurations():
    """GO_TO_BIN and DROP produce different approach/terminal configurations."""
    sm_a = KittingStateMachine(initial_state=KittingState.GO_TO_BIN)
    sm_d = KittingStateMachine(initial_state=KittingState.DROP)
    leg_a = sm_a.current_leg()
    leg_d = sm_d.current_leg()
    # Terminal waypoints (index 1) must differ
    assert leg_a.waypoints[1] != leg_d.waypoints[1]


def test_bin_left_and_bin_right_produce_different_configurations():
    """Picking from the left and right bins uses different joint angles."""
    sm_left = KittingStateMachine(
        initial_state=KittingState.PICK, bin_sequence=("left",)
    )
    sm_right = KittingStateMachine(
        initial_state=KittingState.PICK, bin_sequence=("right",)
    )
    leg_left = sm_left.current_leg()
    leg_right = sm_right.current_leg()
    # The terminal waypoints must differ for left vs right bins
    assert leg_left.waypoints[1] != leg_right.waypoints[1]


def test_trajectory_leg_total_duration_matches_leg_duration():
    sm = _fresh_sm()
    for _ in range(4):
        leg = sm.current_leg()
        assert leg.total_duration_s == pytest.approx(sm.leg_duration_s())
        sm.advance()


def test_n_joints_property():
    sm = _fresh_sm()
    leg = sm.current_leg()
    assert leg.n_joints == _N_JOINTS


def test_time_per_waypoint_is_positive():
    sm = _fresh_sm()
    for _ in range(4):
        leg = sm.current_leg()
        assert leg.time_per_waypoint_s > 0.0
        sm.advance()


# ---------------------------------------------------------------------------
# Complete kitting cycle covers all states' trajectory legs
# ---------------------------------------------------------------------------


def test_all_four_states_return_valid_legs_across_a_full_cycle():
    sm = _fresh_sm()
    visited = set()
    for _ in range(4):
        state = sm.state
        leg = sm.current_leg()
        assert isinstance(leg, TrajectoryLeg)
        assert len(leg.waypoints) > 0
        visited.add(state)
        sm.advance()
    assert visited == set(KittingState)


def test_go_to_tray_uses_bin_left_pick_as_start_in_first_cycle():
    """During GO_TO_TRAY the first waypoint must be the bin_left_pick config."""
    from safecollab.task_node import _Q

    sm = _fresh_sm()
    sm.advance()  # → PICK
    sm.advance()  # → GO_TO_TRAY
    leg = sm.current_leg()
    assert leg.waypoints[0] == _Q["bin_left_pick"]


def test_go_to_tray_uses_bin_right_pick_as_start_in_second_cycle():
    """During the second cycle GO_TO_TRAY uses bin_right_pick as start."""
    from safecollab.task_node import _Q

    sm = _fresh_sm()
    for _ in range(4):
        sm.advance()  # complete first cycle
    sm.advance()  # → PICK (second cycle, right bin)
    sm.advance()  # → GO_TO_TRAY
    leg = sm.current_leg()
    assert leg.waypoints[0] == _Q["bin_right_pick"]
