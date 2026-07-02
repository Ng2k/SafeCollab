"""Unit spec for safecollab.planner_node — the pure kitting-plan logic.

Only the ROS/MoveIt-free parts are tested (``kitting_legs``, ``cycle_sequence``,
``KittingLeg``); the MoveItPy wrapper in ``main()`` needs a live graph and is
``pragma: no cover`` (AGENTS.md ground rule 3 / §9).
"""

import pytest

from safecollab.planning import KittingLeg, cycle_sequence, kitting_legs


def test_one_cycle_has_six_legs_in_order():
    legs = kitting_legs("left")
    assert [leg.state for leg in legs] == [
        "GO_TO_BIN_LEFT",
        "PICK",
        "LIFT_BIN",
        "GO_TO_TRAY",
        "DROP",
        "LIFT_TRAY",
    ]


def test_transits_are_ptp_dips_are_lin():
    legs = {leg.state: leg for leg in kitting_legs("left")}
    assert legs["GO_TO_BIN_LEFT"].motion == "ptp"
    assert legs["GO_TO_TRAY"].motion == "ptp"
    for dip in ("PICK", "LIFT_BIN", "DROP", "LIFT_TRAY"):
        assert legs[dip].motion == "lin", f"{dip} should be a straight LIN move"


def test_pick_dips_below_the_above_height():
    legs = {leg.state: leg for leg in kitting_legs("left")}
    assert legs["PICK"].xyz[2] < legs["GO_TO_BIN_LEFT"].xyz[2]
    assert legs["DROP"].xyz[2] < legs["GO_TO_TRAY"].xyz[2]


def test_lift_returns_to_above_height():
    legs = {leg.state: leg for leg in kitting_legs("left")}
    assert legs["LIFT_BIN"].xyz[2] == pytest.approx(legs["GO_TO_BIN_LEFT"].xyz[2])
    assert legs["LIFT_TRAY"].xyz[2] == pytest.approx(legs["GO_TO_TRAY"].xyz[2])


def test_left_and_right_bins_differ_in_y():
    left = kitting_legs("left")[0].xyz
    right = kitting_legs("right")[0].xyz
    assert left[1] == pytest.approx(-right[1])
    assert left[1] != right[1]


def test_bin_and_tray_targets_are_distinct():
    legs = {leg.state: leg for leg in kitting_legs("left")}
    assert legs["GO_TO_BIN_LEFT"].xyz[:2] != legs["GO_TO_TRAY"].xyz[:2]


def test_invalid_bin_side_raises():
    with pytest.raises(ValueError, match="bin_side"):
        kitting_legs("middle")


def test_all_legs_have_positive_settle():
    for leg in kitting_legs("right"):
        assert leg.settle_s > 0.0


def test_leg_is_frozen():
    leg = kitting_legs("left")[0]
    with pytest.raises(Exception):
        leg.state = "X"  # type: ignore[misc]


def test_cycle_sequence_alternates_bins():
    seq = cycle_sequence(2)
    assert len(seq) == 12  # 2 cycles x 6 legs
    assert seq[0].state == "GO_TO_BIN_LEFT"
    assert seq[6].state == "GO_TO_BIN_RIGHT"


def test_cycle_sequence_zero_is_empty():
    assert cycle_sequence(0) == []


def test_kitting_leg_fields():
    leg = KittingLeg("PICK", "lin", (0.1, 0.2, 0.8), 0.4)
    assert leg.state == "PICK"
    assert leg.motion == "lin"
    assert leg.xyz == (0.1, 0.2, 0.8)
    assert leg.settle_s == 0.4
