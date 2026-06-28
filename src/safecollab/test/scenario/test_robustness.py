# SafeCollab — P4 robustness acceptance harness (launch_testing).
#
# Extends the AT-1..AT-5 ground-truth harness (test_at1_at5.py) with the P4
# robustness acceptance cases (AGENTS.md §10 P4 rows, §12 Definition of Done).
# See docs/ROADMAP-P4.md for the sprint plan. Each case is delivered in its own
# sprint via TDD (red -> green -> refactor); until a case lands it is skipped, so
# the file is collected and the pending cases are VISIBLE in CI while the pipeline
# stays green.
#
#   AT-6  Randomised operator paths generalise (no path-specific tuning)   [Sprint 1]
#   AT-4r Recover cleanly from a protective stop triggered mid-trajectory   [Sprint 2]
#   AT-3r Fast crossing drives red + scale->0 within one control tick       [Sprint 3]
#   AT-5r Transient detection loss -> fail-safe -> re-acquire -> resume      [Sprint 4]
#
# Each sprint replaces the empty harness below with the live headless cell
# bring-up appropriate to the case under test (e.g. safety_source:=perceived for
# the transient-loss recovery case), reusing the topic-collection helpers from the
# AT-1..AT-5 harness (to be factored into a shared module when the first live case
# lands, rather than duplicated).
#
# OWNED BY: the P4 robustness sprint (feat/p4-robustness).

import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Empty harness for the scaffold — the P4 cases are present but skipped.

    Returning only ``ReadyToTest`` lets ``launch_test`` collect this file and
    surface the pending robustness cases in CI without paying for a cell
    bring-up. Each sprint (docs/ROADMAP-P4.md) swaps this for the live launch its
    case needs.
    """
    return (
        launch.LaunchDescription([launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestRobustness(unittest.TestCase):
    """P4 robustness acceptance cases — delivered one per sprint via TDD."""

    def test_harness_scaffolded(self) -> None:
        """Smoke: the P4 robustness harness is collected and runnable.

        Keeps the scenario stage green and gives ``launch_test`` an active case to
        execute while the four acceptance cases below are still pending.
        """
        self.assertTrue(True)

    @unittest.skip("P4 Sprint 1: AT-6 randomised paths — pending")
    def test_at6_randomised_paths_generalise(self) -> None:
        """AT-6: across >=5 operator path seeds the SSM loop still drives
        green -> yellow -> red and then resumes, using the SAME risk.yaml
        thresholds — proving the behaviour generalises with no path-specific
        tuning (FR-11)."""

    @unittest.skip("P4 Sprint 2: mid-trajectory stop/resume — pending")
    def test_at4_recover_from_mid_trajectory_stop(self) -> None:
        """AT-4 hardening: a protective stop triggered BETWEEN waypoints resumes
        cleanly by re-planning from the current state (no jerk, no backtrack)."""

    @unittest.skip("P4 Sprint 3: fast-crossing reaction — pending")
    def test_at3_fast_crossing_stops_within_one_tick(self) -> None:
        """AT-3 hardening: a fast operator crossing drives /safety/zone to red and
        /safety/scale to 0 within one control tick."""

    @unittest.skip("P4 Sprint 4: transient detection loss recovery — pending")
    def test_at5_transient_loss_recovers(self) -> None:
        """AT-5 hardening: a transient perception dropout -> lost/protective-stop
        with no node faulting, then re-acquire -> clean resume."""
