# SafeCollab — P4 robustness acceptance harness (launch_testing).
#
# Extends the AT-1..AT-5 ground-truth harness (test_at1_at5.py) with the P4
# robustness acceptance cases (AGENTS.md §10 P4 rows, §12 Definition of Done).
# See docs/ROADMAP-P4.md for the sprint plan. Each case is delivered in its own
# sprint via TDD (red -> green -> refactor); until a case lands it is skipped, so
# the file is collected and the pending cases are VISIBLE in CI while the pipeline
# stays green.
#
#   AT-6  Randomised operator paths generalise (no path-specific tuning)   [Sprint 1] DONE
#   AT-5r Transient detection loss -> fail-safe -> re-acquire -> resume      [Sprint 4] DONE
#
# Two P4 cases are pure-logic properties, proven faster and deterministically as
# UNIT tests rather than live scenarios:
#   AT-4r mid-trajectory stop resumes without backtrack -> test_motion_node.py
#   AT-3r fast crossing cannot skip the red band (sampling invariant)
#                                                  -> test_fast_crossing.py
#
# Shared topic-collection / zone-analysis helpers come from the sibling
# _ssm_harness module (no duplication with the AT-1..AT-5 harness).
#
# OWNED BY: the P4 robustness sprint (feat/p4-robustness).

import os
import signal
import subprocess
import sys
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# launch_test loads this file by path; make its directory importable first.
sys.path.insert(0, os.path.dirname(__file__))
from _ssm_harness import (  # noqa: E402  (intentional: after the sys.path tweak)
    compress,
    count_escalations,
    count_recoveries,
    locate_cell_launch,
    record_safety_topics,
    wait_controllers_active,
)


# ---------------------------------------------------------------------------
# Test parameters
# ---------------------------------------------------------------------------

#: Operator path seed for AT-6. A *different* deterministic seed from the
#: AT-1..AT-5 harness (42), so the suite collectively exercises two independent
#: seed sequences. With a fixed seed human_node regenerates a fresh random path
#: each operator cycle, so a single long run drives several DISTINCT paths.
_PATH_SEED: int = 7

#: Timing — calibrated for headless Docker with RTF ≈ 0.4–0.5 (software GL).
_SETTLE_SECONDS: float = 15.0
_CONTROLLER_TIMEOUT: float = 60.0
#: Long enough to cover several operator cycles (each ≈ 8–15 s sim → ≈ 20–35 s
#: wall at RTF ≈ 0.4–0.5), so multiple distinct generated paths are exercised.
_RECORD_SECONDS: float = 150.0

#: AT-6 thresholds. Each completed green→yellow→red escalation corresponds to one
#: distinct generated path driving the operator into the protective stop; ≥ 3
#: proves the SSM behaviour generalises across paths (no path-specific tuning).
#: Recoveries are one fewer in the worst case (window may end mid-stop).
_MIN_ESCALATIONS: int = 3
_MIN_RECOVERIES: int = 2

#: AT-5 hardening (transient detection loss) timing.
#: Wall-clock wait after SIGSTOP for loss_timeout (0.5 s sim) + RTF margin, and
#: after SIGCONT for the TF to return and the loop to re-acquire.
_AT5_STALE_WAIT: float = 10.0
_AT5_RESUME_WAIT: float = 5.0


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Bring up the cell headless in ground-truth SSM mode with a fixed seed.

    AT-6 is a *behaviour-generalisation* test, independent of perception, so it
    runs in ``safety_source:=ground_truth`` (same as the AT-1..AT-5 harness). When
    cell.launch.py is not installed the harness is empty and the test self-skips.
    """
    cell_launch = locate_cell_launch()

    actions = []
    if cell_launch is not None:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cell_launch),
                launch_arguments={
                    "headless": "true",
                    "safety_source": "ground_truth",
                    "path_seed": str(_PATH_SEED),
                }.items(),
            )
        )

    actions.append(launch_testing.actions.ReadyToTest())
    context = {"cell_present": cell_launch is not None}
    return launch.LaunchDescription(actions), context


# ---------------------------------------------------------------------------
# Active-test class — runs while the launch is alive
# ---------------------------------------------------------------------------


class TestRobustness(unittest.TestCase):
    """P4 robustness acceptance cases — delivered one per sprint via TDD."""

    def test_at6_randomised_paths_generalise(self, cell_present: bool) -> None:
        """AT-6: the SSM behaviour generalises across several distinct paths.

        With a fixed seed, human_node regenerates a fresh random operator path
        each cycle, so over one long recording the cell is driven by several
        DISTINCT paths. Using the SAME ``risk.yaml`` thresholds (no path-specific
        tuning), each path must escalate green→yellow→red and then resume. We
        require at least ``_MIN_ESCALATIONS`` complete escalations and
        ``_MIN_RECOVERIES`` clean resumes, and that the zone never spuriously
        enters 'lost' (the ground-truth TF is always published).
        """
        if not cell_present:
            self.skipTest(
                "safecollab/launch/cell.launch.py not built yet; "
                "P4 robustness harness is wired and ready."
            )

        # Phase 1 — settle + wait for controllers (mirrors the AT-1..AT-5 harness).
        time.sleep(_SETTLE_SECONDS)
        self.assertTrue(
            wait_controllers_active(_CONTROLLER_TIMEOUT),
            "arm_controller and joint_state_broadcaster must both report 'active' "
            f"within {_CONTROLLER_TIMEOUT:.0f} s of bring-up.",
        )

        # Phase 2 — record zones + scales over several operator cycles with one
        # in-process rclpy subscriber (QoS matched to the §3 contract).
        rec = record_safety_topics(_RECORD_SECONDS, zone=True, scale=True)
        zones = rec["zones"]
        scales = rec["scales"]

        self.assertGreater(
            len(zones), 0, "/safety/zone captured no messages (QoS/monitor issue)."
        )
        self.assertGreater(
            len(scales), 0, "/safety/scale captured no messages (monitor not running)."
        )

        transitions = compress(zones)
        escalations = count_escalations(transitions)
        recoveries = count_recoveries(scales)

        # No spurious fail-safe: ground-truth TF is always present, so 'lost'
        # must never appear during normal operation.
        self.assertNotIn(
            "lost",
            zones,
            "AT-6 FAIL: zone entered 'lost' during normal ground-truth operation — "
            f"the human_gt TF should never go stale here. Transitions: {transitions}.",
        )

        # Generalisation: the green→yellow→red escalation recurs across the
        # distinct generated paths.
        self.assertGreaterEqual(
            escalations,
            _MIN_ESCALATIONS,
            f"AT-6 FAIL: only {escalations} complete green→yellow→red escalation(s) "
            f"observed over {_RECORD_SECONDS:.0f} s (need ≥ {_MIN_ESCALATIONS}). With "
            "a fixed seed each operator cycle is a distinct random path, so too few "
            "escalations means the behaviour does not generalise across paths. "
            f"Transitions: {transitions}.",
        )

        # Clean resume recurs too: each protective stop is followed by recovery.
        self.assertGreaterEqual(
            recoveries,
            _MIN_RECOVERIES,
            f"AT-6 FAIL: only {recoveries} protective-stop→resume recovery(ies) "
            f"observed (need ≥ {_MIN_RECOVERIES}). Each approach should stop then "
            "resume as the operator retreats; too few means resume does not "
            "generalise across paths.",
        )

    def test_recovery_after_transient_detection_loss(self, cell_present: bool) -> None:
        """AT-5 hardening: transient detection loss -> fail-safe -> re-acquire -> resume.

        Runs AFTER ``test_at6_...`` (unittest orders methods alphabetically and
        ``'test_at6' < 'test_r'``), so the SIGSTOP below cannot perturb the AT-6
        generalisation recording.

        SIGSTOP freezes ``human_node`` without killing it: the ground-truth TF
        stops, so after ``loss_timeout`` the monitor must fail-safe (zone ``lost``,
        scale 0) — a transient detection loss. SIGCONT restores the TF; the loop
        must re-acquire and resume (scale recovers above 0.5) with no node
        faulting (the post-shutdown ``test_exit_codes`` asserts the clean exit).
        ``human_node`` is always SIGCONT'd before this method returns — even on an
        assertion failure — so teardown stays clean.
        """
        if not cell_present:
            self.skipTest(
                "safecollab/launch/cell.launch.py not built yet; "
                "P4 robustness harness is wired and ready."
            )

        # The cell is already settled (test_at6 ran first); just confirm the
        # controllers are still active before perturbing the graph.
        self.assertTrue(
            wait_controllers_active(_CONTROLLER_TIMEOUT),
            "controllers must be active before inducing the transient loss.",
        )

        # --- Induce a transient detection loss: freeze human_node (TF goes stale).
        subprocess.run(
            ["pkill", "-STOP", "-f", "human_node"],
            stderr=subprocess.DEVNULL,
            check=False,
        )
        try:
            time.sleep(_AT5_STALE_WAIT)  # past loss_timeout (0.5 s sim) + RTF margin
            stalled = record_safety_topics(5.0, zone=True, scale=True)
            scale_lost = stalled["scales"]
            zone_lost = stalled["zones"]
        finally:
            # ALWAYS resume human_node so the loop can recover and teardown is clean.
            subprocess.run(
                ["pkill", "-CONT", "-f", "human_node"],
                stderr=subprocess.DEVNULL,
                check=False,
            )

        # --- Allow the TF to return and the loop to re-acquire, then sample.
        time.sleep(_AT5_RESUME_WAIT)
        scale_resume = record_safety_topics(10.0, scale=True)["scales"]

        # Fail-safe held during the loss: scale pinned at 0, zone 'lost'.
        self.assertGreater(
            len(scale_lost),
            0,
            "AT-5 FAIL: /safety/scale went silent while the human TF was stale; "
            "safety_monitor must keep publishing (scale=0) even in 'lost'.",
        )
        bad = [s for s in scale_lost if s >= 0.05]
        self.assertEqual(
            len(bad),
            0,
            f"AT-5 FAIL: {len(bad)}/{len(scale_lost)} scale samples were >= 0.05 "
            f"(max {max(scale_lost):.3f}) while the human TF was stale; expected "
            "0.0 (fail-safe 'lost' — FR-9; the last position is never reused).",
        )
        self.assertIn(
            "lost",
            zone_lost,
            "AT-5 FAIL: zone did not enter 'lost' during the transient detection "
            f"loss; observed {sorted(set(zone_lost))}.",
        )

        # Re-acquire and resume: once the TF returns, scale must recover.
        self.assertGreater(
            len(scale_resume),
            0,
            "AT-5 FAIL: /safety/scale silent after the operator was re-acquired.",
        )
        self.assertTrue(
            any(s > 0.5 for s in scale_resume),
            "AT-5 FAIL: scale did not recover above 0.5 after re-acquire "
            f"(max {max(scale_resume):.3f}); the loop must resume once the human "
            "TF returns (no permanent latch in the 'lost' fail-safe).",
        )


# ---------------------------------------------------------------------------
# Post-shutdown test — every launched process must exit cleanly
# ---------------------------------------------------------------------------


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """All launched processes must exit cleanly (mirrors the AT-1..AT-5 harness)."""

    def test_exit_codes(self, proc_info: object, cell_present: bool) -> None:
        if not cell_present:
            self.skipTest("nothing launched; no exit codes to check.")
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )
