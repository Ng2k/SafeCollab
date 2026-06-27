# SafeCollab — AT-1..AT-5 scenario acceptance harness (launch_testing).
#
# Drives the safety loop via the ground-truth operator TF (safety_source:=ground_truth)
# so this harness has NO dependency on perception (Stream C).  A fixed path_seed
# makes the operator path deterministic for reproducible CI results.
#
# What is tested (AGENTS.md §9 "never cut" acceptance tests):
#   AT-1  Operator far → /safety/zone = "green", /safety/scale ≈ 1.0
#   AT-2  Operator approaches → zones observed in sequence: green → yellow → red
#   AT-3  Operator in red zone → /safety/scale ≈ 0 (protective stop)
#   AT-4  Operator retreats → scale recovers, /arm_controller/joint_trajectory resumes
#   AT-5  human_node SIGSTOP (TF goes stale) → /safety/scale → 0 (fail-safe)
#
# How AT-5 avoids messy exit codes
#   SIGSTOP freezes human_node without killing it.  The TF broadcast stops; after
#   loss_timeout (0.5 s sim-time) safety_monitor enters the "lost" fail-safe and
#   publishes scale = 0.  SIGCONT resumes human_node before teardown so it exits
#   cleanly via the normal launch SIGINT path (exit code -2, in the allowed set).
#
# Timing budget (headless CI, RTF ≈ 0.5 conservative)
#   _SETTLE_SECONDS     15 s  — let the full graph (gz + controllers + nodes) stabilise
#   _CONTROLLER_TIMEOUT 60 s  — cm service appears only after first gz world step
#   _RECORD_SECONDS     90 s  — covers ≥ 3 kitting cycles (9 s sim / 0.5 RTF = 18 s/cy)
#                               and ≥ 2 operator cycles (≤ 16 s sim / 0.5 RTF = 32 s/cy)
#   _AT5_STALE_WAIT     10 s  — loss_timeout(0.5 s sim) + RTF buffer + monitor tick
#   Total test time            ~ 175 s worst case (well within CI job defaults)
#
# OWNED BY: Stream G (CI/CD & container) — same ownership as the integration harness.

import os
import queue
import signal
import subprocess
import threading
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


# ---------------------------------------------------------------------------
# Launch file discovery
# ---------------------------------------------------------------------------


def _locate_cell_launch():
    """Return the absolute path to cell.launch.py, or None if not installed."""
    try:
        from ament_index_python.packages import (  # type: ignore[import]
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return None
    try:
        share = get_package_share_directory("safecollab")
    except PackageNotFoundError:
        return None
    cell_launch = os.path.join(share, "launch", "cell.launch.py")
    return cell_launch if os.path.exists(cell_launch) else None


# ---------------------------------------------------------------------------
# Test parameters
# ---------------------------------------------------------------------------

#: Deterministic operator path seed — makes AT-1..AT-4 reproducible across CI runs.
_PATH_SEED: int = 42

#: Timing constants — calibrated for headless Docker with RTF ≈ 0.5.
_SETTLE_SECONDS: float = 15.0
_CONTROLLER_TIMEOUT: float = 60.0
_RECORD_SECONDS: float = 90.0
_AT5_STALE_WAIT: float = 10.0


# ---------------------------------------------------------------------------
# Launch description
# ---------------------------------------------------------------------------


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Bring up the cell headless in ground-truth SSM mode with a fixed path seed.

    When cell.launch.py is not yet built (e.g. during parallel stream development),
    this returns an empty harness and the test methods self-skip — exactly the same
    pattern used by the integration harness in test_bringup.py.
    """
    cell_launch = _locate_cell_launch()

    actions = []
    if cell_launch is not None:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cell_launch),
                launch_arguments={
                    "headless": "true",
                    # Ground-truth mode: safety_monitor tracks world→human_gt TF
                    # published by human_node.  No dependency on perception_node.
                    "safety_source": "ground_truth",
                    # Fixed seed: OperatorPath.generate_random(Random(42)) produces
                    # a deterministic approach-and-retreat sequence that covers all
                    # safety zones within the _RECORD_SECONDS window.
                    "path_seed": str(_PATH_SEED),
                }.items(),
            )
        )

    actions.append(launch_testing.actions.ReadyToTest())
    context = {"cell_present": cell_launch is not None}
    return launch.LaunchDescription(actions), context


# ---------------------------------------------------------------------------
# Topic-collection helpers  (mirror the style of test_bringup.py)
# ---------------------------------------------------------------------------


def _collect_float32_values(topic: str, duration_s: float) -> list:
    """Collect std_msgs/Float32 data values from *topic* for *duration_s* seconds.

    Spawns ``ros2 topic echo topic`` and reads its output via a daemon thread so
    that ``readline()`` never blocks the outer deadline check (same pattern as
    ``_measure_hz`` in test_bringup.py).  Returns a list of floats.
    """
    proc = subprocess.Popen(
        ["ros2", "topic", "echo", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    values: list = []
    lq: queue.Queue = queue.Queue()

    def _reader() -> None:
        for line in proc.stdout:
            lq.put(line)

    threading.Thread(target=_reader, daemon=True).start()

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        try:
            line = lq.get(timeout=0.1)
        except queue.Empty:
            continue
        if line.startswith("data: "):
            try:
                values.append(float(line[6:].strip()))
            except ValueError:
                pass

    proc.terminate()
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    return values


def _collect_string_values(
    topic: str,
    duration_s: float,
    valid_values: "set | None" = None,
    transient_local: bool = False,
) -> list:
    """Collect std_msgs/String data values from *topic* for *duration_s* seconds.

    If *valid_values* is provided only strings in that set are kept (filters
    YAML boilerplate).  Surrounding quotes are stripped for robustness across
    ROS echo formats.

    *transient_local* makes the echo subscriber request a RELIABLE +
    TRANSIENT_LOCAL QoS profile.  This is required for topics the §3 contract
    declares transient_local (e.g. ``/safety/zone``): ``ros2 topic echo`` uses a
    VOLATILE subscription by default, which does not reliably receive from a
    TRANSIENT_LOCAL publisher in this RMW — the symptom is an empty capture even
    though the topic is being published every tick.  Matching the publisher's
    durability fixes it (and also delivers the latched last sample on connect).
    """
    cmd = ["ros2", "topic", "echo", topic]
    if transient_local:
        cmd += [
            "--qos-reliability",
            "reliable",
            "--qos-durability",
            "transient_local",
        ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    values: list = []
    lq: queue.Queue = queue.Queue()

    def _reader() -> None:
        for line in proc.stdout:
            lq.put(line)

    threading.Thread(target=_reader, daemon=True).start()

    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        try:
            line = lq.get(timeout=0.1)
        except queue.Empty:
            continue
        if line.startswith("data: "):
            val = line[6:].strip().strip("'\"")
            if valid_values is None or val in valid_values:
                values.append(val)

    proc.terminate()
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    return values


def _topic_has_messages(topic: str, window_s: float = 15.0) -> bool:
    """Return True if *topic* receives at least one message within *window_s* s.

    Uses ``ros2 topic echo --once`` which exits with code 0 after the first
    message arrives; code 1 / TimeoutExpired means nothing arrived.
    """
    proc = subprocess.Popen(
        ["ros2", "topic", "echo", "--once", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    try:
        proc.wait(timeout=window_s)
        return proc.returncode == 0
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        return False


def _wait_controllers_active(timeout_s: float = 60.0) -> bool:
    """Poll ``ros2 control list_controllers`` until both arm controllers are active.

    Mirrors the implementation in test_bringup.py; retries every 1 s within
    *timeout_s*.  The controller_manager service only appears after gz sim's
    first world step, so the spawners may not have finished at settle time.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            out = subprocess.check_output(
                ["ros2", "control", "list_controllers"],
                text=True,
                stderr=subprocess.DEVNULL,
                timeout=8,
            )
            lines = out.splitlines()
            arm_ok = any("arm_controller" in ln and "active" in ln for ln in lines)
            jsb_ok = any(
                "joint_state_broadcaster" in ln and "active" in ln for ln in lines
            )
            if arm_ok and jsb_ok:
                return True
        except Exception:  # noqa: BLE001
            pass
        time.sleep(1.0)
    return False


# ---------------------------------------------------------------------------
# Active-test class — runs while the launch is alive
# ---------------------------------------------------------------------------


class TestSSMScenario(unittest.TestCase):
    """AT-1..AT-5 acceptance assertions over the live headless SSM cell.

    A single long-running test method drives all five acceptance tests so that
    the sequential phases (settle → record → assert → AT-5 kill/check) share
    one bring-up, minimising total wall-clock time.
    """

    def test_at1_through_at5(self, cell_present: bool) -> None:
        """Record SSM behaviour over one operator cycle and assert AT-1..AT-5."""
        if not cell_present:
            self.skipTest(
                "safecollab/launch/cell.launch.py not built yet; "
                "scenario harness is wired and ready (mirrors integration harness)."
            )

        # ------------------------------------------------------------------
        # Phase 1 — settle + wait for controllers
        # ------------------------------------------------------------------
        # Let gz sim, robot_state_publisher, spawners, and application nodes
        # all reach steady state before sampling any topic.
        time.sleep(_SETTLE_SECONDS)
        self.assertTrue(
            _wait_controllers_active(_CONTROLLER_TIMEOUT),
            f"arm_controller and joint_state_broadcaster must both report 'active' "
            f"within {_CONTROLLER_TIMEOUT:.0f} s of bring-up "
            "(spawners are ordered: spawn → jsb → arm_controller in cell.launch.py).",
        )

        # ------------------------------------------------------------------
        # Phase 2 — record /safety/scale, /safety/zone, /arm_controller/joint_trajectory
        # ------------------------------------------------------------------
        # Three independent subprocess collectors run in parallel for _RECORD_SECONDS.
        # • scales: Float32 samples at ~20 Hz → ~1800 samples over 90 s
        # • zones:  String samples (green|yellow|red|lost) at ~20 Hz → ~1800 samples
        # • arm_traj_seen: bool — did /arm_controller/joint_trajectory publish at all?
        scales: list = []
        zones: list = []
        arm_traj_seen: list = [False]  # mutable container for thread result

        def _do_scales() -> None:
            scales.extend(_collect_float32_values("/safety/scale", _RECORD_SECONDS))

        def _do_zones() -> None:
            zones.extend(
                _collect_string_values(
                    "/safety/zone",
                    _RECORD_SECONDS,
                    valid_values={"green", "yellow", "red", "lost"},
                    # §3 contract: /safety/zone is reliable + transient_local.
                    transient_local=True,
                )
            )

        def _do_arm_traj() -> None:
            # A single message within _RECORD_SECONDS is sufficient to confirm
            # motion_node is producing re-timed trajectory commands.
            arm_traj_seen[0] = _topic_has_messages(
                "/arm_controller/joint_trajectory",
                window_s=_RECORD_SECONDS,
            )

        threads = [
            threading.Thread(target=_do_scales, daemon=True),
            threading.Thread(target=_do_zones, daemon=True),
            threading.Thread(target=_do_arm_traj, daemon=True),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=_RECORD_SECONDS + 10.0)

        # Sanity: both primary topics must have published something.
        self.assertGreater(
            len(scales),
            0,
            "/safety/scale published no messages during the recording window "
            f"({_RECORD_SECONDS:.0f} s); check that safety_monitor started and is "
            "running (it publishes at 20 Hz regardless of zone, even in 'lost').",
        )
        self.assertGreater(
            len(zones),
            0,
            "/safety/zone captured no messages during the recording window. "
            "Since /safety/scale (published on the same tick) was captured, the "
            "monitor is alive — this is a QoS mismatch: /safety/zone is "
            "reliable+transient_local (§3 contract), so the echo subscriber must "
            "request matching durability (see transient_local=True above).",
        )

        # ------------------------------------------------------------------
        # AT-1: green zone and scale ≈ 1.0 observed (operator is far)
        # ------------------------------------------------------------------
        # At the initial standing position (x=0.9, y=±0.65, z=1.10) the operator
        # is > 1 m from the nearest robot frame — well above d_yellow ≈ 0.84 m.
        # classify() returns ("green", 1.0).  The first non-lost samples should
        # all be 1.0; we assert the max over the recording window.
        max_scale = max(scales)
        self.assertGreater(
            max_scale,
            0.9,
            f"AT-1 FAIL: max /safety/scale = {max_scale:.3f}; expected ≈ 1.0 when "
            "the operator is at the initial standing position (far from the arm). "
            "Check safety_source=ground_truth is reaching safety_monitor and that "
            "the human_gt TF is being published by human_node.",
        )
        self.assertIn(
            "green",
            zones,
            "AT-1 FAIL: zone 'green' not observed during the recording window; "
            "expected when the operator is at the standing position (d > d_yellow).",
        )

        # ------------------------------------------------------------------
        # AT-2: zone transitions green → yellow → red (approach sequence)
        # ------------------------------------------------------------------
        # Find the first occurrence index of each zone in the temporal sequence.
        # As the operator walks from the standing position toward the tray,
        # d decreases monotonically → green → yellow → red.

        def _first_idx(z: str) -> int:
            try:
                return zones.index(z)
            except ValueError:
                return -1

        g_idx = _first_idx("green")
        y_idx = _first_idx("yellow")
        r_idx = _first_idx("red")

        self.assertGreater(
            g_idx,
            -1,
            "AT-2 FAIL: 'green' zone not observed; expected at the operator's "
            "starting position where d >> d_yellow ≈ 0.84 m.",
        )
        self.assertGreater(
            y_idx,
            -1,
            "AT-2 FAIL: 'yellow' zone not observed; expected as the operator "
            "enters the yellow band (d_red < d < d_yellow, ≈ 0.43–0.84 m).",
        )
        self.assertGreater(
            r_idx,
            -1,
            "AT-2 FAIL: 'red' zone not observed; expected when the operator's "
            "hand enters the tray (d ≤ d_red ≈ 0.43 m from the nearest robot frame). "
            "If this fails, increase _RECORD_SECONDS or verify the kitting arm "
            "and operator cycles overlap during the recording window.",
        )
        self.assertGreater(
            y_idx,
            g_idx,
            "AT-2 FAIL: yellow must appear after green in the temporal sequence "
            "(operator approaches: d decreases from green through yellow to red).",
        )
        self.assertGreater(
            r_idx,
            y_idx,
            "AT-2 FAIL: red must appear after yellow in the temporal sequence.",
        )

        # ------------------------------------------------------------------
        # AT-3: protective stop (scale = 0) in the red zone
        # ------------------------------------------------------------------
        min_scale = min(scales)
        self.assertLess(
            min_scale,
            0.05,
            f"AT-3 FAIL: min /safety/scale = {min_scale:.3f}; expected ≈ 0.0 "
            "(protective stop) when the operator is inside d_red ≈ 0.43 m. "
            "classify() returns ('red', 0.0) for d ≤ d_red — check that the "
            "safety_monitor is calling safety_logic.classify() correctly.",
        )

        # ------------------------------------------------------------------
        # AT-4: clean resume — scale recovers toward 1.0 after the stop
        #        AND the arm trajectory topic resumes
        # ------------------------------------------------------------------
        # Find the last index where scale was in protective-stop territory (<0.05),
        # then check that subsequent samples climb back above 0.5 (arm resumes).
        stop_indices = [i for i, s in enumerate(scales) if s < 0.05]
        # stop_indices is guaranteed non-empty because AT-3 passed.
        last_stop = max(stop_indices)
        post_stop_scales = [s for s in scales[last_stop + 1 :] if s > 0.5]
        self.assertGreater(
            len(post_stop_scales),
            0,
            "AT-4 FAIL: /safety/scale did not recover above 0.5 after the "
            "protective stop (last stop at sample index "
            f"{last_stop}/{len(scales) - 1}). "
            "Operator retreat should raise scale back toward 1.0 (clean resume). "
            "Check motion_node resume logic and the operator path's withdrawal "
            "waypoints (OperatorPath.generate_random, waypoints 4–6).",
        )
        self.assertTrue(
            arm_traj_seen[0],
            "AT-4 FAIL: /arm_controller/joint_trajectory published no messages "
            f"in the {_RECORD_SECONDS:.0f} s recording window. "
            "motion_node must publish re-timed trajectory commands when scale > 0; "
            "check that task_node is publishing /motion/nominal_trajectory and "
            "motion_node is running.",
        )

        # ------------------------------------------------------------------
        # AT-5: fail-safe on TF loss — SIGSTOP human_node → stale human_gt TF
        # ------------------------------------------------------------------
        # SIGSTOP freezes the human_node Python process without killing it.
        # The world→human_gt TF broadcast stops; after loss_timeout (0.5 s sim-time,
        # i.e. ~1 s+ wall-clock at RTF ≈ 0.5) safety_monitor declares the TF stale,
        # passes d=None to classify(), and publishes scale = 0 ("lost" fail-safe).
        # SIGCONT resumes human_node so it exits cleanly at teardown (-SIGINT).
        subprocess.run(
            ["pkill", "-STOP", "-f", "human_node"],
            stderr=subprocess.DEVNULL,
            check=False,
        )
        # Wait well past loss_timeout (0.5 s sim) + safety_monitor tick + RTF margin.
        time.sleep(_AT5_STALE_WAIT)

        # Collect a 5 s window of scale samples; all must be ≈ 0 (fail-safe active).
        scale_at5 = _collect_float32_values("/safety/scale", 5.0)

        # Resume human_node BEFORE asserting so teardown stays clean even on failure.
        subprocess.run(
            ["pkill", "-CONT", "-f", "human_node"],
            stderr=subprocess.DEVNULL,
            check=False,
        )

        self.assertGreater(
            len(scale_at5),
            0,
            "AT-5 FAIL: /safety/scale topic went silent after human_node was "
            "paused (SIGSTOP); safety_monitor must keep publishing (scale=0) even "
            "in the 'lost' fail-safe state.",
        )
        bad_samples = [s for s in scale_at5 if s >= 0.05]
        self.assertEqual(
            len(bad_samples),
            0,
            f"AT-5 FAIL: {len(bad_samples)}/{len(scale_at5)} scale samples were "
            f"≥ 0.05 (max={max(scale_at5):.3f}) after human_gt TF went stale. "
            "Expected scale = 0.0 (classify(None, ...) → ('lost', 0.0) — FR-9). "
            "Check loss_timeout in config/safety.yaml and _lookup_human_tf() in "
            "safety_monitor.py.",
        )


# ---------------------------------------------------------------------------
# Post-shutdown test — runs after the launch has torn down
# ---------------------------------------------------------------------------


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """All launched processes must exit cleanly (no bring-up crashes).

    Allowed exit codes mirror test_bringup.py exactly:
      0          clean exit (nodes that catch SIGINT and return normally)
      -SIGINT    normal teardown via launch's SIGINT delivery (-2)
      -SIGTERM   gz sim's ruby wrapper sometimes misses the 5 s SIGINT grace
                 period and receives SIGTERM from launch (-15)
    human_node is SIGSTOP/SIGCONT'd during AT-5 (not killed), so it exits via
    the normal -SIGINT path at teardown — no special case needed.
    """

    def test_exit_codes(self, proc_info: object, cell_present: bool) -> None:
        if not cell_present:
            self.skipTest("nothing launched; no exit codes to check.")
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )
