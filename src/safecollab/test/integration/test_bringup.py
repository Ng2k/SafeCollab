# SafeCollab — headless integration bring-up harness (launch_testing).
#
# OWNED BY: Stream G (CI/CD). This is the *scaffolding* only — it wires up a
# headless launch_testing harness and the assertion slots the other streams
# fill in as their nodes land:
#
#   * controllers active   -> `ros2 control list_controllers` (Stream A/D)
#   * TF chain resolves     -> world->...->tcp and world->human_gt (Stream A/E)
#   * topic rates           -> /safety/scale >= 20 Hz (Stream F)
#   * command path re-timed -> /arm_controller/joint_trajectory (Stream D)
#
# Until `safecollab/launch/cell.launch.py` exists the harness brings nothing up
# and the assertion tests self-skip, so the integration stage stays green while
# the pipeline is wired and ready (AGENTS.md §6 Stream G, §9 integration layer).

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


def _locate_cell_launch():
    """Return the path to the headless cell launch file, or None if absent."""
    try:
        from ament_index_python.packages import (
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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Bring the cell up headless when available; otherwise an empty harness."""
    cell_launch = _locate_cell_launch()

    actions = []
    if cell_launch is not None:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cell_launch),
                # Headless: no GUI/RViz/gz client — CI has no display (§8).
                launch_arguments={"headless": "true"}.items(),
            )
        )

    actions.append(launch_testing.actions.ReadyToTest())
    context = {"cell_present": cell_launch is not None}
    return launch.LaunchDescription(actions), context


#: Seconds to let the cell run before the harness tears it down. Without this
#: settle the active tests finish in ~0.01 s and SIGINT lands *during* each
#: node's Python interpreter startup, producing nondeterministic exit codes
#: (-2 normally, but 1 when SIGINT aborts site-module init) that fail the
#: clean-shutdown check for no real reason. Letting the nodes reach steady
#: state first means a SIGINT at teardown yields a clean -2 — and a node with a
#: genuine startup crash now exits during the settle and is still caught.
_SETTLE_SECONDS = 8.0


# ---------------------------------------------------------------------------
# Helpers for live assertions
# ---------------------------------------------------------------------------


def _wait_controllers_active(timeout_s=30.0):
    """Poll ``ros2 control list_controllers`` until both arm controllers are active.

    Retries every 1 s within ``timeout_s``; returns ``True`` on success and
    ``False`` when the deadline expires.  Broad exception handling is
    intentional: the controller_manager service may not exist yet while
    gz sim is still initialising (gz_ros2_control::GazeboSimROS2ControlPlugin
    loads the CM only after the first gz world step).
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


def _measure_rate_simtime(topic, msg_type, n_samples=40, timeout_s=20.0):
    """Average publish rate of *topic* measured in SIMULATION time (Hz).

    ``ros2 topic hz`` measures wall-clock rate, which under-reports a
    sim-time-driven publisher when the headless sim runs below real-time
    (RTF < 1 — typical on a CPU-limited CI runner: a true 20 Hz sim-time timer
    reads ~15.5 Hz wall-clock at RTF≈0.78).  ``safety_monitor``'s timer fires on
    sim time (``use_sim_time:=true``), so we time the messages against a
    ``use_sim_time`` probe node's clock (which follows ``/clock``) — making the
    result independent of the real-time factor.

    Returns the rate in Hz, or ``None`` if fewer than two messages arrive
    within ``timeout_s`` wall-clock seconds.
    """
    import rclpy
    from rclpy.parameter import Parameter

    rclpy.init()
    node = rclpy.create_node(
        "_safecollab_rate_probe",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    sim_stamps_ns = []

    def _cb(_msg):
        sim_stamps_ns.append(node.get_clock().now().nanoseconds)

    node.create_subscription(msg_type, topic, _cb, 10)
    deadline = time.monotonic() + timeout_s
    try:
        while len(sim_stamps_ns) < n_samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(sim_stamps_ns) < 2:
        return None
    sim_dt = (sim_stamps_ns[-1] - sim_stamps_ns[0]) / 1e9
    return (len(sim_stamps_ns) - 1) / sim_dt if sim_dt > 0 else None


def _tf_available(parent, child, timeout_s=8.0):
    """Return ``True`` if the TF transform *parent* → *child* is available.

    Spawns ``ros2 run tf2_ros tf2_echo`` and waits for a ``Translation:``
    line on stdout (the signal that at least one transform was received).
    Uses a daemon thread so the ``readline()`` loop does not block the
    outer timeout check.  Stderr is discarded (tf2_echo writes
    ``[WARN] Waiting for transform`` there while the TF is not yet ready).
    """
    proc = subprocess.Popen(
        ["ros2", "run", "tf2_ros", "tf2_echo", parent, child],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    lq = queue.Queue()

    def _reader():
        for line in proc.stdout:
            lq.put(line)

    threading.Thread(target=_reader, daemon=True).start()

    found = False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            line = lq.get(timeout=0.5)
        except queue.Empty:
            continue
        if "Translation" in line:
            found = True
            break

    proc.terminate()
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    return found


class TestHeadlessBringup(unittest.TestCase):
    """Assertions over a live, headless cell. Filled in by the owning streams."""

    def test_cell_launches(self, cell_present):
        if not cell_present:
            self.skipTest(
                "safecollab/launch/cell.launch.py not built yet "
                "(arrives with Stream A); integration harness is wired and ready."
            )
        # Let the full graph reach steady state before the harness tears it
        # down (see _SETTLE_SECONDS). A node that crashes on startup exits
        # during this window and is caught by TestCleanShutdown below.
        time.sleep(_SETTLE_SECONDS)
        # Assert arm_controller + joint_state_broadcaster are both active via
        # the controller_manager list_controllers service.  A 30 s retry window
        # absorbs gz startup jitter — the CM becomes reachable only after gz
        # sim's first world step, so the spawners (ordered: spawn → jsb →
        # arm_controller in cell.launch.py) may not have finished yet when the
        # settle sleep ends.
        self.assertTrue(
            _wait_controllers_active(timeout_s=30.0),
            "arm_controller and joint_state_broadcaster must both report "
            "'active' in `ros2 control list_controllers` within 30 s of "
            "bring-up (spawners are chained: spawn → jsb → arm_controller "
            "in cell.launch.py).",
        )

    def test_safety_topic_rate(self, cell_present):
        if not cell_present:
            self.skipTest("cell not built yet; /safety/scale rate check pending.")
        # safety_monitor publishes /safety/scale on a 20 Hz timer regardless of
        # zone — even in the 'lost' fail-safe state it emits scale=0.0 (see
        # SafetyMonitorNode._tick).  The timer fires on SIM time (use_sim_time),
        # so we measure in sim time: a wall-clock measurement under-reports by
        # the real-time factor when the headless sim runs below 1x. 18 Hz floor
        # = ~10 % margin on the true 20 Hz, independent of RTF.
        from std_msgs.msg import Float32

        rate = _measure_rate_simtime("/safety/scale", Float32)
        self.assertIsNotNone(
            rate,
            "/safety/scale did not publish any messages within the measurement "
            "window; check that safety_monitor started and is running cleanly.",
        )
        self.assertGreaterEqual(
            rate,
            18.0,
            f"/safety/scale sim-time rate {rate:.1f} Hz is below the 18 Hz floor "
            "(safety_monitor targets 20 Hz; measured in sim time, RTF-independent).",
        )

    def test_tf_chain(self, cell_present):
        """Assert the robot TCP chain and human ground-truth TF are published."""
        if not cell_present:
            self.skipTest("cell not built yet; TF chain check pending.")

        # world -> tcp: published by robot_state_publisher from cell.xacro.
        # Full chain: world -> table -> table_top -> base_link ->
        #             link_1 -> link_2 -> link_3 -> link_4 -> link_5 ->
        #             link_6 -> tcp (fixed joint at the tool centre point).
        # robot_state_publisher fills in all revolute joint states from
        # joint_state_broadcaster; all fixed joints are published at startup.
        self.assertTrue(
            _tf_available("world", "tcp", timeout_s=10.0),
            "TF world -> tcp not available within 10 s; "
            "check robot_state_publisher and joint_state_broadcaster.",
        )

        # world -> human_gt: broadcast by human_node at 50 Hz.
        # 'human_gt' is the ground-truth operator frame used for sim diagnostics
        # only — it is NOT the frame consumed by the safety loop.  The safety
        # monitor reads world -> human (perceived), which is published by
        # perception_node when it detects the operator in the camera feed.
        self.assertTrue(
            _tf_available("world", "human_gt", timeout_s=10.0),
            "TF world -> human_gt not available within 10 s; "
            "check human_node is running and broadcasting at 50 Hz.",
        )

        # NOTE: world -> human is intentionally NOT checked here.
        # perception_node broadcasts world -> human only when it detects the
        # operator in the camera image.  The operator currently has no camera-
        # visible model in gz (human_node publishes only world -> human_gt with
        # no visual geometry in the simulation), so perception sees no blob and
        # never broadcasts the human TF.  The safety monitor therefore stays in
        # the 'lost' fail-safe state — which is correct safe behaviour.
        #
        # Enable the check below once BOTH of these tasks have landed:
        #   Task 1 — a camera-visible operator mesh / colour blob is added to gz
        #   Task 4 — perception calibration is tuned against the real mesh
        #
        # self.assertTrue(
        #     _tf_available("world", "human", timeout_s=10.0),
        #     "TF world -> human not available; enable once Task 1 + 4 land.",
        # )


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Every launched process must exit cleanly (no crashes during bring-up)."""

    def test_exit_codes(self, proc_info, cell_present):
        if not cell_present:
            self.skipTest("nothing launched; no exit codes to check.")
        # Allowed shutdown codes, using Python's signal convention (negative =
        # killed by that signal) because that is exactly what launch reports in
        # `info.returncode`:
        #   0            clean exit (nodes that handle SIGINT and return)
        #   -SIGINT (-2) SIGINT at teardown — normal for the nodes and for the
        #                one-shot controller spawners interrupted mid-run
        #   -SIGTERM(-15) gz sim's `ruby ... gz sim` wrapper misses launch's 5 s
        #                SIGINT grace period, so launch escalates to SIGTERM
        # NB: do NOT use launch_testing.asserts.EXIT_SIGINT here — in this
        # version that constant follows the 128+signum shell convention (130),
        # not the -2 that launch actually reports, so it silently excludes the
        # normal SIGINT code. A genuine bring-up crash exits with a non-signal
        # code (e.g. 1) and is still caught.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )
