# SafeCollab — shared helpers for the scenario (launch_testing) harnesses.
#
# Shared by the AT-1..AT-5 (test_at1_at5) and the P4 robustness (test_robustness)
# harnesses so they use one implementation instead of duplicating the plumbing.
# This module is NOT a test file (leading underscore; no test_* names) so neither
# pytest nor launch_test collects it directly.
#
# OWNED BY: Stream G / the scenario harnesses.

import os
import subprocess
import time


# ---------------------------------------------------------------------------
# Launch file discovery
# ---------------------------------------------------------------------------


def locate_cell_launch():
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
# Topic recording — single in-process rclpy subscriber
# ---------------------------------------------------------------------------


def record_safety_topics(
    duration_s: float,
    *,
    zone: bool = False,
    scale: bool = False,
    min_distance: bool = False,
    arm_traj: bool = False,
    marker: bool = False,
) -> dict:
    """Record cell topics for *duration_s* via one in-process rclpy node.

    Returns ``{"zones": [...], "scales": [...], "min_dists": [...],
    "arm_traj_count": int, "markers": [...]}``; only the requested entries are
    populated (``arm_traj_count`` is the number of
    ``/arm_controller/joint_trajectory`` messages seen). ``markers`` holds the
    DISTINCT ``(id, type, (r, g, b), text)`` signatures seen on
    ``/viz/safety_marker`` (deduplicated to bound memory over the window).

    Why rclpy instead of ``ros2 topic echo``: the CLI echo of the reliable +
    TRANSIENT_LOCAL ``/safety/zone`` can lose the DDS discovery race under load
    and capture nothing — an empty ``/safety/zone`` while ``/safety/scale`` on the
    same tick is fine (the exact flake this replaces). The same race could leave
    the ``ros2 topic echo --once`` arm-trajectory presence check empty. A single
    in-process subscriber with QoS matched to each topic's contract is
    deterministic and avoids spawning concurrent echo subprocesses.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Float32, String
    from trajectory_msgs.msg import JointTrajectory

    if not rclpy.ok():
        rclpy.init()
    node = Node("ssm_topic_recorder")
    out: dict = {
        "zones": [],
        "scales": [],
        "min_dists": [],
        "arm_traj_count": 0,
        "markers": [],
    }

    if scale:
        # /safety/scale — reliable (§3 contract).
        node.create_subscription(
            Float32, "/safety/scale", lambda m: out["scales"].append(m.data), 10
        )
    if zone:
        # /safety/zone — reliable + transient_local (§3 contract). Matching the
        # publisher's durability is what makes the capture reliable here.
        zone_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            String, "/safety/zone", lambda m: out["zones"].append(m.data), zone_qos
        )
    if min_distance:
        # /safety/min_distance — best-effort (§3 contract).
        be_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        node.create_subscription(
            Float32,
            "/safety/min_distance",
            lambda m: out["min_dists"].append(m.data),
            be_qos,
        )
    if arm_traj:
        # /arm_controller/joint_trajectory — motion_node publishes reliable,
        # KEEP_LAST, depth 10 (see motion_node). We only need the count (presence).
        def _bump(_m):
            out["arm_traj_count"] += 1

        node.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory", _bump, 10
        )
    if marker:
        # /viz/safety_marker — safety_monitor publishes best-effort, depth 10.
        # Record DISTINCT (id, type, rgb, text) signatures: the sphere (id 0)
        # and the zone text label (id 1) per zone, so the list stays small.
        from visualization_msgs.msg import Marker

        _seen: set = set()

        def _on_marker(m):
            sig = (
                m.id,
                m.type,
                (round(m.color.r, 3), round(m.color.g, 3), round(m.color.b, 3)),
                m.text,
            )
            if sig not in _seen:
                _seen.add(sig)
                out["markers"].append(sig)

        be_marker_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        node.create_subscription(
            Marker, "/viz/safety_marker", _on_marker, be_marker_qos
        )

    deadline = time.monotonic() + duration_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
    # Note: deliberately not calling rclpy.shutdown() — the context is left up so
    # repeated calls within one test work; the test process exit cleans it up.
    return out


# ---------------------------------------------------------------------------
# Zone-stream analysis helpers
# ---------------------------------------------------------------------------


def is_subsequence(seq: list, pattern: list) -> bool:
    """True if *pattern* occurs as an ordered (not necessarily contiguous) subsequence.

    Used by AT-2/AT-4 instead of comparing global first-occurrence indices: the
    cell cycles continuously (operator approach↔retreat, robot kitting loop), so
    the recording window can start and end at any phase.  What we must prove is
    that a correctly-ordered escalation (green→yellow→red) and de-escalation
    actually occur *somewhere* in the window, not that the very first sample is
    green.  Scans *seq* once.
    """
    it = iter(seq)
    return all(any(x == p for x in it) for p in pattern)


def compress(seq: list) -> list:
    """Collapse consecutive duplicates: ['a','a','b','a'] → ['a','b','a'].

    Turns a long zone stream into its transition sequence for readable
    diagnostics in assertion messages.
    """
    out: list = []
    for x in seq:
        if not out or out[-1] != x:
            out.append(x)
    return out


def count_escalations(transitions: list) -> int:
    """Count completed green→…→yellow→…→red escalations in a transition stream.

    Walks the compressed zone-transition list once, tracking a small state
    machine: after seeing 'green' then 'yellow', the next 'red' completes one
    escalation cycle and the counter resets.  Used by AT-6 to prove the SSM
    behaviour recurs across several distinct (randomly generated) operator paths,
    not just once — i.e. it generalises with no path-specific tuning.
    """
    count = 0
    saw_green = False
    saw_yellow = False
    for z in transitions:
        if z == "green":
            saw_green = True
        elif z == "yellow":
            if saw_green:
                saw_yellow = True
        elif z == "red":
            if saw_green and saw_yellow:
                count += 1
                saw_green = False
                saw_yellow = False
    return count


def count_recoveries(
    scales: list, stop_below: float = 0.05, resume_above: float = 0.5
) -> int:
    """Count protective-stop → resume cycles in a /safety/scale stream.

    A recovery is a transition from a stopped sample (``scale < stop_below``) to a
    later resumed sample (``scale > resume_above``).  Walks the stream once; used
    by AT-6 to prove the clean-resume behaviour also recurs across paths.
    """
    count = 0
    in_stop = False
    for s in scales:
        if s < stop_below:
            in_stop = True
        elif s > resume_above and in_stop:
            count += 1
            in_stop = False
    return count


# ---------------------------------------------------------------------------
# Controller readiness
# ---------------------------------------------------------------------------


def wait_controllers_active(timeout_s: float = 60.0) -> bool:
    """Poll ``ros2 control list_controllers`` until both arm controllers are active.

    Retries every 1 s within *timeout_s*.  The controller_manager service only
    appears after gz sim's first world step, so the spawners may not have finished
    at settle time.
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
