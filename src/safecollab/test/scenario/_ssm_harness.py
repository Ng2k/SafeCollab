# SafeCollab — shared helpers for the scenario (launch_testing) harnesses.
#
# Extracted verbatim from the AT-1..AT-5 harness so the AT-1..AT-5 (test_at1_at5)
# and the P4 robustness (test_robustness) harnesses share one implementation
# instead of duplicating ~150 lines of topic-collection plumbing. This module is
# NOT a test file (leading underscore; no test_* names) so neither pytest nor
# launch_test collects it directly.
#
# OWNED BY: Stream G / the scenario harnesses.

import os
import queue
import subprocess
import threading
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
# Topic-collection helpers
# ---------------------------------------------------------------------------


def collect_float32_values(
    topic: str, duration_s: float, best_effort: bool = False
) -> list:
    """Collect std_msgs/Float32 data values from *topic* for *duration_s* seconds.

    Spawns ``ros2 topic echo topic`` and reads its output via a daemon thread so
    that ``readline()`` never blocks the outer deadline check.  Returns a list of
    floats.

    *best_effort* makes the echo subscriber request BEST_EFFORT reliability,
    required for topics the §3 contract publishes best-effort (e.g.
    ``/safety/min_distance``): a default RELIABLE echo subscriber is QoS-
    incompatible with a BEST_EFFORT publisher and would capture nothing.
    """
    cmd = ["ros2", "topic", "echo", topic]
    if best_effort:
        cmd += ["--qos-reliability", "best_effort"]
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


def collect_string_values(
    topic: str,
    duration_s: float,
    valid_values: "set | None" = None,
    transient_local: bool = False,
) -> list:
    """Collect std_msgs/String data values from *topic* for *duration_s* seconds.

    If *valid_values* is provided only strings in that set are kept (filters YAML
    boilerplate).  Surrounding quotes are stripped for robustness across ROS echo
    formats.

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


def topic_has_messages(topic: str, window_s: float = 15.0) -> bool:
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
