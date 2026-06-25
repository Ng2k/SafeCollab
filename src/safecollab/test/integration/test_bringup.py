# SafeCollab — headless integration bring-up harness (launch_testing).
#
# OWNED BY: Stream G (CI/CD). This is the *scaffolding* only — it wires up a
# headless launch_testing harness and the assertion slots the other streams
# fill in as their nodes land:
#
#   * controllers active   -> `ros2 control list_controllers` (Stream A/D)
#   * TF chain resolves     -> world->...->tcp and world->human (Stream A/C/F)
#   * topic rates           -> /safety/scale >= 20 Hz (Stream F)
#   * command path re-timed -> /arm_controller/joint_trajectory (Stream D)
#
# Until `safecollab/launch/cell.launch.py` exists the harness brings nothing up
# and the assertion tests self-skip, so the integration stage stays green while
# the pipeline is wired and ready (AGENTS.md §6 Stream G, §9 integration layer).

import os
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


class TestHeadlessBringup(unittest.TestCase):
    """Assertions over a live, headless cell. Filled in by the owning streams."""

    def test_cell_launches(self, cell_present):
        if not cell_present:
            self.skipTest(
                "safecollab/launch/cell.launch.py not built yet "
                "(arrives with Stream A); integration harness is wired and ready."
            )
        # TODO(Stream A/D): assert `arm_controller` + `joint_state_broadcaster`
        # report active via the controller_manager list_controllers service.

    def test_safety_topic_rate(self, cell_present):
        if not cell_present:
            self.skipTest("cell not built yet; /safety/scale rate check pending.")
        # TODO(Stream F): assert /safety/scale publishes at >= 20 Hz.


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Every launched process must exit cleanly (no crashes during bring-up)."""

    def test_exit_codes(self, proc_info, cell_present):
        if not cell_present:
            self.skipTest("nothing launched; no exit codes to check.")
        launch_testing.asserts.assertExitCodes(proc_info)
