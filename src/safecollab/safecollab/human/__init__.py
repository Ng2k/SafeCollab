"""Simulated operator (human) — the ground-truth operator model.

Pure ``path`` (geometry + waypoints + traversal generation) and ``model`` are
separated from the ROS ``node``. Broadcasts ``world → human_gt`` only; safety
consumes the *perceived* ``world → human`` TF from perception_node.
"""

from safecollab.human.model import OperatorModel
from safecollab.human.path import (
    APPROACH_Z,
    REACH_Z,
    STANDING_DWELL_S,
    STANDING_Z,
    TRAY_CENTRE,
    TRAY_HALF_X,
    TRAY_HALF_Y,
    TRAY_TOP_Z,
    OperatorPath,
    Waypoint,
    _interpolate_waypoints,
)

__all__ = [
    "APPROACH_Z",
    "OperatorModel",
    "OperatorPath",
    "REACH_Z",
    "STANDING_DWELL_S",
    "STANDING_Z",
    "TRAY_CENTRE",
    "TRAY_HALF_X",
    "TRAY_HALF_Y",
    "TRAY_TOP_Z",
    "Waypoint",
    "_interpolate_waypoints",
]
