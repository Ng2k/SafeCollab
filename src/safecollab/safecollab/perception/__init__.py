"""Perception package — classical-CV operator detection for the SSM loop.

The perception responsibility is split into cohesive, independently-testable
modules (no ROS in any of them except :mod:`node`):

* :mod:`safecollab.perception.geometry`     — pinhole back-projection + camera→world
* :mod:`safecollab.perception.detection`    — the swappable CV blob detector
* :mod:`safecollab.perception.uncertainty`  — σ estimation (feeds Z_d in the risk model)
* :mod:`safecollab.perception.state`        — the loss-timeout state machine
* :mod:`safecollab.perception.node`         — the ROS 2 adapter (PerceptionNode + main)

This package re-exports the pure public API so callers (and tests) can import
from ``safecollab.perception`` directly. The ROS node lives in :mod:`node` and is
reached via the ``perception_node`` console-script entry point.
"""

from safecollab.perception.detection import detect_human
from safecollab.perception.geometry import (
    back_project,
    cam_to_world_transform,
    project_pixel_to_plane,
    transform_point,
    _derive_intrinsics,
)
from safecollab.perception.state import PerceptionLogic
from safecollab.perception.uncertainty import estimate_uncertainty

__all__ = [
    "PerceptionLogic",
    "back_project",
    "cam_to_world_transform",
    "detect_human",
    "estimate_uncertainty",
    "project_pixel_to_plane",
    "transform_point",
    "_derive_intrinsics",
]
