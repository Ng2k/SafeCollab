"""Console safety HUD — a compact terminal readout of the SSM state.

Pure formatting (:mod:`format`) is separated from the ROS view (:mod:`node`).
The package re-exports the pure API for the unit tests.
"""

from safecollab.hud.format import (
    build_colour_map,
    format_status,
    load_hud_config,
    sgr,
)

__all__ = ["build_colour_map", "format_status", "load_hud_config", "sgr"]
