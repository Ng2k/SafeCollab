"""Operator path: cell geometry, waypoints, and randomised traversal generation.

Pure-Python, no ROS. An :class:`OperatorPath` is a timed waypoint sequence that
always includes a tray reach (FR-11 / AT-6).
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Cell geometry constants
# ---------------------------------------------------------------------------

TRAY_CENTRE: tuple[float, float, float] = (0.35, 0.0, 0.76)
TRAY_HALF_X: float = 0.15
TRAY_HALF_Y: float = 0.20
TRAY_TOP_Z: float = TRAY_CENTRE[2] + 0.02  # 0.78 m

REACH_Z: float = 0.82

STANDING_Z: float = 1.10
APPROACH_Z: float = 0.95

#: Seconds the operator stands clear at the start of each traversal — a
#: contiguous GREEN window so the arm can run its kitting between reaches.
STANDING_DWELL_S: float = 5.0


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------


@dataclass
class Waypoint:
    """A 3-D world-frame position ``(x, y, z)`` with an arrival time ``t`` (s)."""

    x: float
    y: float
    z: float
    t: float

    @property
    def position(self) -> Tuple[float, float, float]:
        """(x, y, z) position tuple."""
        return (self.x, self.y, self.z)


def _lerp(a: float, b: float, alpha: float) -> float:
    """Linear interpolation between ``a`` and ``b`` by factor ``alpha``."""
    return a + alpha * (b - a)


def _interpolate_waypoints(
    waypoints: Sequence[Waypoint],
    t: float,
) -> Tuple[float, float, float]:
    """Linearly interpolate ``(x, y, z)`` between consecutive waypoints at ``t``.

    ``t`` before the first waypoint returns the first position; after the last,
    the last position.
    """
    if not waypoints:
        return (0.0, 0.0, 0.0)

    t = max(0.0, t)

    for i in range(len(waypoints) - 1):
        w0 = waypoints[i]
        w1 = waypoints[i + 1]
        if t <= w1.t:
            dt = w1.t - w0.t
            alpha = (t - w0.t) / dt if dt > 1e-12 else 0.0
            alpha = max(0.0, min(1.0, alpha))
            return (
                _lerp(w0.x, w1.x, alpha),
                _lerp(w0.y, w1.y, alpha),
                _lerp(w0.z, w1.z, alpha),
            )

    return waypoints[-1].position


# ---------------------------------------------------------------------------
# Operator path
# ---------------------------------------------------------------------------


class OperatorPath:
    """A timed waypoint sequence for one operator traversal.

    The path always includes at least one waypoint inside the kitting-tray
    footprint (a "tray reach") to satisfy FR-11. :meth:`generate_random`
    randomises approach side (+y/−y), the tray-reach position, and timing.
    """

    def __init__(self, waypoints: List[Waypoint]) -> None:
        """Store an ordered waypoint list. Raises ``ValueError`` if empty."""
        if not waypoints:
            raise ValueError("OperatorPath requires at least one waypoint")
        self._waypoints: List[Waypoint] = list(waypoints)

    @property
    def waypoints(self) -> List[Waypoint]:
        """Ordered list of waypoints (defensive copy)."""
        return list(self._waypoints)

    @property
    def duration_s(self) -> float:
        """Total duration (arrival time of the last waypoint)."""
        return self._waypoints[-1].t

    def has_tray_reach(self) -> bool:
        """True if any waypoint's (x, y) lies within the tray footprint (z ignored)."""
        cx, cy, _ = TRAY_CENTRE
        return any(
            abs(wp.x - cx) <= TRAY_HALF_X and abs(wp.y - cy) <= TRAY_HALF_Y
            for wp in self._waypoints
        )

    def tray_reach_waypoints(self) -> List[Waypoint]:
        """Waypoints that lie inside the tray footprint."""
        cx, cy, _ = TRAY_CENTRE
        return [
            wp
            for wp in self._waypoints
            if abs(wp.x - cx) <= TRAY_HALF_X and abs(wp.y - cy) <= TRAY_HALF_Y
        ]

    @classmethod
    def generate_random(cls, rng: Optional[random.Random] = None) -> "OperatorPath":
        """Generate a random path that includes a tray reach (FR-11 / AT-6).

        Randomises approach side, reach position, and approach speed. ``rng`` is
        an optional seeded ``random.Random`` for reproducibility.
        """
        if rng is None:
            rng = random.Random()

        # Approach from the +y or −y side of the table.
        side_y = rng.choice([0.65, -0.65])

        # Tray-reach position within the tray footprint.
        reach_x = TRAY_CENTRE[0] + rng.uniform(-TRAY_HALF_X * 0.8, TRAY_HALF_X * 0.8)
        reach_y = rng.uniform(-TRAY_HALF_Y * 0.6, TRAY_HALF_Y * 0.6)

        # ±40 % speed variation on the approach/reach/withdraw phase; the dwell is
        # randomised per cycle too, so the operator's period never phase-locks to
        # the arm's kitting period.
        t_scale = rng.uniform(0.7, 1.4)
        d = STANDING_DWELL_S * rng.uniform(0.5, 1.6)  # randomised GREEN window

        waypoints = [
            # 0 — standing well clear (start of dwell); pushed back to ~1 m so the
            #     operator is unambiguously GREEN while the arm is parked at the tray.
            Waypoint(x=1.0, y=side_y * 1.1, z=STANDING_Z, t=0.0),
            # 1 — still standing back (end of dwell): a contiguous green window.
            Waypoint(x=1.0, y=side_y * 1.1, z=STANDING_Z, t=round(d, 3)),
            # 2 — table edge, leaning toward the tray.
            Waypoint(
                x=0.7, y=side_y * 0.4, z=APPROACH_Z, t=round(d + 2.0 * t_scale, 3)
            ),
            # 3 — hover above tray.
            Waypoint(x=reach_x, y=reach_y, z=APPROACH_Z, t=round(d + 4.0 * t_scale, 3)),
            # 4 — tray reach (hand inside tray) → drives the zone to red.
            Waypoint(x=reach_x, y=reach_y, z=REACH_Z, t=round(d + 5.5 * t_scale, 3)),
            # 4b — brief hold in the shared tray (~2 s); closed-loop pacing already
            #      makes the arm WAIT while the operator is close, so red co-occupancy
            #      is reliable without a long static hold.
            Waypoint(x=reach_x, y=reach_y, z=REACH_Z, t=round(d + 7.5 * t_scale, 3)),
            # 5 — withdraw (hand back above tray).
            Waypoint(x=reach_x, y=reach_y, z=APPROACH_Z, t=round(d + 9.0 * t_scale, 3)),
            # 6 — step back from table.
            Waypoint(
                x=0.7, y=side_y * 0.4, z=APPROACH_Z, t=round(d + 11.0 * t_scale, 3)
            ),
            # 7 — return to standing (well clear again).
            Waypoint(
                x=1.0, y=side_y * 1.1, z=STANDING_Z, t=round(d + 13.0 * t_scale, 3)
            ),
        ]

        return cls(waypoints)
