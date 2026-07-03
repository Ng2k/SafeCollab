"""Operator model: advances along an :class:`OperatorPath` (pure-Python, no ROS)."""

from __future__ import annotations

import math
from typing import Tuple

from safecollab.human.path import OperatorPath, _interpolate_waypoints


class OperatorModel:
    """Advances a hand position along an ``OperatorPath``.

    ``position_at(t)`` interpolates the world-frame ``(x, y, z)`` at time ``t``.
    When the path ends, :meth:`is_complete` returns ``True`` and the caller
    (HumanNode) switches to a fresh path. No ROS imports; fully unit-testable.
    """

    def __init__(self, path: OperatorPath) -> None:
        self._path = path

    @property
    def path(self) -> OperatorPath:
        """The current operator path."""
        return self._path

    @path.setter
    def path(self, new_path: OperatorPath) -> None:
        """Replace the current path (e.g. when a cycle ends)."""
        self._path = new_path

    def position_at(self, t: float) -> Tuple[float, float, float]:
        """World-frame ``(x, y, z)`` at time ``t`` (clamped; does not loop)."""
        return _interpolate_waypoints(self._path.waypoints, t)

    def is_complete(self, t: float) -> bool:
        """True once the path has ended at time ``t``."""
        return t >= self._path.duration_s

    @staticmethod
    def distance_to_point(
        pos: Tuple[float, float, float],
        target: Tuple[float, float, float],
    ) -> float:
        """Euclidean distance between two 3-D world-frame points."""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos, target)))
