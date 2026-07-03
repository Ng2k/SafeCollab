"""Pure-Python safety-monitor logic (unit-testable without a live ROS graph).

Computes minimum separation between robot frames and the perceived human,
classifies the zone via :func:`safecollab.safety.zone.classify`, and returns
zone, speed scale, distance, and RViz marker params. No ROS dependencies
(ground rule 3).
"""

from __future__ import annotations

import math
from types import SimpleNamespace

from safecollab.safety.marker import _marker_params
from safecollab.safety.risk import thresholds
from safecollab.safety.zone import classify


class SafetyMonitorLogic:
    """Min-distance → risk zone → speed scale (+ marker params), free of ROS.

    Typical use inside ``SafetyMonitorNode._tick()``::

        zone, scale, dist, marker_p = self._logic.compute(
            robot_frames_xyz=[(x, y, z), ...],
            human_xyz_or_none=(hx, hy, hz),   # None when TF is stale/missing
            uncertainty=sigma,
        )
    """

    def __init__(
        self,
        risk_cfg: SimpleNamespace,
        safety_cfg: SimpleNamespace,
    ) -> None:
        """Store pre-loaded config namespaces (``risk.yaml`` / ``safety.yaml``)."""
        self._risk_cfg = risk_cfg
        self._s_min = safety_cfg.s_min
        self._loss_timeout = safety_cfg.loss_timeout
        self._marker_radius = safety_cfg.marker_radius
        # Label geometry: default to load_safety_config's values so an in-memory
        # cfg without these fields still constructs.
        self._marker_label_offset = getattr(safety_cfg, "marker_label_offset", 0.25)
        self._marker_label_height = getattr(safety_cfg, "marker_label_height", 0.20)

    # ------------------------------------------------------------------
    # Read-only properties consumed by the ROS wrapper
    # ------------------------------------------------------------------

    @property
    def loss_timeout(self) -> float:
        """Stale-TF timeout (s); the wrapper passes ``human_xyz=None`` past it."""
        return self._loss_timeout

    @property
    def marker_radius(self) -> float:
        """Radius of the RViz safety-zone sphere (m)."""
        return self._marker_radius

    @property
    def marker_label_offset(self) -> float:
        """Gap (m) between the sphere top and the floating zone label."""
        return self._marker_label_offset

    @property
    def marker_label_height(self) -> float:
        """RViz TEXT_VIEW_FACING glyph height (m) for the zone label."""
        return self._marker_label_height

    # ------------------------------------------------------------------
    # Static helper
    # ------------------------------------------------------------------

    @staticmethod
    def _min_distance(
        robot_frames_xyz: list[tuple[float, float, float]],
        human_xyz: tuple[float, float, float],
    ) -> float:
        """Euclidean minimum distance (m) from any robot frame to the human."""
        hx, hy, hz = human_xyz
        # sqrt is monotonic: pick the nearest frame by squared distance, root once.
        min_sq = math.inf
        for rx, ry, rz in robot_frames_xyz:
            d_sq = (rx - hx) ** 2 + (ry - hy) ** 2 + (rz - hz) ** 2
            if d_sq < min_sq:
                min_sq = d_sq
        return math.sqrt(min_sq)

    # ------------------------------------------------------------------
    # Main computation (hot path, called every tick)
    # ------------------------------------------------------------------

    def compute(
        self,
        robot_frames_xyz: list[tuple[float, float, float]],
        human_xyz_or_none: tuple[float, float, float] | None,
        uncertainty: float,
    ) -> tuple[str, float, float | None, dict]:
        """Return ``(zone, scale, min_dist_or_none, marker_params)``.

        Args:
            robot_frames_xyz: world-frame positions of TCP/wrist/elbow (or
                whichever frames are available). Empty -> fail-safe.
            human_xyz_or_none: perceived human position, or ``None`` when the
                ``world → human`` TF is missing/stale beyond ``loss_timeout``.
            uncertainty: perception σ (m); widens the thresholds via
                :func:`safecollab.safety.risk.thresholds`.

        Fail-safe: if ``human_xyz_or_none`` is ``None`` or ``robot_frames_xyz``
        is empty, ``d`` is ``None`` and ``classify()`` returns ``("lost", 0.0)``
        — a protective stop (FR-9).
        """
        if human_xyz_or_none is None or not robot_frames_xyz:
            d: float | None = None  # fail-safe -> classify() returns ("lost", 0.0)
        else:
            d = self._min_distance(robot_frames_xyz, human_xyz_or_none)

        d_red, d_yellow = thresholds(self._risk_cfg, uncertainty)
        zone, scale = classify(d, d_red=d_red, d_yellow=d_yellow, s_min=self._s_min)
        marker = _marker_params(
            zone, human_xyz_or_none, self._marker_radius, self._marker_label_offset
        )
        return zone, scale, d, marker
