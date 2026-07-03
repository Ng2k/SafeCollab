"""Trajectory re-timing — the "speed knob" (pure math, no ROS).

Stretches each point's ``time_from_start`` by ``1/scale``; isolated here so it can
be unit-tested without the ROS node.
"""

from __future__ import annotations


def retime(times, scale):
    """Rescale ``time_from_start`` values by a speed ``scale``.

    Slower speed → same waypoints reached later, so each time is stretched by
    ``1 / scale``. Returns ``None`` when ``scale == 0.0`` (protective stop: hold,
    emit nothing). Raises ``ValueError`` if ``scale`` is outside ``[0.0, 1.0]``.
    """
    if not 0.0 <= scale <= 1.0:
        raise ValueError(f"scale must be in [0.0, 1.0], got {scale!r}")
    if scale == 0.0:
        return None  # protective stop — guard div-by-zero
    return [t / scale for t in times]
