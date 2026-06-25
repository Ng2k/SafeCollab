"""Trajectory re-timing -- the "speed knob" (pure math, no ROS).

``motion_node`` fuses a nominal trajectory with the safety scale by stretching
each point's ``time_from_start``. This module holds only that math so it can be
unit-tested in isolation from the ROS node.
"""


def retime(times, scale):
    """Rescale a sequence of ``time_from_start`` values by a speed ``scale``.

    Slower speed means the same waypoints are reached later, so each time is
    stretched by ``1 / scale``.

    Args:
        times: ordered ``time_from_start`` values (seconds).
        scale: speed scale in ``[0.0, 1.0]`` (typically from ``/safety/scale``).

    Returns:
        The re-timed list, or ``None`` when ``scale == 0.0`` (protective stop:
        hold position, emit no motion).

    Raises:
        ValueError: if ``scale`` is outside ``[0.0, 1.0]``.
    """
    if not 0.0 <= scale <= 1.0:
        raise ValueError(f"scale must be in [0.0, 1.0], got {scale!r}")
    if scale == 0.0:
        return None  # protective stop -- guard div-by-zero
    return [t / scale for t in times]
