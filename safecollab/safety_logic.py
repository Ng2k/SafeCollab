"""Zone classifier for Speed-and-Separation Monitoring (ISO/TS 15066).

Pure function, no ROS. Maps a measured minimum separation ``d`` to a safety
zone and a speed scale in ``[0.0, 1.0]``. Thresholds are supplied by the caller
(computed from ``config/risk.yaml`` at start-up) -- nothing is hard-coded here.
"""


def classify(d, *, d_red, d_yellow, s_min):
    """Classify a separation distance into a zone and a speed scale.

    Args:
        d: measured minimum separation in metres, or ``None`` when perception
            is lost/stale.
        d_red: protective-stop threshold (inclusive); ``d <= d_red`` -> red.
        d_yellow: full-speed threshold (exclusive); ``d >= d_yellow`` -> green.
        s_min: minimum non-zero speed scale at the red edge of the ramp.

    Returns:
        ``(zone, scale)`` where ``zone`` is one of ``"green" | "yellow" |
        "red" | "lost"`` and ``scale`` is in ``[0.0, 1.0]``.

    Boundary semantics: yellow edge EXCLUSIVE (``d == d_yellow`` -> green),
    red edge INCLUSIVE (``d == d_red`` -> red), ``d is None`` -> fail-safe stop.
    """
    if d is None:
        return "lost", 0.0          # fail-safe (FR-9)
    if d >= d_yellow:
        return "green", 1.0
    if d <= d_red:
        return "red", 0.0          # red edge INCLUSIVE (d == d_red -> red)
    s = s_min + (1 - s_min) * (d - d_red) / (d_yellow - d_red)
    return "yellow", s
