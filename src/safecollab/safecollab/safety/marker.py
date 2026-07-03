"""RViz marker parameters for the safety zone (viz concern, kept out of Logic).

The single zone->colour source of truth lives here; both the sphere and its
floating text label reuse it.
"""

from __future__ import annotations

#: RGBA per zone. Alpha 0.8 for active zones; 0.4 for ``lost`` (greyed-out to
#: signal perception failure).
_ZONE_RGBA: dict[str, tuple[float, float, float, float]] = {
    "green": (0.0, 1.0, 0.0, 0.8),
    "yellow": (1.0, 1.0, 0.0, 0.8),
    "red": (1.0, 0.0, 0.0, 0.8),
    "lost": (0.5, 0.5, 0.5, 0.4),
}


def _marker_params(
    zone: str,
    human_xyz_or_none: tuple[float, float, float] | None,
    radius: float,
    label_offset: float = 0.0,
) -> dict:
    """Build RViz marker parameters for ``zone`` as a plain ``dict``.

    The ROS wrapper turns this into a ``visualization_msgs/Marker``. Keys:
    ``r,g,b,a`` (RGBA), ``x,y,z`` (sphere centre = human position, or origin when
    ``None``), ``radius``, ``zone``, ``label`` (upper-case text sharing the
    sphere colour), ``label_z`` (sphere top ``z+radius`` plus ``label_offset``).
    """
    r, g, b, a = _ZONE_RGBA.get(zone, (0.5, 0.5, 0.5, 0.4))
    if human_xyz_or_none is not None:
        x, y, z = human_xyz_or_none
    else:
        x, y, z = 0.0, 0.0, 0.0
    return {
        "r": r,
        "g": g,
        "b": b,
        "a": a,
        "x": x,
        "y": y,
        "z": z,
        "radius": radius,
        "zone": zone,
        "label": zone.upper(),  # reuses the SAME (r, g, b)
        "label_z": z + radius + label_offset,
    }
