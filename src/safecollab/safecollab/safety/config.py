"""Loader for the ``safety:`` block of ``config/safety.yaml`` (ground rule 5)."""

from pathlib import Path
from types import SimpleNamespace

import yaml


def load_safety_config(path) -> SimpleNamespace:
    """Load ``config/safety.yaml`` into a namespace.

    Attributes: ``s_min`` (min yellow-band speed scale), ``loss_timeout``
    (stale-TF grace, s), ``robot_frames`` (TF frames for the min-distance sweep),
    ``marker_radius`` (RViz sphere radius, m), ``marker_label_offset`` (gap above
    the sphere for the zone label, m), ``marker_label_height`` (label glyph
    height, m).
    """
    with open(Path(path), "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    safety = data["safety"]
    return SimpleNamespace(
        s_min=float(safety["s_min"]),
        loss_timeout=float(safety["loss_timeout"]),
        robot_frames=list(
            safety.get("robot_frames", ["tool0", "wrist_3_link", "forearm_link"])
        ),
        marker_radius=float(safety.get("marker_radius", 0.15)),
        marker_label_offset=float(safety.get("marker_label_offset", 0.25)),
        marker_label_height=float(safety.get("marker_label_height", 0.20)),
    )
