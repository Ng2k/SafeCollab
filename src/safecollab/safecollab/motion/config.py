"""Motion node config loader — the ``motion:`` block of ``config/motion.yaml``."""

from __future__ import annotations

from types import SimpleNamespace

import yaml

from safecollab.motion.logic import _DEFAULT_HOLD_TIME_S, _DEFAULT_REPUBLISH_EPSILON


def load_motion_config(path) -> SimpleNamespace:
    """Load ``config/motion.yaml`` into a namespace.

    Missing keys fall back to the MotionLogic defaults so a partial file never
    crashes the node. Returns ``republish_scale_epsilon`` and ``hold_time_s``.
    """
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    motion = data.get("motion") or {}
    return SimpleNamespace(
        republish_scale_epsilon=float(
            motion.get("republish_scale_epsilon", _DEFAULT_REPUBLISH_EPSILON)
        ),
        hold_time_s=float(motion.get("hold_time_s", _DEFAULT_HOLD_TIME_S)),
    )
