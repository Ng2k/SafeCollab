"""ISO/TS 15066 protective-separation model and zone-threshold derivation.

Pure functions, no ROS. The model inputs live in ``config/risk.yaml`` and the
zone thresholds are computed at start-up -- no thresholds are hard-coded here
(AGENTS.md ground rule 5).
"""
from pathlib import Path
from types import SimpleNamespace

import yaml


def protective_distance(v_h, t_r, t_s, v_r, s_s, c, z_d, z_r):
    """ISO/TS 15066 protective separation ``S_p``.

    ``S_p = S_H + S_R + S_S + C + Z_d + Z_r`` where ``S_H = v_h*(t_r+t_s)`` is
    the human contribution and ``S_R = v_r*t_r`` the robot contribution during
    the reaction time.
    """
    s_h = v_h * (t_r + t_s)
    s_r = v_r * t_r
    return s_h + s_r + s_s + c + z_d + z_r            # ISO/TS 15066 S_p


def thresholds(cfg, z_d):
    """Derive ``(d_red, d_yellow)`` from the risk config and a live ``z_d``.

    ``z_d`` (operator position uncertainty) comes from perception, so the
    thresholds widen as the perceived operator position becomes less certain.
    The two scenarios share the cell/sensor properties and differ only in the
    robot speed ``v_r`` and stop time ``t_s`` (which shrink together).
    """
    d_yellow = protective_distance(v_r=cfg.full_speed.v_r, t_s=cfg.full_speed.t_s,
                                   v_h=cfg.v_h, t_r=cfg.t_r, s_s=cfg.s_s, c=cfg.c,
                                   z_d=z_d, z_r=cfg.z_r)
    d_red = protective_distance(v_r=cfg.reduced.v_r, t_s=cfg.reduced.t_s,
                                v_h=cfg.v_h, t_r=cfg.t_r, s_s=cfg.s_s, c=cfg.c,
                                z_d=z_d, z_r=cfg.z_r)
    return round(d_red, 2), round(d_yellow, 2)


def load_config(path):
    """Load the ``risk:`` block of ``config/risk.yaml`` into a namespace.

    Returns an object exposing the shared inputs (``v_h``, ``t_r``, ``s_s``,
    ``c``, ``z_r``) plus ``full_speed`` / ``reduced`` sub-namespaces, each with
    its own ``v_r`` and ``t_s``, as consumed by :func:`thresholds`.
    """
    with open(Path(path), "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    risk = data["risk"]
    return SimpleNamespace(
        v_h=risk["v_h"],
        t_r=risk["t_r"],
        s_s=risk["s_s"],
        c=risk["c"],
        z_r=risk["z_r"],
        full_speed=SimpleNamespace(**risk["full_speed"]),
        reduced=SimpleNamespace(**risk["reduced"]),
    )
