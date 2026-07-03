"""Pure HUD line formatting — no ROS, unit-tested in ``test_hud_node.py``.

Renders ONE aligned, colour-coded status line for the 5-second legibility check.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Optional

import yaml

#: ANSI reset sequence (closes any colour opened for the zone token).
_RESET = "\033[0m"

#: Zone-word field width ("YELLOW" is the widest at 6) so every line aligns.
_ZONE_FIELD_WIDTH = 6


def sgr(code: int) -> str:
    """ANSI Select-Graphic-Rendition escape for *code* (e.g. 31 → red)."""
    return f"\033[{int(code)}m"


def build_colour_map(sgr_codes: dict) -> dict:
    """Map ``zone → ANSI-escape`` from a ``zone → SGR-code`` dict.

    Config stays plain integers (31, 33, …); the escapes are built here so no raw
    escape bytes live in YAML.
    """
    return {zone: sgr(code) for zone, code in sgr_codes.items()}


def format_status(
    zone: str,
    scale: float,
    min_distance: Optional[float],
    *,
    colours: Optional[dict] = None,
) -> str:
    """Format one HUD status line from the live safety state.

    Args:
        zone: ``green|yellow|red|lost``; shown upper-cased, fixed width.
        scale: speed scale ``[0, 1]``; shown as a clamped percentage.
        min_distance: metres, or ``None``. Shown as ``--`` when ``None`` or when
            ``zone == "lost"`` (FR-9: a stale last position is never shown as valid).
        colours: optional ``zone → ANSI-escape`` map; wraps the zone token when set.

    Returns:
        e.g. ``"SSM | RED    | speed   0% | min-dist 0.38 m"``.
    """
    label = f"{zone.upper():<{_ZONE_FIELD_WIDTH}}"
    if colours:
        code = colours.get(zone)
        if code:
            label = f"{code}{label}{_RESET}"

    pct = max(0, min(100, round(scale * 100)))

    if min_distance is None or zone == "lost":
        dist = "  --  "
    else:
        dist = f"{min_distance:.2f} m"

    return f"SSM | {label} | speed {pct:>3d}% | min-dist {dist}"


def load_hud_config(path) -> SimpleNamespace:
    """Load the ``hud:`` block of ``config/hud.yaml``.

    Returns a namespace with ``refresh_hz`` (redraw rate), ``use_colour``, and
    ``colours`` (``zone → SGR-code`` ints; see :func:`build_colour_map`).
    """
    with open(Path(path), "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    hud = data["hud"]
    return SimpleNamespace(
        refresh_hz=float(hud.get("refresh_hz", 5.0)),
        use_colour=bool(hud.get("use_colour", True)),
        colours=dict(hud.get("colours", {})),
    )
