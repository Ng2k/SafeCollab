"""Console safety HUD — a compact terminal readout of the SSM state (P5 sprint 2).

Subscribes the three §3 safety topics and prints ONE aligned, colour-coded
status line that refreshes in place, so the green/yellow/red/lost state is
legible at a glance — the "5-second legibility check" (AGENTS.md §10 P5). It is
a *view*: it never touches the safety loop and is launched only on demand
(``hud:=true``), so CI/headless and the scenario harness are unaffected.

The pure formatting logic (``format_status`` + the colour helpers) is separated
from the ROS node and unit-tested in ``test_hud_node.py``; ``HudNode`` is a thin
I/O shell over it (pragma:no-cover), mirroring the other nodes' wrappers.

Reads:
    ``/safety/zone``         (std_msgs/String)  — green|yellow|red|lost
    ``/safety/scale``        (std_msgs/Float32) — 0.0–1.0
    ``/safety/min_distance`` (std_msgs/Float32) — metres (0.0 while 'lost')
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Optional

import yaml

# ---------------------------------------------------------------------------
# Pure formatting logic (unit-testable without ROS)
# ---------------------------------------------------------------------------

#: ANSI reset sequence (closes any colour opened for the zone token).
_RESET = "\033[0m"

#: Field width for the zone word — the widest is "YELLOW" (6 chars), so every
#: line aligns regardless of zone (steady columns aid the 5-second read).
_ZONE_FIELD_WIDTH = 6


def sgr(code: int) -> str:
    """Return the ANSI Select-Graphic-Rendition escape for *code* (e.g. 31 → red)."""
    return f"\033[{int(code)}m"


def build_colour_map(sgr_codes: dict) -> dict:
    """Build a ``zone → ANSI-escape`` map from a ``zone → SGR-code`` dict.

    Keeping the config as plain integers (31, 33, …) avoids embedding raw escape
    bytes in YAML; the escapes are constructed here.
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
        zone:         Current zone string (``green|yellow|red|lost``); shown
                      upper-cased and left-padded to a fixed width.
        scale:        Speed scale in ``[0, 1]``; shown as a clamped percentage.
        min_distance: Minimum separation in metres, or ``None``. Shown as ``--``
                      when ``None`` **or** when ``zone == "lost"`` (FR-9: a stale
                      last position is never displayed as if it were valid).
        colours:      Optional ``zone → ANSI-escape`` map. When given and the zone
                      has an entry, the zone token is wrapped in that colour + a
                      reset; otherwise the line is plain text.

    Returns:
        A single line, e.g. ``"SSM | RED    | speed   0% | min-dist 0.38 m"``.
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
    """Load the ``hud:`` block of ``config/hud.yaml`` into a namespace.

    Returns a ``SimpleNamespace`` with:
        ``refresh_hz`` — HUD redraw rate (Hz).
        ``use_colour`` — whether to colour the zone token with ANSI codes.
        ``colours``    — ``zone → SGR-code`` dict (ints; see :func:`build_colour_map`).
    """
    with open(Path(path), "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    hud = data["hud"]
    return SimpleNamespace(
        refresh_hz=float(hud.get("refresh_hz", 5.0)),
        use_colour=bool(hud.get("use_colour", True)),
        colours=dict(hud.get("colours", {})),
    )


# ---------------------------------------------------------------------------
# ROS 2 node wrapper (requires rclpy — not available in the pure-Python venv)
# ---------------------------------------------------------------------------

try:
    import rclpy  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from rclpy.qos import (  # pragma: no cover
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Float32, String  # pragma: no cover

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]


class HudNode(Node):  # type: ignore[misc]  # pragma: no cover
    """Thin ROS node: cache the latest safety state, redraw the HUD on a timer."""

    def __init__(self) -> None:
        super().__init__("hud_node")  # type: ignore[call-arg]

        _here = Path(__file__).resolve().parent
        cfg = load_hud_config(_here.parent / "config" / "hud.yaml")
        self._colours = build_colour_map(cfg.colours) if cfg.use_colour else None

        # Fail-safe defaults until the first messages arrive.
        self._zone: str = "lost"
        self._scale: float = 0.0
        self._min_distance: Optional[float] = None

        # QoS matched to each topic's §3 contract (same as safety_monitor pubs).
        _reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        _transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        _best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(String, "/safety/zone", self._on_zone, _transient)
        self.create_subscription(Float32, "/safety/scale", self._on_scale, _reliable)
        self.create_subscription(
            Float32, "/safety/min_distance", self._on_distance, _best_effort
        )

        self.create_timer(1.0 / cfg.refresh_hz, self._render)
        self.get_logger().info(
            f"[hud_node] console HUD at {cfg.refresh_hz:.0f} Hz "
            f"(colour={'on' if self._colours else 'off'})"
        )

    def _on_zone(self, msg: "String") -> None:
        self._zone = msg.data

    def _on_scale(self, msg: "Float32") -> None:
        self._scale = float(msg.data)

    def _on_distance(self, msg: "Float32") -> None:
        self._min_distance = float(msg.data)

    def _render(self) -> None:
        line = format_status(
            self._zone, self._scale, self._min_distance, colours=self._colours
        )
        # Carriage return (no newline) redraws the single HUD line in place.
        print(f"\r{line}  ", end="", flush=True)


# ---------------------------------------------------------------------------
# Entry point for `ros2 run safecollab hud_node`
# ---------------------------------------------------------------------------


def main(args: list | None = None) -> None:  # pragma: no cover
    """ROS 2 entry point."""
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — hud_node requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = HudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # Same benign teardown race guard as the other nodes: the executor can
        # raise from take_message if mid-take when SIGINT shuts the context down.
        if rclpy.ok() and "convert call argument" not in str(exc):
            raise
    finally:
        print()  # end the in-place HUD line with a newline on shutdown
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
