"""HUD ROS adapter — caches the latest safety state, redraws on a timer.

A view-only node (``# pragma: no cover``): subscribes the three §3 safety topics
and delegates all formatting to :mod:`safecollab.hud.format`. Launched on demand
(``hud:=true``), so CI/headless and the scenario harness are unaffected.
"""

from __future__ import annotations

from typing import Optional

from safecollab.hud.format import build_colour_map, format_status, load_hud_config

try:
    import rclpy  # pragma: no cover
    from rclpy.node import Node  # pragma: no cover
    from std_msgs.msg import Float32, String  # pragma: no cover

    from safecollab._ros_runtime import (  # pragma: no cover
        best_effort_qos,
        config_path,
        reliable_qos,
        spin_and_shutdown,
        transient_qos,
    )

    _HAS_ROS = True  # pragma: no cover
except ImportError:
    _HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]


class HudNode(Node):  # type: ignore[misc]  # pragma: no cover
    """Cache the latest safety state, redraw the HUD line on a timer."""

    def __init__(self) -> None:
        super().__init__("hud_node")  # type: ignore[call-arg]

        cfg = load_hud_config(config_path("hud.yaml"))
        self._colours = build_colour_map(cfg.colours) if cfg.use_colour else None

        # Fail-safe defaults until the first messages arrive.
        self._zone: str = "lost"
        self._scale: float = 0.0
        self._min_distance: Optional[float] = None

        # QoS matched to each topic's §3 contract (same as safety_monitor pubs).
        self.create_subscription(String, "/safety/zone", self._on_zone, transient_qos())
        self.create_subscription(
            Float32, "/safety/scale", self._on_scale, reliable_qos()
        )
        self.create_subscription(
            Float32, "/safety/min_distance", self._on_distance, best_effort_qos()
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


def main(args: list | None = None) -> None:  # pragma: no cover
    """ROS 2 entry point."""
    if not _HAS_ROS:
        raise RuntimeError(
            "rclpy is not available — hud_node requires a ROS 2 environment."
        )
    rclpy.init(args=args)
    node = HudNode()
    # on_shutdown ends the in-place HUD line with a newline.
    spin_and_shutdown(node, on_shutdown=lambda: print())
