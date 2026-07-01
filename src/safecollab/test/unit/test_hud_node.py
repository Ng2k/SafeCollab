"""TDD spec for safecollab.hud_node — the console safety HUD (P5 sprint 2).

Pure-Python: no ROS. Covers the (zone, scale, min_distance) -> status-line
formatter, the ANSI SGR colour helpers, and load_hud_config. The ROS node shell
(HudNode) is a thin I/O wrapper over format_status and is not unit-tested
(pragma:no-cover), exactly like the other nodes' ROS wrappers.

The acceptance bar for this sprint is the 5-second legibility check (AGENTS.md
§10 P5): a viewer must name the safety state within five seconds. These tests
pin the formatter's content/alignment so that bar is reproducible.
"""

from __future__ import annotations

from pathlib import Path

from safecollab.hud_node import (
    build_colour_map,
    format_status,
    load_hud_config,
    sgr,
)

_CONFIG = Path(__file__).resolve().parents[2] / "config" / "hud.yaml"


# ---------------------------------------------------------------------------
# format_status — the pure HUD line formatter
# ---------------------------------------------------------------------------


class TestFormatStatus:
    def test_green_full_speed(self):
        line = format_status("green", 1.0, 1.23)
        assert "GREEN" in line
        assert "100%" in line
        assert "1.23" in line

    def test_red_protective_stop(self):
        line = format_status("red", 0.0, 0.30)
        assert "RED" in line
        assert "0%" in line

    def test_lost_shows_no_distance(self):
        line = format_status("lost", 0.0, None)
        assert "LOST" in line
        assert "0%" in line
        assert "--" in line

    def test_lost_hides_distance_even_if_a_value_arrives(self):
        # FR-9: in 'lost' the last position is never reused, so the HUD must not
        # display a stale distance even if /safety/min_distance still carries one.
        line = format_status("lost", 0.0, 0.42)
        assert "0.42" not in line
        assert "--" in line

    def test_scale_rounds_to_nearest_percent(self):
        assert "35%" in format_status("yellow", 0.349, 0.6)

    def test_scale_zero_and_one(self):
        assert "0%" in format_status("red", 0.0, 0.3)
        assert "100%" in format_status("green", 1.0, 1.5)

    def test_scale_is_clamped(self):
        # Defensive: an out-of-range scale never prints >100% or a negative %.
        assert "100%" in format_status("green", 1.4, 1.5)
        assert "0%" in format_status("red", -0.2, 0.3)

    def test_zone_is_uppercased(self):
        assert "YELLOW" in format_status("yellow", 0.5, 0.6)

    def test_distance_has_two_decimals(self):
        assert "0.40" in format_status("green", 1.0, 0.4)

    def test_none_distance_shows_placeholder(self):
        assert "--" in format_status("green", 1.0, None)

    def test_unknown_zone_does_not_raise(self):
        line = format_status("weird", 0.5, 0.5)
        assert "WEIRD" in line

    def test_plain_by_default_has_no_ansi(self):
        line = format_status("red", 0.0, 0.3)
        assert "\033[" not in line

    def test_colour_wraps_zone_and_resets(self):
        colours = build_colour_map({"red": 31})
        line = format_status("red", 0.0, 0.3, colours=colours)
        assert "\033[31m" in line  # opening colour
        assert "\033[0m" in line  # reset
        assert "RED" in line  # word still legible

    def test_colour_absent_zone_stays_plain(self):
        colours = build_colour_map({"green": 32})
        line = format_status("red", 0.0, 0.3, colours=colours)
        assert "RED" in line
        assert "\033[" not in line  # no code for red -> no escape


# ---------------------------------------------------------------------------
# ANSI SGR helpers
# ---------------------------------------------------------------------------


class TestSgr:
    def test_sgr_builds_escape(self):
        assert sgr(31) == "\033[31m"

    def test_build_colour_map(self):
        m = build_colour_map({"green": 32, "red": 31})
        assert m == {"green": "\033[32m", "red": "\033[31m"}


# ---------------------------------------------------------------------------
# load_hud_config
# ---------------------------------------------------------------------------


class TestLoadHudConfig:
    def test_refresh_hz_is_positive(self):
        cfg = load_hud_config(_CONFIG)
        assert cfg.refresh_hz > 0.0

    def test_use_colour_is_bool(self):
        cfg = load_hud_config(_CONFIG)
        assert isinstance(cfg.use_colour, bool)

    def test_colours_cover_all_zones(self):
        cfg = load_hud_config(_CONFIG)
        for zone in ("green", "yellow", "red", "lost"):
            assert zone in cfg.colours
