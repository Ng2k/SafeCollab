"""Shared ROS 2 lifecycle helpers for the safecollab nodes.

Centralises the glue every node otherwise copy-pasted: the QoS profiles matched
to the §3 topic contracts, the package ``config/`` path resolver, and the spin +
graceful-shutdown wrapper (which swallows the one known-benign SIGINT teardown
race).

This module imports ``rclpy`` at import time, so it must only be imported from
inside a node's ``try: import rclpy`` guard — never at module top level, or the
pure-Python unit environment (which has no ``rclpy``) could not import the node.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def reliable_qos(depth: int = 10) -> QoSProfile:
    """RELIABLE / KEEP_LAST profile — the default for command/state topics."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def transient_qos(depth: int = 1) -> QoSProfile:
    """RELIABLE / TRANSIENT_LOCAL profile — latches the last value for late joiners."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def best_effort_qos(depth: int = 10) -> QoSProfile:
    """BEST_EFFORT / KEEP_LAST profile — for sensor-style, drop-tolerant topics."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def config_path(name: str) -> Path:
    """Absolute path to ``config/<name>`` beside the installed package.

    The colcon install layout and the local editable layout both place
    ``config/`` next to the ``safecollab/`` package dir (ground rule 5).
    """
    return Path(__file__).resolve().parent.parent / "config" / name


def spin_and_shutdown(node, on_shutdown: Optional[Callable[[], None]] = None) -> None:
    """Spin *node* until interrupted, then tear down cleanly.

    Swallows the benign SIGINT teardown race: rclpy's executor can raise a pybind
    "Unable to convert call argument" ``RuntimeError`` from ``take_message`` (e.g.
    on the ``/clock`` subscription ``use_sim_time`` creates) if it is mid-take when
    the signal handler shuts the context down. ``rclpy.ok()`` is an unreliable
    discriminator, so that specific take-time error is also treated as benign;
    anything else re-raises so real bugs still surface.

    Args:
        node: the ROS 2 node to spin.
        on_shutdown: optional callback run in ``finally`` before the node is
            destroyed (e.g. the HUD ends its in-place line with a newline).
    """
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if rclpy.ok() and "convert call argument" not in str(exc):
            raise
    finally:
        if on_shutdown is not None:
            on_shutdown()
        node.destroy_node()
        # On SIGINT rclpy's default handler already shut the context down; calling
        # rclpy.shutdown() again raises RCLError, so guard with ok().
        if rclpy.ok():
            rclpy.shutdown()
