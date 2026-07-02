"""Pure kitting plan — the leg sequence and Cartesian tool targets (no ROS/MoveIt).

The cycle is expressed as Cartesian ``tool0`` targets so MoveIt/Pilz can IK-solve
them; the planner only PLANS, ``motion_node`` executes under SSM. Unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

MotionType = Literal["ptp", "lin"]


@dataclass(frozen=True)
class KittingLeg:
    """One planned segment of the kitting cycle.

    Attributes:
        state: leg name published on /task/state (demo/HUD narrative).
        motion: "ptp" (joint transit) or "lin" (straight Cartesian).
        xyz: world-frame tool0 target position (metres).
        settle_s: dwell (s) after the segment before the next is planned.
    """

    state: str
    motion: MotionType
    xyz: tuple[float, float, float]
    settle_s: float


# Tool0 target heights (world z). Tool points straight down; _PICK_Z=0.78 dips
# INTO the bin (4 cm below its 0.82 rim) and places ON the 0.78 tray; the raised
# _ABOVE_Z gives a visible ~0.20 m pick/place stroke. Match cell.xacro furniture.
_ABOVE_Z = 0.98
_PICK_Z = 0.78


def kitting_legs(bin_side: str) -> list[KittingLeg]:
    """Ordered legs for one kitting cycle from the given feeder bin.

    Cycle: transit above bin (PTP) → dip in (LIN) → lift (LIN) → transit above
    tray (PTP) → dip in (LIN) → lift (LIN).

    Raises:
        ValueError: if ``bin_side`` is not "left" or "right".
    """
    if bin_side not in ("left", "right"):
        raise ValueError(f"bin_side must be 'left' or 'right', got {bin_side!r}")
    # Bins flank the base FORWARD of centre at world (0.15, ±0.35) — a
    # well-conditioned ~0.43 m front-side reach clear of the shoulder singularity
    # (pure-side bins at base-rel x≈0 made the +y reach near-singular).
    by = 0.35 if bin_side == "left" else -0.35
    bx = 0.15
    tx, ty = 0.35, 0.0
    # The longer DROP settle keeps the arm at the shared tray so an operator reach
    # there co-occurs with it and drives a red stop (AT-6 needs ≥3 escalations).
    return [
        KittingLeg(f"GO_TO_BIN_{bin_side.upper()}", "ptp", (bx, by, _ABOVE_Z), 0.4),
        KittingLeg("PICK", "lin", (bx, by, _PICK_Z), 0.6),
        KittingLeg("LIFT_BIN", "lin", (bx, by, _ABOVE_Z), 0.3),
        KittingLeg("GO_TO_TRAY", "ptp", (tx, ty, _ABOVE_Z), 0.4),
        KittingLeg("DROP", "lin", (tx, ty, _PICK_Z), 2.0),
        KittingLeg("LIFT_TRAY", "lin", (tx, ty, _ABOVE_Z), 0.4),
    ]


def cycle_sequence(n_cycles: int) -> list[KittingLeg]:
    """Flatten ``n_cycles`` kitting cycles, alternating bins left/right."""
    legs: list[KittingLeg] = []
    for i in range(n_cycles):
        legs.extend(kitting_legs("left" if i % 2 == 0 else "right"))
    return legs
