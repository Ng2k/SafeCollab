"""Perception loss-timeout state machine — pure, no ROS.

Tracks the latest detection and enforces a loss timeout, deciding when the
operator counts as *lost*. The ROS node stops broadcasting the ``world → human``
TF while lost, which is how the safety monitor detects a lost operator (FR-9).
"""

from __future__ import annotations

from typing import Optional, Tuple


class PerceptionLogic:
    """Pure-Python perception state machine.

    Tracks the latest human detection and enforces a **loss timeout**: if no
    valid detection has been received within ``loss_timeout_s`` seconds the
    node transitions to ``is_lost=True``, which causes the ROS wrapper to
    stop broadcasting the ``world → human`` TF.  A stale / absent TF is how
    the safety monitor detects a lost operator (AGENTS.md §3, FR-9).

    Typical usage (inside ``PerceptionNode`` callbacks)::

        # image callback:
        detection = detect_human(frame)
        if detection is not None:
            u, v, area, conf = detection
            x, y, z = ...  # back-project + transform
            sigma = estimate_uncertainty(area, depth, conf)
            self._logic.update(position=(x, y, z), sigma=sigma,
                               timestamp_s=now_s)

        # timer callback (≥ 20 Hz):
        self._logic.check_timeout(current_time_s=now_s)
        if not self._logic.is_lost:
            self._broadcast_tf(*self._logic.position)
            self._publish_uncertainty(self._logic.sigma)
    """

    #: Default loss timeout (seconds).  Configurable via the constructor.
    DEFAULT_LOSS_TIMEOUT_S: float = 0.5

    def __init__(self, loss_timeout_s: float = DEFAULT_LOSS_TIMEOUT_S) -> None:
        """Initialise with an optional custom loss-timeout duration.

        Args:
            loss_timeout_s: Seconds without a valid detection before the
                            node declares the operator lost.
        """
        self._loss_timeout_s: float = loss_timeout_s
        self._last_detection_s: Optional[float] = None
        self._last_position: Optional[Tuple[float, float, float]] = None
        self._last_sigma: float = 0.10  # initial σ (unknown) — positive for risk model
        self._lost: bool = True

    # ------------------------------------------------------------------
    # State updates
    # ------------------------------------------------------------------

    def update(
        self,
        position: Optional[Tuple[float, float, float]],
        sigma: float,
        timestamp_s: float,
    ) -> None:
        """Update state with the result of processing one camera frame.

        Args:
            position: Estimated world-frame ``(x, y, z)`` of the operator,
                      or ``None`` when detection failed on this frame.
            sigma: Uncertainty estimate σ (metres) for this detection.
            timestamp_s: Timestamp of the processed frame (seconds, any
                         monotonic clock — just needs to be consistent with
                         the clock passed to :meth:`check_timeout`).
        """
        if position is not None:
            self._last_position = position
            self._last_sigma = sigma
            self._last_detection_s = timestamp_s
            self._lost = False
        # If position is None, we do NOT update _last_detection_s;
        # the timeout check is left to check_timeout() so the caller
        # controls when to enforce the loss (e.g., on a separate timer).

    def check_timeout(self, current_time_s: float) -> None:
        """Enforce the loss timeout.

        Should be called on each timer tick (≥ 20 Hz in the ROS node).
        If ``loss_timeout_s`` seconds have elapsed since the last valid
        detection (or if there has never been a detection), set ``is_lost``.

        Args:
            current_time_s: Current time in seconds (same clock as
                            timestamps passed to :meth:`update`).
        """
        if self._last_detection_s is None:
            self._lost = True
            return
        elapsed = current_time_s - self._last_detection_s
        if elapsed > self._loss_timeout_s:
            self._lost = True

    # ------------------------------------------------------------------
    # Read-only state accessors
    # ------------------------------------------------------------------

    @property
    def is_lost(self) -> bool:
        """``True`` when the operator is lost (no recent valid detection)."""
        return self._lost

    @property
    def position(self) -> Optional[Tuple[float, float, float]]:
        """Last known world-frame position, or ``None`` when ``is_lost``."""
        if self._lost:
            return None
        return self._last_position

    @property
    def sigma(self) -> float:
        """Latest uncertainty estimate σ (metres, always > 0)."""
        return self._last_sigma
