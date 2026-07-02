"""Detection uncertainty estimation — pure, no ROS.

Turns detection quality (depth + confidence) into a σ that feeds ``Z_d`` in the
ISO/TS 15066 risk model (``risk.thresholds()``). A genuinely non-constant σ is
required so the safety thresholds widen and narrow with perception quality.
"""

from __future__ import annotations


def estimate_uncertainty(
    area_px: float,
    depth_m: float,
    confidence: float,
    *,
    base_sigma: float = 0.02,
    depth_coeff: float = 0.05,
    noise_coeff: float = 0.15,
    min_sigma: float = 0.01,
) -> float:
    """Estimate detection uncertainty σ (metres).

    σ grows with depth (farther objects are less precisely localised) and
    grows as confidence drops (noisier or smaller blobs → more uncertainty).

    Model:

    .. code-block::

        σ = base_sigma
            + depth_coeff * depth_m               # grows with range
            + noise_coeff * (1 − confidence)      # grows as quality falls
        σ = max(σ, min_sigma)                     # always positive

    Args:
        area_px: Detected blob area in pixels² (accepted but primarily
                 ``confidence`` and ``depth_m`` drive σ; ``area_px`` is
                 available for callers that wish to pass it explicitly as an
                 additional quality proxy).
        depth_m: Estimated distance to the human (metres).
        confidence: Detection confidence in ``[0, 1]``.  ``1.0`` = very
                    certain; ``0.0`` = effectively a guess.
        base_sigma: Fixed floor of uncertainty even for a perfect detection.
        depth_coeff: Rate at which σ grows per metre of depth.
        noise_coeff: Maximum σ contribution from zero-confidence detection.
        min_sigma: Hard lower bound on the returned σ (metres).

    Returns:
        σ in metres (always ≥ ``min_sigma``).
    """
    depth_term = depth_coeff * depth_m
    confidence_term = noise_coeff * (1.0 - max(0.0, min(1.0, confidence)))
    sigma = base_sigma + depth_term + confidence_term
    return max(sigma, min_sigma)
