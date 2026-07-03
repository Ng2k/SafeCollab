"""Human detection — the classical OpenCV colour-blob detector (pure, no ROS).

This is the swappable *detector* policy: it turns a BGR frame into a pixel
centroid + a confidence, and nothing else. For a real-camera deployment this is
the module to replace (tune the HSV band, or swap in a skin-tone / motion / ML
detector) without touching the geometry, the state machine, or the ROS node.
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np


def detect_human(
    bgr_image: np.ndarray,
    *,
    min_area_px: float = 100.0,
    conf_area_ref_px: float = 2000.0,
    hue_low: int = 20,
    hue_high: int = 40,
    sat_low: int = 100,
    sat_high: int = 255,
    val_low: int = 100,
    val_high: int = 255,
) -> Optional[Tuple[float, float, float, float]]:
    """Detect a human in a BGR image using colour-based blob detection.

    Classical OpenCV pipeline: BGR → HSV → colour-range threshold →
    largest contour → centroid.

    In the Gazebo simulation the human marker uses a **yellow** (BGR
    ``[0, 255, 255]``, HSV hue ≈ 30) colour that falls within the default
    HSV range ``hue_low=20, hue_high=40``.  For real-camera deployment the
    HSV parameters should be tuned or the detector replaced with a skin-tone /
    motion-based approach.

    Args:
        bgr_image: OpenCV-style BGR image (H × W × 3 ``uint8``).
        min_area_px: Minimum blob area (pixels²) to accept as a valid
                     detection.  Smaller blobs are ignored.
        conf_area_ref_px: Reference blob area (pixels²) representing a full,
                     clean operator detection.  ``confidence`` saturates to 1.0
                     at or above this area and scales down for smaller/partial
                     blobs.  Sized to the operator marker under the overhead
                     camera so a normal detection is confident.
        hue_low, hue_high: HSV hue range (OpenCV ``[0, 180]`` convention).
        sat_low, sat_high: HSV saturation range.
        val_low, val_high: HSV value (brightness) range.

    Returns:
        ``(u_px, v_px, area_px2, confidence)`` or ``None`` if no human
        is found.

        - ``u_px``, ``v_px``: centroid pixel coordinates.
        - ``area_px2``: blob area in pixels².
        - ``confidence``: blob area relative to the expected operator size
          (``conf_area_ref_px``), clipped to ``(0, 1]`` — a full, solid blob is
          confident; a partial/occluded blob scores lower.
    """
    if bgr_image is None or bgr_image.size == 0:
        return None

    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([hue_low, sat_low, val_low], dtype=np.uint8),
        np.array([hue_high, sat_high, val_high], dtype=np.uint8),
    )

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Take the single largest contour — assumes one dominant human blob.
    largest = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(largest))

    if area < min_area_px:
        return None

    moments = cv2.moments(largest)
    if moments["m00"] < 1e-6:  # pragma: no cover  # degenerate contour guard
        return None

    u = moments["m10"] / moments["m00"]
    v = moments["m01"] / moments["m00"]

    # Confidence = how completely the blob fills the EXPECTED operator size
    # (conf_area_ref_px), not its fraction of the frame: a full, solid detection
    # is confident (σ small); a partial/occluded blob scores low (σ grows).
    confidence = min(1.0, area / conf_area_ref_px) if conf_area_ref_px > 0 else 0.0

    return u, v, area, confidence
