"""Small, coordinate-explicit helpers for quick-drop impact estimates."""

import math
from typing import Tuple


GRAVITY = 9.81


def impact_ned(
    *,
    north: float,
    east: float,
    yaw: float,
    offset_forward: float,
    offset_right: float,
    vn: float,
    ve: float,
    vd: float,
    release_height: float,
    target_height: float,
    delay: float,
    bias_north: float = 0.0,
    bias_east: float = 0.0,
) -> Tuple[float, float]:
    """Predict one bottle's NED horizontal impact at a target plane.

    ``yaw`` is clockwise from NED north. ``vd`` is positive downward.
    The current horizontal velocity is held constant over the measured
    mechanical release delay and the estimated ballistic fall.
    """
    values = (
        north, east, yaw, offset_forward, offset_right, vn, ve, vd,
        release_height, target_height, delay, bias_north, bias_east,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("release geometry must be finite")
    distance = release_height - target_height
    if distance <= 0.0 or delay < 0.0:
        raise ValueError("release must be above target and delay nonnegative")
    fall_time = (
        math.sqrt(vd * vd + 2.0 * GRAVITY * distance) - vd
    ) / GRAVITY
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    offset_north = offset_forward * cos_yaw - offset_right * sin_yaw
    offset_east = offset_forward * sin_yaw + offset_right * cos_yaw
    return (
        north + offset_north + vn * (delay + fall_time) + bias_north,
        east + offset_east + ve * (delay + fall_time) + bias_east,
    )


def landing_fits(
    impact: Tuple[float, float],
    center: Tuple[float, float],
    target_radius: float,
    bottle_radius: float,
    uncertainty: float,
) -> bool:
    """Whether the whole bottle fits after reserving prediction uncertainty."""
    values = (*impact, *center, target_radius, bottle_radius, uncertainty)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    if target_radius <= 0.0 or bottle_radius < 0.0 or uncertainty < 0.0:
        return False
    allowance = target_radius - bottle_radius - uncertainty
    return allowance >= 0.0 and math.hypot(
        impact[0] - center[0], impact[1] - center[1]
    ) <= allowance + 1e-12
