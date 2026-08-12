"""Immutable transforms between field-local and PX4 NED coordinates."""

from dataclasses import dataclass, field
import math
from typing import Tuple


@dataclass(frozen=True)
class LayoutFrame:
    """Field frame whose forward axis has an absolute clockwise NED heading."""

    heading_deg: float
    _heading_rad: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        """Validate and cache the normalized heading in radians."""
        value = float(self.heading_deg)
        if not math.isfinite(value):
            raise ValueError("layout heading must be finite")
        normalized = value % 360.0
        object.__setattr__(self, "heading_deg", normalized)
        object.__setattr__(self, "_heading_rad", math.radians(normalized))

    @property
    def heading_rad(self) -> float:
        """Return the normalized heading in radians."""
        return self._heading_rad

    @property
    def yaw_rad(self) -> float:
        """Return the heading wrapped to the conventional yaw range."""
        return math.atan2(
            math.sin(self._heading_rad), math.cos(self._heading_rad)
        )

    def local_to_ned(
        self, forward: float, right: float
    ) -> Tuple[float, float]:
        """Transform field-local forward/right coordinates to NED."""
        cos_h = math.cos(self._heading_rad)
        sin_h = math.sin(self._heading_rad)
        return (
            forward * cos_h - right * sin_h,
            forward * sin_h + right * cos_h,
        )

    def ned_to_local(self, north: float, east: float) -> Tuple[float, float]:
        """Transform NED coordinates to field-local forward/right."""
        cos_h = math.cos(self._heading_rad)
        sin_h = math.sin(self._heading_rad)
        return (
            north * cos_h + east * sin_h,
            -north * sin_h + east * cos_h,
        )

    def bounds_center_ned(
        self, forward_min: float, forward_max: float,
        right_min: float, right_max: float,
    ) -> Tuple[float, float]:
        """Return the NED center of a local-coordinate rectangular bound."""
        return self.local_to_ned(
            (forward_min + forward_max) / 2.0,
            (right_min + right_max) / 2.0,
        )

    def contains_ned(
        self, north: float, east: float,
        forward_min: float, forward_max: float,
        right_min: float, right_max: float, margin: float = 0.0,
    ) -> bool:
        """Return whether a NED point lies within local bounds and margin."""
        forward, right = self.ned_to_local(north, east)
        return (
            forward_min - margin <= forward <= forward_max + margin
            and right_min - margin <= right <= right_max + margin
        )
