"""Fail-closed release decision for the optional quick small-bucket profile.

This module contains no ROS calls.  Callers provide capture timestamps and
telemetry timestamps; service ACKs are deliberately outside this decision.
"""

from dataclasses import dataclass
import math
import statistics
from typing import Optional, Tuple

from drone_mission.quick_release_geometry import impact_ned, landing_fits


BUCKET_IDENTITY_RADIUS_M = 0.5


@dataclass(frozen=True)
class Observation:
    stamp: float                 # camera capture time, seconds in ROS/system clock
    north: float
    east: float
    radius: float = 0.0


@dataclass(frozen=True)
class Calibration:
    front_forward: float
    rear_forward: float
    front_right: float
    rear_right: float
    front_delay: float
    rear_delay: float
    rim_height: float
    front_uncertainty: float
    rear_uncertainty: float
    disc_uncertainty: float
    front_bias_north: float = 0.0
    front_bias_east: float = 0.0
    rear_bias_north: float = 0.0
    rear_bias_east: float = 0.0
    home_body_height: float = 0.0
    front_down: float = 0.0
    rear_down: float = 0.0

    def valid_for_bucket(self) -> bool:
        numbers = tuple(self.__dict__.values())
        return (all(math.isfinite(value) for value in numbers)
                and self.front_delay >= 0 and self.rear_delay >= 0
                and self.rim_height >= 0
                and self.home_body_height >= 0
                and 0 <= self.front_down <= self.home_body_height
                and 0 <= self.rear_down <= self.home_body_height
                and 0 <= self.front_uncertainty <= 0.03
                and self.rear_uncertainty >= 0
                and self.disc_uncertainty >= 0)


@dataclass(frozen=True)
class GateResult:
    action: str                  # continue, brake, release, takeover
    reason: str
    front_residual: float = math.inf
    rear_residual: float = math.inf
    front_impact: Optional[Tuple[float, float]] = None
    rear_impact: Optional[Tuple[float, float]] = None


def _xy(item):
    if isinstance(item, dict):
        return float(item.get("world_x", math.nan)), float(item.get("world_y", math.nan))
    return float(getattr(item, "world_x", math.nan)), float(getattr(item, "world_y", math.nan))


def is_small_bucket_candidate(bucket) -> bool:
    """Search/selection must use the same opening-size window as the gate."""
    if isinstance(bucket, dict) and "small_radius_confirmed" in bucket:
        return bool(bucket["small_radius_confirmed"])
    if isinstance(bucket, dict):
        radius = bucket.get("radius", math.nan)
    else:
        radius = getattr(bucket, "radius", math.nan)
    try:
        radius = float(radius)
    except (TypeError, ValueError):
        return False
    return math.isfinite(radius) and 0.06 <= radius <= 0.09


def is_bucket_search_candidate(bucket, now: float) -> bool:
    """Allow a plausible rim to be approached after three distinct recent frames."""
    samples = bucket.get("bucket_search_samples", {}) if isinstance(bucket, dict) else {}
    recent = [radius for stamp, radius in samples.items()
              if 0 <= now - stamp <= 2.0 and math.isfinite(radius)
              and 0.03 <= radius <= 0.30]
    return len(recent) >= 3


def pair_bucket_marker(bucket, cylinders, markers, max_separation=0.25):
    """Return one unique paired blue disc or None (coordinates are World ENU)."""
    bx, by = _xy(bucket)
    if not all(math.isfinite(v) for v in (bx, by)):
        return None
    nearby = []
    for marker in markers:
        mx, my = _xy(marker)
        if math.hypot(mx - bx, my - by) <= max_separation:
            nearby.append(marker)
    if len(nearby) != 1:
        return None
    mx, my = _xy(nearby[0])
    associated = sum(
        math.hypot(_xy(c)[0] - mx, _xy(c)[1] - my) <= max_separation
        for c in cylinders
    )
    return nearby[0] if associated == 1 else None


class BucketReleaseGate:
    """One target's confirmation, brake and single-release permission."""

    def __init__(self, bucket_ne: Optional[Tuple[float, float]],
                 disc_ne: Optional[Tuple[float, float]],
                 calibration: Optional[Calibration]):
        self.bucket_ne = bucket_ne
        self.disc_ne = disc_ne
        self.calibration = calibration
        self.started_at = None
        self._radii = {}
        self.radius_confirmed = False
        self.radius_failed = False
        self._fit_frames = {}
        self._brake_at = None
        self._brake_source = None
        self._last_cylinder_at = None
        self.released = False

    def start(self, now: float):
        if self.started_at is None:
            self.started_at = now
            self._last_cylinder_at = now

    def commit_release(self):
        """Latch after the service request is actually sent, never on a preview."""
        self.released = True

    def observe_radius(self, cylinder: Optional[Observation], now: float) -> bool:
        if self.started_at is None:
            self.start(now)
        if self.radius_failed:
            return False
        if cylinder is None or not self._fresh(cylinder, now, 0.25):
            return self.radius_confirmed
        if cylinder.stamp in self._radii:
            return self.radius_confirmed
        if not math.isfinite(cylinder.radius):
            return self.radius_confirmed
        self._radii[cylinder.stamp] = cylinder.radius
        self._radii = {stamp: radius for stamp, radius in self._radii.items()
                       if now - stamp <= 1.0}
        self._last_cylinder_at = now
        inliers = [radius for radius in self._radii.values()
                   if math.isfinite(radius) and 0.06 <= radius <= 0.09]
        if len(inliers) >= 5:
            self.radius_confirmed = True
        elif len(self._radii) >= 15:
            # A noisy small rim may alternate with the blue disc or a larger
            # radius. Do not reject it after the first five mixed samples.
            self.radius_failed = True
        return self.radius_confirmed

    @staticmethod
    def _fresh(obs, now, max_age):
        return (obs is not None and math.isfinite(obs.stamp)
                and 0 < obs.stamp <= now and now - obs.stamp <= max_age
                and math.isfinite(obs.north) and math.isfinite(obs.east))

    def _impact(self, north, east, yaw, vn, ve, vd, altitude, plane,
                forward, right, delay, bias_n, bias_e):
        return impact_ned(
            north=north, east=east, yaw=yaw,
            offset_forward=forward, offset_right=right,
            vn=vn, ve=ve, vd=vd,
            release_height=altitude, target_height=plane,
            delay=delay, bias_north=bias_n, bias_east=bias_e,
        )

    def evaluate(self, *, now, altitude, north, east, yaw, vn, ve, vd,
                 telemetry_stamp, cylinder, marker) -> GateResult:
        if self.released:
            return GateResult("takeover", "already_released")
        if self.started_at is None:
            return GateResult("continue", "awaiting_2_4m_confirmation")
        if not all(math.isfinite(v) for v in
                   (now, altitude, north, east, yaw, vn, ve, vd, telemetry_stamp)):
            return GateResult("takeover", "nonfinite_telemetry")
        if altitude < 1.5:
            return GateResult("takeover", "below_safe_altitude")
        if now - self.started_at > 15.0:
            return GateResult("takeover", "precision_deadline")
        if not (0 <= now - telemetry_stamp <= 0.10):
            return GateResult("continue", "stale_telemetry")
        if not 1.5 <= altitude <= 1.7:
            return GateResult("continue", "release_height")
        if math.hypot(vn, ve) > 0.08 or abs(vd) > 0.08:
            self._brake_at = None
            self._fit_frames.clear()
            return GateResult("continue", "velocity")

        marker_ok = self.disc_ne is not None and self._fresh(marker, now, 0.4) and (
            math.hypot(marker.north - self.disc_ne[0],
                       marker.east - self.disc_ne[1]) <= BUCKET_IDENTITY_RADIUS_M)
        cylinder_ok = self.bucket_ne is not None and self._fresh(cylinder, now, 0.25) and (
            math.hypot(cylinder.north - self.bucket_ne[0],
                       cylinder.east - self.bucket_ne[1]) <= BUCKET_IDENTITY_RADIUS_M)
        if cylinder_ok:
            self._last_cylinder_at = now

        cal = self.calibration
        if cal is None or not cal.valid_for_bucket():
            bucket_possible = False
            cal = Calibration(0.04, -0.04, 0, 0, 0, 0, 0.3,
                              0.03, 0.03, 0.10)
        else:
            bucket_possible = self.radius_confirmed and not self.radius_failed

        if marker_ok:
            front_height = altitude + cal.home_body_height - cal.front_down
            rear_height = altitude + cal.home_body_height - cal.rear_down
            front_disc = self._impact(north, east, yaw, vn, ve, vd, front_height, 0,
                                      cal.front_forward, cal.front_right,
                                      cal.front_delay, cal.front_bias_north,
                                      cal.front_bias_east)
            rear_disc = self._impact(north, east, yaw, vn, ve, vd, rear_height, 0,
                                     cal.rear_forward, cal.rear_right,
                                     cal.rear_delay, cal.rear_bias_north,
                                     cal.rear_bias_east)
            fallback_fit = all(landing_fits(
                point, (marker.north, marker.east), 0.5, 0.03,
                cal.disc_uncertainty) for point in (front_disc, rear_disc))
        else:
            fallback_fit = False

        small_fit = False
        front_residual = rear_residual = math.inf
        if (bucket_possible and cylinder_ok and marker_ok
                and math.hypot(cylinder.north - marker.north,
                               cylinder.east - marker.east) <= 0.25):
            front = self._impact(north, east, yaw, vn, ve, vd, front_height,
                                 cal.rim_height, cal.front_forward,
                                 cal.front_right, cal.front_delay,
                                 cal.front_bias_north, cal.front_bias_east)
            rear = rear_disc
            front_residual = math.dist(front, (cylinder.north, cylinder.east))
            rear_residual = math.dist(rear, (marker.north, marker.east))
            small_fit = (landing_fits(front, (cylinder.north, cylinder.east),
                                      0.075, 0.03, cal.front_uncertainty)
                         and landing_fits(rear, (marker.north, marker.east),
                                          0.5, 0.03, cal.rear_uncertainty))

        if small_fit:
            if cylinder.stamp not in self._fit_frames:
                self._fit_frames[cylinder.stamp] = now
            self._fit_frames = {stamp: t for stamp, t in self._fit_frames.items()
                                if now - t <= 1.0}
            if len(self._fit_frames) >= 3:
                if self._brake_source != "bucket":
                    self._brake_at = now
                    self._brake_source = "bucket"
                    return GateResult("brake", "small_bucket_brake",
                                      front_residual, rear_residual,
                                      front, rear)
                if (now - self._brake_at >= 0.30
                        and cylinder.stamp > self._brake_at):
                    return GateResult("release", "small_bucket",
                                      front_residual, rear_residual,
                                      front, rear)
            return GateResult("continue", "collecting_bucket_frames",
                              front_residual, rear_residual, front, rear)
        if self._brake_source == "bucket" and (not cylinder_ok or not marker_ok):
            return GateResult("brake", "waiting_post_brake_bucket_frame")
        self._fit_frames.clear()
        if self._brake_source == "bucket":
            self._brake_at = None
            self._brake_source = None

        fallback_due = (self.radius_failed or not bucket_possible
                        or now - self.started_at >= 14.5
                        or (self._last_cylinder_at is not None
                            and now - self._last_cylinder_at >= 2.0))
        if fallback_due and fallback_fit:
            front_residual = math.dist(front_disc, (marker.north, marker.east))
            rear_residual = math.dist(rear_disc, (marker.north, marker.east))
            if self._brake_source != "disc":
                self._brake_at = now
                self._brake_source = "disc"
                return GateResult("brake", "disc_brake",
                                  front_residual, rear_residual,
                                  front_disc, rear_disc)
            if now - self._brake_at >= 0.30 and marker.stamp > self._brake_at:
                return GateResult("release", "blue_disc_fallback",
                                  front_residual, rear_residual,
                                  front_disc, rear_disc)
            return GateResult("brake", "waiting_post_brake_disc_frame",
                              front_residual, rear_residual,
                              front_disc, rear_disc)
        self._brake_at = None
        self._brake_source = None
        return GateResult("continue", "no_safe_release")
