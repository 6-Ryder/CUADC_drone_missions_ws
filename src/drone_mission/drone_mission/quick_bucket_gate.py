"""Quick bucket search evidence and shared final visual-release state."""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class Observation:
    stamp: float                 # Camera capture time in system-clock seconds.
    radius: float = 0.0          # Estimated cylinder opening radius in metres.


@dataclass(frozen=True)
class GateResult:
    action: str                  # continue, brake, release, takeover
    reason: str
    pixel_error_px: Optional[float] = None


def is_bucket_search_candidate(bucket, now: float) -> bool:
    """Allow a plausible rim after three distinct recent camera frames."""
    samples = bucket.get("bucket_search_samples", {}) if isinstance(bucket, dict) else {}
    recent = [radius for stamp, radius in samples.items()
              if 0 <= now - stamp <= 2.0 and math.isfinite(radius)
              and 0.03 <= radius <= 0.30]
    return len(recent) >= 3


class BucketReleaseGate:
    """Shared state across DESCEND and EXECUTE_DROP for one selected cylinder."""

    def __init__(self):
        self.bucket_pixel_uv = None
        self.final_align_started_at = None
        self.visual_center_frames = {}
        self.visual_brake_at = None
        self.timeout_brake_at = None
        self.released = False

    def commit_release(self):
        """Latch after the drop service request is issued, never on a preview."""
        self.released = True
