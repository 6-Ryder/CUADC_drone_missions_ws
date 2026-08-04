"""Deterministic no-progress detection for a single mission action."""

import math
from typing import Optional


class ProgressMonitor:
    """Detect a stalled decreasing error over an explicitly timed window."""

    def __init__(self, window_s: float, min_improvement: float):
        """Configure the observation window and required error reduction."""
        if not math.isfinite(window_s) or window_s <= 0.0:
            raise ValueError("window_s must be finite and greater than zero")
        if not math.isfinite(min_improvement) or min_improvement <= 0.0:
            raise ValueError("min_improvement must be finite and greater than zero")

        self._window_s = float(window_s)
        self._min_improvement = float(min_improvement)
        self._baseline_error: Optional[float] = None
        self._last_error: Optional[float] = None
        self._window_started_at: Optional[float] = None
        self._last_observed_at: Optional[float] = None
        self._retry_used = False

    @property
    def retry_used(self) -> bool:
        """Return whether the one permitted retry has already been marked."""
        return self._retry_used

    def observe(self, error: float, *, now: float) -> bool:
        """Record an error sample and return ``True`` when progress stalled."""
        error = self._finite_value(error, "error")
        now = self._finite_value(now, "now")
        self._require_monotonic(now)

        self._last_error = error
        self._last_observed_at = now
        if self._baseline_error is None:
            self._baseline_error = error
            self._window_started_at = now
            return False

        if self._baseline_error - error >= self._min_improvement:
            self._baseline_error = error
            self._window_started_at = now
            return False

        return now - self._window_started_at >= self._window_s

    def mark_retry(self, *, now: float) -> bool:
        """Start one fresh window for retrying the unchanged current action."""
        now = self._finite_value(now, "now")
        self._require_monotonic(now)
        if self._retry_used:
            return False
        if self._last_error is None:
            raise RuntimeError("cannot mark a retry before observing progress")

        self._retry_used = True
        self._baseline_error = self._last_error
        self._window_started_at = now
        self._last_observed_at = now
        return True

    def reset(self) -> None:
        """Clear observations and retry state before monitoring a new action."""
        self._baseline_error = None
        self._last_error = None
        self._window_started_at = None
        self._last_observed_at = None
        self._retry_used = False

    def _require_monotonic(self, now: float) -> None:
        """Reject timestamps that move backwards within an action."""
        if self._last_observed_at is not None and now < self._last_observed_at:
            raise ValueError("now must be monotonic")

    @staticmethod
    def _finite_value(value: float, name: str) -> float:
        """Convert one numeric input and reject non-finite values."""
        converted = float(value)
        if not math.isfinite(converted):
            raise ValueError(f"{name} must be finite")
        return converted
