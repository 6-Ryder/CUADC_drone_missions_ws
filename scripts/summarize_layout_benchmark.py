#!/usr/bin/env python3
"""Summarize the quick strategy's 0/50 degree layout benchmark."""

import argparse
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


BASELINE_HEADING_DEG = 0.0
ROTATED_HEADING_DEG = 50.0
REQUIRED_SUCCESSES = 3
MAX_RATIO = 1.05


@dataclass(frozen=True)
class BenchmarkResult:
    """Median durations and pass/fail status for the two quick headings."""

    baseline_median_s: float
    rotated_median_s: float
    ratio: float
    passed: bool


def _normalized_heading(value: object, path: Path) -> float:
    """Return a finite heading normalized to the frozen frame's range."""
    try:
        heading = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{path}: layout_heading_deg must be numeric") from error
    if not math.isfinite(heading):
        raise ValueError(f"{path}: layout_heading_deg must be finite")
    return heading % 360.0


def _duration(value: object, path: Path) -> float:
    """Return one finite, positive mission duration."""
    try:
        duration = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{path}: total_duration_s must be numeric") from error
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError(f"{path}: total_duration_s must be finite and positive")
    return duration


def _load_quick_durations(paths: Iterable[Path | str]) -> dict[float, list[float]]:
    """Load valid quick-run durations at the two benchmark headings only."""
    durations = {
        BASELINE_HEADING_DEG: [],
        ROTATED_HEADING_DEG: [],
    }
    seen_paths: dict[Path, Path] = {}
    for raw_path in paths:
        path = Path(raw_path)
        resolved_path = path.resolve()
        previous_path = seen_paths.get(resolved_path)
        if previous_path is not None:
            raise ValueError(
                f"{path}: duplicate summary input "
                f"(same file as {previous_path})"
            )
        seen_paths[resolved_path] = path
        try:
            summary = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"{path}: cannot read summary JSON") from error
        if not isinstance(summary, dict):
            raise ValueError(f"{path}: summary JSON must be an object")
        if summary.get("success") is not True or summary.get("strategy") != "quick":
            continue

        heading = _normalized_heading(summary.get("layout_heading_deg"), path)
        if heading in durations:
            durations[heading].append(_duration(summary.get("total_duration_s"), path))
    return durations


def analyze(paths: Iterable[Path | str]) -> BenchmarkResult:
    """Evaluate valid quick summaries against the layout efficiency baseline."""
    durations = _load_quick_durations(paths)
    missing = [
        f"{heading:g}°={len(durations[heading])}"
        for heading in (BASELINE_HEADING_DEG, ROTATED_HEADING_DEG)
        if len(durations[heading]) < REQUIRED_SUCCESSES
    ]
    if missing:
        raise ValueError(
            "need at least three successful quick runs for each heading "
            f"({', '.join(missing)})"
        )

    baseline = statistics.median(durations[BASELINE_HEADING_DEG])
    rotated = statistics.median(durations[ROTATED_HEADING_DEG])
    ratio = rotated / baseline
    return BenchmarkResult(
        baseline_median_s=baseline,
        rotated_median_s=rotated,
        ratio=ratio,
        passed=ratio <= MAX_RATIO,
    )


def main(arguments: Sequence[str] | None = None) -> int:
    """Run the benchmark CLI and return an explicit shell exit code."""
    parser = argparse.ArgumentParser(
        description="Compare quick-mission 0° and 50° summary durations.",
    )
    parser.add_argument("summaries", nargs="+", metavar="SUMMARY")
    args = parser.parse_args(arguments)
    try:
        result = analyze(args.summaries)
    except ValueError as error:
        print(f"layout benchmark failed: {error}", file=sys.stderr)
        return 1

    print(f"baseline_median_s={result.baseline_median_s:.3f}")
    print(f"rotated_median_s={result.rotated_median_s:.3f}")
    print(f"ratio={result.ratio:.6f}")
    if result.passed:
        print(f"PASS: ratio <= {MAX_RATIO:.2f}")
        return 0
    print(f"FAIL: ratio > {MAX_RATIO:.2f}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
