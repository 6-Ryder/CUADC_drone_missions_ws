"""Contracts for the quick layout-efficiency benchmark summary."""

import importlib.util
import json
from pathlib import Path
import sys

import pytest

from drone_mission.mission_state_machine import MissionFSM
from drone_utils.layout_frame import LayoutFrame


ROOT = Path(__file__).resolve().parents[3]
ANALYZER_PATH = ROOT / "scripts" / "summarize_layout_benchmark.py"


def load_analyzer():
    """Load the standalone benchmark CLI as a testable module."""
    spec = importlib.util.spec_from_file_location(
        "summarize_layout_benchmark", ANALYZER_PATH,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_summaries(tmp_path, durations_by_heading, strategy="quick"):
    """Create one mission summary for every requested benchmark duration."""
    paths = []
    for heading, durations in durations_by_heading.items():
        for index, duration in enumerate(durations):
            path = tmp_path / f"{heading}_{index}.json"
            path.write_text(json.dumps({
                "success": True,
                "strategy": strategy,
                "layout_heading_deg": heading,
                "total_duration_s": duration,
            }), encoding="utf-8")
            paths.append(path)
    return paths


def write_mixed_summaries(tmp_path):
    """Create results that must not supply three valid quick runs per heading."""
    paths = write_summaries(tmp_path, {0.0: [100.0, 101.0]}, strategy="quick")
    paths.extend(write_summaries(tmp_path, {50.0: [102.0, 103.0]}, strategy="quick"))
    wrong_strategy = tmp_path / "wrong_strategy.json"
    wrong_strategy.write_text(json.dumps({
        "success": True,
        "strategy": "main",
        "layout_heading_deg": 0.0,
        "total_duration_s": 99.0,
    }), encoding="utf-8")
    paths.append(wrong_strategy)
    failed = tmp_path / "failed_quick.json"
    failed.write_text(json.dumps({
        "success": False,
        "strategy": "quick",
        "layout_heading_deg": 50.0,
        "total_duration_s": 104.0,
    }), encoding="utf-8")
    paths.append(failed)
    return paths


def test_quick_benchmark_accepts_five_percent_or_better(tmp_path):
    analyzer = load_analyzer()
    paths = write_summaries(tmp_path, {
        0.0: [100.0, 101.0, 99.0],
        50.0: [103.0, 104.0, 102.0],
    })

    result = analyzer.analyze(paths)

    assert result.baseline_median_s == 100.0
    assert result.rotated_median_s == 103.0
    assert result.ratio == pytest.approx(1.03)
    assert result.passed


def test_quick_benchmark_rejects_more_than_five_percent(tmp_path):
    analyzer = load_analyzer()
    paths = write_summaries(tmp_path, {
        0.0: [100.0, 100.0, 100.0],
        50.0: [106.0, 106.0, 106.0],
    })

    assert not analyzer.analyze(paths).passed


def test_failed_or_wrong_strategy_runs_do_not_count(tmp_path):
    analyzer = load_analyzer()

    with pytest.raises(ValueError, match="three successful quick runs"):
        analyzer.analyze(write_mixed_summaries(tmp_path))


def test_duplicate_summary_path_cannot_satisfy_required_runs(tmp_path):
    analyzer = load_analyzer()
    paths = write_summaries(tmp_path, {
        0.0: [100.0],
        50.0: [103.0, 104.0, 102.0],
    })

    with pytest.raises(ValueError, match="duplicate summary input"):
        analyzer.analyze([paths[0], paths[0], paths[0], *paths[1:]])


def test_cli_returns_one_with_a_clear_error_for_insufficient_quick_runs(
    tmp_path, capsys,
):
    analyzer = load_analyzer()

    exit_code = analyzer.main([str(path) for path in write_mixed_summaries(tmp_path)])

    captured = capsys.readouterr()
    assert exit_code == 1
    assert "three successful quick runs" in captured.err


class _Logger:
    def info(self, _message):
        pass


class _SummaryLogger:
    def __init__(self):
        self.summary = None

    def write_summary(self, summary):
        self.summary = summary


class _Bridge:
    def stop_autonomous_output(self):
        pass


@pytest.mark.parametrize(
    ("quick_drop_mode", "expected_strategy"),
    [(True, "quick"), (False, "main")],
)
def test_completion_summary_uses_frozen_normalized_heading_and_strategy(
    quick_drop_mode, expected_strategy,
):
    fsm = type("CompletionFSM", (), {})()
    fsm._mission_failed = False
    fsm._drop_success_count = 1
    fsm._drop_count = 1
    fsm._recon_path_completed = True
    fsm._layout_frame = LayoutFrame(410.0)
    fsm.layout_frame = fsm._layout_frame
    fsm._task_logger = _SummaryLogger()
    fsm.bridge = _Bridge()
    fsm.get_param = lambda name, default=None: {
        "max_drops": 1,
        "quick_drop_mode": quick_drop_mode,
    }.get(name, default)
    fsm.get_coverage = lambda: None
    fsm.get_logger = lambda: _Logger()

    MissionFSM._handle_complete(fsm)

    assert fsm._task_logger.summary["layout_heading_deg"] == 50.0
    assert fsm._task_logger.summary["strategy"] == expected_strategy
