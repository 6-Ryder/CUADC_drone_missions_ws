import numpy as np
import pytest
from types import SimpleNamespace

from drone_utils.layout_frame import LayoutFrame
from drone_vision.coverage_monitor import CoverageMonitor
from drone_vision.vision_pipeline import VisionPipelineNode


def cover_path(monitor, frame, local_points, home=(4.0, 10.0)):
    monitor.begin_scan()
    for forward, right in local_points:
        north, east = frame.local_to_ned(forward, right)
        monitor.update(home[0] + east, home[1] + north, 2.5)
    return monitor.coverage_ratio, monitor.grid.copy()


def test_zero_and_fifty_degree_coverage_are_identical():
    """A rigid field rotation must not change the local coverage grid."""
    points = [
        (55.0, -4.0), (55.0, 4.0), (57.5, 4.0),
        (57.5, -4.0), (60.0, -4.0), (60.0, 4.0),
    ]
    zero_frame = LayoutFrame(0.0)
    fifty_frame = LayoutFrame(50.0)
    zero = CoverageMonitor((55.0, 60.0), (-4.0, 4.0), zero_frame, (4.0, 10.0))
    fifty = CoverageMonitor(
        (55.0, 60.0), (-4.0, 4.0), fifty_frame, (4.0, 10.0),
    )

    ratio_zero, grid_zero = cover_path(zero, zero_frame, points)
    ratio_fifty, grid_fifty = cover_path(fifty, fifty_frame, points)

    assert ratio_fifty == pytest.approx(ratio_zero)
    assert np.array_equal(grid_fifty, grid_zero)


def test_vision_pipeline_rejects_runtime_layout_heading_change():
    """Coverage must retain the same heading snapshot as MissionFSM."""
    node = object.__new__(VisionPipelineNode)

    result = node._reject_layout_heading_update([
        SimpleNamespace(name="layout_heading_deg", value=50.0),
    ])

    assert result.successful is False
    assert "fixed" in result.reason
