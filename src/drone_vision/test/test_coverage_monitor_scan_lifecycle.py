import pytest

from drone_vision.coverage_monitor import CoverageMonitor


def test_coverage_only_accumulates_during_an_explicit_recon_scan():
    """Transit observations must not count toward reconnaissance coverage."""
    monitor = CoverageMonitor(
        zone_x_range=(0.0, 8.0),
        zone_y_range=(65.0, 70.0),
        grid_resolution=0.5,
        fov_half_angle_deg=35.0,
    )

    # The mission crosses the zone at cruise altitude before the scan starts.
    assert monitor.update(4.0, 67.5, 5.0) == 0.0

    monitor.begin_scan()
    during_scan = monitor.update(4.0, 67.5, 2.5)
    assert during_scan > 0.0

    monitor.end_scan()
    assert monitor.update(0.0, 65.0, 5.0) == pytest.approx(during_scan)
