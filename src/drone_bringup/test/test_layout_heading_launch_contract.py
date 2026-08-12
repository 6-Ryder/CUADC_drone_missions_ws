"""Contracts for the single layout-heading launch input."""

from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
LAUNCH_DIR = WORKSPACE / "src/drone_bringup/launch"
QUICK_REAL_SCRIPT = WORKSPACE / "scripts/start_quick_mission_real.sh"
FULL_REAL_SCRIPT = WORKSPACE / "scripts/start_full_real_mission.sh"


@pytest.mark.parametrize("launch_name", [
    "quick_sitl.launch.py", "sitl.launch.py",
    "quick_flight.launch.py", "real_flight.launch.py",
])
def test_every_complete_profile_has_one_layout_heading_and_no_legacy_direction(
    launch_name,
):
    source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "layout_heading_deg"' in source
    assert source.count(
        '"layout_heading_deg": LaunchConfiguration("layout_heading_deg")'
    ) == 2
    assert "flight_axis" not in source
    assert "drop_distance" not in source
    assert "recon_distance" not in source


@pytest.mark.parametrize("launch_name", [
    "quick_sitl.launch.py", "sitl.launch.py",
    "quick_flight.launch.py", "real_flight.launch.py",
])
def test_every_vision_pipeline_receives_home_and_layout_heading(launch_name):
    source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")

    for parameter in ("home_x", "home_y", "layout_heading_deg"):
        assert source.count(
            f'"{parameter}": LaunchConfiguration("{parameter}")'
        ) >= 2


@pytest.mark.parametrize("script", [QUICK_REAL_SCRIPT, FULL_REAL_SCRIPT])
def test_real_wrapper_forwards_layout_heading_as_a_launch_argument(script):
    source = script.read_text(encoding="utf-8")

    assert 'LAUNCH_ARGS+=("$arg")' in source
    assert '"${LAUNCH_ARGS[@]}"' in source
