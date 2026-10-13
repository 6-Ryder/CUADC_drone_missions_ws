"""Contracts for the single layout-heading launch input."""

from pathlib import Path
import re

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
    assert source.count('"layout_heading_deg": ParameterValue(') == 2
    assert "flight_axis" not in source
    assert "drop_distance" not in source
    assert "recon_distance" not in source


@pytest.mark.parametrize("launch_name", [
    "quick_sitl.launch.py", "sitl.launch.py",
    "quick_flight.launch.py", "real_flight.launch.py",
])
def test_every_vision_pipeline_receives_home_and_layout_heading(launch_name):
    """home_x/home_y/layout_heading_deg 都必须以**带类型**的形式下发.

    这三者在 vision_pipeline 里都声明为 double。裸传
    `"home_x": LaunchConfiguration("home_x")` 会把 launch 参数以字符串下发,
    rclpy 抛 InvalidParameterTypeException 并**打死节点** —— 真机证据:
    2026-07-23 `px4_bridge` 被 `home_z` 的 STRING vs DOUBLE 打死 (exit code 1)。

    本条与隔壁 test_heading_launch_override_is_always_typed_as_double 同一标准
    (那条是为了 CLI 覆盖值 0/50 的同类崩溃而加的), 现在把 home_* 也纳入。
    """
    source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")

    for parameter in ("home_x", "home_y", "layout_heading_deg"):
        # layout_heading_deg 恰好下发 2 次 (vision + mission);
        # home_x/home_y 下发 3 次 (bridge + vision + mission), 故用下界。
        expected = 2 if parameter == "layout_heading_deg" else 3
        assert source.count(f'"{parameter}": ParameterValue(') == expected, (
            f"{launch_name} 里的 {parameter} 必须写成 "
            f'ParameterValue(LaunchConfiguration("{parameter}"), value_type=float), '
            f"且出现 {expected} 次"
        )


@pytest.mark.parametrize("launch_name", [
    "quick_sitl.launch.py", "sitl.launch.py",
    "quick_flight.launch.py", "real_flight.launch.py",
])
def test_heading_launch_override_is_always_typed_as_double(launch_name):
    """CLI values such as 0/50 must match the nodes' declared double type."""
    source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")

    assert "from launch_ros.parameter_descriptions import ParameterValue" in source
    heading_values = re.findall(
        r'ParameterValue\(\s*LaunchConfiguration\("layout_heading_deg"\),\s*value_type=float',
        source,
    )
    assert len(heading_values) == 2


@pytest.mark.parametrize("script", [QUICK_REAL_SCRIPT, FULL_REAL_SCRIPT])
def test_real_wrapper_forwards_layout_heading_as_a_launch_argument(script):
    source = script.read_text(encoding="utf-8")

    assert 'LAUNCH_ARGS+=("$arg")' in source
    assert '"${LAUNCH_ARGS[@]}"' in source
