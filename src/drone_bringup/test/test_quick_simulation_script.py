"""Contracts shared by the quick and main SITL wrapper scripts."""

from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
QUICK_SCRIPT = WORKSPACE / "scripts/start_quick_simulation.sh"
FULL_SCRIPT = WORKSPACE / "scripts/start_full_simulation.sh"


@pytest.mark.parametrize("script", [QUICK_SCRIPT, FULL_SCRIPT])
def test_simulation_wrapper_resolves_one_heading_for_field_model_and_launch(
    script,
):
    source = script.read_text(encoding="utf-8")

    assert "layout_heading_deg:=*" in source
    assert "field/layout_field.py" in source
    assert 'PX4_GZ_MODEL_POSE="4,10,0.3,0,0,$GAZEBO_YAW_RAD"' in source
    assert "ros2 launch" in source and '"$@"' in source
    assert 'cp "$PROJ/field/drone_field.sdf" "$PX4_WORLD"' not in source
