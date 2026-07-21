from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[3] / "scripts/start_full_simulation.sh"
).read_text(encoding="utf-8")


def test_simulation_runs_gazebo_without_gui():
    assert "export HEADLESS=1" in SCRIPT


def test_px4_runs_without_interactive_shell():
    assert "./bin/px4 -d -i 0" in SCRIPT


def test_mission_owns_arming_and_offboard_transition():
    assert "px4-commander\" --instance 0 arm" not in SCRIPT
    assert "px4-commander\" --instance 0 mode offboard" not in SCRIPT


def test_workspace_world_is_installed_before_px4_starts():
    assert '"$PROJ/field/drone_field.sdf"' in SCRIPT
    assert '"$PX4_DIR/Tools/simulation/gz/worlds/drone_field.sdf"' in SCRIPT


def test_headless_sitl_does_not_require_a_gcs_link():
    assert "set NAV_DLL_ACT 0" in SCRIPT
