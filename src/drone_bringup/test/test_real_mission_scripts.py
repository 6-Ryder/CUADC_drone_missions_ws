"""Source contracts for the single safe real-mission entry path."""

from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
LAUNCHER_PATH = WORKSPACE / "scripts/start_full_real_mission.sh"
VERIFIER_PATH = WORKSPACE / "scripts/verify_real_flight_ready.sh"


def _source(path: Path) -> str:
    assert path.is_file(), f"required script is missing: {path}"
    return path.read_text(encoding="utf-8")


def test_full_real_launcher_has_one_safe_launch_owner_and_run_log():
    source = _source(LAUNCHER_PATH)

    assert "set -euo pipefail" in source
    assert "real_mission_" in source
    assert 'export ROS_LOG_DIR="$LOG_DIR/ros"' in source
    assert 'ros2 launch drone_bringup real_flight.launch.py "$@"' in source
    assert "command -v MicroXRCEAgent" in source
    assert "MicroXRCEAgent udp4" not in source
    assert "MicroXRCEAgent serial" not in source
    assert "pkill" not in source
    assert "sitl_env.sh" not in source


def test_full_real_launcher_never_issues_vehicle_or_servo_commands():
    source = _source(LAUNCHER_PATH)

    for forbidden in (
        "ros2 topic pub",
        "ros2 service call",
        "VEHICLE_CMD",
        "force_disarm",
        "return_to_launch",
        "land_at_current",
        "trigger_drop",
    ):
        assert forbidden not in source


def test_readiness_verifier_is_bounded_read_only_and_checks_real_interfaces():
    source = _source(VERIFIER_PATH)

    assert "set -euo pipefail" in source
    assert "timeout" in source
    assert "ros2 topic echo" in source
    assert "ros2 topic info" in source
    assert "ros2 service list" in source
    assert '"/fmu/out/vehicle_status_v1"' in source
    assert '"/fmu/out/vehicle_odometry"' in source
    assert '"/fmu/out/battery_status_v1"' in source
    assert '"/fmu/out/vehicle_land_detected"' in source
    assert '"/fmu/out/vehicle_command_ack"' in source
    assert '"/drone/servo/status"' in source
    assert '"/drone/camera/camera_info"' in source
    assert '"/drone/vision/status"' in source
    assert '"/drone/servo/trigger_drop"' in source
    assert "width: 640" in source
    assert "height: 480" in source
    assert "400.0" in source
    assert "320.0" in source
    assert "240.0" in source
    assert '舵机节点尚未 ready=1' in source

    for forbidden in (
        "ros2 topic pub",
        "ros2 service call",
        "MicroXRCEAgent",
        "pkill",
        "force_disarm",
        "trigger_drop ",
    ):
        assert forbidden not in source
