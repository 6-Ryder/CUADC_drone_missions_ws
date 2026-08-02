"""PX4 DDS interface contracts for the SITL and real-flight deployments."""

from pathlib import Path
import subprocess


WORKSPACE = Path(__file__).resolve().parents[3]
SCRIPTS = WORKSPACE / "scripts"
LAUNCH = WORKSPACE / "src/drone_bringup/launch"
CONFIG = WORKSPACE / "src/px4_control/config/px4_params.yaml"


def test_sitl_uses_its_own_px4_msgs_overlay():
    """SITL must not replace the real-flight v1 px4_msgs source package."""
    startup = (SCRIPTS / "start_full_simulation.sh").read_text(encoding="utf-8")
    builder = (SCRIPTS / "build_sitl_px4_msgs.sh").read_text(encoding="utf-8")

    assert 'source "$PROJ/scripts/sitl_env.sh"' in startup
    assert 'mktemp -d' in builder
    assert 'msg/versioned' in builder
    assert 'INSTALL_BASE="$PROJ/install/sitl_px4_msgs"' in builder
    assert '--install-base "$INSTALL_BASE"' in builder


def test_sitl_environment_imports_vehicle_status_v4_after_workspace_setup():
    """SITL must promote v4 even when the workspace already exposes real v1."""
    result = subprocess.run(
        [
            "bash",
            "-c",
            f"source '{SCRIPTS}/sitl_env.sh'; "
            "python3 -c 'from px4_msgs.msg import VehicleStatus; "
            "print(VehicleStatus.MESSAGE_VERSION)'",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert result.stdout.strip() == "4"


def test_sitl_px4_msgs_builder_sources_ros_before_enabling_nounset():
    """ROS Humble setup reads optional variables and cannot run under `set -u`."""
    builder = (SCRIPTS / "build_sitl_px4_msgs.sh").read_text(encoding="utf-8")

    assert builder.index('source /opt/ros/humble/setup.bash') < builder.index('set -u')


def test_sitl_px4_msgs_builder_passes_colcon_global_options_before_build():
    """colcon accepts --log-base only as a global option."""
    builder = (SCRIPTS / "build_sitl_px4_msgs.sh").read_text(encoding="utf-8")

    assert 'colcon --log-base "$LOG_BASE" build' in builder


def test_sitl_px4_msgs_builder_cleans_its_ephemeral_source_cache():
    """Each run stages a new source directory, so the isolated cache is stale."""
    builder = (SCRIPTS / "build_sitl_px4_msgs.sh").read_text(encoding="utf-8")

    assert '--cmake-clean-cache' in builder


def test_sitl_launch_passes_the_v4_status_topic_to_both_bridges():
    """The relay bridge and the mission's standalone bridge need one topic choice."""
    source = (LAUNCH / "sitl.launch.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n        "vehicle_status_topic",\n        default_value="/fmu/out/vehicle_status_v4"' in source
    assert source.count('"vehicle_status_topic": LaunchConfiguration("vehicle_status_topic")') == 2


def test_real_flight_default_remains_the_verified_v1_status_topic():
    """The source workspace remains compatible with the deployed real vehicle."""
    source = CONFIG.read_text(encoding="utf-8")

    assert 'vehicle_status_topic: "/fmu/out/vehicle_status_v1"' in source


def test_real_flight_launch_uses_udp_agent_for_the_ethernet_px4_client():
    """The deployed PX4 uses Ethernet uXRCE-DDS, not a Jetson UART."""
    source = (LAUNCH / "real_flight.launch.py").read_text(encoding="utf-8")

    assert '"dds_port", default_value="8888"' in source
    assert '"MicroXRCEAgent", "udp4", "-p",' in source
    assert 'LaunchConfiguration("dds_port")' in source
    assert '"MicroXRCEAgent", "serial"' not in source
    assert '"serial_device"' not in source


def test_real_flight_stops_when_its_udp_agent_cannot_own_the_port():
    """A duplicate Agent must not leave flight nodes running without PX4 DDS."""
    source = (LAUNCH / "real_flight.launch.py").read_text(encoding="utf-8")

    assert "OnProcessExit" in source
    assert "target_action=xrce_agent" in source
    assert "Shutdown(" in source


def test_real_hardware_scripts_require_ethernet_udp_and_v1_status_topic():
    """No real-flight entry point may silently select the obsolete serial path."""
    for name in ("simple_mission_real.sh", "test_forward_recon.sh"):
        source = (SCRIPTS / name).read_text(encoding="utf-8")

        assert 'MicroXRCEAgent udp4 -p "$DDS_PORT"' in source
        assert "MicroXRCEAgent serial" not in source
        assert "DDS_TRANSPORT" not in source
        assert '"/fmu/out/vehicle_status_v1"' in source
