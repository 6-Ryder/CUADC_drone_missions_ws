"""Source contracts for the single safe real-mission entry path."""

from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
LAUNCHER_PATH = WORKSPACE / "scripts/start_full_real_mission.sh"
VERIFIER_PATH = WORKSPACE / "scripts/verify_real_flight_ready.sh"


def _source(path: Path) -> str:
    assert path.is_file(), f"required script is missing: {path}"
    return path.read_text(encoding="utf-8")


def test_full_real_launcher_owns_one_udp_agent_and_runs_the_mission():
    """The start script (not the launch) owns the only MicroXRCEAgent.

    test_mission 模式: 脚本先清理旧进程、启动 Agent 并等待 DDS 就绪,
    再由 launch 启动任务节点。Agent 意外退出时脚本停止任务 (等价旧 exit guard).
    """
    source = _source(LAUNCHER_PATH)

    assert "set -euo pipefail" in source
    assert "real_mission_" in source
    assert 'export ROS_LOG_DIR="$LOG_DIR/ros"' in source
    assert 'ros2 launch drone_bringup real_flight.launch.py "${LAUNCH_ARGS[@]}"' in source
    # 脚本持有唯一 MicroXRCEAgent, 且先于任务节点启动
    assert 'MicroXRCEAgent udp4 -p "$DDS_PORT"' in source
    assert "MicroXRCEAgent serial" not in source
    assert "command -v MicroXRCEAgent" in source
    # Step 0 清理旧进程, 保证 UDP 端口唯一 (自动启动环境也安全)
    assert "pkill -9 -f \"MicroXRCEAgent\"" in source
    # Agent 意外退出 → 脚本停止任务节点 (watchdog)
    assert "WATCHDOG_PID" in source
    assert "已退出, 停止任务节点" in source
    # 自动启动模式: 仅持有 Agent, 前台运行适配 systemd Type=simple
    assert "--agent-only" in source
    # 环境自愈: SITL overlay 从环境剥离, 不再硬失败阻断 Agent
    assert "sitl_px4_msgs" in source
    assert "已从环境剥离" in source
    # 相机检查限时等待 + 警告, 不得阻断 Agent 与 DDS
    assert "CAMERA_WAIT_SECONDS" in source
    # 清理契约: EXIT trap 回收 Agent, 避免 Ctrl+C 残留
    assert "trap cleanup EXIT" in source


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
