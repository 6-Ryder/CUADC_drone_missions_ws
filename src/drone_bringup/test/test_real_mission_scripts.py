"""Source contracts for the single safe real-mission entry path."""

from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
LAUNCHER_PATH = WORKSPACE / "scripts/start_full_real_mission.sh"
QUICK_LAUNCHER_PATH = WORKSPACE / "scripts/start_quick_mission_real.sh"
VERIFIER_PATH = WORKSPACE / "scripts/verify_real_flight_ready.sh"


def _source(path: Path) -> str:
    assert path.is_file(), f"required script is missing: {path}"
    return path.read_text(encoding="utf-8")


def test_full_real_launcher_owns_one_udp_agent_and_runs_the_mission():
    """
    The start script (not the launch) owns the only MicroXRCEAgent.

    test_mission 模式: 脚本先清理旧进程、启动 Agent 并等待 DDS 就绪,
    再由 launch 启动任务节点。Agent 意外退出时脚本停止任务 (等价旧 exit guard).
    """
    source = _source(LAUNCHER_PATH)

    # set -u 会与 /opt/ros/humble/setup.bash 的未保护变量冲突 (与 test_mission 同为 -eo)
    assert "set -eo pipefail" in source
    assert "set -u" not in source
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
    # 相机检查与 test_mission 一致: 不设脚本级检查 (相机参数仍透传 launch)
    assert "CAMERA_WAIT_SECONDS" not in source
    assert "相机设备" not in source
    # 清理契约: EXIT trap 回收 Agent, 避免 Ctrl+C 残留
    assert "trap cleanup EXIT" in source
    # 自启可观测性: 脚本全部输出镜像到 startup.log (消灭"日志空"盲区);
    # DDS 等待时长可配置 (默认 60, 与 test_mission 一致)
    assert 'tee -a "$LOG_DIR/startup.log"' in source
    assert "DDS_WAIT_SECONDS" in source


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


def test_real_mission_launchers_forward_arbitrary_launch_arguments():
    for path in (LAUNCHER_PATH, QUICK_LAUNCHER_PATH):
        source = _source(path)

        assert 'LAUNCH_ARGS+=("$arg")' in source
        assert '"${LAUNCH_ARGS[@]}"' in source


def test_quick_real_launcher_defaults_to_thirty_degrees_and_allows_override():
    source = _source(QUICK_LAUNCHER_PATH)

    assert 'LAYOUT_HEADING_DEG="30.0"' in source
    assert 'layout_heading_deg:=*' in source
    assert 'LAYOUT_HEADING_DEG="${arg#layout_heading_deg:=}"' in source
    assert 'HAS_LAYOUT_HEADING=1' in source
    assert 'LAUNCH_ARGS+=("layout_heading_deg:=${LAYOUT_HEADING_DEG}")' in source


def test_readiness_verifier_is_bounded_read_only_and_checks_real_interfaces():
    source = _source(VERIFIER_PATH)

    assert "set -eo pipefail" in source
    assert "set -u" not in source
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
