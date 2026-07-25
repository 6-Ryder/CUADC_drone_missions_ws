from pathlib import Path


BRIDGE_SOURCE = (
    Path(__file__).resolve().parents[1] / "px4_control/px4_bridge.py"
)
TELEMETRY_SOURCE = (
    Path(__file__).resolve().parents[1] / "px4_control/telemetry.py"
)


def test_px4_bridge_uses_standard_navigation_commands():
    """标准导航指令常量定义在 telemetry.py 中, px4_bridge 负责导入使用."""
    telemetry = TELEMETRY_SOURCE.read_text(encoding="utf-8")
    bridge = BRIDGE_SOURCE.read_text(encoding="utf-8")
    # 常量定义在 telemetry.py 中 (单一事实来源)
    assert "VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20" in telemetry
    assert "VEHICLE_CMD_NAV_LAND = 21" in telemetry
    # px4_bridge 从 telemetry 导入这些常量
    assert "from px4_control.telemetry import" in bridge
    assert "VEHICLE_CMD_NAV_RETURN_TO_LAUNCH" in bridge
    assert "VEHICLE_CMD_NAV_LAND" in bridge


def test_px4_bridge_consumes_vision_servo_and_battery_topics():
    source = BRIDGE_SOURCE.read_text(encoding="utf-8")
    assert '"/drone/vision/servo_cmd"' in source
    assert '"/fmu/out/battery_status_v1"' in source


def test_fake_registration_message_requests_are_removed():
    source = BRIDGE_SOURCE.read_text(encoding="utf-8")
    assert "_registration_timer" not in source
    assert "_send_request_message" not in source
