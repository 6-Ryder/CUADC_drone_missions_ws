from pathlib import Path


BRIDGE_SOURCE = (
    Path(__file__).resolve().parents[1] / "px4_control/px4_bridge.py"
)


def test_px4_bridge_uses_standard_navigation_commands():
    source = BRIDGE_SOURCE.read_text(encoding="utf-8")
    assert "VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20" in source
    assert "VEHICLE_CMD_NAV_LAND = 21" in source


def test_px4_bridge_consumes_vision_servo_and_battery_topics():
    source = BRIDGE_SOURCE.read_text(encoding="utf-8")
    assert '"/drone/vision/servo_cmd"' in source
    assert '"/fmu/out/battery_status_v1"' in source


def test_fake_registration_message_requests_are_removed():
    source = BRIDGE_SOURCE.read_text(encoding="utf-8")
    assert "_registration_timer" not in source
    assert "_send_request_message" not in source
