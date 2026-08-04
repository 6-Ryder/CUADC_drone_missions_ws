from pathlib import Path
import threading

import px4_control.standalone_bridge as standalone_bridge_module
from px4_control.standalone_bridge import StandaloneBridge
from px4_control.px4_bridge import PX4Bridge
from px4_control.telemetry import Position


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


class _Publisher:
    def __init__(self, callback=None):
        self.messages = []
        self._callback = callback

    def publish(self, message):
        self.messages.append(message)
        if self._callback is not None:
            self._callback(message)


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class _Node:
    def __init__(self):
        self.publishers = {}
        self.publisher_callbacks = {}

    def create_publisher(self, _message_type, topic, _qos):
        publisher = _Publisher(self.publisher_callbacks.get(topic))
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, *_args, **_kwargs):
        return object()

    def create_timer(self, *_args, **_kwargs):
        return object()

    def get_parameter(self, _name):
        return type(
            "Parameter", (), {"value": "/fmu/out/vehicle_status_v1"},
        )()

    def get_logger(self):
        return _Logger()


def _bridge():
    node = _Node()
    return StandaloneBridge(node, sim_mode=False), node


class _Clock:
    class _Now:
        nanoseconds = 1_000_000

        @staticmethod
        def to_msg():
            from builtin_interfaces.msg import Time

            return Time(sec=0, nanosec=1_000_000)

    @staticmethod
    def now():
        return _Clock._Now()


def _px4_relay_bridge():
    bridge = PX4Bridge.__new__(PX4Bridge)
    bridge._setpoint_lock = threading.RLock()
    bridge._autonomy_lease_enabled = False
    bridge._autonomy_lease_last_seen = None
    bridge._autonomy_lease_timeout_s = 0.30
    bridge._autonomy_target_received = False
    bridge._target_setpoint = Position(0.0, 0.0, -5.0)
    bridge._current_setpoint = Position(0.0, 0.0, -5.0)
    bridge._target_yaw = 0.0
    bridge._target_speed = None
    bridge._smoothed_yaw = 0.0
    bridge._velocity_mode = False
    bridge._last_velocity_cmd = (0.0, 0.0, 0.0)
    bridge._offboard_mode_pub = _Publisher()
    bridge._trajectory_pub = _Publisher()
    bridge._vel_pub = _Publisher()
    bridge.get_clock = lambda: _Clock()
    bridge.get_logger = lambda: _Logger()
    bridge.get_parameter = lambda _name: type("Parameter", (), {"value": 5.0})()
    return bridge


def _position_command():
    return type(
        "NedPositionSetpoint",
        (), {
            "north": 10.0,
            "east": -2.0,
            "down": -3.0,
            "yaw": 0.2,
            "speed": 0.8,
            "use_speed": True,
        },
    )()


def test_px4_relay_defaults_to_no_downstream_autonomy_output():
    relay = _px4_relay_bridge()

    relay._offboard_heartbeat()

    assert not relay._offboard_mode_pub.messages
    assert not relay._trajectory_pub.messages


def test_standalone_stop_revokes_the_downstream_px4_autonomy_lease(monkeypatch):
    relay = _px4_relay_bridge()
    node = _Node()
    node.publisher_callbacks["/drone/cmd/autonomy_lease"] = (
        lambda message: relay._autonomy_lease_cb(message)
    )
    node.publisher_callbacks["/drone/cmd_setpoint_ned"] = relay._cmd_setpoint_cb
    standalone = StandaloneBridge(node, sim_mode=False)
    monkeypatch.setattr(standalone_bridge_module.time, "monotonic", lambda: 100.0)
    monkeypatch.setattr("px4_control.px4_bridge.time.monotonic", lambda: 100.0)

    standalone.set_position_target(10.0, -2.0, -3.0, yaw=0.2, speed=0.8)
    standalone._heartbeat()
    relay._offboard_heartbeat()
    assert len(relay._offboard_mode_pub.messages) == 1
    assert len(relay._trajectory_pub.messages) == 1

    standalone.stop_autonomous_output()
    relay._offboard_heartbeat()

    assert len(relay._offboard_mode_pub.messages) == 1
    assert len(relay._trajectory_pub.messages) == 1
    assert relay._target_setpoint is None
    assert relay._last_velocity_cmd == (0.0, 0.0, 0.0)


def test_expired_lease_blocks_vision_servo_from_bypassing_px4_output_gate(monkeypatch):
    from geometry_msgs.msg import TwistStamped

    relay = _px4_relay_bridge()
    relay._autonomy_lease_enabled = True
    relay._autonomy_lease_last_seen = 100.0
    relay._autonomy_target_received = True
    command = TwistStamped()
    command.twist.linear.x = 0.4
    command.twist.linear.y = -0.2
    command.twist.linear.z = 0.1
    monkeypatch.setattr("px4_control.px4_bridge.time.monotonic", lambda: 100.0)

    relay._vision_velocity_cb(command)
    assert len(relay._trajectory_pub.messages) == 1

    monkeypatch.setattr("px4_control.px4_bridge.time.monotonic", lambda: 100.31)
    relay._vision_velocity_cb(command)
    relay._offboard_heartbeat()

    assert len(relay._trajectory_pub.messages) == 1
    assert not relay._offboard_mode_pub.messages
    assert relay._target_setpoint is None


def test_px4_bridge_source_declares_a_short_fail_closed_lease_timeout():
    source = BRIDGE_SOURCE.read_text(encoding="utf-8")

    assert 'declare_parameter("autonomy_lease_timeout_s", 0.30)' in source
    assert "_autonomy_lease_enabled: bool = False" in source
    assert '"/drone/cmd/autonomy_lease"' in source


def test_telemetry_fresh_requires_status_and_odometry_independently(monkeypatch):
    bridge, _node = _bridge()
    assert not bridge.telemetry_fresh(now=10.0)

    status = type(
        "Status",
        (), {"arming_state": 2, "nav_state": 14, "pre_flight_checks_pass": True},
    )()
    monkeypatch.setattr(standalone_bridge_module.time, "monotonic", lambda: 10.0)
    bridge._status_cb(status)
    assert not bridge.telemetry_fresh(now=10.0)

    odometry = type(
        "Odometry",
        (), {"position": [1.0, 2.0, -3.0], "velocity": [0.1, 0.2, 0.3]},
    )()
    monkeypatch.setattr(standalone_bridge_module.time, "monotonic", lambda: 11.0)
    bridge._odom_cb(odometry)

    assert bridge.telemetry_fresh(now=11.5, max_age_s=2.0)
    assert not bridge.telemetry_fresh(now=12.1, max_age_s=2.0)

    monkeypatch.setattr(standalone_bridge_module.time, "monotonic", lambda: 13.0)
    bridge._status_cb(status)
    assert not bridge.telemetry_fresh(now=13.1, max_age_s=2.0)


def test_replayed_px4_source_samples_do_not_refresh_telemetry(monkeypatch):
    bridge, _node = _bridge()
    status = type(
        "Status",
        (), {
            "timestamp": 1_000_000,
            "arming_state": 2,
            "nav_state": 14,
            "pre_flight_checks_pass": True,
        },
    )()
    odometry = type(
        "Odometry",
        (), {
            "timestamp": 1_000_000,
            "timestamp_sample": 900_000,
            "position": [1.0, 2.0, -3.0],
            "velocity": [0.1, 0.2, 0.3],
        },
    )()

    monkeypatch.setattr(standalone_bridge_module.time, "monotonic", lambda: 10.0)
    bridge._status_cb(status)
    bridge._odom_cb(odometry)
    assert bridge.telemetry_fresh(now=11.0, max_age_s=2.0)

    monkeypatch.setattr(standalone_bridge_module.time, "monotonic", lambda: 20.0)
    bridge._status_cb(status)
    bridge._odom_cb(odometry)

    assert not bridge.telemetry_fresh(now=20.0, max_age_s=2.0)


def test_land_readiness_requires_a_real_land_detection_sample():
    bridge, _node = _bridge()

    assert not bridge.land_ready
    bridge._land_cb(type("LandDetected", (), {"landed": True})())

    assert bridge.land_ready


def test_hold_for_takeover_publishes_zero_velocity_with_output_enabled():
    bridge, node = _bridge()
    bridge.set_position_target(4.0, 5.0, -3.0)

    bridge.hold_for_takeover()
    bridge._heartbeat()

    assert bridge.autonomous_output_enabled
    assert bridge._target_ned is None
    assert bridge._velocity_mode
    assert bridge._vel_target == (0.0, 0.0, 0.0)
    messages = node.publishers["/drone/cmd_velocity_ned"].messages
    assert len(messages) == 1
    assert (messages[0].linear.x, messages[0].linear.y, messages[0].linear.z) == (
        0.0, 0.0, 0.0,
    )


def test_stop_autonomous_output_is_latched_and_clears_all_targets():
    bridge, node = _bridge()
    bridge.set_position_target(4.0, 5.0, -3.0)

    bridge.stop_autonomous_output()
    bridge.set_position_target(1.0, 2.0, -4.0)
    bridge._heartbeat()
    bridge.set_velocity_target(0.5, -0.2, 0.1)
    bridge._heartbeat()

    assert not bridge.autonomous_output_enabled
    assert bridge._target_ned is None
    assert bridge._target_speed is None
    assert not bridge._velocity_mode
    assert bridge._vel_target == (0.0, 0.0, 0.0)
    assert not node.publishers["/drone/cmd_setpoint_ned"].messages
    assert not node.publishers["/drone/cmd_velocity_ned"].messages
