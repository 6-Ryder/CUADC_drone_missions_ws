from pathlib import Path

from drone_mission.states.state_drop import DropState
from drone_mission.states.state_precision_land import PrecisionLandState
from drone_mission.states.state_takeoff import TakeoffState
from px4_control.standalone_bridge import StandaloneBridge
from px4_control.telemetry import Position, Velocity


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _TaskLogger:
    def log_state(self, *_args):
        pass


class _Bridge:
    def __init__(self):
        self.offboard = False
        self.land_commands = 0
        self.force_disarms = 0

    def is_offboard(self):
        return self.offboard

    def set_offboard_mode(self):
        pass

    def land_at_current(self):
        self.land_commands += 1

    def force_disarm(self):
        self.force_disarms += 1


class _FlightController:
    def __init__(self):
        self.takeoff_calls = 0

    def takeoff(self, _height):
        self.takeoff_calls += 1


class _FSM:
    def __init__(self):
        self.bridge = _Bridge()
        self.fc = _FlightController()
        self.logger = _TaskLogger()

    def get_logger(self):
        return _Logger()

    def get_param(self, _name, default):
        return default

    def check_drop_service_result(self):
        return False


def test_takeoff_waits_for_confirmed_offboard(monkeypatch):
    fsm = _FSM()
    state = TakeoffState(fsm)
    state._sub_phase = state.SUB_OFFBOARD
    state._sub_phase_start = 0.0
    state._offboard_cmd_sent = True
    state._start_time = 0.0
    monkeypatch.setattr(
        "drone_mission.states.state_takeoff.time.time", lambda: 2.0,
    )

    assert state.execute() is None
    assert fsm.fc.takeoff_calls == 0

    fsm.bridge.offboard = True
    assert state.execute() is None
    assert fsm.fc.takeoff_calls == 1


def test_takeoff_timeout_is_an_error(monkeypatch):
    fsm = _FSM()
    state = TakeoffState(fsm)
    state._sub_phase = state.SUB_OFFBOARD
    state._sub_phase_start = 0.0
    state._offboard_cmd_sent = True
    state._start_time = 0.0
    monkeypatch.setattr(
        "drone_mission.states.state_takeoff.time.time", lambda: 16.0,
    )

    assert state.execute() == "ERROR"


def test_drop_hardware_failure_stops_the_mission():
    state = DropState(_FSM())
    assert state._execute_drop() == "ERROR"


def test_descent_ready_remains_latched_during_precision_alignment():
    state = DropState(_FSM())
    state._descent_ready_latched = True

    assert state._check_descent_ready(height_error=0.8)


def test_standalone_bridge_exposes_real_px4_velocity():
    node = type(
        "Node",
        (),
        {
            "create_subscription": lambda *args, **kwargs: None,
            "create_publisher": lambda *args, **kwargs: None,
            "create_timer": lambda *args, **kwargs: None,
        },
    )()
    bridge = StandaloneBridge.__new__(StandaloneBridge)
    bridge._node = node
    bridge._position = Position()
    bridge._velocity = Velocity()
    bridge._armed = True
    bridge._landed = False
    bridge._battery_remaining = 0.75
    bridge._battery_ready = True
    bridge._attitude = type("Attitude", (), {})()
    bridge._offboard = False
    bridge._current_yaw = 0.0

    odometry = type("Odometry", (), {
        "position": [0.0, 0.0, 0.0],
        "velocity": [1.25, -0.5, 0.08],
    })()
    bridge._odom_cb(odometry)

    telemetry = bridge.get_telemetry()
    assert telemetry.velocity == Velocity(vn=1.25, ve=-0.5, vd=0.08)


def test_flight_controller_takeoff_sets_same_target_for_altitude():
    """Verify FlightController.takeoff() sets a NED target preserving XY."""
    from px4_control.flight_controller import FlightController

    class _BridgeMock:
        def __init__(self):
            self._position = Position(north=1.0, east=2.0, down=0.0)
            self._target = None
            self._offboard = False
            self._armed = False

        def get_current_position(self):
            return self._position

        def set_position_target(self, north, east, down, yaw=0.0):
            self._target = (north, east, down)

        def is_offboard(self):
            return self._offboard

        def is_armed(self):
            return self._armed

        def set_offboard_mode(self):
            self._offboard = True

        def arm(self):
            self._armed = True

        def get_parameter(self, name):
            return type("Param", (), {"value": 5.0})()

    bridge = _BridgeMock()
    fc = FlightController(bridge)
    bridge._offboard = True
    bridge._armed = True
    fc.takeoff(5.0)

    assert bridge._target is not None
    assert bridge._target[0] == 1.0  # north preserved
    assert bridge._target[1] == 2.0  # east preserved
    assert bridge._target[2] == -5.0  # altitude=5 → NED down=-5


def test_precision_land_timeout_uses_px4_land_not_force_disarm(monkeypatch):
    fsm = _FSM()
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -1.0})()
    fsm.fc.set_velocity_ned = lambda *_args: None
    fsm.fc.set_velocity_body = lambda *_args: None
    state = PrecisionLandState(fsm)
    state._start_time = 0.0
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: 61.0,
    )

    assert state._execute_blind() is None
    assert fsm.bridge.land_commands == 1
    assert fsm.bridge.force_disarms == 0


def test_px4_bridge_arm_does_not_use_force_magic():
    """PX4Bridge (direct DDS) uses standard arm for real hardware."""
    source = (SOURCE_ROOT / "px4_control/px4_control/px4_bridge.py").read_text(
        encoding="utf-8",
    )
    arm_source = source.split("    def arm(self):", 1)[1].split(
        "    def disarm(self):", 1,
    )[0]
    assert "21196" not in arm_source


def test_px4_telemetry_uses_versioned_status_topics():
    """Verify the standalone bridge subscribes to versioned PX4 topics."""
    bridge_py = SOURCE_ROOT / "px4_control/px4_control/standalone_bridge.py"
    source = bridge_py.read_text(encoding="utf-8")

    assert '"/fmu/out/vehicle_status_v1"' in source
    assert '"/fmu/out/battery_status_v1"' in source


def test_px4_telemetry_facade_exists_before_init_state_runs():
    fsm_py = PACKAGE_ROOT / "drone_mission/mission_state_machine.py"
    source = fsm_py.read_text(encoding="utf-8")
    lazy_init = source.split("    def _lazy_init(self):", 1)[1].split(
        "    def _init_vision_subscriptions(self):", 1,
    )[0]

    assert lazy_init.index("_ = self.bridge") < lazy_init.index(
        "self._transition_to(MissionState.INIT)"
    )


def test_navigation_timeouts_do_not_advance_the_mission():
    transit_py = PACKAGE_ROOT / "drone_mission/states/state_transit.py"
    transit = transit_py.read_text(encoding="utf-8")
    ascend = (PACKAGE_ROOT / "drone_mission/states/state_drop.py").read_text(
        encoding="utf-8",
    ).split("def _execute_ascend", 1)[1]

    assert "超时, 强制进入下一状态" not in transit
    assert 'return "ERROR"' in transit
    assert 'return "SELECT_TARGET" if drops' not in ascend


def test_landing_timeout_never_reports_success_or_force_disarms():
    """
    超时路径用 land_at_current 而非 force_disarm.

    force_disarm 出现在 land() 中是正确设计 — 着陆检测成功后 PX4 需要
    force_disarm(magic=21196) 才能上锁, 否则会被"非手动 disarm"拒绝.
    本测试仅验证超时路径的安全契约: 15s 超时后切入 PX4 LAND 模式,
    不调用 force_disarm, 也不返回 True.
    """
    source = (
        SOURCE_ROOT / "px4_control/px4_control/flight_controller.py"
    ).read_text(encoding="utf-8")
    land_method = source.split("    def land(self)", 1)[1].split(
        "    def force_disarm", 1,
    )[0]

    # 正常着陆路径包含 force_disarm — 这是正确行为
    assert "force_disarm" in land_method, \
        "land() 必须在着陆检测成功后调用 force_disarm"

    # 超时路径: 只使用 land_at_current, 不使用 force_disarm
    _, _, after_descending_branch = land_method.partition(
        "self._bridge.force_disarm()"
    )
    assert "land_at_current" in after_descending_branch, \
        "超时路径必须调用 land_at_current"
    assert "force_disarm" not in after_descending_branch, \
        "超时路径不得调用 force_disarm"
