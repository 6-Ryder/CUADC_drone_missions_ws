from pathlib import Path

from drone_mission.mission_state_machine import (
    _StandaloneBridgeFacade,
    _StandaloneFlightFacade,
)
from drone_mission.states.state_drop import DropState
from px4_control.telemetry import Position, Velocity
from drone_mission.states.state_precision_land import PrecisionLandState
from drone_mission.states.state_takeoff import TakeoffState


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
    monkeypatch.setattr("drone_mission.states.state_takeoff.time.time", lambda: 2.0)

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
    monkeypatch.setattr("drone_mission.states.state_takeoff.time.time", lambda: 16.0)

    assert state.execute() == "ERROR"


def test_drop_hardware_failure_stops_the_mission():
    state = DropState(_FSM())
    assert state._execute_drop() == "ERROR"


def test_descent_ready_remains_latched_during_precision_alignment():
    state = DropState(_FSM())
    state._descent_ready_latched = True

    assert state._check_descent_ready(height_error=0.8)


def test_standalone_bridge_exposes_real_px4_velocity():
    facade = _StandaloneBridgeFacade.__new__(_StandaloneBridgeFacade)
    facade._fsm = type(
        "FSM",
        (),
        {
            "_standalone_pose": None,
            "_standalone_battery": 0.75,
        },
    )()
    facade._position = Position()
    facade._velocity = Velocity()
    facade._current_yaw = 0.0
    facade._px4_armed = True
    facade._px4_landed = False

    odometry = type("Odometry", (), {"velocity": [1.25, -0.5, 0.08]})()
    facade._odom_cb(odometry)

    telemetry = facade.get_telemetry()
    assert telemetry.velocity == Velocity(vn=1.25, ve=-0.5, vd=0.08)


def test_takeoff_target_is_not_rebuilt_from_position_drift():
    positions = iter(
        [
            Position(north=0.0, east=0.0, down=0.0),
            Position(north=0.02, east=-0.01, down=-0.1),
        ]
    )
    messages = []
    fsm = type(
        "FSM",
        (),
        {
            "bridge": type(
                "Bridge",
                (),
                {"get_current_position": lambda _self: next(positions)},
            )(),
            "get_logger": lambda _self: type(
                "Logger", (), {"info": lambda _logger, message: messages.append(message)},
            )(),
        },
    )()
    facade = _StandaloneFlightFacade.__new__(_StandaloneFlightFacade)
    facade._fsm = fsm
    facade._target_ned = None
    facade._target_set_time = 0.0

    facade.takeoff(5.0)
    first_target = facade._target_ned
    facade.takeoff(5.0)

    assert facade._target_ned == first_target
    assert len(messages) == 1


def test_precision_land_timeout_uses_px4_land_not_force_disarm(monkeypatch):
    fsm = _FSM()
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -1.0})()
    fsm.fc.set_velocity_ned = lambda *_args: None
    state = PrecisionLandState(fsm)
    state._start_time = 0.0
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: 61.0,
    )

    assert state._execute_blind() is None
    assert fsm.bridge.land_commands == 1
    assert fsm.bridge.force_disarms == 0


def test_real_flight_arm_does_not_use_force_magic():
    source = (PACKAGE_ROOT / "drone_mission/mission_state_machine.py").read_text(
        encoding="utf-8",
    )
    arm_source = source.split("    def arm(self):", 1)[1].split(
        "    def disarm(self):", 1,
    )[0]
    assert "21196" not in arm_source


def test_px4_telemetry_uses_versioned_status_topics():
    source = (PACKAGE_ROOT / "drone_mission/mission_state_machine.py").read_text(
        encoding="utf-8",
    )

    assert '"/fmu/out/vehicle_status_v4"' in source
    assert '"/fmu/out/battery_status_v1"' in source


def test_px4_telemetry_facade_exists_before_init_state_runs():
    source = (PACKAGE_ROOT / "drone_mission/mission_state_machine.py").read_text(
        encoding="utf-8",
    )
    lazy_init = source.split("    def _lazy_init(self):", 1)[1].split(
        "    def _init_vision_subscriptions(self):", 1,
    )[0]

    assert lazy_init.index("_ = self.bridge") < lazy_init.index(
        "self._transition_to(MissionState.INIT)"
    )


def test_navigation_timeouts_do_not_advance_the_mission():
    transit = (PACKAGE_ROOT / "drone_mission/states/state_transit.py").read_text(
        encoding="utf-8",
    )
    ascend = (PACKAGE_ROOT / "drone_mission/states/state_drop.py").read_text(
        encoding="utf-8",
    ).split("def _execute_ascend", 1)[1]

    assert "超时, 强制进入下一状态" not in transit
    assert 'return "ERROR"' in transit
    assert 'return "SELECT_TARGET" if drops' not in ascend


def test_landing_timeout_never_reports_success_or_force_disarms():
    source = (
        SOURCE_ROOT / "px4_control/px4_control/flight_controller.py"
    ).read_text(encoding="utf-8")
    land_source = source.split("    def land(self)", 1)[1].split(
        "    def force_disarm", 1,
    )[0]

    assert "force_disarm" not in land_source
    assert "land_at_current" in land_source
