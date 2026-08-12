"""Recovery contracts for the real full-mission path."""

from drone_mission.states.state_recon_patrol import ReconPatrolState
from drone_mission.states.state_return import ReturnState
from drone_mission.states.state_transit import TransitState
from px4_control.flight_controller import FlightController
from px4_control.telemetry import Position, world_to_ned


TASK_YAW = 0.8726646259971648


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


class _FC:
    origin_down = 0.0

    def __init__(self):
        self.goto_calls = []
        self.position = Position()

    def goto_ned(self, *args, **kwargs):
        self.goto_calls.append((args, kwargs))

    def get_position(self):
        return self.position

    def reached(self, *_args):
        return False

    def altitude_to_ned(self, altitude):
        return self.origin_down - altitude

    def ned_to_altitude(self, down):
        return self.origin_down - down


class _BridgeSpy:
    def __init__(self):
        self.position = Position()
        self.position_calls = []

    @staticmethod
    def get_parameter(name):
        values = {
            "takeoff_height": 5.0,
            "waypoint_tolerance": 0.5,
            "takeoff_tolerance": 0.3,
            "waypoint_timeout": 120.0,
        }
        return type("Parameter", (), {"value": values[name]})()

    def get_current_position(self):
        return self.position

    def set_position_target(self, north, east, down, yaw, speed=None):
        self.position_calls.append((north, east, down, yaw, speed))


class _SpyFlightController(FlightController):
    def __init__(self, bridge, task_yaw):
        super().__init__(bridge, task_yaw=task_yaw)
        self.requested_yaws = []

    def goto_ned(self, north, east, down, yaw=None, speed=1.0):
        self.requested_yaws.append(yaw)
        return super().goto_ned(north, east, down, yaw=yaw, speed=speed)


def _use_task_yaw_controller(fsm):
    bridge = _BridgeSpy()
    fsm.bridge = bridge
    fsm.fc = _SpyFlightController(bridge, task_yaw=TASK_YAW)
    return bridge


class _RecoveryFSM:
    current_state = "TAKEOFF"
    logger = _TaskLogger()

    def __init__(self):
        self.fc = _FC()
        self.takeovers = []

    @staticmethod
    def get_param(name, default=None):
        return {
            "navigation_progress_window": 1.0,
            "navigation_progress_min_improvement": 0.1,
            "transit_speed": 2.0,
            "waypoint_tolerance": 0.5,
        }.get(name, default)

    @staticmethod
    def get_logger():
        return _Logger()

    def request_takeover(self, reason):
        self.takeovers.append(reason)


def test_transit_resends_the_unchanged_target_once_then_requests_takeover(
    monkeypatch,
):
    fsm = _RecoveryFSM()
    state = TransitState(fsm)
    state.configure((8.0, 20.0, 5.0), "CYLINDER_SEARCH", "TRANSIT_TEST")
    monkeypatch.setattr("drone_mission.states.state_transit.time.time", lambda: 0.0)
    monotonic = iter((0.0, 1.0, 2.0))
    monkeypatch.setattr(
        "drone_mission.states.state_transit.time.monotonic", lambda: next(monotonic),
    )
    state.enter()
    original_command = fsm.fc.goto_calls[-1]

    assert state.execute() is None
    assert state.execute() is None
    assert fsm.fc.goto_calls[-1] == original_command
    assert fsm.takeovers == []

    assert state.execute() is None
    assert fsm.takeovers == ["TRANSIT_TEST: target made no progress after retry"]


def test_transit_inherits_nonzero_task_yaw_without_state_override():
    fsm = _RecoveryFSM()
    bridge = _use_task_yaw_controller(fsm)
    state = TransitState(fsm)
    state.configure((8.0, 20.0, 5.0), "CYLINDER_SEARCH", "TRANSIT_TEST")

    state.enter()

    assert fsm.fc.requested_yaws == [None]
    assert bridge.position_calls[-1][3] == TASK_YAW


class _Planner:
    @staticmethod
    def plan_coverage_ned(**_kwargs):
        return [{"north": 55.0, "east": -4.0, "down": -3.0}]


class _ReconFSM(_RecoveryFSM):
    def __init__(self):
        super().__init__()
        self.path_planner = _Planner()
        self.watchdog = None
        self.vision_modes = []

    @staticmethod
    def get_param(name, default=None):
        return {
            "navigation_progress_window": 1.0,
            "navigation_progress_min_improvement": 0.1,
            "area2_altitude": 3.0,
            "area2_forward_min": 55.0,
            "area2_forward_max": 60.0,
            "area2_right_min": -4.0,
            "area2_right_max": 4.0,
            "grid_spacing": 1.67,
            "sweep_speed": 0.8,
            "waypoint_tolerance": 0.5,
            "recon_require_full_path": True,
        }.get(name, default)

    @staticmethod
    def get_coverage():
        return None

    @staticmethod
    def zone_bounds(_prefix):
        return (55.0, 60.0, -4.0, 4.0)

    def set_vision_mode(self, mode):
        self.vision_modes.append(mode)


def test_recon_stall_keeps_the_same_index_for_one_retry_then_requests_takeover(
    monkeypatch,
):
    fsm = _ReconFSM()
    state = ReconPatrolState(fsm)
    monkeypatch.setattr("drone_mission.states.state_recon_patrol.time.time", lambda: 0.0)
    monotonic = iter((0.0, 1.0, 2.0))
    monkeypatch.setattr(
        "drone_mission.states.state_recon_patrol.time.monotonic",
        lambda: next(monotonic),
    )
    state.enter()
    original_command = fsm.fc.goto_calls[-1]

    assert state.execute() is None
    assert state.execute() is None
    assert state._wp_idx == 0
    assert fsm.fc.goto_calls[-1] == original_command
    assert fsm.takeovers == []

    assert state.execute() is None
    assert state._wp_idx == 0
    assert fsm.takeovers == ["RECON_PATROL: waypoint 0 made no progress after retry"]


class _ReturnFSM(_RecoveryFSM):
    @staticmethod
    def get_param(name, default=None):
        return {
            "home_x": 4.0,
            "home_y": 10.0,
            "return_altitude": 5.0,
            "return_speed": 3.8,
        }.get(name, default)


def test_return_starts_from_the_last_recon_waypoint_with_home_and_5m_together(
    monkeypatch,
):
    fsm = _ReturnFSM()
    state = ReturnState(fsm)
    monkeypatch.setattr("drone_mission.states.state_return.time.time", lambda: 0.0)

    state.enter()

    (north, east, down), kwargs = fsm.fc.goto_calls[-1]
    expected_north, expected_east, _ = world_to_ned(4.0, 10.0, 0.0)
    assert (north, east, down) == (expected_north, expected_east, -5.0)
    assert kwargs["speed"] == 3.8


def test_return_inherits_nonzero_task_yaw_without_state_override(monkeypatch):
    fsm = _ReturnFSM()
    bridge = _use_task_yaw_controller(fsm)
    state = ReturnState(fsm)
    monkeypatch.setattr("drone_mission.states.state_return.time.time", lambda: 0.0)

    state.enter()

    assert fsm.fc.requested_yaws == [None]
    assert bridge.position_calls[-1][3] == TASK_YAW


def test_task_recovery_paths_do_not_issue_px4_rtl_or_land():
    from pathlib import Path

    states_dir = Path(__file__).resolve().parents[1] / "drone_mission/states"
    for name in (
        "state_takeoff.py",
        "state_transit.py",
        "state_cylinder_search.py",
        "state_cylinder_approach.py",
        "state_drop.py",
        "state_recon_patrol.py",
        "state_return.py",
        "state_precision_land.py",
    ):
        source = (states_dir / name).read_text(encoding="utf-8")
        assert "return_to_launch(" not in source
        assert "land_at_current(" not in source
