"""Contracts for explicit operator takeover after an unrecoverable task fault."""

import importlib.util

from drone_mission.mission_state_machine import MissionFSM, MissionState
from drone_mission.states.state_init import InitState


class _Logger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def info(self, message):
        self.infos.append(message)

    def warn(self, _message):
        pass

    def error(self, message):
        self.errors.append(message)


class _TaskLogger:
    def log_state(self, *_args):
        pass

    def write_summary(self, _summary):
        pass


class _Bridge:
    def __init__(self, armed=True, offboard=True):
        self.armed = armed
        self.offboard = offboard
        self.hold_calls = 0
        self.stop_calls = 0
        self.land_commands = 0
        self.rtl_commands = 0

    def hold_for_takeover(self):
        self.hold_calls += 1

    def stop_autonomous_output(self):
        self.stop_calls += 1

    def is_armed(self):
        return self.armed

    def is_offboard(self):
        return self.offboard

    def telemetry_fresh(self, _now):
        return False

    @property
    def battery_ready(self):
        return True

    @property
    def land_ready(self):
        return True

    @property
    def preflight_ok(self):
        return True


class _HoldFSM:
    def __init__(self):
        self.bridge = _Bridge()
        self.logger = _TaskLogger()
        self._mission_failed = False
        self._takeover_reason = None
        self._current_state = MissionState.TAKEOFF
        self.transitions = []
        self._logger = _Logger()

    def _transition_to(self, state):
        self.transitions.append(state)

    def get_logger(self):
        return self._logger


class _InitWatchdog:
    @staticmethod
    def is_alive(_name):
        return True


class _InitFSM:
    def __init__(self):
        self.bridge = _Bridge()
        self.watchdog = _InitWatchdog()
        self.odom_ready = True
        self.servo_ready = False
        self.vision_ready = True
        self.battery_ready = True
        self.land_ready = True
        self.rc_ready = True
        self.logger = _TaskLogger()
        self._logger = _Logger()

    @staticmethod
    def get_param(_name, default):
        return default

    def get_logger(self):
        return self._logger


def test_takeover_state_holds_until_rc_or_px4_exits_offboard(monkeypatch):
    module_spec = importlib.util.find_spec(
        "drone_mission.states.state_hold_for_takeover"
    )
    assert module_spec is not None
    from drone_mission.states.state_hold_for_takeover import HoldForTakeoverState

    fsm = _HoldFSM()
    state = HoldForTakeoverState(fsm)
    monotonic_values = iter((0.0, 5.1, 5.2))
    monkeypatch.setattr(
        "drone_mission.states.state_hold_for_takeover.time.monotonic",
        lambda: next(monotonic_values),
    )

    state.enter()
    assert state.execute() is None
    assert fsm.bridge.hold_calls == 2
    assert fsm.bridge.stop_calls == 0

    fsm.bridge.offboard = False
    assert state.execute() == "COMPLETE"
    assert fsm.bridge.stop_calls == 1
    assert fsm.bridge.land_commands == 0
    assert fsm.bridge.rtl_commands == 0


def test_request_takeover_latches_first_reason_and_never_transitions_to_error():
    assert callable(getattr(MissionFSM, "request_takeover", None))

    fsm = _HoldFSM()
    MissionFSM.request_takeover(fsm, "first fault")
    MissionFSM.request_takeover(fsm, "later fault")

    assert fsm._mission_failed is True
    assert fsm._takeover_reason == "first fault"
    assert fsm.transitions == [MissionState.HOLD_FOR_TAKEOVER]
    assert fsm.bridge.hold_calls == 1


class _CompletionFSM:
    def __init__(self):
        self.bridge = _Bridge()
        self._mission_failed = False
        self._drop_success_count = 2
        self._drop_count = 2
        self._recon_path_completed = True
        self._task_logger = _TaskLogger()
        self._logger = _Logger()

    @staticmethod
    def get_param(name, default):
        return 2 if name == "max_drops" else default

    @staticmethod
    def get_coverage():
        return None

    def get_logger(self):
        return self._logger


def test_successful_completion_releases_the_autonomous_output_lease():
    fsm = _CompletionFSM()

    MissionFSM._handle_complete(fsm)

    assert fsm.bridge.stop_calls == 1


def test_init_waits_past_thirty_seconds_when_a_prerequisite_is_missing(monkeypatch):
    fsm = _InitFSM()
    state = InitState(fsm)
    state.enter()
    state._odom_stable_threshold = 1
    state._start_time = 0.0
    monkeypatch.setattr("drone_mission.states.state_init.time.time", lambda: 31.0)

    assert state.execute() is None
    assert any("servo" in message for message in fsm.get_logger().infos)
