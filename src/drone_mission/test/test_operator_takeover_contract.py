"""Contracts for the operator-takeover mission state."""

import time

from drone_mission.mission_state_machine import MissionFSM, MissionState
from drone_mission.states.state_hold_for_takeover import HoldForTakeoverState
from drone_mission.states.state_init import InitState
from drone_utils.emergency import EmergencyLevel


class _Logger:
    def __init__(self):
        self.info_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _TaskLogger:
    def log_state(self, *_args):
        pass


class _Watchdog:
    def is_alive(self, _name):
        return False


class _ReadinessBridge:
    battery_ready = False
    land_ready = False
    preflight_ok = False

    @staticmethod
    def telemetry_fresh(_now):
        return False

    @staticmethod
    def is_armed():
        return False


class _InitContext:
    bridge = _ReadinessBridge()
    watchdog = _Watchdog()
    odom_ready = False
    servo_ready = False
    vision_ready = False
    battery_ready = False
    land_ready = False
    rc_ready = False
    logger = _TaskLogger()

    def __init__(self):
        self._logger = _Logger()

    @staticmethod
    def get_param(name, default=None):
        if name == "rc_require_arm_signal":
            return True
        return default

    def get_logger(self):
        return self._logger


def test_init_waits_past_30_seconds_and_reports_every_missing_prerequisite(
    monkeypatch,
):
    context = _InitContext()
    state = InitState(context)
    monkeypatch.setattr("drone_mission.states.state_init.time.time", lambda: 0.0)
    state.enter()
    state._odom_stable_threshold = 1
    monkeypatch.setattr("drone_mission.states.state_init.time.time", lambda: 31.0)

    assert state.execute() is None
    diagnostic = context.get_logger().info_messages[-1]
    for prerequisite in (
        "telemetry",
        "odom",
        "battery",
        "land_status",
        "vision",
        "servo",
        "rc",
    ):
        assert prerequisite in diagnostic


class _HoldBridge:
    def __init__(self):
        self.armed = True
        self.offboard = True
        self.hold_calls = 0
        self.stop_calls = 0

    def is_armed(self):
        return self.armed

    def is_offboard(self):
        return self.offboard

    def hold_for_takeover(self):
        self.hold_calls += 1

    def stop_autonomous_output(self):
        self.stop_calls += 1


class _HoldContext:
    def __init__(self):
        self.bridge = _HoldBridge()
        self.logger = _TaskLogger()
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


def test_hold_state_repeats_zero_velocity_until_px4_leaves_offboard():
    context = _HoldContext()
    state = HoldForTakeoverState(context)
    state.enter()

    assert state.execute() is None
    assert context.bridge.hold_calls == 2

    context.bridge.offboard = False
    assert state.execute() == "COMPLETE"
    assert context.bridge.stop_calls == 1


class _TakeoverContext:
    def __init__(self):
        self._takeover_reason = None
        self._mission_failed = False
        self.bridge = _HoldBridge()
        self.transitions = []
        self._logger = _Logger()

    def _transition_to(self, state):
        self.transitions.append(state)

    def get_logger(self):
        return self._logger


def test_request_takeover_latches_the_first_reason_and_never_commands_rtl_or_land():
    context = _TakeoverContext()

    MissionFSM.request_takeover(context, "takeoff stalled")
    MissionFSM.request_takeover(context, "later fault")

    assert context._takeover_reason == "takeoff stalled"
    assert context._mission_failed
    assert context.bridge.hold_calls == 1
    assert [state.value for state in context.transitions] == ["HOLD_FOR_TAKEOVER"]


class _ErrorTransitionContext:
    def __init__(self):
        self.takeover_reason = None
        self.transitioned_to = None
        self._logger = _Logger()

    def request_takeover(self, reason):
        self.takeover_reason = reason

    def _transition_to(self, state):
        self.transitioned_to = state

    def get_logger(self):
        return self._logger


def test_terminal_state_error_is_mapped_to_operator_takeover():
    context = _ErrorTransitionContext()

    MissionFSM._transition_to_str(context, "ERROR")

    assert context.takeover_reason is not None
    assert context.transitioned_to is None


class _LoopWatchdog:
    def feed(self, _name):
        pass


class _LoopEmergency:
    def __init__(self, level):
        self.level = level

    def update_battery(self, _remaining):
        pass

    def check(self):
        return self.level


class _LoopHandler:
    def __init__(self):
        self.execute_calls = 0

    def execute(self):
        self.execute_calls += 1
        return None


class _LoopBridge(_HoldBridge):
    battery_ready = False


class _LoopContext:
    def __init__(self, *, offboard, emergency_level):
        self._watchdog = _LoopWatchdog()
        self._emergency = _LoopEmergency(emergency_level)
        self._current_state = MissionState.TRANSIT_TO_DROP_ZONE
        self._offboard_confirmed = True
        self.bridge = _LoopBridge()
        self.bridge.offboard = offboard
        self.handler = _LoopHandler()
        self._state_handlers = {self._current_state: self.handler}
        self.takeover_reason = None
        self._logger = _Logger()

    @staticmethod
    def get_param(_name, default=None):
        return default

    def get_logger(self):
        return self._logger

    def request_takeover(self, reason):
        self.takeover_reason = reason


def test_normal_task_stops_for_operator_when_px4_leaves_offboard():
    context = _LoopContext(offboard=False, emergency_level=EmergencyLevel.OK)

    MissionFSM._control_loop_impl(context)

    assert context.takeover_reason == "PX4 left Offboard"
    assert context.handler.execute_calls == 0


def test_emergency_land_is_mapped_to_operator_takeover_without_task_land():
    context = _LoopContext(offboard=True, emergency_level=EmergencyLevel.LAND)

    MissionFSM._control_loop_impl(context)

    assert context.takeover_reason == "emergency land"
    assert context.handler.execute_calls == 0


def test_confirmed_ground_disarm_is_not_mistaken_for_rc_takeover():
    context = _LoopContext(offboard=True, emergency_level=EmergencyLevel.OK)
    context._current_state = MissionState.PRECISION_LAND
    context._state_handlers = {context._current_state: context.handler}
    context.bridge.armed = False
    context.handler.final_disarm_pending = True

    MissionFSM._control_loop_impl(context)

    assert context.takeover_reason is None
    assert context.handler.execute_calls == 1
