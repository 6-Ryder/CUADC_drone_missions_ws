import time
from pathlib import Path

from drone_mission.mission_state_machine import MissionFSM
from drone_mission.states.state_drop import DropState
from drone_mission.states.state_init import InitState
from drone_utils.layout_frame import LayoutFrame


class _Context:
    def __init__(self):
        self._drop_result_consumed = False
        self._drop_service_failed = True  # 模拟舵机服务不可用
        self._drop_sent_ok = False
        self._drop_count = 0
        self._drop_success_count = 0


def test_missing_servo_service_is_a_failed_drop():
    context = _Context()

    result = MissionFSM.check_drop_service_result(context)

    assert result is False
    assert context._drop_count == 1
    assert context._drop_success_count == 0
    assert context._drop_result_consumed is True


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class _Position:
    down = -1.0

    @staticmethod
    def norm_xy():
        return 1.0


class _Bridge:
    @staticmethod
    def telemetry_fresh(_now):
        return True

    @property
    def land_ready(self):
        return True

    @staticmethod
    def get_telemetry():
        return type("Telemetry", (), {"position": _Position()})()


class _Watchdog:
    def is_alive(self, name):
        return name == "vision"


class _InitContext:
    bridge = _Bridge()
    watchdog = _Watchdog()
    odom_ready = True
    servo_ready = False
    vision_ready = True
    battery_ready = True
    land_ready = True
    rc_ready = True
    logger = type("TaskLogger", (), {"log_state": lambda *_args: None})()

    @staticmethod
    def get_logger():
        return _Logger()


def test_init_does_not_arm_without_ready_servo():
    state = InitState(_InitContext())
    state.enter()
    state._odom_stable_threshold = 1

    assert state.execute() is None


class _PendingDropContext:
    logger = type("TaskLogger", (), {"log_state": lambda *_args: None})()

    def __init__(self):
        self.takeovers = []

    @staticmethod
    def get_param(name, default=None):
        if name == "drop_confirmation_timeout":
            return 0.01
        return default

    @staticmethod
    def check_drop_service_result():
        return None

    @staticmethod
    def get_logger():
        return _Logger()

    def request_takeover(self, reason):
        self.takeovers.append(reason)


def test_unconfirmed_drop_aborts_instead_of_retrying_with_unknown_servo_state():
    context = _PendingDropContext()
    state = DropState(context)
    state.set_sub_state("EXECUTE_DROP")
    state._start_time = time.time() - 1.0

    assert state._execute_drop() is None
    assert context.takeovers == ["DROP: release result remained unconfirmed"]


class _SummaryLogger:
    def __init__(self):
        self.summary = None

    def write_summary(self, summary):
        self.summary = summary


class _FailedMissionContext:
    _drop_success_count = 0
    _drop_count = 0
    _mission_failed = True
    layout_frame = LayoutFrame(0.0)

    def __init__(self):
        self._task_logger = _SummaryLogger()

    @staticmethod
    def get_logger():
        return _Logger()

    @staticmethod
    def get_coverage():
        return None

    @staticmethod
    def get_param(_name, default=None):
        return default


def test_takeover_completion_is_not_reported_as_success():
    context = _FailedMissionContext()

    MissionFSM._handle_complete(context)

    assert context._task_logger.summary["success"] is False
    assert context._task_logger.summary["final_state"] == "HOLD_FOR_TAKEOVER"


class _IncompleteWorkContext(_FailedMissionContext):
    _mission_failed = False
    _drop_success_count = 1
    _drop_count = 1
    _recon_path_completed = True

    @staticmethod
    def get_param(name, default=None):
        return 2 if name == "max_drops" else default


def test_complete_summary_requires_all_confirmed_drops_and_full_recon_path():
    context = _IncompleteWorkContext()

    MissionFSM._handle_complete(context)

    assert context._task_logger.summary["success"] is False
    assert context._task_logger.summary["final_state"] == "HOLD_FOR_TAKEOVER"


def test_drop_state_has_one_async_service_path():
    source = (
        Path(__file__).resolve().parents[1]
        / "drone_mission/states/state_drop.py"
    ).read_text(encoding="utf-8")

    assert "call_drop_service" not in source
    assert 'hasattr(self._fsm, "start_drop_service")' not in source
