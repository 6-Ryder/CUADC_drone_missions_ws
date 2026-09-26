from types import SimpleNamespace

from drone_mission.quick_bucket_gate import GateResult
from drone_mission.states.state_drop import DropState


class Logger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def warn(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)

    def log_state(self, *_args):
        pass


class Gate:
    def __init__(self):
        self.committed = 0
        self.bucket_ne = (1.0, 2.0)
        self.disc_ne = (1.0, 2.0)
        self.calibration = object()
        self.started_at = None

    def commit_release(self):
        self.committed += 1


def state(intent="small_bucket"):
    logger = Logger()
    calls = []
    fsm = SimpleNamespace(
        get_param=lambda key, default=None: {
            "quick_drop_mode": True,
            "quick_drop_profile": "bucket",
        }.get(key, default),
        get_logger=lambda: logger,
        logger=logger,
        _quick_release_intent=intent,
        _quick_bucket_gate=Gate(),
        _task_logger=SimpleNamespace(log=lambda *_args: None),
        bridge=SimpleNamespace(
            get_telemetry=lambda: SimpleNamespace(
                velocity=SimpleNamespace(vn=0.0, ve=0.0, vd=0.0),
                timestamp=10.0),
            get_current_position=lambda: SimpleNamespace(down=-1.6),
            _current_yaw=0.0,
        ),
        fc=SimpleNamespace(ned_to_altitude=lambda down: -down),
        cylinder_selector=SimpleNamespace(_selected_positions=[]),
        _current_drop_cyl_id=3,
        start_drop_service=lambda target: calls.append(target),
    )
    handler = DropState(fsm)
    handler._bucket_current_objects = lambda: ((None, None), (None, None))
    handler.set_sub_state("EXECUTE_DROP")
    handler.enter()
    return handler, fsm, calls


def test_entry_never_sends_service_and_execute_rechecks():
    handler, fsm, calls = state()
    assert calls == []
    handler._bucket_gate_result = lambda: GateResult("release", "small_bucket", 0.01, 0.1)
    assert handler.execute() is None
    assert calls == [3]
    assert fsm._quick_bucket_gate.committed == 1
    assert fsm._quick_release_intent is None


def test_stale_or_changed_intent_cannot_bypass_gate():
    handler, fsm, calls = state()
    handler._bucket_gate_result = lambda: GateResult("continue", "stale_telemetry")
    assert handler.execute() == "DESCEND_TO_DROP"
    assert calls == []
    assert fsm._quick_bucket_gate.committed == 0
    assert fsm._quick_release_intent is None


def test_missing_intent_calls_takeover_without_service():
    handler, fsm, calls = state(None)
    reasons = []
    fsm.request_takeover = reasons.append
    assert handler.execute() is None
    assert reasons == ["DROP: missing bucket release intent"]
    assert calls == []


def test_bucket_gate_rejects_non_ned_odometry_before_predicting():
    handler, fsm, _calls = state()
    fsm.bridge._pose_frame = 1
    fsm.bridge._velocity_frame = 3
    result = DropState._bucket_gate_result(handler)
    assert result.action == "takeover"
    assert result.reason == "unsupported_odometry_frame"


def test_blue_shadow_runs_after_original_service_request():
    handler, fsm, _calls = state()
    events = []
    fsm.get_param = lambda key, default=None: {
        "quick_drop_mode": True,
        "quick_drop_profile": "blue",
    }.get(key, default)
    fsm.start_drop_service = lambda target: events.append(("service", target))
    handler._log_blue_shadow = lambda request_time: events.append(
        ("shadow", request_time))

    handler._enter_drop()

    assert events[0] == ("service", 3)
    assert events[1][0] == "shadow"
    assert isinstance(events[1][1], float)


def test_bucket_descend_holds_on_one_missing_frame_then_takes_over(monkeypatch):
    handler, fsm, _calls = state()
    fsm.bridge.get_current_position = lambda: SimpleNamespace(down=-4.4)
    fsm.fc.set_velocity_body = lambda *_args: None
    reasons = []
    fsm.request_takeover = reasons.append
    handler._start_time = 0.0
    handler._bucket_last_seen_at = 4.9
    clock = [5.1]
    monkeypatch.setattr("drone_mission.states.state_drop.time.time", lambda: clock[0])

    assert handler._execute_bucket_descend() is None
    assert reasons == []

    clock[0] = 7.0
    assert handler._execute_bucket_descend() is None
    assert reasons == ["DESCEND: no fresh paired target above 2.4m"]
