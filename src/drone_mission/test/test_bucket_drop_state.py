from types import SimpleNamespace

from drone_mission.quick_bucket_gate import GateResult, Observation
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
        self.visual_center_frames = {}
        self.released = False

    def commit_release(self):
        self.committed += 1
        self.released = True


def state(intent="cylinder_visual"):
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
            get_current_position=lambda: SimpleNamespace(north=0.0, east=0.0, down=-1.6),
            _current_yaw=0.0,
        ),
        fc=SimpleNamespace(ned_to_altitude=lambda down: -down),
        cylinder_selector=SimpleNamespace(_selected_positions=[]),
        _current_drop_cyl_id=3,
        start_drop_service=lambda target: calls.append(target),
    )
    handler = DropState(fsm)
    handler._bucket_current_objects = lambda **_kwargs: ((None, None), (None, None))
    handler.set_sub_state("EXECUTE_DROP")
    handler.enter()
    return handler, fsm, calls


def test_entry_never_sends_service_and_execute_rechecks():
    handler, fsm, calls = state()
    assert calls == []
    handler._bucket_gate_result = lambda: GateResult("release", "cylinder_visual", 3.0)
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


def test_bucket_descend_holds_brief_loss_then_continues_vertical_descent(monkeypatch):
    handler, fsm, _calls = state()
    fsm.bridge.get_current_position = lambda: SimpleNamespace(down=-4.4)
    commands = []
    fsm.fc.set_velocity_body = lambda *args: commands.append(args)
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
    assert reasons == []
    assert commands[-1] == (0.0, 0.0, 0.45)


def _visual_gate_setup(monkeypatch, intent=None):
    handler, fsm, calls = state(intent)
    clock = [10.0]
    monkeypatch.setattr("drone_mission.states.state_drop.time.time", lambda: clock[0])
    monkeypatch.setattr("drone_mission.states.state_drop.time.monotonic", lambda: clock[0])
    fsm.bridge._pose_frame = 1
    fsm.bridge._velocity_frame = 1
    fsm.bridge._last_odom_time = 10.0
    fsm.bridge._last_attitude_time = 10.0
    fsm.bridge.get_telemetry = lambda: SimpleNamespace(
        velocity=SimpleNamespace(vn=0.0, ve=0.0, vd=0.0),
        timestamp=clock[0])
    fsm.get_param = lambda key, default=None: {
        "quick_drop_mode": True,
        "quick_drop_profile": "bucket",
        "bucket_home_body_height": 0.40,
        "bucket_camera_down": 0.365,
        "bucket_rim_height": 0.40,
        "bucket_front_forward": 0.045,
        "camera_cx": 320.0,
        "camera_cy": 240.0,
        "camera_fx": 400.0,
        "camera_fy": 400.0,
    }.get(key, default)
    return handler, fsm, calls, clock


def test_fresh_centered_cylinder_releases_without_blue_or_calibration(monkeypatch):
    handler, fsm, _calls, clock = _visual_gate_setup(monkeypatch)
    fsm._quick_bucket_gate.final_align_started_at = 10.0
    centered_v = 240.0 - 0.045 * 400.0 / 1.235
    cylinder = SimpleNamespace(center_u=320.0, center_v=centered_v)
    handler._bucket_current_objects = lambda **_kwargs: (
        (cylinder, Observation(clock[0], 0.075)),
        (None, None),
    )
    for instant in (10.0, 10.1):
        clock[0] = instant
        fsm.bridge._last_odom_time = instant
        fsm.bridge._last_attitude_time = instant
        assert handler._bucket_gate_result().action == "continue"
    clock[0] = 10.2
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = clock[0]
    assert handler._bucket_gate_result().action == "brake"
    clock[0] = 10.6
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = clock[0]
    result = handler._bucket_gate_result()
    assert result.action == "release"
    assert result.reason == "cylinder_visual"


def test_sixteen_second_timeout_releases_without_current_visual_targets(monkeypatch):
    handler, fsm, calls, clock = _visual_gate_setup(monkeypatch, "cylinder_timeout")
    fsm._quick_bucket_gate.final_align_started_at = 0.0
    clock[0] = 16.0
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = clock[0]
    assert handler._bucket_gate_result().action == "brake"
    clock[0] = 16.4
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = clock[0]
    result = handler._bucket_gate_result()
    assert result.action == "release"
    assert result.reason == "cylinder_timeout"
    assert handler.execute() is None
    assert calls == [3]


def test_timeout_does_not_release_below_safe_height(monkeypatch):
    handler, fsm, calls, clock = _visual_gate_setup(monkeypatch, "cylinder_timeout")
    fsm._quick_bucket_gate.final_align_started_at = 0.0
    fsm.bridge.get_current_position = lambda: SimpleNamespace(north=0.0, east=0.0, down=-1.4)
    reasons = []
    fsm.request_takeover = reasons.append
    clock[0] = 17.0
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = clock[0]
    assert handler._bucket_gate_result().action == "takeover"
    assert handler.execute() is None
    assert reasons == ["DROP: below_safe_altitude"]
    assert calls == []


def test_final_bucket_stage_times_out_into_actual_drop_without_blue(monkeypatch):
    handler, fsm, calls, clock = _visual_gate_setup(monkeypatch)
    commands = []
    release_logs = []
    fsm._task_logger.log = lambda name, payload: release_logs.append((name, payload))
    fsm.fc.set_velocity_body = lambda *args: commands.append(args)
    handler._bucket_current_objects = lambda **_kwargs: ((None, None), (None, None))
    handler.set_sub_state("DESCEND_TO_DROP")
    handler._start_time = 10.0

    assert handler.execute() is None
    assert fsm._quick_bucket_gate.final_align_started_at == 10.0
    assert calls == []

    clock[0] = 26.0
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = 26.0
    assert handler.execute() is None
    assert commands[-1] == (0.0, 0.0, 0.0)

    clock[0] = 26.4
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = 26.4
    assert handler.execute() == "EXECUTE_DROP"
    assert fsm._quick_release_intent == "cylinder_timeout"
    handler.set_sub_state("EXECUTE_DROP")
    handler.enter()
    assert handler.execute() is None
    assert calls == [3]
    assert fsm._quick_bucket_gate.committed == 1
    assert release_logs[0][0] == "quick_bucket_release_gate"
    assert release_logs[0][1]["visual_timeout_fallback"] is True
    assert release_logs[0][1]["pixel_error_px"] is None


def test_timeout_brakes_until_horizontal_velocity_is_low(monkeypatch):
    handler, fsm, _calls, clock = _visual_gate_setup(monkeypatch)
    gate = fsm._quick_bucket_gate
    gate.final_align_started_at = 0.0
    clock[0] = 16.0
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = 16.0
    assert handler._bucket_gate_result().action == "brake"
    fsm.bridge.get_telemetry = lambda: SimpleNamespace(
        velocity=SimpleNamespace(vn=0.2, ve=0.0, vd=0.0), timestamp=clock[0])
    clock[0] = 16.4
    fsm.bridge._last_odom_time = fsm.bridge._last_attitude_time = 16.4
    assert handler._bucket_gate_result().action == "brake"


def test_release_log_failure_does_not_cancel_approved_drop():
    handler, fsm, calls = state()
    fsm._task_logger.log = lambda *_args: (_ for _ in ()).throw(OSError("disk full"))
    handler._bucket_gate_result = lambda: GateResult("release", "cylinder_visual")
    assert handler.execute() is None
    assert calls == [3]
    assert fsm._quick_bucket_gate.committed == 1


def test_bucket_control_does_not_read_marker_detection():
    handler, fsm, _calls = state()
    fsm.get_cylinders = lambda: []
    fsm.get_markers = lambda: (_ for _ in ()).throw(RuntimeError("marker unavailable"))
    fsm._cylinder_capture_stamp = 0.0
    assert DropState._bucket_current_objects(handler) == (
        (None, None), (None, None))
