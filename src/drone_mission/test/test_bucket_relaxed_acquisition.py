"""Relaxed bucket acquisition through real FSM/search/approach/descent handlers."""

import math
import time
from types import SimpleNamespace

import pytest

from drone_mission.cylinder_selector import CylinderSelector
from drone_mission.mission_state_machine import MissionFSM, MissionState
from drone_mission.quick_bucket_gate import BucketReleaseGate, Calibration, Observation
from drone_mission.states.state_cylinder_search import CylinderSearchState
from drone_mission.states.state_cylinder_approach import CylinderApproachState
from drone_mission.states.state_drop import DropState
from drone_utils.layout_frame import LayoutFrame
from px4_control import telemetry


class Recorder:
    def __init__(self):
        self.events = []

    def info(self, *args):
        self.events.append(args)

    warn = error = log = log_state = info


@pytest.fixture
def clock(monkeypatch):
    now = [100.9]
    monkeypatch.setattr(time, "time", lambda: now[0])
    for name in ("_home_world_x", "_home_world_y", "_home_world_z"):
        monkeypatch.setattr(telemetry, name, 0.0)
    return now


def mission(profile="bucket"):
    fsm = object.__new__(MissionFSM)
    parameters = {"quick_drop_mode": True, "quick_drop_profile": profile,
                  "min_drop_targets_required": 1, "camera_cx": 320.0,
                  "camera_cy": 240.0, "area1_search_altitude": 4.5,
                  "bucket_calibration_valid": False}
    fsm.get_param = lambda key, default=None: parameters.get(key, default)
    fsm._known_cylinders = []
    fsm._known_markers = []
    fsm._next_known_cylinder_id = 1
    fsm._next_known_marker_id = 1001
    fsm._cylinder_capture_stamp = fsm._marker_capture_stamp = 0.0
    fsm._cylinders = fsm._markers = []
    fsm._cylinders_timestamp = fsm._markers_timestamp = 100.9
    fsm._current_state = MissionState.CYLINDER_SEARCH
    fsm._current_drop_cyl_id = -1
    fsm._drop_success_count = 0
    fsm._quick_bucket_gate = fsm._quick_bucket_pair = None
    fsm._cylinder_selector = CylinderSelector()
    fsm._layout_frame = LayoutFrame(0.0)
    fsm.zone_bounds = lambda _prefix: (30.0, 35.0, -4.0, 4.0)
    logger = Recorder()
    fsm.get_logger = lambda: logger
    fsm._task_logger = logger
    fsm._emergency = SimpleNamespace(update_vision=lambda _value: None)
    fsm.request_takeover = lambda reason: logger.error("takeover", reason)
    fsm._transition_to = lambda state: setattr(fsm, "_current_state", state)
    fsm._bridge = SimpleNamespace(
        get_current_position=lambda: SimpleNamespace(north=40.0, east=2.0, down=-4.5))
    fsm._fc = SimpleNamespace(
        altitude_to_ned=lambda height: -height, ned_to_altitude=lambda down: -down,
        goto_ned=lambda *args, **kw: logger.info("goto", args),
        set_velocity_body=lambda *args: logger.info("velocity", args),
        stop_velocity=lambda: logger.info("stop"))
    fsm.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(time.time() * 1e9)))
    return fsm


def detection(radius=0.075, x=2.0, y=40.0, u=320.0, v=240.0):
    return SimpleNamespace(id=1, confidence=0.9, world_position_valid=True,
                           world_x=x, world_y=y, radius=radius,
                           center_u=u, center_v=v)


def image_only_detection(radius=0.075, u=320.0, v=240.0):
    target = detection(radius, x=float("nan"), y=float("nan"))
    target.world_position_valid = False
    target.center_u = u
    target.center_v = v
    return target


def feed(fsm, stamp, objects):
    sec = int(stamp)
    fsm._cyl_cb(SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=sec, nanosec=round((stamp-sec)*1e9))), cylinders=objects))


def feed_marker(fsm, stamp, objects):
    sec = int(stamp)
    fsm._marker_cb(SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=sec, nanosec=round((stamp-sec)*1e9))), cylinders=objects))


def search(fsm):
    handler = CylinderSearchState(fsm)
    handler._start_time = time.time()
    handler._min_cylinders_required = 1
    return handler


def three_frames(fsm, objects):
    for stamp in (100.1, 100.2, 100.3):
        feed(fsm, stamp, objects)


def test_bucket_blue_marker_starts_guided_descent_after_distinct_frames(clock):
    fsm = mission()
    handler = search(fsm)
    marker = image_only_detection(u=365.0, v=240.0)
    for stamp in (100.65, 100.75):
        feed_marker(fsm, stamp, [marker])
        assert handler.execute() is None
        assert not any(event[0] == "velocity" for event in fsm._task_logger.events)
    # Re-reading one camera frame at 20 Hz cannot count as another observation.
    assert handler.execute() is None
    assert not any(event[0] == "velocity" for event in fsm._task_logger.events)

    feed_marker(fsm, 100.85, [marker])
    assert handler.execute() is None
    assert fsm._quick_bucket_gate is None
    forward, right, down = [event[1] for event in fsm._task_logger.events
                            if event[0] == "velocity"][-1]
    assert forward == pytest.approx(0.0)
    assert 0.0 < right <= 0.25
    assert down == pytest.approx(0.28)


def test_bucket_blue_guided_descent_stops_when_marker_is_stale(clock):
    fsm = mission()
    handler = search(fsm)
    for stamp in (100.65, 100.75, 100.85):
        feed_marker(fsm, stamp, [image_only_detection()])
        handler.execute()
    clock[0] += 0.5
    assert handler.execute() is None
    assert ("velocity", (0.0, 0.0, 0.0)) in fsm._task_logger.events
    assert fsm._task_logger.events[-1] == ("velocity", (0.0, 0.0, 0.0))


def test_bucket_blue_marker_near_image_edge_moves_without_descending(clock):
    fsm = mission()
    handler = search(fsm)
    for stamp in (100.65, 100.75, 100.85):
        feed_marker(fsm, stamp, [image_only_detection(u=620.0)])
        handler.execute()
    forward, right, down = [event[1] for event in fsm._task_logger.events
                            if event[0] == "velocity"][-1]
    assert forward == pytest.approx(0.0)
    assert right == pytest.approx(0.25)
    assert down == 0.0


def test_bucket_blue_guidance_times_out_if_altitude_does_not_progress(clock):
    fsm = mission()
    handler = search(fsm)
    for stamp in (100.65, 100.75, 100.85):
        feed_marker(fsm, stamp, [image_only_detection()])
        handler.execute()
    clock[0] += 16.0
    feed_marker(fsm, clock[0] - 0.05, [image_only_detection()])
    assert handler.execute() is None
    assert fsm._task_logger.events[-1][0] == "stop"
    assert handler._blue_descent_done is True


def test_bucket_guided_descent_selects_new_cylinder_before_floor(clock):
    fsm = mission()
    handler = search(fsm)
    for stamp in (100.65, 100.75, 100.85):
        feed_marker(fsm, stamp, [image_only_detection()])
        handler.execute()
    for stamp in (100.86, 100.87, 100.88):
        feed(fsm, stamp, [image_only_detection(radius=0.10)])
    for _ in range(4):
        assert handler.execute() is None
    assert handler.execute() == "SELECT_TARGET"
    assert fsm._task_logger.events[-1][0] == "stop"
    fsm._handle_select_target()
    assert fsm._current_state == MissionState.CYLINDER_APPROACH


def test_bucket_guided_descent_floor_scans_at_lower_height(clock):
    fsm = mission()
    altitude = [4.5]
    fsm._bridge.get_current_position = lambda: SimpleNamespace(
        north=40.0, east=2.0, down=-altitude[0])
    fsm._fc.reached = lambda *_args: False
    handler = search(fsm)
    for stamp in (100.65, 100.75, 100.85):
        feed_marker(fsm, stamp, [image_only_detection()])
        handler.execute()
    altitude[0] = 3.0
    assert handler.execute() is None
    assert fsm._task_logger.events[-1][0] == "stop"
    clock[0] += 11.0
    assert handler.execute() is None
    goto_args = [event[1] for event in fsm._task_logger.events
                 if event[0] == "goto"][-1]
    assert goto_args[2] == pytest.approx(-3.0)


def test_blue_profile_does_not_use_bucket_predescent(clock):
    fsm = mission(profile="blue")
    handler = search(fsm)
    for stamp in (100.65, 100.75, 100.85):
        feed_marker(fsm, stamp, [image_only_detection()])
        assert handler.execute() is None
    assert not any(event[0] == "velocity" for event in fsm._task_logger.events)


@pytest.mark.parametrize("radius", [0.04, 0.075, 0.10, 0.125, 0.30])
def test_outside_zone_cylinder_alone_can_enter_approach_after_three_frames(clock, radius):
    fsm = mission()
    three_frames(fsm, [detection(radius)])
    assert len(fsm.get_known_cylinders()) == 1
    handler = search(fsm)
    for _ in range(4):
        assert handler.execute() is None
    assert handler.execute() == "SELECT_TARGET"
    fsm._handle_select_target()
    assert fsm._current_state == MissionState.CYLINDER_APPROACH
    assert fsm._quick_bucket_gate.disc_ne is None


def test_bucket_image_without_world_projection_can_be_selected(clock):
    fsm = mission()
    three_frames(fsm, [image_only_detection()])
    handler = search(fsm)
    for _ in range(4):
        assert handler.execute() is None
    assert handler.execute() == "SELECT_TARGET"
    fsm._handle_select_target()
    assert fsm._current_state == MissionState.CYLINDER_APPROACH
    assert fsm._quick_bucket_gate is not None


def test_bucket_approach_uses_image_servo_without_world_or_goto(clock):
    fsm = mission()
    three_frames(fsm, [image_only_detection(u=380.0)])
    fsm._handle_select_target()
    approach = CylinderApproachState(fsm)
    approach.enter()
    fsm._cylinders = [image_only_detection(u=380.0)]
    fsm._cylinder_capture_stamp = 100.85
    assert approach.execute() is None
    assert any(event[0] == "velocity" for event in fsm._task_logger.events)
    assert not any(event[0] == "goto" for event in fsm._task_logger.events)
    assert not any(event[0] == "takeover" for event in fsm._task_logger.events)


def test_bucket_approach_holds_when_image_is_stale_and_world_is_invalid(clock):
    fsm = mission()
    three_frames(fsm, [image_only_detection(u=380.0)])
    fsm._handle_select_target()
    approach = CylinderApproachState(fsm)
    approach.enter()
    assert approach.execute() is None
    assert any(event[0] == "velocity" and event[1] == (0.0, 0.0, 0.0)
               for event in fsm._task_logger.events)
    assert not any(event[0] == "goto" for event in fsm._task_logger.events)


def test_bucket_world_reference_only_gives_bounded_coarse_move(clock):
    fsm = mission()
    three_frames(fsm, [detection(x=2.0, y=60.0)])
    fsm._handle_select_target()
    approach = CylinderApproachState(fsm)
    approach.enter()
    assert approach.execute() is None
    goto = next(event[1] for event in fsm._task_logger.events
                if event[0] == "goto")
    assert math.hypot(goto[0] - 40.0, goto[1] - 2.0) <= 0.5 + 1e-6
    assert not any(event[0] == "takeover" for event in fsm._task_logger.events)


def test_bucket_descent_uses_current_pixels_when_high_world_is_wrong(clock):
    fsm = mission()
    gate = BucketReleaseGate((100.0, 100.0), None, None)
    gate.bucket_pixel_uv = (320.0, 240.0)
    fsm._quick_bucket_gate = gate
    fsm._cylinders = [image_only_detection()]
    fsm._cylinder_capture_stamp = 100.85
    handler = DropState(fsm)
    handler.enter()
    steps = []
    handler._precision_aligner.step = lambda **kwargs: steps.append(kwargs)
    assert handler._execute_bucket_descend() is None
    assert steps[-1]["cylinders"] == fsm._cylinders
    assert steps[-1]["descent_speed"] > 0
    assert gate.bucket_ne == pytest.approx((40.0, 2.0))


def test_low_altitude_gate_uses_current_image_instead_of_high_world(clock):
    fsm = mission()
    previous_get_param = fsm.get_param
    geometry = {"bucket_home_body_height": 0.40,
                "bucket_camera_down": 0.365, "bucket_rim_height": 0.40}
    fsm.get_param = lambda key, default=None: geometry.get(
        key, previous_get_param(key, default))
    fsm._bridge.get_current_position = lambda: SimpleNamespace(
        north=40.0, east=2.0, down=-1.6)
    fsm._bridge._current_yaw = 0.0
    calibration = Calibration(
        0.045, -0.045, 0.0, 0.0, 0.0, 0.0, 0.40,
        0.025, 0.025, 0.10, home_body_height=0.40,
        front_down=0.345, rear_down=0.345)
    gate = BucketReleaseGate((100.0, 100.0), None, calibration)
    gate.bucket_pixel_uv = (320.0, 225.4)
    gate.start(100.0)
    gate.radius_confirmed = True
    fsm._quick_bucket_gate = gate
    fsm._cylinder_capture_stamp = fsm._marker_capture_stamp = 100.85
    fsm._cylinders = [image_only_detection(v=225.4)]
    fsm._markers = [image_only_detection(radius=0.5, v=225.4)]
    handler = DropState(fsm)
    (_, cylinder), (_, marker) = handler._bucket_current_objects(require_unique=True)
    result = gate.evaluate(
        now=100.9, altitude=1.6, north=40.0, east=2.0,
        yaw=0.0, vn=0.0, ve=0.0, vd=0.0, telemetry_stamp=100.85,
        cylinder=cylinder, marker=marker)
    assert gate.bucket_ne[0] == pytest.approx(40.045, abs=0.002)
    assert result.front_residual < 0.015
    assert result.reason == "collecting_bucket_frames"


def test_image_only_bucket_runs_search_approach_and_descend(clock):
    fsm = mission()
    target = image_only_detection()
    three_frames(fsm, [target])
    state = search(fsm)
    for _ in range(5):
        result = state.execute()
    assert result == "SELECT_TARGET"
    fsm._handle_select_target()
    approach = CylinderApproachState(fsm)
    approach.enter()
    for index in range(10):
        fsm._cylinder_capture_stamp = 100.81 + index * 0.01
        next_state = approach.execute()
    assert next_state == "DESCEND_TO_DROP"
    assert not any(event[0] == "goto" for event in fsm._task_logger.events)
    descend = DropState(fsm)
    descend.enter()
    steps = []
    descend._precision_aligner.step = lambda **kwargs: steps.append(kwargs)
    assert descend._execute_bucket_descend() is None
    assert steps[-1]["cylinders"] == [target]
    assert steps[-1]["descent_speed"] > 0


@pytest.mark.parametrize("radius", [0.02, 0.31, float("nan")])
def test_unreasonable_radius_is_still_rejected(clock, radius):
    fsm = mission()
    three_frames(fsm, [detection(radius, y=32.0)])
    handler = search(fsm)
    for _ in range(5):
        assert handler.execute() is None


@pytest.mark.parametrize("markers", [[], [(2.0, 45.0)], [(2.0, 32.0), (2.1, 32.0)]])
def test_missing_far_or_ambiguous_blue_discs_do_not_block_selection(clock, markers):
    fsm = mission()
    three_frames(fsm, [detection(0.10, y=32.0)])
    fsm._known_markers = [dict(world_x=x, world_y=y, observations=1) for x, y in markers]
    handler = search(fsm)
    for _ in range(5):
        result = handler.execute()
    assert result == "SELECT_TARGET"
    fsm._handle_select_target()
    assert fsm._current_state == MissionState.CYLINDER_APPROACH


def test_same_capture_and_duplicate_detections_cannot_supply_three_frames(clock):
    fsm = mission()
    for _ in range(3):
        feed(fsm, 100.1, [detection(y=32.0)] * 3)
    target = fsm.get_known_cylinders()[0]
    assert len(target.get("bucket_search_samples", {})) == 1
    handler = search(fsm)
    for _ in range(5):
        assert handler.execute() is None
    feed(fsm, 100.2, [detection(y=32.0)])
    feed(fsm, 100.3, [detection(y=32.0)])
    for _ in range(5):
        result = handler.execute()
    assert result == "SELECT_TARGET"


def test_old_search_evidence_expires_without_new_observations(clock):
    fsm = mission()
    three_frames(fsm, [detection(y=32.0)])
    handler = search(fsm)
    clock[0] = 103.0
    for _ in range(5):
        assert handler.execute() is None


def test_small_bucket_preference_falls_back_to_medium_when_small_was_already_selected(clock):
    fsm = mission()
    three_frames(fsm, [detection(0.04, x=0.0, y=32.0, u=180.0),
                       detection(0.075, x=2.0, y=32.0, u=320.0),
                       detection(0.12, x=4.0, y=32.0, u=460.0)])
    fsm._handle_select_target()
    assert fsm.cylinder_selector._selected_positions[-1] == (2.0, 32.0)
    fsm._handle_select_target()
    assert fsm.cylinder_selector._selected_positions[-1] == (0.0, 32.0)


def test_outside_zone_selected_target_reaches_cylinder_only_descent_servo(clock):
    fsm = mission()
    objects = [detection(0.125)]
    three_frames(fsm, objects)
    fsm._handle_select_target()
    approach = CylinderApproachState(fsm)
    approach.enter()
    assert approach._target_world_xy == (2.0, 40.0)
    for index in range(10):
        fsm._cylinder_capture_stamp = 100.81 + index * 0.01
        next_state = approach.execute()
    assert next_state == "DESCEND_TO_DROP"
    fsm._cylinder_capture_stamp = 100.85
    descend = DropState(fsm)
    descend.enter()
    steps = []
    descend._precision_aligner.step = lambda **kwargs: steps.append(kwargs)
    assert descend._execute_bucket_descend() is None
    assert steps[-1]["cylinders"] == objects
    assert steps[-1]["markers"] == []
    assert steps[-1]["descent_speed"] > 0
    assert steps[-1]["allow_release"] is False


def test_duplicate_current_detections_allow_servo_but_not_release_observation(clock):
    fsm = mission()
    fsm._quick_bucket_gate = BucketReleaseGate((40.0, 2.0), None, None)
    fsm._cylinder_capture_stamp = 100.85
    fsm._cylinders = [detection(), detection(x=2.1)]
    handler = DropState(fsm)
    assert handler._bucket_current_objects()[0][0] is fsm._cylinders[0]
    assert handler._bucket_current_objects(require_unique=True)[0] == (None, None)


def test_current_unique_blue_disc_can_become_release_anchor(clock):
    fsm = mission()
    gate = BucketReleaseGate((40.0, 2.0), None, None)
    fsm._quick_bucket_gate = gate
    fsm._cylinder_capture_stamp = fsm._marker_capture_stamp = 100.85
    fsm._cylinders = [detection()]
    fsm._markers = [detection(x=200.0, u=329.0)]
    handler = DropState(fsm)
    (_, cylinder), (_, marker) = handler._bucket_current_objects(require_unique=True)
    assert cylinder is not None and marker is not None
    assert gate.disc_ne == pytest.approx((40.0, 2.10125))


def test_ambiguous_blue_discs_never_create_release_anchor(clock):
    fsm = mission()
    gate = BucketReleaseGate((40.0, 2.0), None, None)
    fsm._quick_bucket_gate = gate
    fsm._cylinder_capture_stamp = fsm._marker_capture_stamp = 100.85
    fsm._cylinders = [detection()]
    fsm._markers = [detection(x=2.1), detection(x=1.9)]
    handler = DropState(fsm)
    assert handler._bucket_current_objects(require_unique=True)[1] == (None, None)
    assert gate.disc_ne is None


def test_two_rims_near_one_disc_never_create_release_anchor(clock):
    fsm = mission()
    gate = BucketReleaseGate((40.0, 2.0), None, None)
    fsm._quick_bucket_gate = gate
    fsm._cylinder_capture_stamp = fsm._marker_capture_stamp = 100.85
    fsm._cylinders = [detection(), detection(x=2.2)]
    fsm._markers = [detection(x=2.1)]
    handler = DropState(fsm)
    assert handler._bucket_current_objects(require_unique=True)[1] == (None, None)
    assert gate.disc_ne is None


def test_missing_blue_disc_anchor_can_never_release(clock):
    gate = BucketReleaseGate((40.0, 2.0), None, None)
    gate.start(100.0)
    result = gate.evaluate(now=100.9, altitude=1.6, north=40.0, east=2.0,
                           yaw=0.0, vn=0.0, ve=0.0, vd=0.0,
                           telemetry_stamp=100.85,
                           cylinder=Observation(100.85, 40.0, 2.0),
                           marker=Observation(100.85, 40.0, 2.0))
    assert result.action != "release"


def test_blue_profile_keeps_drop_zone_memory_filter(clock):
    fsm = mission("blue")
    three_frames(fsm, [detection()])
    assert fsm.get_known_cylinders() == []
