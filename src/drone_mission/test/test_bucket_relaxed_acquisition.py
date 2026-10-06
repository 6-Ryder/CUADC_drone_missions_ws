"""Relaxed bucket acquisition through real FSM/search/approach/descent handlers."""

import time
from types import SimpleNamespace

import pytest

from drone_mission.cylinder_selector import CylinderSelector
from drone_mission.mission_state_machine import MissionFSM, MissionState
from drone_mission.quick_bucket_gate import BucketReleaseGate, Observation
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
        set_velocity_body=lambda *args: logger.info("velocity", args))
    return fsm


def detection(radius=0.075, x=2.0, y=40.0):
    return SimpleNamespace(id=1, confidence=0.9, world_position_valid=True,
                           world_x=x, world_y=y, radius=radius,
                           center_u=320.0, center_v=240.0)


def feed(fsm, stamp, objects):
    sec = int(stamp)
    fsm._cyl_cb(SimpleNamespace(
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
    three_frames(fsm, [detection(0.04, x=0.0, y=32.0),
                       detection(0.075, x=2.0, y=32.0),
                       detection(0.12, x=4.0, y=32.0)])
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
    for _ in range(10):
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
    fsm._markers = [detection(x=2.1)]
    handler = DropState(fsm)
    (_, cylinder), (_, marker) = handler._bucket_current_objects(require_unique=True)
    assert cylinder is not None and marker is not None
    assert gate.disc_ne == pytest.approx((40.0, 2.1))


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
