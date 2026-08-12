"""Mission path contracts for an arbitrarily rotated field layout."""

import math
import pytest

from drone_mission.mission_state_machine import MissionFSM
from drone_mission.path_planner import PathPlanner
from drone_mission.states.state_cylinder_search import CylinderSearchState
from drone_utils.layout_frame import LayoutFrame
from px4_control import telemetry


class _Logger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


def inverse_local_points(waypoints, frame):
    return [frame.ned_to_local(wp["north"], wp["east"]) for wp in waypoints]


def path_length(waypoints):
    return sum(
        math.hypot(
            end["north"] - start["north"],
            end["east"] - start["east"],
        )
        for start, end in zip(waypoints, waypoints[1:])
    )


def test_zero_degree_recon_path_is_the_existing_quick_path():
    planner = PathPlanner(layout_frame=LayoutFrame(0.0), grid_spacing=1.67)
    path = planner.plan_coverage_ned(55.0, 60.0, -4.0, 4.0, altitude=-2.5)

    assert [(wp["north"], wp["east"]) for wp in path] == [
        (55.0, -4.0), (55.0, 4.0),
        (57.5, 4.0), (57.5, -4.0),
        (60.0, -4.0), (60.0, 4.0),
    ]
    assert {wp["yaw"] for wp in path} == {0.0}


def test_fifty_degree_path_is_a_rigid_rotation_with_constant_yaw():
    base = PathPlanner(layout_frame=LayoutFrame(0.0), grid_spacing=1.67)
    rotated = PathPlanner(layout_frame=LayoutFrame(50.0), grid_spacing=1.67)
    path0 = base.plan_coverage_ned(55.0, 60.0, -4.0, 4.0, altitude=-2.5)
    path50 = rotated.plan_coverage_ned(55.0, 60.0, -4.0, 4.0, altitude=-2.5)

    local50 = inverse_local_points(path50, LayoutFrame(50.0))
    local0 = [(wp["north"], wp["east"]) for wp in path0]
    assert len(local50) == len(local0)
    for actual, expected in zip(local50, local0):
        assert actual == pytest.approx(expected)
    assert all(
        wp["yaw"] == pytest.approx(LayoutFrame(50.0).yaw_rad)
        for wp in path50
    )
    assert path_length(path50) == pytest.approx(path_length(path0), abs=1e-9)


class _SearchFC:
    def __init__(self):
        self.goto_calls = []

    @staticmethod
    def altitude_to_ned(altitude):
        return -altitude

    def goto_ned(self, *args, **kwargs):
        self.goto_calls.append((args, kwargs))


class _SearchFSM:
    def __init__(self, heading_deg):
        self.layout_frame = LayoutFrame(heading_deg)
        self.fc = _SearchFC()
        self._params = {
            "area1_forward_min": 30.0,
            "area1_forward_max": 35.0,
            "area1_right_min": -4.0,
            "area1_right_max": 4.0,
            "area1_search_altitude": 5.0,
            "sweep_speed": 0.8,
            "quick_drop_mode": True,
        }

    def get_param(self, name, default=None):
        return self._params.get(name, default)

    def zone_bounds(self, prefix):
        return tuple(
            self._params[f"{prefix}_{suffix}"]
            for suffix in ("forward_min", "forward_max", "right_min", "right_max")
        )


def test_drop_scan_endpoints_are_local_right_edges_at_fifty_degrees():
    fsm = _SearchFSM(heading_deg=50.0)
    state = CylinderSearchState(fsm)

    state._start_scan()

    local = [
        fsm.layout_frame.ned_to_local(wp["north"], wp["east"])
        for wp in state._scan_waypoints
    ]
    assert local[0] == pytest.approx((32.5, -4.0))
    assert local[1] == pytest.approx((32.5, 4.0))


@pytest.fixture
def rotated_fsm():
    home = (
        telemetry._home_world_x,
        telemetry._home_world_y,
        telemetry._home_world_z,
    )
    telemetry.configure_home(4.0, 10.0, 0.0)
    fsm = object.__new__(MissionFSM)
    fsm._layout_frame = LayoutFrame(50.0)
    fsm.test_logger = _Logger()
    params = {
        "area1_forward_min": 30.0,
        "area1_forward_max": 35.0,
        "area1_right_min": -4.0,
        "area1_right_max": 4.0,
    }
    fsm.get_param = lambda name, default=None: params.get(name, default)
    fsm.get_logger = lambda: fsm.test_logger
    try:
        yield fsm
    finally:
        telemetry.configure_home(*home)


def test_rotated_drop_containment_accepts_a_point_inside_local_rectangle(rotated_fsm):
    north, east = rotated_fsm.layout_frame.local_to_ned(32.5, 0.0)
    world_x, world_y, _ = telemetry.ned_to_world(north, east, 0.0)

    assert rotated_fsm.validate_drop_target_world_xy(world_x, world_y) == pytest.approx(
        (world_x, world_y)
    )


def test_rotated_drop_containment_rejects_a_world_aabb_only_corner(rotated_fsm):
    corners = [
        rotated_fsm.layout_frame.local_to_ned(forward, right)
        for forward in (30.0, 35.0)
        for right in (-4.0, 4.0)
    ]
    north = max(point[0] for point in corners)
    east = max(point[1] for point in corners)
    world_x, world_y, _ = telemetry.ned_to_world(north, east, 0.0)

    assert rotated_fsm.validate_drop_target_world_xy(world_x, world_y) is None
    assert rotated_fsm.is_in_drop_zone_world(world_x, world_y, margin=1.0) is False


def test_rotated_memory_margin_is_wider_than_approach_margin(rotated_fsm):
    north, east = rotated_fsm.layout_frame.local_to_ned(35.75, 0.0)
    world_x, world_y, _ = telemetry.ned_to_world(north, east, 0.0)

    assert rotated_fsm.validate_drop_target_world_xy(
        world_x, world_y,
    ) == pytest.approx((world_x, world_y))
    assert rotated_fsm.is_in_drop_zone_world(
        world_x, world_y, margin=0.5,
    ) is False
    assert rotated_fsm.test_logger.warnings == []


def test_drop_zone_predicate_rejects_silently_but_memory_validation_warns(rotated_fsm):
    north, east = rotated_fsm.layout_frame.local_to_ned(40.0, 0.0)
    world_x, world_y, _ = telemetry.ned_to_world(north, east, 0.0)

    assert rotated_fsm.is_in_drop_zone_world(world_x, world_y, margin=0.5) is False
    assert rotated_fsm.test_logger.warnings == []

    assert rotated_fsm.validate_drop_target_world_xy(world_x, world_y) is None
    assert len(rotated_fsm.test_logger.warnings) == 1
