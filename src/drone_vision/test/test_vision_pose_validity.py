"""Fresh-pose contract tests for visual World-coordinate projection."""

import math
import time
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped

from drone_vision.coordinate_transforms import CoordinateTransformer
from drone_vision.vision_pipeline import VisionPipelineNode


class _Projector:
    def __init__(self):
        self.result = None
        self.calls = 0

    def pixel_to_world(self, *_args, **_kwargs):
        self.calls += 1
        return self.result


class _Logger:
    def warn(self, _message):
        pass


def bare_pipeline(*, last_pose, pose_valid, max_age=0.5):
    node = object.__new__(VisionPipelineNode)
    node._pose_received = pose_valid
    node._last_pose_monotonic = last_pose
    node._coord_xform = None
    node._current_altitude = 5.0
    node._drone_pose = (25.0, 30.0, 5.0, 0.0, 0.0, 0.0, 1.0)
    node.get_parameter = lambda name: SimpleNamespace(
        value={"vision_pose_max_age_s": max_age}[name]
    )
    return node


def pipeline_with_fresh_pose(*, yaw_ned):
    node = bare_pipeline(last_pose=time.monotonic(), pose_valid=True)
    # A non-north yaw makes this test reject a north-assuming projection path.
    node._drone_pose = (
        25.0,
        30.0,
        5.0,
        0.0,
        0.0,
        math.sin(yaw_ned / 2.0),
        math.cos(yaw_ned / 2.0),
    )
    projector = _Projector()
    node._coord_xform = projector
    return node, projector


def test_pose_is_unavailable_until_first_pose_callback():
    node = bare_pipeline(last_pose=None, pose_valid=False)

    assert not node._pose_is_fresh(now_monotonic=10.0)


def test_pose_older_than_half_second_is_rejected():
    node = bare_pipeline(last_pose=9.49, pose_valid=True, max_age=0.5)

    assert not node._pose_is_fresh(now_monotonic=10.0)


def test_fresh_fifty_degree_pose_uses_full_projection():
    node, projector = pipeline_with_fresh_pose(yaw_ned=math.radians(50.0))
    projector.result = (28.0, 31.0)

    assert node._project_world_xy(640.0, 480.0) == (28.0, 31.0)
    assert projector.calls == 1


def test_invalid_full_projection_has_no_north_assuming_fallback():
    node, projector = pipeline_with_fresh_pose(yaw_ned=math.radians(50.0))
    projector.result = None

    assert node._project_world_xy(640.0, 480.0) is None
    assert not hasattr(CoordinateTransformer, "pixel_to_world_simple")


def test_invalid_pose_revokes_a_previously_fresh_projection():
    node, projector = pipeline_with_fresh_pose(yaw_ned=math.radians(50.0))
    node.get_logger = lambda: _Logger()
    projector.result = (28.0, 31.0)
    invalid_pose = PoseStamped()
    invalid_pose.pose.position.x = math.nan

    node._pose_cb(invalid_pose)

    assert not node._pose_received
    assert node._last_pose_monotonic is None
    assert not node._pose_is_fresh(now_monotonic=time.monotonic())
    assert node._project_world_xy(640.0, 480.0) is None
    assert projector.calls == 0


def test_near_zero_quaternion_revokes_a_previously_fresh_projection():
    node, projector = pipeline_with_fresh_pose(yaw_ned=math.radians(50.0))
    node.get_logger = lambda: _Logger()
    projector.result = (28.0, 31.0)
    invalid_pose = PoseStamped()
    invalid_pose.pose.orientation.w = 1e-7

    node._pose_cb(invalid_pose)

    assert not node._pose_received
    assert node._last_pose_monotonic is None
    assert not node._pose_is_fresh(now_monotonic=time.monotonic())
    assert node._project_world_xy(640.0, 480.0) is None
    assert projector.calls == 0


def test_bucket_projection_is_independent_of_px4_vertical_origin():
    results = []
    for origin in (-7.817544460296631, 0.0):
        node = bare_pipeline(last_pose=time.monotonic(), pose_valid=True)
        node._bucket_projection = True
        node._bucket_ground_z = origin - 0.40
        node._frame_pose = (0.0, 0.0, origin + 5.6,
                            math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
        node._coord_xform = CoordinateTransformer(
            np.array([[539.9, 0, 320], [0, 539.9, 240], [0, 0, 1]]),
            camera_body_offset=(0.0, 0.0, 0.365))
        results.append(node._project_world_xy(600.0, 450.0, 0.40))

    assert results[0] is not None
    assert results[0] == pytest.approx(results[1])


def test_bucket_pose_height_uses_anchored_ground_not_world_z():
    node = bare_pipeline(last_pose=None, pose_valid=False)
    node._bucket_projection = True
    node._bucket_ground_z = None
    node.get_parameter = lambda name: SimpleNamespace(value={
        "bucket_home_body_height": 0.40,
        "vision_pose_max_age_s": 0.5,
    }[name])
    pose = PoseStamped()
    pose.pose.orientation.w = 1.0
    pose.pose.position.z = -7.817544460296631
    node._pose_cb(pose)
    assert node._current_altitude == pytest.approx(0.40)

    pose.pose.position.z += 5.6
    node._pose_cb(pose)
    assert node._current_altitude == pytest.approx(6.0)
