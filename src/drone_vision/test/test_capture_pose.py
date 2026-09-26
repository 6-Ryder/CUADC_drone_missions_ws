import math

from drone_vision.capture_pose import interpolate_pose


def test_interpolates_position_and_normalized_quaternion():
    history = [(1.0, (0, 0, 2, 0, 0, 0, 1)),
               (1.1, (1, 2, 4, 0, 0, 0.2, 0.98))]
    pose = interpolate_pose(history, 1.05)
    assert pose[:3] == (0.5, 1.0, 3.0)
    assert math.isclose(sum(v*v for v in pose[3:]), 1.0)


def test_rejects_missing_or_distant_capture_pose():
    history = [(1.0, (0, 0, 2, 0, 0, 0, 1))]
    assert interpolate_pose(history, 1.2) is None
    assert interpolate_pose(history, 0.0) is None
    assert interpolate_pose([], 1.0) is None
