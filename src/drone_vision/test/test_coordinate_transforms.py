import math
import importlib.util
from pathlib import Path

import numpy as np

from drone_vision.coordinate_transforms import CoordinateTransformer


_TELEMETRY_PATH = (
    Path(__file__).resolve().parents[2]
    / "px4_control"
    / "px4_control"
    / "telemetry.py"
)
_TELEMETRY_SPEC = importlib.util.spec_from_file_location(
    "px4_control_telemetry_for_test",
    _TELEMETRY_PATH,
)
telemetry = importlib.util.module_from_spec(_TELEMETRY_SPEC)
_TELEMETRY_SPEC.loader.exec_module(telemetry)
ned_to_world_pose = telemetry.ned_to_world_pose


def _transformer():
    return CoordinateTransformer(
        np.array(
            [
                [539.9, 0.0, 640.0],
                [0.0, 539.9, 480.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
    )


def test_downward_camera_center_projects_to_drone_world_xy():
    tf = _transformer()
    pose = ned_to_world_pose(
        north=0.0,
        east=32.5,
        down=-5.0,
        q_x=0.0,
        q_y=0.0,
        q_z=0.0,
        q_w=1.0,
    )

    world_xy = tf.pixel_to_world(
        640.0,
        480.0,
        pose[:3],
        pose[3:],
        q_convention="xyzw",
    )

    assert world_xy is not None
    assert math.isclose(world_xy[0], 42.5, abs_tol=1e-6)
    assert math.isclose(world_xy[1], 4.0, abs_tol=1e-6)


def test_downward_camera_pixel_axes_match_world_enu_axes():
    tf = _transformer()
    pose = ned_to_world_pose(
        north=0.0,
        east=32.5,
        down=-5.0,
        q_x=0.0,
        q_y=0.0,
        q_z=0.0,
        q_w=1.0,
    )

    one_meter_px = 539.9 / 5.0
    east_xy = tf.pixel_to_world(
        640.0 + one_meter_px,
        480.0,
        pose[:3],
        pose[3:],
        q_convention="xyzw",
    )
    north_xy = tf.pixel_to_world(
        640.0,
        480.0 + one_meter_px,
        pose[:3],
        pose[3:],
        q_convention="xyzw",
    )

    assert east_xy is not None
    assert north_xy is not None
    assert math.isclose(east_xy[0], 43.5, abs_tol=1e-6)
    assert math.isclose(east_xy[1], 4.0, abs_tol=1e-6)
    assert math.isclose(north_xy[0], 42.5, abs_tol=1e-6)
    assert math.isclose(north_xy[1], 5.0, abs_tol=1e-6)
