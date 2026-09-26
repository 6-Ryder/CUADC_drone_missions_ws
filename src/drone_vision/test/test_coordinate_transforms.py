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


def test_gz_camera_convention_v_plus_points_north():
    """gz SITL 相机 (image_top_faces_forward=False): 像素 v+ → 世界北 (旧约定).

    无人机 World(32.5, 0, 5), 水平下视. 画面下 1m → 世界北 1m.
    """
    tf = CoordinateTransformer(
        np.array(
            [
                [539.9, 0.0, 640.0],
                [0.0, 539.9, 480.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        ),
        image_top_faces_forward=False,
    )
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
    v_down_xy = tf.pixel_to_world(
        640.0,
        480.0 + one_meter_px,
        pose[:3],
        pose[3:],
        q_convention="xyzw",
    )
    assert v_down_xy is not None
    # 中心 World(32.5, 0), 画面下1m → 世界北 1m
    assert math.isclose(v_down_xy[0], 32.5, abs_tol=1e-6)
    assert math.isclose(v_down_xy[1], 1.0, abs_tol=1e-6)


def test_downward_camera_center_projects_to_drone_world_xy():
    """中心像素 (640,480) 投影到无人机正下方的世界坐标."""
    tf = _transformer()
    # 使用默认 home=(0,0,0), NED(0, 32.5, -5) → World(32.5, 0, 5)
    # 水平姿态下, 中心像素对应正下方 → World(32.5, 0, 0)
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
    assert math.isclose(world_xy[0], 32.5, abs_tol=1e-6)
    assert math.isclose(world_xy[1], 0.0, abs_tol=1e-6)


def test_measured_camera_forward_offset_moves_center_projection():
    tf = CoordinateTransformer(
        np.array([[400.0, 0, 320.0], [0, 400.0, 240.0], [0, 0, 1.0]]),
        camera_body_offset=(0.10, 0.0, 0.0),
    )
    pose = ned_to_world_pose(0.0, 0.0, -2.0, 0, 0, 0, 1)
    world = tf.pixel_to_world(320, 240, pose[:3], pose[3:])
    assert world is not None
    assert math.isclose(world[1], 0.10, abs_tol=1e-6)


def test_downward_camera_pixel_axes_match_world_enu_axes():
    """像素 x→世界东, 像素 y→世界南 (画面顶朝机头安装).

    无人机 World(32.5, 0, 5), 水平下视.
    - 图像 u+dx (右) → 世界东移 → (32.5 + Δ, 0)
    - 图像 v+dy (下) → 世界南移 → (32.5, 0 - Δ)
    1m 对应像素 = fx / altitude = 539.9 / 5.0
    """
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
    south_xy = tf.pixel_to_world(
        640.0,
        480.0 + one_meter_px,
        pose[:3],
        pose[3:],
        q_convention="xyzw",
    )

    assert east_xy is not None
    assert south_xy is not None
    # home=(0,0,0): 中心 World(32.5, 0), 东1m→(33.5, 0), 南1m→(32.5, -1.0)
    assert math.isclose(east_xy[0], 33.5, abs_tol=1e-6)
    assert math.isclose(east_xy[1], 0.0, abs_tol=1e-6)
    assert math.isclose(south_xy[0], 32.5, abs_tol=1e-6)
    assert math.isclose(south_xy[1], -1.0, abs_tol=1e-6)
