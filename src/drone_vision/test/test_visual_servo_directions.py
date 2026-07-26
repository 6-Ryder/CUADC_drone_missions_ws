import math

from drone_vision.visual_servoing import VisualServoController


def _controller():
    return VisualServoController(kp=1.0, ki=0.0, kd=0.0)


def test_target_right_commands_east_at_zero_yaw():
    state = _controller().update(420, 240, 320, 240, 5.0, 400.0, 400.0)

    assert abs(state.velocity_north) < 1e-6
    assert state.velocity_east > 0.0


def test_target_below_commands_north_at_zero_yaw():
    """像下=机头(北)约定: 目标在画面下方→命令向北速度."""
    state = _controller().update(320, 340, 320, 240, 5.0, 400.0, 400.0)

    assert state.velocity_north > 0.0
    assert abs(state.velocity_east) < 1e-6


def test_camera_right_rotates_to_south_at_east_heading():
    state = _controller().update(
        420, 240, 320, 240, 5.0, 400.0, 400.0,
        current_yaw=math.pi / 2,
    )

    assert state.velocity_north < 0.0
    assert abs(state.velocity_east) < 1e-6
