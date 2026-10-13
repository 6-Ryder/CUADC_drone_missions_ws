"""LostTargetTransit 的契约.

本类的作用: 视觉目标丢失后, 用**最后已知位置**继续接近, 而不是原地悬停
(PX4 precland 水平接近段的语义). 它的风险面很窄但很关键 —— 这是在没有测量
的情况下发速度指令, 因此三件事必须钉死:

  1. 符号/坐标约定与 flight_controller.set_velocity_body 一致. 不一致会
     直接变成"朝错误方向飞" (本项目历史上的固定方向偏置类问题).
  2. 锁存点在**世界系**固定, 不随航向变化. 否则航向一动目标就漂.
  3. 到点后输出零速度, 不退化成按最后方向持续飞 (那才是漂移).
"""

import math

import pytest

from drone_utils.lost_target_transit import LostTargetTransit


def test_no_command_before_any_latch():
    """从未锁存过目标 → 零速度. 失败方向是不动作, 与原有悬停行为一致."""
    transit = LostTargetTransit()
    assert transit.command(0.0, 0.0, 0.0) == (0.0, 0.0)
    assert transit.has_target is False


def test_latch_places_target_at_vehicle_plus_offset_yaw_zero():
    """yaw=0 时机体系与 NED 重合: 目标 = 载机位置 + (前向, 右向) 偏移."""
    transit = LostTargetTransit()
    transit.latch(north=10.0, east=20.0, yaw=0.0,
                  error_forward=1.0, error_right=-0.5)

    assert transit.target == pytest.approx((11.0, 19.5))


def test_command_matches_set_velocity_body_convention_at_yaw_zero():
    """yaw=0: 目标在正北 → forward 为正, right 为 0."""
    transit = LostTargetTransit()
    transit.latch(north=0.0, east=0.0, yaw=0.0,
                  error_forward=2.0, error_right=0.0)

    forward, right = transit.command(0.0, 0.0, 0.0)
    assert forward > 0.0
    assert right == pytest.approx(0.0, abs=1e-9)


def test_command_rotates_with_yaw_like_set_velocity_body():
    """机体系指令必须是 NED 误差按当前航向旋转的逆.

    对照 flight_controller.set_velocity_body 的正向约定
    (vn = f·cos − r·sin, ve = f·sin + r·cos): 航向 π/2 (机头朝东) 时,
    正北方向的目标应落在机体**左侧** (right 为负).
    """
    transit = LostTargetTransit()
    transit.latch(north=0.0, east=0.0, yaw=0.0,
                  error_forward=2.0, error_right=0.0)

    forward, right = transit.command(0.0, 0.0, math.pi / 2.0)
    assert forward == pytest.approx(0.0, abs=1e-9)
    assert right < 0.0

    # 正向核对: 把该机体系指令用 set_velocity_body 的公式转回 NED,
    # 必须仍指向目标 (北).
    yaw = math.pi / 2.0
    vn = forward * math.cos(yaw) - right * math.sin(yaw)
    ve = forward * math.sin(yaw) + right * math.cos(yaw)
    assert vn > 0.0
    assert ve == pytest.approx(0.0, abs=1e-9)


def test_latched_point_is_world_fixed_across_yaw_change():
    """锁存点是世界坐标: 载机不动、航向变化时, 目标位置不变.

    若误锁存机体系误差, 航向一转目标就会跟着转 —— 这是"漂移"的直接来源.
    """
    transit = LostTargetTransit()
    transit.latch(north=0.0, east=0.0, yaw=0.0,
                  error_forward=1.0, error_right=1.0)
    assert transit.target == pytest.approx((1.0, 1.0))

    # 航向改变不影响锁存点; 指令方向随之旋转, 但指的还是同一个点
    forward, right = transit.command(0.0, 0.0, math.pi)
    yaw = math.pi
    vn = forward * math.cos(yaw) - right * math.sin(yaw)
    ve = forward * math.sin(yaw) + right * math.cos(yaw)
    assert vn > 0.0 and ve > 0.0


def test_zero_command_inside_tolerance():
    """到点容差内输出零速度 —— 不退化成按最后方向继续飞."""
    transit = LostTargetTransit(tolerance=0.15)
    transit.latch(north=0.0, east=0.0, yaw=0.0,
                  error_forward=1.0, error_right=0.0)

    assert transit.command(0.9, 0.0, 0.0) == (0.0, 0.0)   # 剩 0.1 < 0.15
    assert transit.command(0.8, 0.0, 0.0) != (0.0, 0.0)   # 剩 0.2 > 0.15


def test_speed_is_capped_and_shrinks_near_target():
    """速度不超过上限, 且按剩余距离收尾 (避免到点附近来回冲)."""
    transit = LostTargetTransit(max_speed=0.15, tolerance=0.01)
    transit.latch(north=0.0, east=0.0, yaw=0.0,
                  error_forward=5.0, error_right=0.0)

    far = transit.command(0.0, 0.0, 0.0)
    assert math.hypot(*far) == pytest.approx(0.15)

    near = transit.command(4.9, 0.0, 0.0)   # 剩 0.1 m
    assert math.hypot(*near) == pytest.approx(0.1)


def test_non_finite_inputs_are_ignored():
    """非有限输入既不污染锁存点, 也不产生指令 (绝不把 NaN 传进飞控)."""
    transit = LostTargetTransit()
    transit.latch(0.0, 0.0, 0.0, 1.0, 1.0)
    transit.latch(math.nan, 0.0, 0.0, 1.0, 1.0)

    assert transit.target == pytest.approx((1.0, 1.0))

    transit.latch(0.0, 0.0, 0.0, math.nan, 0.0)
    assert transit.target == pytest.approx((1.0, 1.0))

    assert transit.command(math.nan, 0.0, 0.0) == (0.0, 0.0)
    assert transit.command(0.0, 0.0, math.nan) == (0.0, 0.0)


def test_reset_clears_latch():
    """换相/重新捕获时必须清锁存, 否则上一段的点会继续驱动."""
    transit = LostTargetTransit()
    transit.latch(0.0, 0.0, 0.0, 1.0, 0.0)
    assert transit.has_target is True

    transit.reset()
    assert transit.has_target is False
    assert transit.command(0.0, 0.0, 0.0) == (0.0, 0.0)


def test_rejects_invalid_construction():
    with pytest.raises(ValueError):
        LostTargetTransit(max_speed=0.0)
    with pytest.raises(ValueError):
        LostTargetTransit(max_speed=math.nan)
    with pytest.raises(ValueError):
        LostTargetTransit(tolerance=-1.0)
