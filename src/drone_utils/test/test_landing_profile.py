"""landing_profile 的契约 (跨两条降落分支共用的算法).

合并到 drone_utils 的原因就是"两条分支必须用同一份实现" —— 所以这里既测数值,
也测**退化方向**: 配置缺失/非法时必须退回"原有行为", 而不是产生一个新的、
没人整定过的速度.
"""

import math

import pytest

from drone_utils.landing_profile import (
    LAND_DESCENT_FLOOR_MS,
    ground_error_m,
    hold_at_search_altitude,
    near_ground_descent_speed,
    nominal_descent_speed,
)


# ---------------------------------------------------------------------------
# ground_error_m
# ---------------------------------------------------------------------------

def test_ground_error_uses_altitude_over_focal_length():
    """误差随高度线性放大 —— 这就是"同一个像素值多少米"随高度变的原因."""
    at_5m = ground_error_m(50.0, 0.0, 5.0, 400.0, 400.0)
    at_1m = ground_error_m(50.0, 0.0, 1.0, 400.0, 400.0)

    assert at_5m == pytest.approx(50.0 * 5.0 / 400.0)
    assert at_1m == pytest.approx(50.0 * 1.0 / 400.0)
    assert at_5m == pytest.approx(5.0 * at_1m)


def test_ground_error_is_isotropic_in_image_axes():
    """两轴用各自的焦距, 结果是模长 (与像素误差的符号无关)."""
    assert ground_error_m(30.0, 40.0, 2.0, 400.0, 400.0) == pytest.approx(
        math.hypot(30.0, 40.0) * 2.0 / 400.0)
    assert ground_error_m(-30.0, -40.0, 2.0, 400.0, 400.0) == pytest.approx(
        ground_error_m(30.0, 40.0, 2.0, 400.0, 400.0))


@pytest.mark.parametrize("du,dv,alt,fx,fy", [
    (math.nan, 0.0, 2.0, 400.0, 400.0),
    (0.0, math.inf, 2.0, 400.0, 400.0),
    (0.0, 0.0, -1.0, 400.0, 400.0),
    (0.0, 0.0, 2.0, 0.0, 400.0),
    (0.0, 0.0, 2.0, 400.0, -5.0),
])
def test_ground_error_rejects_invalid_input_as_infinite(du, dv, alt, fx, fy):
    """非法输入返回 inf —— 调用方据此走"不可用"分支, 不用错数继续控制."""
    assert ground_error_m(du, dv, alt, fx, fy) == math.inf


# ---------------------------------------------------------------------------
# near_ground_descent_speed
# ---------------------------------------------------------------------------

def test_aligned_descends_at_nominal_speed():
    assert near_ground_descent_speed(0.4, 0.0, 0.1) == pytest.approx(0.4)


def test_speed_decreases_linearly_with_lateral_error():
    """误差→速度是线性的, 严格单调递减 (ArduPilot mode.cpp:747-756 的结构)."""
    speeds = [near_ground_descent_speed(0.4, e, 0.2)
              for e in (0.0, 0.05, 0.1, 0.15, 0.2)]
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[1] == pytest.approx(0.4 * (1 - 0.05 / 0.2))
    assert speeds[2] == pytest.approx(0.2)


def test_never_stops_descending_even_when_far_off():
    """偏心再大也保持下降 —— 否则就退化成"悬停等超时"这一失败模式."""
    assert near_ground_descent_speed(0.4, 5.0, 0.1) == pytest.approx(
        LAND_DESCENT_FLOOR_MS)
    assert near_ground_descent_speed(0.4, 0.1, 0.1) == pytest.approx(
        LAND_DESCENT_FLOOR_MS)


def test_floor_never_exceeds_nominal():
    """名义速度已经比下限还慢时, 不得把它抬高 —— 那会凭空加快近地下降."""
    assert near_ground_descent_speed(0.05, 9.9, 0.1) == pytest.approx(0.05)
    assert near_ground_descent_speed(0.05, 0.0, 0.1) == pytest.approx(0.05)


def test_never_exceeds_nominal_speed():
    """调度只能减速, 不能加速 (误差为负这类脏输入也不例外)."""
    assert near_ground_descent_speed(0.3, -1.0, 0.1) == pytest.approx(0.3)


@pytest.mark.parametrize("error,acceptable", [
    (0.05, 0.0),          # 调度未配置 (acceptable<=0) → 原行为
    (0.05, -1.0),
    (0.05, math.nan),
    (math.nan, 0.1),      # 误差不可用 → 当作已对准
    (math.inf, 0.1),
])
def test_unusable_scheduling_inputs_keep_nominal_speed(error, acceptable):
    """调度参数不可用时退回原行为, 不引入未整定的新速度."""
    assert near_ground_descent_speed(0.4, error, acceptable) == pytest.approx(0.4)


@pytest.mark.parametrize("nominal", [math.nan, 0.0, -0.2, math.inf])
def test_unusable_nominal_speed_is_returned_unchanged(nominal):
    """名义速度本身不可用时原样返回 —— 由调用方的既有分层逻辑负责它."""
    result = near_ground_descent_speed(nominal, 0.05, 0.1)
    if math.isnan(nominal):
        assert math.isnan(result)
    else:
        assert result == nominal


# ---------------------------------------------------------------------------
# nominal_descent_speed / hold_at_search_altitude
# ---------------------------------------------------------------------------

def test_nominal_descent_speed_uses_three_altitude_bands():
    """三档分界 3.0 / 1.5 m, 与两条降落分支合并前的行为逐值一致."""
    assert nominal_descent_speed(5.0, 0.4, 0.25, 0.12) == 0.4
    assert nominal_descent_speed(3.01, 0.4, 0.25, 0.12) == 0.4
    assert nominal_descent_speed(3.0, 0.4, 0.25, 0.12) == 0.25
    assert nominal_descent_speed(1.6, 0.4, 0.25, 0.12) == 0.25
    assert nominal_descent_speed(1.5, 0.4, 0.25, 0.12) == 0.12
    assert nominal_descent_speed(0.2, 0.4, 0.25, 0.12) == 0.12


class _SearchFC:
    def __init__(self):
        self.waypoints = []
        self.loiter_calls = 0

    def altitude_to_ned(self, altitude):
        return -float(altitude)

    def goto_ned(self, north, east, down, yaw=None, speed=1.0):
        self.waypoints.append((north, east, down, speed))

    def loiter(self, *_a):
        self.loiter_calls += 1


class _SearchFSM:
    def __init__(self, altitude=5.0, speed=0.5, position=(1.0, 2.0)):
        self.fc = _SearchFC()
        self._params = {"landing_search_altitude": altitude,
                        "landing_approach_speed": speed}
        self._position = type("P", (), {"north": position[0],
                                        "east": position[1], "down": -4.0})()

    class _Bridge:
        def __init__(self, position):
            self._position = position

        def get_current_position(self):
            return self._position

    def get_param(self, key, default=None):
        return self._params.get(key, default)

    def loiter(self, *_a):
        self.fc.loiter()


def _search_fsm(**kwargs):
    fsm = _SearchFSM(**kwargs)
    fsm.bridge = _SearchFSM._Bridge(fsm._position)
    return fsm


def test_search_hold_commands_configured_altitude_at_approach_speed():
    """搜索高度必须是被**命令**的, 而不是"停在上一段碰巧留下的高度".

    landing_search_altitude / landing_approach_speed 此前零消费者, 注释里写的
    "保持在 5m" 只是对返航高度的假设.
    """
    fsm = _search_fsm(altitude=4.2, speed=0.3, position=(7.0, 8.0))

    hold_at_search_altitude(fsm)

    assert fsm.fc.waypoints == [(7.0, 8.0, -4.2, 0.3)]
    assert fsm.fc.loiter_calls == 0


@pytest.mark.parametrize("altitude", [0.0, -1.0, math.nan])
def test_search_hold_degrades_to_loiter_on_unusable_altitude(altitude):
    """高度参数不可用时退回"原地悬停" —— 与改动前行为一致, 不产生未知几何.

    (速度不可用是另一条退化路径: 仍飞向该高度, 但用默认接近速度, 见下条测试.)
    """
    fsm = _search_fsm(altitude=altitude, speed=0.5)

    hold_at_search_altitude(fsm)

    assert fsm.fc.waypoints == []
    assert fsm.fc.loiter_calls == 1


@pytest.mark.parametrize("speed", [0.0, -1.0, math.nan])
def test_search_hold_falls_back_to_default_speed(speed):
    """速度为 0/非法时用默认接近速度, 而不是发一个 0 速度的航点(等于悬停)."""
    fsm = _search_fsm(altitude=5.0, speed=speed)

    hold_at_search_altitude(fsm)

    assert fsm.fc.waypoints == [(1.0, 2.0, -5.0, 0.5)]
    assert fsm.fc.loiter_calls == 0
