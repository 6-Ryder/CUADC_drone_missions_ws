"""PoseHistory 的契约 —— 时间对齐的底座.

要点: 取不到就必须返回 None. 这个模块存在的意义是"别拿错时刻的位姿去解析旧帧",
所以它失败的方向只能是"拿不到"(让调用方退回原行为), 绝不能是"随便给一个近似的".
"""

import math

import pytest

from drone_utils.pose_history import PoseHistory


def _history_20hz(seconds=1.0, north_of=lambda t: t, yaw_of=lambda t: 0.0):
    h = PoseHistory()
    steps = int(seconds * 20)
    for k in range(steps + 1):
        t = k / 20.0
        h.add(t, north_of(t), 0.0, -2.0, yaw_of(t))
    return h


def test_exact_sample_is_returned():
    h = _history_20hz()
    pose = h.at(0.5)
    assert pose is not None
    assert pose[0] == pytest.approx(0.5)
    assert pose[3] == pytest.approx(0.0)


def test_interpolates_between_samples():
    """两个采样点之间线性插值 —— 20Hz 采样对 0.2-0.33s 的观测龄够用."""
    h = _history_20hz()
    pose = h.at(0.525)                      # 0.50 与 0.55 之间
    assert pose is not None
    assert pose[0] == pytest.approx(0.525, abs=1e-6)


def test_yaw_interpolates_on_shortest_arc():
    """±π 附近必须走最短弧, 不能绕远路 —— 否则航向会插出一个大跳变."""
    h = PoseHistory()
    h.add(0.0, 0.0, 0.0, 0.0, math.radians(179.0))
    h.add(0.1, 0.0, 0.0, 0.0, math.radians(-179.0))

    pose = h.at(0.05)

    assert pose is not None
    # 最短弧经过 ±180 附近, 而不是经过 0
    assert abs(abs(pose[3]) - math.pi) < math.radians(2.0)


def test_returns_none_when_no_sample_within_max_gap():
    """只在两段之间留一个空洞, 空洞正中就取不到 —— 宁可"拿不到"."""
    h = PoseHistory(max_gap=0.05)
    h.add(0.0, 0.0, 0.0, 0.0, 0.0)
    h.add(1.0, 1.0, 0.0, 0.0, 0.0)

    assert h.at(0.5) is None


def test_returns_none_for_stamp_outside_history():
    h = _history_20hz(seconds=0.5)
    assert h.at(9.0) is None            # 远在未来
    assert h.at(-1.0) is None           # 非法


def test_returns_none_for_invalid_stamp():
    h = _history_20hz()
    assert h.at(math.nan) is None
    assert h.at(0.0) is None


def test_returns_none_when_empty():
    assert PoseHistory().at(1.0) is None


def test_rejects_non_monotonic_and_non_finite_samples():
    """乱序样本会破坏插值区间, 必须丢弃; 非有限值同样."""
    h = PoseHistory()
    h.add(1.0, 1.0, 0.0, 0.0, 0.0)
    h.add(0.5, 9.0, 0.0, 0.0, 0.0)      # 倒退 → 丢
    h.add(2.0, math.nan, 0.0, 0.0, 0.0)  # 非有限 → 丢
    h.add(2.0, 2.0, 0.0, 0.0, 0.0)

    assert len(h) == 2
    assert h.at(1.0)[0] == pytest.approx(1.0)
    assert h.at(2.0)[0] == pytest.approx(2.0)


def test_ring_buffer_keeps_bounded_history():
    h = PoseHistory(maxlen=10)
    for k in range(50):
        h.add(k * 0.05 + 1.0, float(k), 0.0, 0.0, 0.0)
    assert len(h) == 10


def test_reset_clears():
    h = _history_20hz()
    h.reset()
    assert len(h) == 0
    assert h.at(0.5) is None
