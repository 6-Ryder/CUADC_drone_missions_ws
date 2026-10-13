"""landing_estimator 的契约.

权威结构 (PX4 / ArduPilot) 的四条性质必须成立, 否则这不是"复用权威算法"而是
另一个自研滤波器:

  1. 静止目标: 位置收敛, **速度状态收敛到 0** (速度来自滤波器, 不是求差).
  2. 离群门: 单帧远跳被拒且不改状态; 连续 3 次拒绝后强制融合 (否则饿死).
  3. 掉线期间 P 增长 → 重捕获瞬间门变松 (与固定像素阈值相反).
  4. 世界位置建模: 载机自己怎么动/怎么转, 都不改变对静止目标的估计.
"""

import math

import pytest

from drone_utils.landing_estimator import (
    CHI2_GATE_95,
    FORCE_FUSE_AFTER_REJECTS,
    LandingTargetEstimator,
    TargetAxisFilter,
)


# ---------------------------------------------------------------------------
# 单轴滤波器
# ---------------------------------------------------------------------------

def test_predict_propagates_state_and_grows_covariance():
    f = TargetAxisFilter()
    f.init(position=0.0, velocity=1.0, pos_unc=0.1, vel_unc=0.1)
    before = f.covariance

    f.predict(dt=1.0, acc=0.0, acc_unc=0.5)

    assert f.position == pytest.approx(1.0)      # p += v·dt
    assert f.velocity == pytest.approx(1.0)
    assert f.covariance[0] > before[0]           # P00 增长 (门会变松)


def test_predict_ignores_non_positive_dt():
    f = TargetAxisFilter()
    f.init(0.0, 1.0, 0.1, 0.1)

    f.predict(dt=0.0, acc=0.0, acc_unc=1.0)
    f.predict(dt=-1.0, acc=0.0, acc_unc=1.0)

    assert f.position == pytest.approx(0.0)


def test_update_moves_state_toward_measurement():
    f = TargetAxisFilter()
    f.init(0.0, 0.0, 1.0, 1.0)

    assert f.update(1.0, 0.01) is True
    assert 0.0 < f.position < 1.0


def test_single_outlier_is_rejected_and_state_unchanged():
    """单帧远跳必须被拒且**不改状态** (predict 照常跑是调用方的事)."""
    f = TargetAxisFilter()
    f.init(0.0, 0.0, 0.01, 0.01)
    for _ in range(20):
        f.update(0.0, 0.01)
    p_before = f.position
    v_before = f.velocity

    assert f.update(100.0, 0.01) is False
    assert f.position == pytest.approx(p_before)
    assert f.velocity == pytest.approx(v_before)
    assert f.reject_streak == 1


def test_force_fuse_after_consecutive_rejections():
    """连续拒绝到上限后强制融合 —— 否则目标真的移动时滤波器会被自己的门饿死
    (ArduPilot AC_PrecLand.cpp:574-583 的做法)."""
    f = TargetAxisFilter()
    f.init(0.0, 0.0, 0.01, 0.01)
    for _ in range(20):
        f.update(0.0, 0.01)

    rejects = [f.update(50.0, 0.01) for _ in range(FORCE_FUSE_AFTER_REJECTS)]
    assert rejects == [False] * FORCE_FUSE_AFTER_REJECTS

    assert f.update(50.0, 0.01) is True          # 第 4 次被强制接受
    assert f.reject_streak == 0
    assert f.position > 0.0


def test_gate_loosens_after_dropout():
    """掉线期间只 predict → P00 增长 → 重捕获时同一个残差不再是离群.

    这是权威门与"固定像素跳变阈值"最关键的区别.
    """
    f = TargetAxisFilter()
    f.init(0.0, 0.0, 0.01, 0.01)
    for _ in range(20):
        f.update(0.0, 0.01)

    assert f.update(0.5, 0.01) is False          # 收敛后 0.5m 的跳变被拒

    for _ in range(50):                          # 丢线 5s
        f.predict(dt=0.1, acc=0.0, acc_unc=1.0)

    assert f.update(0.5, 0.01) is True           # 重捕获被接受


def test_beta_matches_reported_innovation():
    """上报的 (残差, 新息协方差) 必须能复算出被判定的 beta —— 供日志复盘门限."""
    f = TargetAxisFilter()
    f.init(0.0, 0.0, 0.01, 0.01)
    f.update(0.05, 0.01)

    residual, innov_cov = f.last_innovation
    assert innov_cov > 0.0
    assert residual == pytest.approx(0.05 - 0.0, abs=1e-9)


@pytest.mark.parametrize("meas,unc", [(math.nan, 0.01), (math.inf, 0.01)])
def test_non_finite_measurement_is_rejected(meas, unc):
    f = TargetAxisFilter()
    f.init(0.0, 0.0, 0.01, 0.01)

    assert f.update(meas, unc) is False


def test_uninitialized_filter_rejects_updates():
    f = TargetAxisFilter()
    assert f.update(1.0, 0.01) is False
    assert f.initialized is False


# ---------------------------------------------------------------------------
# 双轴 / 世界位置建模
# ---------------------------------------------------------------------------

def _observe_target(est, target_ned, vehicle_ned, yaw, meas_unc=0.01):
    """从给定载机位置/航向观测一个已知世界目标, 返回机体系偏移观测."""
    dn = target_ned[0] - vehicle_ned[0]
    de = target_ned[1] - vehicle_ned[1]
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    # NED → 机体系 (set_velocity_body 的逆)
    error_forward = dn * cos_yaw + de * sin_yaw
    error_right = -dn * sin_yaw + de * cos_yaw
    return est.observe(vehicle_ned[0], vehicle_ned[1], yaw,
                       error_forward, error_right, meas_unc)


def test_estimator_converges_to_static_target_world_position():
    est = LandingTargetEstimator()
    target = (10.0, -4.0)

    for _ in range(30):
        _observe_target(est, target, (0.0, 0.0), 0.0)

    assert est.position == pytest.approx(target, abs=0.02)


def test_vehicle_motion_and_yaw_do_not_move_the_estimate():
    """核心性质: 估计的是**世界**位置, 载机自己怎么飞/怎么转都不改变它.

    若误把机体系偏移当被估计量, 航向一转目标就会跟着漂 —— 本项目"固定方向
    偏置"类问题的来源.
    """
    est = LandingTargetEstimator()
    target = (6.0, 3.0)

    # 载机沿一条弧线飞、航向一直在变, 但始终看着同一个静止目标
    for i in range(40):
        yaw = i * 0.15
        vehicle = (5.0 + 0.05 * i, 2.0 - 0.04 * i)
        _observe_target(est, target, vehicle, yaw)

    assert est.position == pytest.approx(target, abs=0.05)
    # 目标静止 ⇒ 速度状态应趋于 0
    assert abs(est.velocity[0]) < 0.05
    assert abs(est.velocity[1]) < 0.05


def test_first_observation_seeds_the_estimate():
    est = LandingTargetEstimator()
    assert est.initialized is False
    assert est.position is None

    assert _observe_target(est, (2.0, 1.0), (0.0, 0.0), 0.0) is True

    assert est.initialized is True
    assert est.position == pytest.approx((2.0, 1.0))


def test_outlier_frame_is_rejected_end_to_end():
    """一帧误检 (目标"跳"到很远处) 不得污染估计."""
    est = LandingTargetEstimator()
    for _ in range(20):
        _observe_target(est, (1.0, 1.0), (0.0, 0.0), 0.0)
    settled = est.position

    assert _observe_target(est, (60.0, -60.0), (0.0, 0.0), 0.0) is False
    assert est.position == pytest.approx(settled)


def test_reset_clears_estimate():
    est = LandingTargetEstimator()
    _observe_target(est, (3.0, 3.0), (0.0, 0.0), 0.0)
    assert est.initialized is True

    est.reset()

    assert est.initialized is False
    assert est.position is None


@pytest.mark.parametrize("bad", [math.nan, math.inf])
def test_non_finite_observation_is_refused(bad):
    est = LandingTargetEstimator()
    _observe_target(est, (1.0, 1.0), (0.0, 0.0), 0.0)

    assert est.observe(bad, 0.0, 0.0, 0.0, 0.0, 0.01) is False
    assert est.observe(0.0, 0.0, bad, 0.0, 0.0, 0.01) is False


def test_gate_threshold_is_the_authoritative_value():
    """门限取 PX4 的 3.84 (卡方 1 DOF, 5% 误报); 改它必须是有意的."""
    assert CHI2_GATE_95 == pytest.approx(3.84)
