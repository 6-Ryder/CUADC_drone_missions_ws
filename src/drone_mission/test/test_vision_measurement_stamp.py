"""视觉测量时刻的契约.

控制的积分/微分必须按"真实测量间隔"推进, 而不是按 FSM tick 周期 ——
真机视觉 3-5Hz 而 tick 20Hz, 按 tick 推进会把同一份误差重复累加数倍,
微分也会被放大 (本项目曾因此在标定前把微分项归零).

本测试钉住测量时刻的可获得性与**语义**: 返回的必须是采集时刻 (消息 header
的相机帧时刻), 而不是到达时刻. 与权威一致 —— MAVLink LANDING_TARGET.time_usec
与 PX4 FiducialMarkerPosReport.timestamp_sample 都是"这次测量是什么时候拍的".
按到达时刻求差会把每帧推理耗时的抖动带进积分/微分步长.
"""

import sys

import pytest


def _fsm_module():
    """导入 mission_state_machine, 但绕开 rclpy 依赖."""
    pytest.importorskip("rclpy")
    from drone_mission.mission_state_machine import MissionFSM  # noqa: F401
    return sys.modules["drone_mission.mission_state_machine"]


def _bare_fsm(mod, **stamps):
    """构造只带时间戳字段的 FSM 空壳 (与回调写入的字段集合一致)."""
    fsm = object.__new__(mod.MissionFSM)
    fsm._cylinder_capture_stamp = 0.0
    fsm._marker_capture_stamp = 0.0
    fsm._cylinders_timestamp = 0.0
    fsm._markers_timestamp = 0.0
    fsm._landing_target_stamp = 0.0
    fsm._landing_target_capture_stamp = 0.0
    for key, value in stamps.items():
        setattr(fsm, key, value)
    return fsm


def test_measurement_stamp_is_zero_before_any_message():
    """未收到任何测量时返回 0.0 —— 消费方据此判定"无可用测量"."""
    fsm = _bare_fsm(_fsm_module())

    assert fsm.vision_measurement_stamp("cylinder") == 0.0
    assert fsm.vision_measurement_stamp("marker") == 0.0
    assert fsm.vision_measurement_stamp("landing") == 0.0


def test_measurement_stamp_is_per_source():
    """三个视觉源各自独立, 不得互相污染."""
    fsm = _bare_fsm(
        _fsm_module(),
        _cylinder_capture_stamp=11.0,
        _marker_capture_stamp=22.0,
        _landing_target_capture_stamp=33.0,
    )

    assert fsm.vision_measurement_stamp("cylinder") == 11.0
    assert fsm.vision_measurement_stamp("marker") == 22.0
    assert fsm.vision_measurement_stamp("landing") == 33.0


def test_capture_stamp_wins_over_arrival_stamp():
    """有采集时刻时必须返回采集时刻.

    这是本契约的核心: 回归到"返回到达时刻"会让积分/微分步长重新带上
    每帧推理耗时的抖动, 而真机 3-5Hz 下该抖动与测量间隔同量级.
    """
    fsm = _bare_fsm(
        _fsm_module(),
        _cylinder_capture_stamp=100.0,
        _cylinders_timestamp=100.4,   # 到达晚 0.4s (推理耗时)
        _marker_capture_stamp=200.0,
        _markers_timestamp=200.3,
        _landing_target_capture_stamp=300.0,
        _landing_target_stamp=300.2,
    )

    assert fsm.vision_measurement_stamp("cylinder") == 100.0
    assert fsm.vision_measurement_stamp("marker") == 200.0
    assert fsm.vision_measurement_stamp("landing") == 300.0


def test_missing_header_falls_back_to_arrival_stamp():
    """header 缺失时 _capture_seconds 返回 0.0, 此时退回到达时刻.

    降级而非静默失效: 0.0 会让消费方判定"无测量"并停发指令, 曲线比
    "用一个错误的步长继续积分"更安全, 但到达时刻仍优于完全不可用.
    """
    fsm = _bare_fsm(
        _fsm_module(),
        _cylinder_capture_stamp=0.0,
        _cylinders_timestamp=55.0,
        _marker_capture_stamp=0.0,
        _markers_timestamp=66.0,
    )

    assert fsm.vision_measurement_stamp("cylinder") == 55.0
    assert fsm.vision_measurement_stamp("marker") == 66.0


def test_landing_source_falls_back_to_arrival_without_header():
    """LandingTarget 的 header 为空 (旧发布端/未填时间戳) 时退回到达时刻.

    这是降级路径: 步长会重新带上推理耗时抖动, 但比"无测量"可用. 判据是
    _capture_seconds 返回 0.0 —— 非有限或非正值一律视为"没有采集时刻".
    """
    fsm = _bare_fsm(
        _fsm_module(),
        _landing_target_capture_stamp=0.0,
        _landing_target_stamp=33.0,
    )

    assert fsm.vision_measurement_stamp("landing") == 33.0


def test_unknown_source_returns_zero_not_raises():
    """未知源返回 0.0 而非抛异常 —— 失败方向必须是"当作无测量",
    不能因为拼错源名而让控制继续用陈旧测量."""
    fsm = _bare_fsm(
        _fsm_module(),
        _cylinder_capture_stamp=11.0,
        _marker_capture_stamp=22.0,
        _landing_target_stamp=33.0,
    )

    assert fsm.vision_measurement_stamp("typo") == 0.0
    assert fsm.vision_measurement_stamp("") == 0.0
