"""视觉测量时刻的契约.

控制的积分/微分必须按"真实测量间隔"推进, 而不是按 FSM tick 周期 ——
真机视觉 3-5Hz 而 tick 20Hz, 按 tick 推进会把同一份误差重复累加数倍,
微分也会被放大 (本项目曾因此在标定前把微分项归零).

本测试钉住测量时刻的可获得性与语义, 使消费方可以依赖它.
"""

import sys

import pytest


def _fsm_module():
    """导入 mission_state_machine, 但绕开 rclpy 依赖."""
    pytest.importorskip("rclpy")
    from drone_mission.mission_state_machine import MissionFSM  # noqa: F401
    return sys.modules["drone_mission.mission_state_machine"]


def test_measurement_stamp_is_zero_before_any_message():
    """未收到任何测量时返回 0.0 —— 消费方据此判定"无可用测量"."""
    mod = _fsm_module()
    fsm = object.__new__(mod.MissionFSM)
    fsm._cylinders_timestamp = 0.0
    fsm._markers_timestamp = 0.0
    fsm._landing_target_stamp = 0.0

    assert fsm.vision_measurement_stamp("cylinder") == 0.0
    assert fsm.vision_measurement_stamp("marker") == 0.0
    assert fsm.vision_measurement_stamp("landing") == 0.0


def test_measurement_stamp_is_per_source():
    """三个视觉源各自独立, 不得互相污染."""
    mod = _fsm_module()
    fsm = object.__new__(mod.MissionFSM)
    fsm._cylinders_timestamp = 11.0
    fsm._markers_timestamp = 22.0
    fsm._landing_target_stamp = 33.0

    assert fsm.vision_measurement_stamp("cylinder") == 11.0
    assert fsm.vision_measurement_stamp("marker") == 22.0
    assert fsm.vision_measurement_stamp("landing") == 33.0


def test_unknown_source_returns_zero_not_raises():
    """未知源返回 0.0 而非抛异常 —— 失败方向必须是"当作无测量",
    不能因为拼错源名而让控制继续用陈旧测量."""
    mod = _fsm_module()
    fsm = object.__new__(mod.MissionFSM)
    fsm._cylinders_timestamp = 11.0
    fsm._markers_timestamp = 22.0
    fsm._landing_target_stamp = 33.0

    assert fsm.vision_measurement_stamp("typo") == 0.0
    assert fsm.vision_measurement_stamp("") == 0.0
