"""H 降落目标的龄期契约.

背景 (已记录的真机失效路径): 视觉节点对"本帧没看到 H 标识"也发布消息
(confidence 恰为 0.0), 因此"收到消息"不等于"拿到测量". 只按消息到达刷新
时间戳的话, 相机或推理链路死亡时旧目标永不失效 —— FSM 会一直拿最后一帧
驱动对心与下降, 而 watchdog 因帧流仍在而被喂活, 没有任何告警.

权威口径同此: PX4 precland 按 landing_target_pose 的时间戳判龄 (PLD_BTOUT),
其估计器的 _last_update 只在成功融合/发布时刷新, 所以"一串持续被门拒绝的
测量"同样会走到复位 (LandingTargetEstimator.cpp:76-80,120,180;
precland.cpp:127-135).

本测试钉住两点:
  1. 判据是"距最后一次含检测的消息", 不是"距最后一条消息".
  2. miss 帧不延长寿命.
"""

import sys
import time

import pytest


def _fsm_module():
    """导入 mission_state_machine, 但绕开 rclpy 依赖."""
    pytest.importorskip("rclpy")
    from drone_mission.mission_state_machine import MissionFSM  # noqa: F401
    return sys.modules["drone_mission.mission_state_machine"]


class _Watchdog:
    def __init__(self):
        self.feeds = []

    def feed(self, name):
        self.feeds.append(name)


class _Stamp:
    def __init__(self, seconds):
        self.sec = int(seconds)
        self.nanosec = int((seconds - int(seconds)) * 1e9)


class _Header:
    def __init__(self, seconds):
        self.stamp = _Stamp(seconds)


class _Msg:
    """LandingTarget 的最小替身."""

    def __init__(self, confidence, capture_stamp=0.0):
        self.header = _Header(capture_stamp)
        self.center_u = 320.0
        self.center_v = 240.0
        self.world_x = 1.0
        self.world_y = 2.0
        self.yaw_correction = 0.0
        self.confidence = confidence


def _bare_fsm(mod):
    fsm = object.__new__(mod.MissionFSM)
    fsm._landing_target = None
    fsm._landing_target_stamp = 0.0
    fsm._landing_target_capture_stamp = 0.0
    fsm._landing_target_detection_stamp = 0.0
    fsm._watchdog = _Watchdog()
    return fsm


def test_target_expires_when_no_detection_for_2s():
    """超过 2s 没有一次真实检测 → 视为无目标 (与另三路视觉源同形的龄期门)."""
    fsm = _bare_fsm(_fsm_module())
    fsm._landing_target = {"center_u": 320.0, "confidence": 0.9}
    fsm._landing_target_detection_stamp = time.time() - 2.5

    assert fsm.get_landing_target() is None


def test_target_available_within_2s_of_a_detection():
    fsm = _bare_fsm(_fsm_module())
    fsm._landing_target = {"center_u": 320.0, "confidence": 0.9}
    fsm._landing_target_detection_stamp = time.time() - 0.5

    assert fsm.get_landing_target() is not None


def test_miss_frames_do_not_keep_target_alive():
    """关键的失败方向: 视觉节点还活着并持续发 miss 帧, 但检测已经死了.

    此时 _landing_target_stamp 一直在刷新 (所以"源还在不在"是真), 而
    _landing_target_detection_stamp 停在最后一次真实检测 —— 目标必须失效.
    """
    fsm = _bare_fsm(_fsm_module())
    fsm._landing_target = {"center_u": 320.0, "confidence": 0.0}
    fsm._landing_target_stamp = time.time()          # 刚收到一条 miss 帧
    fsm._landing_target_detection_stamp = time.time() - 3.0  # 3s 前才真有检测

    assert fsm.get_landing_target() is None


def test_callback_advances_arrival_stamp_on_every_message():
    """每条消息都刷新到达时刻 —— watchdog 与"源存活"判断依赖它."""
    fsm = _bare_fsm(_fsm_module())
    before = time.time()

    fsm._landing_target_cb(_Msg(confidence=0.0))

    assert fsm._landing_target_stamp >= before
    assert fsm._landing_target_detection_stamp == 0.0


def test_callback_advances_detection_stamp_only_on_real_detection():
    """只有 confidence > 0 的消息才推进"最后一次检测"时刻.

    confidence 恰为 0.0 是视觉节点对"本帧无目标"的约定
    (vision_pipeline._process_landing_v2 三个发布分支), 所以这里比的是
    "有没有检测", 与消费侧的置信度门槛 (0.60) 是两件事.
    """
    fsm = _bare_fsm(_fsm_module())

    fsm._landing_target_cb(_Msg(confidence=0.42))
    detected_at = fsm._landing_target_detection_stamp
    assert detected_at > 0.0

    fsm._landing_target_cb(_Msg(confidence=0.0))
    assert fsm._landing_target_detection_stamp == detected_at


def test_callback_records_capture_stamp_from_header():
    """采集时刻 (header.stamp) 与到达时刻分开记录.

    控制用采集时刻 (真实步长), 存活判断用到达时刻 —— 二者不能合并, 否则
    要么步长带上推理耗时抖动, 要么回到"帧流即存活"的误判.
    """
    fsm = _bare_fsm(_fsm_module())

    fsm._landing_target_cb(_Msg(confidence=0.9, capture_stamp=123.5))

    assert fsm._landing_target_capture_stamp == pytest.approx(123.5)
    assert fsm._landing_target_stamp != pytest.approx(123.5)


def test_callback_without_header_keeps_previous_capture_stamp():
    """header 缺失 (空 stamp, 解析为 0.0) 不得把采集时刻抹成 0.

    抹成 0 会让 vision_measurement_stamp("landing") 突然退回到达时刻,
    相当于在控制中途悄悄换时间基准 —— 宁可保留上一帧的采集时刻.
    """
    fsm = _bare_fsm(_fsm_module())

    fsm._landing_target_cb(_Msg(confidence=0.9, capture_stamp=123.5))
    fsm._landing_target_cb(_Msg(confidence=0.9, capture_stamp=0.0))

    assert fsm._landing_target_capture_stamp == pytest.approx(123.5)
