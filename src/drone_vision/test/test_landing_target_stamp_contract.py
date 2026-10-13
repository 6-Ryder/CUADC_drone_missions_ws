"""LandingTarget 采集时刻的跨包契约.

与 test_cylinder_world_contract.py 同形: 钉住"消息 header.stamp 来自相机帧
时刻 (_frame_stamp), 而不是本消息的构造/发布时刻".

为什么重要: FSM 侧用这个值当控制的积分/微分步长 (ServoPid.update(stamp=...)),
若发布端改回 now(), 步长会悄悄带上推理耗时抖动, 而消费方无从察觉 —— 这正是
"有测量时刻"与"测量时刻被用于控制"之间最容易断的一环.
"""

from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from drone_vision.vision_pipeline import VisionPipelineNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Clock:
    class _Now:
        sec = 999
        nanosec = 0

        def to_msg(self):
            return Time(sec=999, nanosec=0)

    def now(self):
        return self._Now()


def bare_landing_pipeline():
    pipeline = object.__new__(VisionPipelineNode)
    pipeline._landing_pub = _Publisher()
    pipeline._circle_tracking_active = lambda: False
    pipeline._coord_xform = None
    pipeline._servo_mode = "none"
    pipeline._landing_servo = None
    pipeline._landing_tracker_diagnostic = None
    pipeline._frame_stamp = Time(sec=12, nanosec=345)
    pipeline.get_clock = lambda: _Clock()
    return pipeline


def test_miss_frame_carries_frame_stamp_and_zero_confidence():
    """无检测帧同样带采集时刻, 但 confidence 恰为 0.0.

    发布端对 miss 帧也发消息 (存活判断依赖帧流), 因此 stamp 前进只说明
    "相机还在出帧", 不说明"有新测量" —— 消费方必须用 confidence 区分.
    """
    pipeline = bare_landing_pipeline()
    pipeline._select_landing_legacy_target = lambda image, dets: (None, "none")

    pipeline._process_landing_v2(np.zeros((2, 2, 3)), [])

    target = pipeline._landing_pub.messages[-1]
    assert target.header.stamp.sec == 12
    assert target.header.stamp.nanosec == 345
    assert target.header.frame_id == "world"
    assert target.confidence == 0.0


def test_detection_frame_carries_frame_stamp_not_publish_time():
    """有检测时 stamp 仍是相机帧时刻 —— 不是构造消息的当前时刻 (999s)."""
    pipeline = bare_landing_pipeline()
    pipeline._select_landing_legacy_target = lambda image, dets: (
        (321.0, 239.0, 0.9, 0.0), "yolo"
    )

    pipeline._process_landing_v2(np.zeros((2, 2, 3)), [])

    target = pipeline._landing_pub.messages[-1]
    assert target.header.stamp.sec == 12
    assert target.header.stamp.nanosec == 345
    assert target.confidence == pytest.approx(0.9)


def test_missing_frame_stamp_falls_back_to_publish_time():
    """未经 _image_cb 直接调用时 _frame_stamp 为空, 退回发布时刻而不是崩.

    这是降级: stamp 变成发布时刻, 与"无采集时刻"的旧行为一致 —— 消费方
    仍有可用时间戳 (不会退到 0), 但不再是真实帧时刻.
    """
    pipeline = bare_landing_pipeline()
    pipeline._frame_stamp = None
    pipeline._select_landing_legacy_target = lambda image, dets: (None, "none")

    pipeline._process_landing_v2(np.zeros((2, 2, 3)), [])

    target = pipeline._landing_pub.messages[-1]
    assert target.header.stamp.sec == 999


def test_landing_message_declares_header_first_for_ros_convention():
    """header 必须是 .msg 的第一个字段 (ROS 消息约定, 与同目录另三个一致)."""
    from pathlib import Path
    root = Path(__file__).resolve().parents[3]
    lines = [
        line.split("#", 1)[0].strip()
        for line in (root / "src/drone_interfaces/msg/LandingTarget.msg")
        .read_text(encoding="utf-8").splitlines()
    ]
    fields = [line for line in lines if line]
    assert fields[0] == "std_msgs/Header header"


def test_landing_target_exposes_expected_fields():
    """字段集合契约: 增删字段时本测试会失败, 提醒同步 FSM 的 dict 映射."""
    from drone_interfaces.msg import LandingTarget
    names = set(LandingTarget.get_fields_and_field_types().keys())
    assert names == {
        "header", "center_u", "center_v", "world_x", "world_y",
        "yaw_correction", "confidence",
    }
    assert SimpleNamespace  # keep import used for symmetry with sibling tests
