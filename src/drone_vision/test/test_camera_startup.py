import pytest
import numpy as np
from builtin_interfaces.msg import Time

import drone_vision.camera as camera_module
from drone_vision.camera import CameraDriverNode


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _CameraThatCannotOpen:
    def get_logger(self):
        return _Logger()

    def _open_camera(self):
        raise RuntimeError("camera unavailable")


def test_real_camera_initialization_propagates_open_failure():
    camera = _CameraThatCannotOpen()

    with pytest.raises(RuntimeError, match="camera unavailable"):
        CameraDriverNode._init_real(camera)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _UsbCamera:
    values = {
        "camera_device": "/dev/v4l/by-id/task-camera",
        "camera_width": 640,
        "camera_height": 480,
        "camera_fps": 30.0,
    }

    def get_parameter(self, name):
        return _Parameter(self.values[name])

    def get_logger(self):
        return _Logger()


def test_usb_camera_opens_the_configured_device_with_v4l2(monkeypatch):
    calls = []
    settings = []

    class Capture:
        def __init__(self, source, backend):
            calls.append((source, backend))

        def isOpened(self):
            return calls[-1] == (
                "/dev/v4l/by-id/task-camera",
                camera_module.cv2.CAP_V4L2,
            )

        def set(self, prop, value):
            settings.append((prop, value))
            return True

    monkeypatch.setattr(camera_module.cv2, "VideoCapture", Capture)
    camera = _UsbCamera()

    CameraDriverNode._open_camera(camera)

    assert calls == [
        ("/dev/v4l/by-id/task-camera", camera_module.cv2.CAP_V4L2),
    ]
    assert (camera_module.cv2.CAP_PROP_FRAME_WIDTH, 640) in settings
    assert (camera_module.cv2.CAP_PROP_FRAME_HEIGHT, 480) in settings
    assert (camera_module.cv2.CAP_PROP_FPS, 30.0) in settings


def test_capture_loop_reads_one_frame_per_publish_cycle(monkeypatch):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    class Capture:
        read_count = 0
        grab_count = 0

        def read(self):
            self.read_count += 1
            return True, frame

        def grab(self):
            self.grab_count += 1
            return True

        def retrieve(self):
            return True, frame

    class Publisher:
        messages = []

        def publish(self, message):
            self.messages.append(message)

    class Clock:
        class Now:
            @staticmethod
            def to_msg():
                return Time()

        @staticmethod
        def now():
            return Clock.Now()

    class Camera:
        _running = True
        _cap = Capture()
        _img_pub = Publisher()

        def get_parameter(self, _name):
            return _Parameter(1000.0)

        def get_clock(self):
            return Clock()

        def get_logger(self):
            return _Logger()

    states = iter((True, False))
    monkeypatch.setattr(camera_module.rclpy, "ok", lambda: next(states))
    monkeypatch.setattr(camera_module.time, "sleep", lambda _seconds: None)

    camera = Camera()
    CameraDriverNode._capture_loop(camera)

    assert camera._cap.read_count == 1
    assert camera._cap.grab_count == 0
    assert len(camera._img_pub.messages) == 1
