from types import SimpleNamespace

import numpy as np

from scripts.vision_viewer import VisionViewer, configure_window


def test_rgb8_image_is_converted_to_bgr_for_opencv():
    viewer = object.__new__(VisionViewer)
    message = SimpleNamespace(
        height=1,
        width=1,
        step=3,
        encoding="rgb8",
        data=bytes((10, 20, 30)),
    )

    VisionViewer._on_image(viewer, message)

    np.testing.assert_array_equal(
        viewer._image,
        np.array([[[30, 20, 10]]], dtype=np.uint8),
    )


def test_bgr8_image_respects_row_step_padding():
    viewer = object.__new__(VisionViewer)
    message = SimpleNamespace(
        height=1,
        width=1,
        step=4,
        encoding="bgr8",
        data=bytes((10, 20, 30, 255)),
    )

    VisionViewer._on_image(viewer, message)

    np.testing.assert_array_equal(
        viewer._image,
        np.array([[[10, 20, 30]]], dtype=np.uint8),
    )


def test_viewer_window_uses_explicit_working_dimensions(monkeypatch):
    calls = []
    monkeypatch.setattr(
        "scripts.vision_viewer.cv2.namedWindow",
        lambda name, flags: calls.append(("named", name, flags)),
    )
    monkeypatch.setattr(
        "scripts.vision_viewer.cv2.resizeWindow",
        lambda name, width, height: calls.append(
            ("resize", name, width, height)
        ),
    )

    configure_window()

    assert calls[1][0] == "resize"
    assert calls[1][2:] == (640, 480)
