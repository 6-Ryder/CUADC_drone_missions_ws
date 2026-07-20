import cv2
import numpy as np

from drone_vision.landing_detector import LandingPadRefiner


def _landing_scene(with_h=True):
    image = np.full((480, 640, 3), 180, dtype=np.uint8)
    cv2.circle(image, (320, 240), 80, (55, 55, 55), -1)
    cv2.circle(image, (320, 240), 80, (245, 245, 245), 8)
    if with_h:
        cv2.rectangle(image, (285, 200), (297, 280), (245, 245, 245), -1)
        cv2.rectangle(image, (343, 200), (355, 280), (245, 245, 245), -1)
        cv2.rectangle(image, (292, 234), (348, 246), (245, 245, 245), -1)
    return image


def test_global_detector_finds_h_marker_center():
    result = LandingPadRefiner(circle_min_area=80).detect(_landing_scene())

    assert result is not None
    cx, cy, confidence, _yaw = result
    assert abs(cx - 320) < 4
    assert abs(cy - 240) < 4
    assert confidence > 0.5


def test_global_detector_rejects_plain_circle():
    assert LandingPadRefiner(circle_min_area=80).detect(
        _landing_scene(with_h=False),
    ) is None


def test_global_detector_finds_marker_at_five_meter_image_scale():
    image = np.full((480, 640, 3), 180, dtype=np.uint8)
    cv2.circle(image, (320, 240), 32, (55, 55, 55), -1)
    cv2.circle(image, (320, 240), 32, (245, 245, 245), 3)
    cv2.rectangle(image, (309, 226), (313, 254), (245, 245, 245), -1)
    cv2.rectangle(image, (327, 226), (331, 254), (245, 245, 245), -1)
    cv2.rectangle(image, (312, 238), (328, 242), (245, 245, 245), -1)

    result = LandingPadRefiner(circle_min_area=80).detect(image)

    assert result is not None
    assert abs(result[0] - 320) < 3
    assert abs(result[1] - 240) < 3
