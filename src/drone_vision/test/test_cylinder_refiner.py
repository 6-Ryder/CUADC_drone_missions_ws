import cv2
import numpy as np

from drone_vision.cylinder_detector import (
    CylinderRefiner,
    detect_white_cylinders,
)


def test_refiner_rejects_non_finite_ellipse(monkeypatch):
    contour = np.zeros((5, 1, 2), dtype=np.int32)
    monkeypatch.setattr(
        cv2,
        "findContours",
        lambda *_args, **_kwargs: ([contour], None),
    )
    monkeypatch.setattr(
        cv2,
        "fitEllipse",
        lambda _contour: ((float("inf"), 10.0), (20.0, 20.0), 0.0),
    )
    refiner = CylinderRefiner(blur_ksize=3)

    result = refiner.refine(
        np.full((40, 40, 3), 255, dtype=np.uint8),
        (5, 5, 35, 35),
    )

    assert result is None


def test_fallback_detects_white_cylinders_without_blue_marker():
    image = np.full((480, 640, 3), 50, dtype=np.uint8)
    cv2.circle(image, (220, 240), 140, (255, 0, 0), -1)
    cv2.circle(image, (210, 200), 48, (245, 245, 245), -1)
    cv2.circle(image, (310, 240), 32, (245, 245, 245), -1)

    detections = detect_white_cylinders(image)

    assert len(detections) == 2
    centers = sorted((round(det.cx), round(det.cy)) for det in detections)
    assert centers == [(210, 200), (310, 240)]
