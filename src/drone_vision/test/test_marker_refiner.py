import cv2
import numpy as np

from drone_vision.marker_detector import BlueMarkerRefiner
from drone_vision.models.yolo_engine import Detection


def _scene():
    image = np.full((480, 640, 3), 60, dtype=np.uint8)
    cv2.circle(image, (220, 240), 140, (255, 0, 0), -1)
    cv2.circle(image, (220, 190), 48, (245, 245, 245), -1)
    cv2.circle(image, (285, 230), 34, (245, 245, 245), -1)
    return image


def _detection(bbox, confidence):
    x1, y1, x2, y2 = bbox
    return Detection(
        class_id=0,
        class_name="drop_zone_marker",
        confidence=confidence,
        bbox=bbox,
        cx=(x1 + x2) / 2,
        cy=(y1 + y2) / 2,
    )


def test_refine_restores_full_blue_marker_and_rejects_white_objects():
    refiner = BlueMarkerRefiner(min_detection_blue_ratio=0.4)
    detections = [
        _detection((80, 100, 225, 380), 0.95),
        _detection((250, 195, 320, 265), 0.85),
        _detection((450, 50, 550, 150), 0.80),
    ]

    refined = refiner.refine(_scene(), detections)

    assert len(refined) == 1
    np.testing.assert_allclose(
        refined[0].bbox, (80, 100, 361, 381), atol=2,
    )
    assert abs(refined[0].cx - 220) < 8
    assert abs(refined[0].cy - 240) < 8
    assert refined[0].confidence == 0.95


def test_fallback_detects_the_same_complete_blue_component():
    detections = BlueMarkerRefiner().detect(_scene())

    assert len(detections) == 1
    np.testing.assert_allclose(
        detections[0].bbox, (80, 100, 361, 381), atol=2,
    )
    assert detections[0].class_name == "drop_zone_marker"


def test_fallback_rejects_tiny_blue_clutter():
    image = np.full((480, 640, 3), 60, dtype=np.uint8)
    cv2.circle(image, (320, 240), 10, (255, 0, 0), -1)

    assert BlueMarkerRefiner().detect(image) == []


def test_refine_with_no_detections_skips_entire_scene_split():
    """空检测直接返回, 不再对整图做分割 (低空 FPS 优化核心路径)."""
    refiner = BlueMarkerRefiner()
    assert refiner.refine(_scene(), []) == []


def test_refine_roi_offset_recovers_component_near_image_corner():
    """ROI 裁剪后组件坐标必须偏移回原图像素 (角落场景最容易丢偏移)."""
    image = np.full((480, 640, 3), 60, dtype=np.uint8)
    cv2.circle(image, (580, 420), 50, (255, 0, 0), -1)  # 右下角完整蓝色圆

    refiner = BlueMarkerRefiner()
    refined = refiner.refine(
        image,
        [_detection((550, 390, 610, 430), 0.9)],  # 部分框住圆的框
    )

    assert len(refined) == 1
    np.testing.assert_allclose(
        refined[0].bbox, (530, 370, 630, 470), atol=2,
    )
    assert abs(refined[0].cx - 580) < 8
    assert abs(refined[0].cy - 420) < 8
