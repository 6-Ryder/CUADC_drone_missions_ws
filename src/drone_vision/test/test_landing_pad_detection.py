import cv2
import numpy as np
from types import SimpleNamespace

from drone_vision.landing_detector import LandingCircleTracker, LandingPadRefiner
from drone_vision.vision_pipeline import VisionPipelineNode


def _landing_scene(with_h=True, center=(320, 240), radius=80):
    image = np.full((480, 640, 3), 180, dtype=np.uint8)
    cx, cy = center
    cv2.circle(image, (cx, cy), radius, (55, 55, 55), -1)
    cv2.circle(image, (cx, cy), radius, (245, 245, 245), 8)
    if with_h:
        cv2.rectangle(image, (cx - 35, cy - 40), (cx - 23, cy + 40), (245, 245, 245), -1)
        cv2.rectangle(image, (cx + 23, cy - 40), (cx + 35, cy + 40), (245, 245, 245), -1)
        cv2.rectangle(image, (cx - 28, cy - 6), (cx + 28, cy + 6), (245, 245, 245), -1)
    return image


def _landing_ellipse_scene(center=(320, 240), axes=(90, 50)):
    """Render a perspective-projected landing circle as an image-space ellipse."""
    image = np.full((480, 640, 3), 180, dtype=np.uint8)
    cx, cy = center
    cv2.ellipse(image, (cx, cy), axes, 0, 0, 360, (55, 55, 55), -1)
    cv2.ellipse(image, (cx, cy), axes, 0, 0, 360, (245, 245, 245), 8)
    cv2.rectangle(image, (cx - 35, cy - 30), (cx - 23, cy + 30), (245, 245, 245), -1)
    cv2.rectangle(image, (cx + 23, cy - 30), (cx + 35, cy + 30), (245, 245, 245), -1)
    cv2.rectangle(image, (cx - 28, cy - 6), (cx + 28, cy + 6), (245, 245, 245), -1)
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


def test_circle_tracker_tracks_bounded_translation_after_seed():
    tracker = LandingCircleTracker()

    seed = tracker.seed(_landing_scene(), (220, 140, 420, 340))
    tracked = tracker.track(_landing_scene(center=(332, 248)))

    assert seed is not None
    assert tracked is not None
    assert abs(tracked.center_u - 332) < 4
    assert abs(tracked.center_v - 248) < 4
    assert tracked.edge_margin_px > 100


def test_circle_tracker_rejects_edge_clipped_candidate():
    tracker = LandingCircleTracker(max_center_jump_px=100.0)

    assert tracker.seed(_landing_scene(center=(110, 240)), (20, 140, 220, 340))
    assert tracker.track(_landing_scene(center=(30, 240))) is None
    assert tracker.last_rejection_reason == "non_elliptical"


def test_circle_tracker_accepts_perspective_projected_landing_ellipse():
    tracker = LandingCircleTracker()

    seeded = tracker.seed(_landing_ellipse_scene(), (200, 160, 440, 320))
    tracked = tracker.track(_landing_ellipse_scene(center=(328, 245)))

    assert seeded is not None
    assert tracked is not None
    assert abs(tracked.center_u - 328) < 4
    assert abs(tracked.center_v - 245) < 4


def _landing_pipeline_for_track_selection():
    pipeline = object.__new__(VisionPipelineNode)
    pipeline._landing_refiner = LandingPadRefiner(circle_min_area=80)
    pipeline._landing_circle_tracker = LandingCircleTracker()
    pipeline.get_parameter = lambda _name: SimpleNamespace(value=True)
    return pipeline


def test_pipeline_uses_cv_circle_track_when_yolo_guard_has_no_detection():
    pipeline = _landing_pipeline_for_track_selection()
    detection = SimpleNamespace(
        bbox=(220, 140, 420, 340), confidence=0.9, cx=320.0, cy=240.0,
    )

    seeded, seeded_source = pipeline._select_landing_circle_track(
        _landing_scene(), [detection],
    )
    tracked, tracked_source = pipeline._select_landing_circle_track(
        _landing_scene(center=(330, 246)), [],
    )

    assert seeded is not None
    assert seeded_source == "yolo_refined"
    assert tracked is not None
    assert tracked_source == "cv_tracked"
    assert abs(tracked.center_u - 330) < 4


def test_pipeline_rejects_raw_yolo_bbox_when_circle_refinement_fails():
    pipeline = _landing_pipeline_for_track_selection()
    detection = SimpleNamespace(
        bbox=(220, 140, 420, 340), confidence=0.9, cx=320.0, cy=240.0,
    )

    track, source = pipeline._select_landing_circle_track(
        np.full((480, 640, 3), 180, dtype=np.uint8), [detection],
    )

    assert track is None
    assert source == "none"


def test_pipeline_keeps_legacy_yolo_target_as_circle_tracking_fallback():
    pipeline = _landing_pipeline_for_track_selection()
    detection = SimpleNamespace(
        bbox=(220, 140, 420, 340), confidence=0.9, cx=320.0, cy=240.0,
    )

    fallback, source = pipeline._select_landing_legacy_target(
        _landing_scene(), [detection],
    )

    assert fallback is not None
    assert source == "yolo_refined_fallback"
    assert abs(fallback[0] - 320) < 4
    assert abs(fallback[1] - 240) < 4


def test_pipeline_runs_yolo_guard_only_when_circle_track_is_due():
    pipeline = _landing_pipeline_for_track_selection()
    pipeline._landing_last_yolo_inference_monotonic = 10.0
    pipeline._landing_circle_tracker.seed(_landing_scene(), (220, 140, 420, 340))

    assert pipeline._landing_yolo_inference_due(10.1, guard_rate_hz=2.0) is False
    assert pipeline._landing_yolo_inference_due(10.5, guard_rate_hz=2.0) is True


def test_pipeline_enables_circle_tracking_only_during_landing_mode():
    pipeline = object.__new__(VisionPipelineNode)
    pipeline._detection_mode = "all"
    pipeline.get_parameter = lambda _name: SimpleNamespace(value=True)

    assert pipeline._circle_tracking_active() is False

    pipeline._detection_mode = "landing"
    assert pipeline._circle_tracking_active() is True
