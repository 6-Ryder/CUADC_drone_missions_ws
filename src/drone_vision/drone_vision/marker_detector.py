"""Blue drop-zone marker geometry for YOLO refinement and OpenCV detection."""

from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np

from drone_vision.models.yolo_engine import Detection


@dataclass(frozen=True)
class _BlueComponent:
    label: int
    bbox: Tuple[int, int, int, int]
    cx: float
    cy: float
    circularity: float


class BlueMarkerRefiner:
    """Recover complete blue circular markers and reject non-blue YOLO boxes."""

    def __init__(
        self,
        min_component_area: int = 80,
        min_detection_blue_ratio: float = 0.4,
        min_aspect_ratio: float = 0.4,
        min_circularity: float = 0.35,
        min_component_diameter: int = 30,
    ):
        self.min_component_area = min_component_area
        self.min_detection_blue_ratio = min_detection_blue_ratio
        self.min_aspect_ratio = min_aspect_ratio
        self.min_circularity = min_circularity
        self.min_component_diameter = min_component_diameter

    @staticmethod
    def _blue_mask(image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([95, 60, 60], dtype=np.uint8),
            np.array([135, 255, 255], dtype=np.uint8),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _components(
        self, image: np.ndarray,
    ) -> Tuple[np.ndarray, Dict[int, _BlueComponent]]:
        mask = self._blue_mask(image)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask, connectivity=8,
        )
        components: Dict[int, _BlueComponent] = {}

        for label in range(1, count):
            x, y, width, height, area = stats[label]
            if area < self.min_component_area or width <= 0 or height <= 0:
                continue
            if min(width, height) < self.min_component_diameter:
                continue
            aspect = min(width, height) / max(width, height)
            if aspect < self.min_aspect_ratio:
                continue

            component_mask = np.where(
                labels[y:y + height, x:x + width] == label, 255, 0,
            ).astype(np.uint8)
            contours, _ = cv2.findContours(
                component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
            )
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(contour)
            (local_cx, local_cy), radius = cv2.minEnclosingCircle(contour)
            if radius <= 0:
                continue
            circularity = contour_area / (np.pi * radius * radius)
            if circularity < self.min_circularity:
                continue

            components[label] = _BlueComponent(
                label=label,
                bbox=(int(x), int(y), int(x + width), int(y + height)),
                cx=float(local_cx + x),
                cy=float(local_cy + y),
                circularity=float(min(circularity, 1.0)),
            )

        return labels, components

    def refine(
        self, image: np.ndarray, detections: List[Detection],
    ) -> List[Detection]:
        """Validate YOLO boxes against blue pixels and restore full components."""
        labels, components = self._components(image)
        height, width = image.shape[:2]
        selected: Dict[int, Detection] = {}

        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            x1 = max(0, min(width, int(x1)))
            y1 = max(0, min(height, int(y1)))
            x2 = max(0, min(width, int(x2)))
            y2 = max(0, min(height, int(y2)))
            if x2 <= x1 or y2 <= y1:
                continue

            region = labels[y1:y2, x1:x2]
            counts = np.bincount(region.ravel())
            if counts.size <= 1:
                continue
            counts[0] = 0
            label = int(np.argmax(counts))
            component = components.get(label)
            blue_ratio = counts[label] / region.size
            if component is None or blue_ratio < self.min_detection_blue_ratio:
                continue

            refined = Detection(
                class_id=detection.class_id,
                class_name=detection.class_name,
                confidence=detection.confidence,
                bbox=component.bbox,
                cx=component.cx,
                cy=component.cy,
            )
            previous = selected.get(label)
            if previous is None or refined.confidence > previous.confidence:
                selected[label] = refined

        return sorted(
            selected.values(), key=lambda detection: detection.confidence,
            reverse=True,
        )

    def detect(self, image: np.ndarray) -> List[Detection]:
        """Detect complete blue circular components without a YOLO model."""
        _, components = self._components(image)
        detections = [
            Detection(
                class_id=0,
                class_name="drop_zone_marker",
                confidence=min(0.99, 0.5 + component.circularity * 0.5),
                bbox=component.bbox,
                cx=component.cx,
                cy=component.cy,
            )
            for component in components.values()
        ]
        return sorted(
            detections, key=lambda detection: detection.confidence,
            reverse=True,
        )

    @staticmethod
    def filter_by_expected_diameter(
        detections: List[Detection],
        expected_diameter_px: float,
        min_ratio: float = 0.45,
        max_ratio: float = 1.8,
    ) -> List[Detection]:
        """Reject blue components inconsistent with the known 1m marker size."""
        if expected_diameter_px <= 0:
            return []
        filtered = []
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            diameter = ((x2 - x1) + (y2 - y1)) / 2.0
            ratio = diameter / expected_diameter_px
            if min_ratio <= ratio <= max_ratio:
                filtered.append(detection)
        return filtered
