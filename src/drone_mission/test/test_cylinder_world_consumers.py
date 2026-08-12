"""Cylinder World-coordinate validity tests at mission consumer boundaries."""

import math

import pytest

from drone_mission.cylinder_selector import CylinderSelector
from drone_mission.drop_visual_aligner import DropVisualAligner
from drone_mission.states.state_cylinder_approach import CylinderApproachState


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class _SelectorMemory:
    _selected_positions = [(41.0, 3.0)]


class _AlignFsm:
    def __init__(self, targets):
        self.cylinder_selector = _SelectorMemory()
        self._targets = targets

    def get_markers(self):
        return self._targets

    def get_cylinders(self):
        return []

    def get_param(self, name, default=None):
        return {
            "camera_cx": 640.0,
            "camera_cy": 480.0,
            "drop_align_pixel_tolerance": 35.0,
            "drop_visual_align_stable_frames": 1,
        }.get(name, default)

    def get_logger(self):
        return _Logger()


@pytest.mark.parametrize(
    "target",
    [
        {"id": 1, "world_x": 41.0, "world_y": 3.0, "radius": 0.5},
        {
            "id": 1,
            "world_position_valid": True,
            "world_y": 3.0,
            "radius": 0.5,
        },
        {
            "id": 1,
            "world_position_valid": True,
            "world_x": 41.0,
            "radius": 0.5,
        },
        {
            "id": 1,
            "world_position_valid": True,
            "world_x": math.nan,
            "world_y": 3.0,
            "radius": 0.5,
        },
    ],
)
def test_selector_rejects_incomplete_or_nonfinite_world_contract(target):
    assert CylinderSelector().select([target]) is None


def test_selector_accepts_explicit_valid_finite_world_contract():
    target = {
        "id": 7,
        "world_position_valid": True,
        "world_x": 41.0,
        "world_y": 3.0,
        "radius": 0.5,
        "confidence": 0.9,
    }

    assert CylinderSelector().select([target]) == 7


@pytest.mark.parametrize(
    "target",
    [
        {
            "confidence": 0.9,
            "center_u": 640.0,
            "center_v": 480.0,
            "world_x": 41.0,
            "world_y": 3.0,
        },
        {
            "confidence": 0.9,
            "center_u": 640.0,
            "center_v": 480.0,
            "world_position_valid": True,
            "world_y": 3.0,
        },
        {
            "confidence": 0.9,
            "center_u": 640.0,
            "center_v": 480.0,
            "world_position_valid": True,
            "world_x": math.inf,
            "world_y": 3.0,
        },
    ],
)
def test_pixel_aligner_uses_pixels_without_claiming_invalid_world_identity(target):
    aligner = DropVisualAligner(_AlignFsm([target]))

    selected, status = aligner._choose_target()

    assert selected is target
    assert status.pixel_error == 0.0
    assert status.target_world_xy is None
    assert aligner._locked_target_xy is None
    assert aligner.is_centered() is True


@pytest.mark.parametrize(
    "target",
    [
        {"world_x": 41.0, "world_y": 3.0},
        {"world_position_valid": True, "world_y": 3.0},
        {"world_position_valid": True, "world_x": 41.0},
        {
            "world_position_valid": True,
            "world_x": 41.0,
            "world_y": math.nan,
        },
    ],
)
def test_approach_rejects_incomplete_or_nonfinite_world_contract(target):
    state = CylinderApproachState(object())
    state._target_world_xy = (41.0, 3.0)

    assert state._find_nearest_target([target]) is None
