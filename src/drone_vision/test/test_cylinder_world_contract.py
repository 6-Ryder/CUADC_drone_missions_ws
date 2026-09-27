"""Cross-package contract tests for cylinder World coordinates."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from drone_interfaces.msg import CylinderDetection
from drone_mission.mission_state_machine import MissionFSM
from drone_vision.cylinder_detector import CylinderDetectorNode
from drone_vision.vision_pipeline import VisionPipelineNode


ROOT = Path(__file__).resolve().parents[3]
MESSAGE = ROOT / "src/drone_interfaces/msg/CylinderDetection.msg"
CYLINDER_TARGET_CONSUMERS = (
    ROOT / "src/drone_mission/drone_mission/mission_state_machine.py",
    ROOT / "src/drone_mission/drone_mission/cylinder_selector.py",
    ROOT / "src/drone_mission/drone_mission/drop_visual_aligner.py",
    ROOT / "src/drone_mission/drone_mission/precision_align_controller.py",
    ROOT / "src/drone_mission/drone_mission/states/state_cylinder_approach.py",
)


def _message_fields():
    return [
        line.split("#", 1)[0].strip()
        for line in MESSAGE.read_text(encoding="utf-8").splitlines()
    ]


def _constant_string(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _obsolete_field_accesses(source):
    obsolete = {"center_x", "center_y"}
    accesses = []
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Attribute) and node.attr in obsolete:
            accesses.append((node.lineno, node.attr))
        elif isinstance(node, ast.Subscript):
            field = _constant_string(node.slice)
            if field in obsolete:
                accesses.append((node.lineno, field))
        elif isinstance(node, ast.Call):
            function_name = (
                node.func.attr if isinstance(node.func, ast.Attribute)
                else node.func.id if isinstance(node.func, ast.Name)
                else ""
            )
            field_arg = None
            if function_name == "get" and node.args:
                field_arg = node.args[0]
            elif (
                function_name in {"getattr", "_field", "_cyl_field"}
                and len(node.args) > 1
            ):
                field_arg = node.args[1]
            field = _constant_string(field_arg)
            if field in obsolete:
                accesses.append((node.lineno, field))
    return accesses


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time())


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Tracker:
    def update(self, _detections):
        pass

    def prune(self):
        pass

    def get_active(self):
        return [{
            "id": 4,
            "confidence": 0.9,
            "cx": 321.0,
            "cy": 239.0,
            "radius": 12.0,
            "bbox": (300, 220, 342, 260),
        }]


def bare_pipeline_without_pose():
    pipeline = object.__new__(VisionPipelineNode)
    pipeline._cyl_refiner = None
    pipeline._cyl_tracker = _Tracker()
    pipeline._cyl_pub = _Publisher()
    pipeline._current_altitude = 5.0
    pipeline._pose_received = False
    pipeline._coord_xform = None
    pipeline._drone_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    parameters = {
        "cylinder_refine_enable": False,
        "fx": 400.0,
        "fy": 400.0,
        "cx": 320.0,
        "cy": 240.0,
        "image_top_faces_forward": True,
    }
    pipeline.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    pipeline.get_clock = lambda: _Clock()
    return pipeline


def bare_memory_fsm():
    fsm = object.__new__(MissionFSM)
    fsm._known_cylinders = []
    fsm._known_markers = []
    fsm._next_known_cylinder_id = 1
    fsm._next_known_marker_id = 1001
    fsm.get_param = lambda _name, default=None: default
    fsm.validate_drop_target_world_xy = lambda x, y: (x, y)
    return fsm


def test_cylinder_message_has_one_explicit_world_coordinate_contract():
    fields = _message_fields()

    assert "bool world_position_valid" in fields
    assert "float64 world_x" in fields
    assert "float64 world_y" in fields
    assert not any(line.startswith("float64 center_x") for line in fields)
    assert not any(line.startswith("float64 center_y") for line in fields)


def test_generated_cylinder_message_exposes_explicit_world_contract():
    detection = CylinderDetection()

    assert hasattr(detection, "world_position_valid")
    assert hasattr(detection, "world_x")
    assert hasattr(detection, "world_y")
    assert not hasattr(detection, "center_x")
    assert not hasattr(detection, "center_y")


def test_cylinder_target_consumers_do_not_use_obsolete_world_field_names():
    for consumer in CYLINDER_TARGET_CONSUMERS:
        source = consumer.read_text(encoding="utf-8")
        assert _obsolete_field_accesses(source) == [], consumer


@pytest.mark.parametrize(
    "source",
    [
        "target.center_x",
        "target['center_y']",
        "target.get('center_x')",
        "getattr(target, 'center_y')",
        "self._field(target, 'center_x', 0.0)",
    ],
)
def test_obsolete_field_scan_detects_actual_access_regardless_of_syntax(source):
    assert _obsolete_field_accesses(source)


def test_obsolete_field_scan_ignores_unrelated_local_pixel_geometry():
    source = "center_x, center_y = ellipse_center\nlabel = 'center_x pixel'"

    assert _obsolete_field_accesses(source) == []


def test_pipeline_without_pose_preserves_pixel_detection_and_invalidates_world():
    pipeline = bare_pipeline_without_pose()
    detection = SimpleNamespace(
        bbox=(300, 220, 342, 260),
        confidence=0.9,
        cx=321.0,
        cy=239.0,
    )

    pipeline._process_cylinders_v2(np.zeros((2, 2, 3)), [detection])

    array = pipeline._cyl_pub.messages[-1]
    cylinder = array.cylinders[0]
    assert array.header.frame_id == "world"
    assert cylinder.center_u == 321.0
    assert cylinder.center_v == 239.0
    assert cylinder.world_position_valid is False
    assert math.isnan(cylinder.world_x)
    assert math.isnan(cylinder.world_y)


def test_bucket_cylinder_uses_capture_stamp_and_rim_plane():
    pipeline = bare_pipeline_without_pose()
    pipeline._bucket_projection = True
    pipeline._bucket_ground_z = -0.40
    pipeline._frame_pose = (0, 0, 1.6, 0, 0, 0, 1)
    pipeline._frame_stamp = Time(sec=12, nanosec=345)
    original_get_parameter = pipeline.get_parameter
    pipeline.get_parameter = lambda name: (
        SimpleNamespace(value={"bucket_rim_height": 0.40,
                               "bucket_camera_down": 0.365}[name])
        if name in {"bucket_rim_height", "bucket_camera_down"}
        else original_get_parameter(name))
    planes = []
    pipeline._project_world_xy = lambda u, v, ground_z=0: (
        planes.append(ground_z) or (1.0, 2.0))
    detection = SimpleNamespace(
        bbox=(300, 220, 342, 260), confidence=0.9, cx=321, cy=239)
    pipeline._process_cylinders_v2(np.zeros((2, 2, 3)), [detection])
    array = pipeline._cyl_pub.messages[-1]
    assert array.header.stamp.sec == 12
    assert array.header.stamp.nanosec == 345
    assert planes == [0.40]
    assert array.cylinders[0].radius == pytest.approx(12 * 1.235 / 400)


def test_bucket_projection_uses_takeoff_pose_as_ground_anchor():
    pipeline = bare_pipeline_without_pose()
    pipeline._bucket_projection = True
    pipeline._bucket_ground_z = None
    planes = []
    pipeline._coord_xform = SimpleNamespace(pixel_to_world=lambda *args, **kwargs: (
        planes.append(kwargs["ground_z"]) or (0.0, 0.0)))
    original_get_parameter = pipeline.get_parameter
    pipeline.get_parameter = lambda name: (
        SimpleNamespace(value=0.40) if name == "bucket_home_body_height"
        else original_get_parameter(name))
    pipeline._pose_cb(SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.2),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
        header=SimpleNamespace(stamp=Time(sec=1))))
    pipeline._frame_pose = (0.0, 0.0, 1.8, 0.0, 0.0, 0.0, 1.0)

    assert pipeline._bucket_ground_z == pytest.approx(-0.2)
    assert pipeline._project_world_xy(320.0, 240.0, 0.40) == (0.0, 0.0)
    assert planes == [pytest.approx(0.2)]


def test_bucket_without_ground_anchor_does_not_publish_world_position():
    pipeline = bare_pipeline_without_pose()
    pipeline._bucket_projection = True
    pipeline._bucket_ground_z = None
    pipeline._frame_pose = (0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 1.0)
    pipeline._coord_xform = SimpleNamespace(
        K=np.diag([400.0, 400.0, 1.0]),
        pixel_to_world=lambda *args, **kwargs: (1.0, 2.0))
    pipeline._frame_stamp = Time(sec=12)
    original_get_parameter = pipeline.get_parameter
    pipeline.get_parameter = lambda name: (
        SimpleNamespace(value={"bucket_rim_height": 0.40,
                               "bucket_camera_down": 0.365}[name])
        if name in {"bucket_rim_height", "bucket_camera_down"}
        else original_get_parameter(name))
    detection = SimpleNamespace(
        bbox=(300, 220, 342, 260), confidence=0.9, cx=321, cy=239)

    pipeline._process_cylinders_v2(np.zeros((2, 2, 3)), [detection])

    cylinder = pipeline._cyl_pub.messages[-1].cylinders[0]
    assert cylinder.world_position_valid is False
    assert math.isnan(cylinder.radius)


def test_empty_pipeline_and_standalone_arrays_keep_world_frame():
    pipeline = bare_pipeline_without_pose()

    pipeline._process_cylinders_v2(np.zeros((2, 2, 3)), [])

    assert pipeline._cyl_pub.messages[-1].header.frame_id == "world"
    standalone = object.__new__(CylinderDetectorNode)
    standalone.get_clock = lambda: _Clock()
    assert standalone._new_cylinder_array().header.frame_id == "world"


def test_mission_rejects_invalid_world_position_before_memory():
    fsm = bare_memory_fsm()
    detection = SimpleNamespace(
        world_position_valid=False,
        world_x=math.nan,
        world_y=math.nan,
        confidence=0.9,
        radius=0.5,
    )

    fsm._remember_markers([detection])

    assert fsm._known_markers == []


@pytest.mark.parametrize("remember_name", ["_remember_cylinders", "_remember_markers"])
@pytest.mark.parametrize(
    ("world_x", "world_y"),
    [(math.nan, 1.0), (1.0, math.nan), (math.inf, 1.0), (1.0, -math.inf)],
)
def test_mission_rejects_nonfinite_world_position_before_memory(
    remember_name, world_x, world_y,
):
    fsm = bare_memory_fsm()
    detection = SimpleNamespace(
        world_position_valid=True,
        world_x=world_x,
        world_y=world_y,
        confidence=0.9,
        radius=0.5,
    )

    getattr(fsm, remember_name)([detection])

    assert fsm._known_cylinders == []
    assert fsm._known_markers == []


@pytest.mark.parametrize(
    ("remember_name", "memory_name"),
    [
        ("_remember_cylinders", "_known_cylinders"),
        ("_remember_markers", "_known_markers"),
    ],
)
def test_mission_remembers_valid_world_position_under_explicit_keys(
    remember_name, memory_name,
):
    fsm = bare_memory_fsm()
    detection = SimpleNamespace(
        world_position_valid=True,
        world_x=41.25,
        world_y=3.5,
        confidence=0.9,
        radius=0.5,
    )

    getattr(fsm, remember_name)([detection])

    remembered = getattr(fsm, memory_name)
    assert len(remembered) == 1
    assert remembered[0]["world_position_valid"] is True
    assert remembered[0]["world_x"] == 41.25
    assert remembered[0]["world_y"] == 3.5
    assert "center_x" not in remembered[0]
    assert "center_y" not in remembered[0]

    remembered[0]["world_position_valid"] = False
    getattr(fsm, remember_name)([detection])

    assert len(remembered) == 1
    assert remembered[0]["world_position_valid"] is True
