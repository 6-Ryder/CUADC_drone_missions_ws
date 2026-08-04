# Camera Calibration Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make one calibrated `CameraInfo` stream the only runtime source of image geometry for the camera, vision pipeline, full mission, and independent `test_mission` flow.

**Architecture:** A focused `camera_calibration.py` module loads and validates standard ROS camera-calibration YAML. The camera node publishes that model, while all downstream nodes cache `CameraInfo` and refuse to report ready until its dimensions match incoming images. The full and test FSMs remain independent consumers.

**Tech Stack:** ROS 2 Humble, Python 3.10, `sensor_msgs/CameraInfo`, PyYAML, pytest, ROS 2 launch.

## Global Constraints

- Preserve independent `MissionFSM` and `test_mission` orchestration.
- Use provisional real-camera values `640×480`, `fx=fy=400.0`, `cx=320.0`, `cy=240.0`, and five zero distortion coefficients.
- Do not keep a second live `fx/fy/cx/cy` parameter source in `vision_pipeline`.
- SITL and real hardware may load different calibration files, but consume the same `CameraInfo` contract.
- Treat the deployed real v1 PX4 interface as canonical: source defaults remain `/fmu/out/vehicle_status_v1` and real servo ACK remains `/fmu/out/vehicle_command_ack`; only SITL may select `/fmu/out/vehicle_status_v4` through its isolated overlay and launch override.
- A dimension mismatch must keep vision `ready=0`; it must not silently rescale calibration.
- Commit after every task.

---

## File Structure

- Create `src/drone_vision/drone_vision/camera_calibration.py`: immutable calibration parsing, validation, and `CameraInfo` conversion.
- Create `src/drone_vision/config/real_camera_calibration.yaml`: provisional real-hardware calibration.
- Create `src/drone_vision/config/sitl_camera_calibration.yaml`: current 1280×960 SITL calibration.
- Create `src/drone_vision/test/test_camera_calibration.py`: parser and validation tests.
- Modify `src/drone_vision/drone_vision/camera.py`: load one calibration file and publish it.
- Modify `src/drone_vision/drone_vision/vision_pipeline.py`: cache active `CameraInfo` and use it for every geometric calculation.
- Modify `src/drone_mission/drone_mission/mission_state_machine.py`: cache camera geometry for the full FSM.
- Modify `src/drone_mission/drone_mission/precision_align_controller.py`: obtain geometry from the FSM context.
- Modify `src/drone_mission/drone_mission/drop_visual_aligner.py`: obtain geometry from the FSM context.
- Modify `src/drone_mission/drone_mission/states/state_cylinder_approach.py`: use active geometry.
- Modify `src/drone_mission/drone_mission/states/state_precision_land.py`: use normalized image error.
- Modify `src/drone_mission/config/mission_params.yaml`: replace pixel-based landing tuning with normalized tuning.
- Modify `src/test_mission/test_mission/test_forward_recon.py`: independently cache `CameraInfo` and use normalized image error.
- Modify `src/drone_bringup/scripts/gz_camera_bridge.py`: publish SITL `CameraInfo` beside each image source.
- Modify launch files under `src/drone_bringup/launch/` and `src/test_mission/launch/`: pass calibration paths instead of numeric intrinsics.

### Task 1: Calibration Value Object and Provisional Files

**Files:**
- Create: `src/drone_vision/drone_vision/camera_calibration.py`
- Create: `src/drone_vision/config/real_camera_calibration.yaml`
- Create: `src/drone_vision/config/sitl_camera_calibration.yaml`
- Create: `src/drone_vision/test/test_camera_calibration.py`
- Modify: `src/drone_vision/package.xml`

**Interfaces:**
- Produces: `CameraCalibration.load(path: str) -> CameraCalibration`
- Produces: `CameraCalibration.from_camera_info(msg: CameraInfo) -> CameraCalibration`
- Produces: `CameraCalibration.matches_image(width: int, height: int) -> bool`
- Produces: `CameraCalibration.to_message(stamp, frame_id: str) -> CameraInfo`

- [ ] **Step 1: Write failing parser tests**

```python
from pathlib import Path

import pytest
from sensor_msgs.msg import CameraInfo

from drone_vision.camera_calibration import CameraCalibration


ROOT = Path(__file__).resolve().parents[1]


def test_real_calibration_loads_verified_provisional_values():
    model = CameraCalibration.load(
        str(ROOT / "config/real_camera_calibration.yaml")
    )
    assert (model.width, model.height) == (640, 480)
    assert (model.fx, model.fy, model.cx, model.cy) == (400.0, 400.0, 320.0, 240.0)
    assert model.distortion == (0.0, 0.0, 0.0, 0.0, 0.0)
    assert model.matches_image(640, 480)
    assert not model.matches_image(1280, 960)


def test_camera_info_rejects_invalid_intrinsics():
    msg = CameraInfo(width=640, height=480)
    with pytest.raises(ValueError, match="camera matrix"):
        CameraCalibration.from_camera_info(msg)
```

- [ ] **Step 2: Run the tests and verify import failure**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/drone_vision/test/test_camera_calibration.py
```

Expected: FAIL with `ModuleNotFoundError: No module named 'drone_vision.camera_calibration'`.

- [ ] **Step 3: Implement the immutable calibration model**

Create a frozen dataclass that validates positive dimensions, finite positive focal lengths, a principal point inside the image, a 3×3 `K`, and at least five distortion values. `load()` must read the standard ROS keys `image_width`, `image_height`, `camera_matrix.data`, `distortion_model`, and `distortion_coefficients.data`. `to_message()` must populate `width`, `height`, `k`, `d`, `distortion_model`, identity `r`, and a projection matrix derived from `K`.

```python
@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    matrix: tuple[float, ...]
    distortion_model: str
    distortion: tuple[float, ...]

    @property
    def fx(self) -> float:
        return self.matrix[0]

    @property
    def fy(self) -> float:
        return self.matrix[4]

    @property
    def cx(self) -> float:
        return self.matrix[2]

    @property
    def cy(self) -> float:
        return self.matrix[5]

    @property
    def diagonal(self) -> float:
        return math.hypot(self.width, self.height)

    def matches_image(self, width: int, height: int) -> bool:
        return (int(width), int(height)) == (self.width, self.height)
```

- [ ] **Step 4: Add concrete calibration YAML files**

The real file must contain the approved provisional values. The SITL file must contain `image_width: 1280`, `image_height: 960`, `fx=fy=539.9`, `cx=640.0`, `cy=480.0`, and zero distortion. Both files must include identity rectification and matching projection matrices.

- [ ] **Step 5: Declare the PyYAML runtime dependency and run tests**

Add `<exec_depend>python3-yaml</exec_depend>` to `src/drone_vision/package.xml`.

Run the exact Task 1 test command. Expected: `2 passed`.

- [ ] **Step 6: Commit**

```bash
git add src/drone_vision/drone_vision/camera_calibration.py src/drone_vision/config/real_camera_calibration.yaml src/drone_vision/config/sitl_camera_calibration.yaml src/drone_vision/test/test_camera_calibration.py src/drone_vision/package.xml
git commit -m "feat(vision): add calibrated camera model"
```

### Task 2: Camera Driver Publishes the Calibration Model

**Files:**
- Modify: `src/drone_vision/drone_vision/camera.py`
- Modify: `src/drone_vision/test/test_camera_startup.py`

**Interfaces:**
- Consumes: `CameraCalibration.load()` and `CameraCalibration.to_message()`
- Produces: latched `/drone/camera/camera_info` matching `/drone/camera/image_raw`

- [ ] **Step 1: Add failing driver tests**

Add a fake calibration and assert `_publish_camera_info()` forwards its matrix and distortion. Add a V4L2 test asserting a negotiated dimension mismatch with the calibration raises `RuntimeError` before the capture thread starts.

```python
def test_negotiated_size_must_match_calibration():
    camera = _UsbCamera()
    camera._calibration = SimpleNamespace(width=1280, height=960)
    with pytest.raises(RuntimeError, match="标定分辨率"):
        CameraDriverNode._validate_calibrated_size(camera, 640, 480)
```

- [ ] **Step 2: Verify the new tests fail**

Run `python3 -m pytest -q src/drone_vision/test/test_camera_startup.py`. Expected: FAIL because `_validate_calibrated_size` does not exist.

- [ ] **Step 3: Replace numeric intrinsic parameters with one file parameter**

Declare `camera_calibration_file` and load it during construction. Keep `camera_width` and `camera_height` only as requested V4L2 transport dimensions; verify they equal calibration dimensions. Remove `fx`, `fy`, `cx`, and `cy` declarations. `_publish_camera_info()` must call:

```python
msg = self._calibration.to_message(
    self.get_clock().now().to_msg(),
    "camera_optical_frame",
)
self._info_pub.publish(msg)
```

- [ ] **Step 4: Run camera tests**

Run `python3 -m pytest -q src/drone_vision/test/test_camera_startup.py src/drone_vision/test/test_camera_calibration.py`. Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/drone_vision/drone_vision/camera.py src/drone_vision/test/test_camera_startup.py
git commit -m "feat(vision): publish camera calibration from one file"
```

### Task 3: Vision Pipeline Uses Only Active CameraInfo

**Files:**
- Modify: `src/drone_vision/drone_vision/vision_pipeline.py`
- Create: `src/drone_vision/test/test_camera_info_runtime.py`

**Interfaces:**
- Consumes: `CameraCalibration.from_camera_info()`
- Produces: `VisionPipelineNode._camera_model: Optional[CameraCalibration]`
- Produces: status field `camera_info=0|1` and `ready=1` only after a matching frame

- [ ] **Step 1: Write failing callback and readiness tests**

Test that valid `CameraInfo` creates the coordinate transformer, a 640×480 frame matches, a 1280×960 frame does not, and status remains not ready without calibration.

```python
def test_ready_requires_matching_camera_info():
    pipeline = _PipelineHarness()
    pipeline._initialized = True
    pipeline._models_validated = True
    pipeline._frame_count = 1
    pipeline._camera_model = None
    pipeline._last_image_size = (640, 480)
    assert not VisionPipelineNode._runtime_ready(pipeline)
```

- [ ] **Step 2: Verify tests fail**

Run `python3 -m pytest -q src/drone_vision/test/test_camera_info_runtime.py`. Expected: FAIL because `_runtime_ready` does not exist.

- [ ] **Step 3: Implement active-camera state**

Initialize `_camera_model = None` and `_last_image_size = None`. `_caminfo_cb()` must parse and validate the message, store the model, create or update `CoordinateTransformer`, and reject invalid messages with an error log. `_image_cb()` must record `(msg.width, msg.height)` and return without inference when it does not match the active model.

Add:

```python
def _runtime_ready(self) -> bool:
    return bool(
        self._initialized
        and self._models_validated
        and self._frame_count > 0
        and self._camera_model is not None
        and self._last_image_size is not None
        and self._camera_model.matches_image(*self._last_image_size)
    )
```

- [ ] **Step 4: Replace all live geometry reads**

Remove `fx/fy/cx/cy` parameter declarations and replace every geometric use with `self._camera_model.fx`, `.fy`, `.cx`, `.cy`, `.width`, `.height`, or `.diagonal`. This includes marker expected-diameter filtering, cylinder projection, marker projection, and landing-target projection. No `get_parameter("fx")`, `get_parameter("fy")`, `get_parameter("cx")`, or `get_parameter("cy")` may remain.

- [ ] **Step 5: Run vision tests and source assertion**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/drone_vision/test/test_camera_info_runtime.py src/drone_vision/test/test_coordinate_transforms.py src/drone_vision/test/test_landing_pad_detection.py src/drone_vision/test/test_marker_refiner.py
rg 'get_parameter\("(fx|fy|cx|cy)"\)' src/drone_vision/drone_vision/vision_pipeline.py
```

Expected: tests PASS and `rg` returns no matches.

- [ ] **Step 6: Commit**

```bash
git add src/drone_vision/drone_vision/vision_pipeline.py src/drone_vision/test/test_camera_info_runtime.py
git commit -m "fix(vision): use active CameraInfo for all geometry"
```

### Task 4: Independent FSMs Consume CameraInfo

**Files:**
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Modify: `src/drone_mission/drone_mission/precision_align_controller.py`
- Modify: `src/drone_mission/drone_mission/drop_visual_aligner.py`
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_approach.py`
- Modify: `src/drone_mission/drone_mission/states/state_precision_land.py`
- Modify: `src/drone_mission/config/mission_params.yaml`
- Modify: `src/test_mission/test_mission/test_forward_recon.py`
- Modify: `src/drone_mission/test/test_precision_align_controller.py`
- Modify: `src/drone_mission/test/test_flight_safety_contract.py`

**Interfaces:**
- Produces independently in each FSM: `get_camera_geometry() -> tuple[float, float, float, float, float, float]` returning `(fx, fy, cx, cy, width, height)`
- Produces: normalized landing error `du/diagonal`, `dv/diagonal`

- [ ] **Step 1: Add failing geometry tests**

Extend both FSM test harnesses with a 640×480 `CameraInfo`. Assert the full FSM and `test_mission` cache their own values. Extend precision-align tests so `_camera_geometry()` reads `fsm.get_camera_geometry()` instead of parameters.

- [ ] **Step 2: Verify the tests fail**

Run `python3 -m pytest -q src/drone_mission/test/test_precision_align_controller.py src/drone_mission/test/test_flight_safety_contract.py`. Expected: FAIL because the new context method is not consumed.

- [ ] **Step 3: Add independent CameraInfo subscriptions**

Each FSM stores its own six-value geometry tuple. The callback must reject a zero focal length or zero dimensions. `MissionFSM.get_camera_geometry()` raises `RuntimeError("camera info not ready")` until valid data exists; `test_mission` uses the same contract in its own class without calling `MissionFSM`.

- [ ] **Step 4: Convert full-FSM consumers**

Change `PrecisionAlignController._camera_geometry()`, `CylinderApproachState`, `DropVisualAligner`, and `PrecisionLandState` to consume `get_camera_geometry()`. Replace landing pixel gain with diagonal-normalized values that preserve current 640×480 behavior:

```python
diagonal = math.hypot(width, height)
du_norm = (u - cx) / diagonal
dv_norm = (v - cy) / diagonal
gain = float(self._fsm.get_param("landing_servo_gain_normalized", 3.2))
deadzone = float(self._fsm.get_param("landing_servo_deadzone_ratio", 0.015))
pixel_error_ratio = math.hypot(du_norm, dv_norm)
```

Use `gain * du_norm` and `gain * dv_norm` before the existing velocity clamp.

- [ ] **Step 5: Convert independent test-mission landing control**

Implement the same normalized equations locally in `TestForwardRecon`; do not import a mission state or share runtime state.

- [ ] **Step 6: Run mission and test-mission tests**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/drone_mission/test/test_precision_align_controller.py src/drone_mission/test/test_flight_safety_contract.py
python3 -m pytest -q src/test_mission/test -k 'not flake8 and not pep257 and not copyright'
```

Expected: all selected tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/drone_mission src/test_mission
git commit -m "fix(mission): derive image control from CameraInfo"
```

### Task 5: Launch Wiring and Calibration Replacement Method

**Files:**
- Modify: `src/drone_bringup/scripts/gz_camera_bridge.py`
- Modify: `src/drone_bringup/launch/real_flight.launch.py`
- Modify: `src/drone_bringup/launch/sitl.launch.py`
- Modify: `src/drone_bringup/launch/vision_only.launch.py`
- Modify: `src/test_mission/launch/test_forward_recon.launch.py`
- Modify: `src/drone_bringup/test/test_vision_launch_configuration.py`
- Modify: `docs/REAL_HARDWARE_DEBUG_GUIDE.md`
- Modify: `docs/PARAMETER_USAGE_GUIDE.md`

**Interfaces:**
- Produces launch argument: `camera_calibration_file`
- Produces SITL `/drone/camera/camera_info` with transient-local reliable QoS

- [ ] **Step 1: Add failing launch contract tests**

Assert every real-camera launch exposes `camera_calibration_file`, references `real_camera_calibration.yaml`, and contains no numeric `fx/fy/cx/cy`. Assert the Gazebo bridge publishes `CameraInfo` from `sitl_camera_calibration.yaml`.

- [ ] **Step 2: Verify the launch tests fail**

Run `python3 -m pytest -q src/drone_bringup/test/test_vision_launch_configuration.py`. Expected: FAIL on missing calibration arguments.

- [ ] **Step 3: Wire calibration paths**

Use `get_package_share_directory("drone_vision")` to build default real and SITL calibration paths. Pass the chosen path to camera sources. Remove `camera_fx/camera_fy/camera_cx/camera_cy` launch overrides. Extend `gz_camera_bridge.py` with a `CameraInfo` publisher using `CameraCalibration.load()`.

- [ ] **Step 4: Document replacement and verification**

Document the exact commands:

```bash
ros2 topic echo /drone/camera/camera_info --once
ros2 param get /camera_driver camera_calibration_file
./scripts/debug_vision.sh
```

State that replacing `real_camera_calibration.yaml` requires a rebuild and a ground H-center regression before flight.

- [ ] **Step 5: Run package regressions**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/drone_vision/test -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest -q src/drone_bringup/test -k 'not flake8 and not pep257 and not copyright'
```

Expected: all selected tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/drone_bringup src/test_mission/launch docs/REAL_HARDWARE_DEBUG_GUIDE.md docs/PARAMETER_USAGE_GUIDE.md
git commit -m "feat(bringup): wire calibrated camera runtime"
```
