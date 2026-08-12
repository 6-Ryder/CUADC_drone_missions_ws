# Arbitrary Layout Heading Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make both complete MissionFSM strategies fly an unchanged local mission at any configured absolute NED heading, with the quick strategy as the authoritative implementation and efficiency baseline.

**Architecture:** Add one immutable `LayoutFrame` that converts field-local forward/right coordinates to PX4 NED. Mission geometry, visual coverage, and the generated SITL field consume that same launch-time heading; `FlightController` owns one task yaw that survives every position/velocity mode switch. Implement and validate the quick strategy first, then reuse the same code for the main strategy without changing either strategy's 0° behavior.

**Tech Stack:** Python 3.10, ROS 2 Humble/rclpy, PX4 `TrajectorySetpoint`, Gazebo Harmonic SDF 1.11, pytest, colcon, Bash launch wrappers.

## Global Constraints

- `layout_heading_deg` is the only direction parameter: degrees, 0°=NED north, clockwise positive, default `0.0`, immutable during one mission.
- Normalize every finite input to `[0, 360)` and publish PX4 yaw in `[-π, π]`; reject NaN and infinity before Arm.
- The quick MissionFSM strategy is authoritative. Preserve its marker-only detection, continuous drop servo, simplified landing, speeds, state sequence, and completion conditions.
- Preserve the main MissionFSM strategy's existing drop, reconnaissance, return, precision landing, recovery, and completion behavior.
- Preserve exact 0° north behavior for both strategies except for intentional parameter/field renames.
- Do not add an alignment state, yaw tolerance, yaw timeout, or task delay. Rotate during the existing Offboard climb and continue converging during transit.
- All position and velocity stages hold `layout_heading_deg`; reconnaissance strips never reverse aircraft yaw.
- Remove `flight_axis`, `drop_distance`, `recon_distance`, `area*_north/east_*`,
  `sweep_margin_north/east`, and `coverage_zone_x/y_*`; do not add compatibility aliases.
- Do not reset EKF yaw, redefine NED, capture initial aircraft yaw as layout heading, or support in-flight heading changes.
- World target coordinates are valid only with fresh 6-DoF pose data; never use the north-assuming simple projection as a world-coordinate fallback.
- Primary end-to-end acceptance is quick SITL at 0° and 50°. The 50° median of three successful runs must be no more than 5% slower than the 0° median.
- Main SITL must subsequently complete one 0° and one 50° full-flow regression.

## File and Responsibility Map

| Responsibility | Files |
|---|---|
| Field-local/NED math | Create `src/drone_utils/drone_utils/layout_frame.py`; export from `src/drone_utils/drone_utils/__init__.py` |
| Task-yaw invariant | Modify `src/px4_control/px4_control/flight_controller.py`, `standalone_bridge.py`, `px4_bridge.py` |
| Mission geometry/config | Modify `src/drone_mission/drone_mission/mission_state_machine.py`, four `src/drone_mission/config/mission_params*.yaml` files |
| Coverage/search paths | Modify `path_planner.py`, `state_recon_patrol.py`, `state_cylinder_search.py`, `state_transit.py`, `state_return.py`, `state_cylinder_approach.py` |
| Explicit visual coordinates | Modify `src/drone_interfaces/msg/CylinderDetection.msg`, mission target consumers, `vision_pipeline.py`, `cylinder_detector.py` |
| Rotated coverage | Modify `src/drone_vision/drone_vision/coverage_monitor.py`, `vision_pipeline.py`, `vision_params.yaml` |
| Four deployment profiles | Modify `sitl.launch.py`, `quick_sitl.launch.py`, `real_flight.launch.py`, `quick_flight.launch.py` and their deployment tests |
| Rotated SITL assets | Create `field/layout_field.py`; modify `field/generate_random_field.py`, both simulation start scripts |
| Efficiency evidence | Modify mission summary fields; create `scripts/summarize_layout_benchmark.py` and tests |
| Operator documentation | Modify `docs/ARCHITECTURE.md`, `docs/CODE_READING_GUIDE.md`, `docs/Debug/PARAMETER_USAGE_GUIDE.md`, `field/README.md` |

---

## Required Execution Order

Execute strictly in the section order below:
**1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12**. Tasks 1–3 establish
the shared frame, yaw invariant, and parameters consumed by Tasks 4–10. Task 11 is the
static/package gate; Task 12 is the runtime quick-first acceptance gate.

---

### Task 1: Add the immutable field-local coordinate module

**Files:**
- Create: `src/drone_utils/drone_utils/layout_frame.py`
- Modify: `src/drone_utils/drone_utils/__init__.py`
- Create: `src/drone_utils/test/test_layout_frame.py`

**Interfaces:**
- Produces: `LayoutFrame(heading_deg: float)`
- Produces: `heading_deg: float`, `heading_rad: float`, `yaw_rad: float`
- Produces: `local_to_ned(forward: float, right: float) -> tuple[float, float]`
- Produces: `ned_to_local(north: float, east: float) -> tuple[float, float]`
- Produces: `bounds_center_ned(forward_min, forward_max, right_min, right_max) -> tuple[float, float]`
- Produces: `contains_ned(north, east, forward_min, forward_max, right_min, right_max, margin=0.0) -> bool`

- [ ] **Step 1: Write the failing geometry tests**

```python
import math

import pytest

from drone_utils.layout_frame import LayoutFrame


@pytest.mark.parametrize("raw, normalized", [
    (0.0, 0.0), (50.0, 50.0), (360.0, 0.0),
    (410.0, 50.0), (-10.0, 350.0),
])
def test_heading_normalizes(raw, normalized):
    assert LayoutFrame(raw).heading_deg == normalized


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_heading_is_rejected(bad):
    with pytest.raises(ValueError, match="finite"):
        LayoutFrame(bad)


@pytest.mark.parametrize("heading", [0.0, 50.0, 90.0, 180.0, 350.0])
@pytest.mark.parametrize("point", [(0.0, 0.0), (32.5, 0.0), (55.0, -4.0)])
def test_local_ned_round_trip(heading, point):
    frame = LayoutFrame(heading)
    north, east = frame.local_to_ned(*point)
    forward, right = frame.ned_to_local(north, east)
    assert forward == pytest.approx(point[0], abs=1e-9)
    assert right == pytest.approx(point[1], abs=1e-9)


def test_fifty_degree_zone_center_and_exact_containment():
    frame = LayoutFrame(50.0)
    center = frame.bounds_center_ned(30.0, 35.0, -4.0, 4.0)
    assert center == pytest.approx(frame.local_to_ned(32.5, 0.0))
    inside = frame.local_to_ned(30.0, 4.0)
    outside = frame.local_to_ned(29.99, 4.01)
    assert frame.contains_ned(*inside, 30.0, 35.0, -4.0, 4.0)
    assert not frame.contains_ned(*outside, 30.0, 35.0, -4.0, 4.0)
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `python3 -m pytest src/drone_utils/test/test_layout_frame.py -q`

Expected: collection fails with `ModuleNotFoundError: No module named 'drone_utils.layout_frame'`.

- [ ] **Step 3: Implement `LayoutFrame` and export it**

```python
"""Immutable transforms between field-local and PX4 NED coordinates."""

from dataclasses import dataclass, field
import math
from typing import Tuple


@dataclass(frozen=True)
class LayoutFrame:
    """Field frame whose forward axis has an absolute clockwise NED heading."""

    heading_deg: float
    _heading_rad: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        value = float(self.heading_deg)
        if not math.isfinite(value):
            raise ValueError("layout heading must be finite")
        normalized = value % 360.0
        object.__setattr__(self, "heading_deg", normalized)
        object.__setattr__(self, "_heading_rad", math.radians(normalized))

    @property
    def heading_rad(self) -> float:
        return self._heading_rad

    @property
    def yaw_rad(self) -> float:
        return math.atan2(math.sin(self._heading_rad), math.cos(self._heading_rad))

    def local_to_ned(self, forward: float, right: float) -> Tuple[float, float]:
        cos_h = math.cos(self._heading_rad)
        sin_h = math.sin(self._heading_rad)
        return forward * cos_h - right * sin_h, forward * sin_h + right * cos_h

    def ned_to_local(self, north: float, east: float) -> Tuple[float, float]:
        cos_h = math.cos(self._heading_rad)
        sin_h = math.sin(self._heading_rad)
        return north * cos_h + east * sin_h, -north * sin_h + east * cos_h

    def bounds_center_ned(
        self, forward_min: float, forward_max: float,
        right_min: float, right_max: float,
    ) -> Tuple[float, float]:
        return self.local_to_ned(
            (forward_min + forward_max) / 2.0,
            (right_min + right_max) / 2.0,
        )

    def contains_ned(
        self, north: float, east: float,
        forward_min: float, forward_max: float,
        right_min: float, right_max: float, margin: float = 0.0,
    ) -> bool:
        forward, right = self.ned_to_local(north, east)
        return (
            forward_min - margin <= forward <= forward_max + margin
            and right_min - margin <= right <= right_max + margin
        )
```

Also add `from drone_utils.layout_frame import LayoutFrame` and `"LayoutFrame"` to
`drone_utils.__all__`.

- [ ] **Step 4: Run geometry and lint tests**

Run: `python3 -m pytest src/drone_utils/test/test_layout_frame.py src/drone_utils/test/test_flake8.py src/drone_utils/test/test_pep257.py -q`

Expected: all tests pass.

- [ ] **Step 5: Commit the coordinate primitive**

```bash
git add src/drone_utils/drone_utils/layout_frame.py src/drone_utils/drone_utils/__init__.py src/drone_utils/test/test_layout_frame.py
git commit -m "feat: add field-local layout frame"
```

---
### Task 2: Make task yaw survive takeoff and velocity control

**Files:**
- Modify: `src/px4_control/px4_control/flight_controller.py`
- Modify: `src/px4_control/px4_control/px4_bridge.py`
- Modify: `src/px4_control/px4_control/standalone_bridge.py`
- Modify: `src/px4_control/test/test_mission_setpoint_contract.py`
- Modify: `src/drone_mission/test/test_flight_safety_contract.py`

**Interfaces:**
- Consumes: normalized `LayoutFrame.yaw_rad`
- Produces: `FlightController(bridge, task_yaw: float = 0.0)`
- Produces: `task_yaw` property and `_resolve_yaw(yaw: Optional[float]) -> float`
- Preserves: `Twist.angular.z` remains yaw-rate; no new ROS relay message

- [ ] **Step 1: Add failing task-yaw and mode-switch contract tests**

Add tests proving:

```python
def test_flight_controller_uses_task_yaw_for_takeoff_loiter_and_default_goto():
    bridge = RecordingBridge(position=(1.0, 2.0, 0.0))
    fc = FlightController(bridge, task_yaw=0.8726646259971648)
    bridge.offboard = True
    fc.takeoff(5.0)
    fc.loiter()
    fc.goto_ned(10.0, 3.0, -5.0)
    assert [call[3] for call in bridge.position_calls] == pytest.approx([
        0.8726646259971648,
        0.8726646259971648,
        0.8726646259971648,
    ])


def test_px4_velocity_setpoint_keeps_absolute_task_yaw():
    bridge, publisher = make_lease_enabled_px4_bridge(target_yaw=0.8726646259971648)
    bridge.publish_velocity_cmd(0.2, -0.1, 0.0)
    setpoint = publisher.messages[-1]
    assert setpoint.yaw == pytest.approx(0.8726646259971648)
    assert math.isnan(setpoint.yawspeed)


def test_velocity_heartbeat_keeps_yaw_after_position_to_velocity_switch():
    bridge, publisher = make_velocity_mode_px4_bridge(target_yaw=0.8726646259971648)
    bridge._offboard_heartbeat()
    assert publisher.messages[-1].yaw == pytest.approx(0.8726646259971648)
```

Use small local `RecordingBridge`/publisher fixtures in the existing contract file; do not
start ROS or PX4 in these tests.

- [ ] **Step 2: Run the focused tests and verify RED**

Run: `python3 -m pytest src/px4_control/test/test_mission_setpoint_contract.py src/drone_mission/test/test_flight_safety_contract.py -q`

Expected: failures show takeoff/loiter still send yaw0 and velocity setpoints still send NaN yaw.

- [ ] **Step 3: Implement the task-yaw invariant**

In `FlightController`:

```python
def __init__(self, bridge, task_yaw: float = 0.0):
    # existing initialization...
    self._task_yaw = float(task_yaw)
    if not math.isfinite(self._task_yaw):
        raise ValueError("task yaw must be finite")

@property
def task_yaw(self) -> float:
    return self._task_yaw

def _resolve_yaw(self, yaw: Optional[float]) -> float:
    return self._task_yaw if yaw is None else float(yaw)
```

Change `goto_ned()`/`goto_world()` defaults to `yaw: Optional[float] = None`; pass
`self._task_yaw` explicitly from `takeoff()` and `loiter()`. Leave body-to-NED velocity
rotation based on `_current_yaw()`.

In both immediate and heartbeat PX4 velocity setpoints use:

```python
setpoint.position = [float("nan")] * 3
setpoint.velocity = [float(vn), float(ve), float(vd)]
setpoint.acceleration = [float("nan")] * 3
setpoint.yaw = float(self._target_yaw)
setpoint.yawspeed = float("nan")
```

Set `yawspeed=NaN` for position setpoints too. Do not clear `_target_yaw` in velocity mode,
`hold_for_takeover()`, or `stop_velocity()`. `StandaloneBridge.set_velocity_target()` must
continue preserving its last `_target_yaw` while publishing the existing `Twist` relay.

- [ ] **Step 4: Run control safety regressions**

Run: `python3 -m pytest src/px4_control/test src/drone_mission/test/test_flight_safety_contract.py src/drone_mission/test/test_hold_for_takeover.py -q`

Expected: all tests pass; existing position speed, lease, and takeover contracts remain green.

- [ ] **Step 5: Commit the yaw invariant**

```bash
git add src/px4_control/px4_control src/px4_control/test/test_mission_setpoint_contract.py src/drone_mission/test/test_flight_safety_contract.py
git commit -m "feat: hold task yaw across flight modes"
```

---
### Task 3: Migrate mission parameters and compute rotated zone centers

**Files:**
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Modify: `src/drone_mission/config/mission_params.yaml`
- Modify: `src/drone_mission/config/mission_params_quick_sim.yaml`
- Modify: `src/drone_mission/config/mission_params_real.yaml`
- Modify: `src/drone_mission/config/mission_params_quick.yaml`
- Create: `src/drone_mission/test/test_layout_mission_config.py`
- Modify: `src/drone_mission/test/test_full_real_recovery_contract.py`

**Interfaces:**
- Consumes: `LayoutFrame`
- Produces: `MissionFSM.layout_frame`, `layout_yaw_rad`, `zone_bounds(prefix)`
- Produces: rotated transit centers derived only from area bounds
- Rejects: runtime writes to `layout_heading_deg` after the startup snapshot

- [ ] **Step 1: Write failing config and center tests**

```python
LEGACY_KEYS = {
    "flight_axis", "drop_distance", "recon_distance",
    "area1_north_min", "area1_north_max", "area1_east_min", "area1_east_max",
    "area2_north_min", "area2_north_max", "area2_east_min", "area2_east_max",
    "sweep_margin_north", "sweep_margin_east",
}


@pytest.mark.parametrize("filename", [
    "mission_params.yaml", "mission_params_quick_sim.yaml",
    "mission_params_real.yaml", "mission_params_quick.yaml",
])
def test_all_profiles_use_local_area_names_without_changing_values(filename):
    params = load_mission_params(filename)
    assert LEGACY_KEYS.isdisjoint(params)
    assert params["area1_forward_min"] == 30.0
    assert params["area1_forward_max"] == 35.0
    assert params["area1_right_min"] == -4.0
    assert params["area1_right_max"] == 4.0
    assert params["area2_forward_min"] == 55.0
    assert params["area2_forward_max"] == 60.0
    assert params["area2_right_min"] == -4.0
    assert params["area2_right_max"] == 4.0


def test_zone_centers_come_from_bounds_at_fifty_degrees():
    frame = LayoutFrame(50.0)
    assert frame.bounds_center_ned(30, 35, -4, 4) == pytest.approx(
        frame.local_to_ned(32.5, 0.0)
    )
    assert frame.bounds_center_ned(55, 60, -4, 4) == pytest.approx(
        frame.local_to_ned(57.5, 0.0)
    )
```

Also AST-check that `MissionFSM` declares `layout_heading_deg`, rejects later writes to that
parameter, and declares no legacy key.

- [ ] **Step 2: Run the tests and verify RED**

Run: `python3 -m pytest src/drone_mission/test/test_layout_mission_config.py src/drone_mission/test/test_full_real_recovery_contract.py -q`

Expected: failures list old parameter keys and missing `layout_heading_deg`.

- [ ] **Step 3: Implement fail-closed layout initialization and config migration**

Declare exact defaults:

```python
self.declare_parameter("layout_heading_deg", 0.0)
self.declare_parameter("area1_forward_min", 30.0)
self.declare_parameter("area1_forward_max", 35.0)
self.declare_parameter("area1_right_min", -4.0)
self.declare_parameter("area1_right_max", 4.0)
self.declare_parameter("area2_forward_min", 55.0)
self.declare_parameter("area2_forward_max", 60.0)
self.declare_parameter("area2_right_min", -4.0)
self.declare_parameter("area2_right_max", 4.0)
self.declare_parameter("sweep_margin_forward", 1.0)
self.declare_parameter("sweep_margin_right", 0.5)
```

Immediately after `_declare_params()` in `__init__`, snapshot and validate the launch-time
heading so even a directly injected bridge receives the configured yaw before `_lazy_init()`:

```python
self._layout_frame = LayoutFrame(float(self.get_param("layout_heading_deg", 0.0)))
```

Treat this frozen object as the mission snapshot: do not re-read or mutate the heading during
the run. Register an `add_on_set_parameters_callback` that returns an unsuccessful
`rcl_interfaces.msg.SetParametersResult` whenever a runtime update contains
`layout_heading_deg`, while allowing unrelated parameter updates. At the start of
`_lazy_init()`, before creating handlers or transitioning to INIT, derive all geometry from
that same object:

```python
drop_bounds = self.zone_bounds("area1")
recon_bounds = self.zone_bounds("area2")
drop_north, drop_east = self._layout_frame.bounds_center_ned(*drop_bounds)
recon_north, recon_east = self._layout_frame.bounds_center_ned(*recon_bounds)
drop_world_x, drop_world_y, _ = ned_to_world(drop_north, drop_east, 0.0)
recon_world_x, recon_world_y, _ = ned_to_world(recon_north, recon_east, 0.0)
```

Expose:

```python
@property
def layout_frame(self) -> LayoutFrame:
    return self._layout_frame

@property
def layout_yaw_rad(self) -> float:
    return self._layout_frame.yaw_rad

def zone_bounds(self, prefix: str) -> tuple[float, float, float, float]:
    return (
        float(self.get_param(f"{prefix}_forward_min")),
        float(self.get_param(f"{prefix}_forward_max")),
        float(self.get_param(f"{prefix}_right_min")),
        float(self.get_param(f"{prefix}_right_max")),
    )
```

Construct both direct and standalone `FlightController` instances with
`task_yaw=self.layout_yaw_rad`. The direct bridge setter and lazy standalone controller both
use the already initialized `_layout_frame.yaw_rad`; neither may capture aircraft telemetry
yaw. Validate
every bounds value is finite and min < max; raise `ValueError` before Arm on failure. Add
heading, Home, and both NED centers to startup logs. Migrate all four YAML profiles
mechanically; do not change any unrelated quick/main value. Task 12 adds the persisted
summary fields after the mission behavior is stable.

- [ ] **Step 4: Run mission profile regressions**

Run: `python3 -m pytest src/drone_mission/test/test_layout_mission_config.py src/drone_mission/test/test_quick_drop_strategy.py src/drone_mission/test/test_full_real_recovery_contract.py -q`

Expected: all tests pass and quick/main strategy-specific values remain unchanged.

- [ ] **Step 5: Commit the mission configuration migration**

```bash
git add src/drone_mission/drone_mission/mission_state_machine.py src/drone_mission/config src/drone_mission/test/test_layout_mission_config.py src/drone_mission/test/test_full_real_recovery_contract.py
git commit -m "feat: parameterize mission layout heading"
```

---
### Task 4: Rotate reconnaissance and drop-search paths in the shared MissionFSM

**Files:**
- Modify: `src/drone_mission/drone_mission/path_planner.py`
- Modify: `src/drone_mission/drone_mission/states/state_recon_patrol.py`
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_search.py`
- Modify: `src/drone_mission/drone_mission/states/state_transit.py`
- Modify: `src/drone_mission/drone_mission/states/state_return.py`
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_approach.py`
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Create: `src/drone_mission/test/test_layout_paths.py`
- Modify: `src/px4_control/test/test_mission_setpoint_contract.py`
- Modify: `src/drone_mission/test/test_quick_drop_strategy.py`
- Modify: `src/drone_mission/test/test_recon_coverage_lifecycle.py`

**Interfaces:**
- Consumes: `MissionFSM.layout_frame`, `zone_bounds()` and task-yaw default
- Produces: `PathPlanner(..., layout_frame: LayoutFrame)`
- Produces: `plan_coverage_ned(area_forward_min, area_forward_max, area_right_min, area_right_max, altitude)`
- Produces: exact local-frame drop-zone validation

- [ ] **Step 1: Write failing 0°/50° path and search tests**

```python
def inverse_local_points(waypoints, frame):
    return [frame.ned_to_local(wp["north"], wp["east"]) for wp in waypoints]


def test_zero_degree_recon_path_is_the_existing_quick_path():
    planner = PathPlanner(layout_frame=LayoutFrame(0.0), grid_spacing=1.67)
    path = planner.plan_coverage_ned(55.0, 60.0, -4.0, 4.0, altitude=-2.5)
    assert [(wp["north"], wp["east"]) for wp in path] == [
        (55.0, -4.0), (55.0, 4.0),
        (57.5, 4.0), (57.5, -4.0),
        (60.0, -4.0), (60.0, 4.0),
    ]
    assert {wp["yaw"] for wp in path} == {0.0}


def test_fifty_degree_path_is_a_rigid_rotation_with_constant_yaw():
    base = PathPlanner(layout_frame=LayoutFrame(0.0), grid_spacing=1.67)
    rotated = PathPlanner(layout_frame=LayoutFrame(50.0), grid_spacing=1.67)
    path0 = base.plan_coverage_ned(55.0, 60.0, -4.0, 4.0, altitude=-2.5)
    path50 = rotated.plan_coverage_ned(55.0, 60.0, -4.0, 4.0, altitude=-2.5)
    assert inverse_local_points(path50, LayoutFrame(50.0)) == pytest.approx(
        [(wp["north"], wp["east"]) for wp in path0]
    )
    assert all(wp["yaw"] == pytest.approx(LayoutFrame(50.0).yaw_rad) for wp in path50)
    assert path_length(path50) == pytest.approx(path_length(path0), abs=1e-9)


def test_drop_scan_endpoints_are_local_right_edges_at_fifty_degrees():
    state, fsm = make_search_state(heading_deg=50.0, quick_drop_mode=True)
    state._start_scan()
    local = [fsm.layout_frame.ned_to_local(wp["north"], wp["east"])
             for wp in state._scan_waypoints]
    assert local == pytest.approx([(32.5, -4.0), (32.5, 4.0)])
```

Add exact containment cases showing an AABB-only corner is rejected after World→NED→local
conversion, and retain the existing quick marker-only state assertions.

- [ ] **Step 2: Run the path tests and verify RED**

Run: `python3 -m pytest src/drone_mission/test/test_layout_paths.py src/px4_control/test/test_mission_setpoint_contract.py::MissionSetpointContractTest::test_recon_coverage_waypoints_keep_a_fixed_heading -q`

Expected: failures show old north/east signatures and yaw0 at 50°.

- [ ] **Step 3: Generate locally, rotate once, and remove state-level yaw0**

Update the planner constructor to accept `layout_frame` and rename margins to
`margin_forward`/`margin_right`. Generate the exact current boustrophedon sequence in
forward/right coordinates, then emit each point through:

```python
def waypoint(forward: float, right: float, label: str) -> dict:
    north, east = self.layout_frame.local_to_ned(forward, right)
    return {
        "north": north,
        "east": east,
        "down": altitude,
        "yaw": self.layout_frame.yaw_rad,
        "speed": self.sweep_speed,
        "hover_s": 0.0,
        "label": label,
}
```

The current planner stores sweep margins but does not apply them to strip endpoints. Preserve
that exact 0° behavior here: rename the margins for frame correctness, but do not start using
them as an inset. Activating them later requires separate tuning and acceptance evidence.

`ReconPatrolState` passes `area2_forward/right_*`; `CylinderSearchState` builds its center
and two scan endpoints locally and rotates them. Remove explicit `yaw=0.0` from transit and
return calls so the controller invariant owns yaw. Delete the unused
`CylinderApproachState._goto_pixel_centered_target()` north-assuming helper after confirming
with `rg` that it has no caller.

Replace `_drop_zone_world_bounds()` and dual-frame `_normalize_drop_target_xy()` with:

```python
def validate_drop_target_world_xy(
    self, world_x: float, world_y: float, margin: float = 1.0,
) -> Optional[tuple[float, float]]:
    north, east, _ = world_to_ned(world_x, world_y, 0.0)
    if self.layout_frame.contains_ned(
        north, east, *self.zone_bounds("area1"), margin=margin,
    ):
        return world_x, world_y
    self.get_logger().warn(
        f"CYL_SEARCH: discard out-of-zone world target ({world_x:.2f}, {world_y:.2f})"
    )
    return None
```

Update all three drop-zone safety callers to use this exact method and preserve their
existing margin values.

- [ ] **Step 4: Run quick-first mission path regressions**

Run: `python3 -m pytest src/drone_mission/test/test_layout_paths.py src/drone_mission/test/test_quick_drop_strategy.py src/drone_mission/test/test_recon_coverage_lifecycle.py src/px4_control/test/test_mission_setpoint_contract.py -q`

Expected: all tests pass; quick marker selection and main full-path coverage semantics are unchanged.

- [ ] **Step 5: Commit shared rotated paths**

```bash
git add src/drone_mission/drone_mission src/drone_mission/test src/px4_control/test/test_mission_setpoint_contract.py
git commit -m "feat: rotate shared mission paths by layout heading"
```

---
### Task 5: Replace ambiguous cylinder coordinates with an explicit World contract

**Files:**
- Modify: `src/drone_interfaces/msg/CylinderDetection.msg`
- Modify: `src/drone_vision/drone_vision/vision_pipeline.py`
- Modify: `src/drone_vision/drone_vision/cylinder_detector.py`
- Modify: `src/drone_vision/scripts/vision_viewer.py`
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Modify: `src/drone_mission/drone_mission/cylinder_selector.py`
- Modify: `src/drone_mission/drone_mission/drop_visual_aligner.py`
- Modify: `src/drone_mission/drone_mission/precision_align_controller.py`
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_approach.py`
- Modify: all mission tests that construct target dictionaries
- Create: `src/drone_vision/test/test_cylinder_world_contract.py`

**Interfaces:**
- Produces: `bool world_position_valid`, `float64 world_x`, `float64 world_y`
- Removes: `CylinderDetection.center_x`, `CylinderDetection.center_y`
- Internal mission target dictionaries use `world_x`/`world_y` exclusively

- [ ] **Step 1: Add a failing source and consumer contract**

```python
def test_cylinder_message_has_one_explicit_world_coordinate_contract():
    fields = [line.split("#", 1)[0].strip()
              for line in MESSAGE.read_text(encoding="utf-8").splitlines()]
    assert "bool world_position_valid" in fields
    assert "float64 world_x" in fields
    assert "float64 world_y" in fields
    assert not any(line.startswith("float64 center_x") for line in fields)
    assert not any(line.startswith("float64 center_y") for line in fields)


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
```

Add a repository scan assertion limited to cylinder target consumers so obsolete
`center_x/center_y` cannot return unnoticed.

- [ ] **Step 2: Run the contract and verify RED**

Run: `python3 -m pytest src/drone_vision/test/test_cylinder_world_contract.py src/drone_mission/test/test_quick_drop_strategy.py -q`

Expected: message fields and mission consumers still use ambiguous names.

- [ ] **Step 3: Migrate the message and every producer/consumer in one commit**

Use this exact message prefix:

```text
int32 id
bool world_position_valid
float64 world_x
float64 world_y
float64 center_u
float64 center_v
float64 radius
float64 confidence
```

Keep existing bbox fields unchanged. Producers set invalid coordinates as:

```python
det.world_position_valid = False
det.world_x = math.nan
det.world_y = math.nan
```

and only set the flag true after a valid World projection. `CylinderArray.header.frame_id`
remains `"world"`. Migrate mission memory, selector, aligner, approach, and all test fixtures
to `world_x/world_y`; remove the old World/NED guessing branch entirely. The standalone
`cylinder_detector.py`, which lacks pose, publishes pixel data with invalid World position
rather than placeholder NED coordinates.

- [ ] **Step 4: Build the interface and run all target-consumer tests**

Run: `colcon build --packages-select drone_interfaces --symlink-install`

Expected: `drone_interfaces` builds successfully.

Run: `source install/setup.bash && python3 -m pytest src/drone_vision/test/test_cylinder_world_contract.py src/drone_mission/test/test_quick_drop_strategy.py src/drone_mission/test/test_precision_align_controller.py -q`

Expected: all tests pass with generated `world_position_valid/world_x/world_y` fields.

- [ ] **Step 5: Commit the coordinate-message migration**

```bash
git add src/drone_interfaces/msg/CylinderDetection.msg src/drone_vision src/drone_mission
git commit -m "refactor: make cylinder world coordinates explicit"
```

---
### Task 6: Require fresh 6-DoF pose for visual World coordinates

**Files:**
- Modify: `src/drone_vision/drone_vision/vision_pipeline.py`
- Modify: `src/drone_vision/config/vision_params.yaml`
- Create: `src/drone_vision/test/test_vision_pose_validity.py`
- Modify: `src/drone_vision/test/test_coordinate_transforms.py`

**Interfaces:**
- Produces: parameter `vision_pose_max_age_s=0.5`
- Produces: `_pose_is_fresh(now_monotonic: float) -> bool`
- Produces: `_project_world_xy(u: float, v: float) -> Optional[tuple[float, float]]`

- [ ] **Step 1: Write failing freshness and no-fallback tests**

```python
def test_pose_is_unavailable_until_first_pose_callback():
    node = bare_pipeline(last_pose=None, pose_valid=False)
    assert not node._pose_is_fresh(now_monotonic=10.0)


def test_pose_older_than_half_second_is_rejected():
    node = bare_pipeline(last_pose=9.49, pose_valid=True, max_age=0.5)
    assert not node._pose_is_fresh(now_monotonic=10.0)


def test_fresh_fifty_degree_pose_uses_full_projection():
    node, projector = pipeline_with_fresh_pose(yaw_ned=math.radians(50.0))
    projector.result = (28.0, 31.0)
    assert node._project_world_xy(640.0, 480.0) == (28.0, 31.0)
    assert projector.calls == 1


def test_invalid_full_projection_does_not_call_simple_projection(monkeypatch):
    node, projector = pipeline_with_fresh_pose(yaw_ned=math.radians(50.0))
    projector.result = None
    monkeypatch.setattr(
        CoordinateTransformer, "pixel_to_world_simple",
        lambda *_args, **_kwargs: pytest.fail("north fallback must not run"),
    )
    assert node._project_world_xy(640.0, 480.0) is None
```

- [ ] **Step 2: Run the test and verify RED**

Run: `python3 -m pytest src/drone_vision/test/test_vision_pose_validity.py -q`

Expected: missing freshness helpers and existing simple fallback calls fail the tests.

- [ ] **Step 3: Centralize fresh projection and use it for markers and cylinders**

Initialize `_pose_received=False` and `_last_pose_monotonic=None`; after validating finite
position and a finite nonzero-norm quaternion in `_pose_cb`, store the pose and set:

```python
self._pose_received = True
self._last_pose_monotonic = time.monotonic()
```

Implement:

```python
def _pose_is_fresh(self, now_monotonic: Optional[float] = None) -> bool:
    if not self._pose_received or self._last_pose_monotonic is None:
        return False
    now = time.monotonic() if now_monotonic is None else now_monotonic
    max_age = float(self.get_parameter("vision_pose_max_age_s").value)
    return 0.0 <= now - self._last_pose_monotonic <= max_age

def _project_world_xy(self, u: float, v: float) -> Optional[tuple[float, float]]:
    if not self._pose_is_fresh() or self._coord_xform is None:
        return None
    result = self._coord_xform.pixel_to_world(
        u, v, self._drone_pose[:3], self._drone_pose[3:7],
        ground_z=0.0, q_convention="xyzw",
    )
    if result is None:
        return None
    offset = math.hypot(result[0] - self._drone_pose[0], result[1] - self._drone_pose[1])
    return result if offset <= max(self._current_altitude * 1.5, 3.0) else None
```

Both marker and cylinder publishing call this helper, set `world_position_valid`, and never
call `pixel_to_world_simple`. Keep the pure helper only if another tested debug API consumes
it; otherwise delete it and its north-only tests.

- [ ] **Step 4: Run vision and quick-target regressions**

Run: `python3 -m pytest src/drone_vision/test/test_vision_pose_validity.py src/drone_vision/test/test_coordinate_transforms.py src/drone_vision/test/test_vision_modes.py src/drone_mission/test/test_quick_drop_strategy.py -q`

Expected: all tests pass; marker-only mode still publishes pixel detections while invalid
World coordinates are excluded from mission memory.

- [ ] **Step 5: Commit strict visual projection**

```bash
git add src/drone_vision/drone_vision/vision_pipeline.py src/drone_vision/config/vision_params.yaml src/drone_vision/test
git commit -m "fix: require fresh pose for visual world targets"
```

---
### Task 7: Track reconnaissance coverage in the rotated local frame

**Files:**
- Modify: `src/drone_vision/drone_vision/coverage_monitor.py`
- Modify: `src/drone_vision/drone_vision/vision_pipeline.py`
- Modify: `src/drone_vision/config/vision_params.yaml`
- Create: `src/drone_vision/test/test_coverage_monitor_layout.py`
- Modify: `src/drone_vision/test/test_coverage_monitor_scan_lifecycle.py`

**Interfaces:**
- Consumes: `LayoutFrame`, Home World XY, local recon bounds
- Produces: `CoverageMonitor(zone_forward_range, zone_right_range, layout_frame, home_world_xy, ...)`
- Rejects: runtime writes to VisionPipeline's snapshotted `layout_heading_deg`

- [ ] **Step 1: Write failing rigid-rotation coverage tests**

```python
def cover_path(monitor, frame, local_points, home=(4.0, 10.0)):
    monitor.begin_scan()
    for forward, right in local_points:
        north, east = frame.local_to_ned(forward, right)
        monitor.update(home[0] + east, home[1] + north, 2.5)
    return monitor.coverage_ratio, monitor.grid.copy()


def test_zero_and_fifty_degree_coverage_are_identical():
    points = [(55.0, -4.0), (55.0, 4.0), (57.5, 4.0),
              (57.5, -4.0), (60.0, -4.0), (60.0, 4.0)]
    zero = CoverageMonitor((55, 60), (-4, 4), LayoutFrame(0), (4, 10))
    fifty = CoverageMonitor((55, 60), (-4, 4), LayoutFrame(50), (4, 10))
    ratio0, grid0 = cover_path(zero, LayoutFrame(0), points)
    ratio50, grid50 = cover_path(fifty, LayoutFrame(50), points)
    assert ratio50 == pytest.approx(ratio0)
    assert np.array_equal(grid50, grid0)
```

- [ ] **Step 2: Run the test and verify RED**

Run: `python3 -m pytest src/drone_vision/test/test_coverage_monitor_layout.py -q`

Expected: the old World X/Y constructor cannot represent the rotated local zone.

- [ ] **Step 3: Convert World pose to local before grid indexing**

Use constructor fields `zone_forward_min/max`, `zone_right_min/max`, `layout_frame`, and
`home_world_x/y`. At the top of `update()`:

```python
north = drone_y - self.home_world_y
east = drone_x - self.home_world_x
forward, right = self.layout_frame.ned_to_local(north, east)
```

Run the existing circular-footprint algorithm on forward/right indices. Rename vision
parameters to `coverage_zone_forward_min/max` and `coverage_zone_right_min/max`; add
`home_x`, `home_y`, and `layout_heading_deg`. `VisionPipeline` constructs one
`LayoutFrame` from the launch value and passes it into its `CoverageMonitor`. Register a
parameter callback that rejects runtime changes to `layout_heading_deg`, matching MissionFSM,
so the two snapshots cannot silently diverge during a mission.

- [ ] **Step 4: Run coverage lifecycle and vision tests**

Run: `python3 -m pytest src/drone_vision/test/test_coverage_monitor_layout.py src/drone_vision/test/test_coverage_monitor_scan_lifecycle.py src/drone_mission/test/test_recon_coverage_lifecycle.py -q`

Expected: all tests pass and begin/reset/end scan semantics are unchanged.

- [ ] **Step 5: Commit rotated coverage**

```bash
git add src/drone_vision/drone_vision/coverage_monitor.py src/drone_vision/drone_vision/vision_pipeline.py src/drone_vision/config/vision_params.yaml src/drone_vision/test
git commit -m "feat: rotate reconnaissance coverage grid"
```

---
### Task 8: Propagate one heading through all four launch profiles

**Files:**
- Modify: `src/drone_bringup/launch/quick_sitl.launch.py`
- Modify: `src/drone_bringup/launch/sitl.launch.py`
- Modify: `src/drone_bringup/launch/quick_flight.launch.py`
- Modify: `src/drone_bringup/launch/real_flight.launch.py`
- Modify: `src/drone_bringup/test/test_quick_launch_configuration.py`
- Modify: `src/drone_bringup/test/test_real_full_mission_profile.py`
- Modify: `src/drone_bringup/test/test_real_mission_scripts.py`
- Create: `src/drone_bringup/test/test_layout_heading_launch_contract.py`

**Interfaces:**
- Produces: one launch argument `layout_heading_deg`, default `0.0`
- Passes: heading to MissionFSM and VisionPipeline in every profile
- Preserves: both real-flight wrapper scripts forwarding arbitrary launch arguments through
  their existing `LAUNCH_ARGS` arrays
- Removes: launch arguments `flight_axis`, `drop_distance`, `recon_distance`

- [ ] **Step 1: Add failing four-profile launch tests**

```python
@pytest.mark.parametrize("launch_name", [
    "quick_sitl.launch.py", "sitl.launch.py",
    "quick_flight.launch.py", "real_flight.launch.py",
])
def test_every_complete_profile_has_one_layout_heading_and_no_legacy_direction(launch_name):
    source = (LAUNCH_DIR / launch_name).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n        "layout_heading_deg"' in source
    assert source.count('"layout_heading_deg": LaunchConfiguration("layout_heading_deg")') == 2
    assert "flight_axis" not in source
    assert "drop_distance" not in source
    assert "recon_distance" not in source


@pytest.mark.parametrize("script", [QUICK_REAL_SCRIPT, FULL_REAL_SCRIPT])
def test_real_wrapper_forwards_layout_heading_as_a_launch_argument(script):
    source = script.read_text(encoding="utf-8")
    assert 'LAUNCH_ARGS+=("$arg")' in source
    assert '"${LAUNCH_ARGS[@]}"' in source
```

Keep existing tests that prove quick launch loads quick mission/vision/servo files and main
launch loads main files.

- [ ] **Step 2: Run deployment tests and verify RED**

Run: `python3 -m pytest src/drone_bringup/test/test_layout_heading_launch_contract.py src/drone_bringup/test/test_quick_launch_configuration.py src/drone_bringup/test/test_real_full_mission_profile.py -q`

Expected: all four launch files still expose legacy direction arguments.

- [ ] **Step 3: Replace the three legacy arguments with one heading argument**

Each launch contains:

```python
layout_heading_arg = DeclareLaunchArgument(
    "layout_heading_deg",
    default_value="0.0",
    description="场地前方绝对方位角: 0=正北, 顺时针为正 (deg)",
)
```

Pass `home_x`, `home_y`, and `layout_heading_deg` to VisionPipeline; pass
`layout_heading_deg` to MissionFSM. Do not pass heading to PX4Bridge because it receives
the yaw through mission setpoints. Remove hardcoded real-flight World coverage bounds; the
vision node now consumes local coverage bounds from its config and Home/heading from launch.

- [ ] **Step 4: Run all bringup contracts**

Run: `python3 -m pytest src/drone_bringup/test -q`

Expected: all tests pass; quick and main profile-selection tests remain green.

- [ ] **Step 5: Commit four-profile propagation**

```bash
git add src/drone_bringup/launch src/drone_bringup/test
git commit -m "feat: propagate layout heading through all profiles"
```

---
### Task 9: Generate a rigidly rotated SITL field around Home

**Files:**
- Create: `field/layout_field.py`
- Modify: `field/generate_random_field.py`
- Create: `src/drone_bringup/test/test_layout_field.py`
- Modify: `field/README.md`

**Interfaces:**
- Produces: `rotate_layout_sdf(input_path, output_path, heading_deg, home_x=4.0, home_y=10.0)`
- Produces CLI: `python3 field/layout_field.py INPUT OUTPUT --heading-deg 50 --home-x 4 --home-y 10`
- Produces CLI option: `--print-gazebo-yaw-rad`, printing only the ENU yaw delta to stdout
- Preserves: canonical `field/drone_field.sdf`

- [ ] **Step 1: Write failing SDF rigid-transform tests**

```python
MISSION_MODELS = {
    "field_base", "field_border", "zone_preparation", "zone_takeoff_landing",
    "landing_pad", "h_marking", "takeoff_line", "zone_drop",
    "effective_zone_b", "drop_cylinders", "zone_reconnaissance",
    "recon_cylinders", "hazard_plates", "corner_markers",
    "mission_corner_markers",
}


def test_fifty_degree_field_rotates_every_mission_asset_about_home(tmp_path):
    output = tmp_path / "field_50.sdf"
    rotate_layout_sdf(FIELD, output, 50.0, home_x=4.0, home_y=10.0)
    source_poses = mission_world_poses(FIELD)
    rotated_poses = mission_world_poses(output)
    for key, source_pose in source_poses.items():
        expected_x, expected_y = rotate_world_xy_about_home(
            source_pose.x, source_pose.y, 50.0, 4.0, 10.0,
        )
        assert rotated_poses[key].x == pytest.approx(expected_x, abs=1e-6)
        assert rotated_poses[key].y == pytest.approx(expected_y, abs=1e-6)
        assert angle_delta(rotated_poses[key].yaw, source_pose.yaw - math.radians(50)) == pytest.approx(0.0)


def test_north_point_rotates_toward_east_for_positive_ned_heading():
    x, y = rotate_world_xy_about_home(4.0, 42.5, 50.0, 4.0, 10.0)
    assert x == pytest.approx(4.0 + 32.5 * math.sin(math.radians(50.0)))
    assert y == pytest.approx(10.0 + 32.5 * math.cos(math.radians(50.0)))


def test_gazebo_yaw_delta_has_opposite_sign_to_ned_heading():
    assert gazebo_yaw_rad(0.0) == pytest.approx(0.0)
    assert gazebo_yaw_rad(50.0) == pytest.approx(-math.radians(50.0))


def test_environment_models_and_canonical_file_are_unchanged(tmp_path):
    before = FIELD.read_bytes()
    output = tmp_path / "field_50.sdf"
    rotate_layout_sdf(FIELD, output, 50.0)
    assert FIELD.read_bytes() == before
    assert model_semantics(output, "ground_plane") == model_semantics(FIELD, "ground_plane")
```

Add a fixed-seed random-field test proving the current generator samples drop objects around
World `(4.0, 42.5)` and recon objects around `(4.0, 67.5)`, then inverse-rotation preserves
the same local samples.

- [ ] **Step 2: Run field tests and verify RED**

Run: `python3 -m pytest src/drone_bringup/test/test_layout_field.py -q`

Expected: import fails because `field/layout_field.py` does not exist and the random X/Y
centers are reversed.

- [ ] **Step 3: Implement XML-aware mission-asset rotation**

Use `xml.etree.ElementTree`, an explicit `MISSION_MODELS` set, and this transform:

```python
def rotate_pose_text(text: str, heading_rad: float, home_x: float, home_y: float) -> str:
    values = [float(value) for value in text.split()]
    if len(values) != 6:
        raise ValueError(f"expected six-value SDF pose, got: {text!r}")
    x, y, z, roll, pitch, yaw = values
    dx, dy = x - home_x, y - home_y
    cos_h, sin_h = math.cos(heading_rad), math.sin(heading_rad)
    # NED heading is clockwise from north; Gazebo World yaw is CCW in ENU.
    rotated_x = home_x + dx * cos_h + dy * sin_h
    rotated_y = home_y - dx * sin_h + dy * cos_h
    return f"{rotated_x:.9f} {rotated_y:.9f} {z:.9f} {roll:.9f} {pitch:.9f} {yaw - heading_rad:.9f}"
```

For a selected top-level model with its own `<pose>`, rotate that pose only because link
poses are relative. For a selected model without a model pose, rotate every direct
`link/pose`; do not rotate nested visual/collision poses. Exclude environment objects by
name. Parse, transform, and write only the requested output path.

Correct random zones to World `(x=east, y=north)`:

```python
DROP_ZONE["center"] = (4.0, 42.5)
DROP_ZONE["half_size"] = (4.0, 2.5)
DROP_ZONE["sigma"] = (1.6, 1.0)
RECON_ZONE["center"] = (4.0, 67.5)
RECON_ZONE["half_size"] = (4.0, 2.5)
RECON_ZONE["sigma"] = (1.6, 1.0)
```

Add `--seed` and seed both `random` and NumPy. Add optional `--heading-deg`, applying
`rotate_layout_sdf()` after random object replacement.

- [ ] **Step 4: Run field and XML regressions**

Run: `python3 -m pytest src/drone_bringup/test/test_layout_field.py -q`

Expected: all pose, environment, canonical-file, and fixed-seed invariance tests pass.

- [ ] **Step 5: Commit field generation**

```bash
git add field/layout_field.py field/generate_random_field.py field/README.md src/drone_bringup/test/test_layout_field.py
git commit -m "feat: generate rotated SITL mission fields"
```

---
### Task 10: Make quick and main simulation wrappers own the same runtime heading

**Files:**
- Modify: `scripts/start_quick_simulation.sh`
- Modify: `scripts/start_full_simulation.sh`
- Modify: `src/drone_bringup/test/test_simulation_script.py`
- Create: `src/drone_bringup/test/test_quick_simulation_script.py`

**Interfaces:**
- Consumes: launch argument `layout_heading_deg:=VALUE`
- Produces: runtime SDF inside the run log directory
- Produces: matching Gazebo model yaw and ROS launch parameter

- [ ] **Step 1: Add failing wrapper source and temporary-field tests**

```python
@pytest.mark.parametrize("script", [QUICK_SCRIPT, FULL_SCRIPT])
def test_simulation_wrapper_resolves_one_heading_for_field_model_and_launch(script):
    source = script.read_text(encoding="utf-8")
    assert 'layout_heading_deg:=*' in source
    assert 'field/layout_field.py' in source
    assert 'PX4_GZ_MODEL_POSE="4,10,0.3,0,0,$GAZEBO_YAW_RAD"' in source
    assert 'ros2 launch' in source and '"$@"' in source
    assert 'cp "$PROJ/field/drone_field.sdf" "$PX4_WORLD"' not in source
```

- [ ] **Step 2: Run wrapper tests and verify RED**

Run: `python3 -m pytest src/drone_bringup/test/test_simulation_script.py src/drone_bringup/test/test_quick_simulation_script.py -q`

Expected: both wrappers still copy the north template and spawn yaw0.

- [ ] **Step 3: Resolve, validate, generate, and log the per-run heading**

In both scripts, before any PX4 process starts:

```bash
LAYOUT_HEADING_DEG="0.0"
for task_arg in "$@"; do
    case "$task_arg" in
        layout_heading_deg:=*)
            LAYOUT_HEADING_DEG="${task_arg#layout_heading_deg:=}"
            ;;
    esac
done

RUNTIME_FIELD="$LOG_DIR/drone_field.sdf"
GAZEBO_YAW_RAD="$(python3 "$PROJ/field/layout_field.py" \
    "$PROJ/field/drone_field.sdf" "$RUNTIME_FIELD" \
    --heading-deg "$LAYOUT_HEADING_DEG" --home-x 4.0 --home-y 10.0 \
    --print-gazebo-yaw-rad)"
cp "$RUNTIME_FIELD" "$PX4_WORLD"
```

`layout_field.py --print-gazebo-yaw-rad` must print only the normalized ENU radian delta
`-heading` to stdout and send diagnostics to stderr. Use
`PX4_GZ_MODEL_POSE="4,10,0.3,0,0,$GAZEBO_YAW_RAD"`; at heading0 this remains the current
raw Gazebo yaw0 baseline. The ROS/PX4 task yaw is still the positive NED heading passed via
launch.
Keep passing the original `"$@"` to quick/main ROS launch so the same value reaches mission
and vision. Update the startup banner and save heading/Home/runtime-field path in the run log.

- [ ] **Step 4: Run all simulation wrapper tests**

Run: `python3 -m pytest src/drone_bringup/test/test_simulation_script.py src/drone_bringup/test/test_quick_simulation_script.py src/drone_bringup/test/test_layout_field.py -q`

Expected: all tests pass without launching Gazebo.

- [ ] **Step 5: Commit wrapper integration**

```bash
git add scripts/start_quick_simulation.sh scripts/start_full_simulation.sh src/drone_bringup/test
git commit -m "feat: launch SITL with one layout heading"
```

---
### Task 11: Run quick-first package regression and update operator documentation

**Files:**
- Modify: `docs/ARCHITECTURE.md`
- Modify: `docs/CODE_READING_GUIDE.md`
- Modify: `docs/Debug/PARAMETER_USAGE_GUIDE.md`
- Modify: `src/drone_bringup/test/test_quick_launch_configuration.py`
- Modify: `src/drone_bringup/test/test_real_full_mission_profile.py`
- Modify: any lint tests whose exact documented parameter list changed

**Interfaces:**
- Documents: one command for default0 and one command for50° in quick/main SITL
- Locks: quick strategy values and main strategy values against accidental cross-profile changes

- [ ] **Step 1: Add failing documentation/legacy-name scans**

```python
def test_active_docs_have_no_legacy_direction_parameters():
    active_docs = [ARCHITECTURE, READING_GUIDE, PARAMETER_GUIDE, FIELD_README]
    for path in active_docs:
        text = path.read_text(encoding="utf-8")
        assert "flight_axis" not in text
        assert "area1_north_min" not in text
        assert "coverage_zone_x_min" not in text


def test_quick_profile_keeps_authoritative_behavior_values():
    quick = load_params("mission_params_quick_sim.yaml")
    assert quick["quick_drop_mode"] is True
    assert quick["drop_target_source"] == "marker"
    assert quick["landing_simple_mode"] is True
    assert quick["transit_speed"] == 3.5
    assert quick["return_speed"] == 4.5
```

- [ ] **Step 2: Run the scans and verify RED**

Run: `python3 -m pytest src/drone_bringup/test/test_quick_launch_configuration.py src/drone_bringup/test/test_real_full_mission_profile.py -q`

Expected: active docs and tests still reference legacy direction names.

- [ ] **Step 3: Document the exact operator workflow and coordinate contract**

Document these commands verbatim:

```bash
# Quick authoritative baseline, north
./scripts/start_quick_simulation.sh layout_heading_deg:=0

# Quick authoritative 50-degree validation
./scripts/start_quick_simulation.sh layout_heading_deg:=50

# Main-strategy 50-degree regression
./scripts/start_full_simulation.sh layout_heading_deg:=50

# Real-flight quick/main entry points use the same parameter
./scripts/start_quick_mission_real.sh layout_heading_deg:=50
./scripts/start_full_real_mission.sh layout_heading_deg:=50
```

Explain forward/right region values, clockwise NED heading, climb-and-turn behavior, no yaw
gate, World target validity, and the fact that real operators must physically confirm field
orientation. Preserve all quick/main strategy-specific tuning documentation.

- [ ] **Step 4: Build and run quick-first full package tests**

Run: `colcon build --packages-select drone_utils drone_interfaces px4_control drone_vision drone_mission drone_bringup --symlink-install`

Expected: selected packages build successfully.

Run: `source install/setup.bash && python3 -m pytest src/drone_utils/test src/px4_control/test src/drone_vision/test src/drone_mission/test src/drone_bringup/test -q`

Expected: all tests pass. If a pre-existing unrelated test fails, record the exact test and
prove it also fails on commit `0541efd`; do not weaken a direction or quick-strategy test.

- [ ] **Step 5: Commit docs and complete static regression**

```bash
git add docs src/drone_bringup/test
git commit -m "docs: document arbitrary layout operation"
```

---
### Task 12: Validate quick 0°/50° efficiency, then main-strategy regressions

**Files:**
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Create: `scripts/summarize_layout_benchmark.py`
- Create: `src/drone_mission/test/test_layout_benchmark_summary.py`
- Runtime evidence only: `/tmp/drone_mission_logs/<date>/mission_*/summary.json`

**Interfaces:**
- Summary fields: `layout_heading_deg`, `strategy`, `total_duration_s`, `success`
- CLI: `python3 scripts/summarize_layout_benchmark.py SUMMARY...`
- Exit 0 only when each quick heading has at least three successes and ratio <= 1.05

- [ ] **Step 1: Write failing summary-analyzer tests**

```python
def test_quick_benchmark_accepts_five_percent_or_better(tmp_path):
    paths = write_summaries(tmp_path, {
        0.0: [100.0, 101.0, 99.0],
        50.0: [103.0, 104.0, 102.0],
    }, strategy="quick")
    result = analyze(paths)
    assert result.baseline_median_s == 100.0
    assert result.rotated_median_s == 103.0
    assert result.ratio == pytest.approx(1.03)
    assert result.passed


def test_quick_benchmark_rejects_more_than_five_percent(tmp_path):
    paths = write_summaries(tmp_path, {
        0.0: [100.0, 100.0, 100.0],
        50.0: [106.0, 106.0, 106.0],
    }, strategy="quick")
    assert not analyze(paths).passed


def test_failed_or_wrong_strategy_runs_do_not_count(tmp_path):
    paths = write_mixed_summaries(tmp_path)
    with pytest.raises(ValueError, match="three successful quick runs"):
        analyze(paths)
```

- [ ] **Step 2: Run analyzer tests and verify RED**

Run: `python3 -m pytest src/drone_mission/test/test_layout_benchmark_summary.py -q`

Expected: analyzer module and summary fields do not exist.

- [ ] **Step 3: Add reproducible summary fields and median analysis**

In `_handle_complete()` add:

```python
"layout_heading_deg": self.layout_frame.heading_deg,
"strategy": "quick" if self.get_param("quick_drop_mode", False) else "main",
```

The analyzer loads every path, keeps only `success=true` and `strategy="quick"`, groups exact
normalized headings0 and50, requires at least three per group, uses `statistics.median`,
prints both medians and `ratio=rotated/baseline`, and exits1 when ratio >1.05.

- [ ] **Step 4: Commit benchmark instrumentation and pass code review**

```bash
git add src/drone_mission/drone_mission/mission_state_machine.py src/drone_mission/test/test_layout_benchmark_summary.py scripts/summarize_layout_benchmark.py
git commit -m "test: enforce quick layout efficiency baseline"
```

The implementer stops here. Complete the Task 12 spec/quality review, then the whole-branch
code review, and fix/re-review every Critical or Important finding before starting Gazebo.

- [ ] **Step 5: After code review, run quick authoritative SITL evidence, then main regressions**

Run three successful missions for each quick heading using the same fixed field and code:

```bash
./scripts/start_quick_simulation.sh layout_heading_deg:=0
./scripts/start_quick_simulation.sh layout_heading_deg:=0
./scripts/start_quick_simulation.sh layout_heading_deg:=0
./scripts/start_quick_simulation.sh layout_heading_deg:=50
./scripts/start_quick_simulation.sh layout_heading_deg:=50
./scripts/start_quick_simulation.sh layout_heading_deg:=50
```

After each `summary.json` appears and reports `success=true`, stop the completed launch cleanly
before starting the next run. Then run:

```bash
python3 scripts/summarize_layout_benchmark.py \
  /tmp/drone_mission_logs/<date>/mission_<0a>/summary.json \
  /tmp/drone_mission_logs/<date>/mission_<0b>/summary.json \
  /tmp/drone_mission_logs/<date>/mission_<0c>/summary.json \
  /tmp/drone_mission_logs/<date>/mission_<50a>/summary.json \
  /tmp/drone_mission_logs/<date>/mission_<50b>/summary.json \
  /tmp/drone_mission_logs/<date>/mission_<50c>/summary.json
```

Expected: exit0, both medians printed, `ratio <= 1.05`, and every 50° log shows no alignment
state and a constant target yaw near0.872665 rad.

Only after quick passes, run:

```bash
./scripts/start_full_simulation.sh layout_heading_deg:=0
./scripts/start_full_simulation.sh layout_heading_deg:=50
```

Expected: both summaries report `strategy="main"`, `success=true`, two completed drops,
recon path complete, and precision landing complete. The main runs are regression gates but
do not enter the quick 5% calculation.

Record the six quick and two main summary paths in the implementation handoff; do not commit
runtime logs or generated SDF files.
