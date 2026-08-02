# Mission Leg Speed Relay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `sweep_speed: 0.8` and the other mission leg speeds reach the PX4 position-setpoint ramp while preserving yaw and existing visual velocity control.

**Architecture:** Replace the lossy `geometry_msgs/Point` position relay with a project-local `NedPositionSetpoint` message. `FlightController` requests an optional per-leg speed, `StandaloneBridge` relays it atomically with NED position and yaw, and `PX4Bridge` limits its existing 20 Hz position ramp by both leg speed and a bridge-owned safety cap.

**Tech Stack:** ROS 2 Humble, `rosidl_generator_interfaces`, Python `rclpy`, PX4 `TrajectorySetpoint`, pytest source/behavior contract tests.

## Global Constraints

- Keep `sweep_speed` at exactly `0.8 m/s` in `mission_params.yaml`.
- Keep `/drone/cmd_velocity_ned` and `/drone/vision/servo_cmd` types and behavior unchanged.
- Do not change PX4 message-version selection, launch argument APIs, or the v1/v4 overlay scripts.
- Position relay publisher and subscriber must be upgraded together; mixed binaries are unsupported.
- `max_setpoint_speed_limit` is bridge policy; phase speeds belong only to mission policy.

---

### Task 1: Define the typed position relay contract

**Files:**
- Create: `src/drone_interfaces/msg/NedPositionSetpoint.msg`
- Modify: `src/drone_interfaces/CMakeLists.txt:14-21`
- Test: `src/px4_control/test/test_mission_setpoint_contract.py`

**Interfaces:**
- Produces: `drone_interfaces.msg.NedPositionSetpoint` with `north`, `east`, `down`, `yaw`, `speed` (`float64`) and `use_speed` (`bool`).
- Consumes: no project-local message input.

- [ ] **Step 1: Write the failing interface-contract test**

```python
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INTERFACE_CMAKE = ROOT / "drone_interfaces/CMakeLists.txt"
MESSAGE = ROOT / "drone_interfaces/msg/NedPositionSetpoint.msg"


def test_ned_position_setpoint_exposes_position_yaw_and_optional_speed():
    assert MESSAGE.read_text(encoding="utf-8").splitlines() == [
        "float64 north",
        "float64 east",
        "float64 down",
        "float64 yaw",
        "float64 speed",
        "bool use_speed",
    ]
    assert "msg/NedPositionSetpoint.msg" in INTERFACE_CMAKE.read_text(encoding="utf-8")
```

- [ ] **Step 2: Run the test and verify it fails because the message is absent**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py::test_ned_position_setpoint_exposes_position_yaw_and_optional_speed`

Expected: failure opening `NedPositionSetpoint.msg`.

- [ ] **Step 3: Add the message and register it with rosidl**

```text
# src/drone_interfaces/msg/NedPositionSetpoint.msg
float64 north
float64 east
float64 down
float64 yaw
float64 speed
bool use_speed
```

Insert `msg/NedPositionSetpoint.msg` in the `msg_files` list in `src/drone_interfaces/CMakeLists.txt`.

- [ ] **Step 4: Run the interface-contract test and verify it passes**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py::test_ned_position_setpoint_exposes_position_yaw_and_optional_speed`

Expected: PASS.

### Task 2: Propagate mission speed and yaw through the position relay

**Files:**
- Modify: `src/px4_control/px4_control/flight_controller.py:124-157,187-190`
- Modify: `src/px4_control/px4_control/standalone_bridge.py:30,116-119,126-131,162-179,303-310`
- Modify: `src/px4_control/px4_control/px4_bridge.py:23,176-179,292-301,477-492`
- Modify: `src/drone_mission/test/test_flight_safety_contract.py:175-210`
- Test: `src/px4_control/test/test_mission_setpoint_contract.py`

**Interfaces:**
- Consumes: `NedPositionSetpoint` from Task 1.
- Produces: `set_position_target(north, east, down, yaw=0.0, speed=None)` on both bridge implementations.
- Produces: position mode state containing target NED, target yaw, optional target speed.

- [ ] **Step 1: Write failing propagation tests**

```python
def test_flight_controller_forwards_speed_and_yaw_to_position_bridge():
    from px4_control.flight_controller import FlightController

    class Bridge:
        def get_parameter(self, _name):
            return type("Parameter", (), {"value": 5.0})()

        def set_position_target(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs

    bridge = Bridge()
    FlightController(bridge).goto_ned(10.0, -2.0, -3.0, yaw=3.14, speed=0.8)
    assert bridge.args == (10.0, -2.0, -3.0)
    assert bridge.kwargs == {"yaw": 3.14, "speed": 0.8}


def test_standalone_bridge_relays_position_yaw_and_speed_atomically():
    from px4_control.standalone_bridge import StandaloneBridge

    bridge = StandaloneBridge.__new__(StandaloneBridge)
    bridge._velocity_mode = False
    bridge._target_ned = None
    bridge._target_yaw = 0.0
    bridge._target_speed = None
    bridge._shutdown = False
    bridge._sp_pub = type("Publisher", (), {"publish": lambda self, msg: setattr(self, "msg", msg)})()
    bridge.set_position_target(10.0, -2.0, -3.0, yaw=3.14, speed=0.8)
    bridge._heartbeat()
    assert (bridge._sp_pub.msg.north, bridge._sp_pub.msg.east, bridge._sp_pub.msg.down) == (10.0, -2.0, -3.0)
    assert bridge._sp_pub.msg.yaw == 3.14
    assert bridge._sp_pub.msg.speed == 0.8
    assert bridge._sp_pub.msg.use_speed is True
```

- [ ] **Step 2: Run the propagation tests and verify they fail on the missing `speed` API**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py -k 'forwards_speed or relays_position'`

Expected: failure because `set_position_target()` does not accept `speed`.

- [ ] **Step 3: Implement the minimal relay propagation**

```python
# FlightController.goto_ned
self._bridge.set_position_target(north, east, down, yaw=yaw, speed=speed)

# FlightController.takeoff and loiter
self._bridge.set_position_target(north, east, down, yaw=0.0, speed=None)

# StandaloneBridge.set_position_target
self._target_ned = (float(north), float(east), float(down))
self._target_yaw = float(yaw)
self._target_speed = float(speed) if speed is not None else None

# StandaloneBridge._heartbeat position branch
msg = NedPositionSetpoint()
msg.north, msg.east, msg.down = self._target_ned
msg.yaw = self._target_yaw
msg.use_speed = self._target_speed is not None
msg.speed = self._target_speed if msg.use_speed else 0.0
self._sp_pub.publish(msg)

# PX4Bridge._cmd_setpoint_cb
self._target_setpoint = Position(msg.north, msg.east, msg.down)
self._target_yaw = float(msg.yaw)
self._target_speed = float(msg.speed) if msg.use_speed and msg.speed > 0.0 else None
```

Update the existing bridge mock signature to accept `speed=None` and assert takeoff still sends no leg-speed request.

- [ ] **Step 4: Run propagation tests and existing flight-safety tests**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py src/drone_mission/test/test_flight_safety_contract.py`

Expected: PASS.

### Task 3: Apply requested speed to the existing position ramp

**Files:**
- Modify: `src/px4_control/px4_control/px4_bridge.py:169-175,208-223,433-462`
- Modify: `src/px4_control/config/px4_params.yaml:19-23`
- Test: `src/px4_control/test/test_mission_setpoint_contract.py`

**Interfaces:**
- Consumes: `PX4Bridge._target_speed: Optional[float]` from Task 2 and parameter `max_setpoint_speed_limit`.
- Produces: ramp steps no greater than `min(target_speed, max_setpoint_speed_limit) * 0.05` when a speed is requested, otherwise the global safety limit times `0.05`.

- [ ] **Step 1: Write the failing ramp-limit test**

```python
def test_position_ramp_uses_the_lower_of_leg_speed_and_bridge_limit(monkeypatch):
    from px4_control.px4_bridge import PX4Bridge
    from px4_control.telemetry import Position

    bridge = PX4Bridge.__new__(PX4Bridge)
    bridge._target_setpoint = Position(10.0, 0.0, 0.0)
    bridge._current_setpoint = Position(0.0, 0.0, 0.0)
    bridge._target_yaw = 0.0
    bridge._smoothed_yaw = 0.0
    bridge._target_speed = 0.8
    bridge._setpoint_lock = _NoopLock()
    bridge.get_parameter = lambda name: type("Parameter", (), {"value": 2.0})()

    bridge._control_loop()
    assert bridge._current_setpoint.north == pytest.approx(0.04)
```

Define `_NoopLock` in the test with `__enter__` returning itself and a falsey `__exit__`. The expected step is `min(0.8, 2.0) * 0.05 = 0.04 m`.

- [ ] **Step 2: Run the ramp test and verify it fails because the requested speed is ignored**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py::test_position_ramp_uses_the_lower_of_leg_speed_and_bridge_limit`

Expected: failure because the current step is `0.10 m`.

- [ ] **Step 3: Implement bridge-owned speed cap semantics**

```python
# _declare_params
self.declare_parameter("max_setpoint_speed_limit", 3.0)

# _control_loop
limit = float(self.get_parameter("max_setpoint_speed_limit").value)
requested = self._target_speed if self._target_speed is not None else limit
max_step = max(0.0, min(float(requested), limit)) * 0.05
```

Replace all `max_acceleration` names in `PX4Bridge` and set
`max_setpoint_speed_limit: 3.0` in `px4_params.yaml`. Keep position-mode
`TrajectorySetpoint` publishing and all velocity-mode code unchanged.

- [ ] **Step 4: Run ramp and PX4 safety-contract tests**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py src/px4_control/test/test_px4_safety_contract.py`

Expected: PASS.

### Task 4: Remove duplicate configuration and complete validation

**Files:**
- Modify: `src/drone_mission/config/mission_params.yaml:29-33,55-57`
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py:223-227,385-391,1062-1082`
- Modify: `src/drone_mission/drone_mission/path_planner.py:4-9`
- Modify: `src/drone_mission/drone_mission/states/state_recon_patrol.py:57-62`
- Test: `src/px4_control/test/test_mission_setpoint_contract.py`

**Interfaces:**
- Consumes: bridge `max_setpoint_speed_limit` from Task 3.
- Produces: mission-only phase-speed policy with `sweep_speed: 0.8`, `transit_speed`, `return_speed`; no mission `max_acceleration` or unused `sweep_passes`.

- [ ] **Step 1: Write failing configuration-contract tests**

```python
def test_mission_speed_policy_has_one_owner_and_validates_all_leg_speeds():
    mission_yaml = (ROOT / "drone_mission/config/mission_params.yaml").read_text(encoding="utf-8")
    mission_source = (ROOT / "drone_mission/drone_mission/mission_state_machine.py").read_text(encoding="utf-8")
    bridge_yaml = (ROOT / "px4_control/config/px4_params.yaml").read_text(encoding="utf-8")
    assert "sweep_speed: 0.8" in mission_yaml
    assert "max_acceleration" not in mission_yaml
    assert "sweep_passes" not in mission_yaml
    assert "(\"sweep_speed\", 0.1, 15.0)" in mission_source
    assert "max_acceleration" not in bridge_yaml
    assert "sweep_speed:" not in bridge_yaml
    assert "transit_speed:" not in bridge_yaml
    assert "return_speed:" not in bridge_yaml
    assert "max_setpoint_speed_limit: 3.0" in bridge_yaml
```

- [ ] **Step 2: Run the configuration test and verify it fails on legacy configuration**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py::test_mission_speed_policy_has_one_owner_and_validates_all_leg_speeds`

Expected: failure finding legacy parameters.

- [ ] **Step 3: Make mission policy unambiguous**

Remove `max_acceleration` from mission YAML and FSM declarations. Remove
`sweep_passes` from mission YAML and FSM declarations because no planner
consumes it. Change both mission `grid_spacing` fallbacks and the planner
module docstring from `1.0` to `1.67`. Add
`("sweep_speed", 0.1, 15.0)` to `_validate_critical_params()`.

- [ ] **Step 4: Run the configuration and mission test suites**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py src/drone_mission/test`

Expected: PASS.

### Task 5: Update user-facing architecture and audit documentation

**Files:**
- Modify: `docs/recon_speed_parameter_audit.md:1-179`
- Modify: `docs/ARCHITECTURE.md:20-40,148-163`
- Modify: `docs/CODE_READING_GUIDE.md`
- Test: `src/px4_control/test/test_mission_setpoint_contract.py`

**Interfaces:**
- Consumes: completed relay, configuration ownership, and ramp behavior from Tasks 1-4.
- Produces: accurate operator guidance: editing `mission_params.yaml:sweep_speed` changes nominal reconnaissance setpoint speed after restart, subject to bridge and PX4 safety limits.

- [ ] **Step 1: Write the failing documentation-contract test**

```python
def test_operator_docs_describe_the_speed_control_contract():
    audit = (ROOT.parents[1] / "docs/recon_speed_parameter_audit.md").read_text(encoding="utf-8")
    assert "sweep_speed` 改多少都不会影响" not in audit
    assert "max_setpoint_speed_limit" in audit
    assert "mission_params.yaml" in audit
    assert "0.8" in audit
```

- [ ] **Step 2: Run the documentation test and verify it fails on the obsolete audit conclusion**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py::test_operator_docs_describe_the_speed_control_contract`

Expected: failure because the audit still labels `sweep_speed` as dead.

- [ ] **Step 3: Update operator documentation**

Rewrite the audit as a resolved issue record: document the typed relay, speed
ownership, bridge cap semantics, yaw preservation, and verification status.
Update the architecture command topology to name `NedPositionSetpoint` and
separate it from velocity command topics. Update the reading guide's interface
section to include the new message and relay call path.

- [ ] **Step 4: Run the documentation contract test**

Run: `python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py::test_operator_docs_describe_the_speed_control_contract`

Expected: PASS.

### Task 6: Build the generated interface and run workspace verification

**Files:**
- Modify: no source files.

**Interfaces:**
- Consumes: all completed source changes.
- Produces: generated `NedPositionSetpoint` Python/C++ artifacts and verified package test results.

- [ ] **Step 1: Build all affected dependencies**

Run: `colcon build --symlink-install --packages-up-to drone_bringup test_mission`

Expected: `drone_interfaces`, `px4_control`, `drone_mission`, `drone_bringup`, and `test_mission` build successfully.

- [ ] **Step 2: Source the rebuilt workspace and run primary regression tests**

Run: `source install/setup.bash && python3 -m pytest -q src/drone_mission/test src/drone_vision/test src/px4_control/test src/drone_bringup/test`

Expected: PASS.

- [ ] **Step 3: Record the SITL verification procedure in the audit**

Document inspection of `/drone/cmd_setpoint_ned` and
`/fmu/in/trajectory_setpoint` at `sweep_speed=0.8`, plus a controlled
comparison using `2.0` and `3.0 m/s`. Mark real-flight validation as required
before safety-critical deployment.
