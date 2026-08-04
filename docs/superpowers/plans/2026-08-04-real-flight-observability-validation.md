# Real-Flight Observability and Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make planned versus actual flight behavior auditable, make pre-arm readiness easy to verify on the real vehicle, and prove the completed code through package tests, SITL, and staged real-hardware procedures.

**Architecture:** Structured mission metadata, plan events, telemetry samples, recovery events, and a truthful summary are written by `TaskLogger`. A read-only ground verifier checks the canonical real-v1 topics and all application-level readiness signals. Automated deployment contracts protect the real/SITL boundary, and operator documentation defines the final staged acceptance sequence.

**Tech Stack:** ROS 2 Humble, Python 3.10, JSONL/JSON, bash, pytest, colcon, PX4 SITL, MicroXRCEAgent.

## Global Constraints

- Treat the deployed real v1 interface as authoritative: `/fmu/out/vehicle_status_v1`, `/fmu/out/battery_status_v1`, and `/fmu/out/vehicle_command_ack` are real defaults.
- Keep SITL v4 isolated behind `scripts/sitl_env.sh` and `/fmu/out/vehicle_status_v4` launch override.
- Keep application-level `/drone/*` topics stable across both deployments.
- Never report mission success after hold, takeover, RTL, LAND fallback, incomplete drops, incomplete recon, or unconfirmed touchdown.
- Log at a bounded rate; telemetry logging must not interfere with the 20 Hz control loop.
- Ground verification is read-only and must not arm, switch modes, move the servo, or publish flight commands.
- Preserve the independent `test_mission` and full-mission entry points.
- Commit after every task.

---

## File Structure

- Modify `src/drone_utils/drone_utils/logger.py`: metadata, plan, recovery, and bounded telemetry APIs.
- Modify `src/drone_utils/test/`: logger tests.
- Modify `src/px4_control/px4_control/standalone_bridge.py`: command snapshot for logging.
- Modify `src/drone_mission/drone_mission/mission_state_machine.py`: metadata and telemetry sampling.
- Modify `src/drone_mission/drone_mission/path_planner.py` and states: log plan fragments as they are generated.
- Create `scripts/verify_real_flight_ready.sh`: read-only ground verifier.
- Modify `src/drone_bringup/test/`: launch and topic boundary contracts.
- Modify `docs/DEPLOYMENT.md`, `docs/REAL_HARDWARE_DEBUG_GUIDE.md`, `docs/ARCHITECTURE.md`, `docs/RISKS_AND_TODOS.md`, and `docs/WORKLOG.md`.
- Modify the stale speed-contract test so it references current operator documentation.

### Task 1: Structured Metadata, Plan, Recovery, and Telemetry Logs

**Files:**
- Modify: `src/drone_utils/drone_utils/logger.py`
- Create: `src/drone_utils/test/test_task_logger_runtime.py`

**Interfaces:**
- Produces: `log_metadata(data: dict) -> None`
- Produces: `log_plan(stage: str, waypoints: list[dict]) -> None`
- Produces: `log_recovery(stage: str, reason: str, action: str, attempt: int, success: Optional[bool]) -> None`
- Produces: `log_telemetry_sample(data: dict, now: float) -> bool`
- Produces constructor option: `telemetry_interval_s: float = 0.5`

- [ ] **Step 1: Write failing logger tests**

```python
import json

from drone_utils.logger import TaskLogger


def test_telemetry_is_rate_limited(tmp_path):
    logger = TaskLogger(str(tmp_path), telemetry_interval_s=0.5)
    assert logger.log_telemetry_sample({"north": 1.0}, now=1.0)
    assert not logger.log_telemetry_sample({"north": 2.0}, now=1.2)
    assert logger.log_telemetry_sample({"north": 3.0}, now=1.5)
    lines = logger.telemetry_path.read_text().splitlines()
    assert len(lines) == 2


def test_plan_and_recovery_are_structured_events(tmp_path):
    logger = TaskLogger(str(tmp_path))
    logger.log_plan("recon", [{"north": 55.0, "east": -4.0, "down": -2.5}])
    logger.log_recovery("return", "no_progress", "resend_waypoint", 1, None)
    events = [json.loads(line) for line in logger.events_path.read_text().splitlines()]
    assert [event["event"] for event in events[-2:]] == ["plan", "recovery"]
```

- [ ] **Step 2: Verify tests fail**

Run `python3 -m pytest -q src/drone_utils/test/test_task_logger_runtime.py`. Expected: FAIL on missing APIs.

- [ ] **Step 3: Implement bounded structured APIs**

Use `pathlib.Path` properties for testable paths. `log_telemetry_sample()` must compare the caller-provided monotonic time to `_last_telemetry_write`, write JSONL only when the interval elapsed, and return whether it wrote. `log_plan()` must preserve exact NED values, speed, yaw, hover time, and label fields supplied by planners.

- [ ] **Step 4: Run logger tests**

Run the Task 1 command. Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/drone_utils/drone_utils/logger.py src/drone_utils/test/test_task_logger_runtime.py
git commit -m "feat(logging): record bounded mission plans and recovery"
```

### Task 2: Full-Mission Runtime Metadata and Planned-vs-Actual Samples

**Files:**
- Modify: `src/px4_control/px4_control/standalone_bridge.py`
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_search.py`
- Modify: `src/drone_mission/drone_mission/states/state_recon_patrol.py`
- Modify: `src/drone_mission/drone_mission/states/state_return.py`
- Create: `src/drone_mission/test/test_mission_observability.py`

**Interfaces:**
- Produces: `StandaloneBridge.command_snapshot() -> dict`
- Produces: mission event `metadata`
- Produces: plan events for search, recon, transit, target approach, and return
- Produces: 2 Hz actual position/velocity and current-command samples

- [ ] **Step 1: Write failing observability tests**

Assert a position command snapshot contains mode, NED target, speed, and yaw; a velocity snapshot contains NED velocity; and mission initialization records deployment type, status topic, camera geometry, configured speeds, and servo positions without shelling out during the control loop.

- [ ] **Step 2: Verify tests fail**

Run `python3 -m pytest -q src/drone_mission/test/test_mission_observability.py`. Expected: FAIL on missing `command_snapshot()` and metadata hooks.

- [ ] **Step 3: Add command snapshots**

Return a new dictionary under the bridge lock-free single-threaded state model:

```python
def command_snapshot(self) -> dict:
    if self._velocity_mode:
        return {"mode": "velocity", "velocity_ned": list(self._vel_target)}
    return {
        "mode": "position",
        "position_ned": list(self._target_ned) if self._target_ned else None,
        "yaw": self._target_yaw,
        "speed": self._target_speed,
    }
```

- [ ] **Step 4: Record startup metadata once**

After all runtime prerequisites become ready, record `sim_mode`, `vehicle_status_topic`, Home, flight axis, area bounds, task speeds, camera geometry, and configured servo PWM list. Obtain the git commit once during lazy initialization with a bounded subprocess call or environment value; never call git from the 20 Hz loop.

- [ ] **Step 5: Log plans and actual telemetry**

When planners generate or states select targets, call `log_plan()`. At 2 Hz, log current NED position, velocity, armed/offboard/landed status, current state, command snapshot, coverage ratio, and current visual error when present.

- [ ] **Step 6: Run observability and mission tests**

Run `python3 -m pytest -q src/drone_mission/test/test_mission_observability.py src/drone_mission/test/test_recon_coverage_lifecycle.py`. Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/px4_control/px4_control/standalone_bridge.py src/drone_mission src/drone_mission/test/test_mission_observability.py
git commit -m "feat(mission): log planned and actual flight behavior"
```

### Task 3: Read-Only Real-Flight Ground Verifier

**Files:**
- Create: `scripts/verify_real_flight_ready.sh`
- Create: `src/drone_bringup/test/test_real_flight_verifier.py`

**Interfaces:**
- Consumes real topics and application readiness only
- Produces exit code `0` when all checks pass, nonzero otherwise
- Produces no publishers, service calls, arm commands, mode commands, or actuator commands

- [ ] **Step 1: Write failing source-contract tests**

Assert the script checks a publisher and one received sample for `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_odometry`, `/fmu/out/battery_status_v1`, and `/fmu/out/vehicle_land_detected`; checks `/drone/pose`, camera image, valid `CameraInfo`, vision `ready=1`, servo `ready=1`, and the drop service; and contains none of `ros2 topic pub`, `ros2 service call`, arm, disarm, mode, RTL, LAND, or actuator commands.

- [ ] **Step 2: Verify the test fails**

Run `python3 -m pytest -q src/drone_bringup/test/test_real_flight_verifier.py`. Expected: FAIL because the script is missing.

- [ ] **Step 3: Implement bounded read-only checks**

Source ROS and workspace setup, require `/fmu/out/vehicle_status_v1` exactly, reject `/fmu/out/vehicle_status_v4` in the active real environment, and use `timeout` around every `ros2 topic echo --once`. Resolve the active `camera_calibration_file` parameter, parse that standard ROS YAML with the installed `CameraCalibration` helper, and compare the received `CameraInfo` dimensions, matrix, distortion model, and coefficients to the selected file. The default file therefore checks the provisional 640×480 values, while a later calibrated replacement remains valid without editing this verifier. Print a concise PASS/FAIL table.

- [ ] **Step 4: Run the source-contract test**

Run the Task 3 test command. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add scripts/verify_real_flight_ready.sh src/drone_bringup/test/test_real_flight_verifier.py
git commit -m "feat(bringup): add read-only real-flight readiness check"
```

### Task 4: Repair Stale Test Contracts and Protect Launch Boundaries

**Files:**
- Modify: `src/px4_control/test/test_mission_setpoint_contract.py`
- Modify: `src/drone_bringup/test/test_px4_interface_deployment_contract.py`
- Modify: `src/drone_bringup/test/test_vision_launch_configuration.py`
- Modify: `src/drone_bringup/test/test_simulation_script.py`

**Interfaces:**
- Verifies current documentation and source contracts only

- [ ] **Step 1: Replace the deleted-document dependency**

Change `test_operator_docs_describe_the_speed_control_contract` to read `docs/PARAMETER_USAGE_GUIDE.md`, which is the current operator source, and keep assertions for `sweep_speed: 1.2`, `transit_speed: 2.5`, `return_speed: 3.8`, and `max_setpoint_speed_limit`.

- [ ] **Step 2: Add real-first boundary assertions**

Assert real launch and source defaults remain v1; SITL launch alone uses v4; real servo ACK is unversioned; all `/drone/*` command, pose, vision, and servo service names are identical across launch files; and real scripts never source `sitl_env.sh`.

- [ ] **Step 3: Run contract tests**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/px4_control/test/test_mission_setpoint_contract.py
python3 -m pytest -q src/drone_bringup/test/test_px4_interface_deployment_contract.py src/drone_bringup/test/test_vision_launch_configuration.py src/drone_bringup/test/test_simulation_script.py
```

Expected: all tests PASS, including the previously failing deleted-file test.

- [ ] **Step 4: Commit**

```bash
git add src/px4_control/test src/drone_bringup/test
git commit -m "test: align contracts with current real and SITL deployments"
```

### Task 5: Operator Documentation and Risk Closure

**Files:**
- Modify: `docs/ARCHITECTURE.md`
- Modify: `docs/DEPLOYMENT.md`
- Modify: `docs/REAL_HARDWARE_DEBUG_GUIDE.md`
- Modify: `docs/PARAMETER_USAGE_GUIDE.md`
- Modify: `docs/RISKS_AND_TODOS.md`
- Modify: `docs/WORKLOG.md`
- Modify: `docs/README.md`

**Interfaces:**
- Produces one current operational narrative

- [ ] **Step 1: Update architecture and deployment facts**

Document independent FSMs, camera-calibration flow, progress recovery, takeover hold, strict real touchdown fallback, simultaneous climbing return to 5 m, and the real-v1/SITL-v4 boundary. Remove statements that real landing only accepts `vehicle_land_detected` or that any timeout automatically advances, RTLs, or LANDs.

- [ ] **Step 2: Add exact operator procedures**

Document commands for clean rebuild, calibration replacement, camera inspection, servo ground test, readiness verifier, independent `test_mission`, full real launch, log collection, and RC takeover. State that the readiness verifier never arms or moves hardware.

- [ ] **Step 3: Close risks only when code tests exist**

Mark the swallowed-state-exception, real touchdown fallback, model cleanup, and stale-setpoint risks as code-resolved only if their associated automated tests pass. Keep physical calibration, field PWM travel, surface touchdown behavior, and full real flight as field-verification items.

- [ ] **Step 4: Record the implementation in WORKLOG**

Add one dated entry listing code changes, automated evidence, SITL evidence, and remaining real-hardware steps. Do not claim the full real mission has flown until a field log proves it.

- [ ] **Step 5: Commit**

```bash
git add docs
git commit -m "docs: document full real-flight operation and recovery"
```

### Task 6: Clean Build and Package Test Gate

**Files:**
- No source changes expected; fix only failures caused by this implementation and commit those fixes separately.

**Interfaces:**
- Produces a clean workspace build and package-level test evidence

- [ ] **Step 1: Record the current commit and protect field logs**

Run `git status --short` and confirm only intended implementation changes exist. Do not delete `real_flight_logs/`. If interface definitions changed, move existing root `build/`, `install/`, and `log/` to a timestamped directory under `/tmp` before rebuilding; otherwise use package-selective builds.

- [ ] **Step 2: Build all affected packages**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to drone_bringup test_mission
```

Expected: build summary contains no failed packages.

- [ ] **Step 3: Run functional tests package by package**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/drone_utils/test -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest -q src/px4_control/test -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest -q src/drone_vision/test -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest -q src/drone_mission/test -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest -q src/drone_servo2/test
python3 -m pytest -q src/drone_bringup/test -k 'not flake8 and not pep257 and not copyright'
python3 -m pytest -q src/test_mission/test -k 'not flake8 and not pep257 and not copyright'
```

Expected: every command exits zero.

- [ ] **Step 4: Run package lint through colcon context**

```bash
colcon test --packages-select drone_utils px4_control drone_vision drone_mission drone_servo2 drone_bringup test_mission
colcon test-result --verbose
```

Expected: zero test failures. Running lint through package build directories avoids the known root-workspace flake8 scan pollution.

- [ ] **Step 5: Handle any gate failure as a separate tested fix**

No commit is created when the gate passes. If a command fails, stop this gate, add the smallest regression test to the task that owns the faulty behavior, make that test pass, commit the explicitly named source and test files in that task, and restart Task 6 from Step 1. Do not create an empty or catch-all gate commit.

### Task 7: Full SITL Behavioral Regression

**Files:**
- Modify only if the run exposes a reproducible implementation defect; add a regression test before each fix.

**Interfaces:**
- Produces a successful full-SITL log and summary matching task semantics

- [ ] **Step 1: Start the full simulation through its isolated environment**

Run:

```bash
./scripts/start_full_simulation.sh
```

Expected: the script sources `scripts/sitl_env.sh`, observes `/fmu/out/vehicle_status_v4`, and completes without altering real v1 defaults.

- [ ] **Step 2: Verify the generated run evidence**

In the generated `log/simulation_YYYYMMDD_HHMMSS/` directory, verify:

```bash
rg "投放成功|覆盖|RETURN|盲降|mission_end" log/simulation_*/ros2_system.log
```

Expected: two successful drops, completed recon path, return beginning at the final recon waypoint with target altitude 5 m, H alignment, blind descent, confirmed touchdown, and `success=True`.

- [ ] **Step 3: Compare planned and actual paths**

Use the JSONL plan and telemetry events to verify all planned recon waypoints were visited in order and the return command combined Home North/East with 5 m altitude. Actual tracking may vary, but no recovery should trigger in the nominal run.

- [ ] **Step 4: Add a regression test for any discovered defect**

If the run fails, first add the smallest deterministic unit or contract test reproducing the cause, run it to FAIL, implement the fix, run it to PASS, and repeat the complete SITL run. Do not tune real-v1 topic names to fix a SITL-only issue.

- [ ] **Step 5: Commit SITL-proven corrections**

Commit only when Step 4 changed files; use a message naming the observed defect.

### Task 8: Staged Real-Hardware Handoff

**Files:**
- No autonomous code changes during this task; field-discovered corrections start with a new failing regression test and a scoped commit.

**Interfaces:**
- Produces field evidence without overstating unflown behavior

- [ ] **Step 1: Camera and DDS ground verification**

With propellers removed, start `real_flight.launch.py`, run `scripts/verify_real_flight_ready.sh`, and confirm only `/fmu/out/vehicle_status_v1` is selected. Save mission, Agent, camera, and verifier output.

- [ ] **Step 2: Absolute servo positions**

Run the existing hardware servo test with no payload, verify 1150 → 1700 → 2150 μs physically and in status/ACK logs, and confirm the ACK topic is `/fmu/out/vehicle_command_ack`.

- [ ] **Step 3: Independent test_mission regression flight**

Run `scripts/test_forward_recon.sh`. Verify the final recon waypoint transitions immediately to a Home+5 m return command and actual altitude rises during horizontal return. Verify H landing and truthful summary.

- [ ] **Step 4: Full-mission staged flights**

Perform ground target detection, one controlled drop validation, two-drop validation, then the complete mission. Keep RC takeover available. After each flight, compare plan, actual telemetry, recovery events, drop count, coverage, and landing summary.

- [ ] **Step 5: Update field evidence only**

Append the exact run directory and result to `docs/WORKLOG.md`. Mark the full real mission verified only after logs show two drops, complete recon, climbing return to 5 m, H landing, touchdown, and successful summary.
