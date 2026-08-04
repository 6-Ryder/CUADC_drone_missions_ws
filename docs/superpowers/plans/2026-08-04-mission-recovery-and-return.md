# Mission Recovery and Return Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the independent full and test FSMs progress-based recovery, a latched operator-takeover hold, idempotent servo retry, strict touchdown confirmation, and a common 5 m climbing return behavior.

**Architecture:** A small ROS-independent progress utility and output-lifecycle methods live below both FSMs, while each FSM owns its monitor instances, states, recovery counters, and fault record. Normal mission behavior remains the proven SITL path; one recovery cycle is allowed before the affected FSM independently enters a zero-velocity takeover hold.

**Tech Stack:** ROS 2 Humble, Python 3.10, C++17/rclcpp, PX4 v1 real interface, pytest, ament CMake tests.

## Global Constraints

- `MissionFSM` and `test_mission` remain separate nodes and separate FSMs.
- The real v1 PX4 deployment is canonical: `/fmu/out/vehicle_status_v1`, `/fmu/out/battery_status_v1`, and `/fmu/out/vehicle_command_ack` remain source and real-launch defaults.
- SITL alone may override status to `/fmu/out/vehicle_status_v4` through the isolated v4 overlay; SITL uses the servo mock and does not redefine the real ACK topic.
- Never advance a required mission state merely because wall-clock time elapsed.
- Never skip a required reconnaissance waypoint.
- Never issue task-level automatic RTL or LAND on recovery exhaustion.
- Permit one complete recovery cycle per action; then latch `HOLD_FOR_TAKEOVER`.
- Preserve PX4 native battery, RC-loss, and communication failsafes and stop competing when PX4 leaves Offboard.
- The full mission and test mission both return directly from the final reconnaissance waypoint while climbing to 5 m.
- Commit after every task.

---

## File Structure

- Create `src/drone_utils/drone_utils/progress_monitor.py`: deterministic distance/error progress tracking.
- Create `src/drone_utils/test/test_progress_monitor.py`: progress semantics tests.
- Modify `src/px4_control/px4_control/standalone_bridge.py`: telemetry freshness and autonomous-output lifecycle.
- Modify `src/px4_control/test/test_px4_safety_contract.py`: bridge safety contract.
- Create `src/drone_mission/drone_mission/states/state_hold_for_takeover.py`: full-FSM terminal hold.
- Modify `src/drone_mission/drone_mission/mission_state_machine.py`: fault routing and true failure summaries.
- Modify navigation, search, approach, drop, recon, return, and landing state files under `src/drone_mission/drone_mission/states/`.
- Modify `src/drone_servo2/src/servo_controller.cpp`: one idempotent absolute-PWM retry.
- Modify `src/test_mission/test_mission/test_forward_recon.py`: independent progress recovery and takeover hold.
- Modify mission, control, servo, and bringup contract tests.

### Task 1: Deterministic Progress Monitor

**Files:**
- Create: `src/drone_utils/drone_utils/progress_monitor.py`
- Create: `src/drone_utils/test/test_progress_monitor.py`

**Interfaces:**
- Produces: `ProgressMonitor(window_s: float, min_improvement: float)`
- Produces: `reset(metric: float, now: float) -> None`
- Produces: `observe(metric: float, now: float) -> bool`, where `True` means stalled
- Produces: `mark_recovery(metric: float, now: float) -> None`
- Produces: `recovery_used: bool`

- [ ] **Step 1: Write failing behavior tests**

```python
from drone_utils.progress_monitor import ProgressMonitor


def test_improvement_moves_the_stall_window():
    monitor = ProgressMonitor(window_s=10.0, min_improvement=0.3)
    monitor.reset(metric=10.0, now=0.0)
    assert not monitor.observe(metric=9.6, now=9.0)
    assert not monitor.observe(metric=9.5, now=18.0)
    assert monitor.observe(metric=9.5, now=19.1)


def test_only_one_recovery_is_available():
    monitor = ProgressMonitor(window_s=5.0, min_improvement=0.1)
    monitor.reset(metric=2.0, now=0.0)
    assert monitor.observe(metric=2.0, now=5.1)
    monitor.mark_recovery(metric=2.0, now=5.1)
    assert monitor.recovery_used
    assert monitor.observe(metric=2.0, now=10.2)
```

- [ ] **Step 2: Run and verify import failure**

Run `python3 -m pytest -q src/drone_utils/test/test_progress_monitor.py`. Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement the monitor**

The implementation must reject non-positive windows or improvements, treat lower metrics as better, move the baseline only when improvement is at least `min_improvement`, and never read wall clock internally.

```python
class ProgressMonitor:
    def observe(self, metric: float, now: float) -> bool:
        if self._best_metric is None:
            self.reset(metric, now)
            return False
        if self._best_metric - metric >= self.min_improvement:
            self._best_metric = metric
            self._last_progress_time = now
            return False
        return now - self._last_progress_time >= self.window_s
```

- [ ] **Step 4: Run tests**

Run the Task 1 command. Expected: `2 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/drone_utils/drone_utils/progress_monitor.py src/drone_utils/test/test_progress_monitor.py
git commit -m "feat(utils): add progress-based stall monitor"
```

### Task 2: StandaloneBridge Freshness and Output Lifecycle

**Files:**
- Modify: `src/px4_control/px4_control/standalone_bridge.py`
- Modify: `src/px4_control/test/test_px4_safety_contract.py`

**Interfaces:**
- Produces: `telemetry_fresh(now: float, max_age_s: float = 2.0) -> bool`
- Produces: `hold_for_takeover() -> None`
- Produces: `stop_autonomous_output() -> None`
- Produces: `autonomous_output_enabled: bool`

- [ ] **Step 1: Add failing bridge contract tests**

Assert status and odometry callbacks update separate timestamps, `_heartbeat()` publishes nothing when output is disabled, `hold_for_takeover()` selects velocity mode with `(0,0,0)`, and `stop_autonomous_output()` clears both position and velocity targets.

- [ ] **Step 2: Verify tests fail**

Run `python3 -m pytest -q src/px4_control/test/test_px4_safety_contract.py`. Expected: FAIL on missing lifecycle methods.

- [ ] **Step 3: Implement freshness and lifecycle**

Initialize `_last_status_time = 0.0`, `_last_odom_time = 0.0`, and `_autonomous_output_enabled = True`. Use `time.monotonic()` in callbacks. Start `_heartbeat()` with:

```python
if self._shutdown or not self._autonomous_output_enabled:
    return
```

Implement:

```python
def hold_for_takeover(self) -> None:
    self._autonomous_output_enabled = True
    self._velocity_mode = True
    self._target_ned = None
    self._target_speed = None
    self._vel_target = (0.0, 0.0, 0.0)

def stop_autonomous_output(self) -> None:
    self._autonomous_output_enabled = False
    self._target_ned = None
    self._target_speed = None
    self._vel_target = (0.0, 0.0, 0.0)
```

- [ ] **Step 4: Run control tests**

Run `python3 -m pytest -q src/px4_control/test/test_px4_safety_contract.py`. Expected: all lifecycle and safety tests PASS. The stale documentation assertion in `test_mission_setpoint_contract.py` is repaired and run as a separate gate in Plan 3.

- [ ] **Step 5: Commit**

```bash
git add src/px4_control/px4_control/standalone_bridge.py src/px4_control/test/test_px4_safety_contract.py
git commit -m "feat(control): add takeover-safe output lifecycle"
```

### Task 3: Full-FSM Latched Takeover State

**Files:**
- Create: `src/drone_mission/drone_mission/states/state_hold_for_takeover.py`
- Modify: `src/drone_mission/drone_mission/states/__init__.py`
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Create: `src/drone_mission/test/test_hold_for_takeover.py`
- Modify: `src/drone_mission/test/test_drop_service_failure.py`

**Interfaces:**
- Produces: `MissionState.HOLD_FOR_TAKEOVER`
- Produces: `MissionFSM.request_takeover(reason: str, source_state: Optional[str] = None) -> None`
- Produces: immutable first-fault mapping with `reason`, `source_state`, and `timestamp`

- [ ] **Step 1: Write failing state tests**

Test that entry calls `hold_for_takeover`, repeated failures do not replace the first reason, the state remains latched while armed and Offboard, and leaving Offboard calls `stop_autonomous_output()` then returns `COMPLETE` with failure.

```python
def test_hold_is_latched_until_operator_takes_over():
    fsm = _Fsm(armed=True, offboard=True)
    state = HoldForTakeoverState(fsm)
    state.enter()
    assert state.execute() is None
    fsm.bridge.offboard = False
    assert state.execute() == "COMPLETE"
    assert fsm.bridge.output_stopped
```

- [ ] **Step 2: Verify tests fail**

Run `python3 -m pytest -q src/drone_mission/test/test_hold_for_takeover.py`. Expected: FAIL on missing state module.

- [ ] **Step 3: Implement the dedicated state**

`enter()` logs the first fault and commands zero velocity. `execute()` logs every five seconds; while armed and Offboard it returns `None`; once disarmed or outside Offboard it stops output and returns `COMPLETE`. `exit()` must not restart output.

- [ ] **Step 4: Route all terminal errors to takeover hold**

Add the enum and handler. `request_takeover()` sets `_mission_failed = True`, stores the first fault once, and transitions to `HOLD_FOR_TAKEOVER`. Replace state-handler exceptions, entry exceptions, task-level critical battery handling, and legacy `ERROR` handling with this method. Pre-arm initialization failures remain in `INIT` and report diagnostics rather than entering flight hold.

- [ ] **Step 5: Make summaries truthful**

Only `PRECISION_LAND → COMPLETE` with all mission conditions satisfied may write `success=True`. Operator takeover, disarm before confirmed touchdown, `ERROR`, or hold completion writes `success=False` and includes `first_fault`.

- [ ] **Step 6: Run mission failure tests**

Run `python3 -m pytest -q src/drone_mission/test/test_hold_for_takeover.py src/drone_mission/test/test_drop_service_failure.py`. Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/drone_mission/drone_mission/states src/drone_mission/drone_mission/mission_state_machine.py src/drone_mission/test/test_hold_for_takeover.py src/drone_mission/test/test_drop_service_failure.py
git commit -m "feat(mission): latch terminal faults for operator takeover"
```

### Task 4: Indefinite Pre-Arm Readiness

**Files:**
- Modify: `src/drone_mission/drone_mission/states/state_init.py`
- Modify: `src/drone_mission/test/test_flight_safety_contract.py`

**Interfaces:**
- Consumes: `bridge.telemetry_fresh()` and camera/vision/servo readiness
- Produces: no pre-arm wall-clock failure

- [ ] **Step 1: Write the failing readiness test**

Advance fake time beyond 30 seconds with one missing prerequisite and assert `execute()` returns `None`, reports the missing item, and never requests arm or a state transition.

- [ ] **Step 2: Verify the old forced error fails the test**

Run the single readiness test. Expected: FAIL because current code returns `ERROR` after 30 seconds.

- [ ] **Step 3: Remove the fixed INIT timeout**

Keep periodic diagnostics and all readiness gates. Require fresh status and odometry rather than a nonzero pose. Reset readiness counters when data becomes stale. Do not add a replacement total timeout.

- [ ] **Step 4: Run safety tests**

Run `python3 -m pytest -q src/drone_mission/test/test_flight_safety_contract.py`. Expected: all tests PASS after updating the obsolete timeout assertion.

- [ ] **Step 5: Commit**

```bash
git add src/drone_mission/drone_mission/states/state_init.py src/drone_mission/test/test_flight_safety_contract.py
git commit -m "fix(mission): wait indefinitely for pre-arm readiness"
```

### Task 5: Full-FSM Navigation Progress and 5 m Return

**Files:**
- Modify: `src/drone_mission/drone_mission/states/state_takeoff.py`
- Modify: `src/drone_mission/drone_mission/states/state_transit.py`
- Modify: `src/drone_mission/drone_mission/states/state_recon_patrol.py`
- Modify: `src/drone_mission/drone_mission/states/state_return.py`
- Modify: `src/drone_mission/config/mission_params.yaml`
- Modify: `src/drone_mission/test/test_flight_safety_contract.py`
- Modify: `src/drone_mission/test/test_recon_coverage_lifecycle.py`

**Interfaces:**
- Consumes: `ProgressMonitor`
- Produces per-state: one same-command recovery, followed by `request_takeover(reason)`
- Produces: return altitude from `takeoff_height`, default 5.0 m

- [ ] **Step 1: Add failing progress tests**

For takeoff, transit, recon waypoint, and return: demonstrate that improving metrics never fail, the first stalled window reissues the same target, and the second stalled window requests takeover. Assert recon does not increment its waypoint index on stall.

- [ ] **Step 2: Verify current timeout behavior fails**

Run the two mission test files. Expected: failures showing fixed-time `ERROR`, RTL, or waypoint skip behavior.

- [ ] **Step 3: Integrate one monitor per active action**

Use altitude error for takeoff, 3-D target distance for transit, current-waypoint distance for recon, and combined Home XY/Z distance for return. Reset the monitor on state entry and on each new recon waypoint. The first stall calls `mark_recovery()` and resends the exact command; the second calls `request_takeover()`.

- [ ] **Step 4: Remove fixed navigation fall-throughs**

Delete the paths that return `ERROR` solely due to `waypoint_timeout`, trigger RTL after return timeout, or skip a recon waypoint. Retain stable-arrival counters.

- [ ] **Step 5: Derive return altitude from mission configuration**

Remove `ReturnState.HOME_Z`. In entry and arrival checks use:

```python
return_altitude = float(self._fsm.get_param("takeoff_height", 5.0))
```

The state must send Home North/East and this altitude in one `goto_ned()` call immediately after the final reconnaissance waypoint.

- [ ] **Step 6: Run navigation tests**

Run `python3 -m pytest -q src/drone_mission/test/test_flight_safety_contract.py src/drone_mission/test/test_recon_coverage_lifecycle.py`. Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/drone_mission/drone_mission/states src/drone_mission/config/mission_params.yaml src/drone_mission/test
git commit -m "fix(mission): recover stalled navigation without skipping work"
```

### Task 6: Full-FSM Vision, Search, Drop, and Landing Recovery

**Files:**
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_search.py`
- Modify: `src/drone_mission/drone_mission/states/state_cylinder_approach.py`
- Modify: `src/drone_mission/drone_mission/states/state_drop.py`
- Modify: `src/drone_mission/drone_mission/states/state_precision_land.py`
- Modify: `src/drone_mission/drone_mission/mission_state_machine.py`
- Modify: `src/drone_mission/config/mission_params.yaml`
- Modify: tests under `src/drone_mission/test/`

**Interfaces:**
- Produces: one repeated search path when targets are insufficient
- Produces: one climb-and-research checkpoint after low-altitude alignment failure
- Produces: `touchdown_confirmed() -> bool` for SITL and real hardware

- [ ] **Step 1: Add failing recovery tests**

Cover insufficient targets after one path, short approach target loss, precision-align loss, H loss, and touchdown fallback. Assert no case issues RTL/LAND or reports success.

- [ ] **Step 2: Verify current behavior fails selected tests**

Run the new tests plus `test_drop_service_failure.py`. Expected: failures on fixed errors and sim-only touchdown fallback.

- [ ] **Step 3: Implement one search-path retry**

End search based on path completion rather than `state_max_duration`. If fewer than `min_drop_targets_required` exist, clear transient observations, retain already-dropped positions, reset the same search path once, and then request takeover if still insufficient.

- [ ] **Step 4: Implement visual recovery checkpoints**

Approach uses memory while detections are fresh enough; on recovery exhaustion it releases the current selection and returns to `CYLINDER_SEARCH`. Low-altitude precision failure commands zero horizontal velocity, climbs to `area1_search_altitude`, and returns to search once. A second failure during the same drop requests takeover.

- [ ] **Step 5: Replace H fixed-time failure with one mode reset**

On persistent H loss, stop velocity, request `landing` mode once, clear jump/stability filters, and reacquire. If loss repeats, request takeover. Do not call PX4 LAND.

- [ ] **Step 6: Enable strict touchdown fallback on real hardware**

Rename `_sim_touchdown_confirmed()` to `_fallback_touchdown_confirmed()` and remove the `sim_mode` gate. Only evaluate after visual alignment is latched and blind descent started. Require altitude ≤0.10 m, horizontal speed ≤0.10 m/s, absolute vertical speed ≤0.05 m/s for at least 2.0 s. Primary PX4 `landed` remains checked first.

- [ ] **Step 7: Run all mission functional tests**

Run `python3 -m pytest -q src/drone_mission/test -k 'not flake8 and not pep257 and not copyright'`. Expected: all selected tests PASS.

- [ ] **Step 8: Commit**

```bash
git add src/drone_mission
git commit -m "feat(mission): recover visual stages before takeover hold"
```

### Task 7: Idempotent Absolute-PWM Servo Retry

**Files:**
- Modify: `src/drone_servo2/src/servo_controller.cpp`
- Modify: `src/drone_servo2/config/servo_params.yaml`
- Modify: `src/drone_servo2/test/test_servo_source_contract.py`

**Interfaces:**
- Produces parameter: `command_retries: int = 1`
- Guarantees: ACK timeout resends the same `pwm_us`; `DropManager.drop_count_` advances only after accepted ACK

- [ ] **Step 1: Add failing source-contract tests**

Assert `command_retries` is declared and loaded, the retry loop calls `send_actuator_cmd(pwm_to_normalized(pwm_us))` with unchanged `pwm_us`, and rejection does not retry or advance the drop count.

- [ ] **Step 2: Verify tests fail**

Run `python3 -m pytest -q src/drone_servo2/test/test_servo_source_contract.py`. Expected: FAIL on missing retry parameter.

- [ ] **Step 3: Implement timeout-only retry**

Pass `command_retries` into `PX4CommandServo`. In `send_pwm_command(double pwm_us)`, perform at most `1 + command_retries_` attempts. A timeout starts the next attempt with the same absolute PWM. An explicit non-accepted terminal ACK returns false immediately. Update `current_pwm_` only after accepted ACK.

- [ ] **Step 4: Run servo tests and build**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/drone_servo2/test/test_servo_source_contract.py
colcon build --symlink-install --packages-select drone_servo2
```

Expected: tests PASS and build finishes successfully.

- [ ] **Step 5: Commit**

```bash
git add src/drone_servo2
git commit -m "fix(servo): retry the same absolute PWM after ACK timeout"
```

### Task 8: Independent test_mission Recovery and Climbing Return

**Files:**
- Modify: `src/test_mission/test_mission/test_forward_recon.py`
- Modify: `src/test_mission/launch/test_forward_recon.launch.py`
- Create: `src/test_mission/test/test_forward_recon_contract.py`
- Modify: `scripts/test_forward_recon.sh`

**Interfaces:**
- Produces independent enum state: `MissionState.HOLD_FOR_TAKEOVER`
- Produces independent first-fault and progress-monitor instances
- Produces launch default: `return_altitude=5.0`

- [ ] **Step 1: Write failing independent-flow tests**

Test indefinite `WAIT_ARMED`, true telemetry readiness in `INIT`, progress-based takeoff/transit/descent/recon/return, no waypoint skipping, no PX4 LAND/RTL timeout fallback, independent hold, and a single 3-D Home+5 m return command.

- [ ] **Step 2: Verify tests expose existing forced transitions**

Run `python3 -m pytest -q src/test_mission/test/test_forward_recon_contract.py`. Expected: FAIL on forced next-state, skip, RTL/LAND, and 4 m return assertions.

- [ ] **Step 3: Implement the independent state and monitors**

Do not import `MissionFSM` or `HoldForTakeoverState`. Implement equivalent semantics locally: first stall reissues the same command; second stall enters local hold; local hold sends zero velocity until RC leaves Offboard or disarms, then stops output and records failure.

- [ ] **Step 4: Remove unsafe test-flow fall-throughs**

Delete the 2-second fake telemetry readiness, forced progression after takeoff/transit/descent/return timeouts, recon waypoint skip, global recon RTL, H-search PX4 LAND, vision-loss PX4 LAND, and blind-descent PX4 LAND paths.

- [ ] **Step 5: Set return to 5 m everywhere**

Change Python, launch, shell script, help text, and logs to `return_altitude=5.0`. Preserve immediate diagonal return from the final recon waypoint.

- [ ] **Step 6: Run independent-flow tests**

Run `python3 -m pytest -q src/test_mission/test -k 'not flake8 and not pep257 and not copyright'`. Expected: all selected tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/test_mission scripts/test_forward_recon.sh
git commit -m "fix(test-mission): recover locally and return while climbing to 5m"
```

### Task 9: Real-v1/SITL-v4 Deployment Contract Regression

**Files:**
- Modify: `src/drone_bringup/test/test_px4_interface_deployment_contract.py`
- Modify: `src/drone_servo2/test/test_servo_source_contract.py`
- Modify: `docs/REAL_HARDWARE_DEBUG_GUIDE.md`

**Interfaces:**
- Verifies real canonical topics and SITL-only override

- [ ] **Step 1: Add explicit deployment assertions**

Assert source/default YAML and real launch use `/fmu/out/vehicle_status_v1`, servo subscribes `/fmu/out/vehicle_command_ack`, SITL uses `/fmu/out/vehicle_status_v4`, and no real script sources `sitl_env.sh`.

- [ ] **Step 2: Run deployment tests**

Run `python3 -m pytest -q src/drone_bringup/test/test_px4_interface_deployment_contract.py src/drone_servo2/test/test_servo_source_contract.py`. Expected: all tests PASS.

- [ ] **Step 3: Document the boundary**

Add the exact real/SITL topic table and state that real behavior wins when documentation, generated messages, or SITL assumptions disagree.

- [ ] **Step 4: Commit**

```bash
git add src/drone_bringup/test/test_px4_interface_deployment_contract.py src/drone_servo2/test/test_servo_source_contract.py docs/REAL_HARDWARE_DEBUG_GUIDE.md
git commit -m "test(bringup): protect real-v1 and SITL-v4 topic boundaries"
```
