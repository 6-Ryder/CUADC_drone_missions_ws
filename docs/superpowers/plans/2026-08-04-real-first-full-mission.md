# Real-First Full Mission Implementation Plan

> **For agentic workers:** Execute one task at a time with `superpowers:subagent-driven-development`; commit each completed task only after its stated focused verification passes.

**Goal:** Deliver the one real-hardware full mission path `scripts/start_full_real_mission.sh` → `real_flight.launch.py` → `MissionFSM`. It must complete two confirmed drops, every reconnaissance waypoint, a direct 5 m climb-home return from the final reconnaissance waypoint, and H-guided touchdown confirmation.

**Architecture:** `MissionFSM` remains the sole full state machine. `mission_params_real.yaml` supplies all full-mission movement, visual-control, camera-geometry and recovery values. The existing real launch keeps the already flight-tested camera/vision constants (640×480, `fx=fy=400`, `cx=320`, `cy=240`) but no longer overrides equivalent `MissionFSM` values. No file in `drone_vision`, no camera calibration setup, no SITL camera bridge, and no `test_mission` file is changed.

**Tech stack:** ROS 2 Humble, Python/rclpy, PX4 v1 `px4_msgs`, YAML, bash, pytest, colcon, MicroXRCEAgent.

## Global constraints

- Do not change `src/test_mission/` or `scripts/test_forward_recon.sh`; it is the untouched, independently verified test path.
- Real v1 is canonical: `/fmu/out/vehicle_status_v1`, `/fmu/out/battery_status_v1`, and `/fmu/out/vehicle_command_ack`. v4 stays in the isolated SITL environment and launch only.
- Do not modify `src/drone_vision/`, camera calibration files, or the SITL camera bridge. Preserve launch's established 640×480 and `400/400/320/240` values.
- The real profile uses the proven starting values: takeoff 5.0 m, waypoint tolerance 0.5 m, takeoff tolerance 0.3 m, 20 Hz, transit 2.0 m/s, recon 3.0 m, sweep 0.8 m/s, grid 1.67 m, return 3.8 m/s, H `kp=0.004`, H max 0.6 m/s, 12 px dead zone, signs `+1/-1`, descent `0.4/0.25/0.12` m/s, vision loss 3.0 s, and camera 640×480 with `400/400/320/240`.
- Full-only configuration (two drops, zone boundaries, target search/alignment) remains in the real profile. Full return altitude is 5.0 m; `test_mission` keeps its 4.0 m behavior unchanged.
- No required recon waypoint, successful-drop count, or touchdown confirmation may be skipped by a fixed timeout. One same-action retry is allowed; then enter zero-velocity `HOLD_FOR_TAKEOVER`, never task-level RTL/LAND.
- PX4 native failsafes remain active. Autonomous output stops immediately when PX4 leaves Offboard.
- Scripts do not `pkill`, source SITL, arm, switch mode, publish flight setpoints, call a drop service, command RTL/LAND, or drive a servo.

## Task 1: Real profile and launch/FSM value unification

**Files**

- Create `src/drone_mission/config/mission_params_real.yaml`.
- Modify `src/drone_bringup/launch/real_flight.launch.py`.
- Modify package installation metadata only if it does not already install `config/*.yaml`.
- Create `src/drone_bringup/test/test_real_full_mission_profile.py`.

**Steps**

1. Write a failing source/profile contract test. It must assert the new file has `/mission_state_machine.ros__parameters`, v1 status topic, every proven value listed above, `camera_fx/fy: 400.0`, `camera_cx: 320.0`, `camera_cy: 240.0`, `precision_align_image_width/height: 640/480`, and `return_altitude: 5.0`.
2. Copy the complete existing `mission_params.yaml` mapping into `mission_params_real.yaml`. Preserve full-only area/drop/alignment keys. Replace common real values with the stated baseline and add `return_altitude: 5.0`; do not remove MissionFSM camera parameters.
3. Make `real_flight.launch.py` default its mission parameters to the installed real profile and pass it to the mission node. Remove launch overrides of all movement, visual-control and camera geometry values for that node; preserve only deployment topology overrides (camera device, DDS port, Home, axis, zone distances, RC-arm requirement).
4. Keep existing camera-node and vision-node 640×480/`400/400/320/240` parameters untouched. The test must assert these launch values and the real-profile values match, and must assert the mission node does not duplicate them.
5. Run `python3 -m pytest -q src/drone_bringup/test/test_real_full_mission_profile.py` after sourcing ROS and `install/setup.bash`; then run the existing bringup launch contracts. Commit `feat(real): unify full mission parameter profile`.

## Task 2: Deterministic progress monitor and PX4 output lifecycle

**Files**

- Create `src/drone_utils/drone_utils/progress_monitor.py` and `src/drone_utils/test/test_progress_monitor.py`.
- Modify `src/px4_control/px4_control/standalone_bridge.py` and `src/px4_control/test/test_px4_safety_contract.py`.

**Steps**

1. Add failing pure-Python tests for `ProgressMonitor(window_s, min_improvement)`, including continuing progress, one retry marker, and second no-progress detection. The monitor must receive `now` explicitly, not read ROS or wall time.
2. Add failing bridge tests for separate status/odometry freshness timestamps, `telemetry_fresh(now, max_age_s=2.0)`, `hold_for_takeover()` (zero velocity but outputs enabled), and `stop_autonomous_output()` (clears position target and disables later output).
3. Implement the monitor and bridge APIs. Every position/velocity publisher must no-op when autonomous output is disabled. Commit `feat(real): add progress and output lifecycle controls` after focused tests pass.

## Task 3: Indefinite readiness and operator-takeover state

**Files**

- Create `src/drone_mission/drone_mission/states/state_hold_for_takeover.py` and tests.
- Modify `mission_state_machine.py`, `states/state_init.py`, and existing mission safety tests.

**Steps**

1. Write failing tests that show INIT waits beyond 30 seconds with explicit missing prerequisites and that all terminal task faults route to `HOLD_FOR_TAKEOVER`, not `ERROR`, RTL, or LAND.
2. Add `request_takeover(reason)`: retain the first fault, mark the mission unsuccessful, command the bridge hold, and transition into the latched state. The state publishes zero velocity while armed and in Offboard; RC takeover, disarm, or an Offboard exit calls `stop_autonomous_output()` and ends the state.
3. Remove the fixed 30-second INIT error. Gate it on fresh v1 telemetry, odometry, battery, land state, vision readiness, servo readiness, and RC arm signal where configured; log the missing names every five seconds. Existing camera/vision code stays unmodified.
4. Commit `feat(real): wait for readiness and hold for operator takeover` after focused mission tests pass.

## Task 4: Complete-work recovery and direct 5 m return

**Files**

- Modify `state_takeoff.py`, `state_transit.py`, `state_cylinder_search.py`, `state_cylinder_approach.py`, `state_drop.py`, `state_recon_patrol.py`, `state_return.py`, and `state_precision_land.py`.
- Modify existing recovery/coverage/drop tests and add `test_full_real_recovery_contract.py`.

**Steps**

1. Add failing tests for first stalled action resending/reacquiring exactly the same work, second stall requesting takeover, no recon-index advance on a stall, no unconfirmed-drop increment, and no task-level RTL/LAND from a recovery path.
2. Attach one `ProgressMonitor` per current action. Use target-distance/altitude errors for navigation; preserve current waypoint and target on retry. For temporary vision loss, clear only the local lock and reacquire once. Insufficient targets repeat the same search path once before takeover.
3. Replace fixed timeout-to-error, waypoint-skip, and automatic-RTL/LAND paths. Normal work continues while it advances.
4. Change `ReturnState` to issue its first `goto_ned()` with Home North/East and `fsm.return_altitude()` (5.0 m) together, so the return begins directly at the final scan waypoint; do not introduce a separate climb state.
5. Keep PX4 land detection primary. Allow existing-data fallback only after visual alignment and blind descent, using altitude ≤0.10 m, horizontal speed ≤0.10 m/s, vertical speed ≤0.05 m/s sustained 2.0 s. Commit `fix(real): complete each mission action before takeover` after the mission suite passes.

## Task 5: Idempotent servo confirmation

**Files**

- Modify `src/drone_servo2/src/servo_controller.cpp`, `config/servo_params.yaml`, and `test/test_servo_source_contract.py`.

**Steps**

1. Test a `command_retries: 1` contract: timeout retries the same absolute PWM once; rejected ACK does not retry; `current_pwm` changes only after accepted ACK.
2. Implement the bounded retry and build `drone_servo2`. A drop state may count a drop only after this accepted result; exhausted result requests takeover. Commit `fix(real): retry unchanged PWM after servo ACK timeout`.

## Task 6: Safe launcher, read-only readiness verifier, and documents

**Files**

- Create `scripts/start_full_real_mission.sh`, `scripts/verify_real_flight_ready.sh`, and script tests.
- Update relevant deployment/architecture/parameter/risk documentation only with verified implementation facts.

**Steps**

1. The launcher creates a unique `log/real_mission_*` directory, sources ROS and workspace setup, rejects SITL contamination, checks dependencies/files/camera device, prints v1/RC instructions, and runs only `ros2 launch drone_bringup real_flight.launch.py "$@"`. It must not start a second MicroXRCEAgent because launch owns it.
2. The verifier must be read-only: require publishers and one bounded sample for v1 status, odometry, battery, land detection, unversioned ACK, camera info, and vision status; require vision ready and the drop service to exist without calling it. Check received `CameraInfo` directly against the established 640×480/`400/400/320/240` constants; do not load or introduce calibration files.
3. Test source safety, run `bash -n`, mark scripts executable, and document propellers-removed readiness, manual RC arm/takeover, logs, and staged field validation. Commit `feat(real): add safe full-mission launcher and readiness checks`.

## Task 7: Build gate, SITL semantic regression, and field handoff

**Steps**

1. Build `--packages-up-to drone_bringup`; run functional tests for drone utils, px4 control, mission, servo, and bringup. Vision tests are regression-only and its source remains untouched.
2. Run full SITL only to validate mission semantics: two accepted drops, every recon waypoint, direct Home+5 m return, H alignment, confirmed landing, and `success=True`. Do not change real values to fix SITL-only behavior.
3. With propellers removed, use launcher then verifier; stage field checks as vision detection, one accepted drop, two accepted drops, then the full mission. Preserve run directories and PX4 ULogs. Claim full real success only with evidence for all required outcomes.
