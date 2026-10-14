# Autonomous Drone Mission System

A fully autonomous multirotor mission system built on **ROS 2 + PX4 Offboard**: a single
vehicle performs two precision payload drops, full grid coverage of a reconnaissance zone,
return-to-home, and vision-based precision landing. Everything end to end — flight-controller
integration, vision inference, mission state machine, real-hardware deployment, and flight
forensics — built by one person.

*[中文版 →](README.md)*

**Status**: the full mission has flown successfully in both SITL and on real hardware;
reliability is still converging. Every failure across 12 flight days has been traced to a
root cause — the taxonomy lives in **[Flight Failure Mode Analysis](docs/FAILURE_MODES.md)**,
which says more about this project than this file does.

---

## At a glance

| | |
|---|---|
| **Size** | 33,000 lines of Python (13,000 of them tests) · 10 ROS 2 packages · 510 test functions |
| **Stack** | ROS 2 Humble · PX4 1.17 Offboard · ONNX Runtime / TensorRT · Gazebo Harmonic |
| **Vehicle** | Jetson Orin Nano · CUAV 7-Nano · USB camera · PWM servo (MAIN5) |
| **Flight record** | 12 full days of flight logs (2026-07-28 – 08-15) plus further sorties in Sep/Oct; every ULog archived (not committed — available on request) |
| **Method** | Contract tests · ULog forensics · primary sources first — reading PX4 / ArduPilot / MAVLink directly rather than tuning by intuition |

**Mission flow**

```text
INIT → ARM → TAKEOFF → TRANSIT_TO_DROP_ZONE → CYLINDER_SEARCH → SELECT_TARGET
 → CYLINDER_APPROACH → DESCEND_TO_DROP → EXECUTE_DROP → ASCEND
 →(after 2 drops) TRANSIT_TO_RECON_ZONE → RECON_PATROL → RETURN_TO_HOME
 → PRECISION_LAND → COMPLETE
```

Any state fault, RC loss, critical battery event, or vision timeout falls through to
`HOLD_FOR_TAKEOVER` or `ERROR`: autonomous output stops and control returns to the pilot.

**Runtime topology**

```text
         Camera (Gazebo bridge / USB V4L2)
                    │ Image
                    ▼
             drone_vision              drone_mission
        ├─ cylinder detection (YOLO+shape refine) │  single FSM, 20 Hz orchestration
        ├─ H-marker detection (YOLO)              │
        ├─ coverage monitor                       ├─ position/velocity setpoints ──► px4_control ──► PX4
        └─ vision measurement timestamps ─────────►│                                    (Offboard/DDS)
                                                  └─ TriggerDrop ──► drone_servo2 ──► MAIN5 PWM
```

---

## The three hardest problems

This is the part of the README that matters. Scale is not the hard part here — these are.

### 1. A camera-extrinsics defect that no unit test could ever catch

**Symptom**: on the first real flight (2026-08-05), drop-target world coordinates were
systematically biased ~1.3–1.7 m north. One sortie had its target rejected by the drop-zone
world gate; another descended to 2 m with the target already out of frame.

**Why it survived so long**: the coordinate-transform matrix for the
`image_top_faces_forward=False` branch,

```
[[0, 1, 0],
 [1, 0, 0],
 [0, 0, 1]]     det = −1  ← this is a reflection, not a rotation
```

is a reflection. Reflection matrices satisfy `R² = I`, so `pixel_to_world` and
`world_to_pixel` **round-trip perfectly** — every "transform there and back" test passes.
And a reflection has *zero* error at the image centre, which is exactly where the existing
test case sat (`u = 640.0`), neatly stepping over the one axis that was wrong.

**How it was pinned down**: five independent lines of evidence converged — the sign of the
determinant; an offline recomputation (0.029 m under the correct convention vs 3.199 m under
the wrong one); the pre-existing servo sign conventions; a **ground-truth inversion over
1234 frames** requiring no calibration, which locked both the forward and right axes at once
(96% sign agreement, 1.3% dispersion); and the fact that the Gazebo model and the real camera
are mounted the same way. After the fix, the quick path's first full SITL run returned
`success: True`, with the locked world coordinate **2.8 cm** from ground truth.

> **Transferable lesson**: for extrinsics like this — where there is no physical intuition to
> lean on — you need a test written *from the physics* (a target to the vehicle's front-right
> must land in the image's upper right), never one whose expected values were read back out of
> the current implementation.

### 2. Silent failures: every health signal was green, the aircraft just wouldn't move

Three separate cases, one shared shape — **no error, no watchdog trip, and a perfectly legal
command as far as the flight controller could tell**:

| Case | Surface symptom | Actual mechanism |
|---|---|---|
| Sticky velocity mode (Oct 7) | Hovers in place for 20 s after takeoff instead of flying out | One zero-velocity command permanently flips the Offboard mode bit; `TrajectorySetpoint.position` is NaN for the rest of the flight. The OCM heartbeat runs at a clean 10 Hz with zero drops |
| Watchdog watching the wrong signal | Cylinder detection can die completely without an alarm | The `"vision"` channel is fed by **every** `landing_target` message (missed frames publish `confidence=0.0` too), so any camera frame keeps it green |
| Observation-age gate starving the controller | Looks like "the detector is broken" | The gate must exceed *one frame interval + pipeline latency*; sized too small it drops the probability of three consecutive converged frames to ~2% — structurally unable to converge |

You cannot diagnose these by reading health signals. You reconstruct them from the flight log:
first *when* the `offboard_control_mode` position/velocity bits flip one-way, then whether
`trajectory_setpoint.position` is NaN. That chain is far faster than staring at the vehicle's
position.

### 3. A physical problem that crossed the entire control stack and killed the land detector

**Symptom**: the pilot reported "attitude is very unstable", and was eventually forced to hit Kill.

**Root cause**: mechanical vibration. `gyro_vibration_metric` reached 4.9 against a PX4
threshold of 0.02 — **245×** — at 80 Hz, with amplitude growing 8.7× in a single day.

**The causal chain** (this is the part worth reading): 80 Hz vibration fed straight into the
rate loop → motor outputs oscillating between 0 and 1 (`torque_setpoint_achieved` missed 60.9%
of the time) → the controller commanding **±40° of roll setpoint** back and forth while the
airframe barely moved (tracking-error std 23.65°) → yet **RC stick input was essentially
flat** (roll stick std 0.088), so *the oscillation was generated by the controller itself,
not by the pilot* → and finally it hit the land detector: disarming requires
`LNDMC_ROT_MAX = 20 °/s`, while actual angular rate sat at a median of 40.19 °/s, over
threshold 72.8% of the time → `maybe_landed` / `landed` stuck at 0 → **disarm denied three
times, pilot forced to Kill**. The moment the motors stopped, angular rate fell to 0.30 °/s
and landing was detected immediately — which is what closes the loop on the whole chain.

**And a trap in reading the data**: the `vehicle_angular_velocity` topic logs at only
49.45 Hz, which **aliases** the 80 Hz vibration down to 19 Hz — briefly misread as a
"19.73 Hz limit cycle". Switching to `sensor_combined` at 197.8 Hz put the peak back at
exactly 80.04 Hz. **Check the sample rate and Nyquist before interpreting any spectrum.**

> The shared lesson: a system's behaviour and its health signals are two different things.
> Green watchdogs, legal commands, and self-consistent round-trip tests are not evidence
> that anything is working.

---

## Verification status

Being explicit about what is and isn't proven:

| | Status | Evidence |
|---|---|---|
| Full mission (SITL) | ✅ Passes | 2026-07-30, `mission_end.success=True`, `drops=2/2`, coverage `1.0`, 199.95 s |
| Quick strategy (SITL) | ✅ Passes | 2026-09-25, 125.75 s, locked world coordinate within 2.8 cm |
| **Full mission (real hardware)** | ✅ **Has succeeded** | 2026-08-14 night, Run 3: 135.5 s, drop 1/1 + recon 100% + vision landing complete |
| Real-hardware reliability | ⚠️ **Not converged** | Only 1 of 5 sorties that night completed the full mission; the rest stalled at the drop zone (3/5) or in recon |
| Precision landing (authority-based rework) | 🚧 In progress | Stages 1 / 2a / 2c landed; no full SITL or real-flight re-verification yet |

The unfinished parts aren't untouched — they're converging. Failure classification, evidence,
and fix status: [docs/FAILURE_MODES.md](docs/FAILURE_MODES.md).

---

## Engineering practices

**Contract tests (510 functions — not a coverage number).** These test interface promises,
not lines: `test_flight_safety_contract`, `test_px4_safety_contract`,
`test_mission_setpoint_contract`, `test_operator_takeover_contract`, plus bidirectional
parameter contracts. Two of the parameter-type contracts were **validated by injecting the
real violation and confirming they fail** — they correspond to two classes of real flight
failures that killed nodes outright (rclpy matches parameter types strictly, so writing `35`
instead of `35.0` in YAML makes the FSM die on startup and the aircraft never takes off).

**A ULog flight-forensics toolchain.** Fetching logs directly over MAVLink FTP with ULog-magic
and non-zero-byte verification (because QGC **silently produces all-zero files** on a poor
link); state-change timelines; Offboard mode-bit lifetime; event decoding. The 9/27 vibration
analysis and the 10/7 sticky-velocity diagnosis both came out of this chain.

**Primary sources first.** Before reworking precision landing, the project ran a round of
first-hand research: the MAVLink `LANDING_TARGET` measurement contract, PX4's
`landing_target_estimator` and VTE, PX4's `navigator/precland`, ArduPilot's `AC_PrecLand`,
plus the IBVS/PBVS canon. That produced five points of consensus and **one prohibition**:
there is no D term on raw error anywhere in the canon. The only canonical identity for an
error derivative is a *filtered estimate of target motion*, injected as **feedforward**;
velocity information comes from estimator state, not from differencing measurements. This
directly overturned the planned basis for tuning kd.

**Willing to retract its own conclusions.** Commit `de56e66`: *"retract 'gate reduces 70–90%'
(simulation clock artifact)"* — the desk simulation ran ~1760× faster than real time, giving
the filter `dt ≈ 2e-5`, and the reported gain vanished once a `SimClock` was wired in. Around
the same time, a premise that had been cited for months — "vision runs at 3–5 Hz, τ ≈ 0.2–0.33 s"
— measured **10.5 Hz / τ 95 ms** on real hardware, and the `FPS` field in the logs turned out
to be the *reciprocal of a single frame's duration*, not a frame rate. Corrected in 20 places
across comments and docs.

**CI: a pipeline that never once passed.** GitHub Actions + `ros-tooling/action-ros-ci`
(Ubuntu 22.04 / ROS 2 Humble) went in on 2026-07-15 and came out on 2026-10-14, with
**all 10 runs failing**. The investigation's conclusion: the build was never the problem —
all 9 packages built cleanly, `px4_msgs` included. Every failure was in the test phase:
`test_flake8` scans the build directory under CI and reported 1039 violations where the
handwritten source measures 19; 369 of `test_pep257`'s 760 are "missing docstring", a style
convention rather than a defect; and a collection error in 4 packages would not reproduce
under the same `colcon test` locally.

**A gate that is always red carries exactly as much information as one that is always
green** — the mirror image of the "all health signals green" cases above: those were false
negatives, this was a false positive. If it is ever rebuilt, the minimal set should be real
tests only, without the ament boilerplate lint. Run history remains on the Actions page.

---

## Build and run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# unit tests
python3 -m pytest -q src/drone_mission/test src/drone_vision/test src/px4_control/test
```

**Full simulation** needs PX4 SITL, Gazebo Harmonic, MicroXRCEAgent, and the ONNX models
referenced by the config under `src/drone_vision/drone_vision/models/`:

```bash
./scripts/start_full_simulation.sh              # main mission
./scripts/start_quick_simulation.sh             # quick strategy
./scripts/start_full_simulation.sh layout_heading_deg:=50   # off-north field
```

**Real hardware**: `./scripts/start_full_real_mission.sh`. Run
`./scripts/verify_real_flight_ready.sh` before takeoff for a read-only preflight check — it
only queries the ROS graph and bounded topic samples; it never publishes, calls a service, or
changes vehicle state.

**Deployment discipline**: on the Jetson, any model or parameter change must go through a
branch commit and a rebuild. Two sorties failed to take off because of stale parameters in
`build/install`, or an engine format out of sync with its loader. On real hardware,
"changed but not in effect" is harder to find than a crash.

---

## Repository layout

| Package | Responsibility |
|---|---|
| `drone_interfaces` | Custom ROS 2 interfaces (`NedPositionSetpoint` / `CylinderArray` / `LandingTarget` / `CoverageStatus` …) |
| `px4_control` | PX4 DDS adaptation and command output: Offboard heartbeat, mode bits, safety gates |
| `drone_vision` | Camera, ONNX/TensorRT inference, target refinement, coverage monitoring |
| `drone_mission` | Mission FSM, per-state behaviour, path planning, precision-landing controller |
| `drone_servo2` | C++ multi-position drop control (SITL mock / real PX4 actuator) |
| `drone_utils` | Field coordinate frames, pose history, landing estimator, watchdog, config loading |
| `drone_bringup` | SITL / real × main / quick launch combinations |
| `drone_servo` | Python reference implementation of the drop controller (mirrored from the C++; `drone_servo2` is what deploys) |
| `test_mission` | Staged real-flight test scripts |
| `px4_msgs` | ROS 2 definitions of the PX4 uORB messages (vendored upstream, not this project's code) |

**Documentation map** (suggested reading order):

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — current implementation, coordinate chain, data flow
- [docs/FAILURE_MODES.md](docs/FAILURE_MODES.md) — **flight failure-mode taxonomy and analysis**
- [docs/PRECISION_LANDING_AUTHORITY.md](docs/PRECISION_LANDING_AUTHORITY.md) — authority review and gap table for precision landing
- [docs/CODE_READING_GUIDE.md](docs/CODE_READING_GUIDE.md) — file-by-file guide and runtime call chains
- [docs/Debug/REAL_HARDWARE_DEBUG_GUIDE.md](docs/Debug/REAL_HARDWARE_DEBUG_GUIDE.md) — on-site real-flight checklist

*(Currently in Chinese.)*

## Known limitations

- The control loop and vision inference are both Python. Measured on real hardware: vision
  ≈10.5 Hz, end-to-end latency τ ≈ 95 ms. Real-time behaviour is held together by parameter
  tuning, not guaranteed by the architecture.
- The field coordinate frame currently hangs off the PX4 local NED origin (= the EKF
  initialisation point), and the EKF origin latches at **power-on**. Since the aircraft is
  powered up and then carried to the takeoff point, this shifts the entire drop zone — with no
  error reported. A located, unfixed design defect.
- **No automated regression**: CI is gone (see above) and there is no SITL regression either.
  The cost of validating a change is still one real flight.

## About

Single-author project (245 of the repository's 249 commits). Real hardware uses the `jetson`
branch as the deployment baseline and `Optimization` as the development branch, with
`scripts/check_jetson_sync.sh` preventing the two from drifting.

> For the full failure-mode taxonomy, evidence, and fix status, see
> **[docs/FAILURE_MODES.md](docs/FAILURE_MODES.md)**.
