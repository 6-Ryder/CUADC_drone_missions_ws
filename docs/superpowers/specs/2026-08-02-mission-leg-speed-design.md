# Mission leg speed and yaw relay design

## Goal

Make the existing mission parameters `sweep_speed`, `transit_speed`, and
`return_speed` control their respective position-flight legs. Preserve the
separate velocity-control path used by visual alignment and precision landing.
The configured reconnaissance speed remains `0.8 m/s`.

## Context

The mission FSM, `simple_mission`, and `test_forward_recon` run in a separate
process from `px4_bridge`. They use `StandaloneBridge`, which currently relays
`/drone/cmd_setpoint_ned` as `geometry_msgs/Point`. A `Point` cannot carry the
`yaw` and `speed` accepted by `FlightController.goto_ned()`, so both values are
lost before `px4_bridge` sends its PX4 `TrajectorySetpoint`.

The velocity topics `/drone/cmd_velocity_ned` and `/drone/vision/servo_cmd`
are intentionally independent and remain unchanged.

## Selected design

Add `drone_interfaces/msg/NedPositionSetpoint.msg`:

```text
float64 north
float64 east
float64 down
float64 yaw
float64 speed
bool use_speed
```

The message replaces `geometry_msgs/Point` on `/drone/cmd_setpoint_ned`.
`use_speed=false` denotes a position target without a leg-speed request, which
is used by takeoff and loiter. `use_speed=true` is used by `goto_ned()` and
carries the mission phase speed and planned yaw atomically with the target.

`PX4Bridge` stores the target speed and yaw. Its existing 20 Hz position
setpoint ramp continues to publish position-mode `TrajectorySetpoint`s. The
per-tick step is limited by the requested leg speed and by a bridge-owned
global safety limit. This provides a nominal setpoint progression rate; PX4
limits, vehicle dynamics, and wind can make actual airframe speed lower.

Rename `max_acceleration` to `max_setpoint_speed_limit` in `px4_params.yaml`
and `PX4Bridge`. The current implementation constrains metres per control
period, i.e. a setpoint speed, not acceleration.

## Parameter ownership

`mission_params.yaml` owns phase policy:

- `sweep_speed: 0.8`
- `transit_speed`
- `return_speed`

`px4_params.yaml` owns bridge safety policy:

- `max_setpoint_speed_limit`

Remove the redundant speed parameters from `px4_params.yaml` and the unused
`max_acceleration` declaration/configuration from the mission FSM. PX4
`MPC_XY_VEL_MAX` remains an external, vehicle-level hard safety cap and is not
managed by ROS YAML.

## Compatibility and safety

- The position relay type changes atomically on both publisher and subscriber;
  mixed old/new processes are deliberately incompatible and must not be run.
- SITL PX4 message v4 and real-flight v1 remain unaffected because this is a
  project-local `drone_interfaces` message, not a PX4 DDS message.
- Velocity mode remains exclusive: a received position command switches the
  bridge back to position mode, exactly as today; visual servo/landing continue
  to use their existing velocity topics.
- A non-positive or invalid requested speed is rejected by parameter validation
  and defensively treated as no speed request by the bridge.

## Scope

Modify `drone_interfaces`, `px4_control`, mission configuration/validation,
and relevant architecture/audit documents. Update tests for the new relay
contract and add tests for speed/yaw propagation and the per-leg setpoint cap.
Do not alter `drone_vision`, `drone_servo2`, launch parameter interfaces, or
the PX4 v1/v4 overlay tooling.

## Verification

1. Unit tests prove that `goto_ned()` forwards yaw and speed to the bridge.
2. Unit tests prove the standalone relay publishes the new message and the PX4
   bridge restores target yaw/speed.
3. Unit tests prove a 20 Hz ramp cannot exceed the smaller of leg speed and
   bridge safety limit.
4. Build the full workspace so ROS generates the new interface for all
   dependents, then run package tests.
5. In SITL, inspect the relay and PX4 trajectory setpoints with speed values
   `0.8`, `2.0`, and `3.0`, then conduct a controlled real-flight validation.
