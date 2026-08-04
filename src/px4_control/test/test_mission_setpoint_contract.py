"""Contracts for mission position commands crossing the ROS relay boundary."""

from pathlib import Path
import inspect
import threading
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
INTERFACE_CMAKE = ROOT / "drone_interfaces" / "CMakeLists.txt"
MESSAGE = ROOT / "drone_interfaces" / "msg" / "NedPositionSetpoint.msg"


class MissionSetpointContractTest(unittest.TestCase):
    def test_ned_position_setpoint_exposes_position_yaw_and_optional_speed(self):
        self.assertTrue(MESSAGE.is_file())
        self.assertEqual(
            MESSAGE.read_text(encoding="utf-8").splitlines(),
            [
                "float64 north",
                "float64 east",
                "float64 down",
                "float64 yaw",
                "float64 speed",
                "bool use_speed",
            ],
        )
        self.assertIn(
            "msg/NedPositionSetpoint.msg",
            INTERFACE_CMAKE.read_text(encoding="utf-8"),
        )

    def test_flight_controller_forwards_speed_and_yaw_to_position_bridge(self):
        from px4_control.flight_controller import FlightController

        class Bridge:
            def get_parameter(self, _name):
                return type("Parameter", (), {"value": 5.0})()

            def set_position_target(self, *args, **kwargs):
                self.args = args
                self.kwargs = kwargs

        bridge = Bridge()
        FlightController(bridge).goto_ned(
            10.0, -2.0, -3.0, yaw=3.14, speed=0.8,
        )

        self.assertEqual(bridge.args, (10.0, -2.0, -3.0, 3.14))
        self.assertEqual(bridge.kwargs, {"speed": 0.8})

    def test_standalone_position_relay_accepts_an_optional_leg_speed(self):
        from px4_control.standalone_bridge import StandaloneBridge

        parameters = inspect.signature(StandaloneBridge.set_position_target).parameters

        self.assertIn("speed", parameters)
        self.assertIsNone(parameters["speed"].default)

    def test_standalone_heartbeat_publishes_position_yaw_and_speed_together(self):
        from px4_control.standalone_bridge import StandaloneBridge

        class Publisher:
            def publish(self, message):
                self.message = message

        bridge = StandaloneBridge.__new__(StandaloneBridge)
        bridge._shutdown = False
        bridge._velocity_mode = False
        bridge._target_ned = None
        bridge._target_yaw = 0.0
        bridge._target_speed = None
        bridge._autonomous_output_enabled = True
        bridge._output_lock = threading.RLock()
        bridge._sp_pub = Publisher()
        bridge._lease_pub = Publisher()
        bridge.set_position_target(10.0, -2.0, -3.0, yaw=3.14, speed=0.8)

        bridge._heartbeat()

        message = bridge._sp_pub.message
        self.assertEqual((message.north, message.east, message.down), (10.0, -2.0, -3.0))
        self.assertEqual(message.yaw, 3.14)
        self.assertEqual(message.speed, 0.8)
        self.assertTrue(message.use_speed)

    def test_px4_bridge_restores_yaw_and_speed_from_position_relay(self):
        from px4_control.px4_bridge import PX4Bridge

        class NoopLock:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        message = type(
            "NedPositionSetpoint",
            (),
            {
                "north": 10.0,
                "east": -2.0,
                "down": -3.0,
                "yaw": 3.14,
                "speed": 0.8,
                "use_speed": True,
                # Existing Point-compatible fields keep this a behavior failure,
                # rather than an attribute error, before the migration.
                "x": 10.0,
                "y": -2.0,
                "z": -3.0,
            },
        )()
        bridge = PX4Bridge.__new__(PX4Bridge)
        bridge._setpoint_lock = NoopLock()
        bridge._velocity_mode = True
        bridge._target_speed = None
        bridge._autonomy_lease_enabled = True
        bridge._autonomy_lease_last_seen = time.monotonic()
        bridge._autonomy_lease_timeout_s = 0.30
        bridge._autonomy_target_received = False
        bridge._current_setpoint = None
        bridge._smoothed_yaw = 0.0

        bridge._cmd_setpoint_cb(message)

        self.assertFalse(bridge._velocity_mode)
        self.assertEqual(bridge._target_setpoint.north, 10.0)
        self.assertEqual(bridge._target_setpoint.east, -2.0)
        self.assertEqual(bridge._target_setpoint.down, -3.0)
        self.assertEqual(bridge._target_yaw, 3.14)
        self.assertEqual(bridge._target_speed, 0.8)

    def test_position_ramp_uses_the_lower_of_leg_speed_and_bridge_limit(self):
        from px4_control.px4_bridge import PX4Bridge
        from px4_control.telemetry import Position

        class NoopLock:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        bridge = PX4Bridge.__new__(PX4Bridge)
        bridge._target_setpoint = Position(10.0, 0.0, 0.0)
        bridge._current_setpoint = Position(0.0, 0.0, 0.0)
        bridge._target_yaw = 0.0
        bridge._smoothed_yaw = 0.0
        bridge._target_speed = 0.8
        bridge._setpoint_lock = NoopLock()
        bridge.get_parameter = lambda _name: type("Parameter", (), {"value": 2.0})()
        bridge._autonomy_lease_enabled = True
        bridge._autonomy_lease_last_seen = time.monotonic()
        bridge._autonomy_lease_timeout_s = 0.30
        bridge._autonomy_target_received = True
        bridge._velocity_mode = False

        bridge._control_loop()

        self.assertAlmostEqual(bridge._current_setpoint.north, 0.04)
        self.assertAlmostEqual(bridge._current_setpoint.east, 0.0)
        self.assertAlmostEqual(bridge._current_setpoint.down, 0.0)

    def test_mission_speed_policy_has_one_owner_and_validates_all_leg_speeds(self):
        mission_yaml = (
            ROOT / "drone_mission/config/mission_params.yaml"
        ).read_text(encoding="utf-8")
        mission_source = (
            ROOT / "drone_mission/drone_mission/mission_state_machine.py"
        ).read_text(encoding="utf-8")
        bridge_yaml = (
            ROOT / "px4_control/config/px4_params.yaml"
        ).read_text(encoding="utf-8")
        bridge_source = (
            ROOT / "px4_control/px4_control/px4_bridge.py"
        ).read_text(encoding="utf-8")

        self.assertIn("sweep_speed: 1.2", mission_yaml)
        self.assertNotIn("max_acceleration", mission_yaml)
        self.assertNotIn("sweep_passes", mission_yaml)
        self.assertIn('(\"sweep_speed\", 0.1, 15.0)', mission_source)
        self.assertNotIn('declare_parameter("max_acceleration"', mission_source)
        self.assertNotIn('declare_parameter("sweep_passes"', mission_source)
        self.assertNotIn("max_acceleration", bridge_yaml)
        self.assertNotIn("sweep_speed:", bridge_yaml)
        self.assertNotIn("transit_speed:", bridge_yaml)
        self.assertNotIn("return_speed:", bridge_yaml)
        self.assertIn("max_setpoint_speed_limit: 5.0", bridge_yaml)
        self.assertNotIn('declare_parameter("sweep_speed"', bridge_source)
        self.assertNotIn('declare_parameter("transit_speed"', bridge_source)
        self.assertNotIn('declare_parameter("return_speed"', bridge_source)

    def test_operator_docs_describe_the_speed_control_contract(self):
        workspace = ROOT.parent
        audit = (workspace / "docs/recon_speed_parameter_audit.md").read_text(
            encoding="utf-8",
        )
        architecture = (workspace / "docs/ARCHITECTURE.md").read_text(
            encoding="utf-8",
        )
        reading_guide = (workspace / "docs/CODE_READING_GUIDE.md").read_text(
            encoding="utf-8",
        )

        self.assertNotIn("sweep_speed` 改多少都不会影响", audit)
        self.assertIn("max_setpoint_speed_limit", audit)
        self.assertIn("sweep_speed: 1.2", audit)
        self.assertIn("NedPositionSetpoint", architecture)
        self.assertIn("NedPositionSetpoint", reading_guide)

    def test_standalone_bridge_publishes_the_typed_position_relay(self):
        source = (ROOT / "px4_control/px4_control/standalone_bridge.py").read_text(
            encoding="utf-8",
        )

        self.assertIn("NedPositionSetpoint, \"/drone/cmd_setpoint_ned\"", source)

    def test_recon_coverage_waypoints_keep_a_fixed_heading(self):
        from drone_mission.path_planner import PathPlanner

        waypoints = PathPlanner(
            sweep_speed=0.8,
            grid_spacing=1.67,
        ).plan_coverage_ned(
            area_north_min=55.0,
            area_north_max=60.0,
            area_east_min=-4.0,
            area_east_max=4.0,
            altitude=-2.5,
        )

        self.assertTrue(waypoints)
        self.assertTrue(all(waypoint["yaw"] == 0.0 for waypoint in waypoints))


if __name__ == "__main__":
    unittest.main()
