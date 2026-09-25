from pathlib import Path

import pytest

from drone_mission.states.state_drop import DropState
from drone_mission.states.state_precision_land import PrecisionLandState
from drone_mission.states.state_takeoff import TakeoffState
from px4_control.standalone_bridge import StandaloneBridge
from px4_control.telemetry import Position, Velocity


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _TaskLogger:
    def log_state(self, *_args):
        pass


class _Bridge:
    def __init__(self):
        self.offboard = False
        self.offboard_requests = 0
        self.armed = True
        self.land_commands = 0
        self.force_disarms = 0
        self.landed = False

    def is_offboard(self):
        return self.offboard

    def set_offboard_mode(self):
        self.offboard_requests += 1

    def land_at_current(self):
        self.land_commands += 1

    def is_landed(self):
        return self.landed

    def is_armed(self):
        return self.armed

    def get_telemetry(self):
        return type(
            "Telemetry",
            (),
            {"velocity": Velocity(vn=0.0, ve=0.0, vd=0.3)},
        )()

    def force_disarm(self):
        self.force_disarms += 1


class _FlightController:
    def __init__(self):
        self.takeoff_calls = 0
        self.loiter_calls = 0
        self.velocity_commands = []

    def takeoff(self, _height):
        self.takeoff_calls += 1

    def loiter(self):
        self.loiter_calls += 1

    def set_velocity_body(self, *command):
        self.velocity_commands.append(command)

    def stop_velocity(self):
        self.velocity_commands.append((0.0, 0.0, 0.0))

    origin_down = 0.0

    def altitude_to_ned(self, altitude_m):
        return self.origin_down - altitude_m

    def ned_to_altitude(self, ned_down):
        return self.origin_down - ned_down


class _FSM:
    def __init__(self):
        self.bridge = _Bridge()
        self.fc = _FlightController()
        self.logger = _TaskLogger()
        self.offboard_confirmations = 0
        self.takeovers = []

    def get_logger(self):
        return _Logger()

    def get_param(self, _name, default):
        return default

    def check_drop_service_result(self):
        return False

    def set_vision_mode(self, _mode):
        pass

    def mark_offboard_confirmed(self):
        self.offboard_confirmations += 1

    def request_takeover(self, reason):
        self.takeovers.append(reason)

    def loiter(self):
        self.fc.stop_velocity()


def test_takeoff_waits_for_confirmed_offboard(monkeypatch):
    fsm = _FSM()
    state = TakeoffState(fsm)
    state._sub_phase = state.SUB_OFFBOARD
    state._sub_phase_start = 0.0
    state._offboard_cmd_sent = True
    state._start_time = 0.0
    monkeypatch.setattr(
        "drone_mission.states.state_takeoff.time.time", lambda: 2.0,
    )

    assert state.execute() is None
    assert fsm.fc.takeoff_calls == 0

    fsm.bridge.offboard = True
    assert state.execute() is None
    assert fsm.fc.takeoff_calls == 1
    assert fsm.offboard_confirmations == 1


def test_takeoff_reports_control_loop_gap_while_climbing(monkeypatch):
    """A delayed mission callback must leave a diagnostic trace after recovery."""
    class _RecordingLogger:
        def __init__(self):
            self.errors = []

        def info(self, _message):
            pass

        def warn(self, _message):
            pass

        def error(self, message):
            self.errors.append(message)

    fsm = _FSM()
    logger = _RecordingLogger()
    fsm.get_logger = lambda: logger
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": 0.0})()
    state = TakeoffState(fsm)
    state._sub_phase = state.SUB_CLIMB
    state._start_time = 0.0
    state._last_control_tick_monotonic = 10.0
    monkeypatch.setattr(
        "drone_mission.states.state_takeoff.time.monotonic", lambda: 12.0,
    )
    monkeypatch.setattr(
        "drone_mission.states.state_takeoff.time.time", lambda: 2.0,
    )

    assert state.execute() is None
    assert any("control-loop gap" in message for message in logger.errors)


def test_takeoff_prestreams_a_hold_before_requesting_offboard(monkeypatch):
    fsm = _FSM()
    state = TakeoffState(fsm)
    monotonic = [0.0]
    monkeypatch.setattr("drone_mission.states.state_takeoff.time.time", lambda: 0.0)
    monkeypatch.setattr(
        "drone_mission.states.state_takeoff.time.monotonic", lambda: monotonic[0],
    )

    state.enter()
    assert fsm.fc.loiter_calls == 1

    # RC is already armed, but the required prestream has not elapsed.
    assert state.execute() is None
    assert state.execute() is None
    assert fsm.bridge.offboard_requests == 0

    monotonic[0] = 1.0
    assert state.execute() is None
    assert fsm.bridge.offboard_requests == 1


def test_takeoff_offboard_stall_retries_once_then_requests_takeover(monkeypatch):
    fsm = _FSM()
    state = TakeoffState(fsm)
    state._sub_phase = state.SUB_OFFBOARD
    state._sub_phase_start = 0.0
    state._offboard_cmd_sent = True
    state._start_time = 0.0
    now = [16.0]
    monkeypatch.setattr("drone_mission.states.state_takeoff.time.time", lambda: now[0])

    assert state.execute() is None
    assert fsm.takeovers == []

    now[0] = 32.0
    assert state.execute() is None
    assert fsm.takeovers == ["TAKEOFF: Offboard was not confirmed after retry"]


def test_drop_hardware_failure_stops_the_mission():
    fsm = _FSM()
    state = DropState(fsm)

    assert state._execute_drop() is None
    assert fsm.takeovers == ["DROP: servo did not confirm the release"]


def test_descent_ready_remains_latched_during_precision_alignment():
    state = DropState(_FSM())
    state._descent_ready_latched = True

    assert state._check_descent_ready(height_error=0.8)


def test_standalone_bridge_exposes_real_px4_velocity():
    node = type(
        "Node",
        (),
        {
            "create_subscription": lambda *args, **kwargs: None,
            "create_publisher": lambda *args, **kwargs: None,
            "create_timer": lambda *args, **kwargs: None,
        },
    )()
    bridge = StandaloneBridge.__new__(StandaloneBridge)
    bridge._node = node
    bridge._position = Position()
    bridge._velocity = Velocity()
    bridge._armed = True
    bridge._landed = False
    bridge._battery_remaining = 0.75
    bridge._battery_ready = True
    bridge._attitude = type("Attitude", (), {})()
    bridge._offboard = False
    bridge._current_yaw = 0.0

    odometry = type("Odometry", (), {
        "position": [0.0, 0.0, 0.0],
        "velocity": [1.25, -0.5, 0.08],
    })()
    bridge._odom_cb(odometry)

    telemetry = bridge.get_telemetry()
    assert telemetry.velocity == Velocity(vn=1.25, ve=-0.5, vd=0.08)


def test_flight_controller_takeoff_sets_same_target_for_altitude():
    """Verify FlightController.takeoff() sets a NED target preserving XY."""
    from px4_control.flight_controller import FlightController

    class _BridgeMock:
        def __init__(self):
            self._position = Position(north=1.0, east=2.0, down=0.0)
            self._target = None
            self._offboard = False
            self._armed = False

        def get_current_position(self):
            return self._position

        def set_position_target(self, north, east, down, yaw=0.0):
            self._target = (north, east, down)

        def is_offboard(self):
            return self._offboard

        def is_armed(self):
            return self._armed

        def set_offboard_mode(self):
            self._offboard = True

        def arm(self):
            self._armed = True

        def get_parameter(self, name):
            return type("Param", (), {"value": 5.0})()

    bridge = _BridgeMock()
    fc = FlightController(bridge)
    bridge._offboard = True
    bridge._armed = True
    fc.takeoff(5.0)

    assert bridge._target is not None
    assert bridge._target[0] == 1.0  # north preserved
    assert bridge._target[1] == 2.0  # east preserved
    assert bridge._target[2] == -5.0  # altitude=5 → NED down=-5


def test_flight_controller_uses_task_yaw_for_takeoff_loiter_and_default_goto():
    """Task yaw is used by position commands that do not override it."""
    from px4_control.flight_controller import FlightController

    class RecordingBridge:
        def __init__(self):
            self.position = Position(north=1.0, east=2.0, down=0.0)
            self.position_calls = []
            self.offboard = False

        def get_parameter(self, _name):
            return type("Parameter", (), {"value": 5.0})()

        def is_offboard(self):
            return self.offboard

        def set_offboard_mode(self):
            self.offboard = True

        def get_current_position(self):
            return self.position

        def set_position_target(self, north, east, down, yaw=0.0, speed=None):
            self.position_calls.append((north, east, down, yaw, speed))

    bridge = RecordingBridge()
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


def test_flight_controller_task_yaw_defaults_validates_and_allows_override():
    """Task yaw has a safe default and explicit waypoint yaw takes precedence."""
    from px4_control.flight_controller import FlightController

    class RecordingBridge:
        def __init__(self):
            self.position_calls = []

        def get_parameter(self, _name):
            return type("Parameter", (), {"value": 5.0})()

        def set_position_target(self, *args, **kwargs):
            self.position_calls.append((args, kwargs))

    bridge = RecordingBridge()
    default_fc = FlightController(bridge)
    assert default_fc.task_yaw == 0.0

    fc = FlightController(bridge, task_yaw=0.8726646259971648)
    assert fc.task_yaw == pytest.approx(0.8726646259971648)
    fc.goto_world(2.0, 3.0, 4.0)
    fc.goto_ned(10.0, 3.0, -5.0, yaw=-0.5)

    assert bridge.position_calls[-2] == ((3.0, 2.0, -4.0, 0.8726646259971648), {
        "speed": 1.0,
    })
    assert bridge.position_calls[-1] == ((10.0, 3.0, -5.0, -0.5), {"speed": 1.0})

    with pytest.raises(ValueError, match="task yaw must be finite"):
        FlightController(bridge, task_yaw=float("nan"))
    with pytest.raises(ValueError, match="task yaw must be finite"):
        FlightController(bridge, task_yaw=float("inf"))


def test_precision_land_blind_descent_stays_offboard_and_can_trigger_land_detector(
    monkeypatch,
):
    fsm = _FSM()
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -1.0})()
    fsm.bridge.get_telemetry = lambda: type(
        "Telemetry",
        (),
        {"velocity": Velocity(vn=0.0, ve=0.0, vd=0.3)},
    )()
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_BLIND
    state._blind_start_time = 1.0
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: 2.0,
    )

    assert state.execute() is None
    assert fsm.fc.velocity_commands[-1] == (0.0, 0.0, 0.3)
    assert fsm.bridge.land_commands == 0
    assert fsm.bridge.force_disarms == 0


def test_precision_land_reacquires_lost_h_once_then_requests_takeover(monkeypatch):
    fsm = _FSM()
    fsm.get_landing_target = lambda: None
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -1.0})()
    state = PrecisionLandState(fsm)
    state._has_landing_target = True
    state._vision_lost_start = 1.0
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: now[0],
    )
    now = [5.0]

    assert state.execute() is None
    assert fsm.takeovers == []
    assert fsm.fc.velocity_commands[-1] == (0.0, 0.0, 0.0)
    assert fsm.bridge.land_commands == 0
    assert fsm.bridge.force_disarms == 0

    state._has_landing_target = True
    state._vision_lost_start = 1.0
    now[0] = 9.0
    assert state.execute() is None
    assert fsm.takeovers == ["PRECISION_LAND: H target was lost after reacquire"]


def test_precision_land_sim_touchdown_requires_stable_window_then_disarms(
    monkeypatch,
):
    fsm = _FSM()
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -0.04})()
    fsm.bridge.get_telemetry = lambda: type(
        "Telemetry",
        (),
        {"velocity": Velocity(vn=0.01, ve=-0.01, vd=0.0)},
    )()
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_BLIND
    state._visual_alignment_latched = True
    state._blind_start_time = 1.0
    now = [10.0]
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: now[0],
    )

    assert state.execute() is None
    assert fsm.bridge.force_disarms == 0

    now[0] = 12.1
    assert state.execute() is None
    assert fsm.bridge.force_disarms == 1
    assert fsm.bridge.land_commands == 0

    fsm.bridge.armed = False
    assert state.execute() == "COMPLETE"


def test_precision_land_real_flight_does_not_use_local_altitude_fallback(
    monkeypatch,
):
    fsm = _FSM()
    fsm.get_param = lambda name, default: False if name == "sim_mode" else default
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -0.04})()
    fsm.bridge.get_telemetry = lambda: type(
        "Telemetry",
        (),
        {"velocity": Velocity(vn=0.0, ve=0.0, vd=0.0)},
    )()
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_BLIND
    state._blind_start_time = 1.0
    now = [10.0]
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: now[0],
    )

    assert state.execute() is None
    now[0] = 12.1
    assert state.execute() is None
    assert fsm.bridge.force_disarms == 0


def test_precision_land_real_fallback_requires_alignment_and_blind_descent(
    monkeypatch,
):
    fsm = _FSM()
    fsm.get_param = lambda name, default: False if name == "sim_mode" else default
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -0.04})()
    fsm.bridge.get_telemetry = lambda: type(
        "Telemetry",
        (),
        {"velocity": Velocity(vn=0.0, ve=0.0, vd=0.0)},
    )()
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_BLIND
    state._visual_alignment_latched = True
    state._blind_start_time = 1.0
    now = [10.0]
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: now[0],
    )

    assert state.execute() is None
    now[0] = 12.1
    assert state.execute() is None
    assert fsm.bridge.force_disarms == 1

    fsm.bridge.armed = False
    assert state.execute() == "COMPLETE"


def test_precision_land_keeps_the_lease_until_px4_confirms_final_disarm():
    fsm = _FSM()
    fsm.bridge.landed = True
    state = PrecisionLandState(fsm)

    assert state.execute() is None
    assert fsm.bridge.force_disarms == 1

    # A live zero-velocity setpoint must remain while the VehicleCommand is
    # in flight, so the relay lease cannot gate that final disarm command.
    assert state.execute() is None
    assert fsm.bridge.force_disarms == 1

    fsm.bridge.armed = False
    assert state.execute() == "COMPLETE"


def test_precision_land_does_not_resume_descent_while_final_disarm_is_pending():
    fsm = _FSM()
    fsm.bridge.landed = True
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -0.04})()
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_BLIND

    assert state.execute() is None
    assert fsm.bridge.force_disarms == 1

    # PX4's landed estimate must not be allowed to restart a descent after the
    # terminal disarm request has already been sent.
    fsm.bridge.landed = False
    assert state.execute() is None
    assert fsm.fc.velocity_commands[-1] == (0.0, 0.0, 0.0)
    assert fsm.bridge.force_disarms == 1


def test_precision_land_holds_position_on_a_single_large_target_jump():
    """A one-frame H misdetection must not create a lateral descent command."""
    fsm = _FSM()
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -3.0})()
    commands = []
    fsm.fc.set_velocity_body = lambda *command: commands.append(command)
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    target = {"center_u": 640.0, "center_v": 480.0, "confidence": 0.9}
    fsm.get_landing_target = lambda: target
    assert state.execute() is None

    # Near-ground H detections jumped by about 100 px in the failed SITL run.
    target = {"center_u": 740.0, "center_v": 480.0, "confidence": 0.9}
    assert state.execute() is None
    assert commands[-1] == (0.0, 0.0, 0.0)


def test_precision_land_uses_latched_visual_alignment_before_bad_close_frame():
    """Stable visual alignment must win over the detector's unreliable close range."""
    fsm = _FSM()
    # The successful visual window observed in SITL was 2.5m to 2.2m.
    altitude = [-2.4]
    target = [{
        "center_u": 640.0,
        "center_v": 480.0,
        "confidence": 0.9,
    }]
    fsm.bridge.get_current_position = lambda: type(
        "P", (), {"down": altitude[0]},
    )()
    fsm.get_landing_target = lambda: target[0]
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    for _ in range(10):
        assert state.execute() is None

    # At close range the detector degrades, but visual alignment was already
    # proven for 10 frames and must be handed to vertical blind descent before
    # this bad frame can command a lateral correction.
    altitude[0] = -1.49
    target[0] = {
        "center_u": 640.0,
        "center_v": 580.0,
        "confidence": 0.54,
    }
    command_count = len(fsm.fc.velocity_commands)
    assert state.execute() is None
    assert state._phase == state.PHASE_BLIND
    assert len(fsm.fc.velocity_commands) == command_count


def test_precision_land_rejects_low_confidence_h_detection():
    fsm = _FSM()
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -3.0})()
    fsm.get_landing_target = lambda: {
        "center_u": 700.0,
        "center_v": 540.0,
        "confidence": 0.54,
    }
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_SERVO

    assert state.execute() is None
    assert fsm.fc.velocity_commands[-1] == (0.0, 0.0, 0.0)


def test_precision_land_blind_phase_does_not_retrigger_vision_loss_fallback(
    monkeypatch,
):
    fsm = _FSM()
    fsm.get_landing_target = lambda: None
    fsm.bridge.get_current_position = lambda: type("P", (), {"down": -1.0})()
    commands = []
    fsm.fc.set_velocity_body = lambda *command: commands.append(command)
    state = PrecisionLandState(fsm)
    state._phase = state.PHASE_BLIND
    state._has_landing_target = True
    state._vision_lost_start = 1.0
    monkeypatch.setattr(
        "drone_mission.states.state_precision_land.time.time", lambda: 5.0,
    )

    assert state.execute() is None
    assert commands[-1] == (0.0, 0.0, 0.3)
    assert fsm.bridge.land_commands == 0


def test_px4_bridge_arm_does_not_use_force_magic():
    """PX4Bridge (direct DDS) uses standard arm for real hardware."""
    source = (SOURCE_ROOT / "px4_control/px4_control/px4_bridge.py").read_text(
        encoding="utf-8",
    )
    arm_source = source.split("    def arm(self):", 1)[1].split(
        "    def disarm(self):", 1,
    )[0]
    assert "21196" not in arm_source


def test_px4_telemetry_uses_versioned_status_topics():
    """Verify the standalone bridge receives the deployment-selected status topic."""
    bridge_py = SOURCE_ROOT / "px4_control/px4_control/standalone_bridge.py"
    source = bridge_py.read_text(encoding="utf-8")

    # 状态话题由 vehicle_status_topic 参数选择 (部署方指定 vN), 不硬编码版本.
    assert 'node.get_parameter("vehicle_status_topic")' in source
    assert '"/fmu/out/battery_status_v1"' in source


def test_px4_telemetry_facade_exists_before_init_state_runs():
    fsm_py = PACKAGE_ROOT / "drone_mission/mission_state_machine.py"
    source = fsm_py.read_text(encoding="utf-8")
    lazy_init = source.split("    def _lazy_init(self):", 1)[1].split(
        "    def _init_vision_subscriptions(self):", 1,
    )[0]

    assert lazy_init.index("_ = self.bridge") < lazy_init.index(
        "self._transition_to(MissionState.INIT)"
    )


def test_navigation_timeouts_do_not_advance_the_mission():
    transit_py = PACKAGE_ROOT / "drone_mission/states/state_transit.py"
    transit = transit_py.read_text(encoding="utf-8")
    ascend = (PACKAGE_ROOT / "drone_mission/states/state_drop.py").read_text(
        encoding="utf-8",
    ).split("def _execute_ascend", 1)[1]

    assert "超时, 强制进入下一状态" not in transit
    assert 'return "ERROR"' not in transit
    assert "request_takeover" in transit
    assert 'return "SELECT_TARGET" if drops' not in ascend


def test_landing_timeout_never_reports_success_or_force_disarms():
    """
    超时路径用 land_at_current 而非 force_disarm.

    force_disarm 出现在 land() 中是正确设计 — 着陆检测成功后 PX4 需要
    force_disarm(magic=21196) 才能上锁, 否则会被"非手动 disarm"拒绝.
    本测试仅验证超时路径的安全契约: 15s 超时后切入 PX4 LAND 模式,
    不调用 force_disarm, 也不返回 True.
    """
    source = (
        SOURCE_ROOT / "px4_control/px4_control/flight_controller.py"
    ).read_text(encoding="utf-8")
    land_method = source.split("    def land(self)", 1)[1].split(
        "    def force_disarm", 1,
    )[0]

    # 正常着陆路径包含 force_disarm — 这是正确行为
    assert "force_disarm" in land_method, \
        "land() 必须在着陆检测成功后调用 force_disarm"

    # 超时路径: 只使用 land_at_current, 不使用 force_disarm
    _, _, after_descending_branch = land_method.partition(
        "self._bridge.force_disarm()"
    )
    assert "land_at_current" in after_descending_branch, \
        "超时路径必须调用 land_at_current"
    assert "force_disarm" not in after_descending_branch, \
        "超时路径不得调用 force_disarm"
