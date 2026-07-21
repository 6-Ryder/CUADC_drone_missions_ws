from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1] / "src/servo_controller.cpp"
).read_text(encoding="utf-8")


def test_hardware_servo_waits_for_px4_command_ack():
    assert "VehicleCommandAck" in SOURCE
    assert '"/fmu/out/vehicle_command_ack_v1"' in SOURCE
    assert "VEHICLE_CMD_RESULT_ACCEPTED" in SOURCE


def test_hardware_servo_uses_multithreaded_executor_for_ack_callback():
    assert "MultiThreadedExecutor" in SOURCE


def test_initial_close_command_is_sent_after_executor_starts():
    constructor = SOURCE.index("PX4CommandServo(")
    constructor_end = SOURCE.index("bool initialize()", constructor)

    assert "set_pwm(init_pwm_us)" not in SOURCE[constructor:constructor_end]
    assert "create_wall_timer" in SOURCE
    assert "hardware_servo_->initialize()" in SOURCE


def test_main5_uses_actuator_set_one_without_a_param7_offset():
    assert "cmd.param1 = value;" in SOURCE
    assert "cmd.param7 = 0.0F;" in SOURCE
    assert "actuator_set" not in SOURCE
