from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1] / "src/servo_controller.cpp"
).read_text(encoding="utf-8")
CONFIG = (
    Path(__file__).resolve().parents[1] / "config/servo_params.yaml"
).read_text(encoding="utf-8")


def test_hardware_servo_waits_for_px4_command_ack():
    assert "VehicleCommandAck" in SOURCE
    assert '"/fmu/out/vehicle_command_ack"' in SOURCE
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


def test_drop_completion_callback_runs_after_drop_manager_lock_is_released():
    """The node callback reads DropManager state, so it must not run under mutex_."""
    execute_drop = SOURCE[
        SOURCE.index("bool DropManager::execute_drop"):SOURCE.index(
            "bool DropManager::can_drop()"
        )
    ]

    # The mutation can be protected, but the external callback must be outside
    # the lock scope or on_drop_done() will deadlock when it calls drop_count().
    lock_pos = execute_drop.index("std::lock_guard<std::mutex> lock(mutex_);")
    notify_pos = execute_drop.index("notify_done(success, cylinder_id);")
    assert "  }\n\n  notify_done(success, cylinder_id);" in execute_drop[lock_pos:]
    assert lock_pos < notify_pos


def test_hardware_servo_test_uses_the_ethernet_udp_agent():
    """The deployed flight controller reaches Jetson over Ethernet, not UART."""
    script = (
        Path(__file__).resolve().parents[1] / "scripts/test_servo_hardware.sh"
    ).read_text(encoding="utf-8")

    assert "MicroXRCEAgent udp4 -p \"$DDS_PORT\"" in script
    assert "MicroXRCEAgent serial" not in script
    assert "SERIAL_DEV=" not in script


def test_ack_timeout_retries_the_same_absolute_pwm_exactly_once():
    command_method = SOURCE[
        SOURCE.index("bool send_pwm_command(double pwm_us)"):SOURCE.index(
            "void handle_ack", SOURCE.index("bool send_pwm_command(double pwm_us)")
        )
    ]

    assert "command_retries: 1" in CONFIG
    assert "command_retries_" in command_method
    assert "attempt <= command_retries_" in command_method
    assert "send_actuator_cmd(pwm_to_normalized(pwm_us));" in command_method
    assert "attempt < command_retries_" in command_method


def test_rejected_ack_never_retries_and_current_pwm_changes_only_on_acceptance():
    command_method = SOURCE[
        SOURCE.index("bool send_pwm_command(double pwm_us)"):SOURCE.index(
            "void handle_ack", SOURCE.index("bool send_pwm_command(double pwm_us)")
        )
    ]

    reject = command_method.index("if (!ack_accepted_)")
    accepted_store = command_method.index("current_pwm_.store(pwm_us);")
    assert "return false;" in command_method[reject:accepted_store]
    assert reject < accepted_store
