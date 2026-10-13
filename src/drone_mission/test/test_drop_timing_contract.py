from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def _parameters(path):
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    return next(iter(config.values()))["ros__parameters"]


def test_mission_confirmation_timeout_exceeds_servo_total_budget():
    mission = _parameters(ROOT / "drone_mission/config/mission_params.yaml")
    servo = _parameters(ROOT / "drone_servo2/config/servo_params.yaml")

    assert mission["drop_confirmation_timeout"] > servo["drop_timeout_s"]


def test_precision_alignment_has_a_bounded_fast_control_profile():
    mission = _parameters(ROOT / "drone_mission/config/mission_params.yaml")
    # 米制域速度上限已迁移到 pid_tuning.yaml: 原 precision_align_max_speed
    # 改名 servo_max_speed, 数值期望保持不变 (>= 0.20).
    tuning = _parameters(ROOT / "drone_mission/config/pid_tuning.yaml")

    assert tuning["servo_max_speed"] >= 0.20
    assert mission["precision_align_fine_max_speed"] >= 0.05
    assert mission["precision_align_step_s"] >= 0.15
    assert mission["precision_align_hover_s"] <= 0.05
    assert mission["precision_align_stable_frames"] <= 6
    assert mission["precision_align_final_brake_s"] <= 0.6
