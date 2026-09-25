"""Contracts for the single full real-hardware mission parameter path."""

import ast
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
MISSION_CONFIG = WORKSPACE / "src/drone_mission/config"
REAL_PROFILE = MISSION_CONFIG / "mission_params_real.yaml"
BASE_PROFILE = MISSION_CONFIG / "mission_params.yaml"
REAL_LAUNCH = WORKSPACE / "src/drone_bringup/launch/real_flight.launch.py"


def _parameters(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["/mission_state_machine"]["ros__parameters"]


def _assigned_node_call(source: str, variable_name: str) -> ast.Call:
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if (
            isinstance(target, ast.Name)
            and target.id == variable_name
            and isinstance(node.value, ast.Call)
        ):
            return node.value
    raise AssertionError(f"Node assignment {variable_name!r} was not found")


def _last_parameter_mapping(call: ast.Call) -> ast.Dict:
    for keyword in call.keywords:
        if keyword.arg != "parameters" or not isinstance(keyword.value, ast.List):
            continue
        mapping = keyword.value.elts[-1]
        if isinstance(mapping, ast.Dict):
            return mapping
    raise AssertionError("Node does not end its parameters list with a mapping")


def _literal_mapping(mapping: ast.Dict) -> dict:
    result = {}
    for key, value in zip(mapping.keys, mapping.values):
        assert isinstance(key, ast.Constant) and isinstance(key.value, str)
        try:
            result[key.value] = ast.literal_eval(value)
        except ValueError:
            # Deployment entries such as LaunchConfiguration are not literals.
            continue
    return result


def _mapping_keys(mapping: ast.Dict) -> set[str]:
    keys = set()
    for key in mapping.keys:
        assert isinstance(key, ast.Constant) and isinstance(key.value, str)
        keys.add(key.value)
    return keys


def test_real_profile_preserves_full_mission_and_uses_proven_real_values():
    real = _parameters(REAL_PROFILE)
    base = _parameters(BASE_PROFILE)

    assert set(real) == (set(base) - {"landing_blind_timeout"}) | {
        "return_altitude",
        "navigation_progress_window",
        "navigation_progress_min_improvement",
        "search_progress_window",
        "visual_progress_min_improvement",
        "landing_search_progress_window",
        "precision_align_final_raw_pixel_tolerance",
        "precision_align_tight_window_height",
    }
    assert "landing_blind_timeout" not in real
    assert real["vehicle_status_topic"] == "/fmu/out/vehicle_status_v1"
    assert real["takeoff_height"] == 5.0
    assert real["waypoint_tolerance"] == 0.5
    assert real["takeoff_tolerance"] == 0.3
    assert real["control_frequency"] == 20.0
    assert real["transit_speed"] == 2.0
    assert real["area2_altitude"] == 3.0
    assert real["sweep_speed"] == 0.8
    assert real["grid_spacing"] == 1.67
    assert real["return_speed"] == 3.8
    assert real["return_altitude"] == 5.0
    assert real["navigation_progress_window"] == 30.0
    assert real["navigation_progress_min_improvement"] == 0.25
    assert real["search_progress_window"] == 30.0
    assert real["visual_progress_min_improvement"] == 2.0
    assert real["landing_search_progress_window"] == 15.0

    assert real["landing_servo_kp_pixel"] == 0.0025
    assert real["landing_servo_max_horizontal_vel"] == 0.35
    assert real["landing_servo_deadzone_px"] == 40.0
    assert real["landing_servo_x_to_right_sign"] == 1.0
    assert real["landing_servo_y_to_forward_sign"] == -1.0
    assert real["landing_descent_fast"] == 0.4
    assert real["landing_descent_medium"] == 0.25
    assert real["landing_descent_slow"] == 0.12
    assert real["vision_loss_timeout"] == 5.0

    assert real["camera_fx"] == 400.0
    assert real["camera_fy"] == 400.0
    assert real["camera_cx"] == 320.0
    assert real["camera_cy"] == 240.0
    assert real["precision_align_image_width"] == 640.0
    assert real["precision_align_image_height"] == 480.0

    assert real["max_drops"] == 2
    assert real["min_drop_targets_required"] == 2
    assert real["precision_align_enable"] is True
    assert real["recon_require_full_path"] is True


def test_real_launch_camera_and_vision_values_match_the_mission_profile():
    source = REAL_LAUNCH.read_text(encoding="utf-8")
    real = _parameters(REAL_PROFILE)
    camera = _literal_mapping(
        _last_parameter_mapping(_assigned_node_call(source, "camera_node"))
    )
    vision = _literal_mapping(
        _last_parameter_mapping(_assigned_node_call(source, "vision_node"))
    )

    expected_launch_geometry = {
        "camera_width": real["precision_align_image_width"],
        "camera_height": real["precision_align_image_height"],
        "fx": real["camera_fx"],
        "fy": real["camera_fy"],
        "cx": real["camera_cx"],
        "cy": real["camera_cy"],
    }
    for key, expected in expected_launch_geometry.items():
        assert camera[key] == expected
        assert vision[key] == expected


def test_real_launch_loads_profile_without_mission_value_duplication():
    source = REAL_LAUNCH.read_text(encoding="utf-8")
    mission_parameters = _last_parameter_mapping(
        _assigned_node_call(source, "mission_node")
    )

    assert '"mission_params_real.yaml"' in source
    assert _mapping_keys(mission_parameters) == {
        "sim_mode",
        "home_x",
        "home_y",
        "home_z",
        "layout_heading_deg",
        "rc_require_arm_signal",
        "image_top_faces_forward",
    }
