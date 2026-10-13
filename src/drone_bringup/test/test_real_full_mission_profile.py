"""Contracts for the single full real-hardware mission parameter path."""

import ast
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
MISSION_CONFIG = WORKSPACE / "src/drone_mission/config"
REAL_PROFILE = MISSION_CONFIG / "mission_params_real.yaml"
BASE_PROFILE = MISSION_CONFIG / "mission_params.yaml"
PID_TUNING = MISSION_CONFIG / "pid_tuning.yaml"
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
    """取 parameters 列表里最后一个内联映射.

    列表末尾允许追加参数文件路径 (pid_tuning.yaml 必须排在最后才能压过
    内联 dict 生效, 见 config/pid_tuning.yaml 的生效性自检), 因此不能假定
    最后一个元素就是 dict; 契约只关心内联映射里不重复任务参数值.
    """
    for keyword in call.keywords:
        if keyword.arg != "parameters" or not isinstance(keyword.value, ast.List):
            continue
        for element in reversed(keyword.value.elts):
            if isinstance(element, ast.Dict):
                return element
    raise AssertionError("Node has no inline mapping in its parameters list")


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

    # 降落伺服增益已迁移到 pid_tuning.yaml (原 landing_servo_kp_pixel /
    # landing_servo_max_horizontal_vel), 真机生效值不变: 0.0025 / 0.35.
    tuning = _parameters(PID_TUNING)
    assert tuning["servo_px_kp"] == 0.0025
    assert tuning["servo_px_max_speed"] == 0.35
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


def _launch_loads(launch_name: str, filename: str) -> bool:
    """launch 文件里是否引用了该参数文件名 (字形匹配).

    这里刻意用字面量匹配而不是解析 AST 求值: 目的是钉住"这个 launch 确实加载了
    这份配置", 简单的字符串出现足以表达, 且解析失败时会立刻报错而不是静默通过.
    """
    source = (WORKSPACE / "src/drone_bringup/launch" / launch_name).read_text(
        encoding="utf-8")
    return filename in source


def test_full_launches_load_their_airframe_geometry():
    """full 两条 launch 都必须加载对应机体的 airframe_params, 且两者不同.

    机体几何 (相机安装 / 机体原点离地 / 投放点) 以前只存在于 quick 的配置里,
    于是 full 任务**用代码默认值跑**(相机偏移 0.0)。接上之后 full 才拿到真机
    几何; 这条测试防止它再次静默脱落。
    """
    assert _launch_loads("real_flight.launch.py", "airframe_params_real.yaml")
    assert _launch_loads("sitl.launch.py", "airframe_params_sitl.yaml")
    # 真机 launch 不得加载 SITL 的几何 (反之亦然) —— 仿真相机与机体原点重合,
    # 灌进真机等于把偏移抹成 0.
    assert not _launch_loads("real_flight.launch.py", "airframe_params_sitl.yaml")
    assert not _launch_loads("sitl.launch.py", "airframe_params_real.yaml")


def test_empty_defaults_are_not_silently_used_for_geometry():
    """真机档的机体几何必须来自 airframe 文件, 而不是空的 profile 配置.

    真机相机的安装偏移是几十厘米量级; 若某天它又变回"profile 里没有、代码默认
    0.0", 像素→米的换算会系统性放大地面误差, 而且不会有任何报错。
    """
    real = _parameters(REAL_PROFILE)
    for key in ("bucket_camera_down", "bucket_home_body_height",
                "bucket_front_forward", "bucket_rear_forward"):
        assert key not in real, (
            f"{key} 应只存在于 airframe_params_real.yaml, 不应回到 profile")
