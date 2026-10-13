"""快速投放策略 launch 契约测试.

覆盖:
  1. quick_flight.launch.py (真机) 引用 mission_params_quick.yaml + servo_params_quick.yaml
  2. quick_sitl.launch.py (SITL) 引用 mission_params_quick_sim.yaml + servo_params_quick.yaml
  3. 原 launch 不引用 quick 参数 (策略隔离)
  4. quick 真机/SITL 话题约定各自独立 (SITL 用 vehicle_status_v4)
"""

import ast
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
LAUNCH_DIR = WORKSPACE / "src/drone_bringup/launch"
MISSION_CONFIG = WORKSPACE / "src/drone_mission/config"
DOCS_DIR = WORKSPACE / "docs"
FIELD_README = WORKSPACE / "field/README.md"


def _mission_params(filename):
    document = yaml.safe_load((MISSION_CONFIG / filename).read_text(encoding="utf-8"))
    return document["/mission_state_machine"]["ros__parameters"]


def _param_files(launch_file):
    """提取 launch 中 os.path.join(...) 产生的参数文件路径."""
    tree = ast.parse(launch_file.read_text(encoding="utf-8"))
    files = []
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func = call.func
        if not (isinstance(func, ast.Attribute) and func.attr == "join"):
            continue
        # join(pkg_xxx, "config", "<file>.yaml")
        if (
            len(call.args) == 3
            and isinstance(call.args[1], ast.Constant)
            and call.args[1].value == "config"
            and isinstance(call.args[2], ast.Constant)
        ):
            files.append(call.args[2].value)
    return files


def _launch_text(name):
    return (LAUNCH_DIR / name).read_text(encoding="utf-8")


def test_quick_flight_references_quick_params():
    files = _param_files(LAUNCH_DIR / "quick_flight.launch.py")

    assert "mission_params_quick.yaml" in files
    assert "servo_params_quick.yaml" in files
    assert "mission_params_real.yaml" not in files
    assert "servo_params.yaml" not in files


def test_quick_sitl_references_quick_sim_params():
    files = _param_files(LAUNCH_DIR / "quick_sitl.launch.py")

    assert "mission_params_quick_sim.yaml" in files
    assert "servo_params_quick.yaml" in files
    assert "mission_params.yaml" not in files
    assert "servo_params.yaml" not in files


def test_bucket_camera_geometry_stays_separate_between_real_and_sitl():
    """机体几何真机/SITL 必须分开, 且只存在于 airframe_params_*.yaml.

    相机安装位姿与投放点是**机体**属性 (随飞机变, 不随任务 profile 变), 所以:
      - 真机与 SITL 各一份, 数值刻意不同 (仿真相机与机体原点重合, 偏移全 0);
      - 不再散落在 mission_params_quick.yaml / vision_params_quick*.yaml 里 ——
        此前相机位姿只在 vision 配置、投放点只在 mission 配置, 而 full 任务
        两处都没有, 于是"改了配置却不生效"是静默发生的.
    """
    real_files = _param_files(LAUNCH_DIR / "quick_flight.launch.py")
    sim_files = _param_files(LAUNCH_DIR / "quick_sitl.launch.py")
    airframe_dir = WORKSPACE / "src/drone_bringup/config"

    assert "vision_params_quick.yaml" in real_files
    assert "vision_params_quick_sim.yaml" in sim_files
    assert "airframe_params_real.yaml" in real_files
    assert "airframe_params_sitl.yaml" in sim_files

    def _airframe(filename):
        doc = yaml.safe_load((airframe_dir / filename).read_text(encoding="utf-8"))
        return (doc["/vision_pipeline"]["ros__parameters"],
                doc["/mission_state_machine"]["ros__parameters"])

    real_vision, real_mission = _airframe("airframe_params_real.yaml")
    sim_vision, sim_mission = _airframe("airframe_params_sitl.yaml")

    # 真机: 相机在机体原点下方 36.5cm, 装载时原点离地 40cm
    assert real_vision["bucket_home_body_height"] == 0.40
    assert real_vision["bucket_camera_down"] == 0.365
    assert real_mission["bucket_camera_down"] == 0.365
    assert real_mission["bucket_front_forward"] == 0.045
    # SITL: 仿真相机与原点重合
    assert sim_vision["bucket_home_body_height"] == 0.0
    assert sim_vision["bucket_camera_down"] == 0.0
    assert sim_mission["bucket_camera_down"] == 0.0

    # 两份文件必须覆盖同一组键, 否则"改了真机忘了仿真"会静默漏掉
    assert set(real_vision) == set(sim_vision)
    assert set(real_mission) == set(sim_mission)

    # 几何已从 profile 配置移出
    quick_mission = _mission_params("mission_params_quick.yaml")
    assert "bucket_camera_down" not in quick_mission
    assert "bucket_front_forward" not in quick_mission
    assert quick_mission["bucket_calibration_valid"] is False


def test_original_launches_do_not_reference_quick_params():
    """策略隔离: 原真机/SITL launch 绝不加载 quick 参数."""
    assert "mission_params_quick" not in _launch_text("real_flight.launch.py")
    assert "mission_params_quick" not in _launch_text("sitl.launch.py")
    assert "servo_params_quick" not in _launch_text("real_flight.launch.py")
    assert "servo_params_quick" not in _launch_text("sitl.launch.py")


def test_quick_sitl_keeps_v4_status_topic_override():
    """SITL 独立话题: quick_sitl 仍用 vehicle_status_v4 (与真机 v1 区分)."""
    text = _launch_text("quick_sitl.launch.py")

    assert "/fmu/out/vehicle_status_v4" in text


def test_quick_flight_has_no_status_topic_override():
    """真机 launch 不覆盖 topic, 继承参数文件 v1 (与原 real_flight 一致)."""
    text = _launch_text("quick_flight.launch.py")

    assert "vehicle_status_topic" not in text


def test_active_docs_have_no_legacy_direction_parameters():
    """Operator-facing documents use only the field-local coordinate contract."""
    active_docs = [
        DOCS_DIR / "ARCHITECTURE.md",
        DOCS_DIR / "CODE_READING_GUIDE.md",
        DOCS_DIR / "Debug/PARAMETER_USAGE_GUIDE.md",
        FIELD_README,
    ]
    for path in active_docs:
        text = path.read_text(encoding="utf-8")
        assert "flight_axis" not in text
        assert "area1_north_min" not in text
        assert "coverage_zone_x_min" not in text


def test_quick_profile_keeps_authoritative_behavior_values():
    """Quick SITL stays the authoritative fast marker/drop/landing strategy."""
    quick = _mission_params("mission_params_quick_sim.yaml")

    assert quick["quick_drop_mode"] is True
    assert quick["drop_target_source"] == "marker"
    assert quick["landing_simple_mode"] is True
    assert quick["transit_speed"] == 3.5
    assert quick["return_speed"] == 4.5


def test_main_sitl_profile_keeps_its_distinct_full_mission_tuning():
    """Main SITL must not inherit quick-only modes or speed tuning."""
    main = _mission_params("mission_params.yaml")

    assert "quick_drop_mode" not in main
    assert "landing_simple_mode" not in main
    assert main["transit_speed"] == 2.5
    assert main["return_speed"] == 3.8
