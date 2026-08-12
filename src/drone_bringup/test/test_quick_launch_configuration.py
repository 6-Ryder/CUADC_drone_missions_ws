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
