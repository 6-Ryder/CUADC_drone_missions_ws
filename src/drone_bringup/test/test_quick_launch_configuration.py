"""快速投放策略 launch 契约测试.

覆盖:
  1. quick_flight.launch.py (真机) 引用 mission_params_quick.yaml + servo_params_quick.yaml
  2. quick_sitl.launch.py (SITL) 引用 mission_params_quick_sim.yaml + servo_params_quick.yaml
  3. 原 launch 不引用 quick 参数 (策略隔离)
  4. quick 真机/SITL 话题约定各自独立 (SITL 用 vehicle_status_v4)
"""

import ast
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
LAUNCH_DIR = WORKSPACE / "src/drone_bringup/launch"


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
