"""伺服 PID 调参契约.

pid_tuning.yaml 是"真机调 PID 只改一个文件"这一承诺的载体. 它有三个
静默失效模式, 每一个都会让地面调参变成盲调, 因此各由一条断言钉死:

1. 顶层 node key 不匹配 —— ROS 2 整份忽略参数文件, 不报错也不警告.
2. 文件未被 launch 接给 mission_state_machine, 或被误接给别的节点.
3. 参数名/默认值与代码声明漂移, 或 yaml 写整数而声明为 double
   (本仓库有过 InvalidParameterTypeException 启动崩溃的先例).

另有一条: 迁移到本文件的参数不得在 profile 配置里重复出现, 否则
旧值变成"改了没反应"的死值.
"""

import ast
import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
MISSION_PKG = ROOT / "src" / "drone_mission"
TUNING_FILE = MISSION_PKG / "config" / "pid_tuning.yaml"
FSM_SOURCE = MISSION_PKG / "drone_mission" / "mission_state_machine.py"
LAUNCH_DIR = ROOT / "src" / "drone_bringup" / "launch"

NODE_KEY = "/mission_state_machine"
MISSION_NODE_NAME = "mission_state_machine"

# 由 pid_tuning.yaml 接管, 不得再出现在 profile 配置中的旧键.
MIGRATED_KEYS = (
    "precision_align_gain",
    "precision_align_ground_kp",
    "precision_align_max_speed",
    "precision_align_min_correction_speed",
    "landing_servo_kp_pixel",
    "landing_servo_max_horizontal_vel",
)

PROFILE_CONFIGS = tuple((MISSION_PKG / "config").glob("mission_params*.yaml"))
LAUNCH_FILES = tuple(sorted(LAUNCH_DIR.glob("*.launch.py")))


def _tuning_params():
    """返回 pid_tuning.yaml 中 mission_state_machine 节点的参数字典."""
    data = yaml.safe_load(TUNING_FILE.read_text(encoding="utf-8"))
    assert NODE_KEY in data, (
        f"pid_tuning.yaml 顶层 node key 必须是 '{NODE_KEY}' "
        f"(漏写前导斜杠会让 ROS 2 整份静默忽略该文件), 实际为 {sorted(data)}"
    )
    return data[NODE_KEY]["ros__parameters"]


def _declared_defaults():
    """从 mission_state_machine.py 的 AST 提取 declare_parameter(name, default)."""
    tree = ast.parse(FSM_SOURCE.read_text(encoding="utf-8"))
    declared = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute)
                and func.attr == "declare_parameter"
                and len(node.args) >= 2):
            continue
        try:
            name = node.args[0]
            if not isinstance(name, ast.Constant) or not isinstance(name.value, str):
                continue
            declared[name.value] = ast.literal_eval(node.args[1])
        except ValueError:
            continue
    return declared


def _mission_nodes(launch_path):
    """AST 遍历 launch 中的 Node(...), 返回 name -> 该节点的参数列表元素源码."""
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    found = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not (isinstance(node.func, ast.Name) and node.func.id == "Node"):
            continue
        kwargs = {kw.arg: kw.value for kw in node.keywords}
        name_node = kwargs.get("name")
        params_node = kwargs.get("parameters")
        if name_node is None or params_node is None:
            continue
        if not isinstance(name_node, ast.Constant):
            continue
        if not isinstance(params_node, (ast.List, ast.Tuple)):
            continue
        found[name_node.value] = [ast.unparse(e) for e in params_node.elts]
    return found


def test_node_key_is_qualified():
    """顶层 key 必须带前导斜杠, 否则 ROS 2 静默忽略整份文件."""
    data = yaml.safe_load(TUNING_FILE.read_text(encoding="utf-8"))
    assert list(data) == [NODE_KEY], (
        f"pid_tuning.yaml 顶层 key 应为 ['{NODE_KEY}'], 实际 {list(data)}"
    )


def test_every_tuning_key_is_declared_with_identical_default():
    """yaml 里的每个键都必须在代码中声明, 且默认值逐项相同 (含类型)."""
    params = _tuning_params()
    declared = _declared_defaults()

    missing = sorted(k for k in params if k not in declared)
    assert not missing, f"pid_tuning.yaml 中的键未在 mission_state_machine.py 声明: {missing}"

    for key, value in sorted(params.items()):
        default = declared[key]
        assert float(default) == pytest.approx(float(value)), (
            f"{key}: yaml 值 {value} 与代码默认值 {default} 不一致"
        )


def test_tuning_values_are_floats_not_ints():
    """yaml 必须写浮点字面量 (0.0 而非 0), 否则 double 声明会启动崩溃."""
    text = TUNING_FILE.read_text(encoding="utf-8")
    body = text.split(f"{NODE_KEY}:", 1)[1]
    bad = []
    for line in body.splitlines():
        if re.match(r"^\s*servo_[a-z_]+:\s*-?\d+\s*(#.*)?$", line):
            bad.append(line.strip())
    assert not bad, (
        "以下参数写成了整数字面量, double 声明会触发 "
        f"InvalidParameterTypeException: {bad}"
    )
    for key, value in _tuning_params().items():
        assert isinstance(value, float), f"{key} 的值 {value!r} 不是浮点"


def test_no_servo_param_is_missing_from_the_tuning_file():
    """代码声明的 servo_* 参数不得游离在调参文件之外 (反向覆盖)."""
    params = _tuning_params()
    declared = _declared_defaults()
    undeclared_in_yaml = sorted(
        k for k in declared if k.startswith("servo_") and k not in params
    )
    assert not undeclared_in_yaml, (
        f"这些 servo_* 参数已在代码中声明但不在 pid_tuning.yaml 中: "
        f"{undeclared_in_yaml} —— 调参面不完整"
    )


@pytest.mark.parametrize("launch_path", LAUNCH_FILES, ids=lambda p: p.name)
def test_launch_passes_tuning_file_last_to_mission_node(launch_path):
    """launch 必须把 pid_tuning.yaml 排在 mission 节点参数列表末位.

    launch_ros 是后者覆盖前者, 排最后才能压过 mission_params_*.yaml 与
    内联 dict, "只改一个文件"的承诺才成立.
    """
    source = launch_path.read_text(encoding="utf-8")
    if MISSION_NODE_NAME not in source:
        pytest.skip(f"{launch_path.name} 不启动 {MISSION_NODE_NAME}")

    nodes = _mission_nodes(launch_path)
    assert MISSION_NODE_NAME in nodes, (
        f"{launch_path.name} 启动了 {MISSION_NODE_NAME} 但未能解析其 parameters"
    )
    elts = nodes[MISSION_NODE_NAME]
    last = elts[-1]
    assert "pid_tuning" in last, (
        f"{launch_path.name}: {MISSION_NODE_NAME} 的 parameters 末位应是 "
        f"pid_tuning_file, 实际是 {last}"
    )


@pytest.mark.parametrize("launch_path", LAUNCH_FILES, ids=lambda p: p.name)
def test_tuning_file_is_never_passed_to_vision_pipeline(launch_path):
    """pid_tuning.yaml 只能接给 mission 节点.

    vision_pipeline 也声明了同名的 servo_kp/servo_ki/servo_kd (服务于 B4
    已停用的那个 PID). 参数按节点隔离故不冲突, 但误把本文件喂过去会静默
    把任务伺服增益套到视觉节点上.
    """
    nodes = _mission_nodes(launch_path)
    for name, elts in nodes.items():
        if name == MISSION_NODE_NAME:
            continue
        if name in ("vision_pipeline",):
            for elt in elts:
                assert "pid_tuning" not in elt, (
                    f"{launch_path.name}: pid_tuning_file 被误接给 {name} 节点"
                )


@pytest.mark.parametrize("config_path", PROFILE_CONFIGS, ids=lambda p: p.name)
def test_migrated_keys_are_gone_from_profile_configs(config_path):
    """迁移到调参文件的键不得在 profile 配置里残留."""
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    node = data.get(NODE_KEY, {})
    params = node.get("ros__parameters", {})
    leftovers = sorted(k for k in MIGRATED_KEYS if k in params)
    assert not leftovers, (
        f"{config_path.name} 仍保留已迁移的键 {leftovers} —— 它们是死值, "
        f"改了不会有任何效果, 应删除以免误导真机调参"
    )
