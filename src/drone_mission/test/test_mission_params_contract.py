"""任务参数的完整性契约 —— 两个方向都要守.

这个文件的存在理由: 本项目反复出现两类"配置与代码脱节", 而且都不报错:

  1. **YAML 里有、代码里没有**（死参数）—— 地勤改了不生效, 且没有任何提示.
     2026-10-13 盘点时 196 个声明里有 12 个属于这类, 其中一个甚至写在四个
     profile 里、看起来像正经旋钮.
  2. **YAML 里有、但没 declare_parameter** —— 节点创建时才会炸
     (rclpy 不允许通过参数文件传入未声明的参数), 而且往往直到那次 launch 才暴露.

前者靠"每个声明都要有消费者"守, 后者靠"每个 YAML 键都要有声明"守。两条都是
正则/AST 可判定的机械检查, 适合钉成契约而不是靠人记。
"""

import ast
import re
from pathlib import Path

import yaml

WORKSPACE = Path(__file__).resolve().parents[3]
FSM_SOURCE = WORKSPACE / "src/drone_mission/drone_mission/mission_state_machine.py"
MISSION_CONFIG = WORKSPACE / "src/drone_mission/config"
PACKAGES = ("drone_mission", "drone_utils", "drone_vision", "px4_control",
            "drone_bringup", "drone_servo")

# 有意保留的**迁移记录**: 这几个名字已经没有运行时消费者, 但它们是
# config/pid_tuning.yaml 里"哪个旧名被哪个新增益取代"的对照表, 且被
# test_pid_tuning_contract 断言. 删掉会同时抹掉迁移线索与测试依据, 属另一项决定.
MIGRATION_RECORDS = {
    "landing_servo_kp_pixel",             # → servo_px_kp
    "landing_servo_max_horizontal_vel",   # → servo_px_max_speed
    "precision_align_min_correction_speed",  # → servo_min_speed
}


def _declared_parameters() -> set:
    """AST 解析 declare_parameter 的字面量参数名 (不匹配注释里的同名文本)."""
    tree = ast.parse(FSM_SOURCE.read_text(encoding="utf-8"))
    names = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[0].value, str)):
            names.add(node.args[0].value)
    return names


def _string_literals_outside_tests() -> set:
    """非 test/install/build 源码里出现过的所有字符串字面量.

    用 AST 而不是文本搜索: 文本会把注释和文档里的同名提及也算进来, 于是"只在
    注释里出现过"的死参数会被误判为"有消费者".

    ⚠️ 两处必须小心, 都是"以为在测、其实没测"的形态, 靠变异验证才暴露:
      1. **必须剔除 `declare_parameter("...")` 的那个实参** —— 声明本身就是一个
         字符串字面量, 不剔除的话每个参数都"匹配到自己", 测试永远通过.
      2. **剔除必须在同一个文件的作用域内完成**. 曾经把各文件的实参节点 id 收集到
         一个跨文件的集合里, 结果被 GC 回收后的节点 id 会被后续文件复用, 于是
         随机地把别的参数当成"已声明而跳过" —— 表现为每次运行报出不同的
         "未使用参数"。这里直接持有节点对象 (AST 节点按身份比较, 且持有引用即
         不会被回收), 且逐文件结算。
    """
    literals = set()
    for package in PACKAGES:
        root = WORKSPACE / "src" / package
        if not root.exists():
            continue
        for path in root.rglob("*.py"):
            text = str(path)
            if "/install/" in text or "/build/" in text or "/test/" in text:
                continue
            try:
                tree = ast.parse(path.read_text(encoding="utf-8", errors="ignore"))
            except SyntaxError:
                continue
            declaration_args = {
                node.args[0] for node in ast.walk(tree)
                if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter"
                and node.args
            }
            for node in ast.walk(tree):
                if (isinstance(node, ast.Constant)
                        and isinstance(node.value, str)
                        and node not in declaration_args):
                    literals.add(node.value)
    return literals


def _profile_keys(filename: str) -> set:
    document = yaml.safe_load((MISSION_CONFIG / filename).read_text(encoding="utf-8"))
    return set(document["/mission_state_machine"]["ros__parameters"])


def test_every_declared_parameter_has_a_consumer():
    """每个声明都要有人读 —— 否则它就是个"改了不生效"的误导旋钮."""
    declared = _declared_parameters()
    literals = _string_literals_outside_tests()

    unused = sorted(declared - literals - MIGRATION_RECORDS - {"use_sim_time"})

    assert unused == [], (
        "以下参数已声明但全仓非测试代码里无人读取; 要么接线, 要么连同所有 "
        f"profile YAML 一起删除: {unused}"
    )


def test_every_profile_key_is_declared():
    """每个 YAML 键都必须有 declare_parameter —— 否则该 launch 会在创建节点时报错.

    这类问题的暴露时机很晚 (直到有人启动那个 profile), 所以值得机械地守。
    """
    declared = _declared_parameters()
    problems = {}
    for filename in sorted(p.name for p in MISSION_CONFIG.glob("mission_params*.yaml")):
        missing = sorted(_profile_keys(filename) - declared)
        if missing:
            problems[filename] = missing

    assert problems == {}, (
        "以下 profile 含未声明的键, 启动该 profile 的任务节点会抛 "
        f"ParameterNotDeclaredException: {problems}"
    )


def test_profiles_do_not_declare_parameters_outside_expected_namespace():
    """守卫测试自身: 若 FSM 改了参数声明方式 (比如改用 param 文件生成),

    本文件的 AST 解析会静默返回空集, 于是两条断言都变成"永真". 这里用一个
    下界挡住那种退化。
    """
    declared = _declared_parameters()

    assert len(declared) > 100, (
        f"只解析出 {len(declared)} 个声明 —— 声明方式可能变了, 本契约需要同步更新"
    )
    assert re.match(r"^[a-z0-9_]+$", sorted(declared)[0])
