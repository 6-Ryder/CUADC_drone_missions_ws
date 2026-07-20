import ast
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]


def _servo_nodes(launch_file):
    tree = ast.parse(launch_file.read_text(encoding="utf-8"))
    nodes = []
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if not isinstance(call.func, ast.Name) or call.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        name = keywords.get("name")
        package = keywords.get("package")
        if (
            isinstance(name, ast.Constant)
            and name.value == "servo_controller"
            and isinstance(package, ast.Constant)
        ):
            nodes.append(package.value)
    return nodes


def test_real_flight_uses_cpp_servo_controller():
    launch_file = WORKSPACE / "src/drone_bringup/launch/real_flight.launch.py"

    assert _servo_nodes(launch_file) == ["drone_servo2"]


def test_bringup_declares_cpp_servo_dependency():
    package_xml = (WORKSPACE / "src/drone_bringup/package.xml").read_text(
        encoding="utf-8"
    )

    assert "<depend>drone_servo2</depend>" in package_xml
    assert "<depend>drone_servo</depend>" not in package_xml
