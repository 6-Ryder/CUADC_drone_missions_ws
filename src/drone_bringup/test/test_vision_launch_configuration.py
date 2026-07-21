import ast
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]


def _node_executables(launch_file):
    tree = ast.parse(launch_file.read_text(encoding="utf-8"))
    nodes = []
    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if not isinstance(call.func, ast.Name) or call.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        package = keywords.get("package")
        executable = keywords.get("executable")
        if isinstance(package, ast.Constant) and isinstance(executable, ast.Constant):
            nodes.append((package.value, executable.value))
    return nodes


def test_real_flight_launch_starts_camera_source():
    launch_file = WORKSPACE / "src/drone_bringup/launch/real_flight.launch.py"

    assert ("drone_vision", "camera_node") in _node_executables(launch_file)


def test_debug_script_defaults_to_stable_external_camera_path():
    script = (WORKSPACE / "scripts/debug_vision.sh").read_text(encoding="utf-8")

    assert "/dev/v4l/by-id/usb-D-VITEC_U3344W-video-index0" in script
