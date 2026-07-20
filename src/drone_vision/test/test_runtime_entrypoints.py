import importlib


def test_vision_viewer_is_importable_from_drone_vision_package():
    module = importlib.import_module("drone_vision.vision_viewer")

    assert callable(module.main)
