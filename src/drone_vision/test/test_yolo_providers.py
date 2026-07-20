import sys
from types import SimpleNamespace

import numpy as np

from drone_vision.models.yolo_engine import ONNXYOLOEngine


class _Tensor:
    name = "images"
    shape = (1, 3, 640, 640)


class _Output:
    name = "output0"


def _fake_onnxruntime(active_provider):
    state = SimpleNamespace(preloaded=False, requested=None)

    class Session:
        def __init__(self, _path, providers):
            state.requested = providers

        @staticmethod
        def get_inputs():
            return [_Tensor()]

        @staticmethod
        def get_outputs():
            return [_Output()]

        @staticmethod
        def get_providers():
            return [active_provider, "CPUExecutionProvider"]

    def preload_dlls(directory):
        assert directory == ""
        state.preloaded = True

    module = SimpleNamespace(
        InferenceSession=Session,
        preload_dlls=preload_dlls,
    )
    return module, state


def test_cuda_device_preloads_python_cuda_libraries(monkeypatch):
    ort, state = _fake_onnxruntime("CUDAExecutionProvider")
    monkeypatch.setitem(sys.modules, "onnxruntime", ort)
    engine = ONNXYOLOEngine("model.onnx", class_names=["target"], device="cuda")

    engine._do_load()

    assert state.preloaded
    assert state.requested[0][0] == "CUDAExecutionProvider"
    assert state.requested[0][1]["cudnn_conv_algo_search"] == "HEURISTIC"


def test_tensorrt_device_requests_tensorrt_provider(monkeypatch):
    ort, state = _fake_onnxruntime("TensorrtExecutionProvider")
    monkeypatch.setitem(sys.modules, "onnxruntime", ort)
    engine = ONNXYOLOEngine(
        "model.onnx", class_names=["target"], device="tensorrt",
    )

    engine._do_load()

    assert state.preloaded
    assert state.requested[0] == "TensorrtExecutionProvider"


def test_onnx_preprocess_converts_opencv_bgr_to_model_rgb():
    engine = ONNXYOLOEngine(
        "model.onnx",
        input_size=(1, 1),
        class_names=["target"],
        device="cpu",
    )
    bgr = np.array([[[10, 20, 30]]], dtype=np.uint8)

    tensor = engine._preprocess(bgr)

    np.testing.assert_allclose(
        tensor[0, :, 0, 0],
        np.array([30, 20, 10], dtype=np.float32) / 255.0,
    )


def test_close_releases_runtime_session():
    engine = ONNXYOLOEngine(
        "model.onnx", class_names=["target"], device="cpu",
    )
    engine._session = object()
    engine._is_loaded = True

    engine.close()

    assert engine._session is None
    assert not engine.is_loaded
