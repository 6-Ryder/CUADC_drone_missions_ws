"""视觉模式 mode 6 (marker_only) 契约测试.

覆盖:
  1. _set_mode_cb 对 mode 6 的响应: 切换 _detection_mode=marker_only
  2. 模型已释放 (侦察阶段) 时 mode 6 被拒绝 (与 mode 0/3 同一门控)
  3. 推理块: cylinder 引擎仅 "all" 跑, marker 引擎在 "all"/"marker_only" 跑
  4. 筛选块: marker_only → 圆筒/H 检测全关
"""

from pathlib import Path

from drone_vision.vision_pipeline import VisionPipelineNode as VisionPipeline


SOURCE = Path(__file__).resolve().parents[1] / "drone_vision/vision_pipeline.py"


class _Req:
    def __init__(self, mode, enable=True):
        self.mode = mode
        self.enable = enable


class _Resp:
    def __init__(self):
        self.success = False
        self.message = ""


class _Coverage:
    def __init__(self):
        self.end_calls = 0
        self.begin_calls = 0
        self.reset_calls = 0

    def end_scan(self):
        self.end_calls += 1

    def begin_scan(self):
        self.begin_calls += 1

    def reset(self):
        self.reset_calls += 1


def _bare_pipeline():
    pipe = object.__new__(VisionPipeline)
    pipe._detection_mode = "all"
    pipe._detection_models_released = False
    pipe._coverage = _Coverage()
    pipe.set_servo_mode = lambda _mode: None
    pipe._release_detection_models = lambda: None
    return pipe


def test_mode6_switches_to_marker_only():
    pipe = _bare_pipeline()
    resp = _Resp()

    pipe._set_mode_cb(_Req(mode=6), resp)

    assert resp.success is True
    assert pipe._detection_mode == "marker_only"
    assert pipe._coverage.end_calls == 1


def test_mode6_rejected_after_models_released():
    pipe = _bare_pipeline()
    pipe._detection_models_released = True
    resp = _Resp()

    pipe._set_mode_cb(_Req(mode=6), resp)

    assert resp.success is False
    assert "释放" in resp.message
    assert pipe._detection_mode == "all"  # 保持原模式不变


def test_mode6_is_distinct_from_existing_modes():
    """mode 6 不覆盖 0/2/3/4/5 的既有语义."""
    source = SOURCE.read_text(encoding="utf-8")

    assert 'request.mode == 6' in source
    assert 'self._detection_mode = "marker_only"' in source
    # mode 6 分支带 released 门控 (与 mode 0/3 同一安全契约)
    mode6_block = source.split("elif request.mode == 6:")[1].split("elif request.mode == 4:")[0]
    assert "if self._detection_models_released" in mode6_block


def test_marker_engine_runs_in_all_and_marker_only():
    source = SOURCE.read_text(encoding="utf-8")

    assert 'if self._detection_mode in ("all", "marker_only"):' in source
    assert 'if self._detection_mode == "all":' in source
    # marker 引擎跑条件有 None 防御 (释放后 _drop_marker_engine=None 不崩)
    assert "if self._drop_marker_engine is not None:" in source


def test_marker_only_screens_out_cylinder_and_landing():
    source = SOURCE.read_text(encoding="utf-8")

    assert 'elif self._detection_mode == "marker_only":' in source
    block = source.split('elif self._detection_mode == "marker_only":')[1].split("else:")[0]
    assert "cyl_dets = []" in block
    assert "landing_dets = []" in block
