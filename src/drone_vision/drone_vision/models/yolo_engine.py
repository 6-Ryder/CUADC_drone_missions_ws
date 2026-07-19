"""
YOLO 推理引擎 — 支持 ONNX Runtime / TensorRT 后端.

统一接口:
    engine = YOLOEngine(model_path, conf_threshold, nms_threshold, device)
    detections: list[Detection] = engine.infer(image)

每个 Detection:
    - class_id: int       # 类别 ID
    - class_name: str     # 类别名
    - confidence: float   # 置信度 [0, 1]
    - bbox: [x1, y1, x2, y2]  # 边界框 (像素坐标)
    - cx, cy: float       # bbox 中心 (像素)
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ============================================================
# 数据结构
# ============================================================

@dataclasses.dataclass
class Detection:
    """单个检测结果."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    cx: float  # bbox 中心 x (px)
    cy: float  # bbox 中心 y (px)
    mask: Optional[np.ndarray] = None  # 可选: 实例分割 mask


# ============================================================
# YOLO 引擎基类
# ============================================================

class YOLOEngine:
    """
    YOLO 目标检测推理引擎.

    Parameters
    ----------
    model_path : str
        ONNX 模型文件路径.
    conf_threshold : float
        置信度阈值.
    nms_iou_threshold : float
        NMS IoU 阈值.
    input_size : Tuple[int, int]
        模型输入尺寸 (宽, 高).
    class_names : List[str]
        类别名称列表.
    device : str
        "cuda", "cpu", 或 "tensorrt".
    """

    def __init__(
        self,
        model_path: str,
        conf_threshold: float = 0.5,
        nms_iou_threshold: float = 0.45,
        input_size: Tuple[int, int] = (640, 640),
        class_names: Optional[List[str]] = None,
        device: str = "cuda",
    ):
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self.nms_iou_threshold = nms_iou_threshold
        self.input_width, self.input_height = input_size
        self.class_names = class_names or []
        self.device = device

        self._session = None
        self._input_name: str = ""
        self._output_names: List[str] = []
        self._input_shape: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self._is_loaded = False

        # 检查 backend 可用性
        self._available = self._check_backend()

    # ----------------------------------------------------------
    # 公开接口
    # ----------------------------------------------------------

    @property
    def is_loaded(self) -> bool:
        return self._is_loaded

    @property
    def is_available(self) -> bool:
        """后端运行时是否可用 (如 onnxruntime 已安装)."""
        return self._available

    def load(self) -> bool:
        """加载模型. 返回是否成功."""
        if self._is_loaded:
            return True
        if not self._available:
            return False
        try:
            self._do_load()
            self._is_loaded = True
            return True
        except Exception as exc:
            import sys
            print(f"[YOLOEngine] 模型加载失败: {exc}", file=sys.stderr)
            return False

    def infer(self, image: np.ndarray) -> List[Detection]:
        """
        对 BGR uint8 图像 (H, W, 3) 执行推理,
        返回 Detection 列表 (已按置信度降序排列).
        """
        if not self._is_loaded:
            return []
        return self._do_infer(image)

    # ----------------------------------------------------------
    # 子类实现
    # ----------------------------------------------------------

    def _check_backend(self) -> bool:
        raise NotImplementedError

    def _do_load(self) -> None:
        raise NotImplementedError

    def _do_infer(self, image: np.ndarray) -> List[Detection]:
        raise NotImplementedError

    # ----------------------------------------------------------
    # 通用工具
    # ----------------------------------------------------------

    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        预处理: resize + letterbox + normalize + HWC→CHW + batch dim.

        返回形状 (1, 3, H, W) 的 float32 张量.
        """
        img = _letterbox_resize(image, (self.input_width, self.input_height))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC → CHW
        img = np.expand_dims(img, axis=0)    # → (1, 3, H, W)
        return img

    @staticmethod
    def _rescale_bbox(
        bboxes: np.ndarray,
        orig_shape: Tuple[int, int],
        model_size: Tuple[int, int],
    ) -> np.ndarray:
        """将 letterbox 坐标系下的 bbox 映射回原图坐标."""
        oh, ow = orig_shape[:2]
        mw, mh = model_size
        scale = min(mw / ow, mh / oh)
        pad_w = (mw - ow * scale) / 2
        pad_h = (mh - oh * scale) / 2

        out = bboxes.copy()
        out[:, [0, 2]] = (out[:, [0, 2]] - pad_w) / scale  # x1, x2
        out[:, [1, 3]] = (out[:, [1, 3]] - pad_h) / scale  # y1, y2
        # clamp
        out[:, 0] = np.clip(out[:, 0], 0, ow)
        out[:, 1] = np.clip(out[:, 1], 0, oh)
        out[:, 2] = np.clip(out[:, 2], 0, ow)
        out[:, 3] = np.clip(out[:, 3], 0, oh)
        return out


# ============================================================
# ONNX Runtime 实现
# ============================================================

class ONNXYOLOEngine(YOLOEngine):
    """使用 onnxruntime 进行 YOLO 推理."""

    def _check_backend(self) -> bool:
        try:
            import onnxruntime  # noqa: F401
            return True
        except ImportError:
            print("[ONNXYOLOEngine] onnxruntime 未安装; pip install onnxruntime-gpu")
            return False

    def _do_load(self) -> None:
        import onnxruntime as ort

        if self.device in ("cuda", "tensorrt"):
            preload_dlls = getattr(ort, "preload_dlls", None)
            if preload_dlls is None:
                raise RuntimeError(
                    "GPU inference requires onnxruntime-gpu>=1.21"
                )
            preload_dlls(directory="")

        if self.device == "cuda":
            expected_provider = "CUDAExecutionProvider"
            providers = [
                (expected_provider, {"cudnn_conv_algo_search": "HEURISTIC"}),
                "CPUExecutionProvider",
            ]
        elif self.device == "tensorrt":
            expected_provider = "TensorrtExecutionProvider"
            providers = [expected_provider, "CUDAExecutionProvider"]
        elif self.device == "cpu":
            expected_provider = "CPUExecutionProvider"
            providers = [expected_provider]
        else:
            raise ValueError(
                f"Unsupported inference device: {self.device!r}; "
                "expected cuda, tensorrt, or cpu"
            )

        self._session = ort.InferenceSession(
            str(self.model_path), providers=providers,
        )
        active_providers = self._session.get_providers()
        if not active_providers or active_providers[0] != expected_provider:
            raise RuntimeError(
                f"Requested {expected_provider}, active providers: "
                f"{active_providers}"
            )
        self._input_name = self._session.get_inputs()[0].name
        self._input_shape = self._session.get_inputs()[0].shape
        self._output_names = [o.name for o in self._session.get_outputs()]

    def _do_infer(self, image: np.ndarray) -> List[Detection]:
        tensor = self._preprocess(image)
        outputs = self._session.run(self._output_names, {self._input_name: tensor})
        return self._postprocess(outputs[0], image.shape[:2])

    def _postprocess(
        self, raw: np.ndarray, orig_shape: Tuple[int, int],
    ) -> List[Detection]:
        """
        解析 YOLOv8 ONNX 输出: shape (1, 84, 8400) 或 (1, 6+num_classes, N).
        84 = 4 (bbox cxcywh) + 80 (COCO) 或 4 + num_classes.
        """
        if raw.ndim == 3:
            raw = raw[0]  # (84, 8400)

        raw = np.transpose(raw)  # (N, 84)

        num_classes = len(self.class_names)
        # YOLOv8 输出: [cx, cy, w, h, class_scores...]
        boxes_cxcywh = raw[:, :4]
        scores = raw[:, 4: 4 + num_classes]

        # cxcywh → xyxy
        boxes = np.zeros_like(boxes_cxcywh)
        boxes[:, 0] = boxes_cxcywh[:, 0] - boxes_cxcywh[:, 2] / 2  # x1
        boxes[:, 1] = boxes_cxcywh[:, 1] - boxes_cxcywh[:, 3] / 2  # y1
        boxes[:, 2] = boxes_cxcywh[:, 0] + boxes_cxcywh[:, 2] / 2  # x2
        boxes[:, 3] = boxes_cxcywh[:, 1] + boxes_cxcywh[:, 3] / 2  # y2

        # 从 letterbox (640×640) 空间缩放回原图坐标
        boxes = self._rescale_bbox(
            boxes, orig_shape, (self.input_width, self.input_height),
        )

        # 阈值过滤 + NMS (每类独立)
        detections: List[Detection] = []
        for cls_id in range(num_classes):
            cls_scores = scores[:, cls_id]
            mask = cls_scores > self.conf_threshold
            if not np.any(mask):
                continue

            cls_boxes = boxes[mask]
            cls_conf = cls_scores[mask]
            keep = _nms(cls_boxes, cls_conf, self.nms_iou_threshold)

            for idx in keep:
                detections.append(Detection(
                    class_id=cls_id,
                    class_name=self.class_names[cls_id] if cls_id < len(self.class_names) else f"cls_{cls_id}",
                    confidence=float(cls_conf[idx]),
                    bbox=(
                        int(round(cls_boxes[idx, 0])),
                        int(round(cls_boxes[idx, 1])),
                        int(round(cls_boxes[idx, 2])),
                        int(round(cls_boxes[idx, 3])),
                    ),
                    cx=float((cls_boxes[idx, 0] + cls_boxes[idx, 2]) / 2),
                    cy=float((cls_boxes[idx, 1] + cls_boxes[idx, 3]) / 2),
                ))

        # 按置信度降序
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections


# ============================================================
# Ultralytics PyTorch 实现
# ============================================================

class UltralyticsYOLOEngine(YOLOEngine):
    """使用 ultralytics 直接加载 .pt 权重."""

    def _check_backend(self) -> bool:
        try:
            import ultralytics  # noqa: F401
            import torch  # noqa: F401
            return True
        except ImportError:
            print("[UltralyticsYOLOEngine] ultralytics/torch 未安装")
            return False

    def _do_load(self) -> None:
        from ultralytics import YOLO

        self._model = YOLO(str(self.model_path))

    def _predict_device(self) -> str:
        if self.device == "cuda":
            try:
                import torch
                return "0" if torch.cuda.is_available() else "cpu"
            except Exception:
                return "cpu"
        if self.device == "tensorrt":
            return "0"
        return "cpu"

    def _do_infer(self, image: np.ndarray) -> List[Detection]:
        results = self._model.predict(
            image,
            imgsz=(self.input_height, self.input_width),
            conf=self.conf_threshold,
            iou=self.nms_iou_threshold,
            device=self._predict_device(),
            verbose=False,
        )

        detections: List[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].detach().cpu().numpy()
                class_name = (
                    self.class_names[cls_id]
                    if cls_id < len(self.class_names)
                    else str(getattr(result, "names", {}).get(cls_id, f"cls_{cls_id}"))
                )
                detections.append(Detection(
                    class_id=cls_id,
                    class_name=class_name,
                    confidence=conf,
                    bbox=(
                        int(round(x1)),
                        int(round(y1)),
                        int(round(x2)),
                        int(round(y2)),
                    ),
                    cx=float((x1 + x2) / 2),
                    cy=float((y1 + y2) / 2),
                ))

        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections

'''
#==========TensorRT 引擎类=========================
class TensorRTYOLOEngine(YOLOEngine):
    
    def _check_backend(self) -> bool:
        try:
            import tensorrt
            import ultralytics
            return True
        excpt ImportError:
            print("[TensorRTYOLOEngine] tensorrt/ultralytics do not installed")
            return False
    
    def _do_load(self) ->None:
        from ultralytics import YOLO
        self._model = YOLO(str(self.model_path), task='detect')
    def _predict_device(self) -> str:
        return "0
    def _do_infer(self,image: np.ndarray) -> List[Detection]:
        
        results = self._model.predict(
            image,
            imgsz=(self.input_height, self.input_width),
            conf=self.conf_threshold,
            iou=self.nms_iou_threshold,
            device="0",
            verbose=False,
        )

'''

# ============================================================
# 工厂函数
# ============================================================

def create_engine(
    model_path: str,
    conf_threshold: float = 0.5,
    nms_iou_threshold: float = 0.45,
    input_size: Tuple[int, int] = (640, 640),
    class_names: Optional[List[str]] = None,
    device: str = "cuda",
) -> YOLOEngine:
    """自动选择最佳可用后端."""
    suffix = Path(model_path).suffix.lower()
    if suffix == ".pt":
        engine = UltralyticsYOLOEngine(
            model_path=model_path,
            conf_threshold=conf_threshold,
            nms_iou_threshold=nms_iou_threshold,
            input_size=input_size,
            class_names=class_names,
            device=device,
        )
        if engine.is_available:
            return engine
        raise RuntimeError(
            "无可用的 Ultralytics YOLO 后端. 请安装 ultralytics/torch"
        )

    # 优先 ONNX
    engine = ONNXYOLOEngine(
        model_path=model_path,
        conf_threshold=conf_threshold,
        nms_iou_threshold=nms_iou_threshold,
        input_size=input_size,
        class_names=class_names,
        device=device,
    )
    if engine.is_available:
        return engine
    raise RuntimeError(
        "无可用的 YOLO 推理后端. 请安装: pip install onnxruntime-gpu"
    )


# ============================================================
# 工具函数
# ============================================================

def _letterbox_resize(
    image: np.ndarray,
    new_shape: Tuple[int, int],
    color: Tuple[int, int, int] = (114, 114, 114),
) -> np.ndarray:
    """Resize + pad 保持宽高比."""
    h, w = image.shape[:2]
    nw, nh = new_shape
    scale = min(nw / w, nh / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))

    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    canvas = np.full((nh, nw, 3), color, dtype=np.uint8)
    pad_h = (nh - new_h) // 2
    pad_w = (nw - new_w) // 2
    canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized
    return canvas


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> List[int]:
    """纯 numpy NMS, 返回保留的索引."""
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep: List[int] = []
    while len(order) > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-8)

        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return keep
