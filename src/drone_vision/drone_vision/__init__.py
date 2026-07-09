"""
drone_vision — 无人机视觉系统 (YOLO + OpenCV 混合方案).

架构:
  drone_vision/
  ├── models/          # YOLO 推理引擎 + 预训练权重
  │   ├── yolo_engine.py    — ONNX/TensorRT 推理后端
  │   └── *.onnx            — 训练好的模型文件
  ├── core/            # 纯算法 (无 ROS 依赖, 可独立单元测试)
  ├── (节点文件)       # ROS2 节点
  ├── config/          # YAML 参数文件
  └── training/        # 训练管线 (独立于 ROS, 在 PC 上运行)

数据流:
  相机 (camera.py) → vision_pipeline.py ← YOLO 模型
      ├─ CylinderArray   — 圆筒检测 + 精修
      ├─ HazardArray     — 危险品分类 + 颜色回退
      ├─ LandingTarget   — H 标识 + 姿态补偿坐标
      └─ CoverageStatus  — 覆盖监控
"""

__version__ = "2.0.0"

# ---- Lazy imports: only import what's available, don't crash on missing deps ----

# Pure-algorithm modules (always available, no ROS/cv_bridge needed)
try:
    from drone_vision.coordinate_transforms import CoordinateTransformer, AltitudeAdaptiveParams
except Exception:
    CoordinateTransformer = None
    AltitudeAdaptiveParams = None

try:
    from drone_vision.coverage_monitor import CoverageMonitor
except Exception:
    CoverageMonitor = None

try:
    from drone_vision.visual_servoing import (
        VisualServoController, CylinderApproachController, ServoState,
    )
except Exception:
    VisualServoController = None
    CylinderApproachController = None
    ServoState = None

try:
    from drone_vision.models.yolo_engine import Detection, create_engine, YOLOEngine
except Exception:
    Detection = None
    create_engine = None
    YOLOEngine = None

# ROS2-dependent modules (require rclpy + cv_bridge)
try:
    from drone_vision.vision_pipeline import VisionPipelineNode
except Exception:
    VisionPipelineNode = None

try:
    from drone_vision.cylinder_detector import CylinderDetectorNode, CylinderRefiner, CylinderTracker
except Exception:
    CylinderDetectorNode = CylinderRefiner = CylinderTracker = None

try:
    from drone_vision.hazard_classifier import ColorVerifier, CLASS_TO_HAZARD
except Exception:
    ColorVerifier = None
    CLASS_TO_HAZARD = {}

try:
    from drone_vision.landing_detector import LandingPadRefiner
except Exception:
    LandingPadRefiner = None

__all__ = [
    "VisionPipelineNode",
    "CylinderDetectorNode", "CylinderRefiner", "CylinderTracker",
    "ColorVerifier", "CLASS_TO_HAZARD",
    "LandingPadRefiner",
    "CoverageMonitor",
    "CoordinateTransformer", "AltitudeAdaptiveParams",
    "VisualServoController", "CylinderApproachController", "ServoState",
    "Detection", "create_engine", "YOLOEngine",
]
