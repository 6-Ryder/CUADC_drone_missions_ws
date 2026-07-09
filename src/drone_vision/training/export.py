#!/usr/bin/env python3
"""
导出 ONNX / TensorRT 模型.

ONNX (通用推理):
  python export.py --weights best.pt --format onnx

TensorRT (Jetson Orin Nano 推理):
  python export.py --weights best.pt --format engine --device 0

FP16 ONNX (更小更快):
  python export.py --weights best.pt --format onnx --half
"""

import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="导出 YOLO 模型")
    parser.add_argument("--weights", required=True, help="best.pt 路径")
    parser.add_argument("--format", default="onnx",
                        choices=["onnx", "engine", "tflite", "openvino"])
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--half", action="store_true", help="FP16")
    parser.add_argument("--device", default="0")
    parser.add_argument("--output", default="", help="输出目录")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("需要 ultralytics: pip install ultralytics")
        sys.exit(1)

    model = YOLO(args.weights)

    print(f"导出 {args.weights} → {args.format}")
    print(f"  尺寸: {args.imgsz} | FP16: {args.half}")

    export_kwargs = dict(
        format=args.format,
        imgsz=args.imgsz,
        half=args.half,
        simplify=(args.format == "onnx"),
        opset=12,
        dynamic=False,
    )

    if args.format == "engine":
        export_kwargs["device"] = args.device

    path = model.export(**export_kwargs)

    print(f"\n导出完成: {path}")
    print(f"复制到 drone_vision/models/:")
    print(f"  cp {path} ../../models/")


if __name__ == "__main__":
    main()
