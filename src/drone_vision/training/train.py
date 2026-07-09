#!/usr/bin/env python3
"""
YOLOv8 无人机检测模型训练.

快速开始:
  python train.py --data dataset/data.yaml

训练流程:
  1. 加载预训练 YOLOv8n 模型 (或更大的模型)
  2. 在无人机数据集上微调 200 轮
  3. 自动保存 best.pt
  4. 验证并打印每类 AP
"""

import argparse
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="训练 YOLOv8 无人机检测模型")
    parser.add_argument("--data", required=True, help="data.yaml 路径")
    parser.add_argument("--model", default="yolov8n.pt", help="基础模型")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--optimizer", default="AdamW")
    parser.add_argument("--lr0", type=float, default=0.001)
    parser.add_argument("--lrf", type=float, default=0.01)
    parser.add_argument("--project", default="runs/drone")
    parser.add_argument("--name", default="train")
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
        import torch
    except ImportError:
        print("需要 ultralytics: pip install ultralytics")
        sys.exit(1)

    print(f"PyTorch {torch.__version__} | CUDA: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"  GPU: {torch.cuda.get_device_name(0)}")

    model = YOLO(args.model)

    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        optimizer=args.optimizer,
        lr0=args.lr0,
        lrf=args.lrf,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3,
        cos_lr=True,
        close_mosaic=10,
        # 无人机数据增强 (适度)
        hsv_h=0.015, hsv_s=0.3, hsv_v=0.2,
        degrees=5.0, translate=0.1, scale=0.3,
        flipud=0.0, fliplr=0.5,
        mosaic=0.5, mixup=0.1,
        # 早停
        patience=50,
        # 输出
        val=True, save=True, save_period=10,
        project=args.project, name=args.name,
        exist_ok=True, pretrained=True, verbose=True,
    )

    best = Path(args.project) / args.name / "weights" / "best.pt"
    print(f"\n训练完成! 最佳模型: {best}")
    print(f"复制到 drone_vision/models/:")
    print(f"  cp {best} ../../models/drone_yolo.pt")


if __name__ == "__main__":
    main()
