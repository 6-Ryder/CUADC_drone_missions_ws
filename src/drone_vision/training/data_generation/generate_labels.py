#!/usr/bin/env python3
"""
从 Gazebo SDF 自动生成 YOLO 训练数据.

完整工作流:
  1. 解析 drone_field_random.sdf → 提取所有目标世界坐标
  2. 随机采样无人机位姿 (模拟飞行)
  3. 将目标 3D 坐标投影到 2D 像素 (利用相机模型)
  4. 生成 YOLO 格式标签 (归一化 bbox)
  5. (配合 Gazebo 截图) → 完整训练数据集

用法:
  # 仅生成标签 (离线, 不含图像)
  python generate_labels.py --sdf drone_field.sdf --output dataset/

  # 生成标签 + 记录相机位姿 (后续用 Gazebo 批量截图)
  python generate_labels.py --sdf drone_field_random.sdf --output dataset/ --samples 1000

输出结构:
  dataset/
  ├── images/
  │   └── (占位, 需要从 Gazebo 截图填充)
  ├── labels/
  │   ├── synth_000000.txt
  │   ├── synth_000001.txt
  │   └── ...
  ├── poses/                  # 每张图的相机位姿 (用于 Gazebo 截图)
  │   └── synth_000000.json
  └── data.yaml               # YOLO 训练配置
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


# ============================================================
# SDF 解析
# ============================================================

def parse_sdf_objects(sdf_path: str) -> Dict[str, List[dict]]:
    """解析 SDF 中所有可标注对象."""
    import xml.etree.ElementTree as ET

    tree = ET.parse(sdf_path)
    root = tree.getroot()

    objects = {
        "cylinders": [],      # 投放区 + 侦察区圆筒
        "hazard_plates": [],  # 危险品标识板
        "landing_pad": [],    # 降落底板
    }

    def _parse_pose(s):
        parts = s.strip().split()
        return tuple(float(p) for p in parts[:6])

    for model in root.iter("model"):
        name = model.get("name", "")

        # 投放圆筒
        if name == "drop_cylinders":
            for link in model.findall("link"):
                pose = _parse_pose(link.findtext("pose", "0 0 0 0 0 0"))
                r, h = 0.1, 0.3
                col = link.find("collision/geometry/cylinder")
                if col is not None:
                    r = float(col.findtext("radius", "0.1"))
                    h = float(col.findtext("length", "0.3"))
                objects["cylinders"].append({
                    "x": pose[0], "y": pose[1], "z": h/2,
                    "radius": r, "height": h, "zone": "drop",
                })

        # 侦察圆筒
        if name == "recon_cylinders":
            for link in model.findall("link"):
                pose = _parse_pose(link.findtext("pose", "0 0 0 0 0 0"))
                r, h = 0.1, 0.15
                col = link.find("collision/geometry/cylinder")
                if col is not None:
                    r = float(col.findtext("radius", "0.1"))
                    h = float(col.findtext("length", "0.15"))
                objects["cylinders"].append({
                    "x": pose[0], "y": pose[1], "z": h/2,
                    "radius": r, "height": h, "zone": "recon",
                })

        # 危险品
        if name == "hazard_plates":
            for link in model.findall("link"):
                pose = _parse_pose(link.findtext("pose", "0 0 0 0 0 0"))
                htype = "unknown"
                albedo = link.findtext("visual/material/pbr/metal/albedo_map", "")
                if "flammable" in albedo:
                    htype = "flammable"
                elif "health" in albedo:
                    htype = "health_hazard"
                elif "danger" in albedo:
                    htype = "danger"
                objects["hazard_plates"].append({
                    "x": pose[0], "y": pose[1], "type": htype,
                })

        # 降落标识
        if name == "landing_pad":
            pose = _parse_pose(model.findtext("pose", "10 4 0.01 0 0 0"))
            objects["landing_pad"].append({
                "x": pose[0], "y": pose[1], "radius": 0.40,
            })

    return objects


# ============================================================
# 投影
# ============================================================

def euler_to_rot(roll, pitch, yaw):
    """ZYX 欧拉角 → 旋转矩阵."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def world_to_pixel(world_pt, cam_pos, R_world_to_cam, K):
    """3D 世界点 → 2D 像素."""
    pt_cam = R_world_to_cam @ (world_pt - cam_pos)
    if pt_cam[2] <= 0:
        return None
    px = K @ pt_cam
    return (px[0]/px[2], px[1]/px[2])


def project_cylinder_bbox(cx, cy, z, radius, cam_pos, R_wc, K, img_w, img_h):
    """圆筒投影 → 2D bbox (采样圆周 12 点)."""
    angles = np.linspace(0, 2*np.pi, 12, endpoint=False)
    pixels = []
    for a in angles:
        wx = cx + radius * np.cos(a)
        wy = cy + radius * np.sin(a)
        uv = world_to_pixel(np.array([wx, wy, z]), cam_pos, R_wc, K)
        if uv:
            pixels.append(uv)
    if len(pixels) < 4:
        return None
    pixels = np.array(pixels)
    x1 = max(0, int(np.floor(pixels[:, 0].min())))
    y1 = max(0, int(np.floor(pixels[:, 1].min())))
    x2 = min(img_w, int(np.ceil(pixels[:, 0].max())))
    y2 = min(img_h, int(np.ceil(pixels[:, 1].max())))
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2, y2)


# ============================================================
# 主流程
# ============================================================

def generate(
    sdf_path: str,
    output_dir: str,
    num_samples: int = 500,
    altitude_range: Tuple[float, float] = (1.0, 10.0),
    fx: float = 400.0, fy: float = 400.0,
    img_w: int = 640, img_h: int = 480,
    seed: int = 42,
):
    rng = np.random.RandomState(seed)
    objects = parse_sdf_objects(sdf_path)

    cx_img = img_w / 2
    cy_img = img_h / 2
    K = np.array([[fx, 0, cx_img], [0, fy, cy_img], [0, 0, 1]], dtype=np.float64)

    CLASS_MAP = {
        "cylinder": 0, "flammable": 1, "health_hazard": 2,
        "danger": 3, "landing_H": 4, "empty_cylinder": 5,
    }

    out = Path(output_dir)
    (out / "labels").mkdir(parents=True, exist_ok=True)
    (out / "poses").mkdir(parents=True, exist_ok=True)
    (out / "images").mkdir(parents=True, exist_ok=True)

    generated = 0
    trials = 0
    max_trials = num_samples * 5

    print(f"SDF 对象: {len(objects['cylinders'])} 圆筒, "
          f"{len(objects['hazard_plates'])} 危险品, "
          f"{len(objects['landing_pad'])} 降落标识")
    print(f"目标: {num_samples} 样本")

    while generated < num_samples and trials < max_trials:
        trials += 1

        # 随机相机位姿
        cam_x = rng.uniform(0, 70)
        cam_y = rng.uniform(-2, 10)
        cam_z = rng.uniform(*altitude_range)
        pitch = np.pi/2 + rng.uniform(-0.1, 0.1)  # 接近垂直向下
        roll = rng.uniform(-0.08, 0.08)
        yaw = rng.uniform(0, 2*np.pi)

        cam_pos = np.array([cam_x, cam_y, cam_z])
        R_body_to_world = euler_to_rot(roll, pitch, yaw)
        # 相机→机体: 垂直向下安装
        R_cam_to_body = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_wc = (R_body_to_world @ R_cam_to_body).T  # world→camera = (cam→world)⁻¹

        labels = []

        # 投影圆筒
        for cyl in objects["cylinders"]:
            bbox = project_cylinder_bbox(
                cyl["x"], cyl["y"], cyl["height"],
                cyl["radius"], cam_pos, R_wc, K, img_w, img_h,
            )
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            # 判断类别 (是否含危险品)
            cls_id = CLASS_MAP["empty_cylinder"]
            for hp in objects["hazard_plates"]:
                if np.hypot(cyl["x"]-hp["x"], cyl["y"]-hp["y"]) < cyl["radius"]*1.5:
                    cls_id = CLASS_MAP.get(hp["type"], CLASS_MAP["empty_cylinder"])
                    break
            bx = ((x1+x2)/2)/img_w
            by = ((y1+y2)/2)/img_h
            bw = (x2-x1)/img_w
            bh = (y2-y1)/img_h
            labels.append(f"{cls_id} {bx:.6f} {by:.6f} {bw:.6f} {bh:.6f}")

        # 投影降落标识
        for pad in objects["landing_pad"]:
            bbox = project_cylinder_bbox(
                pad["x"], pad["y"], 0.01, pad["radius"],
                cam_pos, R_wc, K, img_w, img_h,
            )
            if bbox is None:
                continue
            x1, y1, x2, y2 = bbox
            bx = ((x1+x2)/2)/img_w
            by = ((y1+y2)/2)/img_h
            bw = (x2-x1)/img_w
            bh = (y2-y1)/img_h
            labels.append(f"{CLASS_MAP['landing_H']} {bx:.6f} {by:.6f} {bw:.6f} {bh:.6f}")

        if len(labels) < 1:
            continue

        sample_id = f"synth_{generated:06d}"
        (out / "labels" / f"{sample_id}.txt").write_text("\n".join(labels))
        (out / "poses" / f"{sample_id}.json").write_text(json.dumps({
            "x": cam_x, "y": cam_y, "z": cam_z,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "fx": fx, "fy": fy, "img_w": img_w, "img_h": img_h,
        }))

        generated += 1
        if generated % 100 == 0:
            print(f"  {generated}/{num_samples}")

    # data.yaml
    yaml_content = f"""path: {out.absolute()}
train: images
val: images
nc: 6
names:
  0: cylinder
  1: flammable
  2: health_hazard
  3: danger
  4: landing_H
  5: empty_cylinder
"""
    (out / "data.yaml").write_text(yaml_content)

    print(f"\n完成! {generated} 个标签文件")
    print(f"输出: {output_dir}")
    print(f"\n下一步:")
    print(f"  1. 在 Gazebo 中截图 (用 poses/ 中的位姿)")
    print(f"  2. 将截图放入 images/ 目录")
    print(f"  3. 运行: python train.py --data {output_dir}/data.yaml")


def main():
    parser = argparse.ArgumentParser(description="从 SDF 生成 YOLO 训练标签")
    parser.add_argument("--sdf", required=True)
    parser.add_argument("--output", default="dataset/")
    parser.add_argument("--samples", type=int, default=500)
    parser.add_argument("--altitude", nargs=2, type=float, default=[1.0, 10.0])
    parser.add_argument("--fx", type=float, default=400.0)
    parser.add_argument("--fy", type=float, default=400.0)
    parser.add_argument("--img_width", type=int, default=640)
    parser.add_argument("--img_height", type=int, default=480)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    generate(
        sdf_path=args.sdf, output_dir=args.output,
        num_samples=args.samples, altitude_range=tuple(args.altitude),
        fx=args.fx, fy=args.fy,
        img_w=args.img_width, img_h=args.img_height,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
