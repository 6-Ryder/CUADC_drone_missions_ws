#!/usr/bin/env python3
"""
为 drone_field.sdf 随机生成投放区 / 侦察区圆筒位置。

随机策略：
- 以子区域中心为均值，按截断二维正态分布采样 x、y。
  越靠近中心概率越高，向边缘逐渐降低（近似正态）。
- 使用拒绝采样保证：
  1) 圆筒整体位于子区域内；
  2) 任意两个圆筒中心间距不小于设定的最小距离；
  3) 同步生成 B 区蓝色圆盘与危险品标识板位置。

用法：
    python3 generate_random_field.py [输入SDF] [输出SDF]
默认读取脚本所在目录的 drone_field.sdf，输出 drone_field_random.sdf。
"""

import argparse
import re
import random
from pathlib import Path

import numpy as np


# ----------------------------- 配置区 -----------------------------

DROP_ZONE = {
    "center": (4.0, 42.5),        # 投放区中心（World x=east, y=north）
    "half_size": (4.0, 2.5),      # x:0~8, y:40~45
    "sigma": (1.6, 1.0),          # 正态分布标准差，中心密、边缘疏
    "min_gap": 1.1,               # 圆筒中心最小间距（m），防止相距过近
    "cylinders": [
        # (外半径, 高度, 网格文件名)
        (0.0750, 0.3000, "cyl_r0075_h030.stl"),
        (0.1000, 0.3000, "cyl_r0100_h030.stl"),
        (0.1250, 0.3000, "cyl_r0125_h030.stl"),
    ],
    "b_zone_radius": 0.50,
}

RECON_ZONE = {
    "center": (4.0, 67.5),        # 侦察区中心（World x=east, y=north）
    "half_size": (4.0, 2.5),      # x:0~8, y:65~70
    "sigma": (1.6, 1.0),
    "min_gap": 0.60,
    "cylinders": [
        (0.1000, 0.1500, "cyl_r0100_h015.stl"),
        (0.1000, 0.1500, "cyl_r0100_h015.stl"),
        (0.1000, 0.1500, "cyl_r0100_h015.stl"),
        (0.1000, 0.1500, "cyl_r0100_h015.stl"),
        (0.1000, 0.1500, "cyl_r0100_h015.stl"),
    ],
    # 危险品标识：3 个，分别使用 textures 下的三张图
    "hazards": [
        ("hazard_flammable.png", "易燃"),
        ("hazard_health.png", "健康危害"),
        ("hazard_danger.png", "危险"),
    ],
}

MESH_PREFIX = ""
TEXTURE_PREFIX = ""

MAX_TRIALS = 2000


# ----------------------------- 采样函数 -----------------------------

def sample_position(center, half_size, sigma, existing, min_gap, radius):
    """
    按截断正态分布采样一个圆筒中心位置。
    若无法找到满足约束的位置，返回 None。
    """
    cx, cy = center
    hx, hy = half_size
    sx, sy = sigma

    for _ in range(MAX_TRIALS):
        x = np.random.normal(cx, sx)
        y = np.random.normal(cy, sy)

        # 保证圆筒整体在子区域内
        if not (cx - hx + radius <= x <= cx + hx - radius):
            continue
        if not (cy - hy + radius <= y <= cy + hy - radius):
            continue

        # 保证与已放置圆筒相距足够远
        too_close = False
        for ex, ey, *_ in existing:
            if ((x - ex) ** 2 + (y - ey) ** 2) < min_gap ** 2:
                too_close = True
                break
        if too_close:
            continue

        return (x, y)

    return None


def place_cylinders(zone_cfg):
    """返回 [(x, y, radius, height, mesh), ...]"""
    placed = []
    for radius, height, mesh in zone_cfg["cylinders"]:
        pos = sample_position(
            zone_cfg["center"],
            zone_cfg["half_size"],
            zone_cfg["sigma"],
            placed,
            zone_cfg["min_gap"],
            radius,
        )
        if pos is None:
            raise RuntimeError(
                f"无法在区域内为半径 {radius} 的圆筒找到合适位置，请减小 min_gap 或增大区域。"
            )
        placed.append((pos[0], pos[1], radius, height, mesh))
    return placed


# ----------------------------- XML 生成 -----------------------------

def indent(text, level=4):
    pad = " " * level
    return "\n".join(
        pad + line if line.strip() else line
        for line in text.splitlines()
    )


def drop_zone_xml(cylinders):
    """生成投放区圆筒与 B 区蓝色圆盘的 XML。"""
    cyl_links = []
    b_zone_links = []
    for i, (x, y, r, h, mesh) in enumerate(cylinders, start=1):
        cyl_links.append(
            f"""    <link name="cylinder_{i}">
      <pose>{x:.4f} {y:.4f} {h/2:.4f} 0 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 {-h/2:.4f} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{MESH_PREFIX}/{mesh}</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>"""
        )
        b_zone_links.append(
            f"""    <link name="b_zone_{i}">
      <pose>{x:.4f} {y:.4f} 0.010 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.50</radius>
            <length>0.010</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.50</radius>
            <length>0.005</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.3 0.7 1</ambient>
          <diffuse>0.15 0.4 0.85 1</diffuse>
        </material>
      </visual>
    </link>"""
        )

    drop_model = (
        "    <model name=\"drop_cylinders\">\n"
        "      <static>true</static>\n"
        + "\n".join(cyl_links)
        + "\n    </model>"
    )
    b_model = (
        "    <model name=\"effective_zone_b\">\n"
        "      <static>true</static>\n"
        + "\n".join(b_zone_links)
        + "\n    </model>"
    )
    return drop_model, b_model


def recon_zone_xml(cylinders, hazards):
    """生成侦察区圆筒与危险品标识板的 XML。"""
    cyl_links = []
    for i, (x, y, r, h, mesh) in enumerate(cylinders, start=1):
        cyl_links.append(
            f"""    <link name="recon_cyl_{i}">
      <pose>{x:.4f} {y:.4f} {h/2:.4f} 0 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 {-h/2:.4f} 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>{MESH_PREFIX}/{mesh}</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>"""
        )

    # 随机挑选 3 个圆筒放置标识板，并随机打乱三种标识
    hazard_indices = random.sample(range(len(cylinders)), len(hazards))
    hazard_assignments = list(zip(hazard_indices, hazards))
    random.shuffle(hazard_assignments)

    plate_links = []
    for plate_i, (cyl_idx, (texture, _label)) in enumerate(
        hazard_assignments, start=1
    ):
        x, y, *_ = cylinders[cyl_idx]
        plate_links.append(
            f"""    <link name="hazard_plate_{plate_i}">
      <pose>{x:.4f} {y:.4f} 0.010 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.12 0.12 0.012</size>
          </box>
        </geometry>
        <material>
          <ambient>0.65 0.65 0.65 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <emissive>0 0 0 1</emissive>
          <pbr>
            <metal>
              <albedo_map>{TEXTURE_PREFIX}/{texture}</albedo_map>
              <metalness>0</metalness>
              <roughness>1</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>"""
        )

    recon_model = (
        "    <model name=\"recon_cylinders\">\n"
        "      <static>true</static>\n"
        + "\n".join(cyl_links)
        + "\n    </model>"
    )
    hazard_model = (
        "    <model name=\"hazard_plates\">\n"
        "      <static>true</static>\n"
        + "\n".join(plate_links)
        + "\n    </model>"
    )
    return recon_model, hazard_model


# ----------------------------- 主流程 -----------------------------

def replace_model_block(text, model_name, new_xml):
    """用正则替换 <model name="..."> ... </model> 块。"""
    pattern = re.compile(
        rf'(?s)\s*<model name="{re.escape(model_name)}">.*?</model>'
    )
    return pattern.sub("\n" + new_xml + "\n", text)


def main():
    script_dir = Path(__file__).parent
    parser = argparse.ArgumentParser(description="随机生成无人机仿真场地 SDF")
    parser.add_argument(
        "input",
        nargs="?",
        default=str(script_dir / "drone_field.sdf"),
        help="输入 SDF 文件（默认：脚本所在目录的 drone_field.sdf）",
    )
    parser.add_argument(
        "output",
        nargs="?",
        default=str(script_dir / "drone_field_random.sdf"),
        help="输出 SDF 文件（默认：脚本所在目录的 drone_field_random.sdf）",
    )
    parser.add_argument(
        "--seed",
        type=int,
        help="固定 Python 与 NumPy 随机种子",
    )
    parser.add_argument(
        "--heading-deg",
        type=float,
        help="绕 Home 旋转任务资产的 NED 航向角（北为 0，顺时针为正）",
    )
    args = parser.parse_args()

    if args.heading_deg is not None:
        try:
            from .layout_field import gazebo_yaw_rad, rotate_layout_sdf
        except ImportError:
            from layout_field import gazebo_yaw_rad, rotate_layout_sdf
        gazebo_yaw_rad(args.heading_deg)

    if args.seed is not None:
        random.seed(args.seed)
        np.random.seed(args.seed)

    input_path = Path(args.input)
    output_path = Path(args.output)

    if not input_path.exists():
        raise FileNotFoundError(f"找不到输入文件: {input_path.absolute()}")

    # 根据输入文件所在目录自动确定 mesh / texture 的绝对路径
    global MESH_PREFIX, TEXTURE_PREFIX
    mesh_dir = (input_path.parent / "drone_field_meshes").resolve()
    texture_dir = (mesh_dir / "textures").resolve()
    MESH_PREFIX = mesh_dir.as_uri()
    TEXTURE_PREFIX = texture_dir.as_uri()

    text = input_path.read_text(encoding="utf-8")

    # 生成投放区
    drop_cyls = place_cylinders(DROP_ZONE)
    drop_model, b_model = drop_zone_xml(drop_cyls)

    # 生成侦察区
    recon_cyls = place_cylinders(RECON_ZONE)
    recon_model, hazard_model = recon_zone_xml(
        recon_cyls, RECON_ZONE["hazards"]
    )

    # 替换原 SDF 中的四个模型块
    text = replace_model_block(text, "drop_cylinders", drop_model)
    text = replace_model_block(text, "effective_zone_b", b_model)
    text = replace_model_block(text, "recon_cylinders", recon_model)
    text = replace_model_block(text, "hazard_plates", hazard_model)

    output_path.write_text(text, encoding="utf-8")
    if args.heading_deg is not None:
        rotate_layout_sdf(
            output_path,
            output_path,
            args.heading_deg,
            home_x=4.0,
            home_y=10.0,
        )
    print(f"已生成随机场地文件: {output_path.absolute()}")
    print("投放区圆筒位置:")
    for i, (x, y, r, h, _) in enumerate(drop_cyls, start=1):
        print(f"  筒{i}: x={x:.3f}, y={y:.3f}, r={r:.3f}")
    print("侦察区圆筒位置:")
    for i, (x, y, r, h, _) in enumerate(recon_cyls, start=1):
        print(f"  筒{i}: x={x:.3f}, y={y:.3f}")


if __name__ == "__main__":
    main()
