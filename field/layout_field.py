#!/usr/bin/env python3
"""Generate a rotated Gazebo mission field from the canonical north layout."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


MISSION_MODELS = {
    "field_base",
    "field_border",
    "zone_preparation",
    "zone_takeoff_landing",
    "landing_pad",
    "h_marking",
    "takeoff_line",
    "zone_drop",
    "effective_zone_b",
    "drop_cylinders",
    "zone_reconnaissance",
    "recon_cylinders",
    "hazard_plates",
    "corner_markers",
    "mission_corner_markers",
}


def _heading_rad(heading_deg: float) -> float:
    """Return a finite heading normalized to the signed radian interval."""
    heading = float(heading_deg)
    if not math.isfinite(heading):
        raise ValueError("layout heading must be finite")
    normalized_rad = math.radians(heading % 360.0)
    return math.atan2(math.sin(normalized_rad), math.cos(normalized_rad))


def gazebo_yaw_rad(heading_deg: float) -> float:
    """Convert clockwise-positive NED heading to Gazebo ENU yaw delta."""
    return -_heading_rad(heading_deg)


def rotate_world_xy_about_home(
    world_x: float,
    world_y: float,
    heading_deg: float,
    home_x: float = 4.0,
    home_y: float = 10.0,
):
    """Rotate one Gazebo World point for a clockwise-positive NED heading."""
    heading_rad = _heading_rad(heading_deg)
    dx = float(world_x) - float(home_x)
    dy = float(world_y) - float(home_y)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    return (
        float(home_x) + dx * cos_h + dy * sin_h,
        float(home_y) - dx * sin_h + dy * cos_h,
    )


def rotate_pose_text(
    text: str,
    heading_rad: float,
    home_x: float,
    home_y: float,
) -> str:
    """Rotate a six-value World pose while preserving height and attitude."""
    values = [float(value) for value in text.split()]
    if len(values) != 6:
        raise ValueError(f"expected six-value SDF pose, got: {text!r}")
    x, y, z, roll, pitch, yaw = values
    dx = x - home_x
    dy = y - home_y
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    rotated_x = home_x + dx * cos_h + dy * sin_h
    rotated_y = home_y - dx * sin_h + dy * cos_h
    return (
        f"{rotated_x:.9f} {rotated_y:.9f} {z:.9f} "
        f"{roll:.9f} {pitch:.9f} {yaw - heading_rad:.9f}"
    )


def rotate_layout_sdf(
    input_path,
    output_path,
    heading_deg,
    home_x=4.0,
    home_y=10.0,
):
    """Write an SDF with only explicit mission assets rotated about Home."""
    heading_rad = _heading_rad(heading_deg)
    input_path = Path(input_path)
    output_path = Path(output_path)
    tree = ET.parse(input_path)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError(f"SDF has no world element: {input_path}")

    for model in world.findall("model"):
        if model.get("name") not in MISSION_MODELS:
            continue
        model_pose = model.find("pose")
        if model_pose is not None:
            model_pose.text = rotate_pose_text(
                model_pose.text or "",
                heading_rad,
                float(home_x),
                float(home_y),
            )
            continue
        for link in model.findall("link"):
            link_pose = link.find("pose")
            if link_pose is not None:
                link_pose.text = rotate_pose_text(
                    link_pose.text or "", heading_rad,
                    float(home_x), float(home_y),
                )

    tree.write(output_path, encoding="unicode")


def main():
    """Run the rotated-field command-line interface."""
    parser = argparse.ArgumentParser(
        description="Rotate Gazebo mission assets around SITL Home"
    )
    parser.add_argument("input", help="canonical input SDF")
    parser.add_argument("output", help="runtime output SDF")
    parser.add_argument("--heading-deg", required=True, type=float)
    parser.add_argument("--home-x", type=float, default=4.0)
    parser.add_argument("--home-y", type=float, default=10.0)
    parser.add_argument("--print-gazebo-yaw-rad", action="store_true")
    args = parser.parse_args()

    rotate_layout_sdf(
        args.input,
        args.output,
        args.heading_deg,
        home_x=args.home_x,
        home_y=args.home_y,
    )
    if args.print_gazebo_yaw_rad:
        print(gazebo_yaw_rad(args.heading_deg))


if __name__ == "__main__":
    main()
