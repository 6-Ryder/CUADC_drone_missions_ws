"""Contracts for generating a rotated Gazebo mission field."""

import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
FIELD_DIR = WORKSPACE / "field"
FIELD = FIELD_DIR / "drone_field.sdf"
LAYOUT_SCRIPT = FIELD_DIR / "layout_field.py"
RANDOM_SCRIPT = FIELD_DIR / "generate_random_field.py"

sys.path.insert(0, str(WORKSPACE))

from field.generate_random_field import DROP_ZONE, RECON_ZONE  # noqa: E402
from field.layout_field import (  # noqa: E402
    MISSION_MODELS,
    gazebo_yaw_rad,
    rotate_layout_sdf,
    rotate_world_xy_about_home,
)


EXPECTED_MISSION_MODELS = {
    "field_base", "field_border", "zone_preparation",
    "zone_takeoff_landing", "landing_pad", "h_marking",
    "takeoff_line", "zone_drop", "effective_zone_b",
    "drop_cylinders", "zone_reconnaissance", "recon_cylinders",
    "hazard_plates", "corner_markers", "mission_corner_markers",
}


def top_level_models(path):
    """Return top-level SDF world models keyed by name."""
    root = ET.parse(path).getroot()
    world = root.find("world")
    assert world is not None
    return {model.attrib["name"]: model for model in world.findall("model")}


def mission_world_poses(path):
    """Return every mission pose that is expressed in World coordinates."""
    poses = {}
    for model_name, model in top_level_models(path).items():
        if model_name not in EXPECTED_MISSION_MODELS:
            continue
        model_pose = model.find("pose")
        if model_pose is not None:
            poses[(model_name, None)] = tuple(
                float(v) for v in model_pose.text.split()
            )
            continue
        for link in model.findall("link"):
            pose = link.find("pose")
            if pose is not None:
                poses[(model_name, link.attrib["name"])] = tuple(
                    float(v) for v in pose.text.split()
                )
    return poses


def model_semantics(path, model_name):
    """Return a stable structural representation of one model."""
    model = top_level_models(path)[model_name]

    def element_semantics(element):
        text = (
            element.text.strip()
            if element.text and element.text.strip()
            else None
        )
        return (
            element.tag,
            tuple(sorted(element.attrib.items())),
            text,
            tuple(element_semantics(child) for child in element),
        )

    return element_semantics(model)


def angle_delta(left, right):
    """Return the signed shortest angular difference."""
    return math.atan2(math.sin(left - right), math.cos(left - right))


def direct_link_poses(path, model_names):
    """Return World link poses keyed by model and link."""
    poses = {}
    models = top_level_models(path)
    for model_name in model_names:
        for link in models[model_name].findall("link"):
            pose = link.find("pose")
            if pose is not None:
                poses[(model_name, link.attrib["name"])] = tuple(
                    float(v) for v in pose.text.split()
                )
    return poses


def run_random_generator(output, heading_deg):
    """Generate one deterministic random field through the public CLI."""
    subprocess.run(
        [
            sys.executable,
            str(RANDOM_SCRIPT),
            str(FIELD),
            str(output),
            "--seed", "20260812",
            "--heading-deg", str(heading_deg),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def test_fifty_degree_field_rotates_every_mission_asset_about_home(tmp_path):
    output = tmp_path / "field_50.sdf"
    rotate_layout_sdf(FIELD, output, 50.0, home_x=4.0, home_y=10.0)
    source_poses = mission_world_poses(FIELD)
    rotated_poses = mission_world_poses(output)

    assert MISSION_MODELS == EXPECTED_MISSION_MODELS
    assert rotated_poses.keys() == source_poses.keys()
    for key, source_pose in source_poses.items():
        expected_x, expected_y = rotate_world_xy_about_home(
            source_pose[0], source_pose[1], 50.0, 4.0, 10.0,
        )
        rotated_pose = rotated_poses[key]
        assert rotated_pose[0] == pytest.approx(expected_x, abs=1e-6)
        assert rotated_pose[1] == pytest.approx(expected_y, abs=1e-6)
        assert angle_delta(
            rotated_pose[5], source_pose[5] - math.radians(50.0)
        ) == pytest.approx(0.0, abs=1e-8)


def test_selected_model_pose_leaves_relative_nested_poses_unchanged(tmp_path):
    output = tmp_path / "field_50.sdf"
    rotate_layout_sdf(FIELD, output, 50.0)

    source_model = top_level_models(FIELD)["landing_pad"]
    output_model = top_level_models(output)["landing_pad"]
    assert model_semantics_for_path(
        source_model, "./link/visual/pose"
    ) == model_semantics_for_path(output_model, "./link/visual/pose")


def model_semantics_for_path(element, path):
    """Return pose text values below an element for one XML path."""
    return [
        tuple(float(v) for v in pose.text.split())
        for pose in element.findall(path)
    ]


def test_north_point_rotates_toward_east_for_positive_ned_heading():
    x, y = rotate_world_xy_about_home(4.0, 42.5, 50.0, 4.0, 10.0)
    assert x == pytest.approx(4.0 + 32.5 * math.sin(math.radians(50.0)))
    assert y == pytest.approx(10.0 + 32.5 * math.cos(math.radians(50.0)))


def test_gazebo_yaw_delta_has_opposite_sign_to_ned_heading():
    assert gazebo_yaw_rad(0.0) == pytest.approx(0.0)
    assert gazebo_yaw_rad(50.0) == pytest.approx(-math.radians(50.0))
    assert gazebo_yaw_rad(410.0) == pytest.approx(-math.radians(50.0))
    assert gazebo_yaw_rad(-10.0) == pytest.approx(math.radians(10.0))


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_heading_is_rejected(bad, tmp_path):
    with pytest.raises(ValueError, match="finite"):
        rotate_layout_sdf(FIELD, tmp_path / "bad.sdf", bad)
    with pytest.raises(ValueError, match="finite"):
        gazebo_yaw_rad(bad)


def test_environment_models_and_canonical_file_are_unchanged(tmp_path):
    before = FIELD.read_bytes()
    output = tmp_path / "field_50.sdf"
    rotate_layout_sdf(FIELD, output, 50.0)

    assert FIELD.read_bytes() == before
    assert model_semantics(output, "ground_plane") == model_semantics(
        FIELD, "ground_plane"
    )


def test_fixed_seed_random_samples_use_world_axes_and_rotate_rigidly(tmp_path):
    baseline = tmp_path / "random_0.sdf"
    rotated = tmp_path / "random_50.sdf"

    assert DROP_ZONE["center"] == (4.0, 42.5)
    assert DROP_ZONE["half_size"] == (4.0, 2.5)
    assert DROP_ZONE["sigma"] == (1.6, 1.0)
    assert RECON_ZONE["center"] == (4.0, 67.5)
    assert RECON_ZONE["half_size"] == (4.0, 2.5)
    assert RECON_ZONE["sigma"] == (1.6, 1.0)

    run_random_generator(baseline, 0.0)
    run_random_generator(rotated, 50.0)
    baseline_poses = direct_link_poses(
        baseline, {"drop_cylinders", "recon_cylinders"}
    )
    rotated_poses = direct_link_poses(
        rotated, {"drop_cylinders", "recon_cylinders"}
    )

    assert rotated_poses.keys() == baseline_poses.keys()
    for key, baseline_pose in baseline_poses.items():
        x, y = rotate_world_xy_about_home(
            rotated_poses[key][0], rotated_poses[key][1], -50.0, 4.0, 10.0,
        )
        assert x == pytest.approx(baseline_pose[0], abs=1e-6)
        assert y == pytest.approx(baseline_pose[1], abs=1e-6)

        if key[0] == "drop_cylinders":
            center = DROP_ZONE["center"]
            half_size = DROP_ZONE["half_size"]
        else:
            center = RECON_ZONE["center"]
            half_size = RECON_ZONE["half_size"]
        assert (
            center[0] - half_size[0]
            <= baseline_pose[0]
            <= center[0] + half_size[0]
        )
        assert (
            center[1] - half_size[1]
            <= baseline_pose[1]
            <= center[1] + half_size[1]
        )


def test_cli_prints_only_normalized_gazebo_yaw(tmp_path):
    output = tmp_path / "field_410.sdf"
    completed = subprocess.run(
        [
            sys.executable,
            str(LAYOUT_SCRIPT),
            str(FIELD),
            str(output),
            "--heading-deg", "410",
            "--print-gazebo-yaw-rad",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert completed.stdout.strip() == str(-math.radians(50.0))
    assert completed.stderr == ""
    assert output.is_file()
