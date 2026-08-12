"""Contracts for the forward/right layout API of the recon flight test."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE_ROOT.parents[1]
NODE = PACKAGE_ROOT / "test_mission/test_forward_recon.py"
LAUNCH = PACKAGE_ROOT / "launch/test_forward_recon.launch.py"
SCRIPT = WORKSPACE / "scripts/test_forward_recon.sh"
PARAMETER_GUIDE = WORKSPACE / "docs/Debug/PARAMETER_USAGE_GUIDE.md"


def test_forward_recon_uses_only_forward_right_layout_parameter_names():
    source = NODE.read_text(encoding="utf-8")
    launch = LAUNCH.read_text(encoding="utf-8")
    combined = source + launch

    for stale_name in (
        "recon_area_half_north",
        "recon_area_half_east",
        "sweep_margin_north",
        "sweep_margin_east",
    ):
        assert stale_name not in combined

    for current_name in (
        "recon_area_half_forward",
        "recon_area_half_right",
        "sweep_margin_forward",
        "sweep_margin_right",
    ):
        assert current_name in source
        assert current_name in launch


def test_official_forward_recon_script_uses_current_launch_arguments():
    script = SCRIPT.read_text(encoding="utf-8")

    assert "recon_area_half_north" not in script
    assert "recon_area_half_east" not in script
    assert "recon_area_half_forward" in script
    assert "recon_area_half_right" in script


def test_active_parameter_guide_uses_current_layout_contract_and_yaw_semantics():
    guide = PARAMETER_GUIDE.read_text(encoding="utf-8")

    for stale_name in (
        "flight_axis",
        "drop_distance",
        "recon_distance",
        "area1_north",
        "area1_east",
        "area2_north",
        "area2_east",
        "sweep_margin_north",
        "sweep_margin_east",
        "recon_area_half_north",
        "recon_area_half_east",
    ):
        assert stale_name not in guide

    assert "yaw 固定为 `0.0`" not in guide
    assert "所有侦察航点为 `0.0`" not in guide

    for current_name in (
        "layout_heading_deg",
        "area1_forward",
        "area1_right",
        "area2_forward",
        "area2_right",
        "sweep_margin_forward",
        "sweep_margin_right",
        "recon_area_half_forward",
        "recon_area_half_right",
    ):
        assert current_name in guide

    assert "所有任务航点保持配置的布局航向" in guide
