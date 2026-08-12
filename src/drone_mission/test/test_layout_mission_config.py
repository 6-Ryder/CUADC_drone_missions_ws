"""Mission layout configuration contracts."""

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from drone_mission.mission_state_machine import MissionFSM
from drone_utils.layout_frame import LayoutFrame
from px4_control import telemetry


ROOT = Path(__file__).resolve().parents[2]
FSM_PATH = (
    ROOT / "drone_mission" / "drone_mission" / "mission_state_machine.py"
)
CONFIG_DIR = ROOT / "drone_mission" / "config"

LEGACY_KEYS = {
    "flight_axis", "drop_distance", "recon_distance",
    "area1_north_min", "area1_north_max", "area1_east_min", "area1_east_max",
    "area2_north_min", "area2_north_max", "area2_east_min", "area2_east_max",
    "sweep_margin_north", "sweep_margin_east",
}

LAYOUT_KEYS = {
    "layout_heading_deg",
    "area1_forward_min", "area1_forward_max",
    "area1_right_min", "area1_right_max",
    "area2_forward_min", "area2_forward_max",
    "area2_right_min", "area2_right_max",
    "sweep_margin_forward", "sweep_margin_right",
}

STRATEGY_VALUES = {
    "mission_params.yaml": {
        "sweep_speed": 1.2, "transit_speed": 2.5, "return_speed": 3.8,
        "max_drops": 2, "quick_drop_mode": False,
    },
    "mission_params_quick_sim.yaml": {
        "sweep_speed": 1.2, "transit_speed": 3.5, "return_speed": 4.5,
        "max_drops": 1,
        "quick_drop_mode": True,
        "drop_target_source": "marker",
    },
    "mission_params_real.yaml": {
        "sweep_speed": 0.8, "transit_speed": 2.0, "return_speed": 3.8,
        "max_drops": 2, "quick_drop_mode": False,
    },
    "mission_params_quick.yaml": {
        "sweep_speed": 1.2, "transit_speed": 3.2, "return_speed": 4.2,
        "max_drops": 1,
        "quick_drop_mode": True,
        "drop_target_source": "marker",
    },
}


@pytest.fixture(autouse=True)
def restore_telemetry_home():
    """Restore process-wide NED conversion Home after each contract test."""
    home = (
        telemetry._home_world_x,
        telemetry._home_world_y,
        telemetry._home_world_z,
    )
    try:
        yield
    finally:
        telemetry.configure_home(*home)


def load_mission_params(filename):
    config = yaml.safe_load(
        (CONFIG_DIR / filename).read_text(encoding="utf-8")
    )
    return next(iter(config.values()))["ros__parameters"]


@pytest.mark.parametrize("filename", [
    "mission_params.yaml", "mission_params_quick_sim.yaml",
    "mission_params_real.yaml", "mission_params_quick.yaml",
])
def test_all_profiles_use_local_area_names_without_changing_values(filename):
    params = load_mission_params(filename)
    assert LEGACY_KEYS.isdisjoint(params)
    assert params["layout_heading_deg"] == 0.0
    assert params["area1_forward_min"] == 30.0
    assert params["area1_forward_max"] == 35.0
    assert params["area1_right_min"] == -4.0
    assert params["area1_right_max"] == 4.0
    assert params["area2_forward_min"] == 55.0
    assert params["area2_forward_max"] == 60.0
    assert params["area2_right_min"] == -4.0
    assert params["area2_right_max"] == 4.0
    assert params["sweep_margin_forward"] == 1.0
    assert params["sweep_margin_right"] == 0.5
    for name, value in STRATEGY_VALUES[filename].items():
        assert params.get(name, False) == value


def test_zone_centers_come_from_bounds_at_fifty_degrees():
    frame = LayoutFrame(50.0)
    assert frame.bounds_center_ned(30, 35, -4, 4) == pytest.approx(
        frame.local_to_ned(32.5, 0.0)
    )
    assert frame.bounds_center_ned(55, 60, -4, 4) == pytest.approx(
        frame.local_to_ned(57.5, 0.0)
    )


def test_mission_fsm_snapshots_and_freezes_the_layout_heading():
    source = FSM_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    declared = {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "declare_parameter"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[0].value, str)
    }

    assert "layout_heading_deg" in declared
    assert LEGACY_KEYS.isdisjoint(declared)
    assert (
        "LayoutFrame(float(self.get_param(\"layout_heading_deg\", 0.0)))"
        in source
    )
    assert "FROZEN_LAYOUT_PARAMETERS" in source
    assert any(
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "add_on_set_parameters_callback"
        and len(call.args) == 1
        and isinstance(call.args[0], ast.Attribute)
        and call.args[0].attr == "_reject_layout_heading_update"
        for call in ast.walk(tree)
    )


@pytest.mark.parametrize("name", sorted(LAYOUT_KEYS))
def test_layout_parameter_callback_rejects_every_individual_layout_update(
    name,
):
    result = MissionFSM._reject_layout_heading_update(
        object(), [SimpleNamespace(name=name)]
    )

    assert result.successful is False


def test_layout_parameter_callback_rejects_mixed_batches():
    result = MissionFSM._reject_layout_heading_update(
        object(),
        [
            SimpleNamespace(name="transit_speed"),
            SimpleNamespace(name="area2_right_max"),
        ],
    )

    assert result.successful is False


def test_layout_parameter_callback_allows_unrelated_updates():
    result = MissionFSM._reject_layout_heading_update(
        object(), [SimpleNamespace(name="transit_speed")]
    )

    assert result.successful is True


def test_zero_degree_drop_zone_accepts_world_xy_without_legacy_ned_fallback():
    params = {
        "area1_forward_min": 30.0,
        "area1_forward_max": 35.0,
        "area1_right_min": -4.0,
        "area1_right_max": 4.0,
    }
    fsm = object.__new__(MissionFSM)
    fsm._layout_frame = LayoutFrame(0.0)
    fsm.get_param = lambda name, default=None: params.get(name, default)
    fsm.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)
    telemetry.configure_home(4.0, 10.0, 0.0)

    assert fsm.validate_drop_target_world_xy(4.0, 42.5) == (4.0, 42.5)
    assert fsm.validate_drop_target_world_xy(0.0, 32.5) is None


def test_mission_fsm_wires_rotated_centers_and_frozen_yaw_into_controllers():
    source = FSM_PATH.read_text(encoding="utf-8")

    assert "self._layout_frame.bounds_center_ned(*drop_bounds)" in source
    assert "self._layout_frame.bounds_center_ned(*recon_bounds)" in source
    assert "ned_to_world(drop_north, drop_east, 0.0)" in source
    assert "ned_to_world(recon_north, recon_east, 0.0)" in source
    assert source.count("task_yaw=self._layout_frame.yaw_rad") == 2
