"""Impact geometry shared by bucket release and blue-disc fallback."""

import math

import pytest

from drone_mission.quick_release_geometry import impact_ned, landing_fits


def test_front_and_rear_offsets_rotate_with_ned_yaw():
    front = impact_ned(
        north=10.0, east=20.0, yaw=math.pi / 2,
        offset_forward=0.04, offset_right=0.0,
        vn=0.0, ve=0.0, vd=0.0,
        release_height=1.6, target_height=0.3, delay=0.0,
    )
    rear = impact_ned(
        north=10.0, east=20.0, yaw=math.pi / 2,
        offset_forward=-0.04, offset_right=0.0,
        vn=0.0, ve=0.0, vd=0.0,
        release_height=1.6, target_height=0.0, delay=0.0,
    )

    assert front == pytest.approx((10.0, 20.04))
    assert rear == pytest.approx((10.0, 19.96))


def test_independent_release_delays_and_target_planes_change_impact():
    common = dict(
        north=0.0, east=0.0, yaw=0.0,
        offset_forward=0.0, offset_right=0.0,
        vn=0.1, ve=0.0, vd=0.0, release_height=1.6,
    )
    rim = impact_ned(**common, target_height=0.3, delay=0.1)
    ground = impact_ned(**common, target_height=0.0, delay=0.2)

    assert rim[0] == pytest.approx(0.1 * (0.1 + math.sqrt(2 * 1.3 / 9.81)))
    assert ground[0] > rim[0]


def test_vertical_down_velocity_reduces_fall_time():
    common = dict(
        north=0.0, east=0.0, yaw=0.0,
        offset_forward=0.0, offset_right=0.0,
        vn=0.1, ve=0.0, release_height=1.6,
        target_height=0.3, delay=0.0,
    )
    still = impact_ned(**common, vd=0.0)
    descending = impact_ned(**common, vd=0.2)

    assert descending[0] < still[0]


def test_target_fit_includes_bottle_radius_and_uncertainty():
    assert landing_fits((0.0, 0.0), (0.015, 0.0), 0.075, 0.03, 0.03)
    assert not landing_fits((0.0, 0.0), (0.016, 0.0), 0.075, 0.03, 0.03)
    assert not landing_fits((math.nan, 0.0), (0.0, 0.0), 0.5, 0.03, 0.1)


def test_invalid_release_geometry_is_rejected():
    with pytest.raises(ValueError):
        impact_ned(
            north=0.0, east=0.0, yaw=0.0,
            offset_forward=0.0, offset_right=0.0,
            vn=0.0, ve=0.0, vd=0.0,
            release_height=0.2, target_height=0.3, delay=0.0,
        )
