import math

import pytest

from drone_utils.layout_frame import LayoutFrame


@pytest.mark.parametrize("raw, normalized", [
    (0.0, 0.0), (50.0, 50.0), (360.0, 0.0),
    (410.0, 50.0), (-10.0, 350.0),
])
def test_heading_normalizes(raw, normalized):
    assert LayoutFrame(raw).heading_deg == normalized


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_heading_is_rejected(bad):
    with pytest.raises(ValueError, match="finite"):
        LayoutFrame(bad)


def test_zero_degree_heading_uses_ned_basis():
    frame = LayoutFrame(0.0)
    assert frame.local_to_ned(1.0, 0.0) == pytest.approx((1.0, 0.0))
    assert frame.local_to_ned(0.0, 1.0) == pytest.approx((0.0, 1.0))


@pytest.mark.parametrize(
    "heading, expected_yaw", [(0.0, 0.0), (350.0, math.radians(-10.0))]
)
def test_heading_and_yaw_radians_wrap(heading, expected_yaw):
    frame = LayoutFrame(heading)
    assert frame.heading_rad == pytest.approx(math.radians(heading))
    assert frame.yaw_rad == pytest.approx(expected_yaw)


@pytest.mark.parametrize("heading", [0.0, 50.0, 90.0, 180.0, 350.0])
@pytest.mark.parametrize("point", [(0.0, 0.0), (32.5, 0.0), (55.0, -4.0)])
def test_local_ned_round_trip(heading, point):
    frame = LayoutFrame(heading)
    north, east = frame.local_to_ned(*point)
    forward, right = frame.ned_to_local(north, east)
    assert forward == pytest.approx(point[0], abs=1e-9)
    assert right == pytest.approx(point[1], abs=1e-9)


def test_fifty_degree_zone_center_and_exact_containment():
    frame = LayoutFrame(50.0)
    center = frame.bounds_center_ned(30.0, 35.0, -4.0, 4.0)
    assert center == pytest.approx(frame.local_to_ned(32.5, 0.0))
    inside = frame.local_to_ned(30.0, 4.0)
    outside = frame.local_to_ned(29.99, 4.01)
    assert frame.contains_ned(*inside, 30.0, 35.0, -4.0, 4.0)
    assert not frame.contains_ned(*outside, 30.0, 35.0, -4.0, 4.0)
