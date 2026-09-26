from types import SimpleNamespace

from drone_mission.quick_bucket_gate import (
    BucketReleaseGate,
    Calibration,
    Observation,
    pair_bucket_marker,
)


def obs(stamp, north=1.0, east=2.0, radius=0.075):
    return Observation(stamp, north, east, radius)


def params(**changes):
    values = dict(now=10.0, altitude=1.6, north=0.96, east=2.0,
                  yaw=0.0, vn=0.0, ve=0.0, vd=0.0,
                  telemetry_stamp=10.0, cylinder=obs(9.95),
                  marker=obs(9.95))
    values.update(changes)
    return values


def calibrated():
    return Calibration(0.04, -0.04, 0.0, 0.0, 0.0, 0.0,
                       0.3, 0.025, 0.025, 0.1)


def confirmed_gate():
    gate = BucketReleaseGate((1.0, 2.0), (1.0, 2.0), calibrated())
    gate.start(8.0)
    for i in range(5):
        gate.observe_radius(obs(8.0 + i * 0.1), now=8.01 + i * 0.1)
    assert gate.radius_confirmed
    return gate


def test_pair_rejects_ambiguous_and_unpaired_targets():
    cylinders = [dict(world_x=2.0, world_y=1.0, radius=0.075),
                 dict(world_x=5.0, world_y=1.0, radius=0.12)]
    assert pair_bucket_marker(cylinders[0], cylinders,
                              [dict(world_x=2.1, world_y=1.0)]) is not None
    assert pair_bucket_marker(cylinders[0], cylinders,
                              [dict(world_x=2.1, world_y=1.0),
                               dict(world_x=2.2, world_y=1.0)]) is None
    assert pair_bucket_marker(cylinders[0], cylinders,
                              [dict(world_x=3.0, world_y=1.0)]) is None


def test_radius_confirmation_counts_distinct_capture_frames():
    gate = BucketReleaseGate((1, 2), (1, 2), calibrated())
    gate.start(8.0)
    for _ in range(5):
        assert not gate.observe_radius(obs(8.1), now=8.1)
    for i in range(1, 5):
        gate.observe_radius(obs(8.1 + i * 0.1), now=8.1 + i * 0.1)
    assert gate.radius_confirmed


def test_radius_confirmation_rejects_low_effective_frame_rate():
    gate = BucketReleaseGate((1, 2), (1, 2), calibrated())
    gate.start(8.0)
    for i in range(5):
        stamp = 8.0 + i * 0.3
        gate.observe_radius(obs(stamp), now=stamp + 0.01)
    assert not gate.radius_confirmed
    assert gate.radius_failed


def test_requires_three_distinct_fits_then_post_brake_new_frame():
    gate = confirmed_gate()
    assert gate.evaluate(**params(now=10.0, cylinder=obs(9.95),
                                  marker=obs(9.95))).action == "continue"
    assert gate.evaluate(**params(now=10.1, telemetry_stamp=10.1, cylinder=obs(10.05),
                                  marker=obs(10.05))).action == "continue"
    assert gate.evaluate(**params(now=10.2, telemetry_stamp=10.2, cylinder=obs(10.15),
                                  marker=obs(10.15))).action == "brake"
    assert gate.evaluate(**params(now=10.52, telemetry_stamp=10.52, cylinder=obs(10.15),
                                  marker=obs(10.15))).action != "release"
    assert gate.evaluate(**params(now=10.53, telemetry_stamp=10.53, cylinder=obs(10.5),
                                  marker=obs(10.5))).reason == "small_bucket"


def test_stale_telemetry_old_frame_and_speed_block_release():
    gate = confirmed_gate()
    assert gate.evaluate(**params(telemetry_stamp=9.8)).action == "continue"
    assert gate.evaluate(**params(cylinder=obs(9.7))).action == "continue"
    assert gate.evaluate(**params(vn=0.09)).action == "continue"


def test_uncalibrated_never_releases_into_bucket():
    gate = confirmed_gate()
    gate.calibration = None
    for i in range(6):
        result = gate.evaluate(**params(now=10 + i * 0.1,
                                       telemetry_stamp=10 + i * 0.1,
                                       cylinder=obs(9.95 + i * 0.1),
                                       marker=obs(9.95 + i * 0.1)))
        assert result.reason != "small_bucket"


def test_fallback_requires_same_fresh_disc_and_post_brake_frame():
    gate = confirmed_gate()
    gate.calibration = None
    assert gate.evaluate(**params(now=10.0, cylinder=None,
                                  marker=obs(9.95))).action == "brake"
    assert gate.evaluate(**params(now=10.32, telemetry_stamp=10.32, cylinder=None,
                                  marker=obs(9.95))).action != "release"
    assert gate.evaluate(**params(now=10.35, telemetry_stamp=10.35, cylinder=None,
                                  marker=obs(10.33))).reason == "blue_disc_fallback"


def test_deadline_and_low_height_fail_closed():
    gate = confirmed_gate()
    assert gate.evaluate(**params(now=23.01, telemetry_stamp=23.01)).action == "takeover"
    gate = confirmed_gate()
    assert gate.evaluate(**params(altitude=1.49)).action == "takeover"
