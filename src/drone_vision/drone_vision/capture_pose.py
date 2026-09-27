"""Timestamped pose interpolation for bucket-rim image projection."""

import math


def interpolate_pose(history, stamp, max_gap=0.05):
    """Return a pose at capture time, or None if no sufficiently close pose exists.

    History entries are (seconds, x/y/z/qx/qy/qz/qw).  Adjacent quaternions
    are interpolated on their shortest arc and normalized.
    """
    if not math.isfinite(stamp) or stamp <= 0 or not history:
        return None
    before = after = None
    for item in history:
        if item[0] <= stamp:
            before = item
        if item[0] >= stamp:
            after = item
            break
    if before is None and after is not None:
        return after[1] if after[0] - stamp <= max_gap else None
    if after is None and before is not None:
        return before[1] if stamp - before[0] <= max_gap else None
    if before is None or after is None:
        return None
    if stamp - before[0] > max_gap + 1e-9 or after[0] - stamp > max_gap + 1e-9:
        return None
    if before[0] == after[0]:
        return before[1]
    alpha = (stamp - before[0]) / (after[0] - before[0])
    first, second = before[1], after[1]
    q0, q1 = first[3:], second[3:]
    if sum(a*b for a, b in zip(q0, q1)) < 0:
        q1 = tuple(-v for v in q1)
    quat = tuple(a + alpha * (b-a) for a, b in zip(q0, q1))
    norm = math.sqrt(sum(v*v for v in quat))
    if norm <= 1e-12:
        return None
    xyz = tuple(a + alpha * (b-a) for a, b in zip(first[:3], second[:3]))
    return xyz + tuple(v/norm for v in quat)
