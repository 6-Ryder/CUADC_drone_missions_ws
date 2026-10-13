#!/usr/bin/env python3
"""log_257 timeline: offboard setpoints vs actual motion vs RC sticks."""
import math
import sys

from pyulog import ULog

LOG = "/home/ryder/Desktop/log_257_2026-10-7-14-12-16.ulg"


def series(ulog, name):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == 0:
            return d
    return None


def col(d, key):
    return d.data.get(key)


def main():
    ulog = ULog(LOG, ['vehicle_local_position', 'vehicle_status',
                      'offboard_control_mode', 'trajectory_setpoint',
                      'vehicle_attitude_setpoint', 'actuator_outputs',
                      'vehicle_command', 'manual_control_setpoint',
                      'failsafe_flags', 'vehicle_control_mode'])

    def g(name):
        return series(ulog, name)

    lp = g('vehicle_local_position')
    t0 = lp.data['timestamp'][0] / 1e6

    def ts(d):
        return [t / 1e6 - t0 for t in d.data['timestamp']]

    def interp(d, key, t):
        tt = ts(d)
        vv = d.data[key]
        for i in range(len(tt) - 1):
            if tt[i] <= t <= tt[i + 1]:
                f = (t - tt[i]) / max(tt[i + 1] - tt[i], 1e-9)
                return vv[i] + f * (vv[i + 1] - vv[i])
        return float('nan')

    print(f"log duration: {ts(lp)[-1]:.2f}s  samples={len(lp.data['timestamp'])}")
    print()

    # --- mode transitions -------------------------------------------------
    vs = g('vehicle_status')
    prev = None
    print("=== nav_state transitions ===")
    for t, ns in zip(ts(vs), vs.data['nav_state']):
        if ns != prev:
            print(f"  t={t:7.2f}s  nav_state={ns}")
            prev = ns
    print()

    # --- offboard control mode -------------------------------------------
    ocm = g('offboard_control_mode')
    print("=== offboard_control_mode transitions (pos/vel/acc) ===")
    prev = None
    for t, p, v, a in zip(ts(ocm), ocm.data['position'],
                          ocm.data['velocity'], ocm.data['acceleration']):
        cur = (p, v, a)
        if cur != prev:
            print(f"  t={t:7.2f}s  position={p} velocity={v} accel={a}")
            prev = cur
    print()

    # --- failsafe ---------------------------------------------------------
    ff = g('failsafe_flags')
    keys = [k for k in ff.data if k.startswith('offboard') or k in (
        'manual_control', 'rc_override', 'position_control', 'altitude_control')]
    print("=== failsafe flag transitions (after t=0) ===")
    for k in keys:
        prev = None
        for t, val in zip(ts(ff), ff.data[k]):
            if t < 0:
                prev = val
                continue
            if val != prev:
                print(f"  t={t:7.2f}s  {k}={val}")
                prev = val
    print()

    # --- RC sticks --------------------------------------------------------
    mc = g('manual_control_setpoint')
    print("=== manual_control_setpoint (stick deflections > 0.05 from rest) ===")
    prev = None
    for i, t in enumerate(ts(mc)):
        thr = mc.data['throttle'][i]
        st = (mc.data['roll'][i], mc.data['pitch'][i], mc.data['yaw'][i])
        # PX4 manual_control_setpoint throttle rests at -1.0 (or 0 when not
        # publishing); treat any deviation from the resting value as a stick move
        moving = (abs(thr + 1.0) > 0.05 or any(abs(s) > 0.05 for s in st))
        if moving != prev:
            print(f"  t={t:7.2f}s  sticks_moving={int(moving)} roll={st[0]:+.2f} "
                  f"pitch={st[1]:+.2f} yaw={st[2]:+.2f} thr={thr:+.3f}")
            prev = moving
    print()

    # --- command source ---------------------------------------------------
    vc = g('vehicle_command')
    print("=== vehicle_command (commander_set_* 176/177/192/193) ===")
    for i, t in enumerate(ts(vc)):
        cmd = vc.data['command'][i]
        if cmd in (176, 177, 192, 193, 400):
            print(f"  t={t:7.2f}s cmd={cmd} src={vc.data['source_system'][i]} "
                  f"comp={vc.data['source_component'][i]}")
    print()

    # --- dense sample around the exit ------------------------------------
    sp = g('trajectory_setpoint')
    ac = g('actuator_outputs')

    def thr_at(t):
        """Collective thrust proxy = sum of motor outputs."""
        tt = ts(ac)
        vals = [ac.data[f'output[{i}]'][j] for j in range(len(tt)) for i in (0,)]
        for i in range(len(tt) - 1):
            if tt[i] <= t <= tt[i + 1]:
                f = (t - tt[i]) / max(tt[i + 1] - tt[i], 1e-9)
                tot = 0.0
                for k in range(4):
                    a = ac.data[f'output[{k}]'][i]
                    b = ac.data[f'output[{k}]'][i + 1]
                    tot += a + f * (b - a)
                return tot
        return float('nan')

    print("=== t, xyz, vxyz, |NE|, sp_x/y/z, sp_vx/vy/vz, motorsum, out4 ===")
    grid = [t * 0.5 for t in range(40, 122)]  # 20.0 .. 60.5 s
    for t in grid:
        x = interp(lp, 'x', t)
        y = interp(lp, 'y', t)
        z = interp(lp, 'z', t)
        vx = interp(lp, 'vx', t)
        vy = interp(lp, 'vy', t)
        vz = interp(lp, 'vz', t)
        sx = interp(sp, 'x', t) if 'x' in sp.data else float('nan')
        sy = interp(sp, 'y', t) if 'y' in sp.data else float('nan')
        sz = interp(sp, 'z', t) if 'z' in sp.data else float('nan')
        svx = interp(sp, 'vx', t) if 'vx' in sp.data else float('nan')
        svy = interp(sp, 'vy', t) if 'vy' in sp.data else float('nan')
        svz = interp(sp, 'vz', t) if 'vz' in sp.data else float('nan')
        thr = thr_at(t)
        o4 = interp(ac, 'output[4]', t)
        spd = math.hypot(vx, vy)

        def f(v, w=7, p=2):
            if v is None or not math.isfinite(v):
                return " " * w
            return f"{v:{w}.{p}f}"

        print(f"t={t:6.1f} pos=({f(x)},{f(y)},{f(z)}) "
              f"r={math.hypot(x, y):6.2f} v=({f(vx,6)},{f(vy,6)},{f(vz,6)}) "
              f"|v|={spd:4.2f} sp=({f(sx)},{f(sy)},{f(sz)}) "
              f"spv=({f(svx,6)},{f(svy,6)},{f(svz,6)}) thr={f(thr,6,3)} o4={f(o4,6,3)}")


if __name__ == "__main__":
    sys.exit(main())
