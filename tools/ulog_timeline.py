"""Compact PX4 ULog timeline for real-flight forensics."""
import sys
from pyulog import ULog

NAV = {0:"MANUAL",1:"ALTCTL",2:"POSCTL",3:"MISSION",4:"LOITER",5:"RTL",6:"LAND",
       7:"TAKEOFF",8:"FOLLOW",9:"PRECLAND",10:"ORBIT",11:"ACT_TEST",12:"TERMINATION",
       13:"OFFBOARD",14:"STABILIZED",15:"POS_SLOW",17:"TAKEOFF",18:"DESCEND"}

def rel(u, ts): return (ts - u.start_timestamp) / 1e6

def series(u, name, keys, fmt="%7.2fs " + "%-14s"*1):
    d = {x.name: x for x in u.data_list}
    if name not in d: return None
    m = d[name]
    have = [k for k in keys if k in m.data]
    if not have: 
        print(f"   <{name}: none of {keys}; has {sorted(m.data)}>"); return None
    return m, have

def changes(u, name, keys, label):
    r = series(u, name, keys)
    if not r: 
        print(f"  [{label}] unavailable"); return
    m, have = r; prev = None
    for i in range(len(m.data['timestamp'])):
        cur = tuple(m.data[k][i] for k in have)
        if cur != prev:
            print("  %7.2fs %-9s %s" % (rel(u, m.data['timestamp'][i]), label,
                  " ".join("%s=%s" % (k, v) for k, v in zip(have, cur))))
            prev = cur

def samples(u, name, keys, label, every=1.0):
    r = series(u, name, keys)
    if not r: return
    m, have = r; nxt = 0.0
    print(f"  --- {label} (每 {every}s) ---")
    for i in range(len(m.data['timestamp'])):
        t = rel(u, m.data['timestamp'][i])
        if t >= nxt:
            nxt = t + every
            print("  %7.2fs %s" % (t, " ".join("%s=%.2f" % (k, m.data[k][i]) for k in have)))

def events(u):
    r = series(u, "event", ["event_id","arguments"])
    if not r: return
    m, have = r
    print("  --- PX4 events ---")
    for i in range(len(m.data['timestamp'])):
        args = list(m.data["arguments"][i]) if "arguments" in have else []
        args = [a for a in args if a != 0.0]
        print("  %7.2fs id=%-5d %s" % (rel(u, m.data['timestamp'][i]), m.data["event_id"][i], args))

def commands(u):
    r = series(u, "vehicle_command", ["command","param1","param2","param3","param4","param5","param6","param7","source_system","target_system"])
    if not r: return
    m, have = r
    print("  --- vehicle_command ---")
    for i in range(len(m.data['timestamp'])):
        print("  %7.2fs cmd=%-5d %s" % (rel(u, m.data['timestamp'][i]), m.data["command"][i],
              " ".join("%s=%.3f" % (k, m.data[k][i]) for k in have if k.startswith("param"))))

def main(path):
    u = ULog(path)
    d = {x.name: x for x in u.data_list}
    print("="*74)
    print(path.split('/')[-1], " duration %.1fs" % ((u.last_timestamp-u.start_timestamp)/1e6))
    for k in ("sys_name","ver_sw","ver_sw_branch","ver_sw_release","ver_hw","sys_uuid"):
        if k in u.msg_info_dict: print(f"  {k}: {u.msg_info_dict[k]}")
    rel_v = int(u.msg_info_dict.get("ver_sw_release", 0))
    print("  PX4 version: %d.%d.%d" % ((rel_v>>24)&0xFF, (rel_v>>16)&0xFF, (rel_v>>8)&0xFF))
    changes(u, "vehicle_status", ["nav_state","arming_state","failsafe","system_type"], "STATUS")
    changes(u, "vehicle_control_mode", ["flag_control_offboard_enabled","flag_control_position_enabled","flag_control_velocity_enabled","flag_control_altitude_enabled"], "CTRLMODE")
    changes(u, "vehicle_land_detected", ["landed","maybe_landed","ground_contact"], "LAND")
    changes(u, "rtl_status", ["active"], "RTL")
    changes(u, "estimator_status_flags", ["cs_tilt_align","cs_yaw_align","cs_gnss_pos","cs_rng_hgt","cs_ev_pos"], "ESTFLAGS")
    samples(u, "vehicle_local_position", ["x","y","z","vx","vy","vz"], "LOCAL_POS(NED)", 1.5)
    samples(u, "trajectory_setpoint", ["position[0]","position[1]","position[2]","velocity[0]","velocity[1]","velocity[2]"], "TRAJ_SETPOINT", 2.0)
    samples(u, "manual_control_setpoint", ["roll","pitch","throttle","yaw","aux1"], "RC_STICKS", 3.0)
    events(u)
    commands(u)

for p in sys.argv[1:]:
    main(p)
