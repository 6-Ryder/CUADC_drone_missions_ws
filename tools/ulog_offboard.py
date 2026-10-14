"""Pinpoint the offboard stream lifetime and what PX4 said around its death."""
import sys, numpy as np
from pyulog import ULog

for f in sys.argv[1:]:
    u = ULog(f); d = {x.name: x for x in u.data_list}; t0 = u.start_timestamp
    r = lambda ts: (ts - t0)/1e6
    print("="*74); print(f.split('/')[-1], " span %.1fs" % ((u.last_timestamp-t0)/1e6))

    oc = d["offboard_control_mode"]; ts = oc.data["timestamp"]
    print("  OCM first=%.2fs last=%.2fs n=%d" % (r(ts[0]), r(ts[-1]), len(ts)))
    lastvals = [(k, oc.data[k][-1]) for k in ("position","velocity","acceleration","attitude","body_rate","throttle") if k in oc.data]
    print("  OCM flags at last sample: " + " ".join("%s=%s"%kv for kv in lastvals))
    for k in ("position","velocity","acceleration","attitude","body_rate","throttle"):
        if k in oc.data:
            v = np.array(oc.data[k]); ch = np.where(np.diff(v)!=0)[0]
            if len(ch): print("      %s changes at: %s" % (k, ", ".join("%.2fs->%d"%(r(ts[i+1]), v[i+1]) for i in ch[:8])))

    ev = d.get("event")
    if ev:
        print("  --- PX4 events (id / args) ---")
        for i in range(len(ev.data["timestamp"])):
            args = [ev.data[f"arguments[{j}]"][i] for j in range(25) if f"arguments[{j}]" in ev.data]
            args = [a for a in args if a not in (0.0,)]
            print("    %7.2fs id=%-5d %s" % (r(ev.data["timestamp"][i]), ev.data["id"][i], args[:8]))

    ar = d.get("action_request")
    if ar:
        print("  --- action_request ---")
        for i in range(len(ar.data["timestamp"])):
            print("    %7.2fs action=%s source=%s" % (r(ar.data["timestamp"][i]),
                  ar.data.get("action",[None]*len(ar.data["timestamp"]))[i],
                  ar.data.get("source",[None]*len(ar.data["timestamp"]))[i]))

    ao = d.get("actuator_outputs")
    if ao:
        keys = [k for k in ao.data if k.startswith("output[")]
        arr = np.array([ao.data[k] for k in keys])
        print("  --- actuator_outputs ---")
        print("    channels:", len(keys))
        for j,k in enumerate(keys):
            v = arr[j]; ch = np.where(np.abs(np.diff(v))>50)[0]
            print("    %-10s min=%7.1f max=%7.1f  changes>50: %s" % (k, v.min(), v.max(),
                  ", ".join("%.2fs->%.0f"%(r(ao.data["timestamp"][i+1]), v[i+1]) for i in ch[:6]) or "none"))
    tsx = d.get("telemetry_status")
    if tsx and "timestamp" in tsx.data:
        print("  --- telemetry_status rate/heartbeat ---")
        for k in ("rate_tx","rate_rx","heartbeat_type","type","rssi"):
            if k in tsx.data:
                v=np.array(tsx.data[k]); ch=np.where(np.diff(v)!=0)[0]
                print("    %-16s first=%.1f last=%.1f changes=%d" % (k, v[0], v[-1], len(ch)))
