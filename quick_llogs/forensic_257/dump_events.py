import sys
from pyulog import ULog
from pyulog.px4_events import PX4Events

f = sys.argv[1]
u = ULog(f)
t0 = u.start_timestamp
ev = PX4Events()
for ts, level, msg in ev.get_logged_events(u):
    print("  %8.2fs [%-9s] %s" % ((ts - t0) / 1e6, level, msg))
