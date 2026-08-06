#!/usr/bin/env python3
"""replay_align.py — 投放区精校判定链日志回放回归工具.

依据真机 mission.log 回放精校判定链 (precision_align_controller + state_drop),
验证参数组合是否放行, 用于方案回归与复飞前校验.

回放模型 (与 precision_align_controller.py 现场代码逐行核对):
  - "进入最终刹车复查" 行 (stable=N/N) 本身就是"连续稳定帧已达成"的现场证据
  - 刹车 0.5s 期间跳过观测
  - 刹车后下一个新观测帧: 像素 aligned (raw≤窗口 且 smooth≤阈值 且 |dx|,|dy|≤norm)
    且速度 ≤ 门槛 (无 speed 日志的架次视为速度通过, 如 8.05 老版本) → 放行
  - 两级窗口: 回放日志全部处于精校段 (2.8m), 用 raw_hi; 1.5m 投放段 (raw_lo) 无历史日志
  - 节流缺帧: 相邻 aligned 帧间隔 < gap_ok 视为连续

用法:
  python3 tools/replay_align.py real_flight_logs/.../mission.log [--group final|onsite]
  python3 tools/replay_align.py log1 log2 --raw 35 --smooth 30 --norm 0.06 \
      --stable 4 --brake 0.5 --speed 0.22
"""

import argparse
import math
import re
import sys
from typing import Dict, List, Optional, Tuple

# 参数预置组 (真机实测导出)
GROUPS = {
    # 现场参数: 35/35/0.05/5帧/0.5s/速度0.04 → 10:01 保真应不放行
    "onsite": dict(raw=35.0, smooth=35.0, norm=0.05, stable=5, brake=0.5, speed=0.04),
    # 终版方案: 35/30/0.06/4帧/0.5s/速度0.22 → 19:53/10:01 均应放行
    "final": dict(raw=35.0, smooth=30.0, norm=0.06, stable=4, brake=0.5, speed=0.22),
}

RE_OBS = re.compile(
    r"\[(?P<t>[\d.]+)\].*?PREC_ALIGN: source=(?P<src>\w+).*?"
    r"pixel_error=(?P<pe>[\d.]+)px raw=\((?P<dx>[+-][\d.]+),(?P<dy>[+-][\d.]+)\) "
    r"smooth=\((?P<sx>[+-][\d.]+),(?P<sy>[+-][\d.]+)\).*?stable=(?P<st>\d+)")
RE_DESCEND = re.compile(
    r"\[(?P<t>[\d.]+)\].*?DESCEND: 精校投放许可失败.*?"
    r"raw=(?P<pe>[\d.]+)px smooth_px=(?P<sp>[\d.]+)px "
    r"smooth=\((?P<sx>[+-][\d.]+),(?P<sy>[+-][\d.]+)\).*?stable=(?P<st>\d+)/(?P<n>\d+)")
RE_BRAKE = re.compile(
    r"\[(?P<t>[\d.]+)\].*?PREC_ALIGN: 进入最终刹车复查.*?"
    r"raw=(?P<pe>[\d.]+)px smooth=(?P<sp>[\d.]+)px stable=(?P<st>\d+)/(?P<n>\d+)")
RE_SPEED = re.compile(
    r"\[(?P<t>[\d.]+)\].*?等待水平速度归零 speed_xy=(?P<sp>[\d.]+)m/s")


def parse_log(path: str) -> List[Dict]:
    """解析 mission.log → 观测序列 (obs/brake/speed 混合, 按时间排序)."""
    rows: List[Dict] = []
    for line in open(path, errors="ignore"):
        line = line.replace("\x1b[0m", "").replace("\x1b[32m", "")
        for pattern, kind in ((RE_OBS, "obs"), (RE_BRAKE, "brake"),
                              (RE_DESCEND, "obs"), (RE_SPEED, "speed")):
            m = pattern.search(line)
            if m:
                g = m.groupdict()
                rows.append(dict(
                    t=float(g["t"]), kind=kind,
                    pe=float(g.get("pe") or 0),
                    sx=float(g.get("sx") or 0), sy=float(g.get("sy") or 0),
                    dx=float(g.get("dx") or 0), dy=float(g.get("dy") or 0),
                    has_xy="dx" in g and g["dx"] is not None,
                    sp=float(g.get("sp") or 0), st=int(g.get("st") or 0)))
                break
    rows.sort(key=lambda r: r["t"])
    return rows


def replay(rows: List[Dict], raw_hi: float, smooth_tol: float, norm: float,
           stable_frames: int, brake_s: float, max_speed: float,
           gap_ok: float = 2.5) -> Tuple[Optional[float], str]:
    """回放判定链. 返回 (放行时刻-首观测时刻, 说明); 不放行 → (None, ...)."""
    if not rows:
        return None, "no_obs"
    t0 = rows[0]["t"]
    stable, brake_end, brake_t, last_t = 0, 0.0, None, None
    last_speed, has_speed_log = None, False
    for o in rows:
        if o["kind"] == "speed":
            has_speed_log = True
            last_speed = o["sp"]
            continue
        if o["t"] < brake_end:  # 刹车期间跳过观测
            continue
        aligned = (o["pe"] <= raw_hi
                   and math.hypot(o["sx"] * 640, o["sy"] * 480) <= smooth_tol
                   and (not o["has_xy"]
                        or (abs(o["dx"]) <= norm and abs(o["dy"]) <= norm)))
        if o["kind"] == "brake":
            brake_end = o["t"] + brake_s
            brake_t = o["t"]
            continue
        if brake_t is not None:  # 刹车后第一个新观测 = 复查帧
            speed_ok = (not has_speed_log
                        or (last_speed is not None and last_speed <= max_speed))
            if aligned and speed_ok:
                sp_txt = (f"speed={last_speed:.3f}≤{max_speed}" if has_speed_log
                          else "无speed日志(视为通过)")
                return (o["t"] - t0,
                        f"RELEASE@t+{o['t']-t0:.1f}s (复查 {o['pe']:.1f}px, {sp_txt})")
            if not aligned:
                brake_t, brake_end, stable = None, 0.0, 0
                continue
            continue
        if not aligned:
            stable = 0
            last_t = None
            continue
        if last_t is not None and o["t"] - last_t > gap_ok:
            stable = 0
        last_t = o["t"]
        stable += 1
        if stable >= stable_frames:
            brake_end = o["t"] + brake_s
            brake_t = o["t"]
    return None, "no_release"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="mission.log 路径 (可多个)")
    ap.add_argument("--group", choices=sorted(GROUPS), default="final",
                    help="预置参数组 (默认 final)")
    ap.add_argument("--raw", type=float, help="精校段 raw 窗口 (px)")
    ap.add_argument("--smooth", type=float, help="smooth 窗口 (px)")
    ap.add_argument("--norm", type=float, help="|dx|,|dy| 归一化阈值")
    ap.add_argument("--stable", type=int, help="连续稳定帧数")
    ap.add_argument("--brake", type=float, help="刹车时长 (s)")
    ap.add_argument("--speed", type=float, help="释放最大水平速度 (m/s)")
    args = ap.parse_args()

    p = dict(GROUPS[args.group])
    for key in ("raw", "smooth", "norm", "stable", "brake", "speed"):
        val = getattr(args, key)
        if val is not None:
            p[key] = val

    print(f"参数组 [{args.group}]: raw={p['raw']} smooth={p['smooth']} "
          f"norm={p['norm']} stable={p['stable']} brake={p['brake']} "
          f"speed={p['speed']}")
    failed = 0
    for path in args.logs:
        rows = parse_log(path)
        dt, msg = replay(rows, p["raw"], p["smooth"], p["norm"], int(p["stable"]),
                         p["brake"], p["speed"])
        ok = dt is not None
        failed += 0 if ok else 1
        print(f"  {path.split('/')[-1]:>28} → {msg}")
    if failed:
        print(f"FAIL: {failed}/{len(args.logs)} 架次未放行")
        return 1
    print("PASS: 全部放行")
    return 0


if __name__ == "__main__":
    sys.exit(main())
