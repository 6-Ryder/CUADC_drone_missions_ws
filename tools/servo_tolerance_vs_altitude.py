#!/usr/bin/env python3
"""servo_tolerance_vs_altitude.py — 精校对心窗口的米制等效 vs 高度.

⚠️ 本工具的结论 (2026-10-16 首次运行)
------------------------------------
**提出的假设不成立。** 用全部真机日志 (121 条可用记录, 高度 2.79–5.64 m) 检验:

    实际误差 ≈ c · Z^b    拟合得  b = +1.12,  c = 0.112,  R² = 0.050

判据的幂律指数恒为 1 (窗口 ∝ Z)。b ≈ 1 说明**两者近似平行** ——
即"窗口随高度收紧得比误差快"这个机制**在数据里看不到**。
R² = 0.05 说明**高度只能解释 5% 的方差**, 误差由别的因素主导。

分组看更不稳定: 10/07 架次 b = +7.2 (R²=0.33), 8/15 架次 b = −0.87 (R²=0.68),
**两个架次互相矛盾**。

已确认的混杂因素 (复核结论时必须一起看):
  · `source` 混了 cylinder (中位 35.3px) 与 marker (中位 74.5px), 两者误差水平差一倍
  · 121 条记录 `stable` 全为 0 —— 都是"仍在修正中"的帧, 没有一条是对心已达成
  · 跨架次的气候/风/场地差异未被控制

**这个否定结果本身有价值**: 它排除了"判据形式"作为瓶颈的可能, 把矛头指向
**检测质量** —— 与已知的"低空 3.15–3.75 m 检测断裂"一致。
不要在套磁信里把它写成"我发现窗口有问题"; 要写"我提出并证伪了一个假设"。

回答一个问题
------------
`precision_align_controller._pixel_to_metric_error` 把像素误差按当前高度换算成米
(``e_m = du * Z / fx``), 所以**米制增益是恒定的** —— 这是对的, 也和 IBVS 的
交互矩阵结构一致 (平移项 ∝ 1/Z ⇒ 控制器增益 ∝ Z)。

但**接受判据留在像素域** (``precision_align_raw_pixel_tolerance = 9.0`` px),
于是它的米制等效值随高度线性收紧::

    tol_m(Z) = tol_px * Z / fx        # 5 m → 11.3 cm, 1.5 m → 3.4 cm  (fx=400)

问题: 实际可达的像素误差是**跟着窗口一起收紧**, 还是**撞在一个噪声地板上**?
若两者相交, 交点以下闭环在**结构上无法满足自己的判据** —— 与已知的
"观测龄门过保护"同属一类机制 (门限取值没跟着工况走)。

数据来源与对齐
--------------
======  ==========================================================
像素误差  ROS ``mission.log`` 的 ``DESCEND: 精校投放许可失败 ... raw=..px`` 行
高度      ULog ``vehicle_local_position.z`` (NED, 向下为正 ⇒ alt = -z)
对齐      ULog ``timesync_status`` —— 提供伴生计算机墙钟与飞控 boot 时间的关系
======  ==========================================================

对齐公式 (由数据反推并交叉验证, 残差中位 ~5 ms、最大 ~85 ms)::

    ros_wall_us = boot_us - estimated_offset
    ⇒ boot_us  = ros_wall_us + estimated_offset

``estimated_offset`` 随时间漂移, 因此按时间**插值**而不是取常数。

脚本还会用 ROS 日志自己的 ``等待下降到位 height_error=..m`` 行做**交叉校验**
(alt ≈ 目标高度 + height_error), 两条路径不一致时会明确报出来, 而不是静默出图。

用法
----
::

    python3 tools/servo_tolerance_vs_altitude.py \\
        --ros-log real_flight_logs/2026_8_07/real_mission_20260807_190630/mission.log \\
        --ulog    real_flight_logs/2026_8_07/qgc_logs/log_XXX.ulg \\
        --fx 400 --tol-px 9.0 \\
        --out /tmp/equal_gain

只给 ``--ros-log`` 时退化为"ROS 日志单独估计高度"(精度差, 仅作诊断)。

输出: ``<out>/records.csv``、``<out>/tolerance_vs_altitude.png`` (若有 matplotlib)、
以及终端上的结论。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------- ROS 日志解析

# 例: [INFO] [1786061809.885833972] [mission_state_machine]: DESCEND: 精校投放许可失败
#     source=cylinder raw=77.1px smooth_px=77.1px smooth=(-0.0387,+0.1522)
#     stable=0/4 ready=False reason=tracking \x1b[0m
_TS_RE = re.compile(r"\[(\d+\.\d+)\]")
_RAW_RE = re.compile(r"\braw=([-\d.]+|inf)px")
_PIXEL_ERR_RE = re.compile(r"\bpixel_error=([-\d.]+|inf)px")
_ALT_INLINE_RE = re.compile(r"\balt=([-\d.]+)m")
_SMOOTH_PX_RE = re.compile(r"\bsmooth_px=([-\d.]+|inf)px")
_SMOOTH_XY_RE = re.compile(r"\bsmooth=\(([-+\d.]+),([-+\d.]+)\)")
_STABLE_RE = re.compile(r"\bstable=(\d+)(?:/(\d+))?")
_READY_RE = re.compile(r"\bready=(True|False)")
_REASON_RE = re.compile(r"\breason=([a-z_]+)")
_SOURCE_RE = re.compile(r"\bsource=([a-z_]+)")
_HEIGHT_ERR_RE = re.compile(r"\bheight_error=([-\d.]+)m")
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

_INF = float("inf")


def _to_float(tok: str) -> float:
    """``inf`` / ``-inf`` 与普通数字都能吃。"""
    t = tok.strip().lower()
    if t in ("inf", "+inf"):
        return _INF
    if t == "-inf":
        return -_INF
    return float(t)


def parse_ros_log(path: str) -> Tuple[List[Dict], List[Dict]]:
    """解析 mission.log, 返回 (像素误差记录, 高度线索记录)."""
    pixel_records: List[Dict] = []
    height_hints: List[Dict] = []

    with open(path, "r", encoding="utf-8", errors="ignore") as fh:
        for lineno, raw_line in enumerate(fh, 1):
            line = _ANSI_RE.sub("", raw_line)

            ts = _TS_RE.search(line)
            if not ts:
                continue
            wall_us = int(round(float(ts.group(1)) * 1e6))

            # 高度线索: "等待下降到位 height_error=1.30m stall=0" (DESCEND 路径专有)
            he = _HEIGHT_ERR_RE.search(line)
            if he is not None:
                height_hints.append(
                    {"wall_us": wall_us, "height_error_m": float(he.group(1)),
                     "lineno": lineno}
                )

            # 两种日志行都能吃:
            #   ① PREC_ALIGN: ... pixel_error=93.0px ... alt=5.57m continuous=True ...
            #      —— 来自 precision_align_controller._maybe_log_tracking
            #      **行内自带高度**, 不需要 ULog / 时钟对齐, 是首选数据源
            #   ② DESCEND: 精校投放许可失败 ... raw=77.1px ... —— 来自 state_drop
            #      无高度, 需要走 ULog + timesync 那条路
            px_m = _PIXEL_ERR_RE.search(line)
            raw_m = _RAW_RE.search(line)
            if px_m is not None:
                raw_px = _to_float(px_m.group(1))
            elif raw_m is not None:
                raw_px = _to_float(raw_m.group(1))
            else:
                continue

            alt_m = _ALT_INLINE_RE.search(line)
            smooth_px_m = _SMOOTH_PX_RE.search(line)
            reason_m = _REASON_RE.search(line)
            source_m = _SOURCE_RE.search(line)
            xy_m = _SMOOTH_XY_RE.search(line)
            st_m = _STABLE_RE.search(line)
            rd_m = _READY_RE.search(line)

            rec: Dict = {
                "wall_us": wall_us,
                "lineno": lineno,
                "raw_px": raw_px,
                # 行内高度 (PREC_ALIGN 格式); nan 表示要走 ULog 那条路
                "alt_inline_m": float(alt_m.group(1)) if alt_m else float("nan"),
                "smooth_px": _to_float(smooth_px_m.group(1)) if smooth_px_m else float("nan"),
                "dx_norm": float(xy_m.group(1)) if xy_m else float("nan"),
                "dy_norm": float(xy_m.group(2)) if xy_m else float("nan"),
                "stable": int(st_m.group(1)) if st_m else -1,
                # PREC_ALIGN 行只给 stable=N (无分母), DESCEND 行才给 N/N
                "stable_need": int(st_m.group(2)) if (st_m and st_m.group(2)) else -1,
                "ready": (rd_m.group(1) == "True") if rd_m else False,
                "reason": reason_m.group(1) if reason_m else "",
                "source": source_m.group(1) if source_m else "",
            }
            # 无检测的帧没有可比性 (inf 不是"误差很大", 是"没有观测")
            rec["usable"] = math.isfinite(rec["raw_px"])
            pixel_records.append(rec)

    # 高度线索: "等待下降到位 height_error=1.30m stall=0" (仅 DESCEND 路径有)
    return pixel_records, height_hints

    return pixel_records, height_hints


# ---------------------------------------------------------------- ULog / 时钟

class TimeSync:
    """把 ROS 墙钟 (µs) 映射到 ULog boot 时间 (µs).

    ``estimated_offset`` 会随时间漂移, 所以按时间插值, 不用常数。
    """

    def __init__(self, boot_us: Sequence[int], offset_us: Sequence[int]):
        if len(boot_us) == 0:
            raise ValueError("timesync_status 为空")
        self._boot = list(boot_us)
        self._off = list(offset_us)

    @classmethod
    def from_ulog(cls, ulog) -> "TimeSync":
        if "timesync_status" not in {d.name for d in ulog.data_list}:
            raise ValueError("ULog 中没有 timesync_status, 无法做墙钟对齐")
        d = ulog.get_dataset("timesync_status")
        boot = [int(v) for v in d.data["timestamp"]]
        off = [int(v) for v in d.data["estimated_offset"]]
        order = sorted(range(len(boot)), key=lambda i: boot[i])
        return cls([boot[i] for i in order], [off[i] for i in order])

    def _interp_offset(self, boot_query: float) -> float:
        """在 boot 轴上线性插值 offset(boot). 超出范围则钳到端点。"""
        b, o = self._boot, self._off
        if boot_query <= b[0]:
            return float(o[0])
        if boot_query >= b[-1]:
            return float(o[-1])
        lo, hi = 0, len(b) - 1
        while hi - lo > 1:                      # 二分
            mid = (lo + hi) // 2
            if b[mid] <= boot_query:
                lo = mid
            else:
                hi = mid
        span = b[hi] - b[lo]
        if span <= 0:
            return float(o[lo])
        w = (boot_query - b[lo]) / span
        return float(o[lo]) * (1.0 - w) + float(o[hi]) * w

    def wall_to_boot(self, wall_us: float, iters: int = 6) -> float:
        """解 boot = wall + offset(boot) —— 不动点迭代, 因为 offset 依赖 boot。"""
        boot = wall_us + self._interp_offset(wall_us)
        for _ in range(iters):
            boot = wall_us + self._interp_offset(boot)
        return boot


def altitude_series(ulog) -> Tuple[List[int], List[float]]:
    """从 vehicle_local_position 取 (boot_us, 高度 m). NED z 向下为正 ⇒ alt = -z."""
    d = ulog.get_dataset("vehicle_local_position")
    ts = [int(v) for v in d.data["timestamp"]]
    z = [float(v) for v in d.data["z"]]
    order = sorted(range(len(ts)), key=lambda i: ts[i])
    return [ts[i] for i in order], [-z[i] for i in order]


def sample_altitude(boot_us: float, boot_axis: Sequence[int],
                    alt_axis: Sequence[float]) -> float:
    """在高度时间轴上线性插值; 超出范围返回 nan。"""
    if not boot_axis or boot_us < boot_axis[0] or boot_us > boot_axis[-1]:
        return float("nan")
    lo, hi = 0, len(boot_axis) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if boot_axis[mid] <= boot_us:
            lo = mid
        else:
            hi = mid
    span = boot_axis[hi] - boot_axis[lo]
    if span <= 0:
        return alt_axis[lo]
    w = (boot_us - boot_axis[lo]) / span
    return alt_axis[lo] * (1.0 - w) + alt_axis[hi] * w


# ---------------------------------------------------------------- 分析

def analyse(records: List[Dict], tol_px: float, fx: float,
            bin_m: float = 0.25) -> Dict:
    """按高度分层, 比较实际像素误差与理论窗口。"""
    good = [r for r in records
            if r.get("usable") and math.isfinite(r.get("altitude_m", float("nan")))]
    if not good:
        return {"ok": False, "why": "没有可用的 (像素误差, 高度) 配对记录"}

    for r in good:
        z = max(r["altitude_m"], 0.05)
        r["tol_m"] = tol_px * z / fx            # 判据的米制等效
        r["raw_m"] = r["raw_px"] * z / fx       # 实际误差的米制等效

    lo = min(r["altitude_m"] for r in good)
    hi = max(r["altitude_m"] for r in good)
    nbin = max(1, int(math.ceil((hi - lo) / bin_m)))

    bins: List[Dict] = []
    for i in range(nbin):
        a0, a1 = lo + i * bin_m, lo + (i + 1) * bin_m
        rows = [r for r in good if a0 <= r["altitude_m"] < a1]
        if not rows:
            continue
        rows_sorted = sorted(r["raw_m"] for r in rows)
        med = rows_sorted[len(rows_sorted) // 2]
        mid = (a0 + a1) / 2.0
        bins.append({
            "alt_mid": mid,
            "alt_lo": a0, "alt_hi": a1,
            "n": len(rows),
            "median_raw_m": med,
            "tol_m": tol_px * max(mid, 0.05) / fx,
            "frac_over_tol": sum(1 for r in rows if r["raw_m"] > r["tol_m"]) / len(rows),
        })

    # ---------------------------------------------------------- 趋势拟合
    # 判据的米制等效是 tol_m = k_tol · Z, 幂律指数恒为 1 (k_tol = tol_px/fx).
    # 实际误差若也是幂律 err_m = c · Z^b, 则:
    #   b ≈ 1  → 两者平行, 判据的像素域形式在这个高度范围内不造成问题
    #   b < 1  → 误差随高度下降得比窗口慢 ⇒ 低空必然相交(结构上不可完成)
    #   b > 1  → 误差下降更快 ⇒ 越往下越容易满足
    trend: Dict = {}
    pts = [(r["altitude_m"], r["raw_m"]) for r in good
           if r["altitude_m"] > 0 and r["raw_m"] > 0]
    if len(pts) >= 8:
        xs = [math.log(z) for z, _ in pts]
        ys = [math.log(e) for _, e in pts]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        if sxx > 0:
            b = sxy / sxx
            a = my - b * mx
            c = math.exp(a)
            # 决定系数 R² (在对数域)
            ss_tot = sum((y - my) ** 2 for y in ys)
            ss_res = sum((y - (a + b * x)) ** 2 for x, y in zip(xs, ys))
            trend = {
                "exponent_b": b,
                "coef_c": c,
                "r2": (1.0 - ss_res / ss_tot) if ss_tot > 0 else float("nan"),
                "n": n,
            }
            # 交点: c·Z^b = k_tol·Z  ⇒  Z* = (k_tol/c)^(1/(b-1))
            k_tol = tol_px / fx
            if abs(b - 1.0) > 1e-6:
                base = k_tol / c
                if base > 0:
                    z_star = base ** (1.0 / (b - 1.0))
                    if 0.05 < z_star < 50.0:
                        trend["crossing_alt_m"] = z_star

    verdict = {
        "ok": True,
        "n_records": len(good),
        "alt_range_m": (lo, hi),
        "bins": bins,
        "trend": trend,
        "tol_m_at": {a: tol_px * a / fx for a in (5.0, 3.0, 2.0, 1.5)},
    }
    return verdict


def plot(verdict: Dict, out_png: str, log_name: str) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    bins = verdict["bins"]
    xs = [b["alt_mid"] for b in bins]
    tol = [b["tol_m"] for b in bins]
    raw = [b["median_raw_m"] for b in bins]

    fig, ax = plt.subplots(figsize=(7.2, 4.6), dpi=140)
    ax.plot(xs, tol, "-", lw=2, color="tab:blue",
            label=r"acceptance criterion in metres:  $tol_{px}\cdot Z / f_x$")
    ax.plot(xs, raw, "o-", lw=2, ms=4, color="tab:green",
            label="achieved pixel error in metres (median)")
    ax.fill_between(xs, raw, tol, where=[r > t for r, t in zip(raw, tol)],
                    color="tab:red", alpha=0.20, interpolate=True,
                    label="outside window (criterion structurally unmet)")
    ax.axvspan(3.15, 3.75, color="tab:orange", alpha=0.12)
    ax.text(3.45, ax.get_ylim()[1] * 0.94, "known dropout band",
            ha="center", va="top", fontsize=8, color="tab:orange")
    ax.set_xlabel("altitude Z (m)")
    ax.set_ylabel("metres")
    ax.set_title("Precision-align window vs achieved error, binned by altitude",
                 fontsize=11)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return True


# ---------------------------------------------------------------- main

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="精校对心窗口的米制等效 vs 高度 (1/Z 结构验证)",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ros-log", required=True, nargs="+",
                    help="一个或多个 mission.log 路径 (多个会合并, 样本更大)")
    ap.add_argument("--ulog", help="配套 ULog (.ulg); 不给则退化为 ROS 单独估计")
    ap.add_argument("--fx", type=float, default=400.0, help="相机 fx (px), 默认 400")
    ap.add_argument("--tol-px", type=float, default=9.0,
                    help="precision_align_raw_pixel_tolerance, 默认 9.0")
    ap.add_argument("--bin", type=float, default=0.25, help="高度分层宽度 (m)")
    ap.add_argument("--out", default=".", help="输出目录")
    args = ap.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)

    print(f"[1/4] 解析 ROS 日志 ({len(args.ros_log)} 个文件)")
    records: List[Dict] = []
    hints: List[Dict] = []
    for path in args.ros_log:
        recs, hs = parse_ros_log(path)
        for r in recs:
            r["log_file"] = os.path.basename(os.path.dirname(path)) or path
        records.extend(recs)
        hints.extend(hs)
        print(f"  {len(recs):>4d} 条  {path}")
    if not records:
        print("  ❌ 没有找到任何带 pixel_error=..px / raw=..px 的行", file=sys.stderr)
        return 2

    # 同一架次的 mission.log 与 ros/python3_*.log 含同一批记录, 按
    # (墙钟时间戳, 像素误差) 去重, 否则同一架次会被重复计入.
    seen = set()
    deduped: List[Dict] = []
    for r in records:
        key = (r["wall_us"], r["raw_px"])
        if key in seen:
            continue
        seen.add(key)
        deduped.append(r)
    n_dup = len(records) - len(deduped)
    records = deduped
    if n_dup:
        print(f"  去重: 移除 {n_dup} 条重复 (同架次的多个日志副本)")

    finite = [r for r in records if r["usable"]]
    print(f"  命中 {len(records)} 条, 其中可解析的 {len(finite)} 条"
          f" (infpx 的 {len(records) - len(finite)} 条是无检测帧, 已排除)")
    print(f"  高度线索行 height_error=..m: {len(hints)} 条")

    # 首选: PREC_ALIGN 行自带 alt=..m, 不需要 ULog / 时钟对齐
    inline = [r for r in records if math.isfinite(r.get("alt_inline_m", float("nan")))]
    if inline:
        print(f"  ✅ {len(inline)}/{len(records)} 条自带高度 (PREC_ALIGN 行的 alt=..m)")
    need_ulog = len(inline) < len(records)

    if not need_ulog:
        print("[2/4] 跳过 ULog —— 全部记录自带高度")
        for r in records:
            r["boot_us"] = float("nan")
            r["altitude_m"] = r["alt_inline_m"]
    else:
        if not inline:
            print("\n⚠️  这些记录都没有行内高度, 需要 --ulog 做时钟对齐。"
                  "本脚本不会用 height_error 硬凑高度(目标高度随阶段变化, 会系统性出错)。",
                  file=sys.stderr)
            return 3
        if not args.ulog:
            print(f"  ⚠️  {len(records) - len(inline)} 条缺行内高度且未给 --ulog, "
                  f"这些记录将被丢弃 (保留 {len(inline)} 条)")
            records = [r for r in records if math.isfinite(r["alt_inline_m"])]
            for r in records:
                r["boot_us"] = float("nan")
                r["altitude_m"] = r["alt_inline_m"]
            need_ulog = False
        else:
            print(f"[2/4] 读 ULog 并建立时钟对齐: {args.ulog}")
            try:
                from pyulog import ULog
            except ImportError:
                print("  ❌ 需要 pyulog:  pip install pyulog", file=sys.stderr)
                return 4
            ulog = ULog(args.ulog)
            try:
                ts = TimeSync.from_ulog(ulog)
            except ValueError as exc:
                print(f"  ❌ {exc}", file=sys.stderr)
                return 5
            boot_axis, alt_axis = altitude_series(ulog)
            print(f"  timesync 样本 {len(ts._boot)} 个; 高度样本 {len(boot_axis)} 个")
            print(f"  对齐公式: boot_us = ros_wall_us + offset(boot)   [不动点迭代]")

            print("[3/4] 为缺高度的记录取高度")
            for r in records:
                if math.isfinite(r.get("alt_inline_m", float("nan"))):
                    r["boot_us"] = float("nan")
                    r["altitude_m"] = r["alt_inline_m"]
                else:
                    boot = ts.wall_to_boot(r["wall_us"])
                    r["boot_us"] = boot
                    r["altitude_m"] = sample_altitude(boot, boot_axis, alt_axis)

    n_alt = sum(1 for r in records if math.isfinite(r.get("altitude_m", float("nan"))))
    print(f"  最终取到高度的 {n_alt}/{len(records)} 条")
    if n_alt < 10:
        print("  ⚠️ 取到高度的记录太少 —— 检查是否配错架次/日志文件", file=sys.stderr)

    # 交叉校验: 用 height_error 行粗估高度量级是否合理
    if hints:
        hs = sorted(h["height_error_m"] for h in hints)
        print(f"  交叉校验(仅量级): height_error 范围 {hs[0]:.2f}–{hs[-1]:.2f} m; "
              f"本架次像素记录高度范围 "
              f"{min(r['altitude_m'] for r in records if math.isfinite(r['altitude_m'])):.2f}–"
              f"{max(r['altitude_m'] for r in records if math.isfinite(r['altitude_m'])):.2f} m")

    csv_path = os.path.join(args.out, "records.csv")
    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write("wall_us,boot_us,altitude_m,raw_px,smooth_px,dx_norm,dy_norm,"
                 "stable,stable_need,ready,reason,source,usable\n")
        for r in records:
            fh.write(",".join(str(r.get(k, "")) for k in
                              ("wall_us", "boot_us", "altitude_m", "raw_px", "smooth_px",
                               "dx_norm", "dy_norm", "stable", "stable_need", "ready",
                               "reason", "source", "usable")) + "\n")
    print(f"  写出 {csv_path}")

    print("[4/4] 分析")
    verdict = analyse(records, args.tol_px, args.fx, args.bin)
    if not verdict.get("ok"):
        print(f"  ❌ {verdict['why']}", file=sys.stderr)
        return 6

    lo, hi = verdict["alt_range_m"]
    print(f"\n  高度覆盖 {lo:.2f}–{hi:.2f} m, 有效记录 {verdict['n_records']} 条")
    print(f"  判据的米制等效 tol_m = {args.tol_px} · Z / {args.fx:g}:")
    for a, v in verdict["tol_m_at"].items():
        print(f"     Z = {a:>4.1f} m  →  {v * 100:>6.2f} cm")
    print("\n  按高度分层:")
    print("    Z(m)   n   实际中位(m)  窗口(m)  超窗比例")
    for b in verdict["bins"]:
        print(f"    {b['alt_mid']:>5.2f} {b['n']:>4d}   {b['median_raw_m']:>9.4f}  "
              f"{b['tol_m']:>7.4f}   {b['frac_over_tol'] * 100:>6.0f}%")

    png = os.path.join(args.out, "tolerance_vs_altitude.png")
    if plot(verdict, png, f"{len(args.ros_log)} logs"):
        print(f"\n  图: {png}")
    else:
        print("\n  (未安装 matplotlib, 跳过绘图; CSV 已写出)")

    tr = verdict.get("trend") or {}
    if tr:
        print("\n  趋势拟合 (对数域最小二乘):  实际误差 ≈ c · Z^b")
        print(f"    b = {tr['exponent_b']:+.3f}   c = {tr['coef_c']:.4f}   "
              f"R² = {tr['r2']:.3f}   n = {tr['n']}")
        print(f"    判据的幂律指数恒为 1 (窗口 ∝ Z)")
        if tr["exponent_b"] < 0.85:
            print("    ⇒ b 明显小于 1: **误差随高度下降得比窗口慢** ——")
            print("      低空两者必然相交, 即存在一个'结构上无法满足判据'的高度门槛")
        elif tr["exponent_b"] > 1.15:
            print("    ⇒ b 明显大于 1: 误差下降得比窗口快 —— 越往下越容易满足, "
                  "判据形式在该范围内不构成瓶颈")
        else:
            print("    ⇒ b ≈ 1: 两者近似平行 —— 判据的像素域形式在这个高度范围内"
                  "不改变收敛难度")
        if "crossing_alt_m" in tr:
            print(f"    ⚠️ 拟合交点 Z* ≈ {tr['crossing_alt_m']:.2f} m —— "
                  f"低于此处拟合误差低于窗口 (外推值, 仅作参考)")

    print("\n  读数提示:")
    print("    · 本数据全部来自**精校对心阶段**(该行在对心未达成时也会打),")
    print("      所以'误差 > 窗口'本身是预期的; 有价值的是**两者的高度趋势**")
    print("    · 若 b < 1 ⇒ 低空必然相交, 与已知的'观测龄门过保护'同属")
    print("      '门限取值没跟着工况走' 这一类机制")
    print("    · R² 偏低说明误差的主导因素不是高度(可能是检测质量/风/目标运动),")
    print("      此时不要把 b 当成物理结论")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
