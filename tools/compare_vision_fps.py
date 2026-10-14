#!/usr/bin/env python3
"""compare_vision_fps.py — 真机复飞前后视觉 FPS/耗时对比分析.

解析 mission.log 中 vision_pipeline 的 "FPS: N | 耗时: Xms | 高度: Ym"
行 (每 100 帧自动输出一次), 按高度分档统计, 并定位投放区精校窗口
(PREC_ALIGN 首末行时间范围), 输出两架次对比表与达标判断.

判断阈值 (基于优化前基线: 10:01 低空 195-367ms/3-5 FPS, 8.05 低空 125-171ms/6-8):
  - 低空段耗时中位 ≤120ms 且 FPS ≥8 → PASS (方案 A 预期 50-120ms)
  - ≤200ms 或 FPS ≥5          → PARTIAL (改善但未达标, 考虑 ORT_INTRA_OP_THREADS / TensorRT)
  - 其余                       → FAIL (热点不在精修, 需更细粒度耗时拆分)

用法:
  python3 tools/compare_vision_fps.py <基线mission.log> <优化后mission.log> [...]
"""

import re
import sys
from statistics import median
from typing import Dict, List, Optional, Tuple

RE_FPS = re.compile(
    r"\[(?P<t>[\d.]+)\].*?FPS: (?P<fps>\d+) \| 耗时: (?P<ms>\d+)ms "
    r"\| 高度: (?P<h>-?[\d.]+)m")
RE_ALIGN = re.compile(r"\[(?P<t>[\d.]+)\].*?PREC_ALIGN")

# 高度分档 (NED down, 负值=起飞前地面/爬升)
LOW_MIN, LOW_MAX = 1.0, 3.5


def analyze(path: str) -> Optional[Dict]:
    rows: List[Tuple[float, int, int, float]] = []  # (t, fps, ms, h)
    align_ts: List[float] = []
    for line in open(path, errors="ignore"):
        line = line.replace("\x1b[0m", "").replace("\x1b[32m", "")
        m = RE_FPS.search(line)
        if m:
            rows.append((float(m["t"]), int(m["fps"]), int(m["ms"]),
                         float(m["h"])))
            continue
        m = RE_ALIGN.search(line)
        if m:
            align_ts.append(float(m["t"]))
    if not rows:
        return None

    def bucket(lo: float, hi: float) -> Dict:
        sub = [r for r in rows if lo <= r[3] < hi]
        return dict(
            n=len(sub),
            fps_med=median(r[1] for r in sub) if sub else None,
            ms_med=median(r[2] for r in sub) if sub else None,
            ms_avg=sum(r[2] for r in sub) / len(sub) if sub else None,
        )

    # 投放区精校窗口: PREC_ALIGN 首末行之间
    win = None
    if align_ts:
        t0, t1 = min(align_ts), max(align_ts)
        sub = [r for r in rows if t0 <= r[0] <= t1]
        win = dict(
            t0=t0, t1=t1,
            n=len(sub),
            fps_med=median(r[1] for r in sub) if sub else None,
            ms_med=median(r[2] for r in sub) if sub else None,
        )
    return dict(
        total=len(rows),
        ground=bucket(-999.0, LOW_MIN),   # 地面/爬升/返航低空
        low=bucket(LOW_MIN, LOW_MAX),     # 低空精校段 (投放区)
        cruise=bucket(LOW_MAX, 999.0),    # 巡航
        window=win,
    )


def verdict(d: Dict) -> Tuple[str, str]:
    low = d["low"]
    if low["n"] == 0:
        return "N/A", "低空段无采样 (FPS 行每 100 帧一条, 精校期太短会无采样)"
    ms, fps = low["ms_med"], low["fps_med"]
    if ms <= 120.0 and fps >= 8:
        return "PASS", f"低空耗时 {ms:.0f}ms / {fps:.0f} FPS — 方案 A 达标"
    if ms <= 200.0 or fps >= 5:
        return "PARTIAL", f"低空耗时 {ms:.0f}ms / {fps:.0f} FPS — 改善但未达标"
    return "FAIL", f"低空耗时 {ms:.0f}ms / {fps:.0f} FPS — 无明显改善"


def fmt(d: Dict) -> str:
    lines = [f"  {'时段':<10} {'n':>4} {'FPS中位':>7} {'耗时中位':>8} {'耗时均值':>8}"]
    for name, key in (("地面/爬升", "ground"), ("低空精校", "low"),
                      ("巡航", "cruise")):
        b = d[key]
        if b["n"]:
            lines.append(
                f"  {name:<10} {b['n']:>4} {b['fps_med']:>7.1f} "
                f"{b['ms_med']:>8.0f} {b['ms_avg']:>8.0f}")
        else:
            lines.append(f"  {name:<10} {0:>4} {'-':>7} {'-':>8} {'-':>8}")
    w = d["window"]
    if w and w["n"]:
        lines.append(
            f"  投放区窗口 {w['n']:>4} {w['fps_med']:>7.1f} {w['ms_med']:>8.0f} "
            f"({w['t1'] - w['t0']:.0f}s 窗内)")
    else:
        lines.append("  投放区窗口: 无 FPS 采样 (精校窗口内 FPS 行不足)")
    return "\n".join(lines)


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    results = []
    for p in sys.argv[1:]:
        d = analyze(p)
        name = p.split("/")[-2] if "/" in p else p
        if d is None:
            print(f"[{name}] 未找到 FPS 行 — 日志无效?")
            return 1
        results.append((p, name, d))

    print("=" * 64)
    for i, (p, name, d) in enumerate(results):
        tag = "基线" if i == 0 else "对比"
        print(f"[{tag}] {name}")
        print(fmt(d))
        v, why = verdict(d)
        print(f"  判断: {v} — {why}")
        print("-" * 64)

    if len(results) == 2:
        lo_a, lo_b = results[0][2]["low"], results[1][2]["low"]
        if lo_a["n"] and lo_b["n"]:
            print(
                f"结论: 低空耗时 {lo_a['ms_med']:.0f}ms/{lo_a['fps_med']:.1f}FPS "
                f"→ {lo_b['ms_med']:.0f}ms/{lo_b['fps_med']:.1f}FPS "
                f"(改善 {lo_a['ms_med'] / max(lo_b['ms_med'], 1):.1f}x)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
