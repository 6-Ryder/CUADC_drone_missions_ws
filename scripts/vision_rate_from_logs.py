"""
从真机飞行日志里量出**视觉管线的实测帧率与采集→可用延迟 τ**.

为什么需要一个脚本: 项目里到处写着"真机视觉 3-5Hz"、τ≈0.2-0.33s、以及由此推出的
"D 被放大约 6.6 倍"、"同一份测量被复用 4-7 次". 这些都是估的, 而 2175 条真机样本
给出的数字完全不同 (见输出). 数字错了不要紧, 但**负载级的前提**错了会把整条设计
推歪 —— 所以把它固化成可复跑的脚本.

⚠️ 两个易读错的地方:
  1. 日志里的 `FPS: N` 字段**不是帧率**: 看 vision_pipeline.py:949-952, 它是
     `1/(单帧耗时)`, 即那一帧处理时间的倒数. 直接当帧率用会偏乐观.
  2. 真实帧率应用 **1/平均耗时**(长尾拉低速率的调和效应), 不是 1/中位耗时.

用法: python3 scripts/vision_rate_from_logs.py
"""
import pathlib
import re
import statistics

PAT = re.compile(r"耗时:\s*(\d+)ms")
ms = []
for root in (pathlib.Path("real_flight_logs"), pathlib.Path("quick_llogs")):
    for f in root.rglob("*.log"):
        ms += [int(x) for x in PAT.findall(f.read_text(errors="ignore"))]

ms = [m for m in ms if m > 0]
mean, med = statistics.mean(ms), statistics.median(ms)
print(f"样本 {len(ms)} 条单帧耗时")
p95 = sorted(ms)[int(0.95 * len(ms))]
print(f"  中位 {med:.0f}ms   平均 {mean:.0f}ms   95% {p95}ms   最大 {max(ms)}ms")
print(f"  ⇒ 帧率估计: 1/平均 = {1000/mean:.1f} Hz   (1/中位 = {1000/med:.1f} Hz, 偏乐观)")
print(f"  ⇒ 采集→可用延迟 τ ≈ 平均 {mean:.0f}ms;  0.25 m/s 下 v·τ ≈ {0.25*mean/1000*100:.1f} cm")
print(f"  ⇒ 20Hz tick / {1000/mean:.0f}Hz 视觉 = 同一份测量被复用 {20*mean/1000:.1f} 次")
