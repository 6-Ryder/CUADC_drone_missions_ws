# Changelog — 2026-07-12

## 概述

诊断低空投放视觉微调算法 (`DropVisualAligner`) 效果差的根因，发现核心问题是 marker 的 YOLO bbox 中心未经过 OpenCV 精修，帧间抖动 50-100px 淹没了梯度信号。实施最小改动修复：为 marker 管线添加 HSV 蓝色分割 + 轮廓矩质心精修。

同时对标 CUADC-UAV-Project 的精校策略，识别出 5 个后续优化方向。

---

## 问题诊断

### 日志分析

对 3 次仿真日志（`simulation_20260711_201416/202352/203055`）进行了逐帧追踪，发现 5 个导致梯度下降失败的问题：

| # | 问题 | 证据 | 严重度 |
|---|------|------|--------|
| 1 | 单采样噪声淹梯度信号 | 同一位置：probe 76px → commit 117px → hover 166px，波动 90px | 🔴 致命 |
| 2 | pixel_error 同一点持续增长 | 每次提交后 error 单向爬升 (183→235→231)，从不收敛 | 🔴 致命 |
| 3 | 目标频繁丢失 | `inf` 连续 6+ 秒，完全无法校准 | 🟠 高 |
| 4 | marker↔cylinder 身份振荡 | Run 1 中 6 帧 0 有效探测，回基线死循环 | 🟡 中 |
| 5 | 15s 超时是唯一终止条件 | 3 次仿真 `stable_count` 从未达到 1，0 次收敛成功 | 🔴 致命 |

### 根因定位

追踪 `vision_pipeline.py` 中 marker 和 cylinder 的发布路径对比：

```
Cylinder: YOLO bbox → CylinderRefiner.refine() → fitEllipse 亚像素中心 → /drone/vision/cylinders
Marker:   YOLO bbox → md.cx/md.cy 直接使用           → /drone/vision/markers  ← 无精修！
```

低空 `DropVisualAligner._current_targets()` 优先使用 marker（因为 cylinder 在 2m 检测不稳定），但 marker 的 `center_u/center_v` 是 YOLO bbox 中心，每帧可跳动 50-100px。

---

## 已实施修复

### 投放区蓝色标识 OpenCV HSV 精修

**文件**: `src/drone_vision/drone_vision/vision_pipeline.py`

新增 `_refine_marker_center()` 静态方法：

```
YOLO bbox → ROI 裁剪 → BGR→HSV → 蓝色分割 (H∈[95,135]) → 形态学开闭 → findContours
→ 最大蓝色轮廓 → cv2.moments() → 亚像素质心
```

- 精修成功 → 用于 `center_u/center_v` + World 坐标计算
- 精修失败 → 自动回退 YOLO bbox 中心
- 新增参数 `drop_marker_refine_enable` (默认 true)
- 定期日志增加 `refined=N/M` 成功率统计

**预期效果**: marker pixel_error 帧间波动从 50-100px 降至 ~10px 以内。

### 参数与文档

| 文件 | 改动 |
|------|------|
| `vision_params.yaml` | 新增 `drop_marker_refine_enable: true` |
| `ARCHITECTURE.md` | 新增 6.2.3b 小节 + 设计决策 "投放标识 OpenCV 精修" |
| `README.md` | 更新 vision_pipeline.py / drop_visual_aligner.py 行数和说明 |

### 提交

```
44d3c1c feat: 投放区蓝色标识 OpenCV HSV 精修，替代 YOLO bbox 中心
```

---

## 对标 CUADC-UAV-Project

阅读了 CUADC 的 `docs/analysis/02-visual-servo-strategy.md` 和 `04-fusion-plan.md`，其精校策略与本项目的当前实现对比：

### 本项目已有的优势（与 CUADC 一致）

| 能力 | 本项目 | CUADC |
|------|--------|-------|
| OpenCV 圆筒椭圆精修 | `CylinderRefiner.fitEllipse()` | `fitEllipse` + `HoughCircles` |
| OpenCV 标识精修 | **本次新增** `_refine_marker_center()` | `cv2.moments(contour)` |
| 多帧追踪 | `CylinderTracker` (IoU + min_hits) | `LockedCylinderTracker` |
| 连续稳定判定 | `is_centered()` 连续 N 帧 | 同机制 |

### 本项目尚未采用的 CUADC 策略

| 策略 | CUADC 做法 | 本项目现状 |
|------|-----------|-----------|
| 归一化像素误差 | `dx_norm = (u-cx)/img_w` — 与高度/分辨率解耦 | 使用像素误差 `du` + `alt/fx` 转换 |
| 机体坐标系速度 | `VelocityBodyYawspeed` — 不依赖偏航角 | `goto_ned` NED 位置指令 |
| 步进-悬停-检测 | 飞 0.12s → 停 → 重新检测 → 再飞 | 连续位置伺服 (goto_ned 每帧) |
| 三级精校 | 3.5m蓝垫粗校 → 2m圆筒粗校 → 1m精校 | 仅 2m 单级 |
| EMA 滤波 | 2m 粗校 α=0.24, 1m 精校 α=0.16 | 无滤波（端点比较法） |

### 后续优化方向

CUADC 的 `04-fusion-plan.md` 已给出完整的融合方案：将 CUADC 的精校控制器移植到本项目的 `MissionFSM` 架构中，替代当前的 `DropVisualAligner` 端点比较法。优先级建议：

1. **P0**: 验证本次 marker 精修对 pixel_error 稳定性的改善
2. **P1**: `DropVisualAligner` 加入 EMA 滤波，不改其余逻辑
3. **P2**: 或直接替换为 CUADC 风格的 P-only + EMA + Body Velocity 控制器
