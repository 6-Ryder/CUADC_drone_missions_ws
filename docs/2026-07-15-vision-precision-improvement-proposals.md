# 视觉识别精度与稳定性优化方案

> 2026-07-15 | 针对降落 H 标识振荡掉帧 + 投放区蓝色标识误检问题

## 当前架构回顾

```
Camera(1280×960) → YOLO(640×640) → OpenCV 精修 → IoU追踪(min_hits=3)
→ 姿态补偿坐标变换 → 视觉伺服 PID → /drone/vision/servo_cmd
```

两个独立单类 YOLO 模型:
- `cylinder_yolo.pt` (YOLOv8n): 白色圆筒检测
- `drop_marker_yolo.pt` (YOLO11n): 蓝色投放区标识检测
- H 降落标识: 无专用模型, 依赖 `LandingPadRefiner` OpenCV 后处理

---

## 根因分析

### 降落 H 标识振荡 → 掉帧

| 环节 | 问题 | 代码位置 |
|------|------|----------|
| YOLO bbox 跳动 | 640×640 输入下远处 H 标识仅数十像素, bbox 帧间跳动 20-50px | `yolo_engine.py` |
| OpenCV 精修不稳定 | 自适应阈值+轮廓检测对光照/模糊敏感, 精修失败回退 bbox 中心 → 跳变 50-100px | `landing_detector.py:60-104` |
| 无时序滤波 | 检测结果直接送伺服 PID, 无 Kalman/EMA 平滑 | `vision_pipeline.py:705-734` |
| 降落无高度自适应 | 圆筒有 `AltitudeAdaptiveParams`, 降落 conf 阈值固定 0.3 | `vision_pipeline.py:705` |
| 追踪 min_hits=3 | 抖动超 IoU 阈值即掉帧, 需重新累计 3 帧 | `cylinder_detector.py:170` |

### 投放区蓝色标识误检

| 环节 | 问题 | 代码位置 |
|------|------|----------|
| HSV 范围过宽 | `H∈[95,135]` 覆盖天空蓝到深蓝, 未调优饱和度下限 | `vision_pipeline.py:873-875` |
| 缺乏形状约束 | HSV 分割后仅检查面积>80, 不验证圆形度 | `vision_pipeline.py:886-892` |
| 无追踪去重 | 投放区标识不经 tracker (圆筒有), 单帧误检直接输出 | `vision_pipeline.py:449-505` |

---

## 方案 A: 参数调优 + 时序滤波 (轻量)

**核心思路**: 不改架构, 在现有管线加低通滤波 + 收紧参数。

### 具体改动

1. **检测结果 EMA 平滑**
   - 在 `_process_landing_v2` 和投放区检测中对 `(cx, cy, confidence)` 做指数移动平均
   - 平滑系数 α=0.3~0.5, 抑制 bbox 帧间跳动
   - 向量改动: ~20 行

2. **降落高度自适应 conf**
   - 启用 `AltitudeAdaptiveParams.get_landing_params()`, 已实现但未被 pipeline 调用
   - 高度越低 conf 阈值越低 (目标更大更易检测)
   - 向量改动: ~5 行

3. **投放区蓝色标识加形状约束**
   - HSV 分割后验证轮廓圆形度 >0.6
   - 已知蓝色标识 Ø=1m, 根据高度计算预期像素半径范围, 过滤不符的轮廓
   - 向量改动: ~15 行

4. **投放区标识加简单追踪**
   - 复用 `CylinderTracker`, min_hits=2, 过滤单帧闪烁误检
   - 向量改动: ~10 行

| 优点 | 缺点 |
|------|------|
| 改动 ~50 行, 风险极低 | 振荡只能抑制不能根除 |
| 不改模型, 不增加推理延迟 | 模型本身检测能力无提升 |
| 可立即部署验证 | 极端光照/角度下仍有抖动 |

---

## 方案 B: 模型升级 + OpenCV 精修改进 (中等)

**核心思路**: 提升模型检测质量 + 改进后处理鲁棒性。

### 具体改动

1. **提升 YOLO 输入分辨率**
   - 640 → 960 或 1280
   - 远处 H 标识像素数翻倍, bbox 更稳定
   - 代价: 推理延迟 15ms → 30ms (GPU 仍可维持实时)
   - 向量改动: 1 行参数

2. **H 标识精修改用椭圆拟合**
   - 当前 `LandingPadRefiner` 用自适应阈值+minEnclosingCircle
   - 改用类似 `CylinderRefiner` 的 Canny+椭圆拟合, 圆心更稳定
   - 向量改动: ~30 行

3. **投放区 HSV 自适应**
   - 根据图像整体亮度 (V 均值) 动态调整 HSV 阈值
   - 光照充足时收紧范围, 光照不足时放宽
   - 向量改动: ~20 行

4. **重新训练 YOLO 模型** (需要数据集)
   - 增强数据: 运动模糊、不同角度、不同光照、部分遮挡
   - 针对振荡/误检场景做 hard negative mining
   - 训练时间: 1-2 天 (有数据的前提下)

| 优点 | 缺点 |
|------|------|
| 从源头提升检测质量 | 需要重新训练模型 |
| 精修算法鲁棒性提升 | 推理延迟增加 (可用 TensorRT 抵消) |
| 分辨率提升直接改善远处检测 | 数据集可能不够 |

---

## 方案 C: 多帧融合 + Kalman 追踪 + 模型冗余 (重改)

**核心思路**: 不依赖单帧检测, 用多帧信息融合 + 状态估计。

### 具体改动

1. **Kalman 滤波器跟踪降落目标**
   - H 标识世界坐标用匀速模型 Kalman 滤波
   - 预测+更新替代原始检测值
   - 单帧漏检用预测值补齐 → 不掉帧
   - 向量改动: ~80 行

2. **多尺度检测 (ROI 放大)**
   - 降落场景同时跑全图 + ROI 放大 (上一帧检测位置 crop 2×)
   - 远处用全图, 近处用 ROI 提高精度
   - 向量改动: ~60 行

3. **投放区独立追踪**
   - 与圆筒追踪同架构, min_hits=2~3
   - 向量改动: ~20 行

4. **冗余验证**
   - YOLO + OpenCV 双路检测取交集
   - 两者一致才确认, 减少误检
   - 向量改动: ~30 行

5. **伺服控制死区 + 滞后**
   - 检测位置变化 <5px 不更新伺服目标
   - 避免微小振荡被 PID 放大
   - 向量改动: ~10 行

| 优点 | 缺点 |
|------|------|
| 掉帧/误检可根本性解决 | 代码改动 ~300 行 |
| Kalman 天然抗噪声 | 调试复杂度高 |
| 多尺度提升远处精度 | 推理量增加需帧率自适应 |

---

## 推荐路线: A + B 分两阶段

### 第一阶段 (立即, 1-2h): 方案 A

参数调优 + EMA 滤波 + 追踪去重。不改模型, 立即见效, 预计解决 70% 振荡/误检。

改动清单:
- [ ] `vision_pipeline.py`: `_process_landing_v2` 加 EMA 平滑 (`_landing_ema_cx`, `_landing_ema_cy`, `_landing_ema_conf`)
- [ ] `vision_pipeline.py`: `_process_landing_v2` 调用 `_adaptive_params.get_landing_params()` 做高度自适应 conf
- [ ] `vision_pipeline.py`: `_refine_marker_center` 加圆形度验证 + 预期像素半径范围检查
- [ ] `vision_pipeline.py`: 投放区检测结果过 `CylinderTracker` (min_hits=2)
- [ ] `vision_params.yaml`: 收紧 HSV 蓝色 H 范围, 加 S_min 下限

### 第二阶段 (需数据集, 1-2 天): 方案 B

模型升级 + 精修改进。源头上提升检测质量。

前提条件:
- [ ] 确认是否有标注数据集可用于重训练
- [ ] 确认 Jetson 部署时是否使用 TensorRT (抵消分辨率提升的延迟)

---

## 参数速查

| 参数 | 当前值 | 建议值 (方案 A) | 说明 |
|------|--------|-----------------|------|
| `conf_threshold` | 0.35 | 保持 | 已较低 |
| `cylinder_tracking_min_hits` | 3 | 保持 | 已合理 |
| `marker_hsv_h_low` | 95 | 100 | 排除天空蓝 |
| `marker_hsv_h_high` | 135 | 130 | 排除偏紫 |
| `marker_hsv_s_min` | 60 | 80 | 排除低饱和度灰色 |
| `marker_min_circularity` | 无 | 0.6 | 新增形状约束 |
| `landing_ema_alpha` | 无 | 0.4 | 新增 EMA 平滑系数 |
| `marker_tracking_min_hits` | 无 | 2 | 新增投放区追踪 |
