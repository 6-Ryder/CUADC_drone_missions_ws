# 视觉管线帧率优化方案

## 问题

实飞测试中视觉管线帧率仅 42 FPS (24ms/帧)。对低空投放精校阶段（2m→1.25m 热重启复查），控制器每 tick 依赖最新的检测结果，低帧率会加长精校收敛时间。

## 当前管线耗时分布 (1280×960 输入, 640×640 模型)

```
摄像头帧 → ROS2 序列化 (~2ms)
  → imgmsg_to_cv2 解码 (~2ms)
  → YOLO 主模型推理 (~10ms)         ← 瓶颈
  → YOLO 投放标识模型推理 (~10ms)    ← 瓶颈 (与上面串行)
  → OpenCV 精修 + 坐标变换 + 发布 (~3ms)
────────────────────────────────
总计: ~24ms → 42 FPS
```

## 优化方案

### 方案 A: 并行双模型推理 ✅ 已实现 (2026-07-17)

**原理**: 主模型和投放标识模型对同一帧独立推理，用 `ThreadPoolExecutor(max_workers=2)` 并行执行。

**改动**:
- `vision_pipeline.py`: ~15 行，导入 `concurrent.futures`，线程池并行 `infer()`

**效果**: 42 → ~65 FPS (两个 10ms 推理并行 → ~10ms 总推理时间)

**风险**: 零。YOLO 推理瓶颈在 ONNX Runtime C++ 层，无 GIL 争用；结果完全一致。

### 方案 B: 降低模型输入尺寸 640→416 (待评估)

**原理**: YOLOv8 支持动态输入，不需要重新训练。416² = 173K 像素 vs 640² = 410K 像素，计算量减少 58%。

**效果**: 单模型 ~10ms → ~4ms。配合方案 A: ~100 FPS。

**风险**:
- CYL_SEARCH 阶段 (5m 巡航高度): 远处小圆筒 (40-60px@640) 缩到 26-39px，可能掉到 conf=0.35 阈值以下
- APPROACH/DESCEND 阶段 (2-5m): 目标足够大，无影响
- 侦察/降落阶段: 无影响

**待验证**: 在 SITL 中对比 640 vs 416 的 CYL_SEARCH 圆筒检出率。

### 方案 C: 动态分辨率 (待实现)

**原理**: 根据飞行高度/任务阶段动态切换模型输入尺寸:
- 高飞 (>3m): 640×640 (保证远距离检测)
- 低飞 (≤3m): 416×416 或 320×320 (近距离目标足够大)

**改动**: 
- `vision_pipeline.py`: 根据 `_current_altitude` 动态调用 `cv2.resize()` 缩放送入模型的图像
- 或在状态机中通过 ROS2 参数动态设置 `model_width/model_height`

**效果**: 高空搜索不受影响，低空精校帧率翻倍。总体 ~65-100 FPS。

**风险**: 低。只需保证 resize 不引入明显延迟（<1ms）。

### 方案 D: TensorRT 引擎 (待实现)

**原理**: Jetson Orin Nano 硬件加速。用 `trtexec` 或 `ultralytics export` 将 .pt 导出为 .engine，推理速度 2-4x ONNX Runtime。

**改动**:
- `yolo_engine.py`: 新增 `TRTYOLOEngine` 类 (~40 行)
- 离线导出: `yolo export model=cylinder_yolo.pt format=engine device=0`

**效果**: 单模型 640×640 ~10ms → ~3ms。配合方案 A: ~100+ FPS (640) 或 ~200+ FPS (416)。

**前置条件**: 需要导出的 .engine 文件与特定 JetPack/TensorRT 版本绑定，部署环境变更后需重新导出。

## 实施记录

| 日期 | 方案 | 状态 |
|------|------|------|
| 2026-07-17 | A: 并行双模型推理 | ✅ 已实现 |
| - | B: 降低输入尺寸 | ⏳ 待 SITL 验证 CYL_SEARCH 检出率 |
| - | C: 动态分辨率 | 📋 待实现 |
| - | D: TensorRT 引擎 | 📋 待离线导出 .engine 后实现 |
