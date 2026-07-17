# SITL 崩溃预防 & 性能优化方案

> 2026-07-17 | 根因: GPU VRAM 耗尽 (PyTorch + Gazebo 竞争 8GB VRAM) → 内核级死锁

---

## 已完成 ✅：路径 A — ONNX 模型导出

**日期**: 2026-07-17  
**操作**: 将两个 YOLO 模型从 `.pt` (PyTorch) 转为 `.onnx` (ONNX Runtime FP16)

| 模型 | .pt 大小 | .onnx 大小 | 预期效果 |
|------|---------|-----------|---------|
| `cylinder_yolo` | 6.3 MB | 5.9 MB | ONNX CUDA provider 比 PyTorch GPU 内存管理更紧凑 |
| `drop_marker_yolo` | 5.5 MB | 5.2 MB | 同上，VRAM 压力降低 |

**配置已更新**: `vision_params.yaml` 中 `model_path` 和 `drop_marker_model_path` 指向 `.onnx`

**导出命令记录**:
```bash
cd src/drone_vision/training
python3 export.py --weights ../drone_vision/models/cylinder_yolo.pt --format onnx --half
python3 export.py --weights ../drone_vision/models/drop_marker_yolo.pt --format onnx --half
```

> ⚠️ `cylinder_yolo.pt` 导出耗时 ~120s (首次需要安装 onnxslim 等依赖)，后续增量导出更快。

---

## 待实施 P0：立即见效的防护（~30 行改动）

### P0-1: 限制 PyTorch 显存 (紧急)

即使已切到 ONNX，Gazebo 渲染仍需 GPU。限制 CUDA 分配器防止 PyTorch 占用所有剩余 VRAM。

**文件**: `scripts/start_full_simulation.sh`，在 `source install/setup.bash` 之后加：

```bash
# 限制 PyTorch CUDA 显存分配，给 Gazebo 留呼吸空间
export PYTORCH_CUDA_ALLOC_CONF="max_split_size_mb:128"
```

### P0-2: 视觉管线推理超时保护

防止 GPU 卡住时整个回调永久阻塞。

**文件**: `src/drone_vision/drone_vision/vision_pipeline.py`，`_image_cb` 方法中 (约 line 457)：

```python
# 原代码:
# if future_main is not None:
#     all_dets = future_main.result()

# 改为:
import concurrent.futures

INFER_TIMEOUT = 0.5  # 500ms 超时 — 正常推理 <20ms

try:
    if future_main is not None:
        all_dets = future_main.result(timeout=INFER_TIMEOUT)
    else:
        all_dets = self._fallback_all(image)
except concurrent.futures.TimeoutError:
    self.get_logger().warn("YOLO 主模型推理超时，跳过本帧", throttle_duration_sec=1.0)
    self._skip_count += 1
    return
except Exception as e:
    self.get_logger().warn(f"YOLO 推理异常: {e}", throttle_duration_sec=1.0)
    self._skip_count += 1
    return

# 投放区标识模型同样加超时:
try:
    if future_marker is not None:
        marker_dets = future_marker.result(timeout=INFER_TIMEOUT)
    else:
        marker_dets = []
except Exception:
    self.get_logger().warn("投放标识模型推理超时/异常", throttle_duration_sec=1.0)
    marker_dets = []
```

### P0-3: Gazebo Headless 模式选项

**文件**: `scripts/start_full_simulation.sh`，新增环境变量控制：

```bash
# 在 Step 2 启动 PX4 前:
# 设置 GZ_HEADLESS=1 启用 headless 模式 (无 GUI 渲染，节省 ~1-2GB VRAM)
GZ_HEADLESS="${GZ_HEADLESS:-0}"
if [ "$GZ_HEADLESS" = "1" ]; then
    export GZ_SIM_RESOURCES=""
    info "Gazebo headless 模式 — 无 GUI 渲染，节省 GPU 显存"
fi
```

使用方式: `GZ_HEADLESS=1 ./scripts/start_full_simulation.sh`

---

## 待实施 P1：架构级防护（~50 行改动）

### P1-1: 相机分辨率和帧率降级开关

**文件**: `src/drone_vision/config/vision_params.yaml`，新增可切换的低负载配置：

```yaml
    # ---- 低负载模式 (GPU 压力大时启用) ----
    low_resource_mode: false             # true=640x480@15FPS, false=1280x960@30FPS
```

**文件**: `src/drone_bringup/scripts/gz_camera_bridge.py`，约 line 35，改为可配置：

```python
# 原: self._timer = self.create_timer(0.1, self._capture)  # 10 FPS
# 改为从参数读取，低负载模式下用 0.067 (15 FPS)
```

### P1-2: 视觉管线处理耗时异常告警

**文件**: `src/drone_vision/drone_vision/vision_pipeline.py`，在 FPS 监控处 (约 line 559) 加：

```python
if dt > 0.1:  # 单帧超过 100ms
    self.get_logger().warn(
        f"慢帧告警: {dt*1000:.0f}ms (正常 <30ms) — "
        f"可能 GPU 资源不足",
        throttle_duration_sec=2.0,
    )
```

### P1-3: DDS 话题队列深度调整

当前所有订阅器 `depth=10`。视觉话题可降低到 `depth=3`（只保留最新）：

**文件**: `src/drone_vision/drone_vision/vision_pipeline.py`，约 lines 102-111：

```python
# 改为:
self._img_sub = self.create_subscription(Image, "/drone/camera/image_raw", self._image_cb, 3)
self._pose_sub = self.create_subscription(PoseStamped, "/drone/pose", self._pose_cb, 3)
```

### P1-4: GPU VRAM 使用量监控日志

**文件**: `src/drone_vision/drone_vision/vision_pipeline.py`，`_lazy_init` 末尾加：

```python
# GPU 显存监控 (仅 CUDA 设备)
try:
    import pynvml
    pynvml.nvmlInit()
    handle = pynvml.nvmlDeviceGetHandleByIndex(0)
    info = pynvml.nvmlDeviceGetMemoryInfo(handle)
    self.get_logger().info(
        f"GPU 显存: {info.used/1024**3:.1f}/{info.total/1024**3:.1f} GB "
        f"(空闲 {info.free/1024**3:.1f} GB)"
    )
except Exception:
    pass
```

---

## 待实施 P2：Jetson 真机 TensorRT（~60 行新增代码）

### P2-1: yolo_engine.py 新增 TensorRT 引擎类

**文件**: `src/drone_vision/drone_vision/models/yolo_engine.py`，在 `UltralyticsYOLOEngine` 类后 (~line 352) 新增：

```python
# ============================================================
# TensorRT 实现 (Jetson 真机)
# ============================================================

class TensorRTYOLOEngine(YOLOEngine):
    """使用 ultralytics 加载 .engine (TensorRT) 模型."""

    def _check_backend(self) -> bool:
        try:
            import tensorrt  # noqa: F401
            import ultralytics  # noqa: F401
            return True
        except ImportError:
            print("[TensorRTYOLOEngine] tensorrt/ultralytics 未安装")
            return False

    def _do_load(self) -> None:
        from ultralytics import YOLO
        self._model = YOLO(str(self.model_path), task='detect')

    def _predict_device(self) -> str:
        return "0"  # TensorRT 固定用 GPU 0

    def _do_infer(self, image: np.ndarray) -> List[Detection]:
        import torch
        results = self._model.predict(
            image,
            imgsz=(self.input_height, self.input_width),
            conf=self.conf_threshold,
            iou=self.nms_iou_threshold,
            device="0",
            verbose=False,
        )

        detections: List[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].detach().cpu().numpy()
                class_name = (
                    self.class_names[cls_id]
                    if cls_id < len(self.class_names)
                    else str(getattr(result, "names", {}).get(cls_id, f"cls_{cls_id}"))
                )
                detections.append(Detection(
                    class_id=cls_id,
                    class_name=class_name,
                    confidence=conf,
                    bbox=(int(round(x1)), int(round(y1)),
                          int(round(x2)), int(round(y2))),
                    cx=float((x1 + x2) / 2),
                    cy=float((y1 + y2) / 2),
                ))

        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections
```

### P2-2: create_engine() 加入 .engine 路由

**文件**: `src/drone_vision/drone_vision/models/yolo_engine.py`，`create_engine()` 函数 (约 line 357)，在最上方加：

```python
def create_engine(model_path, ...):
    suffix = Path(model_path).suffix.lower()

    # ---- TensorRT .engine ----
    if suffix == ".engine":
        engine = TensorRTYOLOEngine(
            model_path=model_path,
            conf_threshold=conf_threshold,
            nms_iou_threshold=nms_iou_threshold,
            input_size=input_size,
            class_names=class_names,
            device="tensorrt",
        )
        if engine.is_available:
            return engine
        raise RuntimeError(
            "无法加载 TensorRT 模型. 请确认 tensorrt 和 ultralytics 已安装"
        )

    if suffix == ".pt":  # ... 现有逻辑不变
```

### P2-3: 在 Jetson 上导出 TensorRT .engine

在 Jetson Orin 上执行（**必须在目标硬件上运行**，因为 engine 绑定特定 GPU 架构）：

```bash
# 把 .pt 文件传到 Jetson 后
python3 -c "
from ultralytics import YOLO
for name in ['cylinder_yolo', 'drop_marker_yolo']:
    m = YOLO(f'{name}.pt')
    m.export(format='engine', half=True, device=0, imgsz=640)
    print(f'✅ {name}.engine')
"
```

或使用项目自带的 export.py：

```bash
python3 export.py --weights cylinder_yolo.pt --format engine --half --device 0
python3 export.py --weights drop_marker_yolo.pt --format engine --half --device 0
```

### P2-4: Jetson 真机配置文件

**文件**: `config_real/vision_params.yaml` (新建或更新现有)：

```yaml
/vision_pipeline:
  ros__parameters:
    # 真机相机内参 (标定后填入)
    camera_width: 1280
    camera_height: 960
    camera_fps: 30.0
    fx: 0.0   # TODO: 标定后填入
    fy: 0.0   # TODO: 标定后填入
    cx: 0.0   # TODO: 标定后填入
    cy: 0.0   # TODO: 标定后填入
    sim_mode: false

    # TensorRT 模型
    model_path: "cylinder_yolo.engine"
    model_width: 640
    model_height: 640
    conf_threshold: 0.35
    nms_iou_threshold: 0.45
    device: "tensorrt"                           # Jetson 真机用 TensorRT
    class_names:
      - "cylinder"

    drop_marker_model_path: "drop_marker_yolo.engine"
    drop_marker_class_names:
      - "drop_zone_marker"
    drop_marker_refine_enable: true

    # 其余参数与 SITL 保持一致 ...
```

---

## 崩溃恢复检查清单

当仿真卡住时，按顺序检查：

```
1. nvidia-smi            → GPU 显存是否接近 8GB?
2. free -h               → 系统内存是否耗尽? swap 是否被触发?
3. htop                  → 哪个进程 CPU 100%?
4. dmesg | tail -30      → 是否有 GPU 驱动错误 / OOM killer?
5. tail -100 log/simulation_*/px4.log  → PX4 最后的状态?
6. tail -100 log/simulation_*/ros2_system.log → ROS2 最后的日志?
```

如果 `dmesg` 出现 `NVRM: GPU has fallen off the bus` 或 `Xid` 错误 → **确认为 GPU VRAM 耗尽导致的驱动崩溃**。

---

## 方案优先级总结

| 优先级 | 方案 | 改动量 | 效果 |
|:------:|------|:------:|------|
| ✅ 完成 | ONNX 模型导出 | 2 文件 | GPU 内存管理改善 |
| 🔴 P0-1 | PyTorch 显存限制 | 1 行 | 防止 CUDA OOM |
| 🔴 P0-2 | 推理超时保护 | ~15 行 | 防止 GPU 卡死传播 |
| 🔴 P0-3 | Gazebo headless 开关 | ~5 行 | 释放 1-2GB VRAM |
| 🟡 P1-1 | 低负载降级模式 | ~10 行 | 灵活应对不同硬件 |
| 🟡 P1-2 | 慢帧告警 | ~5 行 | 及早发现问题 |
| 🟡 P1-3 | DDS 队列降深 | 2 行 | 减少消息积压 |
| 🟡 P1-4 | GPU 显存监控日志 | ~10 行 | 可观测性 |
| 🟢 P2-1 | TensorRT 引擎类 | ~50 行 | Jetson 真机必备 |
| 🟢 P2-2 | .engine 路由 | ~10 行 | 同上 |
| 🟢 P2-3 | Jetson 导出命令 | 命令 | 同上 |

---

## 相关文档

- [视觉 FPS 优化](2026-07-17-vision-fps-optimization.md) — 双模型并行推理方案
- [真机部署方案](2026-07-17-real-hardware-workspace-plan.md) — 4 阶段真机开发计划
- [视觉精度改进](2026-07-15-vision-precision-improvement-proposals.md) — 检测精度优化提案
- [SITL 审计](2026-07-14-sitl-audit-and-real-hardware-gaps.md) — SITL 真机差距分析
