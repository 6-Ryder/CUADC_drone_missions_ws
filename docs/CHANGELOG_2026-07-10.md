# Changelog — 2026-07-10

## cv_trainer 模型融合

| 模型 | 来源 | 文件 | 类别 |
|------|------|------|------|
| V2 (YOLOv8n) | cv_trainer/best_V2.pt | drone_vision/models/cylinder_yolo.pt | white-cylinder-detection |
| V1 (YOLO11n) | cv_trainer/best_V1.pt | drone_vision/models/drop_marker_yolo.pt | blue-carpet-detection |

## 新增文件
- `drone_vision/scripts/vision_viewer.py` — 实时标注画面查看器
- `drone_vision/scripts/__init__.py`
- `scripts/debug_vision.sh` — 一键视觉调试

## 代码变更

### vision_pipeline.py
- class_names 从 6 类硬编码改为 YAML 参数读取
- bbox 字段随精修链传递, 填入 CylinderDetection 消息
- 新增 `/drone/vision/markers` 话题发布投放区标识检测
- 模型路径支持相对路径自动解析

### camera.py
- 新增 `camera_device` 参数 (默认 /dev/video0)
- `_capture_loop` 改用 grab()×5 + retrieve() 消除 OpenCV 帧缓冲延迟
- 手动构造 Image 消息绕过 cv_bridge KeyError:16 兼容性 bug

### cylinder_detector.py
- CylinderTracker.update() 支持 5 元组 (cx,cy,radius,conf,bbox), 4 元组向后兼容
- CylinderDetectorNode: class_names 默认值 6→1, 模型路径更新, bbox 填入输出

### 消息定义
- `CylinderDetection.msg` — 新增 bbox_x1, bbox_y1, bbox_x2, bbox_y2 (int32)

### 配置文件
- `vision_params.yaml` — class_names: ["cylinder"], model_path 指向 .pt 文件

### Launch 文件
- `vision_only.launch.py` — 移除 model_path 覆盖; start_camera 参数; 修复内参与 640x480 匹配
- `real_flight.launch.py` — 移除 model_path 覆盖
- `sitl.launch.py` — 移除 model_path 覆盖

## 测试结果
- 白色圆筒 (V2): 模型有效, 桌面视角置信度 ~0.25, conf_threshold=0.35 偏高
- 蓝色标识 (V1): 稳定检出, 置信度 0.6-0.99
- 降落 H 标识: 暂无模型

## 已知限制
- landing_H 无模型: 精准降落靠 OpenCV 回退
- .pt 格式: Jetson 实飞前应导出 ONNX/TensorRT
- 延迟修复: camera grab/retrieve + viewer qos=1 消除帧缓冲延迟

## 时域过滤 — 消除背景杂物误检

- `CylinderTracker` 新增 `min_hits` 参数 (默认 3): 新目标需连续命中 N 帧后才确认输出
- 背景杂物闪烁 (单帧误检) 达不到 min_hits, 自动过滤
- 真正目标额外延迟 ~67ms (3帧@30FPS), 对控制回路无影响
- 配置: `vision_params.yaml` → `cylinder_tracking_min_hits`

---

## 2026-07-11 — 仿真视觉调试

### 问题 1: 场地文件圆筒俯视不可见 ✅ 已修复
- **根因**: `drone_field.sdf` 中圆筒 visual 使用开口杯形 STL 网格 (壁厚 5mm, 5m 高处仅 0.5px)
- **修复**: 实体白色 `<cylinder>` 替代 STL mesh + 底部白色圆盘 (8 个圆筒全部修复)
- **结果**: 圆筒检测 0 → 2/3 个, FSM 正常进入 SELECT → APPROACH

### 问题 2: CYL_COORD 姿态投影坐标异常 ⚠️ 已定位
- **根因**: `coordinate_transforms.py` 中 PX4 四元数输出是 NED, 但 World 坐标系是 ENU, 缺少转换
- **影响**: 姿态投影 Y 分量系统性偏大 ~14m, 全部被 `max_reasonable_offset` 拒绝

### 问题 3: APPROACH 逼近跟踪丢失 ⚠️ 已定位
- **根因**: 问题 2 导致圆柱 world 坐标错误 → FSM 飞到错误位置 → 目标丢失 → 放弃
- 详见 `docs/DEBUG_2026-07-11.md`

---

## 2026-07-11 (下午/晚间) — 低空视觉校准重构

### 背景
- 世界坐标粗定位偏差 ~1m (固定, YOLO 系统误差)
- 2m 低空需要像素闭环微调把 ~270px 压到 35px 以内
- Codex 创建的 `drop_visual_aligner.py` 使用硬编码 `x_sign/y_sign` → 实际仿真中方向符号全反

### 九轮迭代

| # | Commit | 内容 | 状态 |
|---|--------|------|------|
| 1 | `20f4677` | 刹车区(80px) + 零交叉轴锁定(策略A+C) | 零交叉从未触发 (方向反) |
| 2 | `d9807f9` | 像素空间梯度下降替代硬编码伺服参数 | 探测被噪声淹没 |
| 3 | `a3597e5` | 端点比较法: 不回锚点, 步长 0.40m | SNR 提升但 cylinder 检测不稳定 |
| 4 | `9159d0d` | 修复 COMMIT_AXIS 轴切换 (引入 _working_axis_x) | — |
| 5 | `6e1d6a4` | 梯度下降超时后原地重试, 不回升 | 有用但非根本修复 |
| 6 | `d35cb30` | 探测周期内锁定目标身份 (cylinder/marker) | 身份切换被拦截 |
| 7 | `639bda0` | COMMIT_AXIS→HOVER 悬停 2.5s | 解决抽搐飞行 |
| 8 | `b55ee41` | 低空 marker 优先替代 cylinder | cylinder 间歇性丢失 → marker 稳定 |
| 9 | `eea9562` | 目标世界坐标锁定 (marker1/2/3) | 阻止 marker 间切换 |

### 当前状态
- **1次成功投放** (端点比较法 X- 方向把 err 从 222 压到 36px)
- **核心问题**: 算法已过度复杂 (6 状态 + 5 时间窗口), 拟简化为两步: 方向发现 → 连续伺服

### 关键发现
1. marker 检测最稳定 (conf=0.95+, 始终可见) — 应作主要伺服目标
2. 目标身份切换是所有失败场景的根源 (3 级: cylinder/marker/marker间)
3. 世界坐标偏差 ~1m 在微调范围内
4. 端点比较法方向判断本身正确 (有一次成功证实)

### 新模块
- `src/drone_mission/drone_mission/drop_visual_aligner.py` — 低空投放视觉校准
- `src/drone_vision/test/test_coordinate_transforms.py` — 坐标变换回归测试
- `docs/DEBUG_2026-07-11.md` — 详细调试日志
