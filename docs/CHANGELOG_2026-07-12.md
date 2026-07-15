# Changelog — 2026-07-12

## 概述

诊断低空投放视觉微调算法 (`DropVisualAligner`) 效果差的根因。对 3 次仿真日志逐帧追踪 pixel_error 的变化轨迹，识别出 5 层逐级放大的问题。定位根因在 marker 管线未经过 OpenCV 精修（YOLO bbox 中心每帧抖动 50-100px 淹没了梯度信号），并实施最小改动修复。

同时对标 CUADC-UAV-Project 的精校策略（`docs/analysis/02-visual-servo-strategy.md` + `04-fusion-plan.md`），形成后续优化路线图。

---

## 一、问题诊断：逐帧日志分析

### 1.1 数据来源

3 次仿真日志，均位于 `log/simulation_20260711_20*/ros2_system.log`：

| 运行 | 时间 | 相关日志行数 | 结果 |
|------|------|-------------|------|
| Run 1 | 20:14:16 | 37 | 超时放弃 |
| Run 2 | 20:23:52 | 96 | 多轮探测，超时放弃 |
| Run 3 | 20:30:55 | ~80 + 截断 | 两轮 DESCEND + retry 后截断 |

### 1.2 Run 3 第一轮 DESCEND 完整轨迹

这是最完整的案例。时间戳 `1783773130` 开始，持续约 18 秒。

#### 阶段 A：降高 + 初始锁目标

```
t=0.2s  DESCEND: 降高至 2.0m
t=3.2s  DESCEND_ALIGN: 锁定目标 world=(41.77,5.57) src=marker
t=3.2s  DESCEND: 投放许可失败 source=marker pixel_error=122.2px stable=0
t=3.2s  DESCEND_PROBE: X+ baseline_err=122px → NED(1.61,32.58) step=0.260m scale=0.65
```

初始 pixel_error = 122px，步长 0.26m（自适应档位 100-200px→0.65×0.40=0.26m）。

#### 阶段 B：X 轴探测 (端点比较法)

```
t=4.3s  X+ 采样: err=154px  delta=-32px  (恶化 32px)
        跳到 X-: NED(1.61,32.06)
t=5.4s  X- 采样: err=104px  delta=+18px  (改善 18px)
        三方比较: baseline=122  +=154  -=104  → 最佳=-1 err=104px
        提交到 X- 位置: NED(1.61,32.06)
```

X 轴探测结论：向 -X (西) 移动 0.26m。选为最优，提交。

#### 阶段 C：提交后误差反弹（关键异常）

```
t=6.4s  提交到位: pixel_error=183px  ← 同一位置，从 104px 变成 183px！
t=7.4s  悬停中:   pixel_error=235px  ← 继续恶化
t=8.4s  悬停中:   pixel_error=231px  ← 稳定在 230+ 区间
```

**这是核心问题**：探测时在 X- 端点采样得到 err=104px，提交到同一位置后 err 立刻变成 183px（+76%），随后攀升到 235px（+126%）。端点比较法选出的"最优方向"并没有真正改善 pixel_error。

#### 阶段 D：Y 轴探测

```
t=8.9s  Y+ baseline_err=232px → NED(1.97,32.01) step=0.400m scale=1.00
t=9.5s  Y+ 采样: err=233px  delta=-1px   (几乎不变)
        跳到 Y-: NED(1.17,32.01)
t=10.5s Y- 采样: err=259px  delta=-27px  (恶化)
        三方比较: baseline=232  +=233  -=259  → 最佳=+0 err=232px (基线最优)
        探测无改善, 缩半步长→0.200m
```

Y 轴 ±0.4m 探测均无改善。步长减半到 0.20m。

#### 阶段 E：第二轮 X 轴探测

```
t=11.6s 提交到位: err=246px (基线位置, err 仍在恶化)
t=12.6s 悬停:     err=248px
t=13.7s 悬停:     err=231px
t=14.6s X+ baseline_err=229px → NED(1.47,32.43) step=0.400m scale=1.00
t=15.7s X+ 采样: err=250px  delta=-21px  (恶化)
        跳到 X-: NED(1.47,31.63)
t=16.7s X- 采样: err=105px  delta=+123px (大幅改善！)
        三方比较: baseline=229  +=250  -=105  → 最佳=-1 err=105px
        提交到 X- 位置
t=17.8s 提交到位: err=210px  ← 又反弹！105→210 (+100%)
t=17.8s 复查超时且投放许可失败, 放弃本次投放
```

第二轮 X- 探测又看到了 err=105px 的好读数，但提交后立刻反弹到 210px。15 秒复查超时触发，放弃。

### 1.3 Run 3 第二轮 DESCEND 轨迹

```
t=23.1s 锁定目标 world=(42.10,5.56) src=marker
t=23.1s pixel_error=83.6px stable=0  ← 初始误差比第一轮好
t=23.2s X+ baseline_err=84px → step=0.160m scale=0.40

t=24.2s X+ 采样: err=109px  delta=-26px
        X- 采样: err=76px   delta=+8px
        最佳=-1 err=76px, 提交

t=26.2s 提交到位: err=117px  ← 76→117 (+54%)
t=27.1s 悬停:     err=166px  ← 继续恶化
t=28.4s 悬停:     err=158px

t=28.9s Y+ baseline_err=157px → step=0.260m
t=29.9s Y+ 采样: err=160px  delta=-3px
t=30.9s Y- 采样: err=193px  delta=-36px
        基线最优, 缩半步长→0.130m

t=31.4s 提交到位: err=187px
t=32.5s 悬停:     err=160px
t=33.5s 悬停:     err=154px
t=34.5s 悬停:     err=157px

t=34.5s X+ baseline_err=157px → step=0.260m
t=35.5s X+ 采样: err=172px  delta=-15px
t=36.5s X- 采样: err=76px   delta=+81px  ← 又看到 76px
        最佳=-1 err=76px, 提交

t=37.5s 提交到位: err=134px  ← 76→134 (+76%)
t=37.8s 超时但 best_err=51px, 原地重置梯度下降 (retry=1)
       ↓ 重置 DropVisualAligner, 重新锁定目标
t=38.1s 锁定目标 world=(41.12,5.59) src=marker
t=38.2s X+ baseline_err=230px → step=0.400m
t=38.6s X+ 采样: err=253px  delta=-24px
t=39.3s X- 采样: err=132px  delta=+97px
        最佳=-1 err=132px, 提交

t=40.6s 提交到位: err=95px
t=41.7s 悬停:     err=inf   ← 目标完全丢失
t=42.7s 悬停:     err=inf   ← 连续 6+ 帧 inf
t=43.7s 悬停:     err=inf
t=44.7s 悬停:     err=inf
t=45.7s 悬停:     err=inf
t=46.8s 悬停:     err=inf
       ...日志截断
```

### 1.4 Run 1 的身份振荡问题

Run 1 的独特模式——探测周期内 cylinder↔marker 反复切换：

```
t=2s   X+ baseline_err=263px src=marker
t=3s   X+ 采样时: src=cylinder (目标切换!) → 回基线重试
t=3s   X+ baseline_err=366px src=cylinder
t=4s   X+ 采样时: src=marker (又切换!) → 回基线重试
t=4s   X+ baseline_err=300px src=marker
t=5s   X+ 采样时: src=cylinder (再切换!) → 回基线重试
t=5s   X+ baseline_err=341px src=cylinder
t=6s   终于稳定: +采样 err=339px delta=+2px
t=6s   X- 采样时: err=inf → src=marker → 回基线重试
       6 帧完成 0 次有效探测
```

6 帧内目标在 marker 和 cylinder 之间切换了 4 次，每次触发 `_target_identity()` 校验失败 → 回基线重试，浪费了整个探测周期。

---

## 二、根因分析：5 层逐级放大的问题

### 问题 1（底层根因）：marker 的 YOLO bbox 中心未经 OpenCV 精修

追踪 `vision_pipeline.py` 的发布路径：

**Cylinder 发布路径** (有精修)：
```python
# _process_cylinders_v2() → CylinderRefiner.refine():
ROI → GaussianBlur → Canny → findContours → fitEllipse
→ 椭圆中心 (亚像素精度)
→ get_active() → 多帧追踪过滤闪烁
→ center_u, center_v = 精修后的椭圆中心
```

**Marker 发布路径** (无精修，本次修复前)：
```python
# _image_cb() 中 marker 发布:
md = drop_marker_engine.infer(image)
det.center_u = float(md.cx)   # ← YOLO bbox 中心，无任何精修！
det.center_v = float(md.cy)   # ← 每帧可跳动 50-100px
```

而低空 2m 时 `_current_targets()` 优先使用 marker：
```python
def _current_targets(self):
    markers = self._fsm.get_markers()   # 优先
    if markers:
        return markers, "marker"
    cylinders = self._fsm.get_cylinders()  # 兜底，有精修但检测不稳定
```

**结论**：低空伺服用的是最不稳定的数据源。

### 问题 2：单采样噪声淹没了真实梯度信号

端点比较法每端只取一次 pixel_error。但从日志可以看到，**同一位置连续两帧的 pixel_error 差可达 50-100px**：

```
X- 探测采样:   76px  (单次采样)
提交到同位置:  117px (+54%)
2 秒后同位置:  166px (+118%)
```

比较阈值：在 2m 高度，0.26m 水平位移对应的像素变化约 70px（计算：0.26m × 539.9px / 2m ≈ 70px）。

噪声幅度 (50-100px) ≈ 或 大于 真实信号幅度 (70px)，信噪比 < 1。端点比较法在这种情况下**无法区分"真梯度"和"噪声波动"**。

### 问题 3：提交位置 ≠ 探测位置（飞行控制误差）

端点比较法假设 `goto_ned(baseline_n + dn, baseline_e + de, -drop_h)` 能让无人机精确到达目标位置。但实际飞行中：

- 探测移动速度 0.30m/s，1.0s settle 时间可能不够无人机完全稳定
- PX4 位置控制有稳态误差（通常 5-15cm）
- 两次 `goto_ned` 到"同一位置"，实际到达的可能差 10-20cm

10cm 的位置差在 2m 高度 ≈ 27px 的像素误差差 —— 足以改变梯度方向判断。

### 问题 4：pixel_error 在不动时持续增长

多次观察到一个规律：提交到某个位置后悬停不动，pixel_error 单向爬升而非稳定：

```
Commit: 183px → +1s: 235px → +2s: 231px
Commit: 117px → +1s: 166px → +2s: 158px
```

可能原因：
- 多个 marker 在场（3 个），`_choose_target()` 的评分函数在相邻帧可能选中不同 marker
- 无人机姿态微小振荡（2m 低空地效/风扰）改变了下视角度
- YOLO bbox 存在系统性漂移

无论原因，这意味着算法假设的"悬停 = 稳定读数"前提不成立。HOVER 悬停观察 2.5s 期间错误可能越积越大。

### 问题 5：目标丢失后无法恢复

```
err=95px → inf → inf → inf → inf → inf → inf  (连续 6+ 秒)
```

锁定目标在 drone 移动 0.4m 后可能完全飞出下视相机视野。锁定机制 (`_locked_target_xy`) 在容差 2m 内只接受同一 marker，但 marker 已经不在画面中。`_choose_target()` 返回 None → pixel_error=inf → `is_centered` 清零 → 算法陷入"既不能探测也不能放弃"的死锁。

---

## 三、已实施修复

### 投放区蓝色标识 OpenCV HSV 精修

**文件**: `src/drone_vision/drone_vision/vision_pipeline.py`

新增 `_refine_marker_center()` 静态方法：

```
YOLO bbox
  → ROI 裁剪 (+3px margin)
  → BGR→HSV
  → 蓝色分割: H ∈ [95, 135], S > 60, V > 60
  → 形态学开运算去噪 → 闭运算填孔 (5×5 椭圆核)
  → findContours(RETR_EXTERNAL)
  → 取最大蓝色轮廓 (area ≥ 80px²)
  → cv2.moments() → 亚像素质心
  → 映射回原图坐标
```

蓝色标识是地面上的标准蓝色圆形，HSV 分割 + 轮廓矩质心的精度远超 YOLO bbox 中心。精修失败则自动回退 YOLO 中心，不影响现有逻辑。

**关键设计**：
- 精修后的 `(cx, cy)` 同时用于 `center_u/center_v`（像素坐标）和 `pixel_to_world()`（世界坐标），保证一致性
- `@staticmethod` 无状态，每帧独立运行
- 参数 `drop_marker_refine_enable` 可在 YAML 中关闭回退

**预期效果**（针对问题 1）：marker pixel_error 帧间波动从 50-100px 降至 ~10px 以内。这是消除噪声主源的最小改动——不改变 DropVisualAligner 的任何逻辑，只提升输入数据质量。

### 参数与文档更新

| 文件 | 改动内容 |
|------|---------|
| `vision_pipeline.py` | +76 行：`_refine_marker_center()` + 参数 `drop_marker_refine_enable` + 日志 `refined=N/M` |
| `vision_params.yaml` | +1 行：`drop_marker_refine_enable: true` |
| `ARCHITECTURE.md` | +23 行：6.2.3b 小节 "投放区蓝色标识 OpenCV 精修" + 设计决策 |
| `README.md` | +4 行：更新 vision_pipeline/drop_visual_aligner 行数和说明 |

### 提交

```
44d3c1c feat: 投放区蓝色标识 OpenCV HSV 精修，替代 YOLO bbox 中心
```

---

## 四、对标 CUADC-UAV-Project 精校策略

### 4.1 信息来源

CUADC 项目位于 `/home/bridge/CUADC-UAV-Project/`，核心文档：

- `docs/analysis/02-visual-servo-strategy.md` — 完整的视觉伺服精校策略（6 步流程）
- `docs/analysis/04-fusion-plan.md` — CUADC 精校 × DTW 架构融合方案
- `docs/analysis/01-code-logic-review.md` — CUADC 代码审查报告

### 4.2 CUADC 的 8 步精校管线

```
Step 1: YOLO bbox → ROI 裁剪
Step 2: OpenCV 几何中心提取
        - 蓝垫: cv2.moments(contour) → 轮廓矩质心
        - 圆筒: cv2.HoughCircles + cv2.fitEllipse → 取最优
        - 只有 OpenCV 成功提取的帧才算 "precision_valid"
Step 3: 归一化像素误差
        dx_norm = (center_x - img_cx) / img_width   → 范围 [-0.5, +0.5]
        dy_norm = (center_y - img_cy) / img_height
        (与图像分辨率、相机内参、飞行高度全部解耦)
Step 4: EMA 平滑
        smooth_dx = α × raw_dx + (1-α) × smooth_dx
        2m 粗校 α=0.24, 1m 精校 α=0.16 (越低空越"粘")
Step 5: 纯 P 控制 (无积分、无微分)
        right_speed = sign × gain × smooth_dx_norm
        forward_speed = sign × gain × smooth_dy_norm
Step 6: 机体坐标系速度 (VelocityBodyYawspeed)
        "画面右边 = 向右飞", 不依赖偏航角
Step 7: 三级递减参数
        3.5m 蓝垫: gain=1.00, speed=0.16, alpha=0.28
        2.0m 粗校: gain=0.65, speed=0.075, alpha=0.24
        1.0m 精校: gain=0.50, speed=0.055, alpha=0.16
Step 8: 步进-悬停-检测循环
        飞 0.12s → 停 → 重新检测 → 重新计算 → 再飞
        每步 1-2cm, 任何一步的误差不影响下一步
```

### 4.3 本项目与 CUADC 的逐项对比

| 环节 | 本项目 (DTW) | CUADC | 差距分析 |
|------|-------------|-------|---------|
| **OpenCV 精修** | cylinder 有椭圆拟合, marker **刚加上** HSV 分割 | cylinder: HoughCircles+fitEllipse, marker: moments | 本次修复后基本对齐 |
| **误差表示** | `du, dv` 像素误差 | `dx_norm, dy_norm` 归一化误差 [-0.5,+0.5] | CUADC 与高度/分辨率/内参全部解耦, DTW 依赖 `alt/fx` |
| **滤波** | 无 (端点比较法的单采样) | EMA α=0.16~0.28 | DTW 无滤波是噪声问题的根源 |
| **控制方式** | `goto_ned` NED 位置指令 | `set_velocity_body` 机体速度 | DTW 依赖 NED→World 坐标链和偏航角, CUADC 零依赖 |
| **控制律** | 端点比较法 (离线探测-比较-提交) | 纯 P 连续反馈 (在线逐帧) | 根本性架构差异 |
| **精校级数** | 1 级 (2m) | 3 级 (3.5m→2m→1m) | CUADC 每级参数递减, DTW 所有压力放在 2m |
| **步进模式** | goto_ned 连续位置伺服 | 飞 0.10-0.16s → 悬停 → 重新检测 | CUADC 飞停分离, 每次检测时飞机稳定; DTW 飞检同时 |
| **收敛判定** | `is_centered()`: pixel_error<35px 稳定6帧 | `aligned`: `abs(smooth_dx_norm)≤threshold` 稳定4~6帧 | 机制相同, 但 CUADC 用平滑后的归一化误差 |
| **目标丢失恢复** | `_locked_target_xy` 锁定, 不切换 | 6帧缓冲→清旧目标→按画面中心就近重捕获 | CUADC 更完善: 不会死锁 |

### 4.4 CUADC 不会出现 DTW 问题的原因

#### 为什么 CUADC 没有"单采样噪声淹没梯度"问题？

CUADC 用 EMA 滤波而非端点比较。EMA 天然具备低通特性：`α=0.24` 等效 ~4 帧的指数窗口，将单帧 50-100px 的跳动平滑到 10-15px。不需要"探测-比较-提交"，每帧都在向目标微调。

#### 为什么 CUADC 没有"提交后误差反弹"问题？

CUADC 用**机体速度**而非 NED 位置。`set_velocity_body(forward=0.01m/s, duration=0.12s)` → 移动 ~1mm → 停止 → 重新检测。每一步极小（1-2mm），即使方向略有偏差，下一帧立刻纠正。没有"提交一个大步然后发现走错了"的概念。

#### 为什么 CUADC 没有"归一化误差 vs 像素误差"的增益漂移？

CUADC 的归一化误差异常关键。同一物理偏差（比如无人机偏离目标中心 10cm）：
- 在 5m 高度：像素偏差 ≈ 10cm × 539.9/5m ≈ 11px，归一化 = 11/640 ≈ 0.017
- 在 2m 高度：像素偏差 ≈ 10cm × 539.9/2m ≈ 27px，归一化 = 27/640 ≈ 0.042

DTW 的 `gain * du * alt / fx` 在 alt 从 5→2m 时等效增益翻 2.5 倍。CUADC 的 `gain * dx_norm` 跟高度完全无关，控制行为在任何高度一致。

### 4.5 本项目已有的优势（与 CUADC 一致的部分）

| 能力 | 本项目 | CUADC |
|------|--------|-------|
| OpenCV 圆筒几何精修 | `CylinderRefiner.refine()`: Canny→fitEllipse→白度评分 | `_find_cylinder_geometry_in_roi()`: HoughCircles+fitEllipse 双选 |
| OpenCV 标识精修 | **本次新增** `_refine_marker_center()`: HSV 分割→轮廓矩质心 | `_extract_blue_outer_contour()`: cv2.moments(contour) |
| 多帧追踪 | `CylinderTracker`: IoU + min_hits(3) + max_age(10) | `LockedCylinderTracker`: 丢失缓冲6帧→32帧重捕获 |
| 连续稳定判定 | `is_centered()`: pixel_error<35px 稳定6帧 | `stable_count` 连续4-6帧, 任一帧不满足清零 |
| 双模型分离推理 | `_engine`(V2圆筒) + `_drop_marker_engine`(V1标识) | 单模型多类, 无独立标识模型 |
| 姿态补偿坐标变换 | `CoordinateTransformer.pixel_to_world()` 6-DoF 射线求交 | 简化投影, 无姿态补偿 |

---

## 五、`04-fusion-plan.md` 的融合方案评估

CUADC 的 `04-fusion-plan.md` 给出了将 CUADC 精校控制器移植到 DTW 架构的完整方案。核心原则：

> **不跑两套系统，用 CUADC 的控制器替换 DTW 低空阶段的控制器，所有其他基础设施沿用 DTW。**

### 5.1 架构融合图

```
Vision (单一来源 — DTW VisionPipeline)
  ┌─────────────────────────────────────┐
  │ DTW VisionPipeline                   │
  │  - YOLO + CylinderRefiner + Tracker  │
  │  - MarkerRefiner (本次新增)           │
  │  - 6-DoF CoordinateTransformer       │
  │  → CylinderArray / MarkerArray       │
  └──────────────┬──────────────────────┘
                 │
Mission (单一状态机 — DTW MissionFSM)
  ┌──────────────▼──────────────────────┐
  │ DTW MissionFSM                       │
  │  States: ... → CYLINDER_APPROACH →   │
  │    PRECISION_ALIGN_3_5M →            │
  │    PRECISION_ALIGN_2M →              │
  │    PRECISION_ALIGN_1M →              │
  │    EXECUTE_DROP → ...                │
  └──────────────┬──────────────────────┘
                 │
Control (新精校控制器 — 替代 DropVisualAligner)
  ┌──────────────▼──────────────────────┐
  │ PrecisionAlignController (新写)       │
  │  - 输入: CylinderDetection (Vision)  │
  │  - 控制: P-only + EMA + Body Vel     │
  │  - 模式: step-hover-detect           │
  │  - 参数: 三级递减 (3.5m/2m/1m)       │
  └──────────────┬──────────────────────┘
                 │
PX4 Bridge (扩展)
  ┌──────────────▼──────────────────────┐
  │ Body Velocity 支持                    │
  │  - 位置模式: /drone/cmd_setpoint_ned │
  │  - 速度模式: /drone/cmd_velocity_body│
  └─────────────────────────────────────┘
```

### 5.2 融合方案要新增/修改的文件

| 文件 | 动作 | 内容 |
|------|------|------|
| `drone_mission/precision_align_controller.py` | 新建 | 移植 CUADC 精校控制器 (P+EMA+Body Vel+step-hover) |
| `drone_mission/locked_tracker.py` | 新建 | 移植 LockedCylinderTracker (丢失恢复+跳变抑制) |
| `drone_mission/states/state_precision_align.py` | 新建 | 3 个精校子状态 (3.5m/2m/1m) |
| `drone_mission/mission_state_machine.py` | 修改 | 加入 PRECISION_ALIGN_* 状态和转移 |
| `px4_control/flight_controller.py` | 修改 | 新增 `set_velocity_body()` |
| `px4_control/px4_bridge.py` | 修改 | 新增 body velocity 话题发布 |
| `drone_mission/config/mission_params.yaml` | 修改 | 加入精校参数组 |
| `drone_mission/drop_visual_aligner.py` | 废弃 | 被 precision_align_controller 取代 |

### 5.3 融合方案的关键设计参数

```python
PROFILES = {
    3.5: {"threshold": (0.040, 0.055), "stable": 3,
          "max_speed": 0.16,  "step_s": 0.16, "gain": 1.00, "alpha": 0.28},
    2.0: {"threshold": (0.034, 0.045), "stable": 4,
          "max_speed": 0.075, "step_s": 0.12, "gain": 0.65, "alpha": 0.24},
    1.0: {"threshold": (0.014, 0.018), "stable": 6,
          "max_speed": 0.055, "step_s": 0.10, "gain": 0.50, "alpha": 0.16},
}
```

### 5.4 不改的部分（保留 DTW 优势）

| 组件 | 保留原因 |
|------|---------|
| VisionPipeline | 多目标追踪+姿态补偿+高度自适应, 比 CUADC 单帧检测完整 |
| MissionFSM | 状态机架构比 CUADC 单体脚本更适合扩展维护 |
| PathPlanner | 弓字形全覆盖路径、侦察区巡视 CUADC 没有 |
| EmergencyHandler | 低电量/视觉丢失处理比 CUADC SafetyManager 完整 |
| TaskLogger | 结构化日志 CUADC 没有 |
| CoverageMonitor | 侦察覆盖率 CUADC 没有 |
| 舵机投放管理 | CUADC 没有投放逻辑 |

---

## 六、后续优化路线图

按优先级排序：

| 优先级 | 任务 | 预期效果 | 改动量 |
|--------|------|---------|--------|
| **P0** | 验证本次 marker 精修对 pixel_error 稳定性的改善 | 帧间波动 50-100px → ~10px | 已完成, 待仿真验证 |
| **P1** | `DropVisualAligner` 最小改动: 端点采样取多帧中值 | 部分缓解问题 2, 不改架构 | ~20 行 |
| **P2** | `DropVisualAligner` 加入 EMA 滤波 + 缩小步长 | 显著改善, 不改控制方式 | ~50 行 |
| **P3** | 实施 CUADC 融合方案: 新建 PrecisionAlignController | 根治, 对齐 CUADC 已验证精度(~2cm) | ~5 新文件 + 4 文件修改 |

---

## 七、提交记录

```
44d3c1c feat: 投放区蓝色标识 OpenCV HSV 精修，替代 YOLO bbox 中心
36fc037 docs: 2026-07-12 工作日志 — 低空微调诊断 + marker 精修 (初版)
```

---

## 八、修改文件清单

```
src/drone_vision/drone_vision/vision_pipeline.py       — +76 行: _refine_marker_center() + 参数 + 日志
src/drone_vision/config/vision_params.yaml              — +1 行: drop_marker_refine_enable
docs/ARCHITECTURE.md                                    — +23 行: 6.2.3b + 设计决策
docs/README.md                                          — +4 行: 行数更新 + marker 精修说明
docs/CHANGELOG_2026-07-12.md                            — 本文件
```
