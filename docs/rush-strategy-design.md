# 抢分策略设计文档

> 状态: 方案设计完成，待实现
> 日期: 2026-07-25
> 对比基线: 完美完成任务策略 (`mission_params.yaml`)

---

## 一、策略目标

在确保不犯规的前提下，以**最短时间**完成全任务流程：
1. 起飞 → 飞向投放区
2. 搜索并识别大桶（半径最大者）
3. **第一投**：快速对准大桶中心投放
4. **第二投**：立即投向同一大桶的蓝色圆形标识区域（不能投到白筒）
5. 侦察区快速覆盖
6. 全速返航降落

---

## 二、核心设计原则

| 原则 | 说明 |
|------|------|
| **中转全速** | 区域间飞行提速到 5 m/s，PX4 MPC_XY_VEL_MAX=8 下有安全裕度 |
| **巡航稳定** | 搜索/逼近阶段速度不能太快，保证视觉检测不丢帧 |
| **选大不选小** | 第一投选择半径最大的圆筒 = 大桶，翻转 `min_radius_first` |
| **第二投追 marker** | 投中大桶后，第二弹锁定同一位置的蓝色圆形标识，天然排除白筒 |
| **精校减配** | 减少稳定帧数、刹车时间、重试次数，像素误差阈值适度放宽 |
| **侦察提速** | 侦察区不需要目标检测，扫描速度从 0.8→1.5 m/s，间距从 1.0→1.5m |
| **返航全速** | 返航速度从 3.0→5.0 m/s |

---

## 三、状态机全流程参数对比

### 3.1 起飞 (TAKEOFF)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| ARM 超时 | 15s | 8s | PX4 解锁通常 2-3s |
| OFFBOARD 超时 | 15s | 8s | 模式切换 <1s |
| 爬升稳定帧数 (20Hz) | 40 (2s) | 20 (1s) | 减少稳态等待 |
| 爬升超时 | 30s | 15s | 5m 爬升足够 |

### 3.2 中转飞行 (TRANSIT)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| transit_speed | 2.0 m/s | 5.0 m/s | 32m/5≈7s，留足裕度 |
| waypoint_timeout | 120s | 40s | 航程最长 60m/5=12s |
| waypoint_tolerance | 0.5m | 0.8m | 微放宽，到达判定更宽容 |

### 3.3 巡航搜索 (CYLINDER_SEARCH) — **速度不能太快**

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| search 高度 | 5.0m | 5.0m | 保持大 FOV，不变 |
| sweep_speed | 0.8 m/s | 1.0 m/s | 微提，检测稳定性不降 |
| 稳定检测帧数 | 5 | 3 | 0.15s 确认即可 |
| min_drop_targets_required | 2 | 1 | 一个大桶就够 |
| 扫描启动延迟 | 10s | 5s | 更快进入主动扫描 |
| drop_marker_fallback_delay | 15s | 5s | 快速允许 marker 兜底 |

### 3.4 目标选择 (SELECT_TARGET) — **策略翻转**

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| min_radius_first | true (选小) | **false (选大)** | 大桶 = 最大半径 |
| 第一投目标 | 半径最小圆筒 | **半径最大圆筒** | = 大桶 |
| 第二投目标 | 半径次小圆筒 | **同一大桶上的蓝色 marker** | 必须命中蓝色标识 |

**关键实现逻辑**：

```
Drop 1: SELECT_TARGET
  ├─ CylinderSelector.select() → 按半径降序 → 选最大 cylinder（大桶）
  └─ 记录选中位置到 _selected_positions

Drop 2: SELECT_TARGET（drop_success_count == 1）
  ├─ 获取 _selected_positions[-1]（第一投的大桶位置）
  ├─ 搜索该位置附近的 marker（蓝色圆形标识）
  ├─ 将 marker 设为第二投目标
  └─ 设置 precision_align_target_source = "marker"
```

### 3.5 逼近 (CYLINDER_APPROACH)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| fast_speed | 0.30 m/s | 0.50 m/s | 粗逼近加速 |
| slow_speed | 0.15 m/s | 0.25 m/s | 精逼近加速 |
| memory_speed | 0.25 m/s | 0.40 m/s | 丢帧时向记忆坐标靠近加速 |
| 稳定帧数 (pixel) | 10 | 5 | 减少对心确认 |
| memory_arrival_count | 20 | 10 | 减少静态到达等待 |
| 超时 | 30s | 15s | |

### 3.6 投放下降+精校 (DESCEND + PRECISION_ALIGN)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| drop_descend_height | 2.0m | 2.0m | 不变 |
| 下降速度 | 0.5 m/s | 1.0 m/s | 更快下降 |
| precision_align_timeout | 25s | 15s | 总精校时长压缩 |
| precision_align_stable_frames | 5 | 3 | 减少稳定确认 |
| precision_align_final_brake_s | 0.5s | 0.25s | 减少刹车复查 |
| precision_align_max_retries | 2 | 1 | 减少原地重试 |
| precision_align_raw_pixel_tolerance | 9.0px | 12.0px | 适度放宽 |
| precision_align_smooth_pixel_tolerance | 7.0px | 10.0px | 适度放宽 |
| precision_align_final_drop_height | 1.25m | 1.5m | 稍高投放减少下降时间 |
| precision_align_target_source | cylinder | **第二投动态切换为 marker** | |
| drop_confirmation_timeout | 5.0s | 3.0s | 匹配舵机 1s 固定延时 |

### 3.7 回升 (ASCEND)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| ascend_speed | 1.0 m/s | 2.0 m/s | 快速回升 |
| 超时 | 15s | 8s | |

### 3.8 侦察区扫描 (RECON_PATROL)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| sweep_speed | 0.8 m/s | 1.5 m/s | 侦察无需目标检测 |
| area2_altitude | 2.5m | 3.0m | 稍高，减少扫描条带 |
| grid_spacing | 1.0m | 1.5m | 增大间距 → 更少条带 |
| recon_require_full_path | true | **false** | 覆盖率达标即结束 |
| sweep_margin_north | 1.0m | 0.5m | 减小内缩 |
| sweep_margin_east | 0.5m | 0.3m | 减小内缩 |

### 3.9 返航 (RETURN)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| return_speed | 3.0 m/s | 5.0 m/s | 全速返航 |
| 超时 | 60s | 30s | |

### 3.10 精准降落 (PRECISION_LAND)

| 参数 | 原值 | 抢分值 | 理由 |
|------|------|--------|------|
| H 标识搜索超时 | 15s | 8s | |
| 盲降高度 | 0.5m | 0.8m | 稍高进入盲降 |
| landing_approach_speed | 0.5 m/s | 0.8 m/s | 加快降落 |

---

## 四、CylinderSelector 策略翻转

当前代码：

```python
# mission_state_machine.py
self._cylinder_selector = CylinderSelector(min_radius_first=True)
```

抢分策略改为：

```python
if self._rush_mode:
    self._cylinder_selector = CylinderSelector(min_radius_first=False)  # 选大桶
else:
    self._cylinder_selector = CylinderSelector(min_radius_first=True)
```

`CylinderSelector.select()` 中已有的排序逻辑天然支持：
- `min_radius_first=True` → `available.sort(key=lambda x: x[3])` (升序，小优先)
- `min_radius_first=False` → `available.sort(key=lambda x: -x[3])` (降序，大优先)

---

## 五、第二投 Marker 目标切换

### 5.1 SELECT_TARGET 阶段

在 `_handle_select_target()` 中添加抢分二投逻辑：

```python
def _handle_select_target(self):
    # 抢分模式第二投: 追踪蓝色标识
    if self._rush_mode and self._drop_success_count == 1:
        markers = self.get_known_markers()
        last_cyl_pos = self._cylinder_selector._selected_positions[-1]
        # 找离大桶最近的 marker
        nearest = self._find_nearest_marker_to(markers, last_cyl_pos)
        if nearest:
            self._current_drop_target_is_marker = True
            self._current_drop_marker = nearest
            self._transition_to(MissionState.CYLINDER_APPROACH)
            return
        # marker 不可用，回退到选第二大圆筒
        # (fall through to normal selection)

    # 正常选择 / 抢分第一投
    targets = self.get_known_drop_targets()
    ...
```

### 5.2 DESCEND 精校阶段

在 `DropState._enter_descend()` 中：

```python
def _enter_descend(self):
    ...
    # 如果当前目标是 marker，强制 precision_align 追踪 marker
    if getattr(self._fsm, '_current_drop_target_is_marker', False):
        # 临时覆盖参数
        self._fsm._precision_align_target_source_override = "marker"
    ...
```

`PrecisionAlignController._choose_target()` 已有 `precision_align_target_source` 参数控制目标偏好：
- `"cylinder"` → 只从 cylinders 中选
- `"marker"` → 只从 markers 中选
- 其他 → cylinders + markers 混合

### 5.3 白筒排除机制

白筒在视觉检测中大概率没有蓝色 marker 关联。额外的安全措施：
- 如果 marker 候选的世界坐标与已投大桶位置距离 > 1.5m，拒绝
- `precision_align_target_world_gate` 参数（1.5m）自然排除远离大桶的 marker

---

## 六、时间估算

| 阶段 | 原策略 | 抢分策略 | 节省 |
|------|--------|----------|------|
| INIT 等待 | ~3s | ~3s | 0s |
| 起飞 | ~8s | ~5s | 3s |
| 中转→投放区 (32.5m) | ~16s | ~7s | 9s |
| 搜索+累计目标 | ~15s | ~8s | 7s |
| 第一投: 逼近+下降+精校+投+升 | ~35s | ~22s | 13s |
| 第二投: 逼近+下降+精校+投+升 | ~35s | ~20s | 15s |
| 中转→侦察区 (25m) | ~29s | ~12s | 17s |
| 侦察扫描 | ~40s | ~25s | 15s |
| 返航 (57.5m) | ~20s | ~12s | 8s |
| 降落 | ~12s | ~8s | 4s |
| **总计** | **~213s** | **~122s** | **~91s (-43%)** |

---

## 七、风险矩阵

| 风险 | 概率 | 影响 | 对策 |
|------|------|------|------|
| 5 m/s 中转急停超调 | 中 | 航点误判 | 提前减速 + PX4 jerk 天然限制 |
| 搜索阶段检测丢帧 (速度1.0) | 低 | 目标漏检 | sweep_speed 仅微增 0.2 |
| 大桶半径远处误判 | 中 | 选了白筒 | 降高到 2m 重新确认半径 |
| marker 检测不稳定 | 中 | 第二投无目标 | 保留 cylinder 兜底 |
| 精校稳定帧减少致误投 | 低 | 投偏 | pixel error 阈值仍在厘米级 |
| 侦察间距增大漏覆盖 | 低 | 侦察分丢失 | `recon_require_full_path=false` 时覆盖率达标才结束 |

---

## 八、实现清单

### 8.1 参数文件
- [ ] 创建 `config/rush_mission_params.yaml`

### 8.2 状态机修改 (`mission_state_machine.py`)
- [ ] 添加 `rush_mode` 参数声明
- [ ] 修改 `CylinderSelector` 初始化：rush 模式下 `min_radius_first=False`
- [ ] 修改 `_handle_select_target()`：第二投切换 marker 目标
- [ ] 添加 `_find_nearest_marker_to()` 辅助方法
- [ ] 透传 `_current_drop_target_is_marker` 标志给 DropState

### 8.3 投放状态修改 (`state_drop.py`)
- [ ] 读 `_current_drop_target_is_marker` 标志
- [ ] 动态设置 `precision_align_target_source`

### 8.4 CylinderSelector
- [ ] 无需修改，已有 `min_radius_first` 参数支持

### 8.5 启动文件
- [ ] 更新 launch 文件支持 `rush` 参数加载

---

## 九、使用方式

```bash
# 完美完成任务策略（默认）
ros2 launch drone_bringup sitl.launch.py

# 抢分策略
ros2 launch drone_bringup sitl.launch.py \
    mission_params:=rush_mission_params.yaml
```
