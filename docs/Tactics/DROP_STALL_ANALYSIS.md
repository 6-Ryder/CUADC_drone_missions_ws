# 投放精校停滞问题 -- 飞行日志分析与修改策略

> 分析依据: `log/real_mission_20260806_100114` 真机飞行日志
> 机型: Nvidia Jetson (aarch64) + PX4 + V4L2 下视相机 (640x480)
> 日期: 2026-08-06

---

## 一、任务时间线

| 时间 | 阶段 | 耗时 | 结果 |
|---|---|---|---|
| T+0s | INIT | 166s | 等遥控器解锁 |
| T+166s | ARM, TAKEOFF | ~6s | 起飞至 5m |
| T+172s | TRANSIT_TO_DROP_ZONE | ~18s | 北飞 32.5m 到达投放区 |
| T+190s | CYLINDER_SEARCH | ~1s | 发现 2 个圆筒 |
| T+191s | SELECT_TARGET | 瞬时 | 选中 ID=2 |
| T+191s | CYLINDER_APPROACH | ~13s | pixel_error 250 -> 31px |
| T+204s | DESCEND_TO_DROP | ~95s | 降高至 2.8m, 进入精校 |
| T+204~264s | PRECISION ALIGN (第 1 轮) | ~60s | 6 次对心成功, 全部卡在刹车 |
| T+264~338s | PRECISION ALIGN (第 2 轮) | ~74s | 2 次对心成功, 全部卡在刹车 |
| T+338s | HOLD_FOR_TAKEOVER | 终止 | **0 次投放**, 人工接管 |

---

## 二、精校阶段详细分析

### 发现 1: 像素对心能力正常

`_is_precise()` 判定受 `precision_align_raw_pixel_tolerance = 35.0` 和 `precision_align_smooth_pixel_tolerance = 35.0` 控制 (已从旧值 9/7 放宽)。在此设定下, 精校器成功触发 `final_brake` 共 **8 次**:

| 次数 | 时间 | raw px | smooth px | 结果 |
|---|---|---|---|---|
| 1 | T+209s | 22.6 | 24.8 | final_brake -> 速度等待失败 |
| 2 | T+227s | 28.3 | 26.0 | final_brake -> 速度等待失败 |
| 3 | T+234s | 23.8 | 23.3 | final_brake -> 速度等待失败 |
| 4 | T+239s | 6.6 | 8.0 | final_brake -> 速度等待失败 |
| 5 | T+250s | 16.9 | 18.3 | final_brake -> 速度等待失败 |
| 6 | T+258s | 14.2 | 14.7 | final_brake -> 速度等待失败 |
| 7 | T+274s | 28.9 | 28.8 | final_brake -> 速度等待失败 |
| 8 | T+291s | 25.9 | 26.2 | final_brake -> 速度等待失败 |

### 发现 2: 真正的瓶颈是刹车速度判定

进入 `final_brake` 后, 控制器调用 `_horizontal_speed_ok()` 检查水平合速度是否低于 `precision_align_release_max_horizontal_speed`。此参数当前值为 **0.04 m/s**。

日志中 8 次刹车阶段实测速度:

```
speed_xy 实测范围: 0.078 ~ 0.302 m/s
max=0.040 m/s
```

**全部远超阈值。** 以下是一次典型的刹车速度序列:

```
0.111 -> 0.117 -> 0.114 -> 0.112 -> 0.109 -> 0.116 -> 0.111 -> 0.100
-> 0.102 -> 0.103 -> 0.109 -> 0.111 -> 0.109 -> 0.127 -> 0.112 -> 0.115
-> 0.124 -> 0.127 -> 0.139 -> 0.151 -> 0.155 -> 0.165 -> 0.183 -> 0.192
-> 0.206 -> 0.223
```

注意到速度**不降反升** (0.11 -> 0.22 m/s), 这是因为 PX4 位置控制器在抵抗风扰, 产生的残余速度远大于 0.04。

### 发现 3: 刹车等待无超时, 导致像素漂移后清零重来

当前 `_horizontal_speed_ok()` 没有超时机制。刹车等待期间, 无人机以 0.1~0.3 m/s 漂移, 目标相对位置持续变化。几秒后 `pixel_error` 超出 `raw_tolerance`, `stable_count` 归零, `release_ready` 被清, 回到伺服状态重新对心。

这形成死循环:

```
对心成功 (5 帧)
  -> final_brake (停止所有速度)
  -> speed_xy = 0.12 > 0.04, 等待...
  -> speed_xy = 0.18 > 0.04, 等待...
  -> pixel_error 漂出 tolerance
  -> stable_count = 0, release_ready = False
  -> 重新伺服 -> 再次对心 -> 再次刹车 -> 再次漂移...
```

### 发现 4: 视觉 FPS 在精校阶段暴跌

| 阶段 | FPS | 单帧耗时 |
|---|---|---|
| 巡航 (5m) | 8-12 | ~100ms |
| 精校 (2.8m) | **3-5** | 195-367ms |

3 FPS 意味着每 333ms 才有一次新观测。但伺服脉冲步长 `precision_align_fine_step_s = 0.12s`, 悬停观测 `precision_align_hover_s = 0.04s`。等于**每发 3 个脉冲才能看到一次结果**, 控制严重滞后。

---

## 三、根因总结

| 编号 | 根因 | 严重程度 |
|---|---|---|
| 1 | `precision_align_release_max_horizontal_speed = 0.04` 物理不可达 | **阻塞** |
| 2 | `_horizontal_speed_ok()` 无超时放行机制 | **阻塞** |
| 3 | `precision_align_hover_s = 0.04s` 远小于单帧观测时间 (333ms) | 严重 |
| 4 | `stall_threshold = 25` 小于新 tolerance (35), 形成中间死区 | 中等 |
| 5 | 精校阶段视觉 FPS 从 10 降到 3-5 | 待查 |
| 6 | `landing_servo_deadzone_px = 12` 与投放 tolerance 35 不一致 | 低 |

---

## 四、修改策略

### P0 -- 阻塞投放, 必须修改

#### 4.1 提高刹车速度阈值

| 文件 | 参数 | 当前值 | 目标值 |
|---|---|---|---|
| mission_params.yaml | precision_align_release_max_horizontal_speed | 0.04 | **0.15** |
| mission_params_real.yaml | 同上 | 0.04 | **0.15** |

理由: 日志中 8 次刹车速度分布在 0.10~0.30 m/s, 0.15 可覆盖约一半的帧, 且投放高度 2.8m 下 0.15m/s 漂移完全安全。

#### 4.2 刹车速度等待增加超时放行

文件: `precision_align_controller.py` step() 和 _horizontal_speed_ok()

理由: 即使提高阈值到 0.15, 强风条件下仍可能不达标。增加 3 秒超时: 若刹车超过 3 秒速度仍未降到阈值以内, 但 pixel_error 仍然在 tolerance 内, 直接放行 release_ready。

#### 4.3 卡滞检测阈值与 tolerance 对齐

文件: `state_drop.py` L181

```python
# 当前
stall_threshold = 25.0
# 改为
stall_threshold = 40.0
```

理由: tolerance=35, stall=25 时, 误差在 25-35 区间既不通过精校也不触发卡滞。stall 须 >= tolerance, 留 5px 缓冲。

### P1 -- 改善稳定性, 强烈建议

#### 4.4 增加悬停观测时间

| 文件 | 参数 | 当前值 | 目标值 |
|---|---|---|---|
| mission_params.yaml | precision_align_hover_s | 0.04 | **0.15** |
| mission_params_real.yaml | 同上 | 0.04 | **0.15** |

理由: 3 FPS 下 333ms 一帧, 0.04s 悬停等不到新观测。0.15s 可确保多数帧能被捕获再进入下一步。

#### 4.5 降落伺服死区放宽

| 文件 | 参数 | 当前值 | 目标值 |
|---|---|---|---|
| mission_params.yaml | landing_servo_deadzone_px | 12.0 | **25.0** |
| mission_params_real.yaml | 同上 | 12.0 | **25.0** |

理由: 投放 tolerance 已放宽到 35, 降落仍然 12 不一致。真机近地振动下 25px 是合理目标。

#### 4.6 降落 H 标识最低置信度

| 文件 | 参数 | 当前值 | 目标值 |
|---|---|---|---|
| mission_params.yaml | landing_target_min_confidence | 0.60 | **0.45** |
| mission_params_real.yaml | 同上 | 0.60 | **0.45** |

理由: 近地 H 标识畸变, YOLO 置信度常掉到 0.4-0.5。

#### 4.7 视觉丢失超时

| 文件 | 参数 | 当前值 | 目标值 |
|---|---|---|---|
| mission_params.yaml | vision_loss_timeout | 3.0 | **5.0** |
| mission_params_real.yaml | 同上 | 3.0 | **5.0** |

理由: 真机光照变化、短暂遮挡很容易超过 3 秒。

---

## 五、涉及文件清单

| 文件 | 修改内容 |
|---|---|
| src/drone_mission/config/mission_params.yaml | SITL 参数值 |
| src/drone_mission/config/mission_params_real.yaml | 真机参数值 |
| src/drone_mission/drone_mission/mission_state_machine.py | ROS 参数默认值声明 |
| src/drone_mission/drone_mission/states/state_drop.py | L181 stall_threshold 硬编码 |
| src/drone_mission/drone_mission/precision_align_controller.py | 刹车速度超时放行逻辑 |

---

## 六、改动前后预期行为对比

```
【改动前】
  对心 -> brake -> speed_xy=0.12 > 0.04 -> 等待 -> 像素漂移 -> 重置
  -> 重新对心 -> brake -> speed_xy=0.18 > 0.04 -> 等待 -> 像素漂移 -> 重置
  -> ... 25s 超时, retry=1 -> 再 25s -> 人工接管
  结果: 0 次投放

【改动后】
  对心 -> brake -> speed_xy=0.12 < 0.15, 通过 -> release_ready -> 投放
  或:
  对心 -> brake -> speed_xy=0.18 > 0.15 -> 3s 超时 + pixel 仍在 tol 内 -> 放行 -> 投放
  结果: 预计 8 次对心全部可投放
```

---

## 七、注意事项

1. 项目使用 `--symlink-install`, Python 和 YAML 改完重启 launch 即生效, 不需要 `colcon build`
2. `landing_blind_align_tolerance_px = 30.0` 是死参数 (mission_state_machine.py:330 声明但无代码引用), 不需要改
3. P0 三项 (4.1, 4.2, 4.3) 是最小可行修改, 改完即可消除停滞
4. 视觉 FPS 下降 (3-5 FPS) 的根本原因待后续排查, 可能涉及 YOLO 推理优化或 OpenCV 后处理管线
