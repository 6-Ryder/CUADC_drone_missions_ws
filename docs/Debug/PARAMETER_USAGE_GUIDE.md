# 工作空间参数使用与修改指南

> 适用范围：`drone_bringup`、`drone_mission`、`px4_control`、`drone_vision`、
> `drone_servo2` 和 `test_mission`。

> 当前基线：侦察速度 `0.8 m/s`，侦察航点 yaw 固定为 `0.0`，不随弓字形条带旋转。

## 1. 先理解参数是如何生效的

### 1.1 参数文件与运行节点

| 文件 | 主要节点 | 作用 |
|---|---|---|
| `src/drone_mission/config/mission_params.yaml` | `mission_state_machine` | 完整任务 FSM、航点、投放、精校和降落 |
| `src/px4_control/config/px4_params.yaml` | `px4_bridge` | PX4 通信、位置控制和安全上限 |
| `src/drone_vision/config/vision_params.yaml` | `vision_pipeline` | 检测模型、视觉后处理、覆盖率和视觉伺服 |
| `src/drone_servo2/config/servo_params.yaml` | `drone_servo2` | 真机/SITL 舵机、PWM 和投放时序 |
| `src/drone_servo/config/servo_params.yaml` | Python `drone_servo` | 旧版参考控制器，完整任务默认使用 `drone_servo2` |
| `src/drone_vision/training/dataset_config.yaml` | 训练脚本 | 数据集训练，不影响飞行运行参数 |

启动文件可能在加载 YAML 后再次覆盖参数。ROS 2 的优先级通常是：

```text
代码默认值 < YAML 文件 < launch 文件内联参数 < 命令行 -p 参数
```

因此必须确认实际启动的 launch 文件。只修改 YAML，不代表 launch 中的覆盖值会改变。

### 1.2 三种运行入口

| 入口 | 适用任务 | 主要参数来源 |
|---|---|---|
| `drone_bringup sitl.launch.py` | 完整 SITL 任务 | 四个运行 YAML，SITL 覆盖 DDS topic 和解锁策略 |
| `drone_bringup real_flight.launch.py` | 完整真机任务 | 四个运行 YAML，真机覆盖 Home、相机、DDS 和 RC 策略 |
| `drone_bringup simple_mission.launch.py` | 简单起飞-投放-返航测试 | launch 参数 + `px4_params.yaml`，不使用完整 mission YAML |
| `test_mission test_forward_recon.launch.py` | 前飞+侦察分阶段测试 | launch 参数，独立于完整 FSM 的任务参数 |
| `drone_bringup vision_only.launch.py` | 单独视觉调试 | `vision_params.yaml` + 相机 launch 参数 |

### 1.3 修改后的统一流程

```bash
cd /home/nvidia/CUADC_drone_missions_ws
编辑对应源文件
colcon build --symlink-install
source install/setup.bash
重新启动对应 launch
```

参数大多在节点初始化时读取。任务已经运行时，修改文件不会改变当前任务。

## 2. 坐标、单位和部署差异

### 2.1 两套坐标系

任务控制统一使用 PX4 NED：

```text
North：北，正方向
East ：东，正方向
Down ：下，正方向；飞行高度为负值
```

Gazebo World 使用 `x=East, y=North, z=Up`。Home 参数负责两套坐标的原点偏移。

### 2.2 Home 参数

| 参数 | 单位 | 完整 SITL 默认 | 真机默认 | 作用 |
|---|---:|---:|---:|---|
| `home_x` | m | `4.0` | `0.0` | World East 原点 |
| `home_y` | m | `10.0` | `0.0` | World North 原点 |
| `home_z` | m | `0.0` | `0.0` | World 高度原点 |

真机通常把解锁点作为 `(0,0,0)`。修改 Home 后，区域位置、覆盖监控坐标和视觉世界坐标必须一起核对。

### 2.3 DDS 版本

| 场景 | `vehicle_status_topic` |
|---|---|
| 真机 | `/fmu/out/vehicle_status_v1` |
| SITL | `/fmu/out/vehicle_status_v4` |

SITL launch 会覆盖 YAML 中的 v1。真机不要直接套用 SITL 的 v4 topic。

## 3. 完整任务 FSM 参数

完整任务参数文件是：

```text
src/drone_mission/config/mission_params.yaml
```

### 3.1 任务入口和全局安全

| 参数 | 当前值 | 作用与修改建议 |
|---|---:|---|
| `sim_mode` | `true` | SITL/真机通信模式。真机由 launch 覆盖为 `false`。 |
| `flight_axis` | `north` | 主飞行轴，只允许 `north` 或 `east`。 |
| `drop_distance` | `32.5` m | Home 到投放区中心的距离。 |
| `recon_distance` | `57.5` m | Home 到侦察区中心的距离。 |
| `takeoff_height` | `5.0` m | 起飞和巡航高度。高度参数为正数，内部转换为 NED 负值。 |
| `waypoint_tolerance` | `0.5` m | 航点到达容差。过小会导致迟迟无法到达，过大会降低路径精度。 |
| `takeoff_tolerance` | `0.3` m | 起飞高度确认容差。 |
| `control_frequency` | `20.0` Hz | FSM 控制频率。建议保持在 5~100 Hz。 |
| `max_error_count` | `5` | 当前仅声明，现行 FSM 没有直接消费；修改它不会改变当前错误处理。 |
| `waypoint_timeout` | `120.0` s | 单个航点最大执行时间。 |
| `land_altitude` | `0.15` m | 当前仅声明，现行着陆流程不直接读取；真机接地以 PX4 状态为准。 |
| `rc_require_arm_signal` | `true` | 真机建议为 `true`，等待遥控器 Arm；SITL 通常由 launch 设为 `false`。 |
| `battery_warning` | `0.25` | 电池剩余比例警告阈值。 |
| `battery_critical` | `0.15` | 电池剩余比例严重阈值。 |
| `vision_loss_timeout` | `3.0` s | 视觉连续丢失后的超时处理。 |
| `state_max_duration` | `180.0` s | 单个 FSM 状态最大持续时间。 |

关键参数会在启动时进行范围检查。当前代码重点检查高度、频率、电量、视觉超时、速度和网格间距。

### 3.2 任务阶段速度

| 参数 | 当前值 | 影响范围 |
|---|---:|---|
| `sweep_speed` | `1.2` m/s | 投放区/侦察区内部扫描航点 |
| `transit_speed` | `2.5` m/s | 投放区与侦察区之间的转场 |
| `return_speed` | `3.8` m/s | 返航航段 |

位置模式的设定点推进速度为：

```text
min(阶段速度, px4_params.yaml:max_setpoint_speed_limit)
```

PX4 的 `MPC_XY_VEL_MAX` 等飞控参数仍可能进一步降低实际速度。

不要在 `px4_params.yaml` 中重新添加三个阶段速度。阶段速度的唯一任务配置来源是 mission 参数。

### 3.3 投放区 Area 1

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `area1_north_min/max` | `30.0/35.0` m | 投放区 NED North 边界 |
| `area1_east_min/max` | `-4.0/4.0` m | 投放区 NED East 边界 |
| `area1_altitude` | `2.0` m | 低空投放高度 |
| `area1_search_altitude` | `5.0` m | 搜索和建图高度 |

修改边界时要保证 `min < max`，并同步检查相机覆盖范围和目标记忆坐标。

### 3.4 侦察区 Area 2 和弓字形路径

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `area2_north_min/max` | `55.0/60.0` m | 侦察区 NED North 边界 |
| `area2_east_min/max` | `-4.0/4.0` m | 侦察区 NED East 边界 |
| `area2_altitude` | `2.5` m | 侦察扫描高度 |
| `grid_spacing` | `1.67` m | 相邻扫描条带间距 |
| `sweep_margin_north` | `1.0` m | 当前 NED 侦察规划未消费；修改不会改变现行侦察边界。 |
| `sweep_margin_east` | `0.5` m | 当前 NED 侦察规划未消费；修改不会改变现行侦察边界。 |
| `recon_require_full_path` | `true` | 是否必须完成全部弓字形航点 |

当前侦察航点 yaw 在 `PathPlanner.plan_coverage_ned()` 中固定为 `0.0`，机头朝 NED North。

这不是 YAML 参数。修改扫描速度或全局速度上限不会改变侦察航向。

`recon_require_full_path=false` 时，覆盖率达标可能提前结束侦察；真机完整覆盖任务建议保持 `true`。

### 3.5 目标记忆与投放选择

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `max_drops` | `2` | 最大投放次数，必须与 PWM 位置数量减一一致。 |
| `min_radius_first` | `true` | 第一轮优先选择半径较小的圆筒。 |
| `min_drop_targets_required` | `2` | 搜索阶段至少累计的目标数量。 |
| `cylinder_memory_merge_distance` | `0.75` m | 合并重复圆筒世界坐标的距离。 |
| `cylinder_memory_min_observations` | `1` | 圆筒进入候选库所需观测次数。 |
| `drop_marker_min_confidence` | `0.35` | 蓝色标识作为兜底候选的最低置信度。 |
| `drop_marker_fallback_delay` | `15.0` s | 圆筒搜索失败后等待多久才启用蓝色标识兜底。 |
| `marker_memory_merge_distance` | `0.75` m | 合并重复蓝色标识的距离。 |
| `marker_memory_min_observations` | `3` | 蓝色标识进入候选库所需观测次数。 |

圆筒候选优先于蓝色标识。只有完全没有合格圆筒候选时才使用标识兜底。

### 3.6 投放降高和旧版视觉探测

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `drop_descend_height` | `2.0` m | 从搜索高度下降到投放高度。 |
| `drop_confirmation_timeout` | `5.0` s | 投放确认总预算，必须大于舵机层约 3 s 预算。 |
| `drop_align_pixel_tolerance` | `35` px | 投放前允许的像素误差。 |
| `drop_visual_align_probe_step` | `0.40` m | 梯度探测单步距离。 |
| `drop_visual_align_probe_speed` | `0.30` m/s | 梯度探测速度。 |
| `drop_visual_align_probe_settle` | `1.0` s | 探测后等待稳定时间。 |
| `drop_visual_align_world_gate` | `1.5` m | 当前检测与记忆目标的最大匹配距离。 |
| `drop_visual_align_stable_frames` | `6` | 连续居中帧数。 |
| `drop_visual_align_timeout` | `18.0` s | 旧版低空校准超时。 |
| `drop_visual_align_brake_threshold` | `80` px | 进入刹车悬停的像素误差。 |
| `drop_visual_align_brake_timeout` | `4.0` s | 刹车悬停最大时间。 |
| `drop_visual_align_hover_time` | `2.5` s | 探测提交后的观察时间。 |
| `drop_visual_align_lock_tolerance` | `2.0` m | 锁定目标后的世界坐标容差。 |

当前 `precision_align_enable=true` 时，完整任务优先使用新版精校控制器；旧版参数主要用于回退路径。

### 3.7 新版投放精校

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `precision_align_enable` | `true` | `true` 使用新版精校，`false` 回退旧版探测。 |
| `precision_align_target_source` | `cylinder` | 优先跟踪 `cylinder`，`marker` 仅作兜底。 |
| `precision_align_image_width/height` | `1280/960` | 精校输入图像尺寸；真机 launch 覆盖为 `640/480`。 |
| `precision_align_alpha` | `0.55` | 像素误差 EMA 平滑系数。越大响应越快，也越容易受抖动影响。 |
| `precision_align_gain` | `1.25` | 归一化像素误差到机体速度的比例增益。 |
| `precision_align_max_speed` | `0.25` m/s | 粗调水平速度上限。 |
| `precision_align_fine_pixel_error` | `90` px | 进入精调速度限制的误差阈值。 |
| `precision_align_fine_max_speed` | `0.065` m/s | 精调速度上限。 |
| `precision_align_min_correction_speed` | `0.012` m/s | 小误差时的最小修正速度。 |
| `precision_align_step_s` | `0.18` s | 粗调脉冲时间。 |
| `precision_align_fine_step_s` | `0.12` s | 精调脉冲时间。 |
| `precision_align_hover_s` | `0.04` s | 每步后的短暂停稳时间。 |
| `precision_align_timeout` | `25.0` s | 单次精校最大时间。 |
| `precision_align_max_retries` | `2` | 超时或丢失后的重试次数。 |
| `precision_align_threshold_norm_x/y` | `0.008/0.008` | 归一化中心误差阈值。 |
| `precision_align_raw_pixel_tolerance` | `9` px | 当前帧像素误差要求。 |
| `precision_align_smooth_pixel_tolerance` | `7` px | 平滑误差要求。 |
| `precision_align_stable_frames` | `5` | 连续稳定帧数。 |
| `precision_align_final_brake_s` | `0.5` s | 投放前零速复查时间。 |
| `precision_align_release_max_horizontal_speed` | `0.04` m/s | 释放时最大水平速度。 |
| `precision_align_min_confidence` | `0.25` | 精校目标最低置信度。 |
| `precision_align_target_world_gate` | `1.5` m | 目标记忆匹配门限。 |
| `precision_align_max_jump_diag_ratio` | `0.55` | 拒绝过大的像素跳变。 |
| `precision_align_reject_center_ratio` | `0.28` | 远离中心的跳变拒绝比例。 |
| `precision_align_reacquire_after_lost` | `12` 帧 | 丢失后清除像素锁定并重捕获。 |
| `precision_align_max_lost_frames` | `48` 帧 | 超过后放弃当前目标。 |
| `precision_align_drop_height_tolerance` | `0.50` m | 进入精校/投放的高度误差。 |
| `precision_align_final_descent_enable` | `true` | 是否先捕获再降到最终高度。 |
| `precision_align_final_drop_height` | `1.25` m | 最终投放高度。 |
| `precision_align_final_descent_speed` | `0.25` m/s | 最终降高速度。 |
| `precision_align_log_interval` | `0.75` s | 精校日志间隔。 |

符号参数决定图像误差到机体移动的方向。更改前必须用受控小幅动作验证，不能只看仿真画面判断符号。

### 3.8 投放区视觉逼近参数状态

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `cylinder_approach_fast_speed` | `0.30` m/s | 历史参数，当前 FSM 仅声明，未被现行投放状态消费。 |
| `cylinder_approach_slow_speed` | `0.15` m/s | 历史参数，当前 FSM 仅声明，未被现行投放状态消费。 |
| `cylinder_approach_memory_speed` | `0.25` m/s | 历史参数，当前 FSM 仅声明，未被现行投放状态消费。 |
| `cylinder_approach_pixel_gain` | `0.65` | 历史参数，当前 FSM 仅声明，未被现行投放状态消费。 |
| `cylinder_approach_max_step` | `0.45` m | 历史参数，当前 FSM 仅声明，未被现行投放状态消费。 |
| `camera_fx/fy/cx/cy` | `539.9/539.9/640/480` | 任务精校使用的相机内参；真机 launch 覆盖为 400/400/320/240。 |

### 3.9 精准降落

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `landing_search_altitude` | `5.0` m | 搜索 H 标识高度。 |
| `landing_approach_speed` | `0.5` m/s | 历史参数，当前着陆状态不直接读取。 |
| `landing_blind_altitude` | `1.5` m | 锁定后切入盲降的高度。 |
| `landing_blind_align_tolerance_px` | `30` px | 允许进入盲降的最大像素误差。 |
| `landing_blind_stable_frames` | `10` | 近地交接所需稳定帧数。 |
| `landing_alignment_latch_max_altitude` | `2.5` m | 允许锁存对心结果的最大高度。 |
| `landing_precision_target` | `0.1` m | 历史参数，当前着陆状态不直接读取。 |
| `landing_servo_kp_pixel` | `0.004` | 像素误差到水平速度的比例增益。 |
| `landing_servo_max_horizontal_vel` | `0.6` m/s | 最大水平修正速度。 |
| `landing_servo_deadzone_px` | `12` px | 小于该误差时停止水平修正。 |
| `landing_servo_x/y_to_right/forward_sign` | `1/-1` | 图像轴到机体方向的符号。 |
| `landing_target_min_confidence` | `0.60` | H 目标最低置信度。 |
| `landing_target_max_jump_px` | `80` px | 连续帧中心最大允许跳变。 |
| `landing_descent_fast/medium/slow` | `0.4/0.25/0.12` m/s | 不同高度段下降速度。 |
| `landing_blind_descent_speed` | `0.30` m/s | 盲降阶段纯垂直下降速度。 |
| `landing_blind_timeout` | `45.0` s | 盲降超时。 |
| `landing_touchdown_max_altitude` | `0.10` m | SITL 备用接地高度阈值。 |
| `landing_touchdown_max_horizontal_speed` | `0.10` m/s | SITL 备用接地水平速度阈值。 |
| `landing_touchdown_max_vertical_speed` | `0.05` m/s | SITL 备用接地垂直速度阈值。 |
| `landing_touchdown_hold_time` | `2.0` s | SITL 备用接地保持时间。 |

真机最终接地以 PX4 `land_detected` 为准，不能只靠 SITL 的本地备用阈值。

## 4. PX4 Bridge 参数

文件：

```text
src/px4_control/config/px4_params.yaml
```

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `vehicle_status_topic` | v1 | PX4 状态 DDS topic，SITL 由 launch 覆盖为 v4。 |
| `takeoff_height` | `5.0` m | FlightController 默认起飞高度。 |
| `waypoint_tolerance` | `0.5` m | Bridge/FlightController 航点容差。 |
| `takeoff_tolerance` | `0.3` m | 起飞确认容差。 |
| `control_frequency` | `20.0` Hz | Bridge 中仅声明；实际 Bridge 定时器当前不是由它配置。 |
| `max_error_count` | `5` | Bridge 中仅声明；当前错误处理不直接读取。 |
| `waypoint_timeout` | `120.0` s | 航点超时。 |
| `land_altitude` | `0.15` m | Bridge 中仅声明；当前接地判定不直接读取。 |
| `max_setpoint_speed_limit` | `5.0` m/s | 所有位置模式设定点的 bridge 安全上限。 |
| `home_x/y/z` | `4/10/0` | Bridge 的 World Home 原点，真机由 launch 覆盖为 0/0/0。 |

`max_setpoint_speed_limit` 影响起飞、转场、侦察、返航和位置悬停修正。

它不限制视觉对准、视觉精校和精准降落使用的速度控制话题。

`control_frequency`、`max_error_count` 和 `land_altitude` 在 Bridge 中也有声明，
但当前 Bridge 控制循环、错误处理和接地判定不直接使用它们；不要把它们当作
已经生效的 PX4 控制开关。

位置设定点消息 `NedPositionSetpoint` 同时携带 NED 位置、yaw 和可选航段速度。

## 5. 视觉参数

文件：

```text
src/drone_vision/config/vision_params.yaml
```

### 5.1 相机和模型基础

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `camera_width/height` | `1280/960` | 相机图像尺寸；真机 launch 覆盖为 640/480。 |
| `camera_fps` | `30.0` | 相机采集帧率。 |
| `fx/fy` | `539.9/539.9` | 视觉节点相机焦距。 |
| `cx/cy` | `640/480` | 主点坐标。 |
| `sim_mode` | `true` | SITL 图像桥或真机 GStreamer。 |
| `model_path` | `cylinder_yolo.onnx` | 主圆筒模型。 |
| `model_width/height` | `640/640` | 模型输入尺寸。 |
| `conf_threshold` | `0.35` | YOLO 最低置信度。降低会增加候选和误检。 |
| `nms_iou_threshold` | `0.45` | NMS 重叠阈值。 |
| `device` | `cuda` | `cuda`、`cpu` 或 `tensorrt`。 |
| `max_inference_fps` | `20.0` | 推理频率上限。 |
| `class_names` | `cylinder` | 主模型类别映射，必须与模型输出一致。 |

模型路径可使用绝对路径或项目模型目录下的相对路径。更换模型时必须同步类别名称和输入尺寸。

### 5.2 蓝色投放标识

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `drop_marker_model_path` | `drop_marker_yolo.onnx` | 蓝色标识模型。 |
| `drop_marker_class_names` | `drop_zone_marker` | 标识类别名称。 |
| `drop_marker_refine_enable` | `true` | 是否进行 HSV 蓝色精修。 |
| `drop_marker_min_blue_ratio` | `0.4` | 蓝色连通域比例下限。 |
| `drop_marker_min_area` | `80` px² | 最小蓝色面积。 |
| `drop_marker_min_aspect_ratio` | `0.4` | 长宽比过滤下限。 |
| `drop_marker_min_circularity` | `0.35` | 圆度过滤下限。 |
| `drop_marker_min_diameter` | `30` px | 最小投影直径。 |
| `drop_marker_min/max_scale_ratio` | `0.45/1.8` | 实测直径与理论直径比例范围。 |

### 5.3 圆筒检测与跟踪

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `cylinder_refine_enable` | `true` | 是否启用 YOLO 框内 OpenCV 精修。 |
| `cylinder_edge_blur` | `3` | 边缘检测前模糊核。应使用合适的奇数。 |
| `cylinder_canny_low/high` | `30/120` | Canny 边缘阈值。 |
| `cylinder_min/max_radius_ratio` | `0.15/1.5` | 椭圆半径与检测框比例范围。 |
| `cylinder_white_brightness` | `100` | 白色验证的亮度下限。 |
| `cylinder_tracking_max_age` | `10` 帧 | 目标允许消失的最大帧数。 |
| `cylinder_tracking_iou_thresh` | `0.3` | 多帧 IoU 匹配阈值。 |
| `cylinder_tracking_min_hits` | `3` | 输出目标前连续命中帧数。 |

误检多时优先提高 `conf_threshold`、`cylinder_tracking_min_hits` 或白色亮度门限。

漏检多时先确认曝光、焦距和模型，再谨慎降低阈值。

### 5.4 H 标识

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `landing_model_path` | `landing_h_yolo.onnx` | H 标识模型。 |
| `landing_class_names` | `landing_H` | H 类别名称。 |
| `landing_refine_enable` | `true` | H 框内 OpenCV 精修。 |
| `landing_adaptive_thresh_block` | `11` | 自适应阈值块大小，应为合适奇数。 |
| `landing_circle_min_area` | `80` px² | H 精修的最小面积。 |
| `landing_H_orientation_enable` | `true` | 是否估计 H 的方向。 |

### 5.5 覆盖率监控

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `coverage_zone_x_min/max` | `0/8` m | World East 覆盖区域边界。 |
| `coverage_zone_y_min/max` | `65/70` m | World North 覆盖区域边界。 |
| `coverage_grid_resolution` | `0.5` m | 覆盖率网格分辨率。 |
| `coverage_fov_half_angle` | `35°` | 相机视场半角。 |
| `coverage_complete_ratio` | `0.95` | 覆盖完成比例。 |

这些参数只决定视觉覆盖率监控，不决定实际弓字形航点；实际航点由 mission 的 Area 2 参数决定。

真机 launch 会把覆盖区域覆盖为 `x=-4..4、y=55..60`，必须与真机 Home 和坐标定义一致。

### 5.6 视觉伺服

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `servo_kp` | `0.5` | 比例增益。 |
| `servo_ki` | `0.02` | 积分增益。 |
| `servo_kd` | `0.1` | 微分增益。 |
| `servo_max_horizontal_vel` | `1.0` m/s | 最大水平修正速度。 |
| `servo_descent_speed` | `0.3` m/s | 视觉伺服下降速度。 |
| `servo_integral_max` | `0.5` | 积分项限幅。 |
| `servo_target_deadzone` | `0.03` m | 目标死区。 |

精校和精准降落还有各自的 mission 参数，不能把这组视觉参数当作全部降落速度控制。

## 6. 舵机投放参数

完整 launch 使用 C++ `drone_servo2`：

```text
src/drone_servo2/config/servo_params.yaml
```

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `platform` | `sitl` | `sitl` 使用 Mock，`jetson` 使用 PX4 VehicleCommand。 |
| `pwm_positions` | `[1150,1700,2150]` μs | 初始闭合、第一次投放、第二次投放位置。 |
| `drop_delay_ms` | `1000` ms | 每次投放位置保持时间。 |
| `drop_timeout_s` | `3.0` s | 舵机命令和 ACK 总预算。 |

`pwm_positions` 至少需要两个值。其数量减一必须不小于 `mission_params.yaml:max_drops`。

真机修改 PWM 前必须确认 PX4 MAIN5、Actuator Set 1 和舵机机械行程，避免卡死或反向动作。

`src/drone_servo/config/servo_params.yaml` 是 Python 参考控制器的同构参数，不是完整任务默认入口。

## 7. 简单任务参数

`simple_mission.launch.py` 不使用完整 FSM 的 mission YAML。常用启动覆盖如下：

| launch 参数 | 当前默认 | 作用 |
|---|---:|---|
| `cruise_altitude` | `5.0` m | 简单任务巡航/起飞高度。 |
| `drop_x/drop_y` | `4.0/42.5` m | World 投放区位置。 |
| `home_x/home_y` | `4.0/10.0` m | World Home。 |
| `loiter_time` | `5.0` s | 投放区悬停时间。 |
| `sim_mode` | `true` | SITL 或真机模式。 |
| `rc_require_arm_signal` | `false` | 真机建议显式设置为 `true`。 |
| `vehicle_status_topic` | v4 | SITL topic；真机传入已验证版本。 |

示例：

```bash
ros2 launch drone_bringup simple_mission.launch.py \
  sim_mode:=false home_x:=0.0 home_y:=0.0 \
  drop_x:=0.0 drop_y:=20.0 rc_require_arm_signal:=true
```

## 8. 前飞侦察测试参数

`test_forward_recon.launch.py` 是独立测试任务，不读取完整任务的 Area 2 和 mission YAML。

| launch 参数 | 当前默认 | 作用 |
|---|---:|---|
| `sim_mode` | `true` | SITL/真机。 |
| `cruise_altitude` | `5.0` m | 巡航高度。 |
| `forward_distance` | `20.0` m | Phase 1 向北直飞距离。 |
| `transit_speed` | `2.5` m/s | 前飞转场速度。 |
| `recon_altitude` | `2.5` m | 测试侦察高度。 |
| `recon_area_half_north` | `2.5` m | 侦察区 North 半宽。 |
| `recon_area_half_east` | `4.0` m | 侦察区 East 半宽。 |
| `sweep_speed` | `1.2` m/s | 测试扫描速度。 |
| `grid_spacing` | `1.67` m | 条带间距。 |
| `rc_require_arm_signal` | `true` | 真机人工解锁。 |
| `camera_device` | USB 稳定路径 | 真机 V4L2 设备。 |
| `vision_params` | `vision_params.yaml` | 视觉参数文件路径。 |

示例：

```bash
ros2 launch test_mission test_forward_recon.launch.py \
  sim_mode:=false sweep_speed:=1.2 recon_altitude:=2.5 \
  rc_require_arm_signal:=true
```

该测试任务的 `sweep_speed` 只影响测试节点，不会改变完整任务的 `mission_params.yaml`。

## 9. 视觉单独调试参数

`vision_only.launch.py` 主要用于不启动任务 FSM 时验证视觉管线。

| launch 参数 | 作用 |
|---|---|
| `start_camera` | 是否同时启动相机节点。 |
| `camera_device` | 真机 V4L2 设备路径。 |

视觉参数仍来自 `vision_params.yaml`。相机尺寸、帧率和内参在 launch 内可能覆盖 YAML。

## 10. 按目标修改参数

### 10.1 只改侦察速度

修改：

```yaml
# mission_params.yaml
sweep_speed: 1.2
```

不要修改 `max_setpoint_speed_limit`（当前 5.0 m/s），除非要改变所有位置模式的安全上限。

### 10.2 同时改变扫描区域

同时检查：

```text
mission_params.yaml: area2_north/east_min/max
vision_params.yaml: coverage_zone_x/y_min/max
```

前者改变实际飞行航点，后者改变覆盖率监控区域。

### 10.3 修改飞行高度

完整任务分别使用 `takeoff_height`、`area1_altitude`、`area1_search_altitude` 和 `area2_altitude`。

高度变化会改变相机地面投影、覆盖率和视觉目标像素尺寸，必须同步检查视觉参数。

### 10.4 调整检测灵敏度

优先调整模型置信度、跟踪命中帧数和 OpenCV 后处理阈值。

不要一开始同时修改多个阈值；每次只改一个参数并记录漏检、误检和目标稳定性。

### 10.5 调整投放机构

先确认 `max_drops` 与 `pwm_positions` 数量关系，再调整 PWM。

`drop_confirmation_timeout` 必须覆盖舵机 ACK、保持时间和任务状态机等待时间。

### 10.6 调整返航速度

修改 `mission_params.yaml:return_speed`。实际位置设定点速度仍受 `max_setpoint_speed_limit` 和 PX4 内部速度参数限制。

## 11. 不要直接修改的项目

- 不要把 `max_setpoint_speed_limit` 当作侦察速度参数。
- 不要重新添加已经移除的 `max_acceleration` 或 `sweep_passes`。
- 不要把测试代码中的坐标值当作真机航点参数。
- 不要只改视觉覆盖区域而不检查实际侦察区域。
- 不要把 `vision_params.yaml` 的相机内参和 mission 的 `camera_fx/fy/cx/cy` 混为一组。
- 不要在真机将 `rc_require_arm_signal` 设为 `false`，除非已经完成明确的安全评审。
- 不要在未确认 PX4 固件版本时切换 `vehicle_status_topic`。

## 12. 修改后的验证清单

### 配置静态检查

```bash
git diff --check
```

确认 YAML 缩进、单位和 `min < max` 边界正确。

### 构建

```bash
colcon build --symlink-install \
  --packages-select drone_interfaces px4_control drone_mission \
  drone_vision drone_servo2 drone_bringup test_mission
source install/setup.bash
```

### 运行前检查

```bash
ros2 param list /mission_state_machine
ros2 param get /mission_state_machine sweep_speed
ros2 param get /px4_bridge max_setpoint_speed_limit
```

注意：节点名称和参数命名空间必须与实际 launch 一致。

### SITL/真机检查重点

| 检查项 | 预期 |
|---|---|
| 阶段速度 | 侦察 `0.8`、转场 `2.0`、返航 `3.0` 的设定点推进速度 |
| 侦察 yaw | 所有侦察航点为 `0.0`，不出现条带交替旋转 |
| 位置话题 | `/drone/cmd_setpoint_ned` 含 NED、yaw 和 speed |
| PX4 轨迹 | `/fmu/in/trajectory_setpoint` 位置与 yaw 连续 |
| 真机 DDS | 使用已验证的 v1 topic |
| 舵机 | PWM、ACK、投放次数与任务参数一致 |
| 降落 | 真机以 PX4 `land_detected` 为最终接地依据 |

真机修改参数后，必须先在受控场地低风险验证，再执行完整任务。

## 13. 参数所有权速查

```text
任务阶段速度/区域/状态逻辑  → drone_mission/config/mission_params.yaml
位置模式全局安全上限         → px4_control/config/px4_params.yaml
模型/检测/覆盖/视觉伺服       → drone_vision/config/vision_params.yaml
PWM/舵机时序                 → drone_servo2/config/servo_params.yaml
部署 Home/DDS/相机/解锁       → 对应 launch 文件参数
简单任务                     → simple_mission.launch.py
前飞侦察测试                 → test_forward_recon.launch.py
```

如果一个参数在多个文件出现，先确认它属于哪个节点，再检查 launch 是否覆盖了 YAML。
