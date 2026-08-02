# 任务参数使用指南

> 当前基线：侦察速度 `0.8 m/s`，侦察航点 yaw 固定为 `0.0`，不随弓字形条带旋转。

## 1. 修改入口

任务阶段参数统一写在：

```text
src/drone_mission/config/mission_params.yaml
```

位置控制安全上限写在：

```text
src/px4_control/config/px4_params.yaml
```

不要在两个文件中重复添加同名阶段速度参数。

## 2. 常用任务参数

| 参数 | 当前值 | 单位 | 作用 |
|---|---:|---|---|
| `sweep_speed` | `0.8` | m/s | 侦察区弓字形扫描速度 |
| `transit_speed` | `2.0` | m/s | 投放区、侦察区之间的转场速度 |
| `return_speed` | `3.0` | m/s | 返航航段的名义速度 |
| `max_setpoint_speed_limit` | `3.0` | m/s | 所有位置模式设定点的桥接层安全上限 |
| `waypoint_tolerance` | `0.5` | m | 航点到达判定容差 |
| `waypoint_timeout` | `120.0` | s | 单个航点最大执行时间 |
| `area2_altitude` | `2.5` | m | 侦察区扫描高度 |
| `grid_spacing` | `1.67` | m | 侦察条带间距 |

实际位置设定点速度遵循：

```text
实际名义速度 = min(阶段速度, max_setpoint_speed_limit)
```

例如全局上限为 `3.0 m/s` 时，侦察、转场、返航分别为 `0.8`、`2.0`、
`3.0 m/s`；将返航速度设为 `4.0` 也只能得到 `3.0 m/s` 的设定点推进速度。
PX4 自身速度参数、风和机体动力学还可能使实际速度更低。

## 3. 哪些功能受全局上限影响

`max_setpoint_speed_limit` 会限制位置模式下的起飞、转场、侦察、返航和位置
悬停修正；不会限制视觉对准、精降等通过速度模式话题发送的指令。

## 4. 侦察 yaw 行为

侦察航点由 `PathPlanner.plan_coverage_ned()` 生成，当前所有侦察航点均为：

```python
"yaw": 0.0
```

含义是机头保持 NED North 方向，不会因为弓字形扫描方向改变而进行 180°
旋转。该设置与 `sweep_speed` 相互独立。

如果将来确实需要改变固定机头方向，只修改路径规划器中的固定 yaw：

```text
0.0       = North
pi / 2    = East
-pi / 2   = West
pi        = South
```

## 5. 修改后生效流程

1. 修改对应 YAML 参数。
2. 检查参数范围和单位。
3. 重新构建相关包：

   ```bash
   colcon build --symlink-install \
     --packages-select drone_interfaces px4_control drone_mission drone_bringup test_mission
   ```

4. 重新加载环境并重启任务节点：

   ```bash
   source install/setup.bash
   ```

5. 首先在 SITL 或受控场地确认航点、速度和航向，再进行真机任务飞行。

参数由任务节点启动时读取；仅编辑文件不会改变已经运行的任务。

## 6. 不要直接修改的项目

- 不要用 `max_setpoint_speed_limit` 代替 `sweep_speed`、`transit_speed`、
  `return_speed`。
- 不要把 `max_acceleration` 当作速度参数；该旧参数已移除。
- 不要把 `NedPositionSetpoint` 中的测试坐标值当作真机航点配置。
- 真机飞行前仍需确认 PX4 的 `MPC_XY_VEL_MAX` 等飞控内部限制。
