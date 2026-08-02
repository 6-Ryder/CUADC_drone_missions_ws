# 侦察区速度参数审计（已修复）

> 更新日期：2026-08-02
> 状态：代码与配置已修复；仍需完成 SITL 和受控实飞验证。

完整工作空间参数说明见 [参数使用与修改指南](PARAMETER_USAGE_GUIDE.md)。

## 结论

`sweep_speed` 不再是死参数。完整任务的侦察航段会将
`mission_params.yaml` 中的 `sweep_speed: 0.8` 作为位置设定点的名义推进
速度传到 `px4_bridge`。修改参数后需要重启任务节点；`PathPlanner` 只在
FSM 初始化时读取这些参数。

```text
mission_params.yaml:sweep_speed (0.8 m/s)
  → PathPlanner waypoint["speed"]
  → FlightController.goto_ned(..., speed=0.8)
  → StandaloneBridge
  → /drone/cmd_setpoint_ned (NedPositionSetpoint)
  → PX4Bridge 20 Hz position-setpoint ramp
```

`NedPositionSetpoint` 原子携带 NED 的 `north/east/down`、`yaw`、`speed`
和 `use_speed`。侦察覆盖规划现在将所有扫描航点的 `yaw` 固定为 `0.0`
（机头朝 NED North），因此不会在条带切换时随飞行方向旋转；其他任务阶段
仍可通过各自航点传递 yaw。

## 速度规则与参数归属

位置模式每个控制周期的名义推进速度为：

```text
min(航段 speed, max_setpoint_speed_limit)
```

默认值如下：

| 参数 | 所有者 | 默认值 | 用途 |
|---|---|---:|---|
| `sweep_speed` | `mission_params.yaml` | 0.8 m/s | 侦察弓字形扫描 |
| `transit_speed` | `mission_params.yaml` | 2.0 m/s | 区域间中转 |
| `return_speed` | `mission_params.yaml` | 3.0 m/s | 返航航段 |
| `max_setpoint_speed_limit` | `px4_params.yaml` | 3.0 m/s | bridge 对所有位置模式指令的安全上限 |
| `MPC_XY_VEL_MAX` | PX4 飞控参数 | 机体配置 | PX4 内部最终水平速度上限 |

因此 `return_speed: 3.0` 在默认 bridge 上限下按 3.0 m/s 推进；若把它设为
4.0，则 bridge 仍限制到 3.0。实际机速可能更低，受 PX4 参数、动力和风况
影响。`max_setpoint_speed_limit` 同样会限制起飞、悬停等位置模式指令；它不
影响视觉精校和精准降落使用的速度模式话题。

## 已清理的配置问题

- 删除 FSM 与 mission YAML 中无消费者的 `max_acceleration`；原实现实际
  限制的是设定点速度，不是加速度。
- 删除 `px4_params.yaml` 和 `PX4Bridge` 中重复的任务速度参数；阶段策略
  只由 mission YAML 管理。
- 删除未被路径规划器使用的 `sweep_passes`。
- 将 `grid_spacing` 的 FSM 默认值、fallback 和文档统一为 `1.67 m`。
- 为 `sweep_speed` 补齐与中转/返航一致的 `0.1..15.0 m/s` 参数校验。

## 修改速度的操作

编辑 `src/drone_mission/config/mission_params.yaml`：

```yaml
sweep_speed: 0.8
transit_speed: 2.0
return_speed: 3.0
```

重新构建工作区并重启完整任务。若要调整全局位置模式上限，编辑
`src/px4_control/config/px4_params.yaml` 的
`max_setpoint_speed_limit`；不要用它替代三种任务阶段速度。

## 修改侦察航向

当前侦察区不需要机头随扫描方向旋转，`PathPlanner.plan_coverage_ned()`
将侦察航点 yaw 固定为 `0.0`。该行为不是速度参数，修改
`sweep_speed`、`transit_speed` 或 `max_setpoint_speed_limit` 不会改变它。
如需改变侦察机头方向，应修改路径规划器中的固定 yaw，并同步进行受控验证。

## 验证要求

单元契约测试验证了 speed/yaw 中继和 20 Hz 推进上限。SITL 应分别检查
`/drone/cmd_setpoint_ned` 与 `/fmu/in/trajectory_setpoint`，以 0.8、2.0、
3.0 m/s 对比推进速度。完成 SITL 后，才可在受控场地进行实飞验证。
