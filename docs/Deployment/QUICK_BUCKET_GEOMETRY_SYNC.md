# Quick bucket 高度几何同步说明

> 本文记录 2026-09-27 至 2026-10-05 的几何配置过程。旧蓝盘兜底门禁与 `bucket_sim_calibration_valid` 启动参数已停用；当前 bucket 依据圆筒视觉对心或 16 秒超时投放。运行逻辑见 [Quick bucket 当前流程](../Tactics/QUICK_BUCKET_CURRENT_FLOW.md)。下文量测数据仍保留供试投核对。

2026-09-27：将 Jetson 分支 `9f8d05e` 的 bucket 高度修正同步到 Optimization。此次仅修改 bucket 投影、视觉伺服距离和投放落点计算；起飞状态、PX4 通信、真机相机设备及话题未改。`quick_drop_profile` 仍默认 `blue`，`bucket_calibration_valid` 仍默认 `false`。

## 真机量测与配置

装载同样物品、机体平放在起飞地面时，O 点约离地 40 cm，镜头光学中心离地 3–4 cm，前后物品脱离点离地 5–6 cm。配置采用中值：`bucket_home_body_height=0.40 m`、`bucket_camera_down=0.365 m`、`bucket_front_down=bucket_rear_down=0.345 m`。前后脱离点水平偏移分别为前方 `+0.045 m`、`-0.045 m`，左右均为零；桶口离地 0.40 m。真机 quick launch 的桶口高度默认值也为 0.40 m。

在 bucket 模式中，视觉以启动后首次收到的地面机体位姿确定世界系地面高度，再按相机安装偏移投影桶口；落点预测使用物品脱离点的实际高度。**必须在起飞前启动视觉节点**。如果节点在空中重启，地面基准不可信，应停止 bucket 自动投放，在地面重新启动。

O 点是现场选定的机体中心，仍须与 PX4 位姿使用的机体参考点核对。释放延迟、系统性落点偏差和重复试投不确定度仍未测量，小桶自动释放锁保持关闭。按用户决定，小桶名义几何门禁继续使用 4.5 cm；实物 14 cm 桶口与 6.3 cm 投放物对应的几何余量为 3.85 cm，试投时需记录该差额。

## 仿真隔离

SITL launch 改为加载 `vision_params_quick_sim.yaml`，沿用此前仿真验证过的零机体离地基准、零相机安装偏移和 0.30 m 桶口。真机 launch 继续加载 `vision_params_quick.yaml`，使用现场量测值。仿真任务配置也保持原值。因此真机几何不会覆盖已跑通的 SITL 条件。

真机部署时以 Jetson 分支为准；Optimization 的实飞配置用于开发与对比。更新机载电脑后，先确认参数加载、模型、图像与静态投影，再做受控试投。单元测试与 SITL 通过不能保证真机起飞或命中，起飞前仍须完成机载构建和常规地面检查。
