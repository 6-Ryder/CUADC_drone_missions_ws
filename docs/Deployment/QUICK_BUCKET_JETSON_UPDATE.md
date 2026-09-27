# Quick 小桶精投：Jetson 分支更新说明

本次从 Optimization 移植 quick 功能逻辑。Jetson 原有相机设备路径、图像话题、PX4 v1 话题、DDS UDP 端口、舵机配置、TensorRT `.engine` 模型路径和真机相机方向不变。默认 `quick_drop_profile=blue`，继续使用原有蓝盘 quick 投放。

## 功能边界

- `blue`：继续以蓝盘为目标，边视觉伺服边下降，并按原有条件发出一次全开双投。新增的 shadow 日志只记录投放时的估算数据，不参与投放决策。
- `bucket`：搜索阶段联合识别小桶与蓝盘，下降阶段使用采集时刻的位姿投影和桶口尺寸确认。投放前复核目标身份、位姿和速度新鲜度、下降高度、前瓶落入小桶与后瓶落入蓝盘的预测误差；条件失效则继续对心或请求人工接管。
- `bucket_calibration_valid: false` 是默认值：小桶自动释放禁用。在此状态下，bucket 模式仅可能在独立的蓝盘回退门禁满足时投放蓝盘，不能据此宣称小桶精投成功。

当前已收到的现场数据：桶口距地面 **0.40 m**、桶口内径 **14 cm**、投放物最大外径 **6.3 cm**，相机光轴朝正下方。真机 quick launch 的桶口高度默认值已同步为 0.40 m。以前述机体 O 点为参考，前舱脱离点为前方 **4.5 cm**、右偏 **0**，后舱为后方 **4.5 cm**、右偏 **0**；镜头水平偏移为 **0**。镜头竖直偏移报告为 `-44 cm`，其方向和参考点待核对，尚未写入配置；前后舱竖直偏移、释放延迟和落点误差仍待测。小桶自动释放锁保持关闭。

按实物尺寸，物品中心通过桶口的几何允许偏差为 **3.85 cm**。按现场决定，现阶段投放门禁保留原有 15 cm 桶口与 6 cm 物品的名义计算，即 **4.5 cm**；它比实物几何宽 **0.65 cm**，不能将门禁通过视为实物必然入桶。待重复试投评估落点误差后，再决定是否收紧。

## 机载电脑更新

在任务未运行时，于 Jetson 本机终端执行。保留 `log/` 飞行记录，不复制开发电脑的 `build/`、`install/` 或 TensorRT 引擎。

```bash
cd /home/nvidia/CUADC_drone_missions_ws
git status --short
git fetch origin jetson
git switch jetson
git pull --ff-only origin jetson
source /opt/ros/humble/setup.bash
rm -rf build install
colcon build --symlink-install
source install/setup.bash
ROS_LOG_DIR=/tmp/quick_launch_check ros2 launch drone_bringup quick_flight.launch.py --show-args
```

若 `git status --short` 有改动，先保留这些现场文件并核对差异，再更新。全量重建遵循 [构建与部署](DEPLOYMENT.md) 的版本切换规则；不要把开发机的 x86 构建产物带到 Jetson。

## 模式与标定

启动脚本默认使用 `blue`：

```bash
./scripts/start_quick_mission_real.sh
```

仅在现场量测和静态投放验证完成后，才启用小桶自动精投。在 `src/drone_mission/config/mission_params_quick.yaml` 中填写前后投放口相对机体的偏移、释放延迟、落点偏差和不确定度，并把 `bucket_calibration_valid` 设为 `true`。在 `src/drone_vision/config/vision_params_quick.yaml` 中填写相机相对机体的安装偏移和姿态；当前 launch 默认将已报告的桶口高度 0.40 m 同时传给视觉与任务节点，也可显式用启动参数覆盖。上面两份配置的新增字段只用于 bucket 功能，不更改现有设备或话题。

```bash
./scripts/start_quick_mission_real.sh quick_drop_profile:=bucket bucket_rim_height:=0.40
```

先在无桨或安全工位核对相机图像、两种模型的 TensorRT 加载、桶口投影和帧率，再做静态释放量测与受控试飞。仿真成功只证明功能链路在仿真条件下可运行，不能替代 Jetson 上的模型性能、标定精度及实飞落点验证。
