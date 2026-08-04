# RTK 官方资料核查（PX4 v1.17 / CUAV 7-Nano / C-RTK）

> 范围：本文是为本工作空间 RTK 部署指南准备的事实核查记录，而不是未经现场验证的参数文件。结论仅引用 PX4、QGroundControl 和 CUAV 的官方文档；参数值在真机上修改前应导出并备份当前参数。

## 结论摘要

- PX4 v1.17 的标准 RTK 架构是**地面 Base + 机载 Rover + 到地面站的 MAVLink2 数传**。Base 通过 USB 接入运行 QGroundControl 的电脑；QGC 将 RTCM 封装成 `GPS_RTCM_DATA` 经数传送给飞控；飞控再把修正转发给已接入的 Rover。Jetson 与飞控之间的 Ethernet/DDS 链路负责本工作空间的 ROS 2 通信，不能替代这条 QGC/RTCM/MAVLink 链路。来源：[PX4 v1.17 — RTK GNSS/GPS（集成）](https://docs.px4.io/v1.17/en/advanced/rtk_gps)
- PX4 v1.17 的支持表明确列出 CUAV C-RTK 9Ps（F9P）及 CUAV C-RTK 系列。CUAV 7-Nano 官方布线说明明确将 `GPS&SAFETY` 用于 NEO GPS 或 C-RTK 系列 RTK，且另有 `GPS2` 接口。来源：[PX4 v1.17 — RTK GNSS/GPS](https://docs.px4.io/v1.17/en/gps_compass/rtk_gps)，[CUAV 7-Nano Quick Wiring](https://doc.cuav.net/controller/7-nano/en/quick-start/quick-start-7-nano.html)
- 常规“Base 通过 QGC 发送修正”的接法不需要在 Rover 上运行 u-blox U-Center；PX4 收到 `GPS_RTCM_DATA` 后会自动使用已有 GPS 数据通道转发 RTCM。来源：[PX4 v1.17 — RTK GNSS/GPS（集成）](https://docs.px4.io/v1.17/en/advanced/rtk_gps)

## 必需链路与接线边界

```text
固定、开阔天空的 C-RTK Base
             │ USB
             ▼
QGroundControl（笔记本/PC）
             │ GPS_RTCM_DATA，MAVLink 2，约 300 B/s 上行
             ▼
地面数传 ────────────────────────── 空中数传 → 7-Nano TELEM1/TELEM2
                                                    │
                                                    ▼
                      C-RTK Rover ← GPS&SAFETY（GPS1）
                                                    │
                                                    ▼
                                PX4 EKF2 → uORB/DDS → px4_control → ROS 任务
```

PX4 官方要求 QGC 运行在笔记本/PC（Android/iOS 不支持 RTK），并要求飞行器用 Wi-Fi 或遥测电台连到该电脑。对于低带宽链路，必须端到端采用 MAVLink2：MAVLink1 会把每一个 RTCM 包填充到 182 字节，约需 700+ B/s，可能使半双工低带宽电台饱和；MAVLink2 的典型上行需求约为 300 B/s。可在 PX4 MAVLink Console 执行 `mavlink status`，确认对应实例显示 `MAVLink version: 2`。来源：[PX4 v1.17 — RTK GNSS/GPS（集成）](https://docs.px4.io/v1.17/en/advanced/rtk_gps)，[PX4 v1.17 — RTK GNSS/GPS](https://docs.px4.io/v1.17/en/gps_compass/rtk_gps)

CUAV 对 7-Nano 的官方接口定义为：`GPS&SAFETY` 可连接 C-RTK 系列，`GPS2` 可连接第二个 GPS/RTK；PX4 端口映射中 GPS1 是 `/dev/ttyS0`，GPS2 是 `/dev/ttyS2`，TELEM1 是 `/dev/ttyS1`、TELEM2 是 `/dev/ttyS3`。因此，本项目首轮应把 **Rover 接 `GPS&SAFETY`/GPS1**，将空中数传接 TELEM1 或 TELEM2；不要将 Rover 接到 Jetson。来源：[CUAV 7-Nano Quick Wiring](https://doc.cuav.net/controller/7-nano/en/quick-start/quick-start-7-nano.html)，[CUAV 7-Nano PX4 Firmware Guide](https://doc.cuav.net/controller/7-nano/en/px4-users-manual.html)

对于 C-RTK 9P，CUAV 要求 Base 和地面遥测分别通过 USB 接 PC，Base 放在无遮挡室外；Rover 要按飞控选择对应线束，空中遥测接飞控 TELEM。其通用 6 针 GPS 线序为 VCC、GPS_RX、GPS_TX、SCL、SDA、GND。必须使用随设备/飞控匹配的线束，**禁止依据颜色自行交叉 TX/RX 或供电**。来源：[CUAV C-RTK 9P](https://doc.cuav.net/gps/c-rtk-series/en/c-rtk-9p.html)

## QGroundControl 配置与可观测验收

1. 先在开阔、固定且不再移动的位置架设 Base；官方建议远离建筑物、可用三脚架或屋顶抬高天线。
2. 在桌面版 QGroundControl 的 `Settings > General Settings > RTK GPS` 选择 `Perform Survey-In`，设置 Survey-In 的最短时长和精度条件；或在已经可靠测量过该固定点后选择 `Use Specified Base Position`。首次 Survey-In 完成后可用 `Save Current Base Position` 保存，以后复用该固定 Base 坐标。
3. 启动 QGC，将 Base USB 连接到电脑；再上电飞行器，并确认飞控已通过遥测/Wi-Fi 连接 QGC。QGC 应自动识别 Base 并开始 Survey-In。
4. **Base 状态验收：**QGC 顶栏 RTK 图标在建站中为红色，Survey-In 完成且 QGC 开始向飞行器流式发送修正后为白色；点击图标可查看当前状态和 RTK 精度。
5. **Rover/飞控状态验收：**普通 GPS 状态图标显示 `3D RTK GPS Lock` 后，才可以把本次试验记为 RTK 已生效。仅有卫星数、`3D GPS Lock` 或白色以外的 Base 图标均不等价于 Rover 取得 RTK 解。

来源：[PX4 v1.17 — RTK GNSS/GPS：Positioning Setup/Configuration 与 RTK Connection Process](https://docs.px4.io/v1.17/en/gps_compass/rtk_gps)，[QGroundControl General Settings — RTK GPS](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html)

飞控侧的第二条证据应来自 `sensor_gps`：PX4 v1.17 定义 `fix_type=5` 为 RTK Float、`fix_type=6` 为 RTK Fixed，并提供 `eph`、`epv`、`rtcm_injection_rate`、`rtcm_crc_failed`、`rtcm_msg_used` 等字段。因而“QGC 白色图标 + `3D RTK GPS Lock` + `fix_type=6` + RTCM 注入计数持续增长”是比单一 UI 图标更强的验收记录；字段名与可用 ROS 2 topic 必须以实机的 `px4_msgs` 版本为准。来源：[PX4 v1.17 — SensorGps 消息定义](https://docs.px4.io/v1.17/en/msg_docs/SensorGps)

## PX4 v1.17 参数：当前文件、建议与边界

以下“当前值”来自仓库的 `controller_params_v3.params`，不是 PX4 官方默认值。修改前应从实机 QGC 再导出一份参数，修改后重启飞控并复查值。

| 参数 | 当前文件值 | RTK 部署处理 | 官方含义/边界 |
|---|---:|---|---|
| `GPS_1_CONFIG` | `201` | 保持；201 对应 GPS 1。Rover 接 `GPS&SAFETY` 时这是正确的首选端口。 | 选择主 GPS 运行的串口；201 = GPS 1。[​PX4 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#GPS_1_CONFIG) |
| `GPS_1_PROTOCOL` | `1` | 保持现有经验证的 u-blox 配置；若实际 Rover 型号/协议不符，按其官方兼容说明处理，不要猜测协议编号。 | 本表不把它当作“开启 RTK”的开关。 |
| `GPS_UBX_MODE` | `0` | 常规 QGC → 飞控 → Rover 的修正链路保持 `0`（Default）。 | PX4 明确说明大多数设置（含 RTK、非航向双 GPS）使用 Default；**只有**静态 Base 的 RTCM 直接进入 Rover UART2 时才用 Mode 5。[​PX4 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#GPS_UBX_MODE) |
| `MAV_PROTO_VER` | `2` | 保持 `2`，并以 `mavlink status` 验证实际数传链路。 | MAVLink2 是 RTK 低带宽链路的必要条件。[​PX4 RTK 指南](https://docs.px4.io/v1.17/en/gps_compass/rtk_gps) |
| `EKF2_GPS_CTRL` | `7` | 保持首轮值。 | bit 0/1/2 分别为经纬度、高度、3D 速度融合；`7` 即三者开启。不要因单天线 C-RTK 而盲目加 bit 3；bit 3 仅为双天线航向融合。[​PX4 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#EKF2_GPS_CTRL) |
| `EKF2_REQ_FIX` | `3` | 首轮可先保持以验证 RTK 链路；若任务策略要求“未 RTK Fixed 禁止使用 GPS/执行任务”，评审后改为 `6`。 | 最小 GPS fix 类型；`6` = RTK fixed，默认值为 `3` = 3D fix。该参数是 EKF GPS 使用门槛，不是完整的任务状态机安全策略。[​PX4 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#EKF2_REQ_FIX) |
| `EKF2_REQ_EPH` | `3.0 m` | 先保留；待记录多架次 QGC/ULog 的 EPH、创新量和轨迹后再收紧。 | 用 GPS 所需的 EPH，v1.17 范围为 2–100 m；不能据此设成厘米级。[​PX4 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#EKF2_REQ_EPH) |
| `EKF2_GPS_P_NOISE` | `0.5 m` | RTK 固定解稳定后可从保守实验值开始评估是否调至 `0.2`。 | PX4 RTK 指南将 `EKF2_GPS_P_NOISE` 与 `EKF2_GPS_V_NOISE` 下降至 `0.2` 作为可选调参示例；不是无条件处方。[​PX4 RTK 指南](https://docs.px4.io/v1.17/en/gps_compass/rtk_gps) |
| `EKF2_GPS_V_NOISE` | `0.3 m/s` | 同上，不在链路未验证时先改。 | 同上。 |
| `GPS_2_CONFIG` | `0` | 首轮保持禁用，避免未验证的双接收机混入结果。 | 若未来增加第二个 GPS，须同时评审主接收机与混合策略。 |
| `SENS_GPS_MASK` / `SENS_GPS_PRIME` | `7` / `0` | 第二 GPS 启用前不改；启用后必须明确是否要 blending。 | `SENS_GPS_MASK=0` 禁用混合并总用第一实例；有混合时 `SENS_GPS_PRIME` 无效，否则定义优先实例。[​PX4 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#SENS_GPS_MASK) |

**不建议作为 RTK 首轮动作的设置：**

- 不设置 `GPS_UBX_MODE=5`：该模式适用于“修正直接接入 Rover UART2”，而本方案的修正由 QGC 通过 MAVLink 到飞控后自动转发。
- 不打开 `EKF2_GPS_CTRL` bit 3 / 不配置 `GPS_YAW_OFFSET`：只有经过设备确认的双天线 GNSS 航向方案才需要；PX4 指南要求同时设置偏航基线角与该 bit。
- 不把 `EKF2_REQ_EPH` 设为 0.2 m：PX4 v1.17 的参数范围下限为 2 m；应使用 RTK 解状态、QGC 精度、ULog 和任务侧质量门综合判定。

## 验证顺序与故障定位

1. **台架（无桨）**：确认 QGC 可以同时看到飞控和 Base；`mavlink status` 显示 MAVLink 2；Base Survey-In 完成，RTK 图标白色；飞控普通 GPS 图标为 `3D RTK GPS Lock`。
2. **静态户外（无任务）**：记录至少一次完整的 Survey-In、Rover 固定解建立、断开/恢复数传后的状态变化；Base 在整个测试中不能移动。
3. **人工低风险飞行**：悬停、短距离往返和返航，保存 ULog/QGC telemetry log；此阶段不启用投放和自主任务。
4. **任务接入前**：只有状态证据表明每次起飞前都达到 `3D RTK GPS Lock`，才考虑将 `EKF2_REQ_FIX=6` 和工作空间的 `position_ready` 质量门纳入任务前置条件。

若 QGC 未识别 Base，检查桌面版 QGC 的 `Settings > General > Autoconnect to the following devices > RTK GPS`；QGC 官方说明串口设备冲突时可关闭此自动连接、退出 QGC、重新插拔设备后再打开。若 Base 已白色但飞行器仍非 `3D RTK GPS Lock`，优先检查飞控到 QGC 的双向 MAVLink2 数传，而不是修改 ROS 2/DDS 代码。来源：[QGroundControl General Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html)，[PX4 v1.17 — RTK GNSS/GPS（集成）](https://docs.px4.io/v1.17/en/advanced/rtk_gps)

## 版本和适配性注意事项

- 本文核查对象为 PX4 **v1.17**。参数名称/位值、QGC UI 与新版本可能变化；实施时必须以飞控运行的固件版本和 QGC 实际界面为准。
- PX4 的 C-RTK 9Ps 支持表描述的是设备能力，不等于本机线束、固件、天线安装和数传带宽已自动正确。采购/装机前应核对具体产品是 Base 还是 Rover，且保留 CUAV 随套件提供的对应飞控线束。CUAV 已将 9P 标为停产并推荐 9Ps；9Ps 官方资料说明其兼容 PX4、支持 Base/Rover 角色互换。来源：[CUAV C-RTK 9P](https://doc.cuav.net/gps/c-rtk-series/en/c-rtk-9p.html)，[CUAV C-RTK 9Ps](https://doc.cuav.net/gps/c-rtk-series/en/c-rtk-9ps/)
- 9Ps 的官方单 Rover 接法是 UART1 专线到飞控 GPS 端口、空中数传到 TELEM1，模块朝向与机身前向一致；该设备供电范围为 4.5–6 V。若使用 GPS2 或转换线束，必须按产品手册逐针核对，不能按线色或 TX/RX 名称猜接。来源：[CUAV C-RTK 9Ps](https://doc.cuav.net/gps/c-rtk-series/en/c-rtk-9ps/)
- RTK 精度改善位置观测，不会消除桨流、机体振动、控制器整定、罗盘干扰、风或任务逻辑误差。采用单天线 Rover 也不会自动得到 GNSS 偏航。

## 官方来源索引

1. [PX4 v1.17 — RTK GNSS/GPS](https://docs.px4.io/v1.17/en/gps_compass/rtk_gps)
2. [PX4 v1.17 — RTK GNSS/GPS（集成）](https://docs.px4.io/v1.17/en/advanced/rtk_gps)
3. [PX4 v1.17 — 参数参考](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference)
4. [QGroundControl — General Settings / RTK GPS](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html)
5. [CUAV — 7-Nano Quick Wiring](https://doc.cuav.net/controller/7-nano/en/quick-start/quick-start-7-nano.html)
6. [CUAV — 7-Nano PX4 Firmware Guide](https://doc.cuav.net/controller/7-nano/en/px4-users-manual.html)
7. [CUAV — C-RTK 9P](https://doc.cuav.net/gps/c-rtk-series/en/c-rtk-9p.html)
