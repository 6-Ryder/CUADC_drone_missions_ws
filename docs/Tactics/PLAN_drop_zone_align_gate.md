# 方案规划:投放区视觉对心判定重构(用户终版决策)

> 状态:**已实施完成**(2026-08-06);剩真机复飞验证 1.5m 投放段窗口
> 基线:HEAD `57ea255`(投放区两级降高调整后)
> 数据源:`real_flight_logs/2026_8_05/ros2_log/real_mission_20260805_195356`(19:53, 效果最好)、`real_flight_logs/2026_8_06/ros2_log/real_mission_20260806_*`(09:30 / 10:01)
> 姊妹分析:`docs/Tactics/DROP_STALL_ANALYSIS.md`(10:01 深入分析,含 FPS 3-5、stall 对齐等被本方案吸收的发现)
> 决策人:用户(2026-08-06 权衡会)

---

## 1. 背景与根因(两文档合并)

### 1.1 现象

| 架次 | 配置 | 实际表现 | 失败根因 |
|---|---|---|---|
| **8.05 19:53**(效果最好) | 2.0m 降高,窗口 9px | 伺服真收敛到 2.5px、稳定 5/5、进入刹车——刹车 0.5s 后漂到 9.3px(**越界 0.3px**),稳定清零;此后 ±120px 极限环不再收敛,74s 超时 → 接管 | 窗口太窄 + 刹车复查不容忍漂移 |
| 8.06 09:30 | 2.0m 降高 | 目标检测频繁丢失(no_candidates),像素 150-270px 徘徊,48 帧丢失超限 | 检测侧问题,与判定无关 |
| 8.06 10:01 | 2.8m 降高,窗口 35px | **9 次对心成功全部进入刹车,刹车后复查帧像素全部 ≤35px 通过像素链**——卡在刹车后速度等待(53 次),speed_xy 实测 0.078~0.302 m/s(集中 0.10~0.12),门槛 0.04 是实测最低值的 2 倍 | **速度门槛 0.04 真机低空不可达(唯一阻塞)** |

### 1.2 根因(合并后,按伤害排序)

1. **速度门槛 `precision_align_release_max_horizontal_speed: 0.04`(主凶)** —— 真机低空悬停水平速度常态 0.10-0.12 m/s(风 + PX4 位置环残差 + 速度估计噪声),0.04 不可达。10:01 的 9 次对心全部死在速度等待,25s 超时 × 2 次重试耗尽 → 接管。
2. **`threshold_norm_x/y: 0.008`(隐藏杀手,回放新发现)** —— 0.008×640 = 5.1px,比 9px 原窗口还紧,两架次在 0.008 下全部锁死。现场若未放宽,9 次刹车根本不会发生;**放宽临界为 0.06**:10:01 第 1 次刹车复查帧 |dy|=0.0515,0.05 会卡(t+20.8s),0.06 放行(t+3.7s)。
3. **`raw_pixel_tolerance: 9.0px`(19:53 根因)** —— 0.1 m/s 漂移下 9px 撑不过一个刹车周期;现场放宽到 35px 后对心能力被证明正常(10:01)。
4. **刹车 0.5s + 速度等待无超时(10:01 死循环)** —— 速度不达标时无限等待,像素漂移后清零重来。
5. **`smooth_pixel_tolerance: 7.0px`(19:53 附加伤害)** —— EMA 滞后使 smooth 高于 raw。
6. **伺服观测滞后(FPS 3-5,DROP_STALL 发现 4)** —— 精校期 FPS 8-12 → 3-5(333ms/帧),`hover_s=0.04` + `fine_step_s=0.12`,每发 3 个脉冲才见一次结果。

### 1.3 伺服"不收敛"的定性

- 19:53:收敛能力**好**(135px→4px 仅 19s),失败全在判定。
- 10:01:对心能力**正常**(9 次进刹车,像素稳定 5-21 帧);"收敛差"观感是速度卡死的假象。
- 共同物理事实:低空水平漂移 ~0.1 m/s 不可消除,判定必须容忍。

---

## 2. 用户终版决策(2026-08-06 权衡会)

| 项 | 决策 | 理由 |
|---|---|---|
| **速度门槛** | `0.04 → 0.22`,**保留阻塞检查**(不删除、不加超时) | 实测速度 0.078~0.302(53 样本,中位 0.127);**0.22 覆盖 81.1% 帧**,>0.22 的 10 帧是单次等待期的事件性爬升序列(0.223→0.302,非随机帧);10:01 第 1 次刹车速度 0.098 直接通过;释放瞬间带速受控;对心误差仍是核心机制,速度是辅助约束 |
| **norm** | `0.008 → 0.06`,**放宽不删除** | 保留防跳变能力(拦 0.0772+ 的单帧跳变);回放证明 ≥0.06 是 10:01 第 1 次刹车放行的临界(0.05 卡 |dy|=0.0515) |
| **稳定帧** | `5 → 4` | 回放敏感性:3/4/5 双架次均放行,4 无副作用且比 5 早 1s |
| **像素对心误差(两级窗口)** | 精校段(2.8m)`raw=35`;投放段(1.5m)`raw=30` | 像素对心是核心机制;低空目标更大、检测更清楚,收紧到 30px 保证投放精度(1.5m 时 30px ≈ 4.3cm,2.8m 时 35px ≈ 9.4cm) |
| **smooth** | `30`(两级统一) | 10:01 复查帧 smooth 24.8-29.4 全部通过;30 比现场 35 紧一点防噪声 |
| **stall_threshold** | `25 → 40`(采纳 DROP_STALL 4.3) | 对齐窗口:35px 下 stall=25 使 25-35px 徘徊不触发停滞;40 ≥ tol+5 兜底不收敛 |
| **hover_s** | `0.04 → 0.15`(采纳 DROP_STALL 4.4,升级必改) | FPS 3-5 下 0.04s 悬停等不到新观测 |
| **vision_loss_timeout** | `3 → 5`(采纳 DROP_STALL 4.7,一并改) | 真机光照/遮挡易超 3s |
| **landing 死区/置信度** | 暂缓(DROP_STALL 4.5/4.6) | 涉及降落质量,真机近地数据不足,减少本次变量 |

---

## 3. 核心设计(改后判定链,两级窗口)

**释放判定 = 像素大致稳定在窗口内 + 0.5s 刹车 + 速度 ≤ 0.22**,窗口随高度两级收紧:

```
目标存在(置信度/世界门/跳变拒绝, 不变)
→ raw_pixel_error ≤ 窗口(raw: 2.8m 段 35px / 1.5m 段 30px)
→ smooth_pixel_error ≤ 30px(EMA 安全网)
→ |dx|,|dy| ≤ 0.06(norm 防跳变, 不再做紧约束)
→ 连续 4 帧稳定 (stable_frames=4)
→ 刹车 0.5s (stop_velocity, 不变)
→ 复查:像素同阈值 + speed_xy ≤ 0.22 → 放行
```

两级窗口语义:
- **精校段(2.8m)**:窗口 35px —— 现场已验证对心能力(9 次刹车),容忍 2.8m 高度下 ~0.1 m/s 漂移(35px ≈ 9.4cm)。
- **投放段(1.5m)**:窗口收紧 30px —— 降高后目标更大、检测更稳,1.5m 时 30px ≈ 4.3cm,保证最终投放精度。
- 切换:按当前高度(切换阈值 2.0m,介于两级降高之间),参数化可调。

物理意义:
- 0.1-0.15 m/s 漂移 × 0.5s 刹车 = 5-8px,35px 窗口容余 4-6 倍(10:01 复查帧 19.8-33.2px 全部通过)。
- 0.22 m/s × 0.5s ≈ 20px,刹车帧 ≤15px 时复查仍稳过;风况恶化可加长 `final_brake_s`。

---

## 4. 参数一览(终版)

| 参数名 | 现值 | 目标值 | 依据 |
|---|---|---|---|
| `precision_align_release_max_horizontal_speed` | 0.04 | **0.22** | 覆盖实测 81.1%(中位 0.127);保留阻塞 |
| `precision_align_raw_pixel_tolerance` | 9.0(SITL)/35.0(真机) | **35.0**(2.8m 精校段) | 现场已验证 |
| `precision_align_final_raw_pixel_tolerance` | —(新增) | **30.0**(1.5m 投放段) | 低空收紧保证投放精度 |
| `precision_align_tight_window_height` | —(新增) | **2.0 m**(切换高度) | 介于两级降高 |
| `precision_align_smooth_pixel_tolerance` | 7.0 / 35.0 | **30.0** | 复查帧实测 24.8-29.4 全过 |
| `precision_align_threshold_norm_x/y` | 0.008 | **0.06** | ≥0.06 是 10:01 第 1 次刹车放行临界 |
| `precision_align_stable_frames` | 5 | **4** | 敏感性无副作用 |
| `precision_align_final_brake_s` | 0.5 | 0.5(不变) | 复查帧用新阈值即通过 |
| `precision_align_hover_s` | 0.04 | **0.15** | FPS 3-5 伺服滞后 |
| `stall_threshold`(state_drop.py L181,硬编码) | 25.0 | **40.0** | ≥ tol+5,兜底不收敛 |
| `vision_loss_timeout` | 3.0 | **5.0** | 一并改(低风险) |
| `landing_servo_deadzone_px` / `landing_target_min_confidence` | 12 / 0.60 | 暂缓 | 单独评估 |
| `precision_align_alpha` | 0.55 | 0.55(不变) | 不引入变量 |

---

## 5. 回放验证(已完成,终版)

### 5.1 方法

`/tmp/replay_align_v5.py`:解析 mission.log 的 PREC_ALIGN tracking / DESCEND 失败 / "进入最终刹车复查" / "等待水平速度归零"行,合并为观测序列(节流缺帧按 gap<2.5s 视为连续)。**模型**:刹车行(`stable=5/5`)是稳定达成证据,刹车 0.5s 后复查 = 下一个新观测帧**像素 aligned 且速度 ≤ 门槛**(无 speed 日志的架次视为速度通过,19:53 现场证据从未因速度失败)。

### 5.2 结果(终版组合:35/30/0.06/4帧/0.5s/0.22)

| 参数组合 | 19:53 | 10:01 |
|---|---|---|
| 现场全参数(35/35/0.05/5帧 + speed 0.04) | — | **不放行** ✓(保真,53 次速度等待) |
| **用户终版(35/30/0.06/4帧 + speed 0.22)** | **RELEASE@t+26.3s**(复查 8.4px) | **RELEASE@t+3.7s**(复查 33.2px, speed 0.098) |
| norm=0.05(低于临界) | RELEASE@t+26.3s | RELEASE@t+20.8s(被 \|dy\|=0.0515 卡 2 次刹车) |
| norm=0.08 | RELEASE@t+19.1s | RELEASE@t+3.7s |
| stable=3 / 4 / 5 | t+18.3 / **26.3** / 27.1s | 均 t+3.7s |
| speed=0.15 / 0.22 / 0.30 | — | 均 t+3.7s(第 1 次刹车 speed=0.098 全过) |

- 10:01 的 9 次刹车复查帧像素:33.2/39.2/26.4/30.2/63.1/24.5/74.2/20.3/19.8px,6 次直接通过像素链(63.1/74.2 为检测跳变)——与 DROP_STALL"8 次对心全部可投放"叙事一致,**对心能力完全正常,唯一苛刻条件是速度 0.04**。
- **复审:判定链与现场代码逐行核对一致**(precision_align_controller.py L198-203 刹车后新观测像素复查隐含在 stable 累积、L235 `_horizontal_speed_ok` 速度检查)——回放模型非自证。速度分布 53 样本:≤0.15 占 62.3%、≤0.22 占 81.1%、≤0.30 占 98.1%;>0.22 的 10 帧为递增序列(0.223→0.302),是单次等待期的事件性爬升而非随机帧。
- 两级窗口第二段(1.5m/30px)无历史日志数据,判定链回放只验证了精校段(35px);1.5m 段由真机复飞验证。
- 回放器已落地 `tools/replay_align.py` 作复飞回归工具。

---

## 6. 代码改动点

| 文件 | 改动 |
|---|---|
| `src/drone_mission/config/mission_params_real.yaml` | §4 参数更新 + 新参数声明(final_raw_pixel_tolerance / tight_window_height) |
| `src/drone_mission/config/mission_params.yaml` | **不改**(用户决策:仿真保持原参数;SITL 不配新参数 → 两级窗口永不切换,行为不变) |
| `src/drone_mission/drone_mission/precision_align_controller.py` | **两级窗口接入(已核对代码)**:`_nudge_to_visible_drop_target`(L441)已传 `drop_height=drop_h`(当前阶段目标高度:2.8m 精校段 / 1.5m 投放段)→ `_is_precise()`(L276)加 drop_height 参数选 raw_tol:≥2.0 → 35,<2.0 → 30(参数化 `tight_window_height`);norm L285-286 0.008→0.06;`_horizontal_speed_ok()`(L315)门槛 0.04→0.22(阈值走参数,逻辑不变) |
| `src/drone_mission/drone_mission/states/state_drop.py` | L181 `stall_threshold` 25→40;L465-468 锁存复查按当前阶段高度选 raw_tol(与控制器同一参数);L460 stable_frames 默认值 4 |
| `src/drone_mission/drone_mission/mission_state_machine.py` | 新增 2 参数声明(final_raw_pixel_tolerance / tight_window_height);**默认值全部保持原样** |
| `tools/replay_align.py` | **新增**:日志回放回归工具(v5 模型) |
| `src/drone_mission/test/test_precision_align_controller.py` | 阈值断言更新(0.22/0.06/两级窗口);新增两级窗口用例(高度切换) |

---

## 7. 测试计划

1. **回放回归**(`tools/replay_align.py`):两架次 + 现场参数 → 不放行;终版参数 → 放行(t+26.3s / t+3.7s)。
2. **单测**:阈值断言更新;两级窗口切换用例(2.8m → 35px、1.5m → 30px);`_horizontal_speed_ok` 0.22。
3. **SITL 端到端**:完整任务投放成功(现有基线复跑)。
4. **真机复飞**:投放区对心 → 降高 → 投放成功;验证 25s 超时不再触发;重点观察 1.5m 投放段 30px 窗口表现与释放瞬间带速对落点的影响。

---

## 8. 实施顺序

| 步 | 内容 | 验证 |
|---|---|---|
| 1 | `tools/replay_align.py` 落地 + 双架次回归 | 现场参数 X / 终版参数 OK |
| 2 | 参数更新(仅 mission_params_real.yaml + mission_state_machine 声明)+ 单测同步 | pytest 全绿 |
| 3 | 两级窗口接入(controller `_is_precise` + state_drop 复查) | 单测全绿 |
| 4 | SITL 全链复跑 | 投放成功 |
| 5 | 真机复飞 | 投放区成功;评估 FPS 问题与 1.5m 段窗口 |

---

## 9. 风险

1. **speed=0.22 无超时**:实测峰值 0.223-0.302 帧会等待(概率 ~4%),像素漂移出窗后清零重来,下次对心大概率通过(常态 0.10-0.12)。预案:若复飞仍见速度卡死,加 3s 超时放行(DROP_STALL 4.2)或降级为日志。
2. **复查帧越界**:0.22 m/s × 0.5s ≈ 20px,刹车帧 ≤15px 时复查稳过;风况恶化可加长 `final_brake_s`。
3. **两级窗口第二段无日志验证**:1.5m/30px 是首次实装,真机复飞为第一验证点;若 1.5m 段检测/对心表现不如预期,可回调 30→35。
4. **回放缺帧近似**:节流吞帧按 gap<2.5s 视为连续;现场 stable 5-21 帧证据表明近似保守安全。
5. **伺服极限环未根治**:19:53 的 ±120px 极限环在判定修复后不再致命(35px 窗口抓稳定机会);`hover_s=0.15` 是第一步,视觉 FPS 暴跌根因待查(见 DROP_STALL 发现 4)。
6. **stall_threshold 40**:像素 35-40px 徘徊 2s 即触发最终下降——对心质量略降换来兜底,可接受。

---

## 10. 关键文件

| 文件 | 改动 |
|---|---|
| `tools/replay_align.py` | **新增**:日志回放回归工具 |
| `src/drone_mission/config/mission_params_real.yaml` | 参数更新(§4) |
| `src/drone_mission/config/mission_params.yaml` | **不改**(仿真保持原参数,见 §6) |
| `src/drone_mission/drone_mission/precision_align_controller.py` | 两级窗口 + norm 0.06 + speed 0.22 |
| `src/drone_mission/drone_mission/states/state_drop.py` | stall 40 + 复查两级窗口 + stable 4 |
| `src/drone_mission/drone_mission/mission_state_machine.py` | 参数默认值声明同步 |
| `src/drone_mission/test/test_precision_align_controller.py` | 断言同步 + 两级窗口用例 |
| 文档 ×2 | PARAMETER_USAGE_GUIDE、本方案 |
