"""
PRECISION_LAND (simple) 状态 — test_mission 方法的三阶段精准降落.

由 landing_simple_mode=True 门控启用 (quick 快速投放策略用), 主任务默认
False 不受影响. 逻辑照搬 test_mission/test_forward_recon.py 的 LAND 实现:

  Phase 1 SEARCH: 悬停搜索 H 标识, 超时 → PX4 自主着陆兜底
  Phase 2 SERVO:  图像空间 P 控制器 (像素误差 → 机体速度), 分层下降
  Phase 3 BLIND:  近地纯垂直下降, 等待 PX4 着陆检测

与主任务 PrecisionLandState 的区别 (刻意简化):
  - 接地判定只依赖 PX4 is_landed()
  - H 目标必须满足置信度、帧间连续性和近地对心锁存；丢失短暂
    重捕获后仍失败 → 直接 PX4 LAND
  - 搜索超时 / 盲降超时 → PX4 LAND (VEHICLE_CMD_NAV_LAND), 不等人工接管
  - 盲降切换高度更低 (landing_blind_altitude 0.5m), 盲降速度更慢
    (landing_blind_descent_speed 0.15 m/s) — 近地段更慢但更简单可靠

退出条件:
  - bridge.is_landed() → COMPLETE
  - 视觉丢失 / 搜索超时 / 盲降超时 → 请求 PX4 自主着陆 → COMPLETE
"""

import math
import time
from typing import Optional

from drone_utils.landing_profile import (
    NEAR_GROUND_SLOWDOWN_MAX_ALT_M,
    camera_plane_distance,
    ground_error_m,
    hold_at_search_altitude,
    near_ground_descent_speed,
    nominal_descent_speed,
)
from drone_utils.landing_estimator import LandingTargetEstimator
from drone_utils.lost_target_transit import LostTargetTransit
from drone_utils.servo_pid import ServoPid


class SimplePrecisionLandState:
    """test_mission 方法的三阶段精准降落 (landing_simple_mode=True 时启用)."""

    PHASE_SEARCH = 0     # 搜索 H 标识
    PHASE_SERVO = 1      # 视觉伺服对心下降
    PHASE_BLIND = 2      # 近地盲降

    def __init__(self, fsm):
        self._fsm = fsm
        self._phase = self.PHASE_SEARCH
        self._start_time: float = 0.0
        self._vision_lost_start: float = 0.0
        self._has_target: bool = False
        self._servo_converged: int = 0
        self._visual_alignment_latched: bool = False
        self._vertical_only_latched: bool = False
        self._handoff_stable_frames: int = 0
        self._last_target_center: Optional[tuple[float, float]] = None
        self._last_target_rejection_reason: Optional[str] = None
        self._touchdown_candidate_start: float = 0.0
        self._disarm_requested: bool = False
        self._servo_last_log_time: float = 0.0
        self._px4_land_requested: bool = False
        # 像素域伺服 PID, 跨帧持久; enter() 时 reset 清积分/微分历史
        self._servo_pid = self._new_servo_pid()
        # 丢失目标后继续飞向最后已知位置 (见 _lost_transit_command)
        self._lost_transit = LostTargetTransit(
            max_speed=float(self._fsm.get_param(
                "landing_lost_transit_max_speed", 0.15)),
            tolerance=float(self._fsm.get_param(
                "landing_lost_transit_tolerance", 0.15)),
        )
        # 目标世界位置的 2 状态卡尔曼 (权威的"微分项"来源 + 卡方误检门).
        # 接在丢失后继续接近的路径上: 单帧误检不再直接决定继续接近的落点.
        self._target_estimator = LandingTargetEstimator.from_params(self._fsm)
        self._estimator_last_predict = 0.0
        self._estimator_last_accept = 0.0
        self._observed_measurement_stamp = -1.0
        self._estimator_reject_log_time = 0.0

    def enter(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "开始精准降落 (simple)")
        self._phase = self.PHASE_SEARCH
        self._start_time = time.time()
        self._vision_lost_start = 0.0
        self._has_target = False
        self._servo_converged = 0
        self._visual_alignment_latched = False
        self._servo_pid.reset()
        self._lost_transit.reset()
        self._target_estimator.reset()
        self._estimator_last_predict = 0.0
        self._estimator_last_accept = 0.0
        self._observed_measurement_stamp = -1.0
        self._vertical_only_latched = False
        self._handoff_stable_frames = 0
        self._last_target_center = None
        self._last_target_rejection_reason = None
        self._touchdown_candidate_start = 0.0
        self._disarm_requested = False
        self._servo_last_log_time = 0.0
        self._px4_land_requested = False

        self._fsm.set_vision_mode("landing")
        self._fsm.get_logger().info("PRECISION_LAND: Phase 1 - 搜索 H 标识 (simple)")
        # 在 landing_search_altitude 悬停等待视觉检测 (此前是裸 loiter(),
        # 即"停在上一段留下的高度", 与参数无关)
        hold_at_search_altitude(self._fsm)

    def execute(self) -> Optional[str]:
        # PX4 已检测到着陆 → 完成
        if self._fsm.bridge.is_landed():
            self._fsm.get_logger().info("PRECISION_LAND: 已着陆 → COMPLETE")
            return "COMPLETE"

        if self._disarm_requested:
            return "COMPLETE" if not self._fsm.bridge.is_armed() else None

        if self._fallback_touchdown_confirmed():
            self._fsm.get_logger().info(
                "PRECISION_LAND: SITL 近地静止持续满足, 备用确认接地"
            )
            self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
            self._fsm.bridge.force_disarm()
            self._disarm_requested = True
            return None

        elapsed = time.time() - self._start_time
        if self._phase == self.PHASE_BLIND:
            return self._execute_blind(elapsed)
        if self._vertical_only_latched:
            return self._execute_servo()

        # Only geometry-confirmed, temporally continuous H measurements may
        # command the aircraft.  The raw stream can jump at close range.
        landing_target = self._get_stable_landing_target()
        has_target = landing_target is not None

        # 视觉丢失计时
        if has_target:
            self._has_target = True
            self._vision_lost_start = 0.0
        elif self._has_target:
            if self._vision_lost_start == 0.0:
                self._vision_lost_start = time.time()

        # 视觉丢失超时 → PX4 LAND 兜底。quick 可配置比全任务更短的
        # 重捕获窗口，避免近地坏帧驱动横向饱和速度。
        vis_loss_to = float(self._fsm.get_param(
            "landing_target_reacquire_timeout",
            self._fsm.get_param("vision_loss_timeout", 3.0),
        ))
        if self._has_target and self._vision_lost_start > 0:
            if time.time() - self._vision_lost_start > vis_loss_to:
                self._fsm.get_logger().error(
                    "PRECISION_LAND: H 目标持续丢失, 请求 PX4 自主着陆"
                )
                self._request_px4_land()
                return "COMPLETE"

        # 分阶段执行
        if self._phase == self.PHASE_SEARCH:
            return self._execute_search(elapsed, has_target)
        elif self._phase == self.PHASE_SERVO:
            return self._execute_servo(landing_target)
        return None

    # ---- Phase 1: 搜索 H 标识 ----

    def _execute_search(self, elapsed: float, has_target: bool) -> Optional[str]:
        if has_target:
            self._fsm.get_logger().info(
                "PRECISION_LAND: 检测到 H 标识, 开始伺服下降"
            )
            self._phase = self.PHASE_SERVO
            self._servo_converged = 0
            return None

        timeout = float(self._fsm.get_param("landing_search_timeout", 15.0))
        if elapsed > timeout:
            self._fsm.get_logger().error(
                "PRECISION_LAND: H 标识搜索超时, 请求 PX4 自主着陆"
            )
            self._request_px4_land()
            return "COMPLETE"
        return None

    # ---- Phase 2: 像素空间 P 控制伺服下降 ----

    def _new_servo_pid(self) -> ServoPid:
        """构造像素域伺服 PID (增益契约见 config/pid_tuning.yaml).

        死区来自 profile (各 profile 值不同), 符号沿用图像→机体标定.
        """
        return ServoPid(
            kp=float(self._fsm.get_param("servo_px_kp", 0.0025)),
            ki=float(self._fsm.get_param("servo_px_ki", 0.0005)),
            kd=float(self._fsm.get_param("servo_px_kd", 0.0003)),
            max_speed=float(self._fsm.get_param("servo_px_max_speed", 0.35)),
            deadzone=float(self._fsm.get_param("landing_servo_deadzone_px", 12.0)),
            sign_forward=float(
                self._fsm.get_param("landing_servo_y_to_forward_sign", -1.0)),
            sign_right=float(
                self._fsm.get_param("landing_servo_x_to_right_sign", 1.0)),
            integral_limit=float(self._fsm.get_param("servo_integral_limit", 0.10)),
            derivative_alpha=float(
                self._fsm.get_param("servo_derivative_alpha", 0.30)),
            integral_band=float(self._fsm.get_param("servo_integral_band", 0.0)),
        )

    def _now(self) -> float:
        """FSM 时钟秒 — PID 用它判定新测量与真实积分/微分步长."""
        return self._fsm.get_clock().now().nanoseconds / 1e9

    def _execute_servo(
        self,
        target: Optional[dict] = None,
        *,
        has_target: Optional[bool] = None,
    ) -> Optional[str]:
        """图像空间 PID 控制器: 像素误差 → 机体速度, 对心下降."""
        self._predict_target_estimator()
        pos = self._fsm.bridge.get_current_position()
        altitude = abs(self._fsm.fc.ned_to_altitude(pos.down))

        # Keep the keyword argument for existing unit-test callers. Runtime
        # code passes the already validated target from execute().
        if has_target is not None:
            target = self._get_stable_landing_target() if has_target else None

        if self._vertical_only_latched:
            descent = self._descent_speed(altitude)
            self._fsm.fc.set_velocity_body(0.0, 0.0, descent)
            return None

        if target is None:
            # 目标丢失: 通知 PID 冻结积分/清微分历史 (否则恢复时会把丢失前的
            # 误差斜率接上), 并**继续飞向最后已知位置**而非原地悬停 ——
            # PX4 precland 水平接近段的语义是"途中丢失不悬停、不判失败,
            # 到达最后目击点仍看不到才评估丢失" (precland.cpp:584-597).
            # 原来的"拒绝一帧 → 悬停 → 超时 → 接管/异地降落"正是 8/12-14 的
            # 失败链条. 纵轴保持不下降 (与权威的水平接近段一致).
            self._servo_pid.update(
                0.0, 0.0, self._now(), measurement_valid=False)
            self._fsm.fc.set_velocity_body(
                *self._lost_transit_command(altitude), 0.0)
            self._servo_converged = 0
            self._handoff_stable_frames = 0
        else:
            u = target.get("center_u", 0.0)
            v = target.get("center_v", 0.0)
            conf = target.get("confidence", 0)

            # 像素误差 (test_mission: 默认 camera 中心 320x240 = 640x480 中心)
            img_cx = float(self._fsm.get_param("camera_cx", 320.0))
            img_cy = float(self._fsm.get_param("camera_cy", 240.0))
            du = u - img_cx
            dv = v - img_cy
            pixel_error = math.hypot(du, dv)

            if self._update_vertical_only_latch(du, dv, altitude):
                descent = self._descent_speed(altitude)
                self._fsm.fc.set_velocity_body(0.0, 0.0, descent)
                return None

            # 死区内计数与近地对心锁存
            # (死区判定与 PID 内部一致; 输出一律由 PID 给出, 死区内恒为零速度)
            if pixel_error < self._servo_pid.deadzone:
                self._servo_converged += 1
                stable_frames = int(self._fsm.get_param(
                    "landing_blind_stable_frames", 10,
                ))
                latch_max_alt = float(self._fsm.get_param(
                    "landing_alignment_latch_max_altitude", 2.5,
                ))
                if (
                    not self._visual_alignment_latched
                    and altitude <= latch_max_alt
                    and self._servo_converged >= stable_frames
                ):
                    self._visual_alignment_latched = True
                    self._fsm.get_logger().info(
                        "PRECISION_LAND: 视觉连续稳定对心, 锁存近地交接许可 "
                        f"({self._servo_converged}/{stable_frames})"
                    )
            else:
                self._servo_converged = 0

            # 估计器融合 + (可选) 本帧接受判决. 放在伺服之前: 判决要影响本 tick
            # 是否据此下令, 而且观测只能喂一次 (喂两次等于重复计入测量).
            self._observe_target_for_estimator(altitude, du, dv)
            if not self._frame_accepted_by_estimator():
                # 本帧与"滤波器认为目标在哪"严重不符 (卡方门) → 不据此下令,
                # 但**也不停下**: 与"本帧没有可用测量"完全同一条处置 ——
                # 继续朝**最后已知位置**接近 (见上面 target is None 分支).
                #
                # 曾经的写法是零速悬停, 与紧邻的丢失分支自相矛盾, 也违背本项目的
                # 控制核心 (对心连续伺服、持续靠近目标). 权威同此: 被拒的测量
                # PX4 根本不发布, 消费方看到的是"没有新测量", 于水平接近段继续
                # 飞向最后目击点, 而不是停住.
                #
                # 不会永久卡住: 连续 3 次拒绝后估计器强制融合 (ArduPilot 逃逸),
                # 且有 2s 丢线重置.
                self._servo_pid.update(
                    0.0, 0.0, self._now(), measurement_valid=False)
                self._fsm.fc.set_velocity_body(
                    *self._lost_transit_command(altitude), 0.0)
                return None

            # PID 输出: 符号标定与合速度(向量模长)限幅均由 PID 承担.
            # 传测量时刻而非 FSM 时钟: 同一份缓存测量会在多个 tick 上被复用,
            # 按 tick 推进会把同一份误差重复积分数倍 (实测 ≈10.5Hz 视觉 vs 20Hz tick).
            forward, right = self._servo_pid.update(
                float(dv), float(du),
                self._fsm.vision_measurement_stamp("landing"))

            # 未通过视觉锁存时，不得穿过盲降交接高度。
            blind_alt = float(self._fsm.get_param("landing_blind_altitude", 0.5))
            fx = max(float(self._fsm.get_param("camera_fx", 400.0)), 1.0)
            fy = max(float(self._fsm.get_param("camera_fy", 400.0)), 1.0)
            # 像素→米用**镜头到平面**的距离, 不是机体原点高度: 相机装在原点
            # 下方 (真机 36.5cm), 直接用 altitude 会把地面误差放大 ~17%(2.5m)
            # 到 ~57%(1.0m). 与投放路径同一公式.
            plane_m = camera_plane_distance(
                altitude, float(self._fsm.get_param("bucket_camera_down", 0.0)))
            descent = self._scheduled_descent(
                altitude, ground_error_m(du, dv, plane_m, fx, fy),
            )
            if altitude <= blind_alt and not self._visual_alignment_latched:
                descent = 0.0

            self._fsm.fc.set_velocity_body(forward, right, descent)

            # 1Hz 日志 (与 test_mission 相同格式)
            now = time.time()
            if now - self._servo_last_log_time > 1.0:
                self._servo_last_log_time = now
                self._fsm.get_logger().info(
                    f"PRECISION_LAND: alt={altitude:.1f}m "
                    f"px_err={pixel_error:.0f}px du={du:+.0f} dv={dv:+.0f} "
                    f"vel=(fwd:{forward:.2f},r:{right:.2f},d:{descent:.2f}) "
                    f"conf={conf:.2f} converged={self._servo_converged}"
                )

        # 新圆形几何交接启用时，只有垂直锁存可进入盲降；未启用时保持旧逻辑。
        blind_alt = float(self._fsm.get_param("landing_blind_altitude", 0.5))
        may_enter_blind = (
            self._vertical_only_latched
            if self._vertical_handoff_enabled()
            else self._visual_alignment_latched
        )
        if altitude < blind_alt and may_enter_blind:
            self._fsm.get_logger().info(
                f"PRECISION_LAND: 到达盲降高度 ({altitude:.2f}m)"
            )
            self._switch_to_blind()
        return None

    def _scheduled_descent(self, altitude: float, ground_error: float) -> float:
        """近地段按水平误差调度下降速度 (ArduPilot mode.cpp:747-756).

        越对准降得越快、越偏降得越慢, 但有下限, 所以**永远不会因为偏心停在
        半空**. 只在使用慢速档的高度内生效 —— 高空横向误差本就由伺服持续
        修正, 再叠加降速只会拖长低空停留.

        调度参数用 landing_precision_target (目标水平精度, m): 它是"认为已经
        对准"的误差量级, 正好是这条律的参考误差. 它此前一直没有消费者.
        """
        nominal = self._descent_speed(altitude)
        if altitude > NEAR_GROUND_SLOWDOWN_MAX_ALT_M:
            return nominal
        return near_ground_descent_speed(
            nominal,
            ground_error,
            float(self._fsm.get_param("landing_precision_target", 0.1)),
        )

    def _vertical_handoff_enabled(self) -> bool:
        return float(self._fsm.get_param("landing_visual_handoff_altitude", 0.0)) > 0.0

    def _descent_speed(self, altitude: float) -> float:
        """离地高度三档下降速度 (共享实现见 drone_utils.landing_profile)."""
        return nominal_descent_speed(
            altitude,
            float(self._fsm.get_param("landing_descent_fast", 0.4)),
            float(self._fsm.get_param("landing_descent_medium", 0.25)),
            float(self._fsm.get_param("landing_descent_slow", 0.12)),
        )

    def _frame_accepted_by_estimator(self) -> bool:
        """本帧是否可以驱动伺服 (B2).

        ⚠️ 判据是 (输入可用 且 未被融合), 而不是"某一个轴被拒" —— 后者是逐轴的,
        只看 north 会漏掉只影响 east 的误检 (u 方向跳变), 反之亦然 (这个坑踩过
        一次, 见 test_east_axis_only_false_frame_is_still_vetoed)。

        也不能直接用 `observe()` 的返回值: 它返回 False 有两种原因 —— 输入非法
        (非有限等), 或卡方门拒绝。两者必须区分: 输入非法只是"本帧没有可用观测",
        沿用既有的置信度/跳变门即可; 只有"与滤波器预测严重不符"才该额外否决。

        默认**关闭** (`landing_servo_estimator_gate_enable=False`): 门限
        (`landing_estimator_meas_unc_tan2` / `acc_psd`) 尚未按真机探测器抖动标定,
        而"门太紧"的失效形态是**真测量被连续拒掉** → 目标看起来丢失 → 有界兜底
        (悬停 → PX4 LAND / 交飞手). 那是安全但不理想的结局, 所以不默认改行为;
        标定后把它打开即可, 机制与两侧测试都已就位.

        即便打开, 也只在估计**成熟**后才有约束力: 播种帧不过卡方门, 用它去否决
        测量等于拿一个还没有模型的估计当权威.
        """
        enabled = bool(self._fsm.get_param(
            "landing_servo_estimator_gate_enable", False))
        if not enabled or not self._target_estimator.is_mature:
            return True
        est = self._target_estimator
        return not (est.last_observation_usable
                    and not est.last_observation_accepted)

    def _note_estimator_result(self, accepted: bool) -> None:
        """记录一帧估计结果: 接受时刻 / 拒绝原因日志.

        两件事各自的理由:
          - 接受时刻: 丢线超时的基准是"距最后一次**被接受**的观测", 不是"距最后
            一条消息" —— 与权威一致 (PX4 的 _last_update 只在成功时刷新).
          - 拒绝日志: 拒绝对了=挡住误检, 拒错了=看得见却永远不接受. 没有这条日志
            事后无法区分这两种情况 (PX4 专门发布 landing_target_innovations)。
        """
        now = time.monotonic()
        if accepted:
            self._estimator_last_accept = now
            return

        residual, innov_cov = self._target_estimator.filter_north.last_innovation
        if now - self._estimator_reject_log_time >= 1.0:
            self._estimator_reject_log_time = now
            self._fsm.get_logger().warn(
                "PRECISION_LAND: 估计器拒绝该帧观测 "
                f"(north 残差={residual:+.3f}m 新息协方差={innov_cov:.4f} "
                f"累计拒绝={self._target_estimator.rejected_count})"
            )

    def _check_estimator_dropout(self) -> None:
        """长时间没有**被接受**的观测 → 显式重置估计器 (PX4 的 2s 硬复位).

        不重置的后果: 协方差被 update 收敛到极小 (P00 只降不升), 于是长丢线后
        目标真移动时滤波器会连续拒绝, 只能靠"连续 3 次强制融合"慢慢恢复 ——
        表现为"看得见却半天不接受".
        """
        if not self._target_estimator.initialized:
            return
        timeout = float(self._fsm.get_param(
            "landing_estimator_reset_timeout", 2.0))
        if timeout <= 0.0 or self._estimator_last_accept <= 0.0:
            return
        if time.monotonic() - self._estimator_last_accept > timeout:
            self._fsm.get_logger().warn(
                f"PRECISION_LAND: 估计器 {timeout:.1f}s 无被接受的观测, 重置"
            )
            self._target_estimator.reset()
            self._estimator_last_accept = 0.0

    def _predict_target_estimator(self) -> None:
        """把目标估计器推进到当前时刻 (**每个伺服 tick 都要调**).

        不调用的后果不是性能问题而是功能缺失, 见
        LandingTargetEstimator.predict 的 docstring: 速度状态恒为 0、
        P00 只降不升(重捕获门不会变松)。dt 用真实经过时间而非 tick 周期,
        所以调用节拍变化不会改变滤波效果 (过程噪声是频率无关的速率 PSD)。
        """
        now = time.monotonic()
        last = self._estimator_last_predict
        self._estimator_last_predict = now
        if last <= 0.0:
            return
        self._check_estimator_dropout()
        dt = now - last
        if dt <= 0.0:
            return
        self._target_estimator.predict(dt)

    def _observe_target_for_estimator(
        self, altitude: float, du: float, dv: float,
    ) -> bool:
        """把本帧观测喂给目标估计器, 返回"这一帧是否被接受".

        做两件事:
          1. 估计器融合 —— 观测 = 载机位置 + 机体系偏移 (旋转到 NED); 卡方门
             在这里判断本帧是否与"滤波器认为目标在哪"一致.
          2. 把融合结果锁存为 `_lost_transit` 的目标点 (仅在估计成熟后, 见
             `is_mature`: 播种帧不过门, 头几帧实质不过滤).

        本帧的接受判决由调用方通过 `_frame_accepted_by_estimator()` 读取
        (它看的是卡方门拒绝计数, 不是本方法的返回值) —— 见该方法的说明.

        符号: 与伺服同源 —— 两个标定符号取自 _new_servo_pid 用的同一组参数,
        因此本方法算出的机体系偏移与伺服收到的是同一个量 (只是单位换成了米).

        ⚠️ 单位: 传入的 du/dv 是**像素**; 先按"镜头到平面"的距离与焦距折成米.
        """
        # ⚠️ 只在**新测量**上融合: 伺服 tick 20Hz 而视觉实测 ≈10.5Hz, 同一份测量会被
        # 复用 4-7 个 tick. 每个 tick 都喂一次等于把同一份测量重复计入 ——
        # 协方差被虚假地收缩、门随之过紧 (表现为"看得见却越来越不接受").
        # 权威同此: PX4 估计器在没有新 irlock_report 时 update() 直接返回.
        stamp = self._fsm.vision_measurement_stamp("landing")
        if stamp <= self._observed_measurement_stamp:
            return False
        self._observed_measurement_stamp = stamp

        sign_forward = float(self._fsm.get_param(
            "landing_servo_y_to_forward_sign", -1.0))
        sign_right = float(self._fsm.get_param(
            "landing_servo_x_to_right_sign", 1.0))
        fx = max(float(self._fsm.get_param("camera_fx", 400.0)), 1.0)
        fy = max(float(self._fsm.get_param("camera_fy", 400.0)), 1.0)

        # ── 时间对齐 ──
        # 观测必须用**拍摄那一刻**的载机位置/航向/高度, 不是处理这一刻的:
        # 两者相差实测均值 95ms (95% 199ms), 平移误差 ≈ v·tau (0.25m/s → 2.4cm/5cm),
        # 旋转误差 ≈ |偏移|·Δψ 且方向随航向变化 (系统偏差, 卡方门抓不到).
        # 取不到时**退回处理时刻** —— 时间对齐是精度改进, 不是可用性前提.
        pose = self._fsm.pose_at(stamp)
        if pose is not None:
            north, east, down, yaw = pose
            plane_m = camera_plane_distance(
                abs(float(self._fsm.fc.ned_to_altitude(down))),
                float(self._fsm.get_param("bucket_camera_down", 0.0)))
        else:
            pos = self._fsm.bridge.get_current_position()
            north, east = float(pos.north), float(pos.east)
            # nan 默认值: 拿不到航向时 latch/command 都拒绝动作, 而不是把 yaw=0
            # 当成"机头朝北"去算出一个方向错误的指令 (与 state_drop 的读法一致).
            yaw = float(getattr(self._fsm.bridge, "_current_yaw", math.nan))
            plane_m = camera_plane_distance(
                altitude, float(self._fsm.get_param("bucket_camera_down", 0.0)))
        error_forward_m = sign_forward * float(dv) * plane_m / fy
        error_right_m = sign_right * float(du) * plane_m / fx

        accepted = self._target_estimator.observe(
            north, east, yaw,
            error_forward_m, error_right_m,
            self._target_estimator.measurement_uncertainty(altitude),
        )
        self._note_estimator_result(accepted)

        # 只在估计成熟后驱动"继续接近"的落点; 不成熟时保持上一次成熟时的落点
        # (或没有落点 → 丢失时不动作, 与原有悬停行为一致).
        filtered = self._target_estimator.position
        if filtered is not None and self._target_estimator.is_mature:
            self._lost_transit.latch_world(filtered[0], filtered[1])
        return accepted

    def _lost_transit_command(self, altitude: float) -> tuple:
        """丢失目标时的机体系横向指令 (forward, right).

        未锁存过目标、或已低于近地阈值时返回 (0, 0) —— 失败方向是不动作,
        与原来的悬停行为一致, 不会凭空产生指令. 近地不做横向机动是对齐
        ArduPilot PLND_ALT_MIN 的取舍: 离地很近时未知的横向移动风险更大.
        """
        min_altitude = float(self._fsm.get_param(
            "landing_lost_transit_min_altitude", 0.75))
        if altitude < min_altitude:
            return (0.0, 0.0)
        pos = self._fsm.bridge.get_current_position()
        yaw = float(getattr(self._fsm.bridge, "_current_yaw", math.nan))
        return self._lost_transit.command(
            float(pos.north), float(pos.east), yaw)

    def _update_vertical_only_latch(
        self,
        du: float,
        dv: float,
        altitude: float,
    ) -> bool:
        if self._vertical_only_latched:
            return True
        handoff_altitude = float(self._fsm.get_param(
            "landing_visual_handoff_altitude", 0.0,
        ))
        if handoff_altitude <= 0.0 or altitude > handoff_altitude:
            self._handoff_stable_frames = 0
            return False

        fx = max(float(self._fsm.get_param("camera_fx", 400.0)), 1.0)
        fy = max(float(self._fsm.get_param("camera_fy", 400.0)), 1.0)
        ground_error = math.hypot(du * altitude / fx, dv * altitude / fy)
        max_error = float(self._fsm.get_param(
            "landing_visual_handoff_max_ground_error_m", 0.20,
        ))
        if ground_error > max_error:
            self._handoff_stable_frames = 0
            return False

        self._handoff_stable_frames += 1
        stable_frames = int(self._fsm.get_param(
            "landing_visual_handoff_stable_frames", 10,
        ))
        if self._handoff_stable_frames < stable_frames:
            return False

        self._vertical_only_latched = True
        self._visual_alignment_latched = True
        self._fsm.get_logger().info(
            "PRECISION_LAND: 圆心几何对心锁存, 后续只垂直下降 "
            f"(error={ground_error:.2f}m, "
            f"{self._handoff_stable_frames}/{stable_frames})"
        )
        return True

    def _get_stable_landing_target(self) -> Optional[dict]:
        """Reject low-confidence or discontinuous H observations.

        YOLO gives coarse acquisition while the vision pipeline's OpenCV
        refiner supplies a geometric centre.  This guard ensures the mission
        consumes a continuous sequence rather than a different raw candidate
        on every frame.
        """
        target = self._fsm.get_landing_target()
        min_confidence = float(self._fsm.get_param(
            "landing_target_min_confidence", 0.60,
        ))
        if not target:
            return self._reject_landing_target("no_target_message")
        confidence = float(target.get("confidence", 0.0))
        if confidence < min_confidence:
            return self._reject_landing_target(
                f"low_confidence({confidence:.2f}<{min_confidence:.2f})"
            )

        center = (
            float(target.get("center_u", 0.0)),
            float(target.get("center_v", 0.0)),
        )
        if not all(math.isfinite(value) for value in center):
            return self._reject_landing_target("non_finite_center")

        if self._last_target_center is not None:
            max_jump = float(self._fsm.get_param(
                "landing_target_max_jump_px", 80.0,
            ))
            jump = math.dist(center, self._last_target_center)
            if jump > max_jump:
                return self._reject_landing_target(
                    f"center_jump({jump:.1f}px>{max_jump:.1f}px)"
                )

        self._last_target_center = center
        self._last_target_rejection_reason = None
        return target

    def _reject_landing_target(self, reason: str) -> None:
        """Log only the first sample of each rejected-target condition."""
        if reason != self._last_target_rejection_reason:
            self._fsm.get_logger().warn(
                f"PRECISION_LAND: 忽略视觉目标, reason={reason}"
            )
        self._last_target_rejection_reason = reason
        return None

    def _fallback_touchdown_confirmed(self) -> bool:
        """SITL-only ground fallback after an already-safe vertical handoff."""
        if (
            not self._vertical_only_latched
            or not bool(self._fsm.get_param("sim_mode", False))
        ):
            self._touchdown_candidate_start = 0.0
            return False

        position = self._fsm.bridge.get_current_position()
        velocity = self._fsm.bridge.get_telemetry().velocity
        altitude = abs(self._fsm.fc.ned_to_altitude(float(position.down)))
        horizontal_speed = math.hypot(float(velocity.vn), float(velocity.ve))
        vertical_speed = abs(float(velocity.vd))
        near_ground = altitude <= float(self._fsm.get_param(
            "landing_touchdown_max_altitude", 0.10,
        ))
        stationary = (
            horizontal_speed <= float(self._fsm.get_param(
                "landing_touchdown_max_horizontal_speed", 0.10,
            ))
            and vertical_speed <= float(self._fsm.get_param(
                "landing_touchdown_max_vertical_speed", 0.05,
            ))
        )
        if not (near_ground and stationary):
            self._touchdown_candidate_start = 0.0
            return False

        now = time.time()
        if self._touchdown_candidate_start == 0.0:
            self._touchdown_candidate_start = now
            return False
        return now - self._touchdown_candidate_start >= float(self._fsm.get_param(
            "landing_touchdown_hold_time", 2.0,
        ))

    # ---- Phase 3: 近地盲降 ----

    def _execute_blind(self, elapsed: float) -> Optional[str]:
        """近地低速下降, 等待 PX4 着陆检测."""
        descent = float(self._fsm.get_param("landing_blind_descent_speed", 0.15))
        self._fsm.fc.set_velocity_body(0.0, 0.0, descent)

        timeout = float(self._fsm.get_param("landing_simple_blind_timeout", 60.0))
        if elapsed > timeout:
            self._fsm.get_logger().error(
                "PRECISION_LAND: 盲降超时, 请求 PX4 自主着陆"
            )
            self._request_px4_land()
            return "COMPLETE"
        return None

    # ---- 辅助 ----

    def _request_px4_land(self):
        """兜底: 请求 PX4 自主着陆 (仅发送一次)."""
        if not self._px4_land_requested:
            self._fsm.get_logger().warn("PRECISION_LAND: 请求 PX4 自主着陆")
            self._fsm.bridge.land_at_current()
            self._px4_land_requested = True

    def _switch_to_blind(self):
        """转到盲降阶段并关闭视觉检测."""
        self._phase = self.PHASE_BLIND
        self._fsm.get_logger().info("PRECISION_LAND: Phase 3 - 盲降模式")
        self._fsm.set_vision_mode("none")

    def exit(self):
        self._fsm.set_vision_mode("none")
        self._fsm.logger.log_state("PRECISION_LAND", "COMPLETE", "降落完成")
