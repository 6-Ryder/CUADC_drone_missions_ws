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
        # 保持在 5m 悬停等待视觉检测
        self._fsm.loiter()

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
            # 目标丢失 → 悬停等待恢复; 通知 PID 冻结积分/清微分历史,
            # 否则恢复时会把丢失前的误差斜率接上
            self._servo_pid.update(
                0.0, 0.0, self._now(), measurement_valid=False)
            self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
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

            # PID 输出: 符号标定与合速度(向量模长)限幅均由 PID 承担
            forward, right = self._servo_pid.update(
                float(dv), float(du), self._now())

            # 未通过视觉锁存时，不得穿过盲降交接高度。
            blind_alt = float(self._fsm.get_param("landing_blind_altitude", 0.5))
            descent = self._descent_speed(altitude)
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

    def _vertical_handoff_enabled(self) -> bool:
        return float(self._fsm.get_param("landing_visual_handoff_altitude", 0.0)) > 0.0

    def _descent_speed(self, altitude: float) -> float:
        if altitude > 3.0:
            return float(self._fsm.get_param("landing_descent_fast", 0.4))
        if altitude > 1.5:
            return float(self._fsm.get_param("landing_descent_medium", 0.25))
        return float(self._fsm.get_param("landing_descent_slow", 0.12))

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
