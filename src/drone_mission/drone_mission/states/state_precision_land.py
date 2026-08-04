"""
PRECISION_LAND 状态 — 视觉伺服 + H 标识检测 + 对心降落.

三阶段:
  Phase 1: 粗定位 (5m 高度, 搜索 H 标识)
  Phase 2: 视觉伺服对心下降 (5m → 1.2m), 图像空间 P 控制器
  Phase 3: 对心后的近地纯垂直 Offboard 下降, 等待接地确认

与投放区精校一致: 闭环在像素层, 不依赖世界坐标转换.
像素误差 → body 速度 → fc.set_velocity_body()

退出条件:
  - land_detected: 已着陆并上锁 → COMPLETE
  - SITL 严格近地停滞判据: 已接地并上锁 → COMPLETE
  - vision lost / timeout: 悬停并报告任务失败, 不切换 PX4 LAND
"""

import math
import time
from typing import Optional

from drone_utils.progress_monitor import ProgressMonitor

class PrecisionLandState:
    """精准降落状态处理器."""

    PHASE_SEARCH = 0     # 粗定位搜索
    PHASE_SERVO = 1      # 视觉伺服下降
    PHASE_BLIND = 2      # 盲降

    def __init__(self, fsm):
        self._fsm = fsm
        self._phase = self.PHASE_SEARCH
        self._start_time: float = 0.0
        self._vision_lost_start: float = 0.0
        self._has_landing_target: bool = False
        self._landing_done: bool = False
        self._disarm_requested: bool = False
        self._blind_start_time: float = 0.0
        self._touchdown_candidate_start: float = 0.0
        # 伺服状态
        self._servo_converged_count: int = 0
        self._visual_alignment_latched: bool = False
        self._servo_last_log_time: float = 0.0
        self._last_target_center: Optional[tuple[float, float]] = None
        self._last_target_jump_log_time: float = 0.0
        self._vision_reacquire_used: bool = False
        self._search_progress: Optional[ProgressMonitor] = None
        self._blind_progress: Optional[ProgressMonitor] = None

    def enter(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "开始精准降落")
        self._phase = self.PHASE_SEARCH
        self._start_time = time.time()
        self._vision_lost_start = 0.0
        self._has_landing_target = False
        self._landing_done = False
        self._disarm_requested = False
        self._blind_start_time = 0.0
        self._touchdown_candidate_start = 0.0
        self._servo_converged_count = 0
        self._visual_alignment_latched = False
        self._servo_last_log_time = 0.0
        self._last_target_center = None
        self._last_target_jump_log_time = 0.0
        self._vision_reacquire_used = False
        self._search_progress = self._new_search_progress()
        self._blind_progress = None

        self._fsm.set_vision_mode("landing")

        self._fsm.get_logger().info("PRECISION_LAND: Phase 1 - 搜索 H 标识")

        # 保持在 5m 悬停等待视觉检测
        self._fsm.loiter()

    def execute(self) -> Optional[str]:
        # 检查是否已着陆
        if self._disarm_requested:
            return self._finish_touchdown()

        if self._fsm.bridge.is_landed():
            self._fsm.get_logger().info("PRECISION_LAND: 已着陆")
            return self._finish_touchdown()

        if self._landing_done:
            return "COMPLETE"

        if self._phase == self.PHASE_BLIND:
            return self._execute_blind()

        # 获取降落目标
        landing_target = self._get_stable_landing_target()
        has_target = landing_target is not None

        if has_target:
            self._has_landing_target = True
            self._vision_lost_start = 0.0
        else:
            if self._has_landing_target:
                if self._vision_lost_start == 0.0:
                    self._vision_lost_start = time.time()

        # 高空视觉丢失后悬停并报告错误，不能无目标下降或切换 PX4 LAND。
        vision_lost_to = self._fsm.get_param("vision_loss_timeout", 3.0)
        if self._has_landing_target and self._vision_lost_start > 0:
            lost_duration = time.time() - self._vision_lost_start
            if lost_duration > vision_lost_to:
                if not self._vision_reacquire_used:
                    self._vision_reacquire_used = True
                    self._fsm.get_logger().warn(
                        "PRECISION_LAND: H 目标持续丢失, 清除本次锁定后重捕获一次"
                    )
                    self._reset_for_h_reacquire()
                    return None
                self._fsm.fc.stop_velocity()
                self._fsm.request_takeover(
                    "PRECISION_LAND: H target was lost after reacquire"
                )
                return None

        # 阶段逻辑
        if self._phase == self.PHASE_SEARCH:
            return self._execute_search(has_target)
        elif self._phase == self.PHASE_SERVO:
            return self._execute_servo(landing_target)
        return None

    @property
    def final_disarm_pending(self) -> bool:
        """Whether a touchdown-confirmed PX4 disarm is awaiting status."""
        return self._disarm_requested and not self._landing_done

    def _execute_search(self, has_target: bool) -> Optional[str]:
        """Phase 1: 搜索 H 标识."""
        if has_target:
            self._fsm.get_logger().info("PRECISION_LAND: 检测到 H 标识, 开始伺服下降")
            self._phase = self.PHASE_SERVO
            self._servo_converged_count = 0
            if self._search_progress is not None:
                self._search_progress.reset()
            return None

        # 没有 H 目标时不得从搜索高度盲降。仅在确实没有任何视觉
        # 进展时重新发一次相同悬停搜索目标，随后等待人工接管。
        if self._search_progress is not None:
            now = time.monotonic()
            if self._search_progress.observe(1.0, now=now):
                if self._search_progress.mark_retry(now=now):
                    self._fsm.get_logger().warn(
                        "PRECISION_LAND: H 搜索无进展, 原地重试一次"
                    )
                    self._fsm.loiter()
                else:
                    self._fsm.request_takeover(
                        "PRECISION_LAND: H target was not acquired after retry"
                    )
        return None

    def _execute_servo(self, target: Optional[dict]) -> Optional[str]:
        """Phase 2: 图像空间 P 控制器, 像素误差 → 机体速度, 向 H 对心下降.

        闭环在像素层:
          du = center_u - img_cx    (+u → 目标在画面右侧 → 机体右移)
          dv = center_v - img_cy    (+v → 目标在画面下方 → 机体后飞)
          pixel_error = sqrt(du² + dv²)

          < deadzone → 已对心, 停止水平修正, 仅下降
          > deadzone → P 控制: body_vel = kp_pixel * pixel_error
        """
        pos = self._fsm.bridge.get_current_position()
        altitude = abs(self._fsm.fc.ned_to_altitude(pos.down))  # NED down 的绝对值 = 高度
        blind_alt = float(self._fsm.get_param("landing_blind_altitude", 1.5))

        # 近地检测容易畸变。一旦视觉已连续证明对心，先在交接高度切换，
        # 不允许随后低置信度坏帧重新产生横向速度。
        if altitude <= blind_alt and self._visual_alignment_latched:
            self._fsm.get_logger().info(
                f"PRECISION_LAND: 到达盲降高度 ({altitude:.2f}m)"
            )
            self._switch_to_blind()
            return None

        if target is None:
            # 目标丢失 → 悬停, 等待恢复
            self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
            self._servo_converged_count = 0
        else:
            u = target.get("center_u", 0.0)
            v = target.get("center_v", 0.0)
            conf = target.get("confidence", 0)

            # ── 1. 像素误差 ──
            img_cx = float(self._fsm.get_param("camera_cx", 640.0))
            img_cy = float(self._fsm.get_param("camera_cy", 480.0))
            du = u - img_cx   # +du = 目标在画面右侧
            dv = v - img_cy   # +dv = 目标在画面下方
            pixel_error = math.hypot(du, dv)

            # ── 2. P 控制: 像素误差 → 机体速度 ──
            kp = float(self._fsm.get_param("landing_servo_kp_pixel", 0.004))
            max_vel = float(self._fsm.get_param("landing_servo_max_horizontal_vel", 0.6))
            deadzone = float(self._fsm.get_param("landing_servo_deadzone_px", 12.0))

            if pixel_error < deadzone:
                forward = 0.0
                right = 0.0
                self._servo_converged_count += 1
                stable_frames = int(
                    self._fsm.get_param("landing_blind_stable_frames", 10)
                )
                latch_max_alt = float(
                    self._fsm.get_param(
                        "landing_alignment_latch_max_altitude", 2.5,
                    )
                )
                if (
                    not self._visual_alignment_latched
                    and altitude <= latch_max_alt
                    and self._servo_converged_count >= stable_frames
                ):
                    self._visual_alignment_latched = True
                    self._fsm.get_logger().info(
                        "PRECISION_LAND: 视觉连续稳定对心, 锁存近地交接许可 "
                        f"({self._servo_converged_count}/{stable_frames})"
                    )
            else:
                # 符号: x_to_right_sign 和 y_to_forward_sign (与投放精校一致)
                x_sign = float(self._fsm.get_param("landing_servo_x_to_right_sign", 1.0))
                y_sign = float(self._fsm.get_param("landing_servo_y_to_forward_sign", -1.0))

                forward = max(-max_vel, min(max_vel, y_sign * kp * dv))
                right = max(-max_vel, min(max_vel, x_sign * kp * du))
                self._servo_converged_count = 0

            # ── 3. 高度分层下降速度 ──
            if altitude > 3.0:
                descent = float(self._fsm.get_param("landing_descent_fast", 0.4))
            elif altitude > 1.5:
                descent = float(self._fsm.get_param("landing_descent_medium", 0.25))
            else:
                descent = float(self._fsm.get_param("landing_descent_slow", 0.12))

            self._fsm.fc.set_velocity_body(forward, right, descent)

            # ── 4. 定期日志 (1Hz) ──
            now = time.time()
            if now - self._servo_last_log_time > 1.0:
                self._servo_last_log_time = now
                self._fsm.get_logger().info(
                    f"PRECISION_LAND: alt={altitude:.1f}m "
                    f"px_err={pixel_error:.0f}px du={du:+.0f} dv={dv:+.0f} "
                    f"body_vel=(fwd:{forward:.2f},r:{right:.2f},d:{descent:.2f}) "
                    f"conf={conf:.2f} dead={self._servo_converged_count}"
                )

        # ── 下降到盲降高度 → Phase 3 ──
        if altitude <= blind_alt and self._visual_alignment_latched:
            self._fsm.get_logger().info(
                f"PRECISION_LAND: 到达盲降高度 ({altitude:.2f}m)"
            )
            self._switch_to_blind()

        return None

    def _get_stable_landing_target(self) -> Optional[dict]:
        """Reject one-frame H detections that cannot be reached between control ticks."""
        target = self._fsm.get_landing_target()
        min_confidence = float(
            self._fsm.get_param("landing_target_min_confidence", 0.60)
        )
        if not target or target.get("confidence", 0.0) < min_confidence:
            return None

        center = (
            float(target.get("center_u", 0.0)),
            float(target.get("center_v", 0.0)),
        )
        if self._last_target_center is not None:
            jump_px = math.dist(center, self._last_target_center)
            max_jump_px = float(
                self._fsm.get_param("landing_target_max_jump_px", 80.0)
            )
            if jump_px > max_jump_px:
                now = time.time()
                if now - self._last_target_jump_log_time > 1.0:
                    self._last_target_jump_log_time = now
                    self._fsm.get_logger().warn(
                        "PRECISION_LAND: 忽略 H 目标单帧跳变 "
                        f"({jump_px:.0f}px > {max_jump_px:.0f}px)"
                    )
                return None

        self._last_target_center = center
        return target

    def _execute_blind(self) -> Optional[str]:
        """Phase 3: centered near-ground Offboard descent until touchdown."""
        descent = float(
            self._fsm.get_param("landing_blind_descent_speed", 0.30)
        )
        self._fsm.fc.set_velocity_body(0.0, 0.0, descent)

        if self._fallback_touchdown_confirmed():
            self._fsm.get_logger().info(
                "PRECISION_LAND: 近地静止持续满足, 备用确认接地"
            )
            return self._finish_touchdown()

        if self._blind_progress is not None:
            pos = self._fsm.bridge.get_current_position()
            altitude = abs(self._fsm.fc.ned_to_altitude(pos.down))
            now = time.monotonic()
            if self._blind_progress.observe(altitude, now=now):
                if self._blind_progress.mark_retry(now=now):
                    self._fsm.get_logger().warn(
                        "PRECISION_LAND: 盲降无进展, 重发垂直下降一次"
                    )
                    self._fsm.fc.set_velocity_body(0.0, 0.0, descent)
                else:
                    self._fsm.fc.stop_velocity()
                    self._fsm.request_takeover(
                        "PRECISION_LAND: blind descent made no progress after retry"
                    )

        return None

    def _fallback_touchdown_confirmed(self) -> bool:
        """Use local stillness only after visual alignment and blind descent."""
        if not self._visual_alignment_latched or self._phase != self.PHASE_BLIND:
            self._touchdown_candidate_start = 0.0
            return False

        position = self._fsm.bridge.get_current_position()
        velocity = self._fsm.bridge.get_telemetry().velocity
        altitude = abs(self._fsm.fc.ned_to_altitude(float(position.down)))
        horizontal_speed = math.hypot(float(velocity.vn), float(velocity.ve))
        vertical_speed = abs(float(velocity.vd))

        near_ground = altitude <= float(
            self._fsm.get_param("landing_touchdown_max_altitude", 0.10)
        )
        stationary = (
            horizontal_speed <= float(
                self._fsm.get_param("landing_touchdown_max_horizontal_speed", 0.10)
            )
            and vertical_speed <= float(
                self._fsm.get_param("landing_touchdown_max_vertical_speed", 0.05)
            )
        )

        now = time.time()
        if near_ground and stationary:
            if self._touchdown_candidate_start == 0.0:
                self._touchdown_candidate_start = now
                return False
            hold_time = float(
                self._fsm.get_param("landing_touchdown_hold_time", 2.0)
            )
            return now - self._touchdown_candidate_start >= hold_time

        self._touchdown_candidate_start = 0.0
        return False

    def _finish_touchdown(self) -> Optional[str]:
        """Hold until PX4 confirms the final ground disarm, then complete."""
        self._fsm.fc.stop_velocity()
        if not self._disarm_requested:
            self._fsm.bridge.force_disarm()
            self._disarm_requested = True
            return None

        if self._fsm.bridge.is_armed():
            return None

        self._landing_done = True
        return "COMPLETE"

    def _switch_to_blind(self):
        """切换到盲降模式."""
        self._phase = self.PHASE_BLIND
        self._blind_start_time = time.time()
        self._touchdown_candidate_start = 0.0
        self._blind_progress = self._new_blind_progress()
        self._fsm.get_logger().info("PRECISION_LAND: Phase 3 - 盲降模式")
        self._fsm.set_vision_mode("none")

    def exit(self):
        self._fsm.set_vision_mode("none")
        self._fsm.logger.log_state("PRECISION_LAND", "COMPLETE", "降落完成")

    def _reset_for_h_reacquire(self) -> None:
        """Clear only the local H lock before retrying the same landing search."""
        self._phase = self.PHASE_SEARCH
        self._has_landing_target = False
        self._vision_lost_start = 0.0
        self._servo_converged_count = 0
        self._visual_alignment_latched = False
        self._last_target_center = None
        self._start_time = time.time()
        self._search_progress = self._new_search_progress()
        self._fsm.loiter()

    def _new_search_progress(self) -> ProgressMonitor:
        return ProgressMonitor(
            window_s=float(self._fsm.get_param("landing_search_progress_window", 15.0)),
            min_improvement=0.5,
        )

    def _new_blind_progress(self) -> ProgressMonitor:
        return ProgressMonitor(
            window_s=float(self._fsm.get_param("navigation_progress_window", 30.0)),
            min_improvement=float(
                self._fsm.get_param("navigation_progress_min_improvement", 0.25)
            ),
        )
