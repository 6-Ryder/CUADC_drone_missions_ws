"""
PRECISION_LAND 状态 — 视觉伺服 + H 标识检测 + 对心降落.

三阶段:
  Phase 1: 粗定位 (5m 高度, 搜索 H 标识)
  Phase 2: 视觉伺服对心下降 (5m → 0.5m), 图像空间 P 控制器
  Phase 3: 近地下降 (< 0.5m, 由 PX4 着陆检测确认)

与投放区精校一致: 闭环在像素层, 不依赖世界坐标转换.
像素误差 → body 速度 → fc.set_velocity_body()

退出条件:
  - land_detected: 已着陆 → COMPLETE
  - vision lost > 3s: 请求 PX4 LAND 并报告任务失败
  - timeout: 请求 PX4 LAND, 等待真实着陆反馈
"""

import math
import time
from typing import Optional


class PrecisionLandState:
    """精准降落状态处理器."""

    PHASE_SEARCH = 0     # 粗定位搜索
    PHASE_SERVO = 1      # 视觉伺服下降
    PHASE_BLIND = 2      # 盲降

    def __init__(self, fsm):
        self._fsm = fsm
        self._phase: int = self.PHASE_SEARCH
        self._start_time: float = 0.0
        self._vision_lost_start: float = 0.0
        self._has_landing_target: bool = False
        self._landing_done: bool = False
        self._land_command_sent: bool = False
        # 伺服状态
        self._servo_converged_count: int = 0
        self._servo_last_log_time: float = 0.0

    def enter(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "开始精准降落")
        self._phase = self.PHASE_SEARCH
        self._start_time = time.time()
        self._vision_lost_start = 0.0
        self._has_landing_target = False
        self._landing_done = False
        self._land_command_sent = False
        self._servo_converged_count = 0
        self._servo_last_log_time = 0.0

        self._fsm.set_vision_mode("landing")

        self._fsm.get_logger().info("PRECISION_LAND: Phase 1 - 搜索 H 标识")

        # 保持在 5m 悬停等待视觉检测
        self._fsm.loiter()

    def execute(self) -> Optional[str]:
        # 检查是否已着陆
        if self._fsm.bridge.is_landed():
            self._fsm.get_logger().info("PRECISION_LAND: 已着陆")
            return "COMPLETE"

        if self._landing_done:
            return "COMPLETE"

        # 获取降落目标
        landing_target = self._fsm.get_landing_target()
        has_target = landing_target and landing_target.get("confidence", 0) > 0.3

        if has_target:
            self._has_landing_target = True
            self._vision_lost_start = 0.0
        else:
            if self._has_landing_target:
                if self._vision_lost_start == 0.0:
                    self._vision_lost_start = time.time()

        # 视觉丢失超时后交给 PX4 LAND，不能从高空继续无目标下降。
        vision_lost_to = self._fsm.get_param("vision_loss_timeout", 3.0)
        if self._has_landing_target and self._vision_lost_start > 0:
            lost_duration = time.time() - self._vision_lost_start
            if lost_duration > vision_lost_to:
                self._fsm.get_logger().error("PRECISION_LAND: H 目标持续丢失")
                self._request_px4_land()
                return "ERROR"

        # 阶段逻辑
        if self._phase == self.PHASE_SEARCH:
            return self._execute_search(has_target)
        elif self._phase == self.PHASE_SERVO:
            return self._execute_servo(has_target)
        elif self._phase == self.PHASE_BLIND:
            return self._execute_blind()

        return None

    def _execute_search(self, has_target: bool) -> Optional[str]:
        """Phase 1: 搜索 H 标识."""
        if has_target:
            self._fsm.get_logger().info("PRECISION_LAND: 检测到 H 标识, 开始伺服下降")
            self._phase = self.PHASE_SERVO
            self._servo_converged_count = 0
            return None

        # 没有 H 目标时不得从搜索高度盲降。
        elapsed = time.time() - self._start_time
        if elapsed > 15.0:
            self._fsm.get_logger().error("PRECISION_LAND: H 标识搜索超时")
            self._request_px4_land()
            return "ERROR"
        return None

    def _execute_servo(self, has_target: bool) -> Optional[str]:
        """Phase 2: 图像空间 P 控制器, 像素误差 → 机体速度, 向 H 对心下降.

        闭环在像素层:
          du = center_u - img_cx    (+u → 目标在画面右侧 → 机体右移)
          dv = center_v - img_cy    (+v → 目标在画面下方 → 机体后飞)
          pixel_error = sqrt(du² + dv²)

          < deadzone → 已对心, 停止水平修正, 仅下降
          > deadzone → P 控制: body_vel = kp_pixel * pixel_error
        """
        pos = self._fsm.bridge.get_current_position()
        altitude = abs(pos.down)  # NED down 的绝对值 = 高度

        if not has_target:
            # 目标丢失 → 悬停, 等待恢复
            self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
            self._servo_converged_count = 0
        else:
            target = self._fsm.get_landing_target()
            if target is None:
                self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
                self._servo_converged_count = 0
                return None
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
        blind_alt = float(self._fsm.get_param("landing_blind_altitude", 0.5))
        if altitude < blind_alt:
            self._fsm.get_logger().info(
                f"PRECISION_LAND: 到达盲降高度 ({altitude:.2f}m)"
            )
            self._switch_to_blind()

        return None

    def _execute_blind(self) -> Optional[str]:
        """Phase 3: near-ground descent while waiting for PX4 land detection."""
        self._fsm.fc.set_velocity_body(0.0, 0.0, 0.15)

        elapsed = time.time() - self._start_time
        if elapsed > 60.0:
            self._request_px4_land()

        return None

    def _request_px4_land(self):
        if not self._land_command_sent:
            self._fsm.get_logger().warn("PRECISION_LAND: 请求 PX4 自主着陆")
            self._fsm.bridge.land_at_current()
            self._land_command_sent = True

    def _switch_to_blind(self):
        """切换到盲降模式."""
        self._phase = self.PHASE_BLIND
        self._fsm.get_logger().info("PRECISION_LAND: Phase 3 - 盲降模式")
        self._fsm.set_vision_mode("none")

    def exit(self):
        self._fsm.set_vision_mode("none")
        self._fsm.logger.log_state("PRECISION_LAND", "COMPLETE", "降落完成")
