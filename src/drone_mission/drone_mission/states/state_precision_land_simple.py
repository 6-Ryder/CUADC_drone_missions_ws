"""
PRECISION_LAND (simple) 状态 — test_mission 方法的三阶段精准降落.

由 landing_simple_mode=True 门控启用 (quick 快速投放策略用), 主任务默认
False 不受影响. 逻辑照搬 test_mission/test_forward_recon.py 的 LAND 实现:

  Phase 1 SEARCH: 悬停搜索 H 标识, 超时 → PX4 自主着陆兜底
  Phase 2 SERVO:  图像空间 P 控制器 (像素误差 → 机体速度), 分层下降
  Phase 3 BLIND:  近地纯垂直下降, 等待 PX4 着陆检测

与主任务 PrecisionLandState 的区别 (刻意简化):
  - 无视觉对心锁存 / 备用接地确认: 接地判定只依赖 PX4 is_landed()
  - 无 H 重获: 目标丢失超过 vision_loss_timeout → 直接 PX4 LAND
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
        self._servo_last_log_time: float = 0.0
        self._px4_land_requested: bool = False

    def enter(self):
        self._fsm.logger.log_state("RETURN_TO_HOME", "PRECISION_LAND",
                                   "开始精准降落 (simple)")
        self._phase = self.PHASE_SEARCH
        self._start_time = time.time()
        self._vision_lost_start = 0.0
        self._has_target = False
        self._servo_converged = 0
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

        # 检查当前目标 (与 test_mission 相同: conf > 0.3 即有效)
        lt = self._fsm.get_landing_target()
        has_target = lt is not None and lt.get("confidence", 0) > 0.3

        # 视觉丢失计时
        if has_target:
            self._has_target = True
            self._vision_lost_start = 0.0
        elif self._has_target:
            if self._vision_lost_start == 0.0:
                self._vision_lost_start = time.time()

        # 视觉丢失超时 → PX4 LAND 兜底 (test_mission: vision_loss_timeout 3.0s)
        vis_loss_to = float(self._fsm.get_param("vision_loss_timeout", 3.0))
        if self._has_target and self._vision_lost_start > 0:
            if time.time() - self._vision_lost_start > vis_loss_to:
                self._fsm.get_logger().error(
                    "PRECISION_LAND: H 目标持续丢失, 请求 PX4 自主着陆"
                )
                self._request_px4_land()
                return "COMPLETE"

        # 分阶段执行
        elapsed = time.time() - self._start_time
        if self._phase == self.PHASE_SEARCH:
            return self._execute_search(elapsed, has_target)
        elif self._phase == self.PHASE_SERVO:
            return self._execute_servo(has_target)
        elif self._phase == self.PHASE_BLIND:
            return self._execute_blind(elapsed)
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

    def _execute_servo(self, has_target: bool) -> Optional[str]:
        """图像空间 P 控制器: 像素误差 → 机体速度, 对心下降."""
        pos = self._fsm.bridge.get_current_position()
        altitude = abs(self._fsm.fc.ned_to_altitude(pos.down))

        if not has_target or self._fsm.get_landing_target() is None:
            # 目标丢失 → 悬停等待恢复
            self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
            self._servo_converged = 0
        else:
            lt = self._fsm.get_landing_target()
            u = lt.get("center_u", 0.0)
            v = lt.get("center_v", 0.0)
            conf = lt.get("confidence", 0)

            # 像素误差 (test_mission: 默认 camera 中心 320x240 = 640x480 中心)
            img_cx = float(self._fsm.get_param("camera_cx", 320.0))
            img_cy = float(self._fsm.get_param("camera_cy", 240.0))
            du = u - img_cx
            dv = v - img_cy
            pixel_error = math.hypot(du, dv)

            # P 控制
            kp = float(self._fsm.get_param("landing_servo_kp_pixel", 0.004))
            max_vel = float(self._fsm.get_param("landing_servo_max_horizontal_vel", 0.6))
            deadzone = float(self._fsm.get_param("landing_servo_deadzone_px", 12.0))

            if pixel_error < deadzone:
                forward = 0.0
                right = 0.0
                self._servo_converged += 1
            else:
                x_sign = float(self._fsm.get_param("landing_servo_x_to_right_sign", 1.0))
                y_sign = float(self._fsm.get_param("landing_servo_y_to_forward_sign", -1.0))
                forward = max(-max_vel, min(max_vel, y_sign * kp * dv))
                right = max(-max_vel, min(max_vel, x_sign * kp * du))
                self._servo_converged = 0

            # 高度分层下降速度
            if altitude > 3.0:
                descent = float(self._fsm.get_param("landing_descent_fast", 0.4))
            elif altitude > 1.5:
                descent = float(self._fsm.get_param("landing_descent_medium", 0.25))
            else:
                descent = float(self._fsm.get_param("landing_descent_slow", 0.12))

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

        # 到达盲降高度 → Phase 3 (test_mission: landing_blind_altitude 0.5m)
        blind_alt = float(self._fsm.get_param("landing_blind_altitude", 0.5))
        if altitude < blind_alt:
            self._fsm.get_logger().info(
                f"PRECISION_LAND: 到达盲降高度 ({altitude:.2f}m)"
            )
            self._switch_to_blind()
        return None

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
