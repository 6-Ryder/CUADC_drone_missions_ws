"""
DROP 状态 — 降高至投放高度 + 触发舵机投放 + 确认.

子状态:
  DESCEND_TO_DROP: 从巡航高度降到投放高度 (2m)
  EXECUTE_DROP:    触发投放, 等待确认
  ASCEND:          回升至巡航高度

投放完成后:
  - 投放次数 < 2: 回到 SELECT_TARGET (选另一个)
  - 投放次数 = 2: TRANSIT_TO_RECON_ZONE
"""

import math
import time
from typing import Optional

from drone_mission.drop_visual_aligner import DropVisualAligner
from drone_mission.precision_align_controller import (
    PrecisionAlignController,
    PrecisionAlignResult,
)
from drone_utils.progress_monitor import ProgressMonitor


class DropState:
    """
    投放状态聚合 (DESCEND → DROP → ASCEND).

    内部有三阶段:
      phase 0: 降高
      phase 1: 投放
      phase 2: 回升
    """

    def __init__(self, fsm):
        self._fsm = fsm
        self._phase: int = 0
        self._start_time: float = 0.0
        self._drop_executed: bool = False
        self._drop_success: bool = False
        self._sub_state: str = "DESCEND_TO_DROP"
        self._last_descend_log_time: float = 0.0
        self._visual_aligner = DropVisualAligner(fsm)
        self._precision_aligner = PrecisionAlignController(fsm)
        self._descend_retry_count: int = 0
        self._descend_best_pixel_error: float = float("inf")
        self._precision_align_failed: bool = False
        self._final_descent_started: bool = False
        self._descent_ready_latched: bool = False
        self._descend_stall_count: int = 0
        self._descent_initial_error: float = 0.0
        self._descent_progress: Optional[ProgressMonitor] = None
        self._ascend_progress: Optional[ProgressMonitor] = None
        self._alignment_progress: Optional[ProgressMonitor] = None
        # 快速模式边伺服边下降: 连续精校模式会把同一个目标锁保持至投放许可。
        self._servo_descend_active: bool = False
        # 对心满足后的投放许可宽限截止时刻: 短暂丢帧不撤销已满足的对心许可
        self._centered_latch_until: float = 0.0

    # ---- 子状态切换 ----

    def set_sub_state(self, sub_state: str):
        self._sub_state = sub_state
        if sub_state == "DESCEND_TO_DROP":
            self._phase = 0
        elif sub_state == "EXECUTE_DROP":
            self._phase = 1
        elif sub_state == "ASCEND":
            self._phase = 2

    # ---- enter / execute / exit ----

    def enter(self):
        self._start_time = time.time()
        self._drop_executed = False
        self._drop_success = False
        self._last_descend_log_time = 0.0
        self._descend_retry_count = 0
        self._descend_best_pixel_error = float("inf")
        self._precision_align_failed = False
        self._final_descent_started = False
        self._descent_ready_latched = False
        self._descend_stall_count = 0
        self._descent_initial_error = 0.0
        self._descent_progress = None
        self._ascend_progress = None
        self._alignment_progress = None
        self._servo_descend_active = False
        self._centered_latch_until = 0.0
        self._visual_aligner.reset()
        self._precision_aligner.reset()
        # 停滞检测: pixel_error 在低值附近振荡但无法收敛 → 提前触发最终下降
        self._stall_start_time: float = 0.0
        self._stall_ref_error: float = float("inf")

        if self._sub_state == "DESCEND_TO_DROP":
            self._enter_descend()
        elif self._sub_state == "EXECUTE_DROP":
            self._enter_drop()
        elif self._sub_state == "ASCEND":
            self._enter_ascend()

    def _enter_descend(self):
        drop_h = self._fsm.get_param("drop_descend_height", 2.0)
        self._fsm.get_logger().info(f"DESCEND: 降高至 {drop_h}m")
        self._precision_aligner.reset(self._selected_memory_xy())

        pos = self._fsm.bridge.get_current_position()
        self._descent_initial_error = abs(pos.down - self._fsm.fc.altitude_to_ned(drop_h))
        self._descent_progress = self._new_navigation_progress()
        self._alignment_progress = self._new_alignment_progress()

        if self._fsm.get_param("quick_drop_mode", False):
            # 快速模式: 边伺服边下降 — 不发纯垂直 goto, 下降中像素 P 控制水平对心
            self._servo_descend_active = True
            self._fsm.get_logger().info(
                "DESCEND: 快速模式边伺服边下降 "
                f"(fast={float(self._fsm.get_param('drop_servo_descend_fast', 0.5)):.2f}, "
                f"slow={float(self._fsm.get_param('drop_servo_descend_slow', 0.3)):.2f} m/s)"
            )
            return

        # 保持水平位置, 降高
        self._fsm.fc.goto_ned(
            pos.north, pos.east, self._fsm.fc.altitude_to_ned(drop_h),
            speed=float(self._fsm.get_param("drop_descend_speed", 0.5)),
        )

    def _enter_drop(self):
        self._fsm.get_logger().info("DROP: 执行投放")
        self._fsm.logger.log_state("DESCEND_TO_DROP", "EXECUTE_DROP", "开始投放")

        # 调用舵机投放服务
        cyl_id = (
            self._fsm._current_drop_cyl_id
            if hasattr(self._fsm, "_current_drop_cyl_id")
            else -1
        )
        self._fsm.start_drop_service(cyl_id)
        self._drop_executed = True
        self._fsm.get_logger().info(f"DROP: 已发送投放请求 cyl={cyl_id}")

    def _enter_ascend(self):
        self._fsm.logger.log_state("EXECUTE_DROP", "ASCEND", "回升")
        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        self._fsm.get_logger().info(f"ASCEND: 回升至巡航高度 {takeoff_h}m")

        pos = self._fsm.bridge.get_current_position()
        self._fsm.fc.goto_ned(pos.north, pos.east, self._fsm.fc.altitude_to_ned(takeoff_h), speed=1.0)
        self._ascend_progress = self._new_navigation_progress()

    def execute(self) -> Optional[str]:
        if self._sub_state == "DESCEND_TO_DROP":
            return self._execute_descend()
        elif self._sub_state == "EXECUTE_DROP":
            return self._execute_drop()
        elif self._sub_state == "ASCEND":
            return self._execute_ascend()

        return None

    def _execute_descend(self) -> Optional[str]:
        drop_h = self._active_drop_height()
        pos = self._fsm.bridge.get_current_position()
        elapsed = time.time() - self._start_time

        # 跟踪梯度下降进展
        status = self._current_align_status()
        if status.pixel_error < self._descend_best_pixel_error:
            self._descend_best_pixel_error = status.pixel_error

        height_error = abs(pos.down - self._fsm.fc.altitude_to_ned(drop_h))
        descent_ready = self._check_descent_ready(height_error)

        # 快速模式: 边伺服边下降 — 到位前每 tick 像素 P 控制水平 + 垂直下降
        if self._servo_descend_active:
            if not descent_ready:
                if self._continuous_drop_servo_enabled():
                    self._continuous_servo_descend_tick(pos.down)
                else:
                    self._servo_descend_tick(pos.down)
                return None
            self._servo_descend_active = False
            if self._continuous_drop_servo_enabled():
                self._fsm.get_logger().info(
                    "DESCEND: 连续地面误差伺服到达投放高度, 保持目标锁进入投放确认"
                )
            else:
                self._fsm.fc.stop_velocity()
                self._fsm.get_logger().info(
                    "DESCEND: 边伺服边下降到位, 悬停复查对心"
                )

        if not descent_ready and self._descent_progress is not None:
            now = time.monotonic()
            if self._descent_progress.observe(height_error, now=now):
                if self._descent_progress.mark_retry(now=now):
                    self._fsm.get_logger().warn(
                        "DESCEND: 高度无进展, 原地重发同一降高目标一次"
                    )
                    self._command_descend_target(drop_h)
                else:
                    self._takeover("DESCEND: altitude made no progress after retry")
                    return None
        if self._precision_enabled() and not descent_ready:
            self._warn_descend_throttled(
                "DESCEND: 等待下降到位 "
                f"height_error={height_error:.2f}m stall={self._descend_stall_count}"
            )
            return None

        if self._precision_enabled() and descent_ready:
            if self._drop_target_is_centered():
                if self._start_final_descent_if_needed():
                    return None
                self._fsm.get_logger().info("DESCEND: 到达投放高度且目标居中")
                return "EXECUTE_DROP"

            # ── 停滞检测: pixel_error 在低值振荡但无法收敛 → 提前触发最终下降 ──
            # 默认 40 = 精校段窗口 35 + 5 兜底: 25-35px 的合法徘徊不再误触发停滞.
            stall_threshold = float(
                self._fsm.get_param("drop_align_stall_threshold", 40.0)
            )
            stall_stable_band = float(
                self._fsm.get_param("drop_align_stall_stable_band", 5.0)
            )
            stall_duration = float(
                self._fsm.get_param("drop_align_stall_duration", 2.0)
            )
            if (math.isfinite(status.pixel_error)
                    and status.pixel_error < stall_threshold
                    and not self._final_descent_started):
                now_stall = time.time()
                if self._stall_start_time == 0.0:
                    self._stall_start_time = now_stall
                    self._stall_ref_error = status.pixel_error
                elif abs(status.pixel_error - self._stall_ref_error) > stall_stable_band:
                    self._stall_start_time = now_stall
                    self._stall_ref_error = status.pixel_error
                elif now_stall - self._stall_start_time > stall_duration:
                    self._fsm.get_logger().info(
                        f"DESCEND: pixel_error={status.pixel_error:.1f}px 停滞 "
                        f"{now_stall - self._stall_start_time:.1f}s, 提前触发最终下降"
                    )
                    self._stall_start_time = 0.0
                    self._stall_ref_error = float("inf")
                    if self._start_final_descent_if_needed():
                        return None
                    # 已在最终下降阶段 → 直接投
                    self._fsm.get_logger().info("DESCEND: 最终下降停滞, 直接执行投放")
                    return "EXECUTE_DROP"
            elif self._final_descent_started:
                # 已在最终下降, 停滞 → 直接投放
                now_stall = time.time()
                if self._stall_start_time == 0.0:
                    self._stall_start_time = now_stall
                    self._stall_ref_error = status.pixel_error
                elif abs(status.pixel_error - self._stall_ref_error) > stall_stable_band:
                    self._stall_start_time = now_stall
                    self._stall_ref_error = status.pixel_error
                elif now_stall - self._stall_start_time > stall_duration:
                    self._fsm.get_logger().info(
                        "DESCEND: 最终下降停滞, 直接执行投放 "
                        f"pixel_error={status.pixel_error:.1f}px"
                    )
                    self._stall_start_time = 0.0
                    self._stall_ref_error = float("inf")
                    return "EXECUTE_DROP"
            else:
                self._stall_start_time = 0.0
                self._stall_ref_error = float("inf")
            # ── 停滞检测结束 ──

            if self._alignment_stalled(status):
                if self._retry_precision_align_in_place(drop_h, "低空精校超时"):
                    return None

                if self._quick_fallback_allow_drop(status):
                    self._fsm.get_logger().warn(
                        "DESCEND: 低空精校超时重试耗尽, quick 兜底直接投放 "
                        f"source={status.source} pixel_error={status.pixel_error:.1f}px"
                    )
                    return "EXECUTE_DROP"

                self._fsm.get_logger().error(
                    "DESCEND: 低空精校重试耗尽 "
                    f"source={status.source} pixel_error={status.pixel_error:.1f}px "
                    f"smooth=({status.smooth_dx_norm:+.4f},{status.smooth_dy_norm:+.4f}) "
                    f"retries={self._descend_retry_count}"
                )
                self._takeover("DESCEND: precision alignment retry exhausted")
                return None

            if self._nudge_to_visible_drop_target(
                drop_h,
                altitude_m=self._fsm.fc.ned_to_altitude(pos.down),
            ):
                return None
            if self._precision_align_failed:
                self._precision_align_failed = False
                self._fsm.get_logger().error("DESCEND: 精校目标丢失重试耗尽")
                self._takeover("DESCEND: precision alignment target lost after retry")
                return None
            self._warn_descend_throttled("DESCEND: 低空精校未取得可用目标, 继续等待复查")

        if descent_ready and not self._precision_enabled():
            if self._drop_target_is_centered():
                if self._precision_enabled() and self._start_final_descent_if_needed():
                    return None
                self._fsm.get_logger().info("DESCEND: 到达投放高度且目标居中")
                return "EXECUTE_DROP"

            if self._nudge_to_visible_drop_target(drop_h):
                return None
            if self._precision_align_failed:
                self._precision_align_failed = False
                self._fsm.get_logger().error("DESCEND: 精校目标丢失重试耗尽")
                self._takeover("DESCEND: precision alignment target lost after retry")
                return None
            self._warn_descend_throttled("DESCEND: 到达投放高度但目标未居中/不可见, 继续等待复查")

        legacy_recheck_timeout = 15.0
        if not self._precision_enabled() and elapsed > legacy_recheck_timeout:
            if self._drop_target_is_centered():
                self._fsm.get_logger().warn("DESCEND: 复查超时但投放许可通过, 执行投放")
                return "EXECUTE_DROP"
            if self._descend_retry_count < 1:
                self._descend_retry_count += 1
                self._visual_aligner.reset()
                self._start_time = time.time()
                self._command_descend_target(drop_h)
                self._fsm.get_logger().warn(
                    "DESCEND: 复查无结果, 原地重试同一低空校准一次"
                )
                return None
            self._fsm.get_logger().error("DESCEND: 复查超时且投放许可失败")
            self._takeover("DESCEND: drop alignment was not confirmed after retry")
            return None

        return None

    def _selected_memory_xy(self):
        selected_positions = getattr(self._fsm.cylinder_selector, "_selected_positions", [])
        if not selected_positions:
            return None
        return selected_positions[-1]

    def _servo_descend_tick(self, down: float) -> None:
        """快速模式边伺服边下降单 tick: 像素 P 控制水平对心 + 分层垂直速度.

        目标选择: quick 只追蓝底标识, 取置信度达标且像素误差最小者
        (与降落区伺服同方法: 连续像素 P 控制, 非步进脉冲).
        """
        altitude = float(self._fsm.fc.ned_to_altitude(down))

        markers = (
            self._fsm.get_markers()
            if hasattr(self._fsm, "get_markers")
            else []
        )
        min_conf = float(self._fsm.get_param("precision_align_min_confidence", 0.25))
        cx = float(self._fsm.get_param("camera_cx", 640.0))
        cy = float(self._fsm.get_param("camera_cy", 480.0))

        target = None
        best_err = float("inf")
        for m in markers:
            conf = float(
                getattr(m, "confidence", 0.0)
                if not isinstance(m, dict)
                else m.get("confidence", 0.0)
            )
            if conf < min_conf:
                continue
            u = float(
                getattr(m, "center_u", cx)
                if not isinstance(m, dict)
                else m.get("center_u", cx)
            )
            v = float(
                getattr(m, "center_v", cy)
                if not isinstance(m, dict)
                else m.get("center_v", cy)
            )
            err = math.hypot(u - cx, v - cy)
            if err < best_err:
                best_err = err
                target = (u, v)

        if target is None:
            # 目标丢失: 悬停等待恢复 (与降落区伺服一致)
            self._fsm.fc.set_velocity_body(0.0, 0.0, 0.0)
            return

        u, v = target
        du = u - cx
        dv = v - cy

        # 像素 P 控制 (复用降落区伺服参数, quick 文件已配置同值)
        kp = float(self._fsm.get_param("landing_servo_kp_pixel", 0.004))
        max_vel = float(self._fsm.get_param("landing_servo_max_horizontal_vel", 0.6))
        deadzone = float(self._fsm.get_param("landing_servo_deadzone_px", 12.0))
        x_sign = float(self._fsm.get_param("landing_servo_x_to_right_sign", 1.0))
        y_sign = float(self._fsm.get_param("landing_servo_y_to_forward_sign", -1.0))

        if best_err < deadzone:
            forward, right = 0.0, 0.0
        else:
            forward = max(-max_vel, min(max_vel, y_sign * kp * dv))
            right = max(-max_vel, min(max_vel, x_sign * kp * du))

        # 分层垂直速度: 高空快降, 近投放高度减速保对心稳定
        if altitude > 3.0:
            descent = float(self._fsm.get_param("drop_servo_descend_fast", 0.5))
        else:
            descent = float(self._fsm.get_param("drop_servo_descend_slow", 0.3))

        self._fsm.fc.set_velocity_body(forward, right, descent)

        # 1Hz 日志
        now = time.time()
        if now - self._last_descend_log_time >= 1.0:
            self._last_descend_log_time = now
            self._fsm.get_logger().info(
                f"DESCEND: servo alt={altitude:.1f}m px_err={best_err:.0f}px "
                f"du={du:+.0f} dv={dv:+.0f} "
                f"vel=(fwd:{forward:.2f},r:{right:.2f},d:{descent:.2f})"
            )

    def _precision_enabled(self) -> bool:
        return bool(self._fsm.get_param("precision_align_enable", True))

    def _align_timeout(self) -> float:
        if self._precision_enabled():
            return float(self._fsm.get_param("precision_align_timeout", 32.0))
        return float(self._fsm.get_param("drop_visual_align_timeout", 18.0))

    def _drop_height_tolerance(self) -> float:
        return float(self._fsm.get_param("precision_align_drop_height_tolerance", 0.30))

    def _check_descent_ready(self, height_error: float) -> bool:
        """
        检查是否已满足开始精校的条件.

        三阶段判断:
          1. 快速通道: height_error < tolerance → 立即通过
             (最终下降阶段禁用 — 无人机可能快速穿过目标高度,
             此时精校介入会导致画面剧烈跳动)
          2. 最小下降时间门控: 下降命令发出后前 N 秒禁止停滞判定,
             避免 PX4 启动延迟期的 vd≈0 被误判为"已到位"
          3. 降速停滞检测: 垂直速度持续归零 + 已明显降过一段高度 →
             SITL 稳态偏差导致无法到达绝对高度, 以物理实际到达为准
        """
        if self._descent_ready_latched:
            return True

        # 快速通道: 最终下降阶段禁用, 强制走停滞检测确保无人机停稳
        if not self._final_descent_started:
            if height_error < self._drop_height_tolerance():
                self._descend_stall_count = 0
                self._descent_ready_latched = True
                return True

        # 最小下降时间门控: 0.5m/s 降速下 5m→2m 约 6s,
        # 前 2.5s 内 vd≈0 属于 PX4 启动延迟, 不是停滞
        elapsed = time.time() - self._start_time
        min_descent_time = 2.5  # 必须尝试下降至少 2.5s
        if elapsed < min_descent_time:
            self._descend_stall_count = 0
            return False

        # 必须有明显的下降运动证据: 初始误差的 25% 或至少 0.3m
        # (适配主下降 5.6m→2.8m 和最终下降 2.8m→1.5m 两种场景)
        min_height_progress = max(0.3, self._descent_initial_error * 0.25)
        if self._descent_initial_error - height_error < min_height_progress:
            self._descend_stall_count = 0
            return False

        # 降速停滞检测
        try:
            telemetry = self._fsm.bridge.get_telemetry()
            vd = float(telemetry.velocity.vd)
        except Exception:
            vd = 999.0

        stall_vd_threshold = 0.08  # m/s
        stall_ticks_required = 15  # 20Hz 下约 0.75s

        if abs(vd) < stall_vd_threshold:
            self._descend_stall_count += 1
        else:
            self._descend_stall_count = max(0, self._descend_stall_count - 1)

        if self._descend_stall_count >= stall_ticks_required:
            # 仅在首次判定到位时记录一次, 避免每 tick 刷屏
            if self._descend_stall_count == stall_ticks_required:
                self._fsm.get_logger().info(
                    "DESCEND: 下降停滞判定到位 "
                    f"height_error={height_error:.2f}m "
                    f"vd={vd:.3f}m/s"
                )
            self._descent_ready_latched = True
            return True

        return False

    def _active_drop_height(self) -> float:
        if self._precision_enabled() and self._final_descent_started:
            return float(self._fsm.get_param("precision_align_final_drop_height", 1.25))
        return float(self._fsm.get_param("drop_descend_height", 2.0))

    def _start_final_descent_if_needed(self) -> bool:
        if not bool(self._fsm.get_param("precision_align_final_descent_enable", True)):
            return False
        if self._final_descent_started:
            return False

        align_h = float(self._fsm.get_param("drop_descend_height", 2.0))
        final_h = float(self._fsm.get_param("precision_align_final_drop_height", 1.25))
        if final_h <= 0.0 or final_h >= align_h - 0.05:
            return False

        self._final_descent_started = True
        self._descent_ready_latched = False
        self._centered_latch_until = 0.0
        # 仅重置确认状态 (稳定计数/刹车/投放许可), 保留目标锁和 EMA 平滑值.
        # 避免冷启动导致低空像素误差放大后精校器来回振荡 (抖动).
        self._precision_aligner.reset_confirmation()
        self._start_time = time.time()
        self._descend_best_pixel_error = float("inf")
        self._descend_stall_count = 0

        pos = self._fsm.bridge.get_current_position()
        self._descent_initial_error = abs(pos.down - self._fsm.fc.altitude_to_ned(final_h))
        speed = float(self._fsm.get_param("precision_align_final_descent_speed", 0.25))
        self._fsm.fc.goto_ned(pos.north, pos.east, self._fsm.fc.altitude_to_ned(final_h), speed=speed)
        self._descent_progress = self._new_navigation_progress()
        self._alignment_progress = self._new_alignment_progress()
        self._fsm.get_logger().info(
            "DESCEND: 初次精校完成, 定点直降至最终投放高度后复查 "
            f"{align_h:.2f}m → {final_h:.2f}m"
        )
        return True

    def _current_align_status(self):
        if self._precision_enabled():
            return self._precision_aligner.last_status
        return self._visual_aligner.last_status

    def _retry_precision_align_in_place(self, drop_h: float, reason: str) -> bool:
        max_retries = int(self._fsm.get_param("precision_align_max_retries", 1))
        if self._descend_retry_count >= max_retries:
            return False

        self._descend_retry_count += 1
        self._precision_aligner.reset(self._selected_memory_xy())
        self._centered_latch_until = 0.0
        self._start_time = time.time()
        self._descend_best_pixel_error = float("inf")
        self._descend_stall_count = 0

        pos = self._fsm.bridge.get_current_position()
        self._descent_initial_error = abs(pos.down - self._fsm.fc.altitude_to_ned(drop_h))
        self._fsm.fc.goto_ned(pos.north, pos.east, self._fsm.fc.altitude_to_ned(drop_h), speed=0.0)
        self._descent_progress = self._new_navigation_progress()
        self._alignment_progress = self._new_alignment_progress()
        self._fsm.get_logger().warn(
            f"DESCEND: {reason}, 低空原地重置精校 "
            f"(retry={self._descend_retry_count}/{max_retries})"
        )
        return True

    def _warn_descend_throttled(self, message: str, interval: float = 1.0):
        now = time.time()
        if now - self._last_descend_log_time >= interval:
            self._last_descend_log_time = now
            self._fsm.get_logger().info(message)

    def _nudge_to_visible_drop_target(
        self,
        drop_h: float,
        *,
        altitude_m: Optional[float] = None,
        descent_speed: float = 0.0,
        allow_release: bool = True,
    ) -> bool:
        if self._precision_enabled():
            result = self._precision_aligner.step(
                markers=self._fsm.get_markers() if hasattr(self._fsm, "get_markers") else [],
                cylinders=self._fsm.get_cylinders(),
                drop_height=drop_h,
                altitude_m=altitude_m,
                descent_speed=descent_speed,
                allow_release=allow_release,
            )
            if result == PrecisionAlignResult.CONVERGED:
                return True
            if result == PrecisionAlignResult.TARGET_LOST:
                if self._retry_precision_align_in_place(drop_h, "精校目标丢失超限"):
                    return True
                self._precision_align_failed = True
                return False
            return True
        return self._visual_aligner.command_alignment(drop_h)

    def _continuous_drop_servo_enabled(self) -> bool:
        return (
            bool(self._fsm.get_param("quick_drop_mode", False))
            and self._precision_enabled()
            and bool(self._fsm.get_param("precision_align_continuous_enable", False))
        )

    def _continuous_servo_descend_tick(self, down: float) -> None:
        """Keep one ground-error controller active during the full quick descent."""
        altitude = float(self._fsm.fc.ned_to_altitude(down))
        if altitude > 3.0:
            descent = float(self._fsm.get_param("drop_servo_descend_fast", 0.5))
        else:
            descent = float(self._fsm.get_param("drop_servo_descend_slow", 0.3))
        self._nudge_to_visible_drop_target(
            self._active_drop_height(),
            altitude_m=altitude,
            descent_speed=descent,
            allow_release=False,
        )

    def _drop_target_is_centered(self) -> bool:
        """投放许可: release_ready 锁存, 对心满足后宽限期内不因短暂丢帧撤销."""
        if self._precision_enabled():
            status = self._precision_aligner.last_status
            stable_frames = int(self._fsm.get_param("precision_align_stable_frames", 4))
            centered = status.release_ready and status.stable_count >= stable_frames
            if centered:
                # 对心一旦满足即锁存宽限窗口: 投放确认期短暂丢帧/像素超标
                # 不撤销已满足的对心许可, 保证对心后及时投放、不卡在投放区.
                # release_ready 本身在任一未对齐帧即清零, 宽限只覆盖刚满足后的
                # 极短窗口 (默认 1.5s), 期间无人机物理位移可忽略 (刹车后 ≤0.04m/s).
                latch_s = float(self._fsm.get_param(
                    "precision_align_release_latch_s", 1.5))
                self._centered_latch_until = time.time() + max(latch_s, 0.0)
                return True
            if time.time() < self._centered_latch_until:
                self._warn_descend_throttled(
                    "DESCEND: 对心宽限期内保持投放许可 "
                    f"raw={status.pixel_error:.1f}px "
                    f"stable={status.stable_count}/{stable_frames} "
                    f"ready={status.release_ready}"
                )
                return True
            self._warn_descend_throttled(
                "DESCEND: 精校投放许可失败 "
                f"source={status.source} raw={status.pixel_error:.1f}px "
                f"smooth_px={status.smooth_pixel_error:.1f}px "
                f"smooth=({status.smooth_dx_norm:+.4f},{status.smooth_dy_norm:+.4f}) "
                f"stable={status.stable_count}/{stable_frames} "
                f"ready={status.release_ready} reason={status.reason}"
            )
            return False

        centered = self._visual_aligner.is_centered()
        if not centered:
            status = self._visual_aligner.last_status
            self._warn_descend_throttled(
                "DESCEND: 投放许可失败 "
                f"source={status.source} pixel_error={status.pixel_error:.1f}px "
                f"stable={status.stable_count}"
            )
        return centered

    def _execute_drop(self) -> Optional[str]:
        # 等待投放确认
        timeout = self._fsm.get_param("drop_confirmation_timeout", 5.0)
        result = self._fsm.check_drop_service_result()
        if result is not None:
            self._drop_success = result
            if self._drop_success:
                self._fsm.get_logger().info("DROP: 投放成功")
            else:
                self._fsm.get_logger().error("DROP: 舵机未确认投放, 等待人工接管")
                self._takeover("DROP: servo did not confirm the release")
                return None
            return "ASCEND"

        if time.time() - self._start_time > timeout:
            self._fsm.get_logger().warn("DROP: 投放结果不明确, 等待人工接管")
            self._takeover("DROP: release result remained unconfirmed")
            return None
        return None

    def _execute_ascend(self) -> Optional[str]:
        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        pos = self._fsm.bridge.get_current_position()

        if abs(self._fsm.fc.ned_to_altitude(pos.down) - takeoff_h) < 0.5:
            self._fsm.get_logger().info("ASCEND: 回到巡航高度")
            # 判断是否需要继续投放
            drops = self._fsm.drop_success_count
            max_drops = self._fsm.get_param("max_drops", 2)
            if drops < max_drops:
                return "SELECT_TARGET"
            else:
                return "TRANSIT_TO_RECON_ZONE"

        if self._ascend_progress is not None:
            altitude_error = abs(self._fsm.fc.ned_to_altitude(pos.down) - takeoff_h)
            now = time.monotonic()
            if self._ascend_progress.observe(altitude_error, now=now):
                if self._ascend_progress.mark_retry(now=now):
                    self._fsm.get_logger().warn(
                        "ASCEND: 高度无进展, 重发同一回升目标一次"
                    )
                    self._command_ascend_target()
                else:
                    self._takeover("ASCEND: altitude made no progress after retry")

        return None

    def _new_navigation_progress(self) -> ProgressMonitor:
        return ProgressMonitor(
            window_s=float(self._fsm.get_param("navigation_progress_window", 30.0)),
            min_improvement=float(
                self._fsm.get_param("navigation_progress_min_improvement", 0.25)
            ),
        )

    def _new_alignment_progress(self) -> ProgressMonitor:
        return ProgressMonitor(
            window_s=self._align_timeout(),
            min_improvement=float(
                self._fsm.get_param("visual_progress_min_improvement", 2.0)
            ),
        )

    def _alignment_stalled(self, status) -> bool:
        if self._alignment_progress is None or not math.isfinite(status.pixel_error):
            return False
        return self._alignment_progress.observe(
            status.pixel_error, now=time.monotonic()
        )

    def _quick_fallback_allow_drop(self, status) -> bool:
        """quick 模式超时兜底: 像素误差有限且 ≤ 门限才允许直接投放, 不做盲投."""
        if not self._fsm.get_param("quick_drop_mode", False):
            return False
        limit = float(self._fsm.get_param(
            "precision_align_fallback_max_pixel_error", 0.0
        ))
        if limit <= 0.0:
            return False
        # TARGET_LOST 走 _nudge_to_visible_drop_target 另一分支, 不会到这里;
        # 兜底仍要求有限像素误差, 丢失时 pixel_error=NaN 自然被拒
        return math.isfinite(status.pixel_error) and status.pixel_error <= limit

    def _command_descend_target(self, drop_height: float) -> None:
        pos = self._fsm.bridge.get_current_position()
        self._fsm.fc.goto_ned(
            pos.north,
            pos.east,
            self._fsm.fc.altitude_to_ned(drop_height),
            speed=float(self._fsm.get_param("drop_descend_speed", 0.5)),
        )

    def _command_ascend_target(self) -> None:
        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        pos = self._fsm.bridge.get_current_position()
        self._fsm.fc.goto_ned(
            pos.north,
            pos.east,
            self._fsm.fc.altitude_to_ned(takeoff_h),
            speed=1.0,
        )

    def _takeover(self, reason: str) -> None:
        self._fsm.request_takeover(reason)

    def exit(self):
        self._fsm.logger.log_state(self._sub_state, "NEXT",
                                   f"投放 #{self._fsm.drop_count} 完成")
