"""
CUADC-style low-altitude precision alignment.

This controller is only for the final drop alignment after world-coordinate
coarse positioning has already brought the drone near the selected target.
It keeps the control problem in image space: normalized pixel error, EMA
smoothing, P-only body velocity, short step/stop cycles, and consecutive
stable-frame confirmation.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Tuple


class PrecisionAlignResult(Enum):
    CONTINUE = "continue"
    CONVERGED = "converged"
    TARGET_LOST = "target_lost"


@dataclass
class PrecisionAlignStatus:
    source: str = "none"
    pixel_error: float = float("inf")
    smooth_pixel_error: float = float("inf")
    raw_dx_norm: float = 0.0
    raw_dy_norm: float = 0.0
    smooth_dx_norm: float = 0.0
    smooth_dy_norm: float = 0.0
    stable_count: int = 0
    lost_frames: int = 0
    target_world_xy: Optional[Tuple[float, float]] = None
    command_forward: float = 0.0
    command_right: float = 0.0
    release_ready: bool = False
    reason: str = ""


class PrecisionAlignController:
    """
    Low-altitude visual alignment controller.

    The caller provides current marker/cylinder detections each control tick.
    The controller chooses and tracks one target, computes a smoothed normalized
    image error, and sends short body-frame velocity commands through fsm.fc.
    """

    def __init__(self, fsm):
        self._fsm = fsm
        self._target_hint_xy: Optional[Tuple[float, float]] = None
        self._locked_pixel_uv: Optional[Tuple[float, float]] = None
        self._locked_world_xy: Optional[Tuple[float, float]] = None
        self._locked_source: str = "none"
        self._smooth_dx: Optional[float] = None
        self._smooth_dy: Optional[float] = None
        self._stable_count = 0
        self._lost_frames = 0
        self._active_until = 0.0
        self._hover_until = 0.0
        self._brake_until = 0.0
        self._release_ready = False
        self._last_forward = 0.0
        self._last_right = 0.0
        self._last_log_time = 0.0
        self._last_status = PrecisionAlignStatus()

    @property
    def last_status(self) -> PrecisionAlignStatus:
        return self._last_status

    def reset(self, target_hint_xy: Optional[Tuple[float, float]] = None) -> None:
        self._target_hint_xy = target_hint_xy or self._selected_memory_xy()
        self._locked_pixel_uv = None
        self._locked_world_xy = None
        self._locked_source = "none"
        self._smooth_dx = None
        self._smooth_dy = None
        self._stable_count = 0
        self._lost_frames = 0
        self._active_until = 0.0
        self._hover_until = 0.0
        self._brake_until = 0.0
        self._release_ready = False
        self._last_forward = 0.0
        self._last_right = 0.0
        self._last_log_time = 0.0
        self._last_status = PrecisionAlignStatus(reason="reset")

    def reset_confirmation(self) -> None:
        """
        仅重置确认状态 (稳定计数/刹车/投放许可).

        保留目标像素锁定和 EMA 平滑值, 使高度变化后无需冷启动
        重新收敛, 仅需在新高度重新确认像素居中。
        """
        self._stable_count = 0
        self._release_ready = False
        self._brake_until = 0.0
        self._active_until = 0.0
        self._hover_until = 0.0
        self._last_forward = 0.0
        self._last_right = 0.0
        self._last_status = PrecisionAlignStatus(reason="altitude_change")

    @staticmethod
    def _field(obj: Any, key: str, default=0.0):
        return getattr(obj, key, obj.get(key, default) if isinstance(obj, dict) else default)

    def _now(self) -> float:
        return self._fsm.get_clock().now().nanoseconds / 1e9

    def _param(self, key: str, default):
        return self._fsm.get_param(key, default)

    def _selected_memory_xy(self) -> Optional[Tuple[float, float]]:
        selected_positions = getattr(self._fsm.cylinder_selector, "_selected_positions", [])
        if not selected_positions:
            return None
        return selected_positions[-1]

    def _camera_geometry(self) -> Tuple[float, float, float, float]:
        cx = float(self._param("camera_cx", 640.0))
        cy = float(self._param("camera_cy", 480.0))
        width = float(self._param("precision_align_image_width", max(cx * 2.0, 1.0)))
        height = float(self._param("precision_align_image_height", max(cy * 2.0, 1.0)))
        return cx, cy, max(width, 1.0), max(height, 1.0)

    def step(self, markers: list, cylinders: list, drop_height: float) -> PrecisionAlignResult:
        """Run one alignment tick."""
        now = self._now()

        if now < self._active_until:
            self._fsm.fc.set_velocity_body(self._last_forward, self._last_right, 0.0)
            return PrecisionAlignResult.CONTINUE

        if self._active_until > 0.0:
            self._fsm.fc.stop_velocity()
            self._active_until = 0.0
            self._hover_until = now + float(self._param("precision_align_hover_s", 0.04))
            return PrecisionAlignResult.CONTINUE

        if now < self._hover_until:
            self._fsm.fc.stop_velocity()
            return PrecisionAlignResult.CONTINUE

        if now < self._brake_until:
            self._fsm.fc.stop_velocity()
            return PrecisionAlignResult.CONTINUE

        target, source, reason = self._choose_target(markers, cylinders)
        if target is None:
            self._stable_count = 0
            self._brake_until = 0.0
            self._release_ready = False
            self._lost_frames += 1
            self._last_status = PrecisionAlignStatus(
                source=source,
                stable_count=0,
                lost_frames=self._lost_frames,
                reason=reason,
            )
            self._fsm.fc.stop_velocity()
            self._maybe_log_missing()

            reacquire_after = int(self._param("precision_align_reacquire_after_lost", 6))
            if self._lost_frames == reacquire_after:
                self._locked_pixel_uv = None
                self._smooth_dx = None
                self._smooth_dy = None
                self._fsm.get_logger().warn(
                    "PREC_ALIGN: 目标短暂丢失, 清除像素锁定并按画面中心重捕获"
                )

            max_lost = int(self._param("precision_align_max_lost_frames", 32))
            if self._lost_frames > max_lost:
                return PrecisionAlignResult.TARGET_LOST
            return PrecisionAlignResult.CONTINUE

        self._lost_frames = 0
        cx, cy, width, height = self._camera_geometry()
        u = float(self._field(target, "center_u", cx))
        v = float(self._field(target, "center_v", cy))
        raw_dx = (u - cx) / width
        raw_dy = (v - cy) / height
        pixel_error = math.hypot(u - cx, v - cy)

        alpha = max(0.01, min(float(self._param("precision_align_alpha", 0.55)), 1.0))
        if self._smooth_dx is None or self._smooth_dy is None:
            self._smooth_dx = raw_dx
            self._smooth_dy = raw_dy
        else:
            self._smooth_dx = alpha * raw_dx + (1.0 - alpha) * self._smooth_dx
            self._smooth_dy = alpha * raw_dy + (1.0 - alpha) * self._smooth_dy

        smooth_pixel_error = math.hypot(self._smooth_dx * width, self._smooth_dy * height)
        aligned = self._is_precise(pixel_error, smooth_pixel_error, raw_dx, raw_dy)
        self._stable_count = self._stable_count + 1 if aligned else 0
        if not aligned:
            self._brake_until = 0.0
            self._release_ready = False

        wx = float(self._field(target, "center_x", 0.0))
        wy = float(self._field(target, "center_y", 0.0))
        self._last_status = PrecisionAlignStatus(
            source=source,
            pixel_error=pixel_error,
            smooth_pixel_error=smooth_pixel_error,
            raw_dx_norm=raw_dx,
            raw_dy_norm=raw_dy,
            smooth_dx_norm=float(self._smooth_dx),
            smooth_dy_norm=float(self._smooth_dy),
            stable_count=self._stable_count,
            lost_frames=0,
            target_world_xy=(wx, wy),
            release_ready=False,
            reason=reason,
        )

        stable_frames = int(self._param("precision_align_stable_frames", 5))
        if self._stable_count >= stable_frames:
            self._fsm.fc.stop_velocity()
            if not self._release_ready and self._brake_until <= 0.0:
                brake_s = float(self._param("precision_align_final_brake_s", 0.5))
                self._brake_until = now + max(brake_s, 0.0)
                self._last_status.reason = "final_brake"
                self._fsm.get_logger().info(
                    "PREC_ALIGN: 进入最终刹车复查 "
                    f"source={source} raw={pixel_error:.1f}px smooth={smooth_pixel_error:.1f}px "
                    f"stable={self._stable_count}/{stable_frames}"
                )
                return PrecisionAlignResult.CONTINUE

            if self._horizontal_speed_ok():
                self._release_ready = True
                self._last_status.release_ready = True
                self._last_status.reason = "ready"
                self._fsm.get_logger().info(
                    "PREC_ALIGN: 视觉精校稳定, 允许投放 "
                    f"source={source} raw={pixel_error:.1f}px smooth={smooth_pixel_error:.1f}px "
                    f"stable={self._stable_count}/{stable_frames}"
                )
                return PrecisionAlignResult.CONVERGED

            self._last_status.reason = "waiting_velocity_settle"
            self._maybe_log_tracking()
            return PrecisionAlignResult.CONTINUE

        if aligned:
            self._fsm.fc.stop_velocity()
            self._last_status.reason = "collecting_stable_frames"
            return PrecisionAlignResult.CONTINUE

        gain = float(self._param("precision_align_gain", 1.25))
        x_to_right_sign = float(self._param("precision_align_image_x_to_right_sign", 1.0))
        y_to_forward_sign = float(self._param("precision_align_image_y_to_forward_sign", -1.0))
        forward = y_to_forward_sign * gain * float(self._smooth_dy)
        right = x_to_right_sign * gain * float(self._smooth_dx)
        forward, right = self._limit_vector(forward, right, pixel_error)

        self._last_forward = forward
        self._last_right = right
        self._last_status.command_forward = forward
        self._last_status.command_right = right
        self._maybe_log_tracking()

        if pixel_error <= float(self._param("precision_align_fine_pixel_error", 90.0)):
            step_s = float(self._param("precision_align_fine_step_s", 0.12))
        else:
            step_s = float(self._param("precision_align_step_s", 0.18))
        self._fsm.fc.set_velocity_body(forward, right, 0.0)
        self._active_until = now + step_s
        return PrecisionAlignResult.CONTINUE

    def _is_precise(
        self,
        raw_pixel_error: float,
        smooth_pixel_error: float,
        raw_dx: float,
        raw_dy: float,
    ) -> bool:
        raw_tol = float(self._param("precision_align_raw_pixel_tolerance", 9.0))
        smooth_tol = float(self._param("precision_align_smooth_pixel_tolerance", 7.0))
        threshold_x = float(self._param("precision_align_threshold_norm_x", 0.008))
        threshold_y = float(self._param("precision_align_threshold_norm_y", 0.008))
        return (
            raw_pixel_error <= raw_tol
            and smooth_pixel_error <= smooth_tol
            and abs(raw_dx) <= threshold_x
            and abs(raw_dy) <= threshold_y
        )

    def _limit_vector(
        self,
        forward: float,
        right: float,
        pixel_error: float,
    ) -> Tuple[float, float]:
        if pixel_error <= float(self._param("precision_align_fine_pixel_error", 90.0)):
            max_speed = float(self._param("precision_align_fine_max_speed", 0.065))
        else:
            max_speed = float(self._param("precision_align_max_speed", 0.25))
        norm = math.hypot(forward, right)
        if norm > max_speed > 0.0:
            scale = max_speed / norm
            return forward * scale, right * scale
        min_speed = float(self._param("precision_align_min_correction_speed", 0.012))
        raw_tol = float(self._param("precision_align_raw_pixel_tolerance", 9.0))
        if 0.0 < norm < min_speed and pixel_error > raw_tol:
            scale = min_speed / norm
            return forward * scale, right * scale
        return forward, right

    def _horizontal_speed_ok(self) -> bool:
        max_speed = float(self._param("precision_align_release_max_horizontal_speed", 0.04))
        bridge = getattr(self._fsm, "bridge", None)
        get_telemetry = getattr(bridge, "get_telemetry", None)
        if not callable(get_telemetry):
            # 无法获取遥测时不能假设速度为零, 延迟此帧等遥测就绪
            return False
        try:
            velocity = get_telemetry().velocity
            speed_xy = math.hypot(float(velocity.vn), float(velocity.ve))
        except Exception:
            return False
        if speed_xy <= max_speed:
            return True
        self._fsm.fc.stop_velocity()
        self._fsm.get_logger().info(
            "PREC_ALIGN: 等待水平速度归零 "
            f"speed_xy={speed_xy:.3f}m/s max={max_speed:.3f}m/s"
        )
        return False

    def _choose_target(self, markers: list, cylinders: list):
        source_preference = str(self._param("precision_align_target_source", "cylinder")).lower()
        min_conf = float(self._param("precision_align_min_confidence", 0.25))

        marker_candidates = [
            (m, "marker")
            for m in markers
            if float(self._field(m, "confidence", 0.0)) >= min_conf
        ]
        cylinder_candidates = [
            (c, "cylinder")
            for c in cylinders
            if float(self._field(c, "confidence", 0.0)) >= min_conf
        ]

        if source_preference == "marker":
            candidates = marker_candidates
        elif source_preference == "cylinder":
            candidates = cylinder_candidates
        else:
            candidates = marker_candidates + cylinder_candidates
        if not candidates:
            return None, source_preference, "no_candidates"

        cx, cy, width, height = self._camera_geometry()
        diag = max(math.hypot(width, height), 1.0)
        world_gate = float(self._param("precision_align_target_world_gate", 1.5))
        jump_gate = float(self._param("precision_align_max_jump_diag_ratio", 0.45))
        center_reject = float(self._param("precision_align_reject_center_ratio", 0.28))

        scored = []
        for target, source in candidates:
            u = float(self._field(target, "center_u", cx))
            v = float(self._field(target, "center_v", cy))
            center_ratio = math.hypot(u - cx, v - cy) / diag

            wx = float(self._field(target, "center_x", 0.0))
            wy = float(self._field(target, "center_y", 0.0))
            world_ratio = 0.0
            world_error = None
            if self._target_hint_xy is not None and self._locked_pixel_uv is None:
                tx, ty = self._target_hint_xy
                world_error = math.hypot(wx - tx, wy - ty)
                if world_error > world_gate:
                    continue
                world_ratio = min(world_error / max(world_gate, 0.01), 1.0)

            continuity_ratio = center_ratio
            if self._locked_pixel_uv is not None:
                lu, lv = self._locked_pixel_uv
                continuity_ratio = math.hypot(u - lu, v - lv) / diag
                if continuity_ratio > jump_gate and center_ratio > center_reject:
                    continue

            # CUADC-style locked tracking: pixel continuity dominates; center
            # distance helps reacquire after altitude changes or short loss.
            #
            # 评分权重设计原则 (越小越好):
            #   0.68 continuity_ratio  — 主项: 同一目标的帧间像素位移, 防止锁定跳变
            #   0.32 center_ratio       — 辅项: 距画面中心的归一化距离, 倾向选居中的目标
            #   0.08 world_ratio        — 弱项: 距预期 World 位置的距离, 仅在未锁定时生效
            #  -0.04 confidence         — 微调: 轻微偏好高置信度 (负权重→置信度越高 score 越低)
            #
            # continuity_ratio 占主导 (~65%) 确保锁定的目标不会被画面边缘的新目标"抢走".
            # world_ratio 权重很小 (~7%) 因为 World 坐标在高空时噪声大, 仅作弱先验.
            score = (
                0.68 * continuity_ratio
                + 0.32 * center_ratio
                + 0.08 * world_ratio
                - 0.04 * float(self._field(target, "confidence", 0.0))
            )
            scored.append((score, target, source))

        if not scored:
            return None, source_preference, "all_candidates_rejected"

        _, selected, source = min(scored, key=lambda item: item[0])
        u = float(self._field(selected, "center_u", cx))
        v = float(self._field(selected, "center_v", cy))
        wx = float(self._field(selected, "center_x", 0.0))
        wy = float(self._field(selected, "center_y", 0.0))

        if self._locked_pixel_uv is None:
            self._fsm.get_logger().info(
                "PREC_ALIGN: 锁定低空精校目标 "
                f"source={source} pixel=({u:.0f},{v:.0f}) world=({wx:.2f},{wy:.2f})"
            )

        self._locked_pixel_uv = (u, v)
        self._locked_world_xy = (wx, wy)
        self._locked_source = source
        return selected, source, "tracking"

    def _maybe_log_tracking(self) -> None:
        now = self._now()
        interval = float(self._param("precision_align_log_interval", 0.5))
        if now - self._last_log_time < interval:
            return
        self._last_log_time = now
        s = self._last_status
        self._fsm.get_logger().info(
            "PREC_ALIGN: "
            f"source={s.source} pixel_error={s.pixel_error:.1f}px "
            f"raw=({s.raw_dx_norm:+.4f},{s.raw_dy_norm:+.4f}) "
            f"smooth=({s.smooth_dx_norm:+.4f},{s.smooth_dy_norm:+.4f}) "
            f"world=({s.target_world_xy[0]:.2f},{s.target_world_xy[1]:.2f}) "
            f"stable={s.stable_count} cmd=({s.command_forward:+.3f},{s.command_right:+.3f})"
        )

    def _maybe_log_missing(self) -> None:
        now = self._now()
        interval = float(self._param("precision_align_log_interval", 0.5))
        if now - self._last_log_time < interval:
            return
        self._last_log_time = now
        self._fsm.get_logger().warn(
            "PREC_ALIGN: 未见可用目标 "
            f"lost={self._lost_frames} reason={self._last_status.reason}"
        )
