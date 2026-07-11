"""低空投放视觉校准.

该模块只负责 DESCEND_TO_DROP 阶段的最后校准:
  1. 选择当前可见的投放目标: 圆筒优先, 无圆筒时 marker 兜底
  2. 控制飞机向视觉识别出的目标世界坐标小步移动
  3. 当目标像素中心连续稳定接近相机中心时, 允许投放

高空搜索/目标选择仍由 MissionFSM + CylinderSelector 负责。
"""

import math
from dataclasses import dataclass
from typing import Any, Optional, Tuple

from px4_control.telemetry import ned_to_world, world_to_ned


@dataclass
class VisualAlignStatus:
    """低空视觉校准当前状态."""

    source: str = "none"
    pixel_error: float = float("inf")
    pixel_du: float = 0.0
    pixel_dv: float = 0.0
    world_error_to_memory: float = float("inf")
    target_world_xy: Optional[Tuple[float, float]] = None
    stable_count: int = 0


class DropVisualAligner:
    """投放前低空视觉精修控制器.

    策略 (A+C 组合):
      A) pixel_error < brake_threshold 时刹停悬停, 让 is_centered 稳定累积
      C) du/dv 过零时锁定该轴, 防止 overshoot 后继续反向推动
    """

    def __init__(self, fsm):
        self._fsm = fsm
        self._stable_count = 0
        self._last_log_time = 0.0
        self._last_status = VisualAlignStatus()
        # 零交叉检测 & 刹车状态
        self._prev_du: float = 0.0
        self._prev_dv: float = 0.0
        self._u_locked: bool = False
        self._v_locked: bool = False
        self._brake_active: bool = False
        self._brake_enter_time: float = 0.0

    def reset(self) -> None:
        self._stable_count = 0
        self._last_log_time = 0.0
        self._last_status = VisualAlignStatus()
        self._prev_du = 0.0
        self._prev_dv = 0.0
        self._u_locked = False
        self._v_locked = False
        self._brake_active = False
        self._brake_enter_time = 0.0

    @property
    def last_status(self) -> VisualAlignStatus:
        return self._last_status

    @staticmethod
    def _field(obj: Any, key: str, default=0.0):
        return getattr(obj, key, obj.get(key, default) if isinstance(obj, dict) else default)

    def _selected_memory_xy(self) -> Optional[Tuple[float, float]]:
        selected_positions = getattr(self._fsm.cylinder_selector, "_selected_positions", [])
        if not selected_positions:
            return None
        return selected_positions[-1]

    def _current_targets(self) -> Tuple[list, str]:
        cylinders = self._fsm.get_cylinders()
        if cylinders:
            return cylinders, "cylinder"

        markers = self._fsm.get_markers() if hasattr(self._fsm, "get_markers") else []
        if markers:
            return markers, "marker"

        return [], "none"

    def _pixel_delta(self, target: Any) -> Tuple[float, float]:
        img_cx = float(self._fsm.get_param("camera_cx", 640.0))
        img_cy = float(self._fsm.get_param("camera_cy", 480.0))
        u = float(self._field(target, "center_u", 0.0))
        v = float(self._field(target, "center_v", 0.0))
        return u - img_cx, v - img_cy

    def _pixel_error(self, target: Any) -> float:
        du, dv = self._pixel_delta(target)
        return math.hypot(du, dv)

    def _choose_target(self) -> Tuple[Optional[Any], VisualAlignStatus]:
        targets, source = self._current_targets()
        memory_xy = self._selected_memory_xy()
        if not targets or memory_xy is None:
            return None, VisualAlignStatus(source=source, stable_count=self._stable_count)

        tx, ty = memory_xy
        world_gate = float(self._fsm.get_param("drop_visual_align_world_gate", 1.5))

        best = None
        best_score = float("inf")
        best_pixel = float("inf")
        best_du = 0.0
        best_dv = 0.0
        best_world = float("inf")

        for target in targets:
            conf = float(self._field(target, "confidence", 0.0))
            if conf < 0.25:
                continue

            wx = float(self._field(target, "center_x", 0.0))
            wy = float(self._field(target, "center_y", 0.0))
            world_error = math.hypot(wx - tx, wy - ty)
            du, dv = self._pixel_delta(target)
            pixel_error = math.hypot(du, dv)

            # 仍然围绕已选目标做校准, 但允许 1m 量级的视觉坐标偏差被修正。
            if world_error > world_gate and pixel_error > float(self._fsm.get_param("drop_align_pixel_tolerance", 35.0)):
                continue

            score = min(world_error, world_gate) * 1000.0 + pixel_error
            if score < best_score:
                best = target
                best_score = score
                best_pixel = pixel_error
                best_du = du
                best_dv = dv
                best_world = world_error

        if best is None:
            return None, VisualAlignStatus(source=source, stable_count=self._stable_count)

        status = VisualAlignStatus(
            source=source,
            pixel_error=best_pixel,
            pixel_du=best_du,
            pixel_dv=best_dv,
            world_error_to_memory=best_world,
            target_world_xy=(
                float(self._field(best, "center_x", tx)),
                float(self._field(best, "center_y", ty)),
            ),
            stable_count=self._stable_count,
        )
        return best, status

    def is_centered(self) -> bool:
        """目标是否已经在画面中心稳定对齐."""
        target, status = self._choose_target()
        tolerance = float(self._fsm.get_param("drop_align_pixel_tolerance", 35.0))
        stable_frames = int(self._fsm.get_param("drop_visual_align_stable_frames", 6))

        if target is None or status.pixel_error > tolerance:
            self._stable_count = 0
            status.stable_count = 0
            self._last_status = status
            return False

        self._stable_count += 1
        status.stable_count = self._stable_count
        self._last_status = status
        if self._stable_count >= stable_frames:
            self._fsm.get_logger().info(
                "DESCEND_ALIGN: 视觉中心稳定, 允许投放 "
                f"source={status.source} pixel_error={status.pixel_error:.1f}px "
                f"stable={self._stable_count}/{stable_frames}"
            )
            return True
        return False

    def command_alignment(self, drop_height: float) -> bool:
        """向视觉识别到的目标小步移动. 返回是否发送了校准 setpoint."""
        target, status = self._choose_target()
        self._last_status = status
        if target is None or status.target_world_xy is None:
            return False

        pos = self._fsm.bridge.get_current_position()
        drone_wx, drone_wy, _ = ned_to_world(pos.north, pos.east, pos.down)
        now = self._fsm.get_clock().now().nanoseconds / 1e9
        brake_threshold = float(self._fsm.get_param("drop_visual_align_brake_threshold", 80.0))

        # ==== 策略 C: 零交叉检测 — du/dv 符号翻转时锁定该轴, 防止 overshoot ====
        if self._prev_du != 0.0 and (status.pixel_du * self._prev_du) < 0:
            self._u_locked = True
            self._fsm.get_logger().info(
                f"DESCEND_ALIGN: U轴过零锁定 du={status.pixel_du:.1f}px 前={self._prev_du:.1f}px"
            )
        if self._prev_dv != 0.0 and (status.pixel_dv * self._prev_dv) < 0:
            self._v_locked = True
            self._fsm.get_logger().info(
                f"DESCEND_ALIGN: V轴过零锁定 dv={status.pixel_dv:.1f}px 前={self._prev_dv:.1f}px"
            )
        self._prev_du = status.pixel_du
        self._prev_dv = status.pixel_dv

        # ==== 策略 A: 刹车区 — pixel_error 进入阈值范围后悬停, 让 is_centered 稳定累积 ====
        if status.pixel_error < brake_threshold:
            if not self._brake_active:
                self._brake_active = True
                self._brake_enter_time = now
                self._fsm.get_logger().info(
                    f"DESCEND_ALIGN: 进入刹车区 pixel_error={status.pixel_error:.1f}px < {brake_threshold}px"
                )
            # 悬停在当前位置, 不继续移动
            tn, te, _ = world_to_ned(drone_wx, drone_wy, 0.0)
            self._fsm.fc.goto_ned(tn, te, -drop_height, speed=0.0)

            # 刹车超时仍未通过 is_centered → 解锁双轴, 退出刹车重试
            brake_timeout = float(self._fsm.get_param("drop_visual_align_brake_timeout", 4.0))
            if now - self._brake_enter_time > brake_timeout:
                self._u_locked = False
                self._v_locked = False
                self._brake_active = False
                self._fsm.get_logger().info(
                    f"DESCEND_ALIGN: 刹车超时 ({brake_timeout}s), 解锁重试"
                )

            if now - self._last_log_time >= 1.0:
                self._last_log_time = now
                self._fsm.get_logger().info(
                    "DESCEND_ALIGN: 刹车悬停中 "
                    f"pixel_error={status.pixel_error:.1f}px "
                    f"du={status.pixel_du:.1f} dv={status.pixel_dv:.1f} "
                    f"u_locked={self._u_locked} v_locked={self._v_locked} "
                    f"brake_t={now - self._brake_enter_time:.1f}s"
                )
            return True

        # 退出刹车区
        self._brake_active = False

        # ==== 正常像素伺服, 受轴锁定约束 ====
        max_step = float(self._fsm.get_param("drop_visual_align_max_step", 0.12))
        align_speed = float(self._fsm.get_param("drop_visual_align_speed", 0.08))
        gain = float(self._fsm.get_param("drop_visual_align_pixel_gain", 0.45))
        x_sign = float(self._fsm.get_param("drop_visual_align_x_sign", -1.0))
        y_sign = float(self._fsm.get_param("drop_visual_align_y_sign", 1.0))
        fx = float(self._fsm.get_param("camera_fx", 539.9))
        fy = float(self._fsm.get_param("camera_fy", 539.9))
        alt = max(abs(pos.down), 0.5)

        dx = x_sign * gain * status.pixel_du * alt / fx
        dy = y_sign * gain * status.pixel_dv * alt / fy
        dx = max(-max_step, min(max_step, dx))
        dy = max(-max_step, min(max_step, dy))

        if self._u_locked:
            dx = 0.0
        if self._v_locked:
            dy = 0.0

        # 双轴都锁定但 pixel_error 仍大 — 可能锁定过早或方向误判, 重置
        if self._u_locked and self._v_locked:
            self._u_locked = False
            self._v_locked = False
            self._fsm.get_logger().info("DESCEND_ALIGN: 双轴锁定但未刹车, 重置锁定")

        target_wx = drone_wx + dx
        target_wy = drone_wy + dy

        if hasattr(self._fsm, "_drop_zone_world_bounds"):
            wx_min, wx_max, wy_min, wy_max = self._fsm._drop_zone_world_bounds(margin=0.0)
            target_wx = max(wx_min, min(wx_max, target_wx))
            target_wy = max(wy_min, min(wy_max, target_wy))

        tn, te, _ = world_to_ned(target_wx, target_wy, 0.0)
        self._fsm.fc.goto_ned(tn, te, -drop_height, speed=align_speed)

        if now - self._last_log_time >= 1.0:
            self._last_log_time = now
            self._fsm.get_logger().info(
                "DESCEND_ALIGN: 低空视觉校准 "
                f"source={status.source} pixel_error={status.pixel_error:.1f}px "
                f"du={status.pixel_du:.1f}px dv={status.pixel_dv:.1f}px "
                f"world_error={status.world_error_to_memory:.2f}m "
                f"u_locked={self._u_locked} v_locked={self._v_locked} "
                f"setpoint=({target_wx:.2f},{target_wy:.2f})"
            )
        return True
