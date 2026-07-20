"""
CYLINDER_APPROACH 状态 — 视觉伺服移动到目标圆筒正上方.

流程:
  1. 从 cylinder_selector 获取目标圆筒 ID
  2. 视觉伺服: 将圆筒图像中心对齐到画面中心
  3. 对心完成后 → DESCEND_TO_DROP

包含 SELECT_TARGET 子状态逻辑:
  - 选择半径最小的圆筒
  - 设置目标 ID
"""

import time
import math
from typing import Optional


class CylinderApproachState:
    """圆筒对心逼近状态 — 使用位置匹配而非 ID (vision 每帧重新分配 ID)."""

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._target_world_xy: Optional[tuple] = None  # (world_x, world_y) 目标位置
        self._target_lost_count: int = 0
        self._align_stable_count: int = 0
        self._converged: bool = False
        self._memory_arrival_count: int = 0

    def enter(self):
        self._fsm.logger.log_state("SELECT_TARGET", "APPROACH", "开始逼近圆筒")
        self._start_time = time.time()
        self._align_stable_count = 0
        self._target_lost_count = 0
        self._converged = False
        self._memory_arrival_count = 0

        # 从 SELECT_TARGET 已选中的目标获取世界坐标
        if hasattr(self._fsm, "get_known_drop_targets"):
            targets = self._fsm.get_known_drop_targets()
        else:
            targets = self._fsm.get_cylinders()
        if not targets:
            self._fsm.get_logger().warn("APPROACH: 无可用目标")
            return

        # 选择器已经记录了上次选择的位置, 用位置找最近的圆柱
        if self._fsm.cylinder_selector._selected_positions:
            last_pos = self._fsm.cylinder_selector._selected_positions[-1]
            self._target_world_xy = last_pos
        else:
            # 回退: 直接选第一个圆柱
            c = targets[0]
            cx = c.center_x if hasattr(c, 'center_x') else c.get('center_x', 0)
            cy = c.center_y if hasattr(c, 'center_y') else c.get('center_y', 0)
            self._target_world_xy = (cx, cy)

        self._fsm.get_logger().info(
            f"APPROACH: 目标位置 World({self._target_world_xy[0]:.2f}, {self._target_world_xy[1]:.2f})"
        )
        if hasattr(self._fsm, "is_in_drop_zone_world"):
            if not self._fsm.is_in_drop_zone_world(
                self._target_world_xy[0], self._target_world_xy[1], margin=0.5
            ):
                self._fsm.get_logger().error(
                    "APPROACH: 目标坐标不在投放区内, 拒绝逼近 "
                    f"World({self._target_world_xy[0]:.2f}, {self._target_world_xy[1]:.2f})"
                )
                self._target_world_xy = None

    def _goto_target_xy(self, speed: float) -> float:
        """飞到记忆目标正上方, 返回水平误差."""
        from px4_control.telemetry import ned_to_world, world_to_ned

        tx, ty = self._target_world_xy
        if hasattr(self._fsm, "is_in_drop_zone_world"):
            if not self._fsm.is_in_drop_zone_world(tx, ty, margin=0.5):
                raise ValueError(f"target outside drop zone: ({tx:.2f}, {ty:.2f})")
        pos = self._fsm.bridge.get_current_position()
        drone_wx, drone_wy, _ = ned_to_world(pos.north, pos.east, pos.down)
        error = math.hypot(drone_wx - tx, drone_wy - ty)

        tn, te, _ = world_to_ned(tx, ty, 0)
        alt = self._fsm.get_param("area1_search_altitude", 5.0)
        self._fsm.fc.goto_ned(tn, te, -alt, speed=speed)
        return error

    @staticmethod
    def _field(obj, key: str, default=0.0):
        return getattr(obj, key, obj.get(key, default) if isinstance(obj, dict) else default)

    def _pixel_error(self, cyl) -> tuple:
        u = float(self._field(cyl, "center_u", 0.0))
        v = float(self._field(cyl, "center_v", 0.0))
        img_cx = float(self._fsm.get_param("camera_cx", 640.0))
        img_cy = float(self._fsm.get_param("camera_cy", 480.0))
        return u - img_cx, v - img_cy

    def _goto_pixel_centered_target(self, cyl, speed: float) -> float:
        """Move the drone over the detected cylinder and return current pixel error."""
        from px4_control.telemetry import ned_to_world, world_to_ned

        du, dv = self._pixel_error(cyl)
        pos = self._fsm.bridge.get_current_position()
        drone_wx, drone_wy, _ = ned_to_world(pos.north, pos.east, pos.down)
        alt = max(abs(pos.down), 0.5)
        fx = float(self._fsm.get_param("camera_fx", 539.9))
        fy = float(self._fsm.get_param("camera_fy", 539.9))
        gain = float(self._fsm.get_param("cylinder_approach_pixel_gain", 0.65))
        max_step = float(self._fsm.get_param("cylinder_approach_max_step", 0.45))
        step_x = max(-max_step, min(max_step, gain * du * alt / fx))
        step_y = max(-max_step, min(max_step, gain * dv * alt / fy))
        target_wx = drone_wx + step_x
        target_wy = drone_wy + step_y

        if hasattr(self._fsm, "is_in_drop_zone_world"):
            if not self._fsm.is_in_drop_zone_world(target_wx, target_wy, margin=0.75):
                # 不让像素伺服把飞机带出投放区, 但仍返回误差用于日志。
                target_wx, target_wy = self._target_world_xy

        tn, te, _ = world_to_ned(target_wx, target_wy, 0.0)
        hold_alt = self._fsm.get_param("area1_search_altitude", 5.0)
        self._fsm.fc.goto_ned(tn, te, -hold_alt, speed=speed)
        return (du * du + dv * dv) ** 0.5

    def _find_nearest_cylinder(self, cylinders: list) -> Optional[object]:
        """用位置匹配找最近的圆柱 (不用 ID, 因为 vision 每帧重新分配)."""
        return self._find_nearest_target(cylinders, max_dist=2.0)

    def _find_nearest_target(self, targets: list, max_dist: float = 2.0) -> Optional[object]:
        if not self._target_world_xy or not targets:
            return None
        tx, ty = self._target_world_xy
        best, best_dist = None, float('inf')
        for c in targets:
            cx = c.center_x if hasattr(c, 'center_x') else c.get('center_x', 0)
            cy = c.center_y if hasattr(c, 'center_y') else c.get('center_y', 0)
            d = math.hypot(cx - tx, cy - ty)
            if d < best_dist and d < max_dist:
                best, best_dist = c, d
        return best

    def _find_pixel_centered_cylinder(self, targets: list) -> Optional[object]:
        """World 匹配失败时, 选当前画面中最接近中心的目标作为视觉伺服目标."""
        if not targets:
            return None

        best = None
        best_err = float("inf")
        for c in targets:
            conf = float(self._field(c, "confidence", 0.0))
            if conf < 0.25:
                continue
            du, dv = self._pixel_error(c)
            err = (du * du + dv * dv) ** 0.5
            if err < best_err:
                best = c
                best_err = err
        return best

    def execute(self) -> Optional[str]:
        if self._target_world_xy is None:
            self._fsm.get_logger().error("APPROACH: 没有有效投放目标")
            return "ERROR"

        cylinders = self._fsm.get_cylinders()
        markers = self._fsm.get_markers() if hasattr(self._fsm, "get_markers") else []
        target_cyl = self._find_nearest_target(cylinders, max_dist=2.0)
        if target_cyl is None:
            target_cyl = self._find_nearest_target(markers, max_dist=2.5)

        try:
            speed = (
                float(self._fsm.get_param("cylinder_approach_slow_speed", 0.15))
                if target_cyl is not None
                else float(self._fsm.get_param("cylinder_approach_memory_speed", 0.25))
            )
            error = self._goto_target_xy(speed=speed)
        except ValueError as exc:
            self._fsm.get_logger().error(f"APPROACH: {exc}")
            return "ERROR"

        tolerance_px = float(self._fsm.get_param("drop_align_pixel_tolerance", 35.0))
        pixel_error = float("inf")

        if target_cyl is not None:
            self._target_lost_count = 0
            du, dv = self._pixel_error(target_cyl)
            pixel_error = (du * du + dv * dv) ** 0.5
            if pixel_error > tolerance_px:
                self._align_stable_count = 0
            else:
                self._align_stable_count += 1
        else:
            self._target_lost_count += 1
            self._align_stable_count = 0
            if self._target_lost_count == 1:
                self._fsm.get_logger().info("APPROACH: 当前帧未见目标, 只飞向静态记忆坐标")

        if error < 0.30:
            self._memory_arrival_count += 1
        else:
            self._memory_arrival_count = 0

        elapsed = time.time() - self._start_time
        if elapsed > 2.0 and elapsed % 3.0 < 0.2:
            self._fsm.get_logger().info(
                f"APPROACH: target_error={error:.2f}m pixel_error={pixel_error:.1f}px "
                f"stable={self._align_stable_count}"
            )

        if self._align_stable_count >= 10:
            self._fsm.get_logger().info(f"APPROACH: 当前视觉目标已居中 (error={pixel_error:.1f}px)")
            return "DESCEND_TO_DROP"

        if self._memory_arrival_count >= 20:
            self._fsm.get_logger().warn("APPROACH: 到达静态目标点, 进入低空视觉复查")
            return "DESCEND_TO_DROP"

        if elapsed > 30.0:
            self._fsm.get_logger().error("APPROACH: 逼近超时")
            return "ERROR"

        return None

    def exit(self):
        self._fsm.logger.log_state("APPROACH", "DESCEND_TO_DROP",
                                   f"逼近完成, 目标={self._target_world_xy}")
