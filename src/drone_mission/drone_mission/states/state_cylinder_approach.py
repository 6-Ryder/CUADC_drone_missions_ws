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
        cylinders = self._fsm.get_known_cylinders() if hasattr(self._fsm, "get_known_cylinders") else self._fsm.get_cylinders()
        if not cylinders:
            self._fsm.get_logger().warn("APPROACH: 无可用目标")
            return

        # 选择器已经记录了上次选择的位置, 用位置找最近的圆柱
        if self._fsm.cylinder_selector._selected_positions:
            last_pos = self._fsm.cylinder_selector._selected_positions[-1]
            self._target_world_xy = last_pos
        else:
            # 回退: 直接选第一个圆柱
            c = cylinders[0]
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
        """Use current pixel error to move the drone over the detected cylinder."""
        from px4_control.telemetry import ned_to_world, world_to_ned

        du, dv = self._pixel_error(cyl)
        pos = self._fsm.bridge.get_current_position()
        drone_wx, drone_wy, _ = ned_to_world(pos.north, pos.east, pos.down)
        alt = max(abs(pos.down), 0.5)
        fx = float(self._fsm.get_param("camera_fx", 539.9))
        fy = float(self._fsm.get_param("camera_fy", 539.9))

        # 相机近似垂直向下: u→world x/east, v→world y/north.
        target_wx = drone_wx + du * alt / fx
        target_wy = drone_wy + dv * alt / fy
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
        if not self._target_world_xy or not cylinders:
            return None
        tx, ty = self._target_world_xy
        best, best_dist = None, float('inf')
        for c in cylinders:
            cx = c.center_x if hasattr(c, 'center_x') else c.get('center_x', 0)
            cy = c.center_y if hasattr(c, 'center_y') else c.get('center_y', 0)
            d = math.hypot(cx - tx, cy - ty)
            if d < best_dist and d < 2.0:  # 2m 内认为是同一圆筒
                best, best_dist = c, d
        return best

    def execute(self) -> Optional[str]:
        if self._target_world_xy is None:
            return "ASCEND"

        cylinders = self._fsm.get_cylinders()
        target_cyl = self._find_nearest_cylinder(cylinders)

        if target_cyl is None:
            self._target_lost_count += 1
            try:
                error = self._goto_target_xy(speed=0.5)
            except ValueError as exc:
                self._fsm.get_logger().error(f"APPROACH: {exc}, 放弃当前目标")
                return "ASCEND"
            if error < 0.25:
                self._memory_arrival_count += 1
            if self._target_lost_count == 1:
                self._fsm.get_logger().info("APPROACH: 当前帧未见目标, 只使用记忆坐标粗靠近")
            if self._memory_arrival_count >= 20:
                self._fsm.get_logger().warn("APPROACH: 到达记忆坐标但未重新看见目标, 放弃投放")
                return "ASCEND"
            if time.time() - self._start_time > 25.0:
                self._fsm.get_logger().warn("APPROACH: 目标未在当前视场恢复, 放弃投放")
                return "ASCEND"
            return None
        self._target_lost_count = 0

        cx = target_cyl.center_x if hasattr(target_cyl, 'center_x') else target_cyl.get('center_x', 0)
        cy = target_cyl.center_y if hasattr(target_cyl, 'center_y') else target_cyl.get('center_y', 0)
        # 更新目标位置 (视觉检测可能有漂移)
        self._target_world_xy = (cx, cy)

        pixel_error = self._goto_pixel_centered_target(
            target_cyl, speed=0.6 if self._align_stable_count == 0 else 0.25
        )
        tolerance_px = float(self._fsm.get_param("drop_align_pixel_tolerance", 35.0))

        if pixel_error > tolerance_px:
            self._align_stable_count = 0
        else:
            self._align_stable_count += 1

        elapsed = time.time() - self._start_time
        if elapsed > 2.0 and elapsed % 3.0 < 0.2:
            self._fsm.get_logger().info(
                f"APPROACH: pixel_error={pixel_error:.1f}px stable={self._align_stable_count}"
            )

        if self._align_stable_count >= 20:
            self._fsm.get_logger().info(f"APPROACH: 像素对心完成 (error={pixel_error:.1f}px)")
            return "DESCEND_TO_DROP"

        if elapsed > 30.0:
            self._fsm.get_logger().warn("APPROACH: 像素对心超时, 放弃当前目标")
            return "ASCEND"

        return None

    def exit(self):
        self._fsm.logger.log_state("APPROACH", "DESCEND_TO_DROP",
                                   f"逼近完成, 目标={self._target_world_xy}")
