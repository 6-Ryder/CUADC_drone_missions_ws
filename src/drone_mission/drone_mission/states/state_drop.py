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

import time
import math
from typing import Optional


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

        if self._sub_state == "DESCEND_TO_DROP":
            self._enter_descend()
        elif self._sub_state == "EXECUTE_DROP":
            self._enter_drop()
        elif self._sub_state == "ASCEND":
            self._enter_ascend()

    def _enter_descend(self):
        drop_h = self._fsm.get_param("drop_descend_height", 2.0)
        self._fsm.get_logger().info(f"DESCEND: 降高至 {drop_h}m")

        # 保持水平位置, 降高
        pos = self._fsm.bridge.get_current_position()
        self._fsm.fc.goto_ned(pos.north, pos.east, -drop_h, speed=0.5)

    def _enter_drop(self):
        self._fsm.get_logger().info("DROP: 执行投放")
        self._fsm.logger.log_state("DESCEND_TO_DROP", "EXECUTE_DROP", "开始投放")

        # 调用舵机投放服务
        cyl_id = self._fsm._current_drop_cyl_id if hasattr(self._fsm, '_current_drop_cyl_id') else -1
        if hasattr(self._fsm, "start_drop_service"):
            self._fsm.start_drop_service(cyl_id)
            self._drop_executed = True
            self._fsm.get_logger().info(f"DROP: 已发送投放请求 cyl={cyl_id}")
        else:
            self._drop_success = self._fsm.call_drop_service(cyl_id)
            self._drop_executed = True

    def _enter_ascend(self):
        self._fsm.logger.log_state("EXECUTE_DROP", "ASCEND", "回升")
        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        self._fsm.get_logger().info(f"ASCEND: 回升至巡航高度 {takeoff_h}m")

        pos = self._fsm.bridge.get_current_position()
        self._fsm.fc.goto_ned(pos.north, pos.east, -takeoff_h, speed=1.0)

    def execute(self) -> Optional[str]:
        elapsed = time.time() - self._start_time

        if self._sub_state == "DESCEND_TO_DROP":
            return self._execute_descend()
        elif self._sub_state == "EXECUTE_DROP":
            return self._execute_drop()
        elif self._sub_state == "ASCEND":
            return self._execute_ascend()

        return None

    def _execute_descend(self) -> Optional[str]:
        drop_h = self._fsm.get_param("drop_descend_height", 2.0)
        pos = self._fsm.bridge.get_current_position()

        if abs(pos.down + drop_h) < 0.3:
            if self._drop_target_is_centered():
                self._fsm.get_logger().info("DESCEND: 到达投放高度且目标居中")
                return "EXECUTE_DROP"
            self._fsm.get_logger().warn("DESCEND: 到达投放高度但目标未居中/不可见, 放弃本次投放")
            return "ASCEND"

        if time.time() - self._start_time > 15.0:
            self._fsm.get_logger().warn("DESCEND: 超时, 强制投放")
            return "EXECUTE_DROP"

        return None

    def _drop_target_is_centered(self) -> bool:
        """投放许可: 当前帧必须看到目标, 且像素中心接近画面中心."""
        cylinders = self._fsm.get_cylinders()
        if not cylinders:
            return False

        selected_positions = getattr(self._fsm.cylinder_selector, "_selected_positions", [])
        if not selected_positions:
            return False

        tx, ty = selected_positions[-1]
        best = None
        best_dist = float("inf")
        for c in cylinders:
            cx = getattr(c, "center_x", 0.0)
            cy = getattr(c, "center_y", 0.0)
            dist = math.hypot(cx - tx, cy - ty)
            if dist < best_dist:
                best = c
                best_dist = dist

        if best is None or best_dist > 1.0:
            return False

        u = getattr(best, "center_u", 0.0)
        v = getattr(best, "center_v", 0.0)
        img_cx = self._fsm.get_param("camera_cx", 640.0)
        img_cy = self._fsm.get_param("camera_cy", 480.0)
        pixel_error = math.hypot(u - img_cx, v - img_cy)
        tolerance = self._fsm.get_param("drop_align_pixel_tolerance", 35.0)
        if pixel_error > tolerance:
            self._fsm.get_logger().warn(
                f"DESCEND: 投放许可失败 pixel_error={pixel_error:.1f}px > {tolerance:.1f}px"
            )
            return False

        self._fsm.get_logger().info(
            f"DESCEND: 投放许可通过 pixel_error={pixel_error:.1f}px dist={best_dist:.2f}m"
        )
        return True

    def _execute_drop(self) -> Optional[str]:
        # 等待投放确认
        timeout = self._fsm.get_param("drop_confirmation_timeout", 3.0)
        if hasattr(self._fsm, "check_drop_service_result"):
            result = self._fsm.check_drop_service_result()
            if result is not None:
                self._drop_success = result
                if self._drop_success:
                    self._fsm.get_logger().info("DROP: 投放成功")
                else:
                    self._fsm.get_logger().warn("DROP: 投放失败")
                return "ASCEND"

        if time.time() - self._start_time > timeout:
            self._fsm.get_logger().warn("DROP: 投放确认超时")
            return "ASCEND"
        return None

    def _execute_ascend(self) -> Optional[str]:
        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        pos = self._fsm.bridge.get_current_position()

        if abs(pos.down + takeoff_h) < 0.5:
            self._fsm.get_logger().info("ASCEND: 回到巡航高度")
            # 判断是否需要继续投放
            drops = self._fsm.drop_count
            max_drops = self._fsm.get_param("max_drops", 2)
            if drops < max_drops:
                return "SELECT_TARGET"
            else:
                return "TRANSIT_TO_RECON_ZONE"

        if time.time() - self._start_time > 15.0:
            drops = self._fsm.drop_count
            max_drops = self._fsm.get_param("max_drops", 2)
            return "SELECT_TARGET" if drops < max_drops else "TRANSIT_TO_RECON_ZONE"

        return None

    def exit(self):
        self._fsm.logger.log_state(self._sub_state, "NEXT",
                                   f"投放 #{self._fsm.drop_count} 完成")
