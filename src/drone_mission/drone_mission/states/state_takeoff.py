"""
TAKEOFF 状态 — 垂直起飞至巡航高度 (5m).

流程:
  1. 设置 Offboard 模式
  2. 解锁
  3. 发布当前位置 + 目标高度的 setpoint
  4. 等待到达巡航高度
  5. 到达 → TRANSIT_TO_DROP_ZONE
"""

import time
from typing import Optional


class TakeoffState:
    """
    起飞状态处理器 — 非阻塞异步实现.

    子阶段:
      SUB_ARM:        发送解锁指令, 等待 PX4 反馈
      SUB_OFFBOARD:   切换 Offboard 模式
      SUB_CLIMB:      发布爬升 setpoint, 等待到达巡航高度
    """

    SUB_ARM = 0
    SUB_OFFBOARD = 1
    SUB_CLIMB = 2

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._sub_phase: int = self.SUB_ARM
        self._sub_phase_start: float = 0.0
        self._stable_count: int = 0
        self._arm_cmd_sent: bool = False
        self._offboard_cmd_sent: bool = False

    def enter(self):
        self._fsm.logger.log_state("ARM", "TAKEOFF", "开始起飞")
        self._start_time = time.time()
        self._sub_phase = self.SUB_ARM
        self._sub_phase_start = time.time()
        self._stable_count = 0
        self._arm_cmd_sent = False
        self._offboard_cmd_sent = False

        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        self._fsm.get_logger().info(f"TAKEOFF: 目标高度 {takeoff_h}m")

    def execute(self) -> Optional[str]:
        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        elapsed = time.time() - self._start_time
        sub_elapsed = time.time() - self._sub_phase_start

        # === SUB_ARM: 发送解锁指令, 等待 PX4 反馈 ===
        if self._sub_phase == self.SUB_ARM:
            if not self._arm_cmd_sent:
                self._fsm.get_logger().info("TAKEOFF: 发送解锁指令...")
                self._fsm.bridge.arm()
                self._arm_cmd_sent = True
                self._sub_phase_start = time.time()

            # 等待 PX4 反馈 arm 状态 (非阻塞, 每次 execute 检查)
            if self._fsm.bridge.is_armed():
                self._fsm.get_logger().info("TAKEOFF: 解锁成功, 切换 Offboard")
                self._sub_phase = self.SUB_OFFBOARD
                self._sub_phase_start = time.time()
            elif sub_elapsed > 15.0:
                self._fsm.get_logger().warn("TAKEOFF: 解锁超时, 重试")
                self._arm_cmd_sent = False  # 重新发送
                self._sub_phase_start = time.time()
            return None

        # === SUB_OFFBOARD: 切换 Offboard 模式 ===
        if self._sub_phase == self.SUB_OFFBOARD:
            if not self._offboard_cmd_sent:
                self._fsm.bridge.set_offboard_mode()
                self._offboard_cmd_sent = True
                self._sub_phase_start = time.time()

            # 给 PX4 一点时间切换模式, 然后开始爬升
            if sub_elapsed > 1.5:
                self._fsm.get_logger().info("TAKEOFF: Offboard 模式已设置, 开始爬升")
                self._sub_phase = self.SUB_CLIMB
                self._sub_phase_start = time.time()
                # 发布起飞 setpoint
                self._fsm.fc.takeoff(takeoff_h)
                self._fsm.get_logger().info(
                    f"[PX4] takeoff → NED(0.0, 0.0, {-takeoff_h})"
                )
            return None

        # === SUB_CLIMB: 等待到达巡航高度 ===
        tolerance = self._fsm.get_param("takeoff_tolerance", 0.3)
        pos = self._fsm.bridge.get_current_position()
        target_down = -takeoff_h
        altitude_error = abs(pos.down - target_down)

        if altitude_error < tolerance:
            self._stable_count += 1
        else:
            self._stable_count = 0

        # 持续更新 setpoint
        self._fsm.fc.takeoff(takeoff_h)

        if elapsed > 2.0 and int(elapsed) % 5 == 0:
            armed = self._fsm.bridge.is_armed()
            self._fsm.get_logger().info(
                f"TAKEOFF: alt_error={altitude_error:.2f}m "
                f"stable={self._stable_count} armed={armed}"
            )

        if self._stable_count >= 40:  # 20Hz × 2s
            self._fsm.get_logger().info(f"TAKEOFF: 到达巡航高度 {takeoff_h}m")
            return "TRANSIT_TO_DROP_ZONE"

        if sub_elapsed > 30.0:
            self._fsm.get_logger().warn("TAKEOFF: 爬升超时, 强制进入下一状态")
            return "TRANSIT_TO_DROP_ZONE"

        return None

    def exit(self):
        self._fsm.logger.log_state("TAKEOFF", "TRANSIT_TO_DROP_ZONE", "起飞完成")
