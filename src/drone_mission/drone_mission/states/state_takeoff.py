"""
TAKEOFF 状态 — 垂直起飞至巡航高度 (5m).

流程:
  1. 解锁 (RC 模式: 等待遥控器已解锁; 仿真模式: 代码发送 arm)
  2. 设置 Offboard 模式
  3. 发布当前位置 + 目标高度的 setpoint
  4. 等待到达巡航高度
  5. 到达 → TRANSIT_TO_DROP_ZONE

安全:
  - rc_require_arm_signal=True: 代码绝不发送 arm 指令,
    仅等待 PX4 is_armed() 由遥控器触发.
  - rc_require_arm_signal=False: 仿真/无遥控器场景, 代码主动 arm.
"""

import time
from typing import Optional

from drone_utils.progress_monitor import ProgressMonitor

class TakeoffState:
    """
    起飞状态处理器 — 非阻塞异步实现.

    子阶段:
      SUB_ARM:        等待解锁 (RC 模式) 或发送 arm (仿真模式)
      SUB_OFFBOARD:   切换 Offboard 模式
      SUB_CLIMB:      发布爬升 setpoint, 等待到达巡航高度
    """

    SUB_ARM = 0
    SUB_OFFBOARD = 1
    SUB_CLIMB = 2
    OFFBOARD_PRESTREAM_S = 1.0

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._sub_phase: int = self.SUB_ARM
        self._sub_phase_start: float = 0.0
        self._stable_count: int = 0
        self._arm_cmd_sent: bool = False
        self._offboard_cmd_sent: bool = False
        self._offboard_retry_used: bool = False
        self._arm_retry_used: bool = False
        self._climb_progress: Optional[ProgressMonitor] = None
        self._rc_arm_mode: bool = True  # True=等遥控器, False=代码arm
        self._rc_wait_logged: bool = False
        self._offboard_prestream_start: float = 0.0

    def enter(self):
        self._fsm.logger.log_state("ARM", "TAKEOFF", "开始起飞")
        self._start_time = time.time()
        self._sub_phase = self.SUB_ARM
        self._sub_phase_start = time.time()
        self._stable_count = 0
        self._arm_cmd_sent = False
        self._offboard_cmd_sent = False
        self._offboard_retry_used = False
        self._arm_retry_used = False
        self._climb_progress = ProgressMonitor(
            window_s=float(self._fsm.get_param("navigation_progress_window", 30.0)),
            min_improvement=float(
                self._fsm.get_param("navigation_progress_min_improvement", 0.25)
            ),
        )
        self._rc_arm_mode = self._fsm.get_param("rc_require_arm_signal", True)
        self._rc_wait_logged = False
        self._offboard_prestream_start = time.monotonic()

        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        self._fsm.get_logger().info(f"TAKEOFF: 目标高度 {takeoff_h}m")
        # PX4 accepts an Offboard mode request only after receiving a live
        # setpoint stream.  Keep the current position while the pilot arms;
        # this is ignored outside Offboard and becomes the required prestream.
        self._fsm.fc.loiter()

    def execute(self) -> Optional[str]:
        takeoff_h = self._fsm.get_param("takeoff_height", 5.0)
        elapsed = time.time() - self._start_time
        sub_elapsed = time.time() - self._sub_phase_start

        # === SUB_ARM: 等待解锁 (RC 模式) 或发送 arm (仿真模式) ===
        if self._sub_phase == self.SUB_ARM:
            if self._rc_arm_mode:
                # ── RC 模式: 代码绝不发送 arm, 只等遥控器 ──
                if self._fsm.bridge.is_armed():
                    self._fsm.get_logger().info(
                        "TAKEOFF: RC 已解锁 (遥控器 Arm Switch), 切换 Offboard"
                    )
                    self._sub_phase = self.SUB_OFFBOARD
                    self._sub_phase_start = time.time()
                    return None

                if not self._rc_wait_logged:
                    self._rc_wait_logged = True
                    self._fsm.get_logger().info(
                        "TAKEOFF: 等待遥控器解锁... (请拨动 Arm Switch)"
                    )
                return None

            # ── 仿真模式: 代码发送 arm ──
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
                if not self._arm_retry_used:
                    self._arm_retry_used = True
                    self._arm_cmd_sent = False
                    self._sub_phase_start = time.time()
                    self._fsm.get_logger().warn("TAKEOFF: 解锁无反馈, 重发一次")
                else:
                    self._fsm.request_takeover("TAKEOFF: arm made no progress after retry")
            return None

        # === SUB_OFFBOARD: 切换 Offboard 模式 ===
        if self._sub_phase == self.SUB_OFFBOARD:
            if not self._offboard_cmd_sent:
                prestream_elapsed = time.monotonic() - self._offboard_prestream_start
                if prestream_elapsed < self.OFFBOARD_PRESTREAM_S:
                    return None
                self._fsm.bridge.set_offboard_mode()
                self._offboard_cmd_sent = True
                self._sub_phase_start = time.time()
                return None

            if self._fsm.bridge.is_offboard():
                self._fsm.get_logger().info("TAKEOFF: PX4 已确认 Offboard, 开始爬升")
                self._fsm.mark_offboard_confirmed()
                self._sub_phase = self.SUB_CLIMB
                self._sub_phase_start = time.time()
                # 发布起飞 setpoint
                self._fsm.fc.takeoff(takeoff_h)
                self._fsm.get_logger().info(
                    f"[PX4] takeoff → NED(0.0, 0.0, {self._fsm.fc.altitude_to_ned(takeoff_h)})"
                )
            elif sub_elapsed > 15.0:
                if not self._offboard_retry_used:
                    self._offboard_retry_used = True
                    self._fsm.get_logger().warn(
                        "TAKEOFF: Offboard 未确认, 重发模式请求一次"
                    )
                    self._fsm.bridge.set_offboard_mode()
                    self._sub_phase_start = time.time()
                else:
                    self._fsm.request_takeover(
                        "TAKEOFF: Offboard was not confirmed after retry"
                    )
            return None

        # === SUB_CLIMB: 等待到达巡航高度 ===
        tolerance = self._fsm.get_param("takeoff_tolerance", 0.3)
        pos = self._fsm.bridge.get_current_position()
        target_down = self._fsm.fc.altitude_to_ned(takeoff_h)
        altitude_error = abs(pos.down - target_down)

        if altitude_error < tolerance:
            self._stable_count += 1
        else:
            self._stable_count = 0

        # 持续更新 setpoint
        self._fsm.fc.takeoff(takeoff_h)

        if self._climb_progress is not None:
            now = time.monotonic()
            if self._climb_progress.observe(altitude_error, now=now):
                if self._climb_progress.mark_retry(now=now):
                    self._fsm.get_logger().warn(
                        "TAKEOFF: 爬升无进展, 重发当前起飞目标一次"
                    )
                    self._fsm.fc.takeoff(takeoff_h)
                else:
                    self._fsm.request_takeover(
                        "TAKEOFF: climb made no progress after retry"
                    )
                    return None

        if elapsed > 2.0 and int(elapsed) % 5 == 0:
            armed = self._fsm.bridge.is_armed()
            self._fsm.get_logger().info(
                f"TAKEOFF: alt_error={altitude_error:.2f}m "
                f"stable={self._stable_count} armed={armed}"
            )

        if self._stable_count >= 40:  # 20Hz × 2s
            self._fsm.get_logger().info(f"TAKEOFF: 到达巡航高度 {takeoff_h}m")
            return "TRANSIT_TO_DROP_ZONE"

        return None

    def exit(self):
        self._fsm.logger.log_state("TAKEOFF", "TRANSIT_TO_DROP_ZONE", "起飞完成")
