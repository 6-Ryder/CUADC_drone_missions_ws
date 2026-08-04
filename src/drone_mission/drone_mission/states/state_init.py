"""
INIT 状态 — 等待 PX4 连接 + 数据预热.

功能:
  1. 等待 /fmu/out/vehicle_odometry 有有效数据
  2. 等待 /drone/vision 各话题就绪
  3. 初始参数预热
  4. 注册完成 → 跳转 ARM
"""

import time
from typing import Optional


class InitState:
    """
    INIT 状态处理器.

    条件:
      - odom 位置标准差稳定 (< 0.1m 持续 2s)
      - 视觉管线节点心跳正常
      - 舵机节点心跳正常
    """

    def __init__(self, fsm):
        self._fsm = fsm
        self._start_time: float = 0.0
        self._odom_ready_count: int = 0
        self._odom_stable_threshold: int = 20  # 20 帧 ≈ 1s
        self._min_wait_time: float = 2.0  # 最小预热时间

    def enter(self):
        self._fsm.logger.log_state("INIT", "INIT", "开始初始化")
        self._start_time = time.time()
        self._odom_ready_count = 0
        self._fsm.get_logger().info("INIT: 等待系统就绪...")

    def execute(self) -> Optional[str]:
        """返回下一状态名称, 或 None 保持当前状态."""
        # 原点是合法位置；必须以真实 /drone/pose 回调作为就绪依据。
        if self._fsm.odom_ready:
            self._odom_ready_count += 1
        else:
            self._odom_ready_count = 0

        # 视觉/舵机节点检查
        telemetry_ready = self._fsm.bridge.telemetry_fresh(time.monotonic())
        vision_alive = self._fsm.watchdog.is_alive("vision")
        servo_alive = self._fsm.watchdog.is_alive("servo")
        vision_ready = self._fsm.vision_ready
        battery_ready = self._fsm.battery_ready
        land_ready = self._fsm.land_ready
        rc_ready = self._fsm.rc_ready

        elapsed = time.time() - self._start_time

        missing = []
        if not telemetry_ready:
            missing.append("telemetry")
        if self._odom_ready_count < self._odom_stable_threshold:
            missing.append("odom")
        if not battery_ready:
            missing.append("battery")
        if not land_ready:
            missing.append("land_status")
        if not vision_alive or not vision_ready:
            missing.append("vision")
        if not servo_alive or not self._fsm.servo_ready:
            missing.append("servo")
        if not rc_ready:
            missing.append("rc")

        # 诊断日志 — RC 模式时显示 armed/preflight 明细
        if elapsed - getattr(self, '_last_diag_log', 0) > 5.0:
            self._last_diag_log = elapsed
            require_rc = self._fsm.get_param("rc_require_arm_signal", True)
            if require_rc:
                armed = self._fsm.bridge.is_armed()
                preflight = self._fsm.bridge.preflight_ok
                rc_detail = f"armed={'Y' if armed else 'N'} preflight={'Y' if preflight else 'N'}"
            else:
                rc_detail = "skip"
            self._fsm.get_logger().info(
                f"INIT: waiting={','.join(missing) or 'none'}, "
                f"odom={self._odom_ready_count}, "
                f"telemetry={'OK' if telemetry_ready else 'WAIT'}, "
                f"vision={'OK' if vision_alive else 'WAIT'}, "
                f"vision_ready={'OK' if vision_ready else 'WAIT'}, "
                f"servo={'OK' if servo_alive else 'WAIT'}, "
                f"battery={'OK' if battery_ready else 'WAIT'}, "
                f"land={'OK' if land_ready else 'WAIT'}, "
                f"rc=[{rc_detail}], "
                f"elapsed={elapsed:.0f}s"
            )

        # 就绪条件
        if (
            self._odom_ready_count >= self._odom_stable_threshold
            and telemetry_ready
            and vision_alive
            and vision_ready
            and servo_alive
            and self._fsm.servo_ready
            and battery_ready
            and land_ready
            and rc_ready
        ):
            self._fsm.get_logger().info("INIT: 系统就绪 (含遥控器确认), 进入 ARM")
            return "ARM"

        return None

    def exit(self):
        self._fsm.logger.log_state("INIT", "ARM", "初始化完成")
