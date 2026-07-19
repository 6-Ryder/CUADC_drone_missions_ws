"""
舵机多位置步进投放控制器 — Python 版 (参考, 实际部署用 C++ drone_servo2).

投放序列:
  初始:  1150μs → 闭合, 挂载两枚载荷
  投放1: 1700μs → 释放第一枚
  投放2: 2150μs → 释放第二枚

运行模式:
  - SITL:    ServoMock (仅日志)
  - jetson:  PX4CommandServo (发布 VehicleCommand 到 px4_bridge 中继)

与 C++ drone_servo2 逻辑完全一致, 仅语言不同.
"""

import math
import time

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand

from drone_interfaces.srv import TriggerDrop
from drone_servo.servo_mock import ServoMock
from drone_servo.drop_manager import DropManager, DropState
from drone_utils.logger import TaskLogger


class PX4CommandServo:
    """
    通过 DDS VehicleCommand 控制飞控输出.

    set_pwm(pwm_us) → 归一化 → DO_SET_ACTUATOR → /drone/cmd/vehicle_command_relay
                                                      → px4_bridge → PX4 → MAIN5
    """

    def __init__(self, node: Node, init_pwm_us: float,
                 pwm_min: float, pwm_max: float,
                 actuator_index: int = 0, actuator_set: int = 1):
        self._node = node
        self._current_pwm = init_pwm_us
        self._pwm_min = pwm_min
        self._pwm_max = pwm_max
        self._pwm_center = (pwm_min + pwm_max) / 2.0
        self._pwm_half_range = (pwm_max - pwm_min) / 2.0
        self._actuator_index = actuator_index
        self._actuator_set = actuator_set

        self._cmd_pub = node.create_publisher(
            VehicleCommand, "/drone/cmd/vehicle_command_relay", 10)

        # 初始化: 舵机置为初始位置
        self.set_pwm(init_pwm_us)
        self._ready = True

    def _pwm_to_normalized(self, pwm_us: float) -> float:
        """PWM μs → normalized [-1.0, +1.0]."""
        clamped = max(self._pwm_min, min(self._pwm_max, pwm_us))
        return (clamped - self._pwm_center) / self._pwm_half_range

    def _send_actuator_cmd(self, value: float):
        """发送 DO_SET_ACTUATOR VehicleCommand."""
        cmd = VehicleCommand()
        cmd.timestamp = self._node.get_clock().now().nanoseconds // 1000
        cmd.command = 187  # VEHICLE_CMD_DO_SET_ACTUATOR

        # 将 value 填入对应的 param 槽位, 其余为 NaN
        params = [float('nan')] * 7
        params[self._actuator_index] = value
        cmd.param1 = params[0]
        cmd.param2 = params[1]
        cmd.param3 = params[2]
        cmd.param4 = params[3]
        cmd.param5 = float(params[4])
        cmd.param6 = float(params[5])
        cmd.param7 = float(self._actuator_set)

        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 100
        cmd.from_external = True

        self._cmd_pub.publish(cmd)

    def set_pwm(self, pwm_us: float) -> bool:
        """设置 PWM 输出."""
        if not self._ready:
            return False
        self._current_pwm = pwm_us
        self._send_actuator_cmd(self._pwm_to_normalized(pwm_us))
        return True

    def is_ready(self) -> bool:
        return self._ready

    def reset(self):
        self.set_pwm(self._pwm_min)

    @property
    def current_pwm(self) -> float:
        return self._current_pwm

    @property
    def status_dict(self) -> dict:
        return {
            "pwm_us": self._current_pwm,
            "ready": 1.0 if self._ready else 0.0,
        }


class ServoController(Node):
    """舵机多位置步进投放控制器 ROS2 节点."""

    def __init__(self):
        super().__init__("servo_controller")

        self._declare_params()

        platform = self.get_parameter("platform").value
        is_sitl = (platform == "sitl")

        positions = self.get_parameter("pwm_positions").value
        if len(positions) < 2:
            self.get_logger().error("pwm_positions 至少需要 2 个值")
            return

        init_pwm = positions[0]
        pwm_min = positions[0]
        pwm_max = positions[-1]

        if is_sitl:
            self._servo = ServoMock(init_pwm)
            self.get_logger().info("舵机: SITL Mock 模式")
        else:
            self._servo = PX4CommandServo(
                self, init_pwm, pwm_min, pwm_max,
                actuator_index=self.get_parameter("actuator_index").value,
                actuator_set=self.get_parameter("actuator_set").value,
            )
            self.get_logger().info(
                f"舵机: PX4 VehicleCommand 模式 "
                f"(set={self.get_parameter('actuator_set').value} "
                f"idx={self.get_parameter('actuator_index').value} "
                f"pwm=[{pwm_min:.0f}..{pwm_max:.0f}] init={init_pwm:.0f}μs)"
            )

        self._drop_manager = DropManager(
            servo=self._servo,
            positions=positions,
            delay_ms=self.get_parameter("drop_delay_ms").value,
            timeout_s=self.get_parameter("drop_timeout_s").value,
        )
        self._drop_manager.set_on_drop_done(self._on_drop_done)

        self._task_logger = TaskLogger(node_name="servo")

        self._drop_srv = self.create_service(
            TriggerDrop, "/drone/servo/trigger_drop", self._handle_drop_request,
        )

        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(f"投放序列 ({self._drop_manager.max_drops} 次):")
        for i, p in enumerate(positions):
            tag = " ← 初始/闭合" if i == 0 else ""
            self.get_logger().info(f"  [{i}] {p:.0f} μs{tag}")
        self.get_logger().info("舵机投放服务已就绪")

    def _declare_params(self):
        self.declare_parameter("platform", "sitl")
        self.declare_parameter("pwm_positions", [1150.0, 1700.0, 2150.0])
        self.declare_parameter("actuator_index", 0)
        self.declare_parameter("actuator_set", 1)
        self.declare_parameter("drop_delay_ms", 1000.0)
        self.declare_parameter("drop_timeout_s", 3.0)

    def _handle_drop_request(self, request: TriggerDrop.Request,
                             response: TriggerDrop.Response):
        """处理投放请求."""
        if not self._drop_manager.can_drop():
            self.get_logger().warn(
                f"投放请求被拒绝: state={self._drop_manager.state.value} "
                f"count={self._drop_manager.drop_count}/{self._drop_manager.max_drops} "
                f"pwm={self._drop_manager.current_pwm:.0f}μs"
            )
            response.success = False
            response.message = f"Cannot drop: {self._drop_manager.state.value}"
            return response

        target = self._drop_manager.positions[self._drop_manager.drop_count + 1]
        self.get_logger().info(
            f"投放 #{self._drop_manager.drop_count + 1}: "
            f"cyl={request.cylinder_id} "
            f"{self._drop_manager.current_pwm:.0f} → {target:.0f} μs"
        )

        success = self._drop_manager.execute_drop(request.cylinder_id)
        response.success = success
        response.message = "Drop OK" if success else "Drop FAILED"
        return response

    def _on_drop_done(self, success: bool, cylinder_id: int):
        """投放完成回调."""
        status = "成功" if success else "失败"
        self.get_logger().info(
            f"投放 #{self._drop_manager.drop_count} 完成: {status} "
            f"(cyl={cylinder_id}) pwm={self._drop_manager.current_pwm:.0f}μs"
        )
        self._task_logger.log_drop(cylinder_id, success)

    def _publish_status(self):
        """1Hz 状态发布."""
        st = self._servo.status_dict
        self.get_logger().debug(
            f"舵机状态: pwm={st['pwm_us']:.0f}μs ready={st['ready']:.0f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
