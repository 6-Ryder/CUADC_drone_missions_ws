#!/usr/bin/env python3
"""
Mock PX4 ACK — 无需飞控, 欺骗 servo_controller 完成完整状态机流转.

原理:
  servo_controller(jetson) 发 VehicleCommand 到 /drone/cmd/vehicle_command_relay
  → 本脚本监听 → 伪造 VehicleCommandAck 回 /fmu/out/vehicle_command_ack_v1
  → servo_controller 收到 ACK → ready=true → 允许投放

这样在没有 PX4 / SITL 的情况下, 也能端到端测试投放序列:
  初始闭合(1150μs) → 投放#1(1700μs) → 投放#2(2150μs)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleCommand, VehicleCommandAck


class MockPX4Ack(Node):
    def __init__(self):
        super().__init__("mock_px4_ack")

        # 订阅 servo_controller 发出的 VehicleCommand
        self._cmd_sub = self.create_subscription(
            VehicleCommand,
            "/drone/cmd/vehicle_command_relay",
            self._on_cmd,
            10,
        )

        # 发布伪造 ACK (SensorDataQoS 以匹配 servo_controller 的订阅 QoS)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._ack_pub = self.create_publisher(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack_v1",
            sensor_qos,
        )

        self._ack_count = 0
        self.get_logger().info("Mock PX4 ACK 就绪 — 模拟飞控回复 DO_SET_ACTUATOR")

    def _on_cmd(self, cmd: VehicleCommand):
        if cmd.command != VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR:
            return

        # 提取 PWM 归一化值
        pwm_norm = cmd.param1

        self._ack_count += 1
        ack = VehicleCommandAck()
        ack.timestamp = self.get_clock().now().nanoseconds // 1000
        ack.command = cmd.command
        ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        ack.result_param1 = 0
        ack.result_param2 = 0
        ack.target_system = cmd.source_system
        ack.target_component = cmd.source_component  # = 100
        ack.from_external = True

        self._ack_pub.publish(ack)
        self.get_logger().info(
            f"ACK #{self._ack_count}: cmd=DO_SET_ACTUATOR "
            f"pwm_norm={pwm_norm:.4f} "
            f"→ result=ACCEPTED"
        )


def main():
    rclpy.init()
    node = MockPX4Ack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
