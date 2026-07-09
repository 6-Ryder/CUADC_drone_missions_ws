"""
舵机投放控制器 — GPIO/PWM 舵机控制 ROS2 节点.

运行模式:
  - SITL: 使用 ServoMock, 仅记录日志, 模拟投放动作
  - Jetson: 使用 Jetson.GPIO 或 PCA9685 PWM 驱动板

提供 ROS2 Service:
  /drone/servo/trigger_drop  (TriggerDrop.srv)

发布:
  /drone/servo/status        (ServoStatus — 自定义消息, 通过日志替代)
"""

import time

import rclpy
from rclpy.node import Node

from drone_interfaces.srv import TriggerDrop
from drone_servo.servo_mock import ServoMock
from drone_servo.drop_manager import DropManager, DropState
from drone_utils.logger import TaskLogger


class ServoController(Node):
    """
    舵机投放控制器 ROS2 节点.

    封装硬件差异 (SITL vs Jetson), 对外提供统一的服务接口.
    """

    def __init__(self):
        super().__init__("servo_controller")

        self._declare_params()

        # 平台检测
        platform = self.get_parameter("platform").value
        self._is_sitl = (platform == "sitl")

        # 创建舵机实例
        if self._is_sitl:
            self._servo = ServoMock(
                drop_delay_ms=self.get_parameter("drop_delay_ms").value,
                drop_timeout_s=self.get_parameter("drop_timeout_s").value,
            )
            self.get_logger().info("舵机控制器: SITL Mock 模式")
        else:
            # 实飞模式: 使用 Jetson.GPIO 或 PCA9685
            self._servo = self._create_real_servo()
            self.get_logger().info("舵机控制器: 真实硬件模式")

        # 投放管理器
        self._drop_manager = DropManager(
            servo=self._servo,
            max_drops=self.get_parameter("max_drops").value,
            drop_timeout_s=self.get_parameter("drop_timeout_s").value,
        )
        self._drop_manager.set_on_drop_done(self._on_drop_done)

        # 日志
        self._task_logger = TaskLogger(node_name="servo")

        # 服务
        self._drop_srv = self.create_service(
            TriggerDrop, "/drone/servo/trigger_drop", self._handle_drop_request,
        )

        # 状态定时器 (1Hz)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info("舵机投放服务已就绪")

    def _declare_params(self):
        self.declare_parameter("platform", "sitl")
        self.declare_parameter("pwm_pin", 33)
        self.declare_parameter("pwm_frequency", 50)
        self.declare_parameter("open_pulse_us", 1500)
        self.declare_parameter("close_pulse_us", 500)
        self.declare_parameter("drop_delay_ms", 1000)
        self.declare_parameter("drop_timeout_s", 3.0)
        self.declare_parameter("max_drops", 2)

    def _create_real_servo(self):
        """
        创建真实舵机控制器.

        尝试顺序:
          1. PCA9685 (I2C PWM 驱动板)
          2. Jetson.GPIO (直接 GPIO)
          3. RPi.GPIO (树莓派)
        都不可用则回退到 ServoMock.
        """
        try:
            # 尝试 PCA9685
            import board
            import busio
            import adafruit_pca9685

            i2c = busio.I2C(board.SCL, board.SDA)
            pca = adafruit_pca9685.PCA9685(i2c)
            pca.frequency = self.get_parameter("pwm_frequency").value
            self.get_logger().info("舵机硬件: PCA9685 (I2C)")
            return _PCA9685Servo(pca, self)

        except ImportError:
            pass

        try:
            # 尝试 Jetson.GPIO
            import Jetson.GPIO as GPIO
            pin = self.get_parameter("pwm_pin").value
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(pin, GPIO.OUT)
            self.get_logger().info(f"舵机硬件: Jetson.GPIO (pin {pin})")
            return _GPIOServo(GPIO, pin, self)

        except ImportError:
            pass

        try:
            # 尝试 RPi.GPIO
            import RPi.GPIO as GPIO
            pin = self.get_parameter("pwm_pin").value
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.OUT)
            self.get_logger().info(f"舵机硬件: RPi.GPIO (pin {pin})")
            return _GPIOServo(GPIO, pin, self)

        except ImportError:
            pass

        # 无可用的硬件 → 回退到 Mock
        self.get_logger().warn("无可用的 GPIO 库, 回退到 ServoMock")
        return ServoMock(
            drop_delay_ms=self.get_parameter("drop_delay_ms").value,
            drop_timeout_s=self.get_parameter("drop_timeout_s").value,
        )

    # ================================================================
    # 服务处理
    # ================================================================

    def _handle_drop_request(self, request: TriggerDrop.Request,
                             response: TriggerDrop.Response):
        """
        处理投放请求.

        request.cylinder_id: 目标圆筒 ID
        """
        cyl_id = request.cylinder_id

        if not self._drop_manager.can_drop():
            self.get_logger().warn(
                f"投放请求被拒绝: state={self._drop_manager.state.value}, "
                f"drops={self._drop_manager.drop_count}/{self._drop_manager._max_drops}"
            )
            response.success = False
            response.message = f"Cannot drop: {self._drop_manager.state.value}"
            return response

        self.get_logger().info(f"执行投放: cylinder_id={cyl_id}")
        success = self._drop_manager.execute_drop(cyl_id)

        response.success = success
        response.message = "Drop OK" if success else "Drop FAILED"
        return response

    def _on_drop_done(self, success: bool, cylinder_id: int):
        """投放完成回调."""
        status = "成功" if success else "失败"
        self.get_logger().info(
            f"投放 #{self._drop_manager.drop_count} 完成: {status} (cyl={cylinder_id})"
        )
        self._task_logger.log_drop(cylinder_id, success)

    # ================================================================
    # 状态发布
    # ================================================================

    def _publish_status(self):
        """1Hz 发布舵机状态 (日志)."""
        status = self._servo.status_dict
        self._task_logger.log("servo_status", status)
        self.get_logger().debug(
            f"舵机状态: open={status['is_open']}, "
            f"drops={status['drop_count']}, ready={status['ready']}"
        )


# ================================================================
# 真实舵机适配器
# ================================================================

class _PCA9685Servo:
    """PCA9685 PWM 驱动板适配器."""

    def __init__(self, pca, node):
        self._pca = pca
        self._node = node
        self._channel = 0  # 默认通道
        self._open_us = node.get_parameter("open_pulse_us").value
        self._close_us = node.get_parameter("close_pulse_us").value
        self._delay = node.get_parameter("drop_delay_ms").value / 1000.0
        self._is_open = False

    def open(self) -> bool:
        duty = int(self._open_us / 20000.0 * 65535)
        self._pca.channels[self._channel].duty_cycle = duty
        self._is_open = True
        return True

    def close(self) -> bool:
        duty = int(self._close_us / 20000.0 * 65535)
        self._pca.channels[self._channel].duty_cycle = duty
        self._is_open = False
        return True

    def drop(self) -> bool:
        self.open()
        time.sleep(self._delay)
        self.close()
        return True

    def is_ready(self) -> bool:
        return True

    @property
    def status_dict(self) -> dict:
        return {"is_open": self._is_open, "ready": True}


class _GPIOServo:
    """直接 GPIO 适配器."""

    def __init__(self, gpio_module, pin, node):
        self._GPIO = gpio_module
        self._pin = pin
        self._node = node
        self._delay = node.get_parameter("drop_delay_ms").value / 1000.0
        self._is_open = False

    def open(self) -> bool:
        self._GPIO.output(self._pin, self._GPIO.HIGH)
        self._is_open = True
        return True

    def close(self) -> bool:
        self._GPIO.output(self._pin, self._GPIO.LOW)
        self._is_open = False
        return True

    def drop(self) -> bool:
        self.open()
        time.sleep(self._delay)
        self.close()
        return True

    def is_ready(self) -> bool:
        return True

    @property
    def status_dict(self) -> dict:
        return {"is_open": self._is_open, "ready": True}


# ================================================================
# 入口
# ================================================================

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
