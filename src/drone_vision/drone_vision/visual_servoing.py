"""
视觉伺服 (IBVS) — 精准降落 + 圆筒逼近控制.

双模式:
  1. LandingApproach: 跟踪 H 降落标识, 对心 + 下降
  2. CylinderApproach: 跟踪指定圆筒, 水平对心 (不下滑)

基于图像的视觉伺服 (IBVS):
  1. 接收目标像素坐标 (LandingTarget / CylinderDetection)
  2. 计算像素偏差 → 世界偏差 (考虑高度 + 内参)
  3. PID 控制器 → 平滑水平速度修正
  4. 输出速度指令

使用方式:
  # 降落模式
  servo = VisualServoController(...)
  state = servo.update_from_pixels(target_u, target_v, image_cx, image_cy, altitude, fx, fy)
  # 或: 世界坐标模式
  state = servo.update_from_world(target_wx, target_wy, drone_wx, drone_wy)

提供:
  - VisualServoController: 通用 PID 视觉伺服
  - CylinderApproachController: 圆筒对心 (包装类)
  - VisualServoNode: ROS2 节点
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
from collections import deque


@dataclass
class ServoState:
    """视觉伺服状态."""

    velocity_north: float = 0.0   # NED north 速度 (m/s)
    velocity_east: float = 0.0    # NED east 速度 (m/s)
    velocity_down: float = 0.0    # NED down 速度 (m/s, 正=下降)
    yaw_rate: float = 0.0         # 偏航速率 (rad/s)
    horizontal_error: float = 0.0  # 当前水平误差 (m)
    is_converged: bool = False    # 是否已收敛到目标


class VisualServoController:
    """
    基于图像的视觉伺服 (IBVS) 控制器.

    使用 PID 控制, 将像素偏差转换为水平速度修正,
    使无人机逐渐对准降落目标.

    Parameters
    ----------
    kp, ki, kd : float
        PID 增益.
    max_horizontal_vel : float
        最大水平修正速度 (m/s).
    descent_speed : float
        下降速度 (m/s, 正值).
    integral_max : float
        积分项限幅.
    deadzone : float
        目标死区 (m), 进入后停止水平修正.

    """

    def __init__(
        self,
        kp: float = 0.5,
        ki: float = 0.02,
        kd: float = 0.1,
        max_horizontal_vel: float = 1.0,
        descent_speed: float = 0.3,
        integral_max: float = 0.5,
        deadzone: float = 0.03,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_horizontal_vel = max_horizontal_vel
        self.descent_speed = descent_speed
        self.integral_max = integral_max
        self.deadzone = deadzone

        # 内部状态
        self._integral_north = 0.0
        self._integral_east = 0.0
        self._prev_error_north = 0.0
        self._prev_error_east = 0.0
        self._dt = 0.05  # 控制周期 (s)

        # 错误历史 (用于调试)
        self._error_history = deque(maxlen=100)

    def reset(self):
        """重置积分和微分状态."""
        self._integral_north = 0.0
        self._integral_east = 0.0
        self._prev_error_north = 0.0
        self._prev_error_east = 0.0
        self._error_history.clear()

    def update(
        self,
        target_u: float,           # 目标像素 u (图像 x)
        target_v: float,           # 目标像素 v (图像 y)
        image_center_u: float,     # 图像中心 u
        image_center_v: float,     # 图像中心 v
        altitude: float,           # 当前高度 (m)
        focal_length_u: float,     # 焦距 fx (px)
        focal_length_v: float,     # 焦距 fy (px)
        current_yaw: float = 0.0,  # 当前偏航角 (rad)
        target_yaw: Optional[float] = None,  # None=保持当前偏航
    ) -> ServoState:
        """
        更新伺服控制.

        Returns
        -------
        ServoState: 计算出的速度指令和状态.

        """
        # 像素误差
        du = target_u - image_center_u
        dv = target_v - image_center_v

        # 下视相机: +u 是机体右侧, +v 是机体后方。
        body_forward = -dv * altitude / focal_length_v
        body_right = du * altitude / focal_length_u
        error_north = (
            body_forward * np.cos(current_yaw)
            - body_right * np.sin(current_yaw)
        )
        error_east = (
            body_forward * np.sin(current_yaw)
            + body_right * np.cos(current_yaw)
        )

        horizontal_error = np.hypot(error_north, error_east)

        # 死区检查
        in_deadzone = horizontal_error < self.deadzone

        # PID 计算
        p_north = self.kp * error_north
        p_east = self.kp * error_east

        # 积分 (仅在非死区累加)
        if not in_deadzone:
            self._integral_north += self.ki * error_north * self._dt
            self._integral_east += self.ki * error_east * self._dt

        # 积分限幅
        self._integral_north = np.clip(self._integral_north, -self.integral_max, self.integral_max)
        self._integral_east = np.clip(self._integral_east, -self.integral_max, self.integral_max)

        # 微分
        d_north = self.kd * (error_north - self._prev_error_north) / max(self._dt, 0.01)
        d_east = self.kd * (error_east - self._prev_error_east) / max(self._dt, 0.01)

        vel_north = p_north + self._integral_north + d_north
        vel_east = p_east + self._integral_east + d_east

        # 限幅
        vel_north = np.clip(vel_north, -self.max_horizontal_vel, self.max_horizontal_vel)
        vel_east = np.clip(vel_east, -self.max_horizontal_vel, self.max_horizontal_vel)

        # 偏航修正
        yaw_rate = 0.0
        if target_yaw is not None:
            yaw_error = target_yaw - current_yaw
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
            yaw_rate = np.clip(yaw_error * 0.5, -0.5, 0.5)

        # 保存状态
        self._prev_error_north = error_north
        self._prev_error_east = error_east
        self._error_history.append(horizontal_error)

        return ServoState(
            velocity_north=float(vel_north) if not in_deadzone else 0.0,
            velocity_east=float(vel_east) if not in_deadzone else 0.0,
            velocity_down=self.descent_speed,
            yaw_rate=float(yaw_rate),
            horizontal_error=float(horizontal_error),
            is_converged=in_deadzone and self._is_stable(),
        )

    def _is_stable(self, window: int = 10) -> bool:
        """检查最近 N 帧误差是否持续低于死区."""
        if len(self._error_history) < window:
            return False
        recent = list(self._error_history)[-window:]
        return all(e < self.deadzone * 2 for e in recent)


# ============================================================
# 圆筒逼近控制器
# ============================================================

class CylinderApproachController:
    """
    圆筒对心逼近 — 将无人机移动到目标圆筒正上方.

    与降落模式的区别:
      - 不下滑 (velocity_down = 0), 仅水平对心
      - 对心完成后由 mission 节点接管降高
      - 使用相同的 PID 控制但参数更激进
    """

    def __init__(
        self,
        kp: float = 0.8,          # 比降落更激进
        ki: float = 0.03,
        kd: float = 0.15,
        max_horizontal_vel: float = 1.5,
        deadzone: float = 0.05,    # 5cm 对心精度
    ):
        self._servo = VisualServoController(
            kp=kp, ki=ki, kd=kd,
            max_horizontal_vel=max_horizontal_vel,
            descent_speed=0.0,      # 不下滑!
            integral_max=0.5,
            deadzone=deadzone,
        )
        self._target_cylinder_id: Optional[int] = None
        self._is_active = False

    @property
    def is_converged(self) -> bool:
        return self._servo._is_stable(window=15)

    @property
    def horizontal_error(self) -> float:
        if self._servo._error_history:
            return self._servo._error_history[-1]
        return float("inf")

    def activate(self, cylinder_id: int):
        """开始逼近指定圆筒."""
        self._target_cylinder_id = cylinder_id
        self._servo.reset()
        self._is_active = True

    def deactivate(self):
        self._is_active = False
        self._servo.reset()

    def update(
        self,
        cylinder_u: float,
        cylinder_v: float,
        image_center_u: float,
        image_center_v: float,
        altitude: float,
        focal_length_u: float,
        focal_length_v: float,
    ) -> ServoState:
        """
        更新圆筒逼近控制.

        将圆筒中心 (u,v) 与图像中心对齐.
        """
        if not self._is_active:
            return ServoState()

        state = self._servo.update(
            target_u=cylinder_u,
            target_v=cylinder_v,
            image_center_u=image_center_u,
            image_center_v=image_center_v,
            altitude=altitude,
            focal_length_u=focal_length_u,
            focal_length_v=focal_length_v,
            current_yaw=0.0,
            target_yaw=0.0,
        )
        # 覆盖: 圆筒逼近时不下滑
        state.velocity_down = 0.0
        return state


# ============================================================
# 世界坐标伺服 (用于已知目标世界坐标的场景)
# ============================================================

class WorldCoordinateServo:
    """
    世界坐标伺服 — 将无人机移动到指定世界坐标.

    这不是基于图像的伺服, 而是简单的 P 控制器:
      v = Kp * (target - drone) / dt

    用于: 侦察区中飞到已知圆筒位置的初始导航.
    """

    def __init__(
        self,
        kp: float = 0.5,
        max_vel: float = 1.5,
        tolerance: float = 0.1,
    ):
        self.kp = kp
        self.max_vel = max_vel
        self.tolerance = tolerance

    def update(
        self,
        drone_x: float, drone_y: float,
        target_x: float, target_y: float,
    ) -> Tuple[float, float, bool]:
        """
        计算速度指令.

        Returns (vel_x, vel_y, is_arrived).
        """
        error_x = target_x - drone_x
        error_y = target_y - drone_y
        error = np.hypot(error_x, error_y)

        if error < self.tolerance:
            return (0.0, 0.0, True)

        vel_x = np.clip(self.kp * error_x, -self.max_vel, self.max_vel)
        vel_y = np.clip(self.kp * error_y, -self.max_vel, self.max_vel)
        return (vel_x, vel_y, False)


# ============================================================
# ROS2 节点 (可选, 独立的伺服控制节点)
# ============================================================

try:
    import rclpy
    from rclpy.node import Node
    from drone_interfaces.msg import LandingTarget
    from geometry_msgs.msg import TwistStamped

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


if HAS_ROS2:
    class VisualServoNode(Node):
        """视觉伺服 ROS2 节点 — 订阅 LandingTarget, 发布速度指令."""

        def __init__(self):
            super().__init__("visual_servo")

            self._declare_params()
            self._controller = self._build_controller()

            self._current_altitude = 5.0
            self._landing_active = False

            # 订阅 LandingTarget
            self._target_sub = self.create_subscription(
                LandingTarget, "/drone/vision/landing_target", self._target_cb, 10,
            )

            # 发布速度指令 (到 px4_control)
            self._vel_pub = self.create_publisher(
                TwistStamped, "/drone/servo/velocity_cmd", 10,
            )

            # 控制循环 (20Hz)
            self._ctrl_timer = self.create_timer(0.05, self._control_loop)

            self.get_logger().info("视觉伺服节点已启动")

        def _declare_params(self):
            self.declare_parameter("servo_kp", 0.5)
            self.declare_parameter("servo_ki", 0.02)
            self.declare_parameter("servo_kd", 0.1)
            self.declare_parameter("servo_max_horizontal_vel", 1.0)
            self.declare_parameter("servo_descent_speed", 0.3)
            self.declare_parameter("servo_integral_max", 0.5)
            self.declare_parameter("servo_target_deadzone", 0.03)
            self.declare_parameter("fx", 400.0)
            self.declare_parameter("fy", 400.0)
            self.declare_parameter("cx", 320.0)
            self.declare_parameter("cy", 240.0)

        def _build_controller(self) -> VisualServoController:
            return VisualServoController(
                kp=self.get_parameter("servo_kp").value,
                ki=self.get_parameter("servo_ki").value,
                kd=self.get_parameter("servo_kd").value,
                max_horizontal_vel=self.get_parameter("servo_max_horizontal_vel").value,
                descent_speed=self.get_parameter("servo_descent_speed").value,
                integral_max=self.get_parameter("servo_integral_max").value,
                deadzone=self.get_parameter("servo_target_deadzone").value,
            )

        def _target_cb(self, msg: LandingTarget):
            """接收 LandingTarget 并标记活跃."""
            self._latest_target = msg
            self._landing_active = msg.confidence > 0.3

        def _control_loop(self):
            """20Hz 控制循环."""
            if not self._landing_active or not hasattr(self, "_latest_target"):
                # 无目标时发布零速度
                self._publish_velocity(0.0, 0.0, 0.0, 0.0)
                return

            target = self._latest_target
            fx = self.get_parameter("fx").value
            fy = self.get_parameter("fy").value
            cx_img = self.get_parameter("cx").value
            cy_img = self.get_parameter("cy").value

            state = self._controller.update(
                target_u=target.center_u,
                target_v=target.center_v,
                image_center_u=cx_img,
                image_center_v=cy_img,
                altitude=max(self._current_altitude, 0.3),
                focal_length_u=fx,
                focal_length_v=fy,
            )

            self._publish_velocity(
                state.velocity_north,
                state.velocity_east,
                -state.velocity_down,  # NED: down 为负
                state.yaw_rate,
            )

        def _publish_velocity(self, vn: float, ve: float, vd: float, yaw_rate: float):
            vel = TwistStamped()
            vel.header.stamp = self.get_clock().now().to_msg()
            vel.header.frame_id = "base_link"
            vel.twist.linear.x = vn   # NED north
            vel.twist.linear.y = ve   # NED east
            vel.twist.linear.z = vd   # NED down (负=向上)
            vel.twist.angular.z = yaw_rate
            self._vel_pub.publish(vel)

        def update_altitude(self, altitude_m: float):
            self._current_altitude = altitude_m

        def activate(self):
            self._controller.reset()
            self._landing_active = True

        def deactivate(self):
            self._landing_active = False
            self._controller.reset()

    def servo_main(args=None):
        rclpy.init(args=args)
        node = VisualServoNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
