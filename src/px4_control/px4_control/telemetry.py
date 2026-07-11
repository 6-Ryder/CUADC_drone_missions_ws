"""
遥测数据模型 — Position / Status / Attitude.

解析 PX4 VehicleOdometry, VehicleStatus, VehicleAttitude,
并转换为简洁的 Python dataclass 供上层使用.

坐标系约定:
  - NED: PX4 原生坐标系 (x=前/North, y=右/East, z=下)
  - 本模块提供 NED↔World 双向转换
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple


# ============================================================
# 数据模型
# ============================================================

@dataclass
class Position:
    """无人机位置 (NED 坐标系)."""
    north: float = 0.0   # NED x (前)
    east: float = 0.0    # NED y (右)
    down: float = 0.0    # NED z (下, 负值=向上)

    def norm_xy(self) -> float:
        return math.hypot(self.north, self.east)

    def distance_to(self, other: "Position") -> float:
        return math.sqrt(
            (self.north - other.north) ** 2 +
            (self.east - other.east) ** 2 +
            (self.down - other.down) ** 2
        )

    def distance_xy(self, other: "Position") -> float:
        return math.hypot(self.north - other.north, self.east - other.east)

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.north, self.east, self.down)


@dataclass
class Attitude:
    """无人机姿态."""
    q_w: float = 1.0
    q_x: float = 0.0
    q_y: float = 0.0
    q_z: float = 0.0
    roll: float = 0.0   # rad
    pitch: float = 0.0  # rad
    yaw: float = 0.0    # rad

    def to_quat_xyzw(self) -> Tuple[float, float, float, float]:
        """返回 (x, y, z, w) 四元数."""
        return (self.q_x, self.q_y, self.q_z, self.q_w)


@dataclass
class Velocity:
    """无人机速度 (NED 坐标系, m/s)."""
    vn: float = 0.0  # north
    ve: float = 0.0  # east
    vd: float = 0.0  # down


@dataclass
class DroneStatus:
    """无人机综合状态."""
    armed: bool = False
    in_air: bool = False
    landed: bool = True
    offboard_mode: bool = False
    navigation_state: int = 0
    battery_remaining: float = 1.0
    gps_fix_type: int = 0
    gps_satellites: int = 0


@dataclass
class Telemetry:
    """一次遥测快照."""
    timestamp: float = 0.0
    position: Position = field(default_factory=Position)
    attitude: Attitude = field(default_factory=Attitude)
    velocity: Velocity = field(default_factory=Velocity)
    status: DroneStatus = field(default_factory=DroneStatus)


# ============================================================
# 坐标变换
# ============================================================

# 场地 World 原点 → 起飞点 H 的偏移
# H 在 Gazebo World 中的位置: (x=10, y=4, z=0)
H_WORLD_X = 10.0   # east
H_WORLD_Y = 4.0    # north
H_WORLD_Z = 0.0    # up


def world_to_ned(world_x: float, world_y: float, world_z: float) -> Tuple[float, float, float]:
    """
    Gazebo World → PX4 NED (相对起飞点 H).

    World: x=east, y=north, z=up
    NED:   x=north=World_y - 4, y=east=World_x - 10, z=down=-World_z
    """
    north = world_y - H_WORLD_Y
    east = world_x - H_WORLD_X
    down = -world_z
    return (north, east, down)


def ned_to_world(north: float, east: float, down: float) -> Tuple[float, float, float]:
    """
    PX4 NED (相对起飞点 H) → Gazebo World.

    World_x = east + H_WORLD_X
    World_y = north + H_WORLD_Y
    World_z = -down
    """
    world_x = east + H_WORLD_X
    world_y = north + H_WORLD_Y
    world_z = -down
    return (world_x, world_y, world_z)


def ned_to_world_pose(north: float, east: float, down: float,
                      q_x: float, q_y: float, q_z: float, q_w: float
                      ) -> Tuple[float, float, float, float, float, float, float]:
    """
    将 NED 位姿转换为 World 位姿.

    Returns (world_x, world_y, world_z, qx, qy, qz, qw).

    输入四元数为 PX4 body→NED 姿态, 但顺序是 ROS 常用 xyzw。
    输出四元数为 body→World(ENU/Gazebo) 姿态, 顺序仍为 xyzw。
    """
    wx, wy, wz = ned_to_world(north, east, down)
    qx, qy, qz, qw = ned_body_to_world_quat(q_x, q_y, q_z, q_w)
    return (wx, wy, wz, qx, qy, qz, qw)


def _normalize_quat_xyzw(
    q_x: float,
    q_y: float,
    q_z: float,
    q_w: float,
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(q_x * q_x + q_y * q_y + q_z * q_z + q_w * q_w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q_x / norm, q_y / norm, q_z / norm, q_w / norm)


def _quat_multiply_xyzw(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Hamilton product for xyzw quaternions: result applies b, then a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def ned_body_to_world_quat(
    q_x: float,
    q_y: float,
    q_z: float,
    q_w: float,
) -> Tuple[float, float, float, float]:
    """
    Convert a PX4 body→NED quaternion to body→World(ENU/Gazebo).

    NED axes:   x=north, y=east, z=down.
    World axes: x=east,  y=north, z=up.

    The fixed NED→World rotation is a 180° rotation around axis (1, 1, 0).
    """
    q_body_to_ned = _normalize_quat_xyzw(q_x, q_y, q_z, q_w)
    half_sqrt = math.sqrt(0.5)
    q_ned_to_world = (half_sqrt, half_sqrt, 0.0, 0.0)
    return _normalize_quat_xyzw(*_quat_multiply_xyzw(q_ned_to_world, q_body_to_ned))


def yaw_from_quat(q_w: float, q_x: float, q_y: float, q_z: float) -> float:
    """从四元数提取偏航角 (rad, NED 帧)."""
    siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    return math.atan2(siny_cosp, cosy_cosp)
