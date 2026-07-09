"""
坐标变换: 像素 ↔ 世界 (姿态补偿).

核心公式:
  给定无人机位姿 (位置 + 姿态四元数) 和相机内参,
  将图像中的像素坐标转换为精确的世界坐标.

与简化版 (纯高度/焦距) 的区别:
  - 简化版: 假设相机完美垂直向下, 仅用高度/焦距缩放
  - 本模块: 利用完整 6-DoF 位姿, 通过相机投影模型反算,
           补偿 roll/pitch 带来的视轴偏移

变换链:
  Pixel(u,v) → Camera 3D ray → Intersect with ground plane (z=0)
  → World coordinates (x, y, 0)

用法:
  transformer = CoordinateTransformer(K)
  wx, wy = transformer.pixel_to_world(u, v, altitude, q_wxyz)
"""

import numpy as np
from typing import Tuple, Optional


# ---- Pure numpy quaternion utilities (no scipy dependency) ----

def _quat_to_matrix(q_xyzw: np.ndarray) -> np.ndarray:
    """
    Convert quaternion (w, x, y, z) to 3×3 rotation matrix.
    (调用方 pixel_to_world 已转换为 wxyz 格式)

    This replaces scipy.spatial.transform.Rotation.from_quat().
    """
    w, x, y, z = q_xyzw[0], q_xyzw[1], q_xyzw[2], q_xyzw[3]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ])


def _quat_to_matrix_inv(q_xyzw: np.ndarray) -> np.ndarray:
    """Inverse of quaternion rotation (transpose for unit quat)."""
    return _quat_to_matrix(q_xyzw).T


class CoordinateTransformer:
    """
    像素 → 世界坐标变换器 (带姿态补偿).

    Parameters
    ----------
    K : np.ndarray
        相机内参矩阵 (3×3): [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    """

    def __init__(self, K: np.ndarray):
        self.K = K
        self.K_inv = np.linalg.inv(K)

    def update_intrinsics(self, fx: float, fy: float, cx: float, cy: float):
        """更新内参."""
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.K_inv = np.linalg.inv(self.K)

    # ----------------------------------------------------------------
    # 像素 → 世界 (主接口)
    # ----------------------------------------------------------------

    def pixel_to_world(
        self,
        u: float,
        v: float,
        drone_position: Tuple[float, float, float],  # (x, y, z) World frame (z=上)
        drone_attitude_q: Tuple[float, float, float, float],  # (x, y, z, w) or (w, x, y, z)
        ground_z: float = 0.0,
        q_convention: str = "xyzw",
    ) -> Optional[Tuple[float, float]]:
        """
        像素坐标 → 地面投影世界坐标.

        算法:
          1. 像素反投影 → 相机坐标系中的射线方向
          2. 射线方向 × 相机→世界旋转 → 世界坐标系中的射线
          3. 射线与地面平面 (z=ground_z) 求交

        Parameters
        ----------
        u, v : 像素坐标.
        drone_position : (x, y, z) 无人机世界坐标 (z 向上为正).
        drone_attitude_q : 无人机姿态四元数.
        ground_z : 地面高度 (默认 0).
        q_convention : "xyzw" (PX4/ROS) 或 "wxyz" (scipy).

        Returns
        -------
        (world_x, world_y) 或 None (射线向上不与地面相交).
        """
        # 1. 像素 → 归一化相机坐标 (z_cam = 1)
        pixel = np.array([u, v, 1.0])
        ray_cam = self.K_inv @ pixel  # 相机坐标系中的射线方向

        # 2. 相机 → 机体
        # 相机安装: 垂直向下, 光轴 = -body_z
        # R_cam_to_body: 相机 x→body right, 相机 y→body forward, 相机 z→body -down
        R_cam_to_body = np.array([
            [0,  1,  0],   # 相机 y → body front
            [1,  0,  0],   # 相机 x → body right
            [0,  0, -1],   # 相机 z (光轴) → body -down
        ])
        ray_body = R_cam_to_body @ ray_cam

        # 3. 机体 → World
        if q_convention == "xyzw":
            q = np.array([drone_attitude_q[3], drone_attitude_q[0],
                          drone_attitude_q[1], drone_attitude_q[2]])
        else:
            q = np.array(drone_attitude_q)
        R_body_to_world = _quat_to_matrix(q)
        ray_world = R_body_to_world @ ray_body

        # 4. 射线与地面求交
        # 无人机位置: P_drone
        # 射线方向: ray_world (从相机出发, 在世界系中)
        # 地面平面: z = ground_z
        # 参数方程: P = P_drone + t * ray_world
        # 令 P_z = ground_z → t = (ground_z - P_drone_z) / ray_world_z
        P = np.array(drone_position)
        if abs(ray_world[2]) < 1e-10:
            return None  # 射线平行于地面

        t = (ground_z - P[2]) / ray_world[2]

        if t <= 0:
            return None  # 射线背离地面 (视轴朝上)

        P_ground = P + t * ray_world
        return (float(P_ground[0]), float(P_ground[1]))

    # ----------------------------------------------------------------
    # 世界 → 像素 (用于 Ground Truth 生成)
    # ----------------------------------------------------------------

    def world_to_pixel(
        self,
        world_x: float,
        world_y: float,
        world_z: float,
        drone_position: Tuple[float, float, float],
        drone_attitude_q: Tuple[float, float, float, float],
        img_w: int = 640,
        img_h: int = 480,
        q_convention: str = "xyzw",
    ) -> Optional[Tuple[float, float]]:
        """
        世界坐标 → 像素投影. 用于训练数据 Ground Truth 生成.

        Returns (u, v) 或 None (在相机后方/图像外).
        """
        P_world = np.array([world_x, world_y, world_z])
        P_drone = np.array(drone_position)

        # World → Body
        if q_convention == "xyzw":
            q = np.array([drone_attitude_q[3], drone_attitude_q[0],
                          drone_attitude_q[1], drone_attitude_q[2]])
        else:
            q = np.array(drone_attitude_q)
        R_world_to_body = _quat_to_matrix_inv(q)

        # Body → Camera
        R_body_to_cam = np.array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1],
        ])  # R_cam_to_body 的逆 = 转置

        R_world_to_cam = R_body_to_cam @ R_world_to_body

        P_cam = R_world_to_cam @ (P_world - P_drone)

        if P_cam[2] <= 0:
            return None  # 在相机后方

        # Camera → Pixel
        pixel = self.K @ P_cam
        u = pixel[0] / pixel[2]
        v = pixel[1] / pixel[2]

        if u < 0 or u >= img_w or v < 0 or v >= img_h:
            return None

        return (float(u), float(v))

    # ----------------------------------------------------------------
    # 简化版 (无姿态信息时的回退)
    # ----------------------------------------------------------------

    @staticmethod
    def pixel_to_world_simple(
        u: float, v: float,
        altitude: float,
        fx: float, fy: float,
        cx: float, cy: float,
        drone_world_x: float = 0.0,
        drone_world_y: float = 0.0,
    ) -> Tuple[float, float]:
        """
        简化版 (假设相机垂直向下, 无姿态).

        用于姿态数据不可用时的回退.
        """
        dx = (v - cy) * altitude / fy  # 像素行 → world north (前)
        dy = (u - cx) * altitude / fx  # 像素列 → world east  (右)

        # 无人机在前方看, 相机 x→右, y→前
        world_x = drone_world_x + dy
        world_y = drone_world_y + dx
        return (world_x, world_y)

    # ----------------------------------------------------------------
    # 批量操作
    # ----------------------------------------------------------------

    def pixels_to_world_batch(
        self,
        pixels: np.ndarray,  # (N, 2) — (u, v)
        drone_position: Tuple[float, float, float],
        drone_attitude_q: Tuple[float, float, float, float],
        ground_z: float = 0.0,
        q_convention: str = "xyzw",
    ) -> np.ndarray:
        """批量像素→世界."""
        results = np.zeros((len(pixels), 2))
        for i, (u, v) in enumerate(pixels):
            r = self.pixel_to_world(
                u, v, drone_position, drone_attitude_q, ground_z, q_convention,
            )
            if r:
                results[i] = r
            else:
                results[i] = [np.nan, np.nan]
        return results


# ============================================================
# 高度自适应检测阈值
# ============================================================

class AltitudeAdaptiveParams:
    """
    根据飞行高度动态调整检测参数.

    原理:
      - 目标物理尺寸固定 (圆筒Ø=15/20/25cm, H标识Ø=80cm)
      - 像素尺寸 = 物理尺寸 × 焦距 / 高度
      - 高度↑ → 像素尺寸↓ → 降低 conf 阈值、调整 min/max 像素半径

    使用:
      adaptive = AltitudeAdaptiveParams()
      params = adaptive.get_cylinder_params(altitude_m=5.0)
      # params = {"min_radius_px": 6, "max_radius_px": 20, "conf_scale": 0.8, ...}
    """

    # 物理尺寸 (m)
    PHYSICAL = {
        "cylinder_small": 0.075,   # 半径
        "cylinder_medium": 0.100,
        "cylinder_large": 0.125,
        "landing_pad": 0.40,
        "hazard_sign": 0.06,       # 12cm×12cm 的半边长
    }

    def __init__(
        self,
        focal_length_px: float = 400.0,
        base_altitude: float = 5.0,  # 基准高度 (m)
        base_conf_threshold: float = 0.5,
    ):
        self.f = focal_length_px
        self.h0 = base_altitude
        self.conf0 = base_conf_threshold

    def _pixel_size(self, physical_m: float, altitude_m: float) -> float:
        """物理尺寸 → 像素尺寸."""
        if altitude_m <= 0.1:
            altitude_m = 0.1
        return physical_m * self.f / altitude_m

    def get_cylinder_params(self, altitude_m: float) -> dict:
        """
        返回高度自适应的圆筒检测参数.

        用于 HoughCircles 回退和 OpenCV 精修的范围约束.
        """
        # 在视野中可见的最小/最大圆筒像素半径
        r_min_m = self.PHYSICAL["cylinder_small"] * 0.6  # 允许部分遮挡
        r_max_m = self.PHYSICAL["cylinder_large"] * 1.5

        min_r_px = max(3, int(self._pixel_size(r_min_m, altitude_m)))
        max_r_px = min(200, int(self._pixel_size(r_max_m, altitude_m) * 1.5))

        # 高度↑ → 目标像素占比↓ → 降低置信度阈值
        scale_factor = self.h0 / max(altitude_m, 0.5)
        conf_scale = np.clip(scale_factor, 0.5, 1.5)

        return {
            "min_radius_px": min_r_px,
            "max_radius_px": max_r_px,
            "conf_scale": float(conf_scale),
            "effective_conf_threshold": float(self.conf0 / conf_scale),
        }

    def get_landing_params(self, altitude_m: float) -> dict:
        """高度自适应的降落标识检测参数."""
        r_phys = self.PHYSICAL["landing_pad"]
        r_px = self._pixel_size(r_phys, altitude_m)

        return {
            "expected_radius_px": r_px,
            "min_radius_px": max(5, int(r_px * 0.3)),
            "max_radius_px": min(300, int(r_px * 1.5)),
            "min_area_px2": max(50, int(np.pi * (r_px * 0.3) ** 2)),
        }

    def is_object_detectable(
        self, physical_size_m: float, altitude_m: float,
        min_pixels: int = 5,
    ) -> bool:
        """判断一个物理尺寸的目标在当前高度下是否可被 YOLO 检测."""
        px = self._pixel_size(physical_size_m, altitude_m)
        return px >= min_pixels
