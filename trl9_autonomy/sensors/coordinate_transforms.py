"""Rigid 3D SE(3) Coordinate Transformations & Frame Definitions.

Conventions:
- World Frame (W): Topocentric East-North-Up (ENU).
- Body Frame (B): X-Forward, Y-Port (Left), Z-Zenith (Up).
- Sensor Frames:
    - LiDAR (L): Mast mounted (+1.0m, 0.0m, +1.8m)
    - Camera (C): Mast mounted (+1.1m, 0.0m, +1.7m)
    - IMU (I): Vehicle Center of Mass (0.0m, 0.0m, 0.0m)
"""
from __future__ import annotations
import math
from typing import Tuple, Union
import numpy as np


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Converts Euler roll, pitch, yaw (rad, ZYX sequence) to unit Hamilton quaternion [qw, qx, qy, qz]."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm > 1e-12:
        return np.array([qw / norm, qx / norm, qy / norm, qz / norm], dtype=np.float64)
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def quaternion_to_euler(q: Union[np.ndarray, list]) -> Tuple[float, float, float]:
    """Converts unit quaternion [qw, qx, qy, qz] to Euler roll, pitch, yaw (rad)."""
    qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Computes 3x3 orthonormal SO(3) rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]
    ], dtype=np.float64)
    return R


class TransformSE3:
    """Homogeneous 4x4 SE(3) transformation representing rigid pose and coordinate frame change."""

    def __init__(self, matrix: np.ndarray | None = None) -> None:
        if matrix is None:
            self.matrix = np.eye(4, dtype=np.float64)
        else:
            self.matrix = np.array(matrix, dtype=np.float64)

    @classmethod
    def from_xyz_rpy(cls, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> TransformSE3:
        """Constructs SE(3) from Cartesian position [x, y, z] and Euler angles (roll, pitch, yaw)."""
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rpy_to_rotation_matrix(roll, pitch, yaw)
        T[:3, 3] = [x, y, z]
        return cls(T)

    @property
    def translation(self) -> np.ndarray:
        return self.matrix[:3, 3].copy()

    @property
    def rotation(self) -> np.ndarray:
        return self.matrix[:3, :3].copy()

    def inverse(self) -> TransformSE3:
        """Computes analytical inverse T^-1 = [R^T, -R^T * t; 0, 1]."""
        R = self.matrix[:3, :3]
        t = self.matrix[:3, 3]
        T_inv = np.eye(4, dtype=np.float64)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        return TransformSE3(T_inv)

    def compose(self, other: TransformSE3) -> TransformSE3:
        """Composes this transformation with another: T_result = self * other."""
        return TransformSE3(self.matrix @ other.matrix)

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """Transforms N x 3 Cartesian point array using the 4x4 matrix."""
        if len(points) == 0:
            return np.zeros((0, 3), dtype=np.float64)
        pts_hom = np.hstack([points, np.ones((len(points), 1), dtype=np.float64)])
        pts_trans = (self.matrix @ pts_hom.T).T
        return pts_trans[:, :3]


class SensorExtrinsics:
    """Rigid mounting extrinsics for onboard sensors relative to Body Frame B."""

    def __init__(self) -> None:
        # LiDAR: Mast mounted at x=+1.0m, y=0.0m, z=+1.8m
        self.T_body_lidar = TransformSE3.from_xyz_rpy(1.0, 0.0, 1.8, 0.0, 0.0, 0.0)

        # Camera: Mast mounted at x=+1.1m, y=0.0m, z=+1.7m, 10 deg pitch down
        self.T_body_camera = TransformSE3.from_xyz_rpy(1.1, 0.0, 1.7, 0.0, math.radians(10.0), 0.0)

        # IMU: Vehicle Center of Mass origin
        self.T_body_imu = TransformSE3.from_xyz_rpy(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
