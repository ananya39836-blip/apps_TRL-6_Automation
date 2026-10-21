"""15-State Error-State Extended Kalman Filter (ES-EKF) with Dual-Mode Oracle Support.

State Vector:
    Nominal State (16): [p (3), v (3), q (4), b_a (3), b_g (3)]
    Error State   (15): [delta_p (3), delta_v (3), delta_theta (3), delta_b_a (3), delta_b_g (3)]

Measurement Updates:
    - Wheel Odometry Forward Velocity with Non-Holonomic Constraints (NHC) on v_y and v_z.
    - Joseph-form covariance update to guarantee numerical symmetry and positive-definiteness:
      P+ = (I - K*H) * P- * (I - K*H)^T + K * R * K^T
"""
from __future__ import annotations
import math
from typing import Tuple, Optional
import numpy as np

from trl9_autonomy.sensors.coordinate_transforms import (
    euler_to_quaternion,
    quaternion_to_euler,
    rpy_to_rotation_matrix,
)


class ErrorStateEKF:
    """15-state Error-State EKF with Joseph-form updates and Non-Holonomic Constraints."""

    def __init__(
        self,
        dt_imu: float = 0.010,
        accel_noise_density: float = 0.02,
        gyro_noise_density: float = 0.002,
        accel_bias_instability: float = 0.0005,
        gyro_bias_instability: float = 0.00005,
        oracle_mode: bool = False,
    ) -> None:
        self.dt = dt_imu
        self.oracle_mode = oracle_mode

        # Nominal States
        self.p = np.zeros(3, dtype=np.float64)  # Position [x, y, z] (m)
        self.v = np.zeros(3, dtype=np.float64)  # Velocity [vx, vy, vz] (m/s)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # Quaternion [qw, qx, qy, qz]
        self.b_a = np.zeros(3, dtype=np.float64)  # Accel bias (m/s^2)
        self.b_g = np.zeros(3, dtype=np.float64)  # Gyro bias (rad/s)

        # 15x15 Error State Covariance Matrix P
        self.P = np.eye(15, dtype=np.float64) * 1e-3
        self.P[0:3, 0:3] *= 0.01   # Position variance
        self.P[3:6, 3:6] *= 0.01   # Velocity variance
        self.P[6:9, 6:9] *= 0.001  # Attitude variance
        self.P[9:12, 9:12] *= 0.01 # Accel bias variance
        self.P[12:15, 12:15] *= 0.001 # Gyro bias variance

        # Continuous-time Process Noise Power Spectral Densities
        self.Q_c = np.zeros((15, 15), dtype=np.float64)
        np.fill_diagonal(self.Q_c[3:6, 3:6], accel_noise_density**2)
        np.fill_diagonal(self.Q_c[6:9, 6:9], gyro_noise_density**2)
        np.fill_diagonal(self.Q_c[9:12, 9:12], accel_bias_instability**2)
        np.fill_diagonal(self.Q_c[12:15, 12:15], gyro_bias_instability**2)

        # Mars gravity in World ENU
        self.g_mars_w = np.array([0.0, 0.0, -3.72076], dtype=np.float64)

    def get_rotation_matrix(self) -> np.ndarray:
        """Converts internal quaternion to 3x3 rotation matrix R_WB."""
        roll, pitch, yaw = quaternion_to_euler(self.q)
        return rpy_to_rotation_matrix(roll, pitch, yaw)

    def predict(self, accel_b: np.ndarray, gyro_b: np.ndarray, dt: Optional[float] = None) -> None:
        """Propagates nominal kinematics and error covariance at IMU rate."""
        h = dt if dt is not None else self.dt
        if h <= 0.0:
            return

        # Correct sensor measurements with bias
        accel_corr = accel_b - self.b_a
        gyro_corr = gyro_b - self.b_g

        R_wb = self.get_rotation_matrix()

        # 1. Propagate Nominal Position and Velocity
        accel_w = R_wb @ accel_corr + self.g_mars_w
        self.p += self.v * h + 0.5 * accel_w * (h * h)
        self.v += accel_w * h

        # 2. Propagate Nominal Quaternion (Zero-order quaternion integration)
        omega_norm = float(np.linalg.norm(gyro_corr))
        if omega_norm > 1e-8:
            theta = omega_norm * h
            axis = gyro_corr / omega_norm
            dq = np.array([
                math.cos(theta * 0.5),
                axis[0] * math.sin(theta * 0.5),
                axis[1] * math.sin(theta * 0.5),
                axis[2] * math.sin(theta * 0.5),
            ])
            # Quaternion multiplication: self.q = self.q * dq
            qw, qx, qy, qz = self.q
            dw_val, dx_val, dy_val, dz_val = dq
            new_q = np.array([
                qw * dw_val - qx * dx_val - qy * dy_val - qz * dz_val,
                qw * dx_val + qx * dw_val + qy * dz_val - qz * dy_val,
                qw * dy_val - qx * dz_val + qy * dw_val + qz * dx_val,
                qw * dz_val + qx * dy_val - qy * dx_val + qz * dw_val,
            ])
            self.q = new_q / np.linalg.norm(new_q)

        # 3. Propagate Error State Transition Matrix F
        F = np.eye(15, dtype=np.float64)
        F[0:3, 3:6] = np.eye(3) * h

        # Skew symmetric matrix of acceleration vector: S = [a_w]x
        ax, ay, az = accel_w - self.g_mars_w
        S_accel = np.array([
            [0.0, -az, ay],
            [az, 0.0, -ax],
            [-ay, ax, 0.0]
        ], dtype=np.float64)
        F[3:6, 6:9] = -S_accel * h
        F[3:6, 9:12] = -R_wb * h

        # Skew symmetric of angular rate
        wx, wy, wz = gyro_corr
        S_gyro = np.array([
            [0.0, -wz, wy],
            [wz, 0.0, -wx],
            [-wy, wx, 0.0]
        ], dtype=np.float64)
        F[6:9, 6:9] = np.eye(3) - S_gyro * h
        F[6:9, 12:15] = -np.eye(3) * h

        # Discrete Process Noise Q_d = F * Q_c * F^T * h
        Q_d = self.Q_c * h
        self.P = F @ self.P @ F.T + Q_d

        # Maintain exact symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def update_wheel_odometry(
        self,
        forward_velocity_mps: float,
        dt: Optional[float] = None,
        sigma_vx: float = 0.05,
        sigma_vy: float = 0.02,
        sigma_vz: float = 0.02,
    ) -> None:
        """Updates EKF using forward wheel velocity and Non-Holonomic Constraints (NHC) on v_y, v_z."""
        R_wb = self.get_rotation_matrix()
        R_bw = R_wb.T

        # Velocity in vehicle body frame: v_b = R_bw * v_w
        v_b = R_bw @ self.v

        # Measurement residual y = z_meas - h(x)
        # z_meas = [forward_v, 0.0, 0.0]^T (NHC implies zero lateral/vertical speed)
        z_meas = np.array([forward_velocity_mps, 0.0, 0.0], dtype=np.float64)
        y = z_meas - v_b

        # Measurement Jacobian H (3x15) mapping error state delta_x to delta_y
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = R_bw

        # Measurement noise covariance R (3x3)
        R = np.diag([sigma_vx**2, sigma_vy**2, sigma_vz**2])

        # Innovation Covariance S = H * P * H^T + R
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # State Correction delta_x (15-vector)
        dx_state = K @ y

        # Apply correction to nominal states
        self.p += dx_state[0:3]
        self.v += dx_state[3:6]

        # Correct quaternion with attitude error delta_theta
        d_theta = dx_state[6:9]
        d_norm = float(np.linalg.norm(d_theta))
        if d_norm > 1e-10:
            dq_c = np.array([1.0, 0.5 * d_theta[0], 0.5 * d_theta[1], 0.5 * d_theta[2]])
            dq_c /= np.linalg.norm(dq_c)
            qw, qx, qy, qz = self.q
            dq_w, dq_x, dq_y, dq_z = dq_c
            new_q = np.array([
                qw * dq_w - qx * dq_x - qy * dq_y - qz * dq_z,
                qw * dq_x + qx * dq_w + qy * dq_z - qz * dq_y,
                qw * dq_y - qx * dq_z + qy * dq_w + qz * dq_x,
                qw * dq_z + qx * dq_y - qy * dq_x + qz * dq_w,
            ])
            self.q = new_q / np.linalg.norm(new_q)

        # Correct sensor biases
        self.b_a += dx_state[9:12]
        self.b_g += dx_state[12:15]

        # Joseph-form Covariance Update: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        I_KH = np.eye(15, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_position(
        self,
        meas_pos: np.ndarray,
        r_cov: float = 0.05 ** 2,
    ) -> None:
        """Corrects state using visual/LiDAR ICP absolute position measurement."""
        res = meas_pos - self.p
        H = np.zeros((3, 15), dtype=np.float64)
        H[0:3, 0:3] = np.eye(3)

        R = np.eye(3, dtype=np.float64) * r_cov
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        dx_state = K @ res
        self.p += dx_state[0:3]
        self.v += dx_state[3:6]

        d_theta = dx_state[6:9]
        d_norm = float(np.linalg.norm(d_theta))
        if d_norm > 1e-10:
            dq_c = np.array([1.0, 0.5 * d_theta[0], 0.5 * d_theta[1], 0.5 * d_theta[2]])
            dq_c /= np.linalg.norm(dq_c)
            qw, qx, qy, qz = self.q
            dq_w, dq_x, dq_y, dq_z = dq_c
            new_q = np.array([
                qw * dq_w - qx * dq_x - qy * dq_y - qz * dq_z,
                qw * dq_x + qx * dq_w + qy * dq_z - qz * dq_y,
                qw * dq_y - qx * dq_z + qy * dq_w + qz * dq_x,
                qw * dq_z + qx * dq_y - qy * dq_x + qz * dq_w,
            ])
            self.q = new_q / np.linalg.norm(new_q)

        self.b_a += dx_state[9:12]
        self.b_g += dx_state[12:15]

        I_KH = np.eye(15, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns (position [x, y, z], velocity [vx, vy, vz], Euler RPY [roll, pitch, yaw])."""
        roll, pitch, yaw = quaternion_to_euler(self.q)
        return self.p.copy(), self.v.copy(), np.array([roll, pitch, yaw], dtype=np.float64)
