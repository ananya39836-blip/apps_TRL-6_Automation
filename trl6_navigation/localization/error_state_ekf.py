"""Error-State Extended Kalman Filter (ES-EKF) for Planetary Rover Navigation (TRL 6).

Implements 15-state Error-State EKF fusing:
- 100 Hz Tactical IMU kinematic propagation (prediction)
- 10 Hz Wheel Odometry linear velocity and angular rate (measurement update)
- Joseph-form covariance updates guaranteeing positive semi-definiteness
"""
from __future__ import annotations
import math
from typing import Tuple, Dict, Any
import numpy as np


class ErrorStateEKF:
    """Error-State Extended Kalman Filter fusing IMU kinematics and wheel odometry."""

    def __init__(
        self,
        init_x: float = 0.0,
        init_y: float = 0.0,
        init_yaw_rad: float = 0.0,
        init_speed_mps: float = 0.0
    ) -> None:
        # Nominal state: [x, y, z, vx, vy, vz, roll, pitch, yaw, b_ax, b_ay, b_az, b_wx, b_wy, b_wz]
        self.state = np.zeros(15, dtype=np.float64)
        self.state[0] = float(init_x)
        self.state[1] = float(init_y)
        self.state[3] = float(init_speed_mps * math.cos(init_yaw_rad))
        self.state[4] = float(init_speed_mps * math.sin(init_yaw_rad))
        self.state[8] = float(init_yaw_rad)

        # Error state covariance matrix P (15x15)
        self.P = np.eye(15, dtype=np.float64) * 0.1
        self.P[0:3, 0:3] *= 0.05   # Position variance
        self.P[3:6, 3:6] *= 0.02   # Velocity variance
        self.P[6:9, 6:9] *= 0.01   # Orientation variance
        self.P[9:12, 9:12] *= 0.001 # Accel bias variance
        self.P[12:15, 12:15] *= 0.0005 # Gyro bias variance

        # Continuous process noise spectral density Q_c
        self.q_pos = 0.001
        self.q_vel = 0.02
        self.q_rot = 0.005
        self.q_abias = 0.0001
        self.q_gbias = 0.00005

        # Measurement noise covariance R (wheel odometry: [v_lin, w_yaw])
        self.R_odom = np.diag([0.002, 0.002])

    def predict(self, gyro_z_rps: float, accel_x_mps2: float, dt_s: float = 0.01) -> None:
        """High-rate prediction step propagating nominal state and error covariance."""
        dt = float(dt_s)
        yaw = self.state[8]
        vx = self.state[3]
        vy = self.state[4]

        # Bias compensation
        w_z = float(gyro_z_rps) - self.state[14]
        a_x = float(accel_x_mps2) - self.state[9]

        # 1. Propagate Nominal State
        new_yaw = (yaw + w_z * dt + math.pi) % (2.0 * math.pi) - math.pi
        mid_yaw = (yaw + 0.5 * w_z * dt)

        # Acceleration rotated into world horizontal frame
        a_world_x = a_x * math.cos(mid_yaw)
        a_world_y = a_x * math.sin(mid_yaw)

        self.state[0] += vx * dt + 0.5 * a_world_x * (dt ** 2)
        self.state[1] += vy * dt + 0.5 * a_world_y * (dt ** 2)
        self.state[3] += a_world_x * dt
        self.state[4] += a_world_y * dt
        self.state[8] = new_yaw

        # 2. Propagate Error State Covariance (F matrix linearization)
        F = np.eye(15, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[3, 8] = -a_x * math.sin(mid_yaw) * dt
        F[4, 8] = a_x * math.cos(mid_yaw) * dt
        F[3, 9] = -math.cos(mid_yaw) * dt
        F[4, 9] = -math.sin(mid_yaw) * dt
        F[8, 14] = -dt

        # Discrete process noise Q_d
        Q = np.zeros((15, 15), dtype=np.float64)
        Q[0:3, 0:3] = np.eye(3) * self.q_pos * dt
        Q[3:6, 3:6] = np.eye(3) * self.q_vel * dt
        Q[6:9, 6:9] = np.eye(3) * self.q_rot * dt
        Q[9:12, 9:12] = np.eye(3) * self.q_abias * dt
        Q[12:15, 12:15] = np.eye(3) * self.q_gbias * dt

        self.P = F @ self.P @ F.T + Q

    def update_wheel_odometry(self, meas_v_mps: float, meas_w_rps: float) -> None:
        """Corrects state using 2D wheel odometry linear velocity and heading rate."""
        yaw = self.state[8]
        vx = self.state[3]
        vy = self.state[4]

        # Predicted measurement: [linear_speed_forward, yaw_rate]
        pred_speed = vx * math.cos(yaw) + vy * math.sin(yaw)
        pred_w = 0.0  # Steady turn rate is observed directly via gyro

        z = np.array([float(meas_v_mps), float(meas_w_rps)])
        h = np.array([pred_speed, pred_w])
        y = z - h

        # Measurement Jacobian H (2x15)
        H = np.zeros((2, 15), dtype=np.float64)
        H[0, 3] = math.cos(yaw)
        H[0, 4] = math.sin(yaw)
        H[0, 8] = -vx * math.sin(yaw) + vy * math.cos(yaw)
        H[1, 14] = -1.0  # Gyro bias sensitivity

        # Innovation covariance S = H P H^T + R
        S = H @ self.P @ H.T + self.R_odom

        # Kalman Gain K = P H^T S^-1
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update nominal state
        delta_x = K @ y
        self.state += delta_x
        self.state[8] = (self.state[8] + math.pi) % (2.0 * math.pi) - math.pi

        # Joseph-form covariance update: P = (I - KH) P (I - KH)^T + K R K^T
        I_KH = np.eye(15, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_odom @ K.T

    def get_estimated_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns estimated 2D pose [x, y, yaw] and 3x3 pose covariance matrix."""
        pose_xytheta = np.array([self.state[0], self.state[1], self.state[8]])
        cov_3x3 = np.array([
            [self.P[0, 0], self.P[0, 1], self.P[0, 8]],
            [self.P[1, 0], self.P[1, 1], self.P[1, 8]],
            [self.P[8, 0], self.P[8, 1], self.P[8, 8]],
        ], dtype=np.float64)
        return pose_xytheta, cov_3x3

    def get_forward_speed(self) -> float:
        """Returns forward body velocity in m/s."""
        yaw = self.state[8]
        return float(self.state[3] * math.cos(yaw) + self.state[4] * math.sin(yaw))
