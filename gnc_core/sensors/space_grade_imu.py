"""Space-Grade Inertial Measurement Unit (IMU) Error Engine (TRL 4).
Representative of Honeywell MIMU / Northrop Grumman LN-200 Space-Grade Standards.

Injects Allan variance noise parameters:
- Gyro Angular Random Walk (ARW) & Accel Velocity Random Walk (VRW)
- 1st-Order Gauss-Markov in-run bias instability
- Scale factor error & cross-axis non-orthogonality
- Physical sensor clipping and saturation limits

Under PROTOTYPE_LOCK = True, navigation_ready evaluates strictly to False.
"""
from __future__ import annotations
import math
from typing import Dict, Any, Tuple, Optional
import numpy as np

from gnc_core import PROTOTYPE_LOCK, NAVIGATION_READY


class SpaceGradeIMU:
    """Navigation-grade 6-DOF IMU sensor emulator with stochastic error propagation."""

    def __init__(
        self,
        dt_s: float = 0.01,  # 100 Hz update
        gyro_arw_dps_rt_hz: float = 0.002,  # Angular Random Walk (deg/s/sqrt(Hz))
        gyro_bias_instability_dph: float = 0.01,  # Bias instability (deg/hr)
        gyro_corr_time_s: float = 3600.0,  # Gauss-Markov correlation time (s)
        accel_vrw_ug_rt_hz: float = 15.0,  # Velocity Random Walk (micro-g/sqrt(Hz))
        accel_bias_instability_ug: float = 25.0,  # Accel bias instability (micro-g)
        accel_corr_time_s: float = 1800.0,
        gyro_saturation_dps: float = 400.0,
        accel_saturation_g: float = 35.0,
        seed: int = 42
    ) -> None:
        self.dt = float(dt_s)
        self.rng = np.random.default_rng(seed)

        # Convert Gyro parameters to SI (rad/s)
        self.gyro_arw = math.radians(gyro_arw_dps_rt_hz)
        self.gyro_b_std = math.radians(gyro_bias_instability_dph / 3600.0)
        self.gyro_tau = float(gyro_corr_time_s)
        self.gyro_max = math.radians(gyro_saturation_dps)

        # Convert Accel parameters to SI (m/s^2)
        g0 = 9.80665
        self.accel_vrw = (accel_vrw_ug_rt_hz * 1e-6) * g0
        self.accel_b_std = (accel_bias_instability_ug * 1e-6) * g0
        self.accel_tau = float(accel_corr_time_s)
        self.accel_max = accel_saturation_g * g0

        # State: 1st-order Gauss-Markov bias vectors
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.accel_bias = np.zeros(3, dtype=np.float64)

        # Misalignment matrix (cross-axis coupling ~ 50 ppm)
        self.gyro_misalignment = np.eye(3) + self.rng.normal(0, 5e-5, (3, 3))
        self.accel_misalignment = np.eye(3) + self.rng.normal(0, 5e-5, (3, 3))

    def step(
        self,
        true_omega_rps: np.ndarray,
        true_accel_mps2: np.ndarray
    ) -> Dict[str, Any]:
        """Injects stochastic errors onto ground truth kinematics.
        
        Args:
            true_omega_rps: True angular velocity [p, q, r] in rad/s
            true_accel_mps2: True specific force [ax, ay, az] in m/s^2
            
        Returns:
            Dictionary with measured rates, specific forces, biases, and saturation flags.
        """
        w_true = np.asarray(true_omega_rps, dtype=np.float64)
        a_true = np.asarray(true_accel_mps2, dtype=np.float64)

        # 1. Update Gauss-Markov biases: b_{k+1} = (1 - dt/tau)*b_k + w
        # Gyro bias update
        gyro_beta = math.exp(-self.dt / self.gyro_tau)
        gyro_driving_noise = self.gyro_b_std * math.sqrt(1.0 - gyro_beta**2) * self.rng.normal(0, 1, 3)
        self.gyro_bias = gyro_beta * self.gyro_bias + gyro_driving_noise

        # Accel bias update
        accel_beta = math.exp(-self.dt / self.accel_tau)
        accel_driving_noise = self.accel_b_std * math.sqrt(1.0 - accel_beta**2) * self.rng.normal(0, 1, 3)
        self.accel_bias = accel_beta * self.accel_bias + accel_driving_noise

        # 2. White noise components (Random Walk)
        gyro_white = (self.gyro_arw / math.sqrt(self.dt)) * self.rng.normal(0, 1, 3)
        accel_white = (self.accel_vrw / math.sqrt(self.dt)) * self.rng.normal(0, 1, 3)

        # 3. Apply misalignment, biases, and white noise
        omega_meas = self.gyro_misalignment @ w_true + self.gyro_bias + gyro_white
        accel_meas = self.accel_misalignment @ a_true + self.accel_bias + accel_white

        # 4. Sensor saturation clipping
        gyro_clipped = np.clip(omega_meas, -self.gyro_max, self.gyro_max)
        accel_clipped = np.clip(accel_meas, -self.accel_max, self.accel_max)

        is_gyro_saturated = bool(np.any(np.abs(omega_meas) >= self.gyro_max))
        is_accel_saturated = bool(np.any(np.abs(accel_meas) >= self.accel_max))

        return {
            "meas_omega_rps": gyro_clipped,
            "meas_accel_mps2": accel_clipped,
            "gyro_bias_rps": self.gyro_bias.copy(),
            "accel_bias_mps2": self.accel_bias.copy(),
            "bias_gyro_rps": self.gyro_bias.copy(),
            "bias_accel_mps2": self.accel_bias.copy(),
            "is_gyro_saturated": is_gyro_saturated,
            "is_accel_saturated": is_accel_saturated,
            "clipped": is_gyro_saturated or is_accel_saturated,
        }

    # Alias for sample method
    sample = step
