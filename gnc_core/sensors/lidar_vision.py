"""Spacecraft Flash LiDAR & Vision Sensor Error Engine with Latency Ring-Buffering (TRL 4).
Representative of NASA OSIRIS-REx / Mars 2020 Lander Vision Navigation Standards.

Implements range-squared Gaussian noise, dust/glare dropouts,
and hardware transport latency FIFO ring buffering.

Under PROTOTYPE_LOCK = True, navigation_ready evaluates strictly to False.
"""
from __future__ import annotations
import math
from typing import Dict, Any, List, Optional
from collections import deque
import numpy as np

from gnc_core import PROTOTYPE_LOCK, NAVIGATION_READY


class SpacecraftLiDARVisionSensor:
    """Flash LiDAR and Optical Hazard Camera with stochastic noise and latency delay."""

    def __init__(
        self,
        base_noise_m: float = 0.05,  # 5 cm at 100 m
        ref_distance_m: float = 100.0,
        dropout_probability: float = 0.04,  # 4% dropout rate
        latency_steps: int = 5,  # 5 steps = 50 ms delay at 100 Hz
        seed: int = 777
    ) -> None:
        self.sigma_0 = float(base_noise_m)
        self.r_0 = float(ref_distance_m)
        self.p_dropout = float(dropout_probability)
        self.latency_steps = max(0, int(latency_steps))
        self.rng = np.random.default_rng(seed)

        # FIFO ring buffer for transport delay emulation
        self.buffer: deque[Optional[Dict[str, Any]]] = deque(maxlen=max(1, self.latency_steps + 1))
        # Pre-fill buffer with initial empty states
        for _ in range(self.latency_steps):
            self.buffer.append(None)

    def process_measurement(
        self,
        true_range_m: float,
        bearing_angles_rad: Tuple[float, float] = (0.0, 0.0)
    ) -> Optional[Dict[str, Any]]:
        """Processes current ground truth reading, queues into latency buffer, and returns delayed packet.
        
        Range Noise: sigma_R = sigma_0 * (R / R_0)^2
        """
        r = max(0.1, float(true_range_m))

        # Check for dropout
        if self.rng.uniform(0.0, 1.0) < self.p_dropout:
            current_reading = {
                "range_m": None,
                "bearing_az_rad": bearing_angles_rad[0],
                "bearing_el_rad": bearing_angles_rad[1],
                "status": "DROPOUT_DETECTED",
                "valid": False,
            }
        else:
            # Range-squared noise scaling
            sigma_r = self.sigma_0 * ((r / self.r_0) ** 2)
            noise = float(self.rng.normal(0, sigma_r))
            measured_range = max(0.0, r + noise)

            current_reading = {
                "range_m": measured_range,
                "bearing_az_rad": float(bearing_angles_rad[0] + self.rng.normal(0, 1e-4)),
                "bearing_el_rad": float(bearing_angles_rad[1] + self.rng.normal(0, 1e-4)),
                "sigma_m": sigma_r,
                "status": "VALID_RETURN",
                "valid": True,
            }

        # Push to latency queue
        self.buffer.append(current_reading)

        # Return delayed measurement from front of queue
        return self.buffer.popleft()

    def sample_range(self, true_range_m: float) -> Dict[str, Any]:
        """Convenience method returning immediate simulated range reading."""
        r = max(0.1, float(true_range_m))
        sigma_r = self.sigma_0 * ((r / self.r_0) ** 2)
        noise = float(self.rng.normal(0, sigma_r))
        is_dropout = bool(self.rng.uniform(0.0, 1.0) < self.p_dropout)
        meas_r = 0.0 if is_dropout else max(0.0, r + noise)
        return {
            "range_m": meas_r if not is_dropout else None,
            "measured_range_m": meas_r,
            "status": "DROPOUT_DETECTED" if is_dropout else "VALID_RETURN",
            "valid": not is_dropout,
            "dropout": is_dropout,
            "sigma_m": sigma_r,
            "noise_sigma_m": sigma_r,
            "latency_s": self.latency_steps * 0.01,
        }
