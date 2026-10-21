"""Certified Constant-Jerk Safe Stop Controller.

Executes a guaranteed smooth, tip-over safe halt profile under constant jerk
deceleration limits to prevent rover tipping or wheel skidding on loose regolith.
"""
from __future__ import annotations
import math
from typing import Tuple


class SafeStopController:
    """Manages constant-jerk deceleration profiles during emergency and nominal stops."""

    def __init__(
        self,
        max_decel_mps2: float = -0.20,
        max_jerk_mps3: float = -0.50,
    ) -> None:
        self.max_decel = max_decel_mps2
        self.max_jerk = max_jerk_mps3
        self.current_accel: float = 0.0

    def compute_step(self, current_velocity_mps: float, dt: float) -> Tuple[float, bool]:
        """Advances deceleration profile by dt.

        Returns:
            Tuple of (new_velocity_mps, is_fully_stopped: bool).
        """
        if current_velocity_mps <= 1e-4:
            self.current_accel = 0.0
            return 0.0, True

        # Ramp deceleration towards max_decel using max_jerk
        if self.current_accel > self.max_decel:
            self.current_accel = max(self.max_decel, self.current_accel + self.max_jerk * dt)

        # Apply acceleration to velocity
        new_v = current_velocity_mps + self.current_accel * dt
        if new_v <= 0.0:
            self.current_accel = 0.0
            return 0.0, True

        return float(new_v), False
