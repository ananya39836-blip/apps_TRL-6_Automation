"""6-Wheel Rocker-Bogie Kinematic Articulation & Normal Load Solver.

Models passive pitch bogie articulation and differential bar tilt to maintain
all 6 wheels in continuous ground contact across uneven Martian terrain.
"""
from __future__ import annotations
import math
from typing import Dict, List, Tuple
import numpy as np


class RockerBogieKinematics:
    """Computes articulation angles and normal wheel loads for a 6-wheel rover."""

    def __init__(
        self,
        rover_mass_kg: float = 850.0,
        g_mars: float = 3.72076,
        track_width_m: float = 2.00,
        wheelbase_m: float = 2.20,
    ) -> None:
        self.mass = rover_mass_kg
        self.g = g_mars
        self.total_weight_n = self.mass * self.g  # ~3162.6 N
        self.track_width = track_width_m
        self.wheelbase = wheelbase_m

        # 6 Wheel relative offsets from chassis center:
        # [Front-Left, Mid-Left, Rear-Left, Front-Right, Mid-Right, Rear-Right]
        self.wheel_offsets = {
            "FL": np.array([wheelbase_m * 0.5, track_width_m * 0.5, 0.0]),
            "ML": np.array([0.0, track_width_m * 0.5, 0.0]),
            "RL": np.array([-wheelbase_m * 0.5, track_width_m * 0.5, 0.0]),
            "FR": np.array([wheelbase_m * 0.5, -track_width_m * 0.5, 0.0]),
            "MR": np.array([0.0, -track_width_m * 0.5, 0.0]),
            "RR": np.array([-wheelbase_m * 0.5, -track_width_m * 0.5, 0.0]),
        }

    def compute_articulation(
        self, wheel_contact_z: Dict[str, float]
    ) -> Dict[str, float]:
        """Computes left/right rocker pitch, left/right bogie pitch, and differential tilt angles."""
        # Left side contact heights
        z_fl = wheel_contact_z.get("FL", 0.0)
        z_ml = wheel_contact_z.get("ML", 0.0)
        z_rl = wheel_contact_z.get("RL", 0.0)

        # Right side contact heights
        z_fr = wheel_contact_z.get("FR", 0.0)
        z_mr = wheel_contact_z.get("MR", 0.0)
        z_rr = wheel_contact_z.get("RR", 0.0)

        # Bogie pitch angles (between mid and rear wheels)
        half_wb = self.wheelbase * 0.5
        pitch_bogie_l = math.atan2(z_ml - z_rl, half_wb)
        pitch_bogie_r = math.atan2(z_mr - z_rr, half_wb)

        # Rocker pitch angles (between front wheel and bogie pivot)
        z_pivot_l = 0.5 * (z_ml + z_rl)
        z_pivot_r = 0.5 * (z_mr + z_rr)
        pitch_rocker_l = math.atan2(z_fl - z_pivot_l, half_wb)
        pitch_rocker_r = math.atan2(z_fr - z_pivot_r, half_wb)

        # Differential bar averages the left and right rocker pitches
        diff_tilt = 0.5 * (pitch_rocker_l - pitch_rocker_r)

        return {
            "bogie_left_rad": float(pitch_bogie_l),
            "bogie_right_rad": float(pitch_bogie_r),
            "rocker_left_rad": float(pitch_rocker_l),
            "rocker_right_rad": float(pitch_rocker_r),
            "differential_tilt_rad": float(diff_tilt),
        }

    def compute_normal_loads(
        self, roll_rad: float, pitch_rad: float
    ) -> Dict[str, float]:
        """Computes normal ground contact forces (N) across all 6 wheels.

        Sum of forces equals total gravitational weight: W = m * g_mars.
        """
        # Nominal equal distribution (1/6 per wheel)
        base_load = self.total_weight_n / 6.0

        # Roll load transfer (lateral tilt shifts weight port/starboard)
        delta_roll = (self.total_weight_n * math.sin(roll_rad) * 0.3) / self.track_width

        # Pitch load transfer (longitudinal tilt shifts weight front/rear)
        delta_pitch = (self.total_weight_n * math.sin(pitch_rad) * 0.3) / self.wheelbase

        loads = {
            "FL": float(np.clip(base_load - delta_roll + delta_pitch, 50.0, base_load * 2.5)),
            "ML": float(np.clip(base_load - delta_roll, 50.0, base_load * 2.5)),
            "RL": float(np.clip(base_load - delta_roll - delta_pitch, 50.0, base_load * 2.5)),
            "FR": float(np.clip(base_load + delta_roll + delta_pitch, 50.0, base_load * 2.5)),
            "MR": float(np.clip(base_load + delta_roll, 50.0, base_load * 2.5)),
            "RR": float(np.clip(base_load + delta_roll - delta_pitch, 50.0, base_load * 2.5)),
        }

        # Normalize sum exactly to total weight
        raw_sum = sum(loads.values())
        scale = self.total_weight_n / raw_sum
        return {k: v * scale for k, v in loads.items()}
