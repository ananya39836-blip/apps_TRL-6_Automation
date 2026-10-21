"""Adaptive Pure Pursuit Path Tracking Controller with Curvature Speed Derating.

Controls rover steering and velocity to follow reference paths while strictly
enforcing lateral tip-over acceleration limits (a_lat <= 0.30 m/s^2).
"""
from __future__ import annotations
import math
from typing import List, Tuple, Optional
import numpy as np


class AdaptivePurePursuitTracker:
    """Adaptive Pure Pursuit controller with forward closest-point search."""

    def __init__(
        self,
        wheelbase_m: float = 2.20,
        min_lookahead_m: float = 1.0,
        max_lookahead_m: float = 3.5,
        lookahead_ratio_s: float = 2.5,
        max_velocity_mps: float = 0.40,
        max_steering_rad: float = 0.65,
        max_lateral_accel_mps2: float = 0.30,
    ) -> None:
        self.wheelbase_m = wheelbase_m
        self.min_lookahead = min_lookahead_m
        self.max_lookahead = max_lookahead_m
        self.lookahead_ratio = lookahead_ratio_s
        self.max_velocity = max_velocity_mps
        self.max_steering = max_steering_rad
        self.max_lat_accel = max_lateral_accel_mps2

    def compute_control(
        self,
        current_pose: Tuple[float, float, float],
        current_velocity: float,
        path: List[Tuple[float, float, float]],
    ) -> Tuple[float, float, float]:
        """Computes commanded velocity (m/s), steering angle (rad), and cross-track error (m).

        Returns:
            Tuple of (cmd_velocity_mps, cmd_steer_rad, cross_track_error_m).
        """
        if not path:
            return 0.0, 0.0, 0.0

        rx, ry, ryaw = current_pose

        # 1. Find index of closest waypoint on path to avoid chasing points behind
        min_dist = float("inf")
        closest_idx = 0
        for i, (px, py, _) in enumerate(path):
            d = math.hypot(px - rx, py - ry)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        cross_track_error = min_dist

        # 2. Dynamic Lookahead Distance based on current speed
        lookahead = float(np.clip(
            self.lookahead_ratio * abs(current_velocity) + self.min_lookahead,
            self.min_lookahead,
            self.max_lookahead,
        ))

        # 3. Search forward along path from closest_idx for the lookahead target
        target_pt = path[-1]
        for i in range(closest_idx, len(path)):
            px, py, _ = path[i]
            dist_to_rover = math.hypot(px - rx, py - ry)
            if dist_to_rover >= lookahead:
                target_pt = path[i]
                break

        tx, ty, _ = target_pt
        dx = tx - rx
        dy = ty - ry

        # Transform target vector to rover Body frame B
        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)
        target_x_b = cos_yaw * dx + sin_yaw * dy
        target_y_b = -sin_yaw * dx + cos_yaw * dy

        # Pure pursuit curvature: kappa = 2 * y_b / (L_d^2)
        dist_sq = target_x_b**2 + target_y_b**2
        if dist_sq < 1e-4:
            return 0.0, 0.0, cross_track_error

        kappa = 2.0 * target_y_b / dist_sq
        cmd_steer = math.atan(self.wheelbase_m * kappa)
        cmd_steer = float(np.clip(cmd_steer, -self.max_steering, self.max_steering))

        # 4. Curvature Speed Derating: v_max = sqrt(a_lat_max / |kappa|)
        if abs(kappa) > 1e-3:
            v_derated = math.sqrt(self.max_lat_accel / abs(kappa))
            cmd_velocity = min(self.max_velocity, v_derated)
        else:
            cmd_velocity = self.max_velocity

        # Decelerate near final goal
        dist_to_final = math.hypot(path[-1][0] - rx, path[-1][1] - ry)
        if dist_to_final < 1.5:
            cmd_velocity = min(cmd_velocity, max(0.08, cmd_velocity * (dist_to_final / 1.5)))

        return cmd_velocity, cmd_steer, cross_track_error
