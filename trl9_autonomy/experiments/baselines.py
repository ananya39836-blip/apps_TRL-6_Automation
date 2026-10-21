"""Baseline Autonomy Pipelines for Comparative Verification.

1. Pure Odometry (No-EKF, open loop dead-reckoning).
2. Geometric 2D Grid A* Planner (No kinodynamic curvature or footprint OBB checking).
"""
from __future__ import annotations
import math
from typing import List, Tuple, Dict, Any, Optional
import numpy as np

from trl9_autonomy.planning.grid_astar import GridAStarBaseline


class PureOdometryBaseline:
    """Propagates vehicle pose using wheel odometry alone without sensor fusion."""

    def __init__(self, initial_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.x, self.y, self.yaw = initial_pose
        self.trajectory: List[Tuple[float, float, float]] = [(self.x, self.y, self.yaw)]

    def step(self, v_mps: float, yaw_rate_rads: float, dt: float) -> Tuple[float, float, float]:
        """Integrates forward velocity and yaw rate."""
        self.yaw += yaw_rate_rads * dt
        self.x += v_mps * math.cos(self.yaw) * dt
        self.y += v_mps * math.sin(self.yaw) * dt
        self.trajectory.append((self.x, self.y, self.yaw))
        return self.x, self.y, self.yaw
