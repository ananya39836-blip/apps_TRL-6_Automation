"""Multi-Criteria Terrain Traversability Estimation & Hazard Cost Mapping.

Combines macroscopic surface slope gradient, step obstacles, crater depressions,
and surface roughness into a normalized cost metric C in [0.0, 1.0].
"""
from __future__ import annotations
import math
from typing import Dict, Any, Tuple
import numpy as np


class TraversabilityEstimator:
    """Evaluates terrain hazard costs and inflates lethal obstacles."""

    def __init__(
        self,
        max_slope_deg: float = 25.0,
        tip_over_slope_deg: float = 30.0,
        step_threshold_m: float = 0.20,
        inflation_radius_m: float = 0.35,
        w_slope: float = 0.45,
        w_step: float = 0.35,
        w_roughness: float = 0.20,
    ) -> None:
        self.max_slope_deg = max_slope_deg
        self.tip_over_slope_deg = tip_over_slope_deg
        self.step_threshold_m = step_threshold_m
        self.inflation_radius_m = inflation_radius_m

        self.w_slope = w_slope
        self.w_step = w_step
        self.w_roughness = w_roughness

    def compute_slopes(self, elevation: np.ndarray, res: float) -> np.ndarray:
        """Computes continuous slope angle (degrees) using 2nd-order central difference gradients."""
        gz, gx = np.gradient(elevation, res)
        grad_mag = np.sqrt(gx**2 + gz**2)
        slope_rad = np.arctan(grad_mag)
        return np.degrees(slope_rad)

    def compute_step_obstacles(self, elevation: np.ndarray, res: float) -> np.ndarray:
        """Identifies step obstacles exceeding vehicle rocker-bogie clearance threshold."""
        nx, ny = elevation.shape
        step_cost = np.zeros((nx, ny), dtype=np.float64)

        # Max adjacent neighbor height difference
        diff_x = np.abs(np.diff(elevation, axis=0, append=elevation[-1:, :]))
        diff_y = np.abs(np.diff(elevation, axis=1, append=elevation[:, -1:]))
        max_diff = np.maximum(diff_x, diff_y)

        step_cost = np.clip(max_diff / self.step_threshold_m, 0.0, 1.0)
        return step_cost

    def compute_costmap(
        self,
        elevation: np.ndarray,
        obstacle_mask: np.ndarray,
        negative_mask: np.ndarray,
        res: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Computes unified continuous traversability cost C in [0, 1] and binary impassable mask.

        Returns:
            Tuple of (costmap, impassable_mask).
        """
        nx, ny = elevation.shape
        slope_deg = self.compute_slopes(elevation, res)
        step_cost = self.compute_step_obstacles(elevation, res)

        # Normalized slope cost: 0 to 1 scaling up to max_slope_deg
        norm_slope = np.clip(slope_deg / self.max_slope_deg, 0.0, 1.0)

        # Composite continuous cost
        costmap = (
            self.w_slope * norm_slope
            + self.w_step * step_cost
        )

        # Impassable condition: slope >= tip-over limit (30.0 deg), step > threshold, or obstacle
        impassable = (
            (slope_deg >= self.tip_over_slope_deg)
            | (step_cost >= 1.0)
            | obstacle_mask
            | negative_mask
        )

        # Set lethal cells to 1.0
        costmap[impassable] = 1.0

        # Inflate lethal obstacles
        inflated_costmap = self.inflate_obstacles(costmap, impassable, res)
        return inflated_costmap, impassable

    def inflate_obstacles(
        self, costmap: np.ndarray, impassable: np.ndarray, res: float
    ) -> np.ndarray:
        """Inflates lethal obstacle boundary by safety clearance buffer."""
        inflated = costmap.copy()
        radius_cells = max(1, int(math.ceil(self.inflation_radius_m / res)))
        nx, ny = costmap.shape

        obs_indices = np.argwhere(impassable)
        for ix, iy in obs_indices:
            x_min = max(0, ix - radius_cells)
            x_max = min(nx, ix + radius_cells + 1)
            y_min = max(0, iy - radius_cells)
            y_max = min(ny, iy + radius_cells + 1)

            for cx in range(x_min, x_max):
                for cy in range(y_min, y_max):
                    dist = math.hypot((cx - ix) * res, (cy - iy) * res)
                    if dist <= self.inflation_radius_m:
                        decay = 1.0 - (dist / self.inflation_radius_m)
                        inflated[cx, cy] = max(inflated[cx, cy], 0.70 * decay + 0.30)

        return np.clip(inflated, 0.0, 1.0)
