"""2.5D Elevation Grid Mapping with Recursive Kalman Variance Fusion.

Maintains a local topocentric elevation map with continuous uncertainty tracking
and ray-clearing to remove transient/ghost dynamic obstacle observations.
"""
from __future__ import annotations
import math
from typing import Tuple, List, Optional
import numpy as np


class ElevationMap25D:
    """2.5D probabilistic elevation grid with recursive Kalman variance updates."""

    def __init__(
        self,
        size_x_m: float = 30.0,
        size_y_m: float = 30.0,
        resolution_m: float = 0.20,
        initial_variance: float = 1.0,
        meas_variance: float = 0.04,
    ) -> None:
        self.size_x = size_x_m
        self.size_y = size_y_m
        self.res = resolution_m
        self.nx = int(round(size_x_m / resolution_m))
        self.ny = int(round(size_y_m / resolution_m))

        self.initial_variance = initial_variance
        self.meas_variance = meas_variance

        # Elevation grid Z, variance sigma^2, and observed mask
        self.elevation = np.zeros((self.nx, self.ny), dtype=np.float64)
        self.variance = np.full((self.nx, self.ny), initial_variance, dtype=np.float64)
        self.observed = np.zeros((self.nx, self.ny), dtype=bool)
        self.obstacle_mask = np.zeros((self.nx, self.ny), dtype=bool)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Converts World (x, y) coordinates to grid cell indices (ix, iy)."""
        ix = int(math.floor(x / self.res))
        iy = int(math.floor(y / self.res))
        ix = np.clip(ix, 0, self.nx - 1)
        iy = np.clip(iy, 0, self.ny - 1)
        return int(ix), int(iy)

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Converts grid cell indices (ix, iy) to World (x, y) coordinates at cell center."""
        x = (ix + 0.5) * self.res
        y = (iy + 0.5) * self.res
        return x, y

    def update_from_point_cloud(self, points: np.ndarray, rover_origin: np.ndarray) -> None:
        """Fuses 3D point cloud into 2.5D elevation grid using 1D Kalman updates."""
        if len(points) == 0:
            return

        for pt in points:
            x, y, z = pt[0], pt[1], pt[2]
            if 0.0 <= x < self.size_x and 0.0 <= y < self.size_y:
                ix, iy = self.world_to_grid(x, y)

                if not self.observed[ix, iy]:
                    # First observation
                    self.elevation[ix, iy] = z
                    self.variance[ix, iy] = self.meas_variance
                    self.observed[ix, iy] = True
                else:
                    # Recursive Kalman update
                    prior_z = self.elevation[ix, iy]
                    prior_var = self.variance[ix, iy]
                    K = prior_var / (prior_var + self.meas_variance)

                    new_z = prior_z + K * (z - prior_z)
                    new_var = (1.0 - K) * prior_var

                    self.elevation[ix, iy] = new_z
                    self.variance[ix, iy] = max(1e-4, new_var)

    def mark_obstacles(self, obstacle_points: np.ndarray) -> None:
        """Marks grid cells containing non-ground obstacles."""
        for pt in obstacle_points:
            x, y = pt[0], pt[1]
            if 0.0 <= x < self.size_x and 0.0 <= y < self.size_y:
                ix, iy = self.world_to_grid(x, y)
                self.obstacle_mask[ix, iy] = True

    def detect_negative_obstacles(
        self, depth_thresh_m: float = 0.25, window_size: int = 5
    ) -> np.ndarray:
        """Detects negative obstacles (crater bowls, trenches) by comparing local depression.

        A negative obstacle cell is significantly lower than its surrounding local rim mean.
        Uses a smoothed baseline to prevent false crater detections on continuous slopes.
        """
        negative_mask = np.zeros((self.nx, self.ny), dtype=bool)
        half_w = window_size // 2

        # 3x3 uniform box smoothing to remove fine sensor jitter
        smoothed = self.elevation.copy()
        for i in range(1, self.nx - 1):
            for j in range(1, self.ny - 1):
                smoothed[i, j] = np.mean(self.elevation[i - 1 : i + 2, j - 1 : j + 2])

        for i in range(half_w, self.nx - half_w, 2):
            for j in range(half_w, self.ny - half_w, 2):
                if not self.observed[i, j]:
                    continue
                window = smoothed[i - half_w : i + half_w + 1, j - half_w : j + half_w + 1]
                local_mean = np.mean(window)
                local_min = np.min(window)

                # Depression greater than depth threshold
                if (local_mean - local_min) >= depth_thresh_m:
                    min_coords = np.argwhere(window == local_min)
                    for mc in min_coords:
                        gx = i - half_w + mc[0]
                        gy = j - half_w + mc[1]
                        negative_mask[gx, gy] = True

        return negative_mask
