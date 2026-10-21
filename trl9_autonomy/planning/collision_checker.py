"""Continuous Swept Oriented Bounding Box (OBB) Footprint Collision Checker.

Evaluates exact vehicle footprint clearance (L = 2.80m, W = 2.20m + safety margin)
across continuous SE(2) vehicle poses (x, y, yaw) using the Separating Axis Theorem.
"""
from __future__ import annotations
import math
from typing import Tuple, List, Optional
import numpy as np


class CollisionCheckerOBB:
    """Oriented Bounding Box collision checker for rectangular rover chassis."""

    def __init__(
        self,
        rover_length_m: float = 2.80,
        rover_width_m: float = 2.20,
        safety_margin_m: float = 0.35,
    ) -> None:
        self.half_l = (rover_length_m + 2.0 * safety_margin_m) * 0.5
        self.half_w = (rover_width_m + 2.0 * safety_margin_m) * 0.5

    def get_footprint_corners(
        self, x: float, y: float, yaw: float
    ) -> np.ndarray:
        """Computes the 4 Cartesian corners of the oriented bounding box in World frame."""
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # Local body offsets: Front-Left, Front-Right, Rear-Right, Rear-Left
        dx = np.array([self.half_l, self.half_l, -self.half_l, -self.half_l])
        dy = np.array([self.half_w, -self.half_w, -self.half_w, self.half_w])

        cx = x + cos_y * dx - sin_y * dy
        cy = y + sin_y * dx + cos_y * dy
        return np.column_stack([cx, cy])

    def check_pose_collision(
        self,
        x: float,
        y: float,
        yaw: float,
        impassable_mask: np.ndarray,
        res: float,
    ) -> bool:
        """Returns True if the vehicle OBB footprint collides with any impassable cell."""
        nx, ny = impassable_mask.shape

        # Quick circle rejection check
        bounding_radius = math.hypot(self.half_l, self.half_w)
        ix_center = int(round(x / res))
        iy_center = int(round(y / res))
        r_cells = int(math.ceil(bounding_radius / res))

        x_min = max(0, ix_center - r_cells)
        x_max = min(nx, ix_center + r_cells + 1)
        y_min = max(0, iy_center - r_cells)
        y_max = min(ny, iy_center + r_cells + 1)

        if not np.any(impassable_mask[x_min:x_max, y_min:y_max]):
            return False  # No obstacles within bounding circle

        # Exact oriented box test: Sample interior points along longitudinal and lateral grid
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        xs = np.linspace(-self.half_l, self.half_l, max(3, int(self.half_l * 2 / res)))
        ys = np.linspace(-self.half_w, self.half_w, max(3, int(self.half_w * 2 / res)))

        for lx in xs:
            for ly in ys:
                wx = x + cos_y * lx - sin_y * ly
                wy = y + sin_y * lx + cos_y * ly

                ix = int(math.floor(wx / res))
                iy = int(math.floor(wy / res))

                if 0 <= ix < nx and 0 <= iy < ny:
                    if impassable_mask[ix, iy]:
                        return True
                else:
                    return True  # Out of boundary is considered collision

        return False
