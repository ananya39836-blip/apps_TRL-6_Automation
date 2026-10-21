"""Local Trajectory Validator & Reactive Replanning Sentinel.

Continuously monitors planned trajectory against rolling perception costmaps
and commands immediate replans or emergency stops upon hazard detection.
"""
from __future__ import annotations
import math
from typing import List, Tuple, Optional
import numpy as np

from trl9_autonomy.planning.collision_checker import CollisionCheckerOBB


class LocalPathSentinel:
    """Monitors the active forward path corridor for newly discovered obstacles."""

    def __init__(self, forward_lookahead_m: float = 6.0) -> None:
        self.lookahead_m = forward_lookahead_m
        self.checker = CollisionCheckerOBB()

    def is_path_safe(
        self,
        current_pose: Tuple[float, float, float],
        path: List[Tuple[float, float, float]],
        impassable_mask: np.ndarray,
        res: float,
    ) -> Tuple[bool, Optional[int]]:
        """Checks path ahead of rover within lookahead distance.

        Returns:
            Tuple of (is_safe: bool, first_colliding_index: Optional[int]).
        """
        if not path:
            return False, None

        rx, ry, ryaw = current_pose
        for i, (px, py, pyaw) in enumerate(path):
            dist = math.hypot(px - rx, py - ry)
            if dist > self.lookahead_m:
                break  # Beyond local monitoring horizon

            if self.checker.check_pose_collision(px, py, pyaw, impassable_mask, res):
                return False, i

        return True, None
