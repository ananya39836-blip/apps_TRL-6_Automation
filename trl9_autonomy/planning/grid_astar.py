"""Geometric 2D Grid A* Planner Baseline.

Provides a classical discrete 8-connected grid A* algorithm for comparative
scientific benchmarking against the Kinodynamic Hybrid A* planner.
"""
from __future__ import annotations
import math
import heapq
from typing import List, Tuple, Optional, Dict
import numpy as np


class GridAStarBaseline:
    """Standard 2D grid A* shortest path search on discretized costmaps."""

    def __init__(self) -> None:
        pass

    def plan(
        self,
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
        costmap: np.ndarray,
        impassable_mask: np.ndarray,
        res: float,
    ) -> Optional[List[Tuple[float, float, float]]]:
        """Plans a 2D geometric path from start to goal."""
        sx, sy = start_xy
        gx, gy = goal_xy

        six, siy = int(round(sx / res)), int(round(sy / res))
        gix, giy = int(round(gx / res)), int(round(gy / res))

        nx, ny = costmap.shape
        six = int(np.clip(six, 0, nx - 1))
        siy = int(np.clip(siy, 0, ny - 1))
        gix = int(np.clip(gix, 0, nx - 1))
        giy = int(np.clip(giy, 0, ny - 1))

        open_set: List[Tuple[float, int, int]] = []
        heapq.heappush(open_set, (0.0, six, siy))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_cost: Dict[Tuple[int, int], float] = {(six, siy): 0.0}

        # 8-connected motion primitives
        moves = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
        ]

        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (gix, giy) or math.hypot(cx - gix, cy - giy) <= 1:
                # Reconstruct path
                path_xy: List[Tuple[float, float, float]] = []
                curr = (cx, cy)
                while curr in came_from:
                    wx = (curr[0] + 0.5) * res
                    wy = (curr[1] + 0.5) * res
                    path_xy.append((wx, wy, 0.0))
                    curr = came_from[curr]
                path_xy.append(((six + 0.5) * res, (siy + 0.5) * res, 0.0))
                path_xy.reverse()

                # Add smooth yaw headings
                for i in range(len(path_xy) - 1):
                    dx = path_xy[i + 1][0] - path_xy[i][0]
                    dy = path_xy[i + 1][1] - path_xy[i][1]
                    yaw = math.atan2(dy, dx)
                    path_xy[i] = (path_xy[i][0], path_xy[i][1], yaw)
                if len(path_xy) > 1:
                    path_xy[-1] = (path_xy[-1][0], path_xy[-1][1], path_xy[-2][2])
                return path_xy

            for dx, dy, move_dist in moves:
                nx_c = cx + dx
                ny_c = cy + dy

                if not (0 <= nx_c < nx and 0 <= ny_c < ny):
                    continue
                if impassable_mask[nx_c, ny_c]:
                    continue

                tentative_g = g_cost[(cx, cy)] + move_dist * res * (1.0 + costmap[nx_c, ny_c] * 3.0)
                if (nx_c, ny_c) not in g_cost or tentative_g < g_cost[(nx_c, ny_c)]:
                    g_cost[(nx_c, ny_c)] = tentative_g
                    came_from[(nx_c, ny_c)] = (cx, cy)
                    h = math.hypot(gix - nx_c, giy - ny_c) * res
                    heapq.heappush(open_set, (tentative_g + h, nx_c, ny_c))

        return None
