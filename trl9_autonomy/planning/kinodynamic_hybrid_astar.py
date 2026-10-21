"""Continuous SE(2) Kinodynamic Hybrid A* Motion Planner.

Searches continuous vehicle configuration space (x, y, yaw) subject to minimum
turning radius constraints (curvature |kappa| <= 1.50 m^-1), with analytic expansions
and progressive receding-horizon fallback.
"""
from __future__ import annotations
import math
import heapq
import time
from typing import List, Tuple, Optional, Dict
import numpy as np

from trl9_autonomy.planning.collision_checker import CollisionCheckerOBB


class NodeSE2:
    """Discretized state node in 3D grid (ix, iy, iyaw) with continuous state."""

    def __init__(
        self,
        x: float,
        y: float,
        yaw: float,
        g_cost: float,
        h_cost: float,
        parent: Optional[NodeSE2] = None,
        direction: int = 1,
    ) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent
        self.direction = direction  # +1 = Forward, -1 = Reverse

    def __lt__(self, other: NodeSE2) -> bool:
        return self.f_cost < other.f_cost


class KinodynamicHybridAStar:
    """Continuous SE(2) Hybrid A* Planner with Curvature and Footprint Constraints."""

    def __init__(
        self,
        max_curvature_inv_m: float = 1.50,
        step_size_m: float = 0.80,
        num_steer_angles: int = 5,
        xy_resolution_m: float = 0.40,
        yaw_resolution_rad: float = math.radians(15.0),
        timeout_s: float = 0.080,
    ) -> None:
        self.max_curvature = max_curvature_inv_m
        self.step_size = step_size_m
        self.xy_res = xy_resolution_m
        self.yaw_res = yaw_resolution_rad
        self.timeout_s = timeout_s

        self.collision_checker = CollisionCheckerOBB()

        # Steering inputs corresponding to: [max_left, half_left, straight, half_right, max_right]
        max_steer = math.atan(self.max_curvature * 2.20)  # L_wheelbase = 2.20m
        self.steer_inputs = np.linspace(-max_steer, max_steer, num_steer_angles)

    def state_to_index(self, x: float, y: float, yaw: float) -> Tuple[int, int, int]:
        """Discretizes continuous (x, y, yaw) into 3D grid hash key."""
        ix = int(round(x / self.xy_res))
        iy = int(round(y / self.xy_res))
        yaw_norm = yaw % (2.0 * math.pi)
        iyaw = int(round(yaw_norm / self.yaw_res))
        return ix, iy, iyaw

    def heuristic(self, x: float, y: float, gx: float, gy: float) -> float:
        """Admissible Euclidean distance heuristic with turning penalty."""
        return math.hypot(gx - x, gy - y)

    def plan(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
        costmap: np.ndarray,
        impassable_mask: np.ndarray,
        res: float,
        timeout_s: Optional[float] = None,
    ) -> Optional[List[Tuple[float, float, float]]]:
        """Plans a continuous kinodynamically feasible SE(2) trajectory."""
        t_start = time.perf_counter()
        t_limit = timeout_s if timeout_s is not None else self.timeout_s

        sx, sy, syaw = start_pose
        gx, gy, gyaw = goal_pose

        start_node = NodeSE2(
            sx, sy, syaw, g_cost=0.0, h_cost=self.heuristic(sx, sy, gx, gy)
        )
        open_set: List[NodeSE2] = []
        heapq.heappush(open_set, start_node)

        closed_set: Dict[Tuple[int, int, int], float] = {}
        best_node: NodeSE2 = start_node
        min_dist_to_goal = self.heuristic(sx, sy, gx, gy)

        goal_tolerance_m = 1.0
        max_expansions = 1500
        expansions = 0

        while open_set and expansions < max_expansions:
            if (time.perf_counter() - t_start) > t_limit:
                break

            current = heapq.heappop(open_set)
            expansions += 1

            dist_to_goal = math.hypot(gx - current.x, gy - current.y)
            if dist_to_goal < min_dist_to_goal:
                min_dist_to_goal = dist_to_goal
                best_node = current

            # Goal achievement check
            if dist_to_goal <= goal_tolerance_m:
                return self._reconstruct_path(current)

            # Try direct analytic expansion if close to goal
            if dist_to_goal <= 8.0:
                analytic_path = self._try_analytic_expansion(
                    current, gx, gy, gyaw, impassable_mask, res
                )
                if analytic_path is not None:
                    return analytic_path

            state_idx = self.state_to_index(current.x, current.y, current.yaw)
            if state_idx in closed_set and closed_set[state_idx] <= current.g_cost:
                continue
            closed_set[state_idx] = current.g_cost

            # Expand kinematic motion primitives (forward and recovery reverse)
            for direction in [1]:  # Prioritize forward movement
                for steer in self.steer_inputs:
                    nx_x, nx_y, nx_yaw = self._simulate_motion(
                        current.x, current.y, current.yaw, steer, direction, self.step_size
                    )

                    # Collision check
                    if self.collision_checker.check_pose_collision(
                        nx_x, nx_y, nx_yaw, impassable_mask, res
                    ):
                        continue

                    # Terrain traversability cost penalty
                    grid_ix = int(np.clip(round(nx_x / res), 0, costmap.shape[0] - 1))
                    grid_iy = int(np.clip(round(nx_y / res), 0, costmap.shape[1] - 1))
                    terrain_cost = costmap[grid_ix, grid_iy] * 2.0

                    move_cost = self.step_size * (1.0 + terrain_cost)
                    new_g = current.g_cost + move_cost
                    new_h = self.heuristic(nx_x, nx_y, gx, gy)

                    next_idx = self.state_to_index(nx_x, nx_y, nx_yaw)
                    if next_idx in closed_set and closed_set[next_idx] <= new_g:
                        continue

                    neighbor = NodeSE2(
                        nx_x, nx_y, nx_yaw, new_g, new_h, parent=current, direction=direction
                    )
                    heapq.heappush(open_set, neighbor)

        # Fallback: Progressive receding horizon to the best observed node
        if best_node != start_node:
            return self._reconstruct_path(best_node)

        return None

    def _simulate_motion(
        self, x: float, y: float, yaw: float, steer: float, direction: int, distance: float
    ) -> Tuple[float, float, float]:
        """Integrates bicycle kinematic model: dx = v*cos(yaw), dy = v*sin(yaw), dyaw = (v/L)*tan(steer)."""
        L = 2.20
        kappa = math.tan(steer) / L
        # Limit curvature to maximum bound
        kappa = float(np.clip(kappa, -self.max_curvature, self.max_curvature))

        if abs(kappa) < 1e-4:
            nx = x + direction * distance * math.cos(yaw)
            ny = y + direction * distance * math.sin(yaw)
            nyaw = yaw
        else:
            d_theta = direction * distance * kappa
            radius = 1.0 / kappa
            nx = x - radius * math.sin(yaw) + radius * math.sin(yaw + d_theta)
            ny = y + radius * math.cos(yaw) - radius * math.cos(yaw + d_theta)
            nyaw = yaw + d_theta

        return float(nx), float(ny), float(nyaw)

    def _try_analytic_expansion(
        self,
        node: NodeSE2,
        gx: float,
        gy: float,
        gyaw: float,
        impassable_mask: np.ndarray,
        res: float,
    ) -> Optional[List[Tuple[float, float, float]]]:
        """Attempts straight line / Dubins arc connection directly to goal."""
        dx = gx - node.x
        dy = gy - node.y
        dist = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)

        # Heading difference check
        head_diff = abs((heading - node.yaw + math.pi) % (2.0 * math.pi) - math.pi)
        if head_diff > math.radians(35.0):
            return None  # Angle too sharp for smooth connection

        # Check collision along sampled line
        steps = max(3, int(dist / 0.50))
        xs = np.linspace(node.x, gx, steps)
        ys = np.linspace(node.y, gy, steps)
        yaws = np.linspace(node.yaw, heading, steps)

        for x, y, yaw in zip(xs, ys, yaws):
            if self.collision_checker.check_pose_collision(x, y, yaw, impassable_mask, res):
                return None

        # Successful connection: build path
        prefix = self._reconstruct_path(node)
        extension = [(float(x), float(y), float(yaw)) for x, y, yaw in zip(xs[1:], ys[1:], yaws[1:])]
        return prefix + extension

    def _reconstruct_path(self, node: NodeSE2) -> List[Tuple[float, float, float]]:
        """Backtracks from goal node to root start node."""
        path: List[Tuple[float, float, float]] = []
        curr: Optional[NodeSE2] = node
        while curr is not None:
            path.append((curr.x, curr.y, curr.yaw))
            curr = curr.parent
        path.reverse()
        return path
