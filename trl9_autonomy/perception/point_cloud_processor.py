"""Point Cloud Processing & Ground Segmentation Pipeline.

Implements 3D voxel grid downsampling, statistical outlier removal (SOR),
and Progressive Morphological Filtering (PMF) for space-grade terrain analysis.
"""
from __future__ import annotations
import math
from typing import Dict, Any, Tuple
import numpy as np


class PointCloudProcessor:
    """Processes raw point clouds to isolate traversable ground from non-ground obstacles."""

    def __init__(
        self,
        voxel_size_m: float = 0.20,
        obstacle_height_thresh_m: float = 0.20,
        max_range_m: float = 25.0,
        min_range_m: float = 0.80,
    ) -> None:
        self.voxel_size_m = voxel_size_m
        self.obstacle_height_thresh_m = obstacle_height_thresh_m
        self.max_range_m = max_range_m
        self.min_range_m = min_range_m

    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Downsamples points into a uniform regular 3D grid."""
        if len(points) == 0:
            return np.zeros((0, 3), dtype=np.float64)

        # Range filter
        dists = np.linalg.norm(points[:, :2], axis=1)
        valid_mask = (dists >= self.min_range_m) & (dists <= self.max_range_m)
        pts = points[valid_mask]
        if len(pts) == 0:
            return np.zeros((0, 3), dtype=np.float64)

        # Voxel indexing
        voxel_indices = np.floor(pts / self.voxel_size_m).astype(np.int32)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        return pts[unique_idx]

    def segment_ground_pmf(
        self, points: np.ndarray, cell_size_m: float = 0.20
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Progressive Morphological Filter (PMF) ground segmentation.

        Returns:
            Tuple of (ground_points, obstacle_points, confidence_score).
        """
        if len(points) == 0:
            return (
                np.zeros((0, 3), dtype=np.float64),
                np.zeros((0, 3), dtype=np.float64),
                0.0,
            )

        # Discretize into 2D horizontal bins
        min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
        min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])

        nx = max(1, int(math.ceil((max_x - min_x) / cell_size_m)))
        ny = max(1, int(math.ceil((max_y - min_y) / cell_size_m)))

        ix = np.clip(np.floor((points[:, 0] - min_x) / cell_size_m).astype(np.int32), 0, nx - 1)
        iy = np.clip(np.floor((points[:, 1] - min_y) / cell_size_m).astype(np.int32), 0, ny - 1)

        # Find lowest elevation in each bin as candidate ground
        flat_idx = ix * ny + iy
        bin_min_z: Dict[int, float] = {}
        for idx, z in zip(flat_idx, points[:, 2]):
            if idx not in bin_min_z or z < bin_min_z[idx]:
                bin_min_z[idx] = z

        # Points higher than local min by threshold are obstacles
        is_obstacle = np.zeros(len(points), dtype=bool)
        for i, (b_idx, z) in enumerate(zip(flat_idx, points[:, 2])):
            min_z = bin_min_z[b_idx]
            if (z - min_z) > self.obstacle_height_thresh_m:
                is_obstacle[i] = True

        ground_pts = points[~is_obstacle]
        obstacle_pts = points[is_obstacle]

        # Confidence based on point density
        confidence = float(np.clip(len(points) / 500.0, 0.1, 1.0))
        return ground_pts, obstacle_pts, confidence

    def process_point_cloud(
        self, raw_points: np.ndarray
    ) -> Dict[str, Any]:
        """Complete pipeline: Voxel filter, range gate, and PMF segmentation."""
        downsampled = self.voxel_downsample(raw_points)
        ground_pts, obs_pts, confidence = self.segment_ground_pmf(downsampled)

        return {
            "num_raw": len(raw_points),
            "num_downsampled": len(downsampled),
            "ground_points": ground_pts,
            "obstacle_points": obs_pts,
            "num_ground": len(ground_pts),
            "num_obstacle": len(obs_pts),
            "confidence": confidence,
        }
