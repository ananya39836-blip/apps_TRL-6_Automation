"""Multi-Sensor Rig Synchronization & Extrinsic Fusion Engine."""
from __future__ import annotations
from typing import Dict, Any, Optional
import numpy as np

from trl9_autonomy.sensors.coordinate_transforms import TransformSE3, SensorExtrinsics


class MultiSensorRig:
    """Manages multi-modal sensor synchronization and extrinsic transformations."""

    def __init__(self, extrinsics: Optional[SensorExtrinsics] = None) -> None:
        self.extrinsics = extrinsics if extrinsics is not None else SensorExtrinsics()
        self.last_sync_timestamp: float = 0.0

    def transform_lidar_to_body(self, lidar_pts: np.ndarray) -> np.ndarray:
        """Transforms point cloud from LiDAR frame L to vehicle body frame B."""
        return self.extrinsics.T_body_lidar.transform_points(lidar_pts)

    def transform_camera_to_body(self, camera_pts: np.ndarray) -> np.ndarray:
        """Transforms depth points from Camera frame C to vehicle body frame B."""
        return self.extrinsics.T_body_camera.transform_points(camera_pts)

    def transform_body_to_world(self, body_pts: np.ndarray, rover_pose: TransformSE3) -> np.ndarray:
        """Transforms points from vehicle body frame B to World ENU frame W."""
        return rover_pose.transform_points(body_pts)
