"""Sensor Rig Extrinsic Calibration Parameters."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class SensorRigCalibration:
    """Rigid body mounting extrinsics from Body origin B to sensor frames."""
    # Body to LiDAR mast mount: +1.0m X, 0.0m Y, +1.8m Z
    r_body_lidar: np.ndarray = np.array([1.0, 0.0, 1.8], dtype=np.float64)
    rpy_body_lidar: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    
    # Body to Stereo Camera mast mount: +1.1m X, 0.0m Y, +1.7m Z, 10 deg pitch down
    r_body_camera: np.ndarray = np.array([1.1, 0.0, 1.7], dtype=np.float64)
    rpy_body_camera: np.ndarray = np.array([0.0, 0.174533, 0.0], dtype=np.float64)
    
    # Body to IMU center: Coincident with center of mass
    r_body_imu: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    rpy_body_imu: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float64)
