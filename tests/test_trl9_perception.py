"""Formal Unit & Integration Test Suite for TRL 9 Perception & Sensors.

Verifies:
- TRL9-REQ-01: Research safety guard latches
- TRL9-REQ-02: Rigid SE(3) coordinate transformations
- TRL9-REQ-04: Telemetry validator NaN/Inf and timestamp rejection
- TRL9-REQ-05: 3D point cloud voxel filtering & PMF ground segmentation
- TRL9-REQ-06: 2.5D recursive Kalman elevation grid mapping
- TRL9-REQ-07: Multi-criteria traversability cost and obstacle inflation
"""
from __future__ import annotations
import math
import numpy as np
import pytest

from trl9_autonomy import (
    PROTOTYPE_LOCK,
    NAVIGATION_READY,
    G_MARS,
    SCENARIO_IDS,
)
from trl9_autonomy.sensors.coordinate_transforms import (
    euler_to_quaternion,
    quaternion_to_euler,
    TransformSE3,
    SensorExtrinsics,
)
from trl9_autonomy.sensors.sensor_validator import SensorDataValidator
from trl9_autonomy.perception.point_cloud_processor import PointCloudProcessor
from trl9_autonomy.perception.elevation_mapping import ElevationMap25D
from trl9_autonomy.perception.traversability import TraversabilityEstimator
from trl9_autonomy.perception.perception_metrics import PerceptionMetrics


def test_safety_invariants_latched():
    """Confirms compile-time research guard locks remain strictly active."""
    assert PROTOTYPE_LOCK is True
    assert NAVIGATION_READY is False
    assert len(SCENARIO_IDS) == 20


def test_euler_quaternion_roundtrip():
    """Tests Euler <-> Quaternion mathematical equivalence across 3D orientations."""
    r, p, y = math.radians(12.5), math.radians(-8.0), math.radians(45.0)
    q = euler_to_quaternion(r, p, y)
    assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-6)

    r_est, p_est, y_est = quaternion_to_euler(q)
    assert np.isclose(r, r_est, atol=1e-4)
    assert np.isclose(p, p_est, atol=1e-4)
    assert np.isclose(y, y_est, atol=1e-4)


def test_transform_se3_composition_and_inverse():
    """Tests SE(3) transformation composition and inversion identity T * T^-1 = I."""
    t1 = TransformSE3.from_xyz_rpy(2.0, 3.0, 1.0, 0.1, -0.2, 0.5)
    t1_inv = t1.inverse()
    ident = t1.compose(t1_inv)
    assert np.allclose(ident.matrix, np.eye(4), atol=1e-10)

    # Point round-trip
    pts = np.array([[5.0, 2.0, 1.0], [0.0, 0.0, 0.0]], dtype=np.float64)
    pts_trans = t1.transform_points(pts)
    pts_recovered = t1_inv.transform_points(pts_trans)
    assert np.allclose(pts, pts_recovered, atol=1e-10)


def test_sensor_validator_rejects_nan_inf():
    """Validates telemetry gatekeeper rejects NaN, Inf, and non-monotonic timestamps."""
    validator = SensorDataValidator(max_accel_mps2=50.0, max_gyro_rads=10.0)

    # Nominal check
    assert validator.validate_imu(0.1, 0.0, 3.72, 0.01, 0.0, 0.0, 1.0) is True

    # NaN / Inf rejection
    assert validator.validate_imu(float("nan"), 0.0, 3.72, 0.0, 0.0, 0.0, 1.01) is False
    assert validator.validate_imu(0.1, float("inf"), 3.72, 0.0, 0.0, 0.0, 1.02) is False

    # Timestamp monotonicity
    assert validator.validate_imu(0.1, 0.0, 3.72, 0.0, 0.0, 0.0, 0.95) is False

    # Out of range acceleration
    assert validator.validate_imu(60.0, 0.0, 3.72, 0.0, 0.0, 0.0, 1.05) is False


def test_point_cloud_processor_voxel_and_ground():
    """Verifies voxel grid downsampling and PMF ground obstacle extraction."""
    processor = PointCloudProcessor(voxel_size_m=0.20, obstacle_height_thresh_m=0.20)

    # Synthesize flat terrain with a rock obstacle
    ground = np.array([[x, y, 0.0] for x in np.linspace(1.0, 5.0, 15) for y in np.linspace(1.0, 5.0, 15)])
    rock = np.array([[3.0, 3.0, 0.50], [3.1, 3.0, 0.45]])
    raw_cloud = np.vstack([ground, rock])

    res = processor.process_point_cloud(raw_cloud)
    assert res["num_downsampled"] > 0
    assert res["num_ground"] >= 50
    assert res["num_obstacle"] >= 1
    assert res["confidence"] > 0.1


def test_elevation_mapping_kalman_update():
    """Verifies recursive Kalman variance reduction across sequential observations."""
    grid = ElevationMap25D(size_x_m=10.0, size_y_m=10.0, resolution_m=0.20)
    pt1 = np.array([[2.0, 2.0, 1.5]])

    ix, iy = grid.world_to_grid(2.0, 2.0)
    grid.update_from_point_cloud(pt1, np.zeros(3))
    var1 = grid.variance[ix, iy]

    # Second observation of same point reduces variance
    grid.update_from_point_cloud(pt1, np.zeros(3))
    var2 = grid.variance[ix, iy]

    assert var2 < var1
    assert grid.observed[ix, iy] is True or grid.observed[ix, iy] == True


def test_negative_obstacle_detection():
    """Verifies detection of crater bowl depressions."""
    grid = ElevationMap25D(size_x_m=10.0, size_y_m=10.0, resolution_m=0.20)
    grid.observed[:, :] = True
    grid.elevation[:, :] = 0.0  # Flat plain

    # Crater depression at center (5.0, 5.0)
    cx, cy = grid.world_to_grid(5.0, 5.0)
    grid.elevation[cx - 1 : cx + 2, cy - 1 : cy + 2] = -0.50

    neg_mask = grid.detect_negative_obstacles(depth_thresh_m=0.25, window_size=5)
    assert np.any(neg_mask[cx - 1 : cx + 2, cy - 1 : cy + 2])


def test_traversability_evaluation_and_inflation():
    """Verifies slope cost scaling and obstacle inflation."""
    estimator = TraversabilityEstimator(max_slope_deg=25.0, tip_over_slope_deg=30.0)
    elev = np.zeros((30, 30), dtype=np.float64)
    obs_mask = np.zeros((30, 30), dtype=bool)
    neg_mask = np.zeros((30, 30), dtype=bool)

    # Smooth flat terrain has low cost
    costmap, impassable = estimator.compute_costmap(elev, obs_mask, neg_mask, res=0.20)
    assert np.all(costmap <= 0.20)
    assert not np.any(impassable)

    # Inject obstacle at center
    obs_mask[15, 15] = True
    costmap_obs, impassable_obs = estimator.compute_costmap(elev, obs_mask, neg_mask, res=0.20)
    assert impassable_obs[15, 15] == True
    assert costmap_obs[15, 15] == 1.0
    # Surrounding buffer should be inflated
    assert costmap_obs[15, 16] > 0.50


def test_perception_metrics():
    """Verifies IoU, Precision, Recall, and RMSE calculation against ground truth."""
    pred_mask = np.array([[True, False], [False, True]])
    gt_mask = np.array([[True, True], [False, True]])

    metrics = PerceptionMetrics.evaluate_obstacle_detection(pred_mask, gt_mask)
    assert metrics["precision"] == 1.0
    assert np.isclose(metrics["recall"], 2.0 / 3.0)
    assert metrics["iou"] == 2.0 / 3.0

    pred_elev = np.array([1.0, 2.0, 3.0])
    gt_elev = np.array([1.1, 1.9, 3.0])
    obs = np.array([True, True, True])
    elev_metrics = PerceptionMetrics.evaluate_elevation_reconstruction(pred_elev, gt_elev, obs)
    assert elev_metrics["rmse_m"] > 0.0
    assert elev_metrics["rmse_m"] < 0.15
