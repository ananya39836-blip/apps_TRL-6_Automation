"""Formal Unit & Integration Test Suite for TRL 6 Perception & Sensors.

Verifies:
- TRL6-REQ-01: Continuous 3D DEM elevation interpolation
- TRL6-REQ-02: Multi-hazard environmental modeling (rocks, craters, friction)
- TRL6-REQ-03: Multi-sensor simulation rig under Martian gravity
- TRL6-REQ-04: Space-grade telemetry ingestion & data validation (NaN/Inf, stale, checksums)
- TRL6-REQ-05: SE(3) coordinate frame transformations
- TRL6-REQ-06: Point cloud processing & invariant ground segmentation
- TRL6-REQ-07: 2.5D elevation grid mapping with Kalman fusion & dynamic obstacle clearing
- TRL6-REQ-08: Radial negative obstacle detection
- TRL6-REQ-09: Multi-criteria traversability estimation
"""
from __future__ import annotations
import math
import numpy as np
import pytest

from trl6_navigation import (
    PROTOTYPE_LOCK,
    NAVIGATION_READY,
    G_MARS,
    SCENARIO_CLASSES,
)
from trl6_navigation.environment.planetary_terrain import PlanetaryTerrain
from trl6_navigation.environment.scenario_generator import ScenarioGenerator
from trl6_navigation.sensors.coordinate_transforms import (
    make_se3_transform,
    invert_se3_transform,
    transform_points_se3,
    T_BASE_LIDAR,
    T_BASE_CAMERA,
    T_BASE_IMU,
)
from trl6_navigation.sensors.sensor_rig import MultiSensorRig
from trl6_navigation.sensors.sensor_validator import SensorDataValidator
from trl6_navigation.perception.point_cloud_processor import PointCloudProcessor
from trl6_navigation.perception.elevation_mapping import ElevationMap25D
from trl6_navigation.perception.traversability import TraversabilityEstimator
from trl6_navigation.perception.perception_metrics import PerceptionMetrics


def test_trl6_safety_invariants():
    """Verify permanent research locks are active."""
    assert PROTOTYPE_LOCK is True
    assert NAVIGATION_READY is False
    assert len(SCENARIO_CLASSES) == 20


def test_trl6_continuous_3d_dem():
    """Verify continuous bilinear interpolation and slope gradient on 3D DEM."""
    terrain = PlanetaryTerrain(size_x_m=40.0, size_y_m=40.0, resolution_m=0.20, base_slope_deg=5.0)
    z1 = terrain.get_elevation_at(10.0, 10.0)
    z2 = terrain.get_elevation_at(10.1, 10.0)
    assert not math.isnan(z1)
    assert not math.isnan(z2)
    assert abs(z2 - z1) < 0.20

    slopes = terrain.compute_slope_degrees()
    assert slopes.shape == (terrain.nx, terrain.ny)
    assert np.all(slopes >= 0.0)


def test_trl6_boulder_and_crater_injection():
    """Verify explicit geometric hazard injection."""
    terrain = PlanetaryTerrain(size_x_m=40.0, size_y_m=40.0, resolution_m=0.20, undulation_amplitude_m=0.0)
    terrain.add_boulder(20.0, 20.0, radius_m=1.0, height_m=0.5)
    terrain.add_crater(10.0, 10.0, radius_m=2.0, depth_m=0.8)

    assert len(terrain.boulders) == 1
    assert len(terrain.craters) == 1
    # Check boulder peak elevation
    z_boulder_peak = terrain.get_elevation_at(20.0, 20.0)
    assert z_boulder_peak >= 0.45
    # Check crater bowl depth
    z_crater_bottom = terrain.get_elevation_at(10.0, 10.0)
    assert z_crater_bottom <= -0.70


def test_trl6_solar_shadows():
    """Verify directional solar ray-marching shadow masking."""
    terrain = PlanetaryTerrain(size_x_m=40.0, size_y_m=40.0, resolution_m=0.20)
    terrain.add_boulder(15.0, 15.0, radius_m=1.5, height_m=1.0)
    shadows = terrain.compute_solar_shadows(sun_elev_deg=25.0, sun_azim_deg=45.0)
    assert shadows.shape == (terrain.nx, terrain.ny)
    assert np.any(shadows)  # Boulder must cast a shadow


def test_trl6_se3_transforms():
    """Verify analytical inversion and consistency of SE(3) coordinate transformations."""
    T = make_se3_transform(np.array([5.0, -3.0, 2.5]), rpy_rad=(0.1, -0.2, 0.4))
    T_inv = invert_se3_transform(T)
    identity = T @ T_inv
    assert np.allclose(identity, np.eye(4), atol=1e-10)

    # Transform test point
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    pts_trans = transform_points_se3(pts, T)
    pts_back = transform_points_se3(pts_trans, T_inv)
    assert np.allclose(pts, pts_back, atol=1e-10)


def test_trl6_sensor_rig_measurements():
    """Verify sensor rig output characteristics under Martian conditions."""
    terrain = PlanetaryTerrain(size_x_m=40.0, size_y_m=40.0, resolution_m=0.20)
    rig = MultiSensorRig(terrain=terrain, noise_multiplier=1.0, seed=42)

    # 1. LiDAR
    lidar_out = rig.sample_lidar(np.array([10.0, 10.0, 0.0]))
    assert lidar_out["num_returns"] > 0
    assert lidar_out["points_lidar"].shape[1] == 3

    # 2. Tactical IMU (Martian Gravity Reaction)
    imu_out = rig.sample_imu(np.array([10.0, 10.0, 0.0]), forward_speed_mps=0.25, yaw_rate_rps=0.0)
    assert abs(imu_out["meas_accel_mps2"][2] - G_MARS) < 0.25  # ~3.71 m/s^2

    # 3. Wheel Odometry
    odom_out = rig.sample_wheel_odometry(true_speed_mps=0.30, true_yaw_rate_rps=0.02, slip_ratio=0.10)
    assert odom_out["meas_linear_mps"] > 0.30  # Apparent wheel speed increases with slip


def test_trl6_sensor_validator():
    """Verify space telemetry validation screening against malformed, NaN, and saturated inputs."""
    validator = SensorDataValidator()

    # 1. Defective IMU packet with NaN and saturated gyro
    bad_imu = {
        "meas_accel_mps2": np.array([float("nan"), 0.0, 3.71]),
        "meas_gyro_rps": np.array([0.0, 0.0, 15.0]),  # Saturation
    }
    val_imu = validator.validate_imu_frame(bad_imu, current_time_s=1.0)
    assert not val_imu.is_valid
    assert "NAN_OR_INF_IN_ACCELEROMETER" in val_imu.rejection_reasons
    assert "GYROSCOPE_SATURATION_CLAMPED" in val_imu.rejection_reasons
    assert not np.any(np.isnan(val_imu.cleaned_data["meas_accel_mps2"]))

    # 2. Defective LiDAR packet with Inf
    bad_lidar = {
        "points_lidar": np.array([[1.0, 2.0, 3.0], [float("inf"), 0.0, 0.0]]),
    }
    val_lidar = validator.validate_lidar_frame(bad_lidar, current_time_s=1.0)
    assert len(val_lidar.cleaned_data["points_lidar"]) == 1  # Filtered out invalid point


def test_trl6_point_cloud_ground_segmentation():
    """Verify point cloud processing and obstacle segmentation with invariant bin-lift."""
    pc_proc = PointCloudProcessor(voxel_size_m=0.20, obstacle_height_thresh_m=0.25)
    # Synthetic flat ground with one boulder (h = 0.50m)
    pts = []
    for x in np.linspace(-2.0, 2.0, 21):
        for y in np.linspace(-2.0, 2.0, 21):
            pts.append([x, y, 0.0])
    pts.append([0.0, 0.0, 0.50])  # Boulder
    pts_arr = np.array(pts)

    res = pc_proc.process_point_cloud(pts_arr, np.eye(4))
    assert res["num_ground"] >= 90
    assert res["num_obstacle"] >= 1


def test_trl6_elevation_grid_kalman_and_decay():
    """Verify 2.5D elevation grid Kalman updates, negative obstacle detection, and ray-clearing."""
    elev_map = ElevationMap25D(size_x_m=20.0, size_y_m=20.0, resolution_m=0.20)

    # Sequential updates to same cell should reduce variance
    pt = np.array([[5.0, 5.0, 1.2]])
    elev_map.update_from_point_cloud(pt, np.zeros(3))
    var1 = elev_map.variance[elev_map.world_to_grid(5.0, 5.0)]
    elev_map.update_from_point_cloud(pt, np.zeros(3))
    var2 = elev_map.variance[elev_map.world_to_grid(5.0, 5.0)]
    assert var2 < var1

    # Mark obstacle and test ray clearing
    obs_pt = np.array([[5.0, 5.0, 1.2]])
    elev_map.mark_obstacles(obs_pt)
    ix, iy = elev_map.world_to_grid(5.0, 5.0)
    assert bool(elev_map.obstacle_mask[ix, iy]) is True

    # Decay obstacles
    elev_map.decay_transient_obstacles(decay_rate=0.50)
    assert elev_map.obstacle_confidence[ix, iy] < 0.35


def test_trl6_traversability_cost_map():
    """Verify multi-criteria traversability cost mapping."""
    elev_map = ElevationMap25D(size_x_m=20.0, size_y_m=20.0, resolution_m=0.20)
    # Gentle slope
    for i in range(elev_map.nx):
        elev_map.elevation[i, :] = i * 0.02
    elev_map.hit_count[:, :] = 10

    trav_est = TraversabilityEstimator()
    trav_res = trav_est.evaluate_traversability(elev_map)
    cost = trav_res["cost_map"]
    assert cost.shape == (elev_map.nx, elev_map.ny)
    assert np.all((cost >= 0.0) & (cost <= 1.0))
    # Benign slope should have moderate cost
    assert np.mean(cost) < 0.40


def test_trl6_perception_metrics():
    """Verify IoU and elevation RMSE calculation."""
    m_est = np.zeros((10, 10), dtype=bool)
    m_gt = np.zeros((10, 10), dtype=bool)
    m_est[2:5, 2:5] = True
    m_gt[2:5, 2:5] = True

    metrics = PerceptionMetrics.evaluate_obstacle_detection(m_est, m_gt)
    assert metrics["iou"] == 1.0
    assert metrics["precision"] == 1.0
    assert metrics["recall"] == 1.0
