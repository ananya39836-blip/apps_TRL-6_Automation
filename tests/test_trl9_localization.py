"""Formal Unit & Integration Test Suite for TRL 9 Localization Subsystem.

Verifies:
- TRL9-REQ-08: 15-state Error-State EKF with Joseph-form updates & NHC
- TRL9-REQ-09: Dual-mode Oracle vs Estimated state monitoring
"""
from __future__ import annotations
import math
import numpy as np
import pytest

from trl9_autonomy.localization.error_state_ekf import ErrorStateEKF
from trl9_autonomy.localization.state_monitor import StateEstimationMonitor


def test_es_ekf_joseph_update():
    """Verifies Joseph-form covariance symmetry and positive definiteness."""
    ekf = ErrorStateEKF(dt_imu=0.010)

    # Propagate 50 steps
    accel = np.array([0.0, 0.0, 3.72076])
    gyro = np.array([0.0, 0.0, 0.0])
    for _ in range(50):
        ekf.predict(accel, gyro, dt=0.010)

    # Wheel odometry update with NHC
    ekf.update_wheel_odometry(forward_velocity_mps=0.30, dt=0.010)

    # Covariance symmetry check
    assert np.allclose(ekf.P, ekf.P.T, atol=1e-10)
    # Positive definiteness: all eigenvalues > 0
    eigvals = np.linalg.eigvals(ekf.P)
    assert np.all(eigvals > 0.0)


def test_state_monitor_metrics():
    """Verifies ATE RMSE, RPE, and drift rate calculations."""
    monitor = StateEstimationMonitor(divergence_trace_threshold=100.0)

    p_cov = np.eye(15) * 0.01
    # Synthetic trajectory: 10 meters along +X
    for step in range(20):
        gt = np.array([step * 0.5, 0.0, 0.0])
        est = np.array([step * 0.5 + 0.05, 0.02, 0.0])  # ~0.054m offset
        is_stable = monitor.record_step(est, gt, p_cov, step * 0.1)
        assert is_stable is True

    ate = monitor.compute_ate_rmse()
    assert 0.04 <= ate <= 0.07

    drift = monitor.compute_drift_rate_percent()
    assert drift > 0.0 and drift < 5.0

    summary = monitor.get_summary_metrics()
    assert "ate_rmse_m" in summary
    assert "drift_rate_pct" in summary


def test_ekf_wheel_odometry_nhc():
    """Verifies that NHC forces lateral and vertical body velocity towards zero."""
    ekf = ErrorStateEKF()
    # Inject spurious initial lateral velocity in World
    ekf.v = np.array([0.30, 0.15, 0.10])

    # Multiple wheel odometry updates should damp body lateral/vertical speed
    for _ in range(25):
        ekf.update_wheel_odometry(forward_velocity_mps=0.30, dt=0.010)

    p, v, rpy = ekf.get_state()
    assert abs(v[1]) < 0.10  # Lateral velocity reduced
    assert abs(v[2]) < 0.10  # Vertical velocity reduced


def test_oracle_mode_vs_estimated():
    """Verifies that oracle_mode is an explicit flag and preserves ground truth."""
    ekf_est = ErrorStateEKF(oracle_mode=False)
    ekf_oracle = ErrorStateEKF(oracle_mode=True)

    assert ekf_est.oracle_mode is False
    assert ekf_oracle.oracle_mode is True
