"""Trajectory Tracking & State Estimation Performance Monitor.

Computes Absolute Trajectory Error (ATE), Relative Pose Error (RPE),
drift rate percentage, and covariance divergence alarms.
"""
from __future__ import annotations
import math
from typing import List, Tuple, Dict, Any
import numpy as np


class StateEstimationMonitor:
    """Monitors localization errors between estimated EKF state and oracle ground truth."""

    def __init__(self, divergence_trace_threshold: float = 100.0) -> None:
        self.divergence_thresh = divergence_trace_threshold
        self.est_positions: List[np.ndarray] = []
        self.gt_positions: List[np.ndarray] = []
        self.timestamps: List[float] = []
        self.covariance_traces: List[float] = []

    def record_step(
        self, est_pos: np.ndarray, gt_pos: np.ndarray, P_cov: np.ndarray, timestamp: float
    ) -> bool:
        """Records a trajectory state step. Returns False if covariance diverges."""
        self.est_positions.append(np.array(est_pos[:3], dtype=np.float64))
        self.gt_positions.append(np.array(gt_pos[:3], dtype=np.float64))
        self.timestamps.append(timestamp)

        trace_p = float(np.trace(P_cov[:3, :3]))
        self.covariance_traces.append(trace_p)

        return trace_p <= self.divergence_thresh

    def compute_ate_rmse(self) -> float:
        """Computes Absolute Trajectory Error (ATE) Root Mean Square Error (m)."""
        if len(self.est_positions) < 2:
            return 0.0

        est_arr = np.array(self.est_positions)
        gt_arr = np.array(self.gt_positions)
        errors = np.linalg.norm(est_arr - gt_arr, axis=1)
        return float(np.sqrt(np.mean(errors ** 2)))

    def compute_rpe(self, delta_steps: int = 10) -> float:
        """Computes Relative Pose Error (RPE) over specified delta step window (m)."""
        if len(self.est_positions) <= delta_steps:
            return 0.0

        est_arr = np.array(self.est_positions)
        gt_arr = np.array(self.gt_positions)

        d_est = est_arr[delta_steps:] - est_arr[:-delta_steps]
        d_gt = gt_arr[delta_steps:] - gt_arr[:-delta_steps]

        rpe_err = np.linalg.norm(d_est - d_gt, axis=1)
        return float(np.mean(rpe_err))

    def compute_drift_rate_percent(self) -> float:
        """Computes accumulated drift as percentage of total distance traveled."""
        if len(self.gt_positions) < 2:
            return 0.0

        gt_arr = np.array(self.gt_positions)
        diffs = np.diff(gt_arr, axis=0)
        total_dist = float(np.sum(np.linalg.norm(diffs, axis=1)))

        if total_dist < 0.1:
            return 0.0

        final_err = float(np.linalg.norm(self.est_positions[-1] - self.gt_positions[-1]))
        return float((final_err / total_dist) * 100.0)

    def get_summary_metrics(self) -> Dict[str, float]:
        """Returns comprehensive localization performance dictionary."""
        return {
            "ate_rmse_m": self.compute_ate_rmse(),
            "rpe_m": self.compute_rpe(),
            "drift_rate_pct": self.compute_drift_rate_percent(),
            "max_cov_trace": float(np.max(self.covariance_traces)) if self.covariance_traces else 0.0,
            "total_steps": float(len(self.est_positions)),
        }
