"""Perception Performance Evaluation Metrics.

Computes IoU, Precision, Recall, F1-Score, and Elevation RMSE against ground-truth DEM.
"""
from __future__ import annotations
import math
from typing import Dict, Any
import numpy as np


class PerceptionMetrics:
    """Calculates ground truth verification metrics for perception outputs."""

    @staticmethod
    def evaluate_obstacle_detection(
        pred_mask: np.ndarray, gt_mask: np.ndarray
    ) -> Dict[str, float]:
        """Computes Precision, Recall, F1, and Intersection-over-Union (IoU)."""
        tp = int(np.sum(pred_mask & gt_mask))
        fp = int(np.sum(pred_mask & ~gt_mask))
        fn = int(np.sum(~pred_mask & gt_mask))

        precision = tp / (tp + fp) if (tp + fp) > 0 else 1.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 1.0
        f1 = (
            2.0 * precision * recall / (precision + recall)
            if (precision + recall) > 0
            else 0.0
        )
        iou = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 1.0

        return {
            "true_positives": float(tp),
            "false_positives": float(fp),
            "false_negatives": float(fn),
            "precision": float(precision),
            "recall": float(recall),
            "f1_score": float(f1),
            "iou": float(iou),
        }

    @staticmethod
    def evaluate_elevation_reconstruction(
        pred_elev: np.ndarray, gt_elev: np.ndarray, observed_mask: np.ndarray
    ) -> Dict[str, float]:
        """Computes elevation Root Mean Square Error (RMSE) on observed cells."""
        if not np.any(observed_mask):
            return {"rmse_m": 0.0, "max_error_m": 0.0}

        diff = pred_elev[observed_mask] - gt_elev[observed_mask]
        rmse = float(np.sqrt(np.mean(diff ** 2)))
        max_err = float(np.max(np.abs(diff)))

        return {
            "rmse_m": rmse,
            "max_error_m": max_err,
        }
