"""Reproducible Operational Qualification Campaign Runner.

Executes 20 standardized scenarios (SC-01 to SC-20) under nominal and perturbed
Mars surface conditions, calculating Wilson score 95% Confidence Intervals.
"""
from __future__ import annotations
import os
import json
import csv
import math
import time
from typing import Dict, Any, List
import numpy as np

from trl9_autonomy.environment.scenario_generator import ScenarioGenerator
from trl9_autonomy.mission.mission_executive import AutonomousMissionExecutive


def compute_wilson_ci(k: int, n: int, confidence: float = 0.95) -> Dict[str, float]:
    """Computes Wilson score 95% confidence interval for proportion k / n."""
    if n == 0:
        return {"proportion": 0.0, "ci_lower": 0.0, "ci_upper": 0.0}

    z = 1.95996  # 95% two-sided normal quantile
    p_hat = k / n
    denom = 1.0 + (z**2) / n
    center = (p_hat + (z**2) / (2.0 * n)) / denom
    spread = (z * math.sqrt((p_hat * (1.0 - p_hat) / n) + ((z**2) / (4.0 * n**2)))) / denom

    return {
        "proportion": float(p_hat),
        "ci_lower": float(max(0.0, center - spread)),
        "ci_upper": float(min(1.0, center + spread)),
    }


class QualificationCampaignRunner:
    """Executes multi-scenario Monte-Carlo qualification benchmarks."""

    def __init__(self, output_dir: str = "results/trl9") -> None:
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.scenario_results: List[Dict[str, Any]] = []

    def run_all_scenarios(self, base_seed: int = 100) -> Dict[str, Any]:
        """Executes SC-01 through SC-20 sequentially."""
        print(f"=== Starting TRL 9 Operational Qualification Campaign (20 Scenarios) ===")
        t_start = time.time()

        for idx in range(1, 21):
            sc_id = f"SC-{idx:02d}"
            seed = base_seed + idx
            sc_data = ScenarioGenerator.generate_scenario(sc_id, seed=seed)

            executive = AutonomousMissionExecutive(
                terrain=sc_data["terrain"],
                start_pose=sc_data["start_pose"],
                goal_pose=sc_data["goal_pose"],
                seed=seed,
                oracle_mode=False,
            )

            report = executive.run_mission(max_duration_s=120.0)
            report["scenario_id"] = sc_id
            report["seed"] = seed
            self.scenario_results.append(report)

            status = "SUCCESS" if report["success"] else f"FAIL ({report['failure_reason']})"
            print(f"[{sc_id}] {status} | Dist: {report['distance_traversed_m']:.2f}m | ATE: {report['ate_rmse_m']:.3f}m | Lat: {report['mean_cycle_latency_ms']:.2f}ms")

        total_time = time.time() - t_start
        summary = self._generate_summary(total_time)
        self._export_results(summary)
        return summary

    def _generate_summary(self, elapsed_s: float) -> Dict[str, Any]:
        """Aggregates metrics and computes statistical confidence intervals."""
        n_total = len(self.scenario_results)
        n_success = sum(1 for r in self.scenario_results if r["success"])
        success_ci = compute_wilson_ci(n_success, n_total)

        distances = [r["distance_traversed_m"] for r in self.scenario_results]
        ate_errors = [r["ate_rmse_m"] for r in self.scenario_results]
        latencies = [r["mean_cycle_latency_ms"] for r in self.scenario_results]
        plan_latencies = [r["planning_latency_ms"] for r in self.scenario_results]
        collisions = sum(r["collision_count"] for r in self.scenario_results)

        return {
            "campaign_timestamp": time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime()),
            "total_scenarios": n_total,
            "total_successes": n_success,
            "success_rate": success_ci["proportion"],
            "wilson_ci_95": {
                "lower": success_ci["ci_lower"],
                "upper": success_ci["ci_upper"],
            },
            "total_collisions": collisions,
            "mean_traverse_distance_m": float(np.mean(distances)),
            "mean_ate_rmse_m": float(np.mean(ate_errors)),
            "max_ate_rmse_m": float(np.max(ate_errors)) if ate_errors else 0.0,
            "mean_cycle_latency_ms": float(np.mean(latencies)),
            "mean_planning_latency_ms": float(np.mean(plan_latencies)),
            "total_campaign_duration_s": float(elapsed_s),
            "software_version": "4.0.0-trl9",
        }

    def _export_results(self, summary: Dict[str, Any]) -> None:
        """Saves summary JSON and individual runs CSV."""
        # Summary JSON
        json_path = os.path.join(self.output_dir, "qualification_summary.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

        # Runs CSV
        csv_path = os.path.join(self.output_dir, "qualification_runs.csv")
        fieldnames = [
            "scenario_id",
            "seed",
            "success",
            "failure_reason",
            "duration_s",
            "distance_traversed_m",
            "replan_count",
            "collision_count",
            "ate_rmse_m",
            "drift_rate_pct",
            "mean_cycle_latency_ms",
            "planning_latency_ms",
        ]
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.scenario_results)

        print(f"Qualification campaign saved: {json_path} and {csv_path}")
