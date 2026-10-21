"""CLI Runner for TRL 9 Operational Qualification Campaign.

Executes all 20 standardized scenarios (SC-01 to SC-20), generates
Wilson score 95% Confidence Intervals, and writes qualification evidence files.
"""
from __future__ import annotations
import sys
import os

# Ensure project root is in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from trl9_autonomy.experiments.qualification_runner import QualificationCampaignRunner

def main():
    runner = QualificationCampaignRunner(output_dir="results/trl9")
    summary = runner.run_all_scenarios(base_seed=100)
    print("\n=== Qualification Campaign Complete ===")
    print(f"Total Scenarios: {summary['total_scenarios']}")
    print(f"Success Rate: {summary['success_rate']*100:.1f}%")
    print(f"Wilson 95% CI: [{summary['wilson_ci_95']['lower']*100:.1f}%, {summary['wilson_ci_95']['upper']*100:.1f}%]")
    print(f"Mean ATE RMSE: {summary['mean_ate_rmse_m']:.3f} m")
    print(f"Mean Loop Latency: {summary['mean_cycle_latency_ms']:.2f} ms")
    print(f"Collisions: {summary['total_collisions']}")

if __name__ == "__main__":
    main()
