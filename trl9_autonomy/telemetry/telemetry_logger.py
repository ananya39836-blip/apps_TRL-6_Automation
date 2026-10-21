"""Flight Data Recorder & Telemetry Logging Engine.

Records structured mission events, vehicle poses, sensor telemetry,
and state machine transitions to JSON and tabular CSV files for deterministic replay.
"""
from __future__ import annotations
import json
import csv
import os
from typing import Dict, Any, List


class FlightTelemetryLogger:
    """Manages high-rate telemetry recording and file persistence."""

    def __init__(self, log_dir: str = "results/trl9") -> None:
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.telemetry_records: List[Dict[str, Any]] = []

    def log_telemetry_point(
        self,
        timestamp: float,
        x: float,
        y: float,
        z: float,
        yaw: float,
        velocity: float,
        state: str,
        energy_wh: float,
    ) -> None:
        """Appends a synchronous telemetry sample."""
        self.telemetry_records.append({
            "timestamp_s": round(timestamp, 3),
            "x_m": round(x, 4),
            "y_m": round(y, 4),
            "z_m": round(z, 4),
            "yaw_rad": round(yaw, 4),
            "velocity_mps": round(velocity, 4),
            "operating_state": state,
            "energy_wh": round(energy_wh, 4),
        })

    def save_to_json(self, filename: str = "mission_telemetry.json") -> str:
        """Writes telemetry log to JSON file."""
        filepath = os.path.join(self.log_dir, filename)
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(self.telemetry_records, f, indent=2)
        return filepath

    def save_to_csv(self, filename: str = "mission_telemetry.csv") -> str:
        """Writes telemetry log to CSV file."""
        if not self.telemetry_records:
            return ""
        filepath = os.path.join(self.log_dir, filename)
        keys = list(self.telemetry_records[0].keys())
        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.telemetry_records)
        return filepath
