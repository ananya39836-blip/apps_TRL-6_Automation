"""Standardized 20-Scenario Operational Qualification Generator.

Generates reproducible test environments covering nominal regolith, dunes,
dense boulder fields, impact craters, steep slopes, low friction, sensor outages,
and dynamic rockfalls.
"""
from __future__ import annotations
import math
from typing import Dict, Any, Tuple
import numpy as np

from trl9_autonomy.environment.planetary_terrain import PlanetaryTerrain


class ScenarioGenerator:
    """Instantiates standardized operational scenarios SC-01 through SC-20."""

    @staticmethod
    def generate_scenario(scenario_id: str, seed: int = 100) -> Dict[str, Any]:
        """Builds terrain, hazards, initial pose, and goal for scenario_id."""
        rng = np.random.RandomState(seed)

        size_x = 40.0
        size_y = 40.0
        res = 0.20

        start_pose = (5.0, 5.0, math.radians(45.0))
        goal_pose = (22.0, 22.0, math.radians(45.0))

        # Base terrain undulations
        base_slope = 0.0
        amplitude = 0.20
        wavelength = 15.0

        if scenario_id in ["SC-01", "SC-02", "SC-03", "SC-04"]:
            # Nominal & Undulating Regolith
            amplitude = 0.15 + (int(scenario_id[-1]) - 1) * 0.05
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope, amplitude, wavelength)

        elif scenario_id in ["SC-05", "SC-06", "SC-07", "SC-08"]:
            # Geometric Hazards: Boulder Fields & Craters
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope, 0.20, wavelength)
            num_rocks = 8 + (int(scenario_id[-1]) - 5) * 4
            for _ in range(num_rocks):
                rx = rng.uniform(8.0, 25.0)
                ry = rng.uniform(8.0, 25.0)
                rr = rng.uniform(0.6, 1.2)
                rh = rng.uniform(0.3, 0.6)
                terrain.add_boulder(rx, ry, rr, rh)

            if scenario_id in ["SC-07", "SC-08"]:
                terrain.add_crater(14.0, 14.0, radius_m=2.5, depth_m=0.8)

        elif scenario_id in ["SC-09", "SC-10", "SC-11", "SC-12"]:
            # Steep Incline & Low Traction Regolith
            base_slope = 10.0 + (int(scenario_id[-2:]) - 9) * 2.5
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope, 0.15, wavelength)
            # Low friction sand drift
            terrain.regolith_friction[:, :] = 0.35 if scenario_id == "SC-12" else 0.50

        elif scenario_id in ["SC-13", "SC-14", "SC-15", "SC-16"]:
            # Sensor Disturbance & Outage Environments
            terrain = PlanetaryTerrain(size_x, size_y, res, 0.0, 0.25, wavelength)
            terrain.add_boulder(12.0, 12.0, 1.0, 0.5)
            terrain.add_boulder(18.0, 18.0, 1.2, 0.6)

        else:  # SC-17, SC-18, SC-19, SC-20
            # Dynamic Obstacles & Long Duration
            terrain = PlanetaryTerrain(size_x, size_y, res, 0.0, 0.20, wavelength)
            if scenario_id == "SC-17":
                # Rockfall directly in corridor at (12, 12)
                terrain.add_boulder(12.0, 12.0, radius_m=1.0, height_m=0.6)
            elif scenario_id == "SC-18":
                # Narrow canyon between two boulders
                terrain.add_boulder(12.0, 10.5, 1.2, 0.6)
                terrain.add_boulder(10.5, 12.0, 1.2, 0.6)
            elif scenario_id == "SC-20":
                # Extended traverse
                goal_pose = (32.0, 32.0, math.radians(45.0))

        return {
            "scenario_id": scenario_id,
            "terrain": terrain,
            "start_pose": start_pose,
            "goal_pose": goal_pose,
            "seed": seed,
        }
