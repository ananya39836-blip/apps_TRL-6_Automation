"""Standardized 20-Scenario Environment Generator (TRL 6).

Generates reproducible scenario configurations mapping to formal Operational Design
Domain (ODD) boundaries, sensor noise stress tests, and dynamic hazard injections.
"""
from __future__ import annotations
import math
from typing import Dict, Any, Tuple
import numpy as np

from trl6_navigation.environment.planetary_terrain import PlanetaryTerrain


class ScenarioGenerator:
    """Configures reproducible environments for the 20 standardized TRL 6 benchmark scenarios."""

    @staticmethod
    def build_scenario(scenario_id: str, seed: int = 100) -> Tuple[PlanetaryTerrain, Dict[str, Any]]:
        """Instantiates terrain geometry, hazard placements, and metadata for scenario_id."""
        rng = np.random.default_rng(seed)

        # Base configuration parameters
        size_x = 60.0
        size_y = 60.0
        res = 0.20
        base_slope_deg = 0.0
        undulation_amp = 0.4
        undulation_wave = 15.0

        start_state = np.array([5.0, 5.0, math.pi / 4.0, 0.0])  # [x, y, yaw, speed]
        goal_pos = np.array([25.0, 25.0])                       # Default 28.3 m traverse

        sensor_noise_mult = 1.0
        lidar_dropout_prob = 0.0
        wheel_slip_ratio = 0.05
        sun_elevation_deg = 45.0
        sun_azimuth_deg = 45.0
        pop_up_obstacle = False
        completely_blocked = False
        critical_fault = False
        high_latency = False
        transient_glitch = False

        if scenario_id == "SC-01_flat_baseline":
            undulation_amp = 0.1
            base_slope_deg = 0.0

        elif scenario_id == "SC-02_moderate_slopes":
            base_slope_deg = 8.0
            undulation_amp = 0.8
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for _ in range(10):
                rx = float(rng.uniform(10.0, 40.0))
                ry = float(rng.uniform(10.0, 40.0))
                terrain.add_boulder(rx, ry, radius_m=float(rng.uniform(0.4, 0.7)), height_m=float(rng.uniform(0.3, 0.5)))
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            return terrain, meta

        elif scenario_id == "SC-03_steep_slopes":
            base_slope_deg = 16.0
            undulation_amp = 1.2
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for _ in range(6):
                rx = float(rng.uniform(12.0, 38.0))
                ry = float(rng.uniform(12.0, 38.0))
                terrain.add_boulder(rx, ry, radius_m=0.5, height_m=0.35)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            return terrain, meta

        elif scenario_id == "SC-04_dense_obstacle_field":
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for i in range(18):
                rx = 10.0 + (i % 6) * 4.5 + float(rng.uniform(-1.0, 1.0))
                ry = 10.0 + (i // 6) * 6.0 + float(rng.uniform(-1.0, 1.0))
                terrain.add_boulder(rx, ry, radius_m=float(rng.uniform(0.5, 0.8)), height_m=float(rng.uniform(0.35, 0.6)))
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            return terrain, meta

        elif scenario_id == "SC-05_narrow_passages":
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for y_off in [12.0, 15.0, 18.0]:
                terrain.add_boulder(16.0, y_off, radius_m=0.9, height_m=0.7)
                terrain.add_boulder(20.5, y_off, radius_m=0.9, height_m=0.7)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            return terrain, meta

        elif scenario_id == "SC-06_crater_avoidance":
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            terrain.add_crater(15.0, 15.0, radius_m=3.5, depth_m=0.8)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            return terrain, meta

        elif scenario_id == "SC-07_sensor_noise":
            sensor_noise_mult = 3.0
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for _ in range(12):
                rx = float(rng.uniform(10.0, 35.0))
                ry = float(rng.uniform(10.0, 35.0))
                terrain.add_boulder(rx, ry, radius_m=0.6, height_m=0.4)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            return terrain, meta

        elif scenario_id == "SC-08_sensor_dropout":
            lidar_dropout_prob = 0.35
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for _ in range(8):
                rx = float(rng.uniform(10.0, 35.0))
                ry = float(rng.uniform(10.0, 35.0))
                terrain.add_boulder(rx, ry, radius_m=0.6, height_m=0.4)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            return terrain, meta

        elif scenario_id == "SC-09_localization_drift":
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for _ in range(12):
                rx = float(rng.uniform(10.0, 35.0))
                ry = float(rng.uniform(10.0, 35.0))
                terrain.add_boulder(rx, ry, radius_m=0.6, height_m=0.4)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult=1.5, lidar_dropout_prob=0.0, wheel_slip_ratio=0.15)
            meta["imu_drift_active"] = True
            return terrain, meta

        elif scenario_id == "SC-10_wheel_slip_disturbance":
            wheel_slip_ratio = 0.45
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            terrain.set_regolith_friction_patch(12.0, 28.0, 12.0, 28.0, friction_mu=0.35)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["slip_disturbance_active"] = True
            return terrain, meta

        elif scenario_id == "SC-11_unexpected_obstacles":
            pop_up_obstacle = True
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for _ in range(8):
                rx = float(rng.uniform(10.0, 35.0))
                ry = float(rng.uniform(10.0, 35.0))
                terrain.add_boulder(rx, ry, radius_m=0.5, height_m=0.4)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["pop_up_obstacle_active"] = pop_up_obstacle
            return terrain, meta

        elif scenario_id == "SC-12_completely_blocked":
            completely_blocked = True
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            for y_wall in np.arange(8.0, 32.0, 1.2):
                terrain.add_boulder(16.0, y_wall, radius_m=0.8, height_m=0.7)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["completely_blocked"] = completely_blocked
            return terrain, meta

        elif scenario_id == "SC-13_low_traction_regolith":
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            terrain.set_regolith_friction_patch(0.0, size_x, 0.0, size_y, friction_mu=0.40)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio=0.30)
            meta["low_traction_active"] = True
            return terrain, meta

        elif scenario_id == "SC-14_environmental_replan":
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg=6.0, undulation_amp=0.7, undulation_wavelength_m=15.0, seed=seed)
            sun_elevation_deg = 15.0
            sun_azimuth_deg = 60.0
            for i in range(12):
                rx = 12.0 + (i % 4) * 5.0 + float(rng.uniform(-0.8, 0.8))
                ry = 12.0 + (i // 4) * 6.0 + float(rng.uniform(-0.8, 0.8))
                terrain.add_boulder(rx, ry, radius_m=0.7, height_m=0.55)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["sun_elevation_deg"] = sun_elevation_deg
            meta["sun_azimuth_deg"] = sun_azimuth_deg
            return terrain, meta

        elif scenario_id == "SC-15_critical_fault_safestop":
            critical_fault = True
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["critical_fault_active"] = critical_fault
            return terrain, meta

        # Additional TRL 6 Scenarios (SC-16 through SC-20)
        elif scenario_id == "SC-16_extended_horizon_traverse":
            # Extended 45-meter traverse across multi-zone terrain
            goal_pos = np.array([45.0, 45.0])
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg=4.0, undulation_amplitude_m=0.5, undulation_wavelength_m=18.0, seed=seed)
            for _ in range(16):
                rx = float(rng.uniform(10.0, 42.0))
                ry = float(rng.uniform(10.0, 42.0))
                terrain.add_boulder(rx, ry, radius_m=float(rng.uniform(0.4, 0.7)), height_m=float(rng.uniform(0.3, 0.5)))
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["extended_horizon"] = True
            return terrain, meta

        elif scenario_id == "SC-17_unreachable_goal_recovery":
            # Goal deliberately positioned within an impassable ring of boulders
            goal_pos = np.array([24.0, 24.0])
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg=0.0, undulation_amplitude_m=0.3, undulation_wavelength_m=15.0, seed=seed)
            # Enclosing boulder circle around goal
            for th in np.linspace(0.0, 2 * math.pi, 10, endpoint=False):
                bx = 24.0 + 1.8 * math.cos(th)
                by = 24.0 + 1.8 * math.sin(th)
                terrain.add_boulder(bx, by, radius_m=0.7, height_m=0.6)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["unreachable_goal"] = True
            return terrain, meta

        elif scenario_id == "SC-18_communication_latency_jitter":
            high_latency = True
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg=3.0, undulation_amplitude_m=0.4, undulation_wavelength_m=15.0, seed=seed)
            for _ in range(10):
                rx = float(rng.uniform(10.0, 35.0))
                ry = float(rng.uniform(10.0, 35.0))
                terrain.add_boulder(rx, ry, radius_m=0.55, height_m=0.4)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult=1.5, lidar_dropout_prob=0.1, wheel_slip_ratio=0.1)
            meta["high_latency_jitter"] = high_latency
            return terrain, meta

        elif scenario_id == "SC-19_transient_sensor_glitch":
            transient_glitch = True
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg=2.0, undulation_amplitude_m=0.4, undulation_wavelength_m=15.0, seed=seed)
            for _ in range(8):
                rx = float(rng.uniform(10.0, 35.0))
                ry = float(rng.uniform(10.0, 35.0))
                terrain.add_boulder(rx, ry, radius_m=0.5, height_m=0.35)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult=1.0, lidar_dropout_prob=0.0, wheel_slip_ratio=0.05)
            meta["transient_sensor_glitch"] = transient_glitch
            return terrain, meta

        elif scenario_id == "SC-20_combined_multihazard_stress":
            # Multi-hazard: steep slopes + boulder field + loose regolith + sensor noise
            base_slope_deg = 10.0
            undulation_amp = 0.9
            sensor_noise_mult = 2.0
            lidar_dropout_prob = 0.20
            wheel_slip_ratio = 0.35
            terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
            terrain.set_regolith_friction_patch(10.0, 35.0, 10.0, 35.0, friction_mu=0.45)
            for _ in range(14):
                rx = float(rng.uniform(10.0, 35.0))
                ry = float(rng.uniform(10.0, 35.0))
                terrain.add_boulder(rx, ry, radius_m=float(rng.uniform(0.45, 0.75)), height_m=float(rng.uniform(0.35, 0.55)))
            terrain.add_crater(20.0, 20.0, radius_m=2.5, depth_m=0.6)
            meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
            meta["combined_stress_active"] = True
            return terrain, meta

        # Default fallback
        terrain = PlanetaryTerrain(size_x, size_y, res, base_slope_deg, undulation_amp, undulation_wave, seed)
        meta = ScenarioGenerator._build_meta(start_state, goal_pos, sensor_noise_mult, lidar_dropout_prob, wheel_slip_ratio)
        return terrain, meta

    @staticmethod
    def _build_meta(
        start_state: np.ndarray,
        goal_pos: np.ndarray,
        sensor_noise_mult: float,
        lidar_dropout_prob: float,
        wheel_slip_ratio: float,
    ) -> Dict[str, Any]:
        """Constructs standardized metadata dictionary for closed-loop execution."""
        return {
            "start_state": start_state.copy(),
            "goal_pos": goal_pos.copy(),
            "sensor_noise_factor": float(sensor_noise_mult),
            "lidar_dropout_prob": float(lidar_dropout_prob),
            "wheel_slip_ratio": float(wheel_slip_ratio),
            "pop_up_obstacle_active": False,
            "completely_blocked": False,
            "critical_fault_active": False,
            "slip_disturbance_active": False,
            "low_traction_active": False,
            "imu_drift_active": False,
            "extended_horizon": False,
            "unreachable_goal": False,
            "high_latency_jitter": False,
            "transient_sensor_glitch": False,
            "combined_stress_active": False,
        }
