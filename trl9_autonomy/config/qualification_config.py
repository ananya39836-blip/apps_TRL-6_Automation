"""APPS TRL 9 Configuration & Operational Parameter Specifications."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Any
import numpy as np

@dataclass(frozen=True)
class AutonomyConfig:
    """Immutable operational configuration with physical validity constraints."""
    # Timing & Execution Frequencies (Hz / ms)
    imu_rate_hz: float = 100.0
    navigation_rate_hz: float = 10.0
    max_cycle_latency_ms: float = 100.0
    planner_timeout_ms: float = 80.0
    
    # Rover Physical Dimensions (m / kg)
    rover_length_m: float = 2.80
    rover_width_m: float = 2.20
    rover_height_m: float = 1.80
    rover_mass_kg: float = 850.0
    wheel_radius_m: float = 0.25
    wheel_base_m: float = 2.20
    track_width_m: float = 2.00
    
    # Kinematic Limits
    max_velocity_mps: float = 0.40
    max_reverse_mps: float = 0.15
    max_curvature_inv_m: float = 1.50
    max_steer_rad: float = 0.65
    max_lat_accel_mps2: float = 0.30
    
    # Terrain & Hazard Thresholds
    max_slope_deg: float = 25.0
    tip_over_slope_deg: float = 30.0
    step_obstacle_m: float = 0.20
    safety_margin_m: float = 0.35
    
    # Mapping Resolution
    grid_resolution_m: float = 0.20
    local_map_size_m: float = 30.0
    
    # Power & Energy
    battery_capacity_wh: float = 1200.0  # Flight-representative Li-ion battery
    nominal_voltage_v: float = 28.0      # Avionics bus voltage
    base_avionics_power_w: float = 85.0   # Onboard compute + sensors idle power
    motor_efficiency: float = 0.75        # Wheel drive mechanical-electrical efficiency
    
    def validate(self) -> bool:
        """Validates all physical bounds and safety margins."""
        assert self.imu_rate_hz >= 50.0, "IMU rate must be >= 50 Hz"
        assert self.navigation_rate_hz >= 5.0, "Navigation rate must be >= 5 Hz"
        assert self.rover_mass_kg > 0.0, "Rover mass must be positive"
        assert self.max_velocity_mps <= 1.0, "Speed exceeds safe planetary rover limit"
        assert self.max_slope_deg < self.tip_over_slope_deg, "Slope thresholds inverted"
        assert self.battery_capacity_wh > 0.0, "Battery capacity must be positive"
        return True
