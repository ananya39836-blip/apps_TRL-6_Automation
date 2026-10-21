"""Full 6-DOF Multibody Rover Dynamics Simulation Engine.

Integrates chassis rigid-body kinematics, rocker-bogie passive suspension,
terramechanics wheel-soil traction, and electrical power consumption.
"""
from __future__ import annotations
import math
from typing import Dict, Any, Tuple
import numpy as np

from trl9_autonomy.dynamics.rocker_bogie import RockerBogieKinematics
from trl9_autonomy.dynamics.terramechanics import TerramechanicsModel
from trl9_autonomy.dynamics.energy_model import RoverEnergyModel


class MultibodyRoverSimulation:
    """Simulates 6-DOF chassis motion, suspension articulation, and terramechanics."""

    def __init__(
        self,
        mass_kg: float = 850.0,
        dt_dyn: float = 0.010,
    ) -> None:
        self.mass = mass_kg
        self.dt = dt_dyn

        self.suspension = RockerBogieKinematics(rover_mass_kg=mass_kg)
        self.soil = TerramechanicsModel()
        self.battery = RoverEnergyModel()

        # Rover 6-DOF pose in World ENU
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0

        # Body velocities
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.vz: float = 0.0
        self.yaw_rate: float = 0.0

        # Slip ratio
        self.slip_ratio: float = 0.05

    def step(
        self,
        cmd_velocity: float,
        cmd_steer: float,
        terrain_z_func,
        terrain_slope_func=None,
    ) -> Dict[str, Any]:
        """Steps dynamic simulation forward by dt."""
        # 1. Kinematic velocity response with first-order motor lag
        tau_accel = 0.15
        target_v = cmd_velocity * (1.0 - self.slip_ratio)
        self.vx += (target_v - self.vx) * (self.dt / tau_accel)

        # Yaw rate from bicycle model
        L = 2.20
        self.yaw_rate = (self.vx / L) * math.tan(cmd_steer)

        # 2. Integrate World horizontal position
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)

        self.x += (cos_y * self.vx - sin_y * self.vy) * self.dt
        self.y += (sin_y * self.vx + cos_y * self.vy) * self.dt
        self.yaw += self.yaw_rate * self.dt

        # 3. Ground contact and attitude from terrain
        z_ground = terrain_z_func(self.x, self.y)
        self.z = z_ground

        # Sample wheel contact heights for rocker-bogie
        wheel_zs = {}
        for w_name, offset in self.suspension.wheel_offsets.items():
            wx = self.x + cos_y * offset[0] - sin_y * offset[1]
            wy = self.y + sin_y * offset[0] + cos_y * offset[1]
            wheel_zs[w_name] = terrain_z_func(wx, wy)

        articulation = self.suspension.compute_articulation(wheel_zs)
        normal_loads = self.suspension.compute_normal_loads(self.roll, self.pitch)

        # Approximate chassis pitch and roll from suspension
        self.pitch = (articulation["rocker_left_rad"] + articulation["rocker_right_rad"]) * 0.5
        self.roll = articulation["differential_tilt_rad"]

        # 4. Terramechanics: sinkage and traction force
        avg_load = sum(normal_loads.values()) / 6.0
        sinkage = self.soil.compute_sinkage_m(avg_load)
        resistance = self.soil.compute_motion_resistance_n(sinkage)
        traction = max(0.0, self.mass * 0.20 + resistance)

        # 5. Energy consumption
        power_info = self.battery.step(self.vx, traction, self.dt)

        return {
            "pose": (self.x, self.y, self.z, self.roll, self.pitch, self.yaw),
            "velocity": (self.vx, self.vy, self.vz),
            "sinkage_m": sinkage,
            "normal_loads": normal_loads,
            "articulation": articulation,
            "power": power_info,
        }
