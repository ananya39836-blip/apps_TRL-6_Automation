"""Planetary Rover Battery Energy Consumption & Power Budget Model.

Tracks battery State of Charge (SoC), drive motor mechanical-to-electrical
power draw, and avionics base loads across traverse profiles.
"""
from __future__ import annotations
import math
from typing import Dict, Any


class RoverEnergyModel:
    """Estimates energy consumption and monitors battery state of charge."""

    def __init__(
        self,
        battery_capacity_wh: float = 1200.0,
        nominal_voltage_v: float = 28.0,
        base_avionics_power_w: float = 85.0,
        motor_efficiency: float = 0.75,
        initial_soc_pct: float = 100.0,
    ) -> None:
        self.capacity_wh = battery_capacity_wh
        self.voltage = nominal_voltage_v
        self.p_avionics = base_avionics_power_w
        self.efficiency = motor_efficiency

        self.energy_remaining_wh = battery_capacity_wh * (initial_soc_pct / 100.0)
        self.total_energy_consumed_wh: float = 0.0

    @property
    def soc_percent(self) -> float:
        """Current battery state of charge (%)."""
        return max(0.0, min(100.0, (self.energy_remaining_wh / self.capacity_wh) * 100.0))

    def step(self, forward_velocity_mps: float, traction_force_n: float, dt: float) -> Dict[str, float]:
        """Advances energy simulation by dt seconds.

        Returns:
            Dictionary with instantaneous power (W) and accumulated Wh.
        """
        # Mechanical mechanical drive power: P_mech = F * v
        p_mech = max(0.0, abs(traction_force_n) * abs(forward_velocity_mps))
        p_drive_elec = p_mech / self.efficiency if self.efficiency > 0 else p_mech

        # Total instantaneous power draw (W)
        p_total = self.p_avionics + p_drive_elec

        # Energy consumed in this step (Watt-hours)
        d_wh = (p_total * dt) / 3600.0

        self.energy_remaining_wh = max(0.0, self.energy_remaining_wh - d_wh)
        self.total_energy_consumed_wh += d_wh

        return {
            "power_total_w": float(p_total),
            "power_drive_w": float(p_drive_elec),
            "power_avionics_w": float(self.p_avionics),
            "energy_consumed_wh": float(self.total_energy_consumed_wh),
            "soc_pct": float(self.soc_percent),
        }

    def is_low_battery_warning(self) -> bool:
        """Returns True if battery is below 20% SoC."""
        return self.soc_percent <= 20.0

    def is_critical_battery(self) -> bool:
        """Returns True if battery is below 10% SoC."""
        return self.soc_percent <= 10.0
