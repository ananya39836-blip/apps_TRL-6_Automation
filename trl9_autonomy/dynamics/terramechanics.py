"""Bekker-Wong Soil Sinkage & Janosi-Hanamoto Shear Terramechanics Model.

Computes wheel penetration sinkage and traction drawbar pull on Martian regolith.
"""
from __future__ import annotations
import math
from typing import Dict, Any
import numpy as np


class TerramechanicsModel:
    """Terramechanics model for Martian regolith wheel-soil interaction."""

    def __init__(
        self,
        cohesion_pa: float = 1200.0,       # Soil cohesion c (Pa)
        internal_friction_deg: float = 35.0, # Internal friction angle phi (deg)
        shear_modulus_k_m: float = 0.025,   # Shear deformation modulus K (m)
        kc_sinkage: float = 1400.0,         # Pressure-sinkage parameter k_c (N/m^(n+1))
        kphi_sinkage: float = 820000.0,     # Pressure-sinkage parameter k_phi (N/m^(n+2))
        sinkage_exponent_n: float = 1.0,    # Sinkage exponent n
        wheel_radius_m: float = 0.25,
        wheel_width_m: float = 0.20,
    ) -> None:
        self.c = cohesion_pa
        self.phi_rad = math.radians(internal_friction_deg)
        self.K = shear_modulus_k_m
        self.k_c = kc_sinkage
        self.k_phi = kphi_sinkage
        self.n = sinkage_exponent_n
        self.r = wheel_radius_m
        self.b = wheel_width_m

    def compute_sinkage_m(self, normal_load_n: float) -> float:
        """Computes wheel sinkage z_0 (m) using Bekker-Wong equation:

        z_0 = [ W / (b * (k_c/b + k_phi)) ] ^ (1/n)
        """
        if normal_load_n <= 0.0:
            return 0.0

        denom = self.b * (self.k_c / self.b + self.k_phi)
        if denom <= 0.0:
            return 0.0

        ratio = normal_load_n / denom
        z0 = math.pow(ratio, 1.0 / self.n)
        return float(np.clip(z0, 0.001, 0.15))

    def compute_shear_traction_pa(self, normal_stress_pa: float, slip_ratio: float) -> float:
        """Computes shear stress tau (Pa) via Janosi-Hanamoto formulation:

        tau(j) = (c + sigma * tan(phi)) * (1 - exp(-j / K))
        where j = slip_ratio * contact_length.
        """
        contact_length = math.sqrt(self.r * 0.05)
        j = max(0.0, abs(slip_ratio) * contact_length)

        max_shear = self.c + normal_stress_pa * math.tan(self.phi_rad)
        tau = max_shear * (1.0 - math.exp(-j / self.K))
        return float(tau)

    def compute_motion_resistance_n(self, sinkage_m: float) -> float:
        """Computes soil compaction motion resistance force R_c (N):

        R_c = b * (k_c/b + k_phi) * z_0^(n+1) / (n + 1)
        """
        if sinkage_m <= 0.0:
            return 0.0
        factor = self.b * (self.k_c / self.b + self.k_phi) / (self.n + 1.0)
        return float(factor * math.pow(sinkage_m, self.n + 1.0))
