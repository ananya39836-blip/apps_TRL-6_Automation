"""Bekker-Wong Soil Terramechanics & Drawbar Pull Engine (TRL 6).

Implements:
1. Bekker pressure-sinkage relationship: p = (k_c / b + k_phi) * z^n
2. Janosi-Hanamoto dynamic shear stress: tau(j) = (c + sigma * tan(phi)) * (1 - exp(-j / K))
3. Compaction motion resistance: R_c
4. Gross traction force and net Drawbar Pull (DP)
"""
from __future__ import annotations
import math
from typing import Dict, Any
import numpy as np


class SoilTerramechanics:
    """Computes wheel-regolith terramechanical interaction forces and sinkage."""

    def __init__(
        self,
        cohesion_pa: float = 800.0,            # Martian regolith cohesion c [Pa]
        internal_friction_deg: float = 34.0,     # Internal friction angle phi [deg]
        cohesion_modulus_kc: float = 1400.0,    # Bekker cohesive modulus k_c [N/m^(n+1)]
        frictional_modulus_kphi: float = 820000.0, # Bekker frictional modulus k_phi [N/m^(n+2)]
        sinkage_exponent_n: float = 1.0,         # Sinkage exponent n
        shear_deformation_mod_k: float = 0.025   # Janosi shear deformation modulus K [m]
    ) -> None:
        self.c = float(cohesion_pa)
        self.phi_deg = float(internal_friction_deg)
        self.phi_rad = math.radians(self.phi_deg)
        self.kc = float(cohesion_modulus_kc)
        self.kphi = float(frictional_modulus_kphi)
        self.n = float(sinkage_exponent_n)
        self.K = float(shear_deformation_mod_k)

    def compute_wheel_sinkage_and_resistance(
        self,
        normal_load_n: float,
        wheel_radius_m: float = 0.25,
        wheel_width_m: float = 0.40
    ) -> Dict[str, float]:
        """Calculates wheel sinkage depth z0 and compaction motion resistance Rc."""
        W = max(10.0, float(normal_load_n))
        r = float(wheel_radius_m)
        b = float(wheel_width_m)

        # Bekker soil stiffness factor k_eq = (kc / b + kphi)
        k_eq = (self.kc / b) + self.kphi

        # Contact patch length approximation: l_c = sqrt(2 * r * z - z^2) approx sqrt(2 * r * z)
        # Numerical iterative solve for static sinkage z0
        # W = b * k_eq * (z0^(n+1) / (n + 1)) * (3 - n) ... or standard Bekker plate:
        # p_avg = W / (b * sqrt(2 * r * z0)) = k_eq * z0^n
        # => z0^(n + 0.5) = W / (b * sqrt(2 * r) * k_eq)
        exp_term = self.n + 0.5
        denom = b * math.sqrt(2.0 * r) * k_eq
        z0 = (W / max(1.0, denom)) ** (1.0 / exp_term)
        z0 = min(z0, r * 0.40)  # Physical geometric limit

        # Compaction motion resistance force Rc [N]
        # Rc = (b * k_eq / (n + 1)) * z0^(n + 1)
        Rc = (b * k_eq / (self.n + 1.0)) * (z0 ** (self.n + 1.0))

        return {
            "sinkage_depth_m": round(float(z0), 4),
            "sinkage_depth_cm": round(float(z0 * 100.0), 2),
            "compaction_resistance_n": round(float(Rc), 2),
        }

    def compute_traction_and_drawbar_pull(
        self,
        normal_load_n: float,
        slip_ratio: float,
        wheel_radius_m: float = 0.25,
        wheel_width_m: float = 0.40
    ) -> Dict[str, float]:
        """Calculates gross traction force H, motion resistance Rc, and net drawbar pull DP."""
        W = max(10.0, float(normal_load_n))
        s = np.clip(float(slip_ratio), 0.0, 0.90)

        # 1. Compaction sinkage & resistance
        sink_res = self.compute_wheel_sinkage_and_resistance(W, wheel_radius_m, wheel_width_m)
        Rc = sink_res["compaction_resistance_n"]
        z0 = sink_res["sinkage_depth_m"]

        # 2. Contact patch area: A = b * l_c
        l_c = math.sqrt(max(0.001, 2.0 * wheel_radius_m * z0))
        A = wheel_width_m * l_c

        # 3. Janosi-Hanamoto Shear Stress: tau = (c + sigma * tan(phi)) * (1 - exp(-j / K))
        sigma = W / max(0.01, A)
        shear_displacement_j = s * l_c
        tau_max = self.c + sigma * math.tan(self.phi_rad)
        tau = tau_max * (1.0 - math.exp(-shear_displacement_j / self.K))

        # 4. Gross Traction Force: H = A * tau
        H = A * tau

        # 5. Net Drawbar Pull: DP = H - Rc
        DP = H - Rc

        return {
            "gross_traction_n": round(float(H), 2),
            "compaction_resistance_n": round(float(Rc), 2),
            "drawbar_pull_n": round(float(DP), 2),
            "shear_stress_pa": round(float(tau), 2),
            "sinkage_depth_m": round(float(z0), 4),
            "has_positive_pull": bool(DP > 0.0),
        }
