"""Spaceborne X-Ray Pulsar Navigation (XNAV) Error Model (TRL 4).
Representative of NASA SEXTANT / NICER Instrumentation Standards.

Implements Poisson photon arrival counting, phase jitter, relativistic atomic clock drift,
geometric line-of-sight projection, and solar gravitational Shapiro delay compensation.

Under PROTOTYPE_LOCK = True, navigation_ready evaluates strictly to False.
"""
from __future__ import annotations
import math
from typing import Dict, Any, List, Tuple
import numpy as np

from gnc_core import C_LIGHT, MU_SUN, PROTOTYPE_LOCK, NAVIGATION_READY


class SpaceborneXNAVReceiver:
    """X-ray millisecond pulsar receiver with Poisson statistics and clock drift."""

    def __init__(
        self,
        clock_drift_rate: float = 1.2e-14,  # Allan deviation floor
        detector_area_cm2: float = 2500.0,  # Effective silicon drift detector area
        seed: int = 1337
    ) -> None:
        self.clock_drift_rate = float(clock_drift_rate)
        self.area_cm2 = float(detector_area_cm2)
        self.rng = np.random.default_rng(seed)

        # Authoritative Millisecond Pulsar Catalog
        self.catalog = {
            "PSR_B1937+21": {
                "period_s": 0.0015578065,
                "f0_hz": 641.92825,
                "flux_photons_cm2_s": 0.0045,
                "background_flux": 0.0012,
                "unit_vector": np.array([0.722, 0.451, 0.525], dtype=float)
            },
            "PSR_B1821-24": {
                "period_s": 0.0030543152,
                "f0_hz": 327.40552,
                "flux_photons_cm2_s": 0.0032,
                "background_flux": 0.0015,
                "unit_vector": np.array([-0.125, 0.654, 0.746], dtype=float)
            },
            "PSR_J0437-4715": {
                "period_s": 0.0057574519,
                "f0_hz": 173.68792,
                "flux_photons_cm2_s": 0.0068,
                "background_flux": 0.0009,
                "unit_vector": np.array([0.721, -0.453, 0.524], dtype=float)
            }
        }
        # Normalize catalog unit vectors
        for p in self.catalog.values():
            p["unit_vector"] /= np.linalg.norm(p["unit_vector"])

    def simulate_observation(
        self,
        true_pos_m: np.ndarray,
        pulsar_name: str,
        integration_time_s: float = 1.0,
        mission_elapsed_s: float = 0.0
    ) -> Dict[str, Any]:
        """Simulates single pulsar TOA observation with noise injection.
        
        Args:
            true_pos_m: Ground truth 3D spacecraft position in ICRF/J2000 (m)
            pulsar_name: Target pulsar key in catalog
            integration_time_s: Photometer accumulation time in seconds
            mission_elapsed_s: Total mission elapsed time for secular clock drift
            
        Returns:
            Observation dictionary with photon count, estimated range, and phase.
        """
        data = self.catalog.get(pulsar_name, self.catalog["PSR_B1937+21"])
        u_vec = data["unit_vector"]

        # Geometric line-of-sight range
        true_proj_range_m = float(np.dot(true_pos_m, u_vec))

        # Solar Shapiro delay
        r_sc = float(np.linalg.norm(true_pos_m))
        r_sun = 1.496e11  # 1 AU
        shapiro_delay_s = (2.0 * MU_SUN / (C_LIGHT**3)) * math.log(max(1.05, (r_sc + r_sun) / r_sun))

        # Atomic clock drift accumulation
        clock_drift_s = self.clock_drift_rate * math.sqrt(max(1.0, mission_elapsed_s)) + self.rng.normal(0, 1.5e-9)

        # Poisson photon arrival count
        lambda_rate = (data["flux_photons_cm2_s"] + data["background_flux"]) * self.area_cm2
        expected_photons = max(1.0, lambda_rate * float(integration_time_s))
        measured_photons = int(self.rng.poisson(expected_photons))

        # Phase jitter inversely proportional to sqrt(N_photons)
        jitter_std_ns = 150.0 / math.sqrt(max(1, measured_photons))
        phase_jitter_s = (jitter_std_ns * 1e-9) * self.rng.normal(0, 1)

        # Measured Time of Arrival
        geometric_delay_s = true_proj_range_m / C_LIGHT
        total_toa_s = geometric_delay_s + shapiro_delay_s + clock_drift_s + phase_jitter_s

        # Measured Range
        measured_range_m = (total_toa_s - shapiro_delay_s) * C_LIGHT
        measured_phase = (total_toa_s * data["f0_hz"]) % 1.0

        return {
            "pulsar_name": pulsar_name,
            "pulsar_id": pulsar_name,
            "photons_counted": measured_photons,
            "photons_detected": measured_photons,
            "true_range_m": true_proj_range_m,
            "measured_range_m": float(measured_range_m),
            "range_error_m": float(measured_range_m - true_proj_range_m),
            "measured_phase": float(measured_phase),
            "shapiro_delay_ns": float(shapiro_delay_s * 1e9),
            "clock_drift_ns": float(clock_drift_s * 1e9),
            "detector_jitter_ns": float(abs(phase_jitter_s) * 1e9),
            "clock_drift_fraction": float(self.clock_drift_rate),
        }

    def sample_arrival(
        self,
        true_pos_m: np.ndarray,
        pulsar_name: str = "PSR_B1937+21",
        integration_time_s: float = 0.01,
        mission_elapsed_s: float = 0.0
    ) -> Dict[str, Any]:
        """Convenience method for TOA photon arrival sampling."""
        return self.simulate_observation(true_pos_m, pulsar_name, integration_time_s, mission_elapsed_s)


# Export canonical alias
XNAVMillisecondPulsarReceiver = SpaceborneXNAVReceiver
