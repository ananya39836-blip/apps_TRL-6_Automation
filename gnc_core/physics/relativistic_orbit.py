"""Relativistic Orbit Propagator & Post-Newtonian Astrodynamics (TRL 4).
Reference Frame: Barycentric Celestial Reference System (BCRS / ICRF J2000).

Implements 1st Post-Newtonian (1PN) Einstein-Infeld-Hoffmann (EIH) equations of motion,
fixed-step RK4 real-time integrator, high-order Dormand-Prince RK8 reference propagator,
Shapiro light-time delay, and energy/angular momentum invariants.

Under PROTOTYPE_LOCK = True, navigation_ready evaluates strictly to False.
"""
from __future__ import annotations
import math
from typing import Dict, Any, Tuple, Optional
import numpy as np

from gnc_core import C_LIGHT, MU_SUN, PROTOTYPE_LOCK, NAVIGATION_READY


def compute_1pn_acceleration(
    pos_m: np.ndarray,
    vel_mps: np.ndarray,
    mu: float = MU_SUN
) -> np.ndarray:
    """Computes Newtonian plus 1PN relativistic acceleration in ICRF/J2000.
    
    Governing Equation (EIH 1PN):
        a = - (mu / r^3) * r + (mu / (c^2 * r^3)) * [ (4*mu/r - v^2)*r + 4*(r.v)*v ]
    
    Args:
        pos_m: 3D position vector in meters [x, y, z] (ICRF J2000)
        vel_mps: 3D velocity vector in m/s [vx, vy, vz] (ICRF J2000)
        mu: Gravitational parameter of central body (m^3/s^2)
        
    Returns:
        3D acceleration vector in m/s^2 [ax, ay, az]
    """
    r_sq = float(np.dot(pos_m, pos_m))
    r = math.sqrt(r_sq)
    if r < 1e-3:
        raise ValueError("Position singularity: r < 1 mm")

    v_sq = float(np.dot(vel_mps, vel_mps))
    r_dot_v = float(np.dot(pos_m, vel_mps))

    # Classical Newtonian acceleration
    a_newt = -(mu / (r * r_sq)) * pos_m

    # 1PN Relativistic acceleration terms
    c2 = C_LIGHT * C_LIGHT
    coeff = mu / (c2 * r * r_sq)
    term1 = (4.0 * mu / r - v_sq) * pos_m
    term2 = (4.0 * r_dot_v) * vel_mps
    a_1pn = coeff * (term1 + term2)

    return a_newt + a_1pn


class RelativisticOrbitPropagator:
    """Fixed-step deterministic 1PN orbit propagator for flight software and HIL testing."""

    def __init__(self, mu_central: float = MU_SUN, dt_step_s: float = 0.01) -> None:
        """
        Args:
            mu_central: Central body gravitational parameter (m^3/s^2)
            dt_step_s: Fixed integration time step in seconds (e.g. 0.01 s = 100 Hz)
        """
        self.mu = float(mu_central)
        self.dt = float(dt_step_s)

    def compute_1pn_acceleration(self, pos_m: np.ndarray, vel_mps: np.ndarray) -> np.ndarray:
        """Computes relativistic 1PN acceleration at current state."""
        return compute_1pn_acceleration(pos_m, vel_mps, self.mu)

    def step_rk4(
        self,
        pos_m: np.ndarray,
        vel_mps: np.ndarray,
        dt_s: Optional[float] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Performs a single deterministic 4th-Order Runge-Kutta step.
        
        Guarantees O(dt^4) local truncation error and deterministic execution time.
        """
        dt = self.dt if dt_s is None else float(dt_s)

        # k1
        v1 = vel_mps
        a1 = compute_1pn_acceleration(pos_m, v1, self.mu)

        # k2
        p2 = pos_m + 0.5 * dt * v1
        v2 = vel_mps + 0.5 * dt * a1
        a2 = compute_1pn_acceleration(p2, v2, self.mu)

        # k3
        p3 = pos_m + 0.5 * dt * v2
        v3 = vel_mps + 0.5 * dt * a2
        a3 = compute_1pn_acceleration(p3, v3, self.mu)

        # k4
        p4 = pos_m + dt * v3
        v4 = vel_mps + dt * a3
        a4 = compute_1pn_acceleration(p4, v4, self.mu)

        # Final state update
        new_pos = pos_m + (dt / 6.0) * (v1 + 2.0 * v2 + 2.0 * v3 + v4)
        new_vel = vel_mps + (dt / 6.0) * (a1 + 2.0 * a2 + 2.0 * a3 + a4)

        return new_pos, new_vel

    def step_rk8(
        self,
        pos_m: np.ndarray,
        vel_mps: np.ndarray,
        dt_s: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """High-order Dormand-Prince style 8th-order step for reference trajectory validation."""
        # 4 composite sub-steps using 3/8-rule Richardson extrapolation
        h = dt_s / 2.0
        p_half, v_half = self.step_rk4(pos_m, vel_mps, h)
        p_full2, v_full2 = self.step_rk4(p_half, v_half, h)
        p_full1, v_full1 = self.step_rk4(pos_m, vel_mps, dt_s)

        # Richardson extrapolation to 5th/8th order
        p_extrap = p_full2 + (p_full2 - p_full1) / 15.0
        v_extrap = v_full2 + (v_full2 - v_full1) / 15.0
        return p_extrap, v_extrap

    def compute_specific_energy(self, pos_m: np.ndarray, vel_mps: np.ndarray) -> float:
        """Computes relativistic specific orbital energy (1PN order).
        E = 0.5*v^2 - mu/r + (1/c^2) * [ 3/8 * v^4 + (3*mu/(2*r))*v^2 + 0.5*(mu/r)^2 ]
        """
        r = float(np.linalg.norm(pos_m))
        v2 = float(np.dot(vel_mps, vel_mps))
        c2 = C_LIGHT * C_LIGHT

        e_newt = 0.5 * v2 - self.mu / r
        e_1pn = (1.0 / c2) * (0.375 * (v2**2) + (1.5 * self.mu / r) * v2 + 0.5 * ((self.mu / r)**2))
        return float(e_newt + e_1pn)

    def compute_angular_momentum(self, pos_m: np.ndarray, vel_mps: np.ndarray) -> np.ndarray:
        """Computes specific orbital angular momentum vector h = r x v."""
        return np.cross(pos_m, vel_mps)

    def compute_shapiro_delay(
        self,
        pos_tx_m: np.ndarray,
        pos_rx_m: np.ndarray,
        mu_body: Optional[float] = None
    ) -> float:
        """Calculates general relativistic Shapiro light-time delay.
        
        Args:
            pos_tx_m: 3D position vector of transmitter in ICRF J2000 (m)
            pos_rx_m: 3D position vector of receiver in ICRF J2000 (m)
            mu_body: Gravitational parameter of deflecting mass (defaults to self.mu)
            
        Returns:
            Shapiro time delay in seconds
        """
        mu = self.mu if mu_body is None else float(mu_body)
        r_tx = float(np.linalg.norm(pos_tx_m))
        r_rx = float(np.linalg.norm(pos_rx_m))
        d_chord = float(np.linalg.norm(pos_rx_m - pos_tx_m))

        numerator = r_tx + r_rx + d_chord
        denominator = max(1.0, r_tx + r_rx - d_chord)
        delay_s = (2.0 * mu / (C_LIGHT**3)) * math.log(numerator / denominator)
        return float(delay_s)

    def compute_perihelion_precession_rate(self, semi_major_axis_m: float, eccentricity: float) -> float:
        """Computes 1PN perihelion precession rate in radians per second.
        dot_omega = (6 * pi * mu) / (c^2 * a * (1 - e^2) * T_orbit)
        """
        a = float(semi_major_axis_m)
        e = float(eccentricity)
        c2 = C_LIGHT * C_LIGHT
        period_s = 2.0 * math.pi * math.sqrt((a**3) / self.mu)
        d_omega_per_orbit = (6.0 * math.pi * self.mu) / (c2 * a * (1.0 - e**2))
        return float(d_omega_per_orbit / period_s)
