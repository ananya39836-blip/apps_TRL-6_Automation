"""3D Representative Planetary Terrain Environment (TRL 6).

Generates continuous 3D digital elevation models (DEM) with procedural undulations,
slopes, rocky obstacle fields, impact craters, variable regolith friction,
and directional solar shadow raycasting.
"""
from __future__ import annotations
import math
from typing import Dict, Any, List, Tuple, Optional
import numpy as np


class PlanetaryTerrain:
    """Configurable 3D planetary digital elevation model and geological hazard engine."""

    def __init__(
        self,
        size_x_m: float = 60.0,
        size_y_m: float = 60.0,
        resolution_m: float = 0.20,
        base_slope_deg: float = 0.0,
        undulation_amplitude_m: float = 0.5,
        undulation_wavelength_m: float = 15.0,
        seed: int = 42,
        **kwargs
    ) -> None:
        self.size_x = float(size_x_m)
        self.size_y = float(size_y_m)
        self.res = float(resolution_m)
        self.nx = int(round(self.size_x / self.res))
        self.ny = int(round(self.size_y / self.res))
        self.base_slope_deg = float(kwargs.get("base_slope", base_slope_deg))
        self.amplitude = float(kwargs.get("undulation_amp", undulation_amplitude_m))
        self.wavelength = float(kwargs.get("undulation_wave", undulation_wavelength_m))
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)

        # Coordinate axes
        self.x_coords = np.linspace(0.0, self.size_x, self.nx)
        self.y_coords = np.linspace(0.0, self.size_y, self.ny)
        self.X, self.Y = np.meshgrid(self.x_coords, self.y_coords, indexing="ij")

        # Elevation grid (Z) and hazard masks
        self.elevation = np.zeros((self.nx, self.ny), dtype=np.float64)
        self.rock_mask = np.zeros((self.nx, self.ny), dtype=bool)
        self.crater_mask = np.zeros((self.nx, self.ny), dtype=bool)
        self.regolith_friction = np.ones((self.nx, self.ny), dtype=np.float64) * 0.85  # Nominal friction mu

        # List of explicit obstacle geometries for ground-truth validation
        self.boulders: List[Dict[str, float]] = []
        self.craters: List[Dict[str, float]] = []

        self._generate_base_topography()

    def _generate_base_topography(self) -> None:
        """Generates continuous harmonic undulations and regional base slope."""
        # Regional slope gradient along +X axis
        slope_rad = math.radians(self.base_slope_deg)
        z_slope = self.X * math.tan(slope_rad)

        # Multi-harmonic surface undulation
        k1 = 2.0 * math.pi / self.wavelength
        k2 = 2.0 * math.pi / (self.wavelength * 0.45)
        z_undulation = (
            self.amplitude * np.sin(k1 * self.X) * np.cos(k1 * self.Y)
            + 0.35 * self.amplitude * np.cos(k2 * self.X + 0.5) * np.sin(k2 * self.Y - 0.2)
        )

        self.elevation = z_slope + z_undulation

    def get_elevation_at(self, x: float, y: float) -> float:
        """Computes continuous elevation Z at world position (x, y) via bilinear interpolation."""
        gx = (x / self.res)
        gy = (y / self.res)

        ix = int(math.floor(gx))
        iy = int(math.floor(gy))

        # Clamp boundary indices
        ix = np.clip(ix, 0, self.nx - 2)
        iy = np.clip(iy, 0, self.ny - 2)

        u = gx - ix
        v = gy - iy
        u = np.clip(u, 0.0, 1.0)
        v = np.clip(v, 0.0, 1.0)

        z00 = self.elevation[ix, iy]
        z10 = self.elevation[ix + 1, iy]
        z01 = self.elevation[ix, iy + 1]
        z11 = self.elevation[ix + 1, iy + 1]

        z_interp = (1 - u) * (1 - v) * z00 + u * (1 - v) * z10 + (1 - u) * v * z01 + u * v * z11
        return float(z_interp)

    def compute_slope_degrees(self) -> np.ndarray:
        """Computes local surface gradient slope in degrees using 2nd-order central differences."""
        gz, gx = np.gradient(self.elevation, self.res)
        slope_rad = np.arctan(np.sqrt(gx**2 + gz**2))
        return np.degrees(slope_rad)

    def add_boulder(self, x: float, y: float, radius_m: float, height_m: float) -> None:
        """Injects an explicit spherical-cap boulder into the elevation grid."""
        r = float(radius_m)
        h = float(height_m)
        self.boulders.append({"x": float(x), "y": float(y), "radius": r, "height": h})

        # Bounding box in grid coordinates
        ix_min = max(0, int(math.floor((x - r) / self.res)))
        ix_max = min(self.nx - 1, int(math.ceil((x + r) / self.res)))
        iy_min = max(0, int(math.floor((y - r) / self.res)))
        iy_max = min(self.ny - 1, int(math.ceil((y + r) / self.res)))

        for i in range(ix_min, ix_max + 1):
            for j in range(iy_min, iy_max + 1):
                px = self.x_coords[i]
                py = self.y_coords[j]
                d = math.hypot(px - x, py - y)
                if d <= r:
                    # Parabolic / spherical cap profile
                    dz = h * math.sqrt(max(0.0, 1.0 - (d / r) ** 2))
                    self.elevation[i, j] += dz
                    if dz >= 0.15:  # Lethal boulder threshold
                        self.rock_mask[i, j] = True

    def add_crater(self, x: float, y: float, radius_m: float, depth_m: float) -> None:
        """Injects a parabolic impact crater with a raised rim into the elevation grid."""
        r = float(radius_m)
        d = float(depth_m)
        self.craters.append({"x": float(x), "y": float(y), "radius": r, "depth": d})

        rim_radius = r * 1.25
        rim_height = d * 0.20

        ix_min = max(0, int(math.floor((x - rim_radius) / self.res)))
        ix_max = min(self.nx - 1, int(math.ceil((x + rim_radius) / self.res)))
        iy_min = max(0, int(math.floor((y - rim_radius) / self.res)))
        iy_max = min(self.ny - 1, int(math.ceil((y + rim_radius) / self.res)))

        for i in range(ix_min, ix_max + 1):
            for j in range(iy_min, iy_max + 1):
                px = self.x_coords[i]
                py = self.y_coords[j]
                dist = math.hypot(px - x, py - y)
                if dist <= r:
                    # Parabolic bowl depression
                    bowl_drop = d * (1.0 - (dist / r) ** 2)
                    self.elevation[i, j] -= bowl_drop
                    self.crater_mask[i, j] = True
                elif dist <= rim_radius:
                    # Raised rim decay
                    rim_factor = math.cos(math.pi * (dist - r) / (rim_radius - r) * 0.5) ** 2
                    self.elevation[i, j] += rim_height * rim_factor
                    self.crater_mask[i, j] = True

    def set_regolith_friction_patch(
        self, x_min: float, x_max: float, y_min: float, y_max: float, friction_mu: float
    ) -> None:
        """Sets localized regolith friction (e.g. low-traction loose sand or high-traction bedrock)."""
        ix_min = max(0, int(math.floor(x_min / self.res)))
        ix_max = min(self.nx - 1, int(math.ceil(x_max / self.res)))
        iy_min = max(0, int(math.floor(y_min / self.res)))
        iy_max = min(self.ny - 1, int(math.ceil(y_max / self.res)))

        self.regolith_friction[ix_min : ix_max + 1, iy_min : iy_max + 1] = float(friction_mu)

    def compute_solar_shadows(self, sun_elev_deg: float = 35.0, sun_azim_deg: float = 45.0) -> np.ndarray:
        """Computes directional solar ray-tracing shadow mask (True = shadowed cell)."""
        elev_rad = math.radians(sun_elev_deg)
        azim_rad = math.radians(sun_azim_deg)

        dx = math.cos(elev_rad) * math.cos(azim_rad)
        dy = math.cos(elev_rad) * math.sin(azim_rad)
        dz = math.sin(elev_rad)

        shadow_mask = np.zeros((self.nx, self.ny), dtype=bool)
        step_m = self.res * 0.8
        max_dist_m = 25.0

        for i in range(0, self.nx, 2):
            for j in range(0, self.ny, 2):
                x0 = self.x_coords[i]
                y0 = self.y_coords[j]
                z0 = self.elevation[i, j] + 0.05

                d = step_m
                while d <= max_dist_m:
                    rx = x0 + d * dx
                    ry = y0 + d * dy
                    rz = z0 + d * dz

                    if not (0.0 <= rx < self.size_x and 0.0 <= ry < self.size_y):
                        break

                    terr_z = self.get_elevation_at(rx, ry)
                    if terr_z > rz:
                        shadow_mask[i, j] = True
                        break
                    d += step_m

        return shadow_mask
