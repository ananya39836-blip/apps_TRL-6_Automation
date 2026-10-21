"""Procedural Continuous 3D Planetary Terrain & Multi-Hazard Environment Model.

Generates continuous 3D digital elevation models (DEM), regional gradients,
harmonic surface undulations, impact craters with raised rims, boulders, and
directional solar ray-marching shadow masks.
"""
from __future__ import annotations
import math
from typing import Dict, List, Tuple, Any
import numpy as np


class PlanetaryTerrain:
    """Continuous 3D planetary digital elevation model with hazard injection."""

    def __init__(
        self,
        size_x_m: float = 40.0,
        size_y_m: float = 40.0,
        resolution_m: float = 0.20,
        base_slope_deg: float = 0.0,
        undulation_amplitude_m: float = 0.35,
        undulation_wavelength_m: float = 12.0,
    ) -> None:
        self.size_x = size_x_m
        self.size_y = size_y_m
        self.res = resolution_m
        self.nx = int(round(size_x_m / resolution_m))
        self.ny = int(round(size_y_m / resolution_m))

        self.base_slope_deg = base_slope_deg
        self.amplitude = undulation_amplitude_m
        self.wavelength = undulation_wavelength_m

        # Coordinate axes
        self.x_coords = np.linspace(0.0, self.size_x, self.nx)
        self.y_coords = np.linspace(0.0, self.size_y, self.ny)
        self.X, self.Y = np.meshgrid(self.x_coords, self.y_coords, indexing="ij")

        # Elevation grid and hazard masks
        self.elevation = np.zeros((self.nx, self.ny), dtype=np.float64)
        self.rock_mask = np.zeros((self.nx, self.ny), dtype=bool)
        self.crater_mask = np.zeros((self.nx, self.ny), dtype=bool)
        self.regolith_friction = np.ones((self.nx, self.ny), dtype=np.float64) * 0.85

        self.boulders: List[Dict[str, float]] = []
        self.craters: List[Dict[str, float]] = []

        self._generate_base_topography()

    def _generate_base_topography(self) -> None:
        """Generates continuous harmonic undulations and regional base slope."""
        slope_rad = math.radians(self.base_slope_deg)
        z_slope = self.X * math.tan(slope_rad)

        k1 = 2.0 * math.pi / self.wavelength
        k2 = 2.0 * math.pi / (self.wavelength * 0.45)
        z_undulation = (
            self.amplitude * np.sin(k1 * self.X) * np.cos(k1 * self.Y)
            + 0.35 * self.amplitude * np.cos(k2 * self.X + 0.5) * np.sin(k2 * self.Y - 0.2)
        )

        self.elevation = z_slope + z_undulation

    def get_elevation_at(self, x: float, y: float) -> float:
        """Computes continuous elevation Z at world position (x, y) via bilinear interpolation."""
        gx = x / self.res
        gy = y / self.res

        ix = int(math.floor(gx))
        iy = int(math.floor(gy))

        ix = np.clip(ix, 0, self.nx - 2)
        iy = np.clip(iy, 0, self.ny - 2)

        u = np.clip(gx - ix, 0.0, 1.0)
        v = np.clip(gy - iy, 0.0, 1.0)

        z00 = self.elevation[ix, iy]
        z10 = self.elevation[ix + 1, iy]
        z01 = self.elevation[ix, iy + 1]
        z11 = self.elevation[ix + 1, iy + 1]

        z_interp = (1.0 - u) * (1.0 - v) * z00 + u * (1.0 - v) * z10 + (1.0 - u) * v * z01 + u * v * z11
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
                    dz = h * math.sqrt(max(0.0, 1.0 - (d / r) ** 2))
                    self.elevation[i, j] += dz
                    if dz >= 0.15:
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
                    dz = -d * (1.0 - (dist / r) ** 2)
                    self.elevation[i, j] += dz
                    if abs(dz) >= 0.15:
                        self.crater_mask[i, j] = True
                elif dist <= rim_radius:
                    rim_t = (dist - r) / (rim_radius - r)
                    dz = rim_height * math.sin(rim_t * math.pi)
                    self.elevation[i, j] += dz

    def compute_solar_shadows(self, sun_elev_deg: float = 30.0, sun_azim_deg: float = 45.0) -> np.ndarray:
        """Computes cast shadows across the terrain via directional ray-marching."""
        shadow_mask = np.zeros((self.nx, self.ny), dtype=bool)
        tan_elev = math.tan(math.radians(max(5.0, sun_elev_deg)))
        azim_rad = math.radians(sun_azim_deg)
        dx = -math.cos(azim_rad) * self.res
        dy = -math.sin(azim_rad) * self.res

        for i in range(self.nx):
            for j in range(self.ny):
                cur_x = self.x_coords[i]
                cur_y = self.y_coords[j]
                cur_z = self.elevation[i, j]

                ray_x, ray_y = cur_x, cur_y
                ray_dist = 0.0

                for _ in range(25):
                    ray_x += dx
                    ray_y += dy
                    ray_dist += self.res
                    if not (0.0 <= ray_x < self.size_x and 0.0 <= ray_y < self.size_y):
                        break
                    terrain_z = self.get_elevation_at(ray_x, ray_y)
                    sun_ray_z = cur_z + ray_dist * tan_elev
                    if terrain_z > sun_ray_z:
                        shadow_mask[i, j] = True
                        break

        return shadow_mask
