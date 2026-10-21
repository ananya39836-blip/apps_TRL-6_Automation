"""APPS TRL-9 Operational-System Readiness and Evidence Framework.

Technology Readiness Level 9 (TRL 9) Research & Qualification Baseline:
"Actual system flight proven through successful mission operations."

Boundary of Technology Scope & Academic Non-Affiliation Invariant:
This system represents an advanced software-in-the-loop and synthetic
laboratory autonomy prototype. NASA and international space agencies do not
automatically certify or qualify third-party software without physical flight
avionics integration, environmental TVAC testing, launch vibration qualification,
and flight heritage.

Safety Invariants:
    PROTOTYPE_LOCK = True: Permanently latched to prevent unauthorized physical actuator firing.
    NAVIGATION_READY = False: Latched to require explicit ground-station verification.
"""
from __future__ import annotations
from typing import Final, List, Dict, Any
from enum import Enum

# Permanent Research Safety Locks
PROTOTYPE_LOCK: Final[bool] = True
NAVIGATION_READY: Final[bool] = False

# Physical & Planetary Environmental Constants (Mars Surface Nominal)
G_MARS: Final[float] = 3.72076  # Surface gravitational acceleration (m/s^2)
ROVER_MASS_KG: Final[float] = 850.0  # Overall structural mass (kg)
ROVER_LENGTH_M: Final[float] = 2.80  # Structural chassis length (m)
ROVER_WIDTH_M: Final[float] = 2.20  # Structural chassis width (m)
ROVER_HEIGHT_M: Final[float] = 1.80  # Sensor mast zenith height (m)
WHEEL_RADIUS_M: Final[float] = 0.25  # Nominal wheel radius (m)
WHEEL_BASE_M: Final[float] = 2.20  # Longitudinal axle separation (m)
TRACK_WIDTH_M: Final[float] = 2.00  # Lateral axle separation (m)

# Motion Constraints & Kinodynamics
MAX_FORWARD_VELOCITY_MPS: Final[float] = 0.40  # Max cruising speed (m/s)
MAX_REVERSE_VELOCITY_MPS: Final[float] = 0.15  # Max recovery reverse speed (m/s)
MAX_CURVATURE_INV_M: Final[float] = 1.50  # Max curvature (m^-1), R_min = 0.67m
MAX_STEERING_ANGLE_RAD: Final[float] = 0.65  # Max wheel steering angle (~37.2 deg)
MAX_ACCELERATION_MPS2: Final[float] = 0.20  # Max linear acceleration (m/s^2)
MAX_LATERAL_ACCEL_MPS2: Final[float] = 0.30  # Max lateral tip-over acceleration (m/s^2)
SAFE_STOP_DECEL_MPS2: Final[float] = -0.20  # Certified constant-jerk deceleration (m/s^2)
SAFE_STOP_JERK_MPS3: Final[float] = -0.50  # Constant jerk rate (m/s^3)

# Terrain & Hazard Thresholds
MAX_TRAVERSABLE_SLOPE_DEG: Final[float] = 25.0  # Soft cost threshold (deg)
TIP_OVER_LIMIT_SLOPE_DEG: Final[float] = 30.0  # Hard geometric impassability (deg)
MAX_STEP_OBSTACLE_M: Final[float] = 0.20  # Max step height clearable by rocker-bogie (m)
MIN_OBSTACLE_CLEARANCE_M: Final[float] = 0.35  # Footprint safety inflation margin (m)

# 13 Formal Operating States
class OperatingState(str, Enum):
    STARTUP = "STARTUP"
    SELF_TEST = "SELF_TEST"
    IDLE = "IDLE"
    PERCEPTION = "PERCEPTION"
    LOCALIZATION = "LOCALIZATION"
    PLANNING = "PLANNING"
    EXECUTION = "EXECUTION"
    REPLANNING = "REPLANNING"
    SAFE_STOP = "SAFE_STOP"
    RECOVERY = "RECOVERY"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_ABORT = "MISSION_ABORT"
    FAULT = "FAULT"

# 20 Standardized Operational Scenarios
SCENARIO_IDS: Final[List[str]] = [
    f"SC-{i:02d}" for i in range(1, 21)
]

__version__: Final[str] = "4.0.0-trl9"
__author__: Final[str] = "APPS Multidisciplinary Planetary Robotics Engineering Team"
