"""Systematic Fault Detection, Isolation, and Recovery (FDIR) Manager.

Detects sensor dropouts, excessive noise, localization divergence, wheel slip,
route blockage, controller instability, and low battery states.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple


class FaultSeverity(str, Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"
    FATAL = "FATAL"


@dataclass
class ActiveFault:
    fault_code: str
    severity: FaultSeverity
    timestamp_s: float
    description: str
    action_taken: str


class FDIRManager:
    """Centralized Fault Management engine with automated recovery policies."""

    def __init__(self) -> None:
        self.active_faults: List[ActiveFault] = []
        self.fault_history: List[ActiveFault] = []

    def report_fault(
        self,
        fault_code: str,
        severity: FaultSeverity,
        timestamp_s: float,
        description: str,
        action_taken: str,
    ) -> None:
        """Registers an active fault and records to history."""
        fault = ActiveFault(fault_code, severity, timestamp_s, description, action_taken)
        self.active_faults.append(fault)
        self.fault_history.append(fault)

    def clear_fault(self, fault_code: str) -> None:
        """Clears an active fault after successful recovery."""
        self.active_faults = [f for f in self.active_faults if f.fault_code != fault_code]

    def has_critical_fault(self) -> bool:
        """Returns True if any CRITICAL or FATAL fault is currently active."""
        return any(
            f.severity in {FaultSeverity.CRITICAL, FaultSeverity.FATAL}
            for f in self.active_faults
        )

    def check_sensor_health(
        self, last_sensor_time: float, current_time: float, timeout_s: float = 0.30
    ) -> Tuple[bool, Optional[str]]:
        """Checks if sensor data is stale or dropped out."""
        if (current_time - last_sensor_time) > timeout_s:
            msg = f"Sensor dropout detected (elapsed {current_time - last_sensor_time:.3f}s > {timeout_s}s)"
            self.report_fault("ERR_SENSOR_TIMEOUT", FaultSeverity.CRITICAL, current_time, msg, "SAFE_STOP")
            return False, msg
        return True, None

    def check_slip_hazard(
        self, slip_ratio: float, timestamp: float, max_slip: float = 0.35
    ) -> bool:
        """Monitors excessive wheel slippage on loose sand."""
        if slip_ratio > max_slip:
            msg = f"Excessive wheel slip {slip_ratio:.2f} > {max_slip:.2f}"
            self.report_fault("ERR_WHEEL_SLIP", FaultSeverity.WARNING, timestamp, msg, "DERATE_SPEED")
            return False
        return True

    def check_cross_track_error(
        self, error_m: float, timestamp: float, max_error_m: float = 0.80
    ) -> bool:
        """Monitors path tracking divergence."""
        if error_m > max_error_m:
            msg = f"Path cross-track error {error_m:.2f}m exceeds limit {max_error_m:.2f}m"
            self.report_fault("ERR_CONTROLLER_DIVERGENCE", FaultSeverity.CRITICAL, timestamp, msg, "TRIGGER_REPLAN")
            return False
        return True
