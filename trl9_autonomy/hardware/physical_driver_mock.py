"""Physical Breadboard Hardware Drivers & Socket Emulators.

Provides RS-422 Serial IMU, Ethernet UDP LiDAR (port 2368), CANopen Motor
Controller, and Hardware Watchdog deadman circuit emulators.
"""
from __future__ import annotations
import time
from typing import Optional, Dict, Any
import numpy as np


class SerialIMUDriver:
    """Emulates RS-422 / UART streaming binary IMU packets at 100 Hz."""

    def __init__(self, port: str = "COM3", baudrate: int = 460800) -> None:
        self.port = port
        self.baudrate = baudrate
        self.connected: bool = True
        self.packet_count: int = 0

    def read_packet(self) -> Dict[str, Any]:
        """Reads next binary packet from serial buffer."""
        self.packet_count += 1
        return {
            "timestamp": time.time(),
            "accel": [0.01, -0.02, 3.72],
            "gyro": [0.001, -0.001, 0.005],
            "seq": self.packet_count,
            "status": "VALID",
        }


class EthernetLiDARDriver:
    """Emulates Ethernet UDP streaming LiDAR packets (UDP port 2368)."""

    def __init__(self, ip: str = "192.168.1.201", port: int = 2368) -> None:
        self.ip = ip
        self.port = port
        self.connected: bool = True

    def poll_scan(self) -> Dict[str, Any]:
        """Polls latest 360-degree or forward-sector point cloud frame."""
        return {
            "timestamp": time.time(),
            "points_count": 384,
            "packet_status": "LIDAR_LOCK",
        }


class MotorControllerCAN:
    """Emulates CANopen (CiA 402) profile velocity motor controllers."""

    def __init__(self, can_bus_id: str = "can0", bitrate: int = 1000000) -> None:
        self.bus_id = can_bus_id
        self.bitrate = bitrate
        self.enabled: bool = True
        self.commanded_rpm: Dict[str, float] = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}

    def set_drive_velocity(self, v_mps: float, steer_rad: float) -> bool:
        """Translates vehicle linear velocity and steering into 6 individual wheel RPMs."""
        if not self.enabled:
            return False
        r_wheel = 0.25
        rpm = (v_mps / (2.0 * 3.14159 * r_wheel)) * 60.0
        for w in self.commanded_rpm:
            self.commanded_rpm[w] = rpm
        return True


class HardwareWatchdogEStop:
    """Hardware deadman watchdog requiring 100 Hz heartbeat pulse (150ms timeout)."""

    def __init__(self, timeout_s: float = 0.150) -> None:
        self.timeout_s = timeout_s
        self.last_heartbeat: float = time.time()
        self.estop_triggered: bool = False

    def kick(self) -> None:
        """Kicks hardware watchdog timer."""
        self.last_heartbeat = time.time()

    def check(self) -> bool:
        """Evaluates watchdog timer. If elapsed > timeout_s, drops hardware relay."""
        if (time.time() - self.last_heartbeat) > self.timeout_s:
            self.estop_triggered = True
            return False
        return True
