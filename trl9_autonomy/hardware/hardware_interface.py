"""Hardware Abstraction Layer (HAL) & Space-Grade CCSDS Packet Framing.

Implements ISensorHAL, IActuatorHAL, and CCSDS 133.0-B-2 primary header structures
with CCITT-16 checksums for breadboard and flight interface standardization.
"""
from __future__ import annotations
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Tuple
import numpy as np


def compute_crc16_ccitt(data: bytes) -> int:
    """Computes standard CCITT-16 polynomial 0x1021 CRC over byte payload."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class PacketHeader:
    """CCSDS 16-Byte Primary Space Packet Header."""
    version: int           # 3 bits (000 = Version 1)
    packet_type: int       # 1 bit (0=TLM, 1=CMD)
    sec_header_flag: int   # 1 bit (1 = Secondary header present)
    apid: int              # 11 bits (Application Process Identifier)
    sequence_flags: int    # 2 bits (11 = Unsegmented user data)
    sequence_count: int    # 14 bits (Cyclic packet counter)
    packet_length: int     # 16 bits (Total bytes in data field minus 1)
    timestamp_s: float     # 64 bits (SI mission elapsed seconds)
    crc16: int             # 16 bits (CCITT-16 checksum)

    def pack(self) -> bytes:
        """Serializes 16-byte binary header."""
        p_id = ((self.version & 0x7) << 13) | ((self.packet_type & 0x1) << 12) | ((self.sec_header_flag & 0x1) << 11) | (self.apid & 0x7FF)
        p_seq = ((self.sequence_flags & 0x3) << 14) | (self.sequence_count & 0x3FFF)
        return struct.pack(">HHHdH", p_id, p_seq, self.packet_length, self.timestamp_s, self.crc16 & 0xFFFF)

    @classmethod
    def unpack(cls, buffer: bytes) -> PacketHeader:
        """Deserializes 16-byte binary header."""
        p_id, p_seq, p_len, t_s, crc = struct.unpack(">HHHdH", buffer[:16])
        version = (p_id >> 13) & 0x7
        packet_type = (p_id >> 12) & 0x1
        sec_header = (p_id >> 11) & 0x1
        apid = p_id & 0x7FF
        seq_flags = (p_seq >> 14) & 0x3
        seq_count = p_seq & 0x3FFF
        return cls(version, packet_type, sec_header, apid, seq_flags, seq_count, p_len, t_s, crc)


@dataclass
class IMUPacket:
    timestamp_s: float
    accel_mps2: np.ndarray     # [ax, ay, az] in body frame
    gyro_rads: np.ndarray      # [gx, gy, gz] in body frame
    temperature_c: float
    header: PacketHeader


@dataclass
class LiDARScanPacket:
    timestamp_s: float
    points_xyz: np.ndarray     # N x 3 Cartesian points in LiDAR frame
    intensities: np.ndarray    # N x 1 reflectivity intensities
    header: PacketHeader


@dataclass
class WheelOdometryPacket:
    timestamp_s: float
    forward_velocity_mps: float
    yaw_rate_rads: float
    wheel_speeds_rads: np.ndarray  # 6-element wheel angular speeds
    header: PacketHeader


@dataclass
class ActuatorCommandPacket:
    timestamp_s: float
    commanded_velocity_mps: float
    commanded_steer_rad: float
    brake_engaged: bool
    crc_valid: bool


class ISensorHAL(ABC):
    """Abstract Hardware Abstraction Layer for Sensor Ingestion."""

    @abstractmethod
    def read_imu(self) -> Optional[IMUPacket]:
        """Polls latest IMU telemetry packet at 100 Hz."""
        pass

    @abstractmethod
    def read_lidar(self) -> Optional[LiDARScanPacket]:
        """Polls latest LiDAR point cloud packet at 10 Hz."""
        pass

    @abstractmethod
    def read_wheel_odometry(self) -> Optional[WheelOdometryPacket]:
        """Polls latest wheel encoder odometry packet at 100 Hz."""
        pass


class IActuatorHAL(ABC):
    """Abstract Hardware Abstraction Layer for Rover Actuation."""

    @abstractmethod
    def send_drive_command(self, cmd: ActuatorCommandPacket) -> bool:
        """Sends drive velocity and steering angles to wheel motor controllers."""
        pass

    @abstractmethod
    def trigger_emergency_stop(self) -> bool:
        """Asserts physical/simulated hardware deadman emergency brake."""
        pass
