"""Hardware Abstraction Layer Package."""
from trl9_autonomy.hardware.hardware_interface import (
    ISensorHAL,
    IActuatorHAL,
    PacketHeader,
    IMUPacket,
    LiDARScanPacket,
    WheelOdometryPacket,
    ActuatorCommandPacket,
    compute_crc16_ccitt,
)
from trl9_autonomy.hardware.simulated_hal import SimulatedHAL
from trl9_autonomy.hardware.physical_driver_mock import (
    SerialIMUDriver,
    EthernetLiDARDriver,
    MotorControllerCAN,
    HardwareWatchdogEStop,
)

__all__ = [
    "ISensorHAL",
    "IActuatorHAL",
    "PacketHeader",
    "IMUPacket",
    "LiDARScanPacket",
    "WheelOdometryPacket",
    "ActuatorCommandPacket",
    "compute_crc16_ccitt",
    "SimulatedHAL",
    "SerialIMUDriver",
    "EthernetLiDARDriver",
    "MotorControllerCAN",
    "HardwareWatchdogEStop",
]
