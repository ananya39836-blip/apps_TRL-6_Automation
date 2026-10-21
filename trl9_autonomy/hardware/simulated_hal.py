"""Simulated Hardware Abstraction Layer (HAL) with Discrete Multi-Rate Timing.

Simulates 100 Hz IMU, 10 Hz LiDAR point clouds, 100 Hz wheel odometry,
and actuator drive commands with realistic space-grade sensor error models.
"""
from __future__ import annotations
import math
from typing import Optional, List
import numpy as np

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
from trl9_autonomy.sensors.coordinate_transforms import TransformSE3, SensorExtrinsics


class SimulatedHAL(ISensorHAL, IActuatorHAL):
    """Discrete multi-rate synthetic hardware abstraction layer."""

    def __init__(
        self,
        terrain,
        seed: int = 42,
        imu_rate_hz: float = 100.0,
        lidar_rate_hz: float = 10.0,
        accel_noise_density: float = 0.02,
        gyro_noise_density: float = 0.002,
        lidar_range_noise_m: float = 0.03,
    ) -> None:
        self.terrain = terrain
        self.rng = np.random.RandomState(seed)

        self.imu_dt = 1.0 / imu_rate_hz
        self.lidar_dt = 1.0 / lidar_rate_hz
        self.sim_time: float = 0.0

        self.accel_noise = accel_noise_density
        self.gyro_noise = gyro_noise_density
        self.lidar_noise_m = lidar_range_noise_m

        # Dynamic vehicle ground-truth state
        self.x: float = 2.0
        self.y: float = 2.0
        self.z: float = 0.0
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0

        self.vx: float = 0.0
        self.vy: float = 0.0
        self.vz: float = 0.0
        self.yaw_rate: float = 0.0

        # Actuator command buffers
        self.cmd_v: float = 0.0
        self.cmd_steer: float = 0.0
        self.e_stop_active: bool = False

        # Periodic timers
        self.last_imu_time: float = -1.0
        self.last_lidar_time: float = -1.0
        self.seq_imu: int = 0
        self.seq_lidar: int = 0
        self.seq_odom: int = 0

    def step_simulation(self, dt: float) -> None:
        """Advances physics clock and updates ground-truth vehicle pose."""
        self.sim_time += dt

        if self.e_stop_active:
            self.vx = 0.0
            self.yaw_rate = 0.0
            return

        # Simple vehicle kinematics
        L = 2.20
        self.yaw_rate = (self.vx / L) * math.tan(self.cmd_steer)
        self.yaw += self.yaw_rate * dt

        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += cos_y * self.vx * dt
        self.y += sin_y * self.vx * dt

        # Query terrain elevation
        if 0.0 <= self.x < self.terrain.size_x and 0.0 <= self.y < self.terrain.size_y:
            self.z = self.terrain.get_elevation_at(self.x, self.y)
        else:
            self.z = 0.0

        # Update forward velocity towards commanded velocity
        tau = 0.20
        self.vx += (self.cmd_v - self.vx) * (dt / tau)

    def read_imu(self) -> Optional[IMUPacket]:
        """Polls IMU at 100 Hz with Allan variance sensor noise."""
        if (self.sim_time - self.last_imu_time) < (self.imu_dt - 1e-6):
            return None

        self.last_imu_time = self.sim_time
        self.seq_imu = (self.seq_imu + 1) & 0x3FFF

        # Nominal specific force (balancing Mars gravity 3.72 m/s^2)
        ax = float(self.rng.normal(0.0, self.accel_noise))
        ay = float(self.rng.normal(0.0, self.accel_noise))
        az = 3.72076 + float(self.rng.normal(0.0, self.accel_noise))

        gx = float(self.rng.normal(0.0, self.gyro_noise))
        gy = float(self.rng.normal(0.0, self.gyro_noise))
        gz = self.yaw_rate + float(self.rng.normal(0.0, self.gyro_noise))

        payload = np.array([ax, ay, az, gx, gy, gz], dtype=np.float32).tobytes()
        crc = compute_crc16_ccitt(payload)

        header = PacketHeader(
            version=0,
            packet_type=0,
            sec_header_flag=1,
            apid=0x010,
            sequence_flags=3,
            sequence_count=self.seq_imu,
            packet_length=len(payload) + 16 - 1,
            timestamp_s=self.sim_time,
            crc16=crc,
        )

        return IMUPacket(
            timestamp_s=self.sim_time,
            accel_mps2=np.array([ax, ay, az], dtype=np.float64),
            gyro_rads=np.array([gx, gy, gz], dtype=np.float64),
            temperature_c=-25.0,
            header=header,
        )

    def read_lidar(self) -> Optional[LiDARScanPacket]:
        """Polls LiDAR point cloud at 10 Hz with exact 6-DOF SE(3) transformation."""
        if (self.sim_time - self.last_lidar_time) < (self.lidar_dt - 1e-6):
            return None

        self.last_lidar_time = self.sim_time
        self.seq_lidar = (self.seq_lidar + 1) & 0x3FFF

        # Direct sampling of terrain ahead of rover within FOV
        T_world_body = TransformSE3.from_xyz_rpy(self.x, self.y, self.z, self.roll, self.pitch, self.yaw)
        T_world_lidar = T_world_body.compose(SensorExtrinsics().T_body_lidar)
        T_lidar_world = T_world_lidar.inverse()

        ranges = np.linspace(1.0, 12.0, 10)
        az_angles = np.linspace(-math.radians(35.0), math.radians(35.0), 12)
        world_pts = []

        for az in az_angles:
            world_az = self.yaw + az
            c_waz = math.cos(world_az)
            s_waz = math.sin(world_az)
            for r in ranges:
                wx = self.x + r * c_waz
                wy = self.y + r * s_waz
                if 0.0 <= wx < self.terrain.size_x and 0.0 <= wy < self.terrain.size_y:
                    wz = self.terrain.get_elevation_at(wx, wy)
                    world_pts.append([wx, wy, wz])

        if world_pts:
            w_arr = np.array(world_pts, dtype=np.float64)
            pts_arr = T_lidar_world.transform_points(w_arr)
            pts_arr += self.rng.normal(0.0, self.lidar_noise_m, pts_arr.shape)
            ints_arr = np.ones(len(pts_arr), dtype=np.float64) * 0.85
        else:
            pts_arr = np.zeros((0, 3), dtype=np.float64)
            ints_arr = np.zeros(0, dtype=np.float64)

        header = PacketHeader(
            version=0,
            packet_type=0,
            sec_header_flag=1,
            apid=0x020,
            sequence_flags=3,
            sequence_count=self.seq_lidar,
            packet_length=len(pts_arr) * 12 + 16 - 1,
            timestamp_s=self.sim_time,
            crc16=0xAAAA,
        )

        return LiDARScanPacket(
            timestamp_s=self.sim_time,
            points_xyz=pts_arr,
            intensities=ints_arr,
            header=header,
        )

    def read_wheel_odometry(self) -> Optional[WheelOdometryPacket]:
        """Polls wheel encoders at 100 Hz."""
        self.seq_odom = (self.seq_odom + 1) & 0x3FFF
        v_enc = self.vx + float(self.rng.normal(0.0, 0.01))
        w_enc = self.yaw_rate + float(self.rng.normal(0.0, 0.005))

        header = PacketHeader(
            version=0,
            packet_type=0,
            sec_header_flag=1,
            apid=0x040,
            sequence_flags=3,
            sequence_count=self.seq_odom,
            packet_length=24 + 16 - 1,
            timestamp_s=self.sim_time,
            crc16=0xBBBB,
        )

        return WheelOdometryPacket(
            timestamp_s=self.sim_time,
            forward_velocity_mps=v_enc,
            yaw_rate_rads=w_enc,
            wheel_speeds_rads=np.ones(6) * (v_enc / 0.25),
            header=header,
        )

    def send_drive_command(self, cmd: ActuatorCommandPacket) -> bool:
        """Applies velocity and steering commands to rover dynamics."""
        if not cmd.crc_valid:
            return False
        if cmd.brake_engaged:
            self.e_stop_active = True
            self.cmd_v = 0.0
            return True

        self.cmd_v = float(np.clip(cmd.commanded_velocity_mps, -0.20, 0.40))
        self.cmd_steer = float(np.clip(cmd.commanded_steer_rad, -0.65, 0.65))
        return True

    def trigger_emergency_stop(self) -> bool:
        """Latches emergency stop."""
        self.e_stop_active = True
        self.cmd_v = 0.0
        return True
