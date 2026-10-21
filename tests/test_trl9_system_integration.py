"""Formal Integration & Fault Management Test Suite for TRL 9.

Verifies:
- TRL9-REQ-03: CCSDS space packet framing & CCITT-16 checksum
- TRL9-REQ-16: Execution cycle watchdog deadline monitoring
- Phase 3: FDIR fault detection & recovery injection
- Subsystem K: Flight telemetry flight data logger export
"""
from __future__ import annotations
import os
import numpy as np
import pytest

from trl9_autonomy.hardware.hardware_interface import (
    PacketHeader,
    compute_crc16_ccitt,
)
from trl9_autonomy.fdir.watchdog import CycleExecutionWatchdog
from trl9_autonomy.fdir.fdir_manager import FDIRManager, FaultSeverity
from trl9_autonomy.telemetry.telemetry_logger import FlightTelemetryLogger


def test_ccsds_telemetry_packet_framing():
    """Verifies binary pack/unpack of CCSDS primary headers and CRC-16 computation."""
    payload = b"MARS_ROVER_TELEMETRY_DATA_BLOCK_001"
    crc = compute_crc16_ccitt(payload)

    header = PacketHeader(
        version=0,
        packet_type=0,
        sec_header_flag=1,
        apid=0x010,
        sequence_flags=3,
        sequence_count=42,
        packet_length=len(payload) + 16 - 1,
        timestamp_s=123.456,
        crc16=crc,
    )

    buf = header.pack()
    assert len(buf) == 16

    unpacked = PacketHeader.unpack(buf)
    assert unpacked.version == 0
    assert unpacked.packet_type == 0
    assert unpacked.apid == 0x010
    assert unpacked.sequence_count == 42
    assert unpacked.crc16 == crc


def test_watchdog_deadline_monitor():
    """Verifies watchdog deadline trips on consecutive cycle latency overruns."""
    watchdog = CycleExecutionWatchdog(max_cycle_latency_ms=100.0, max_consecutive_misses=3)

    # Nominal fast cycles
    is_ok, trigger_stop = watchdog.evaluate_cycle(35.0)
    assert is_ok is True
    assert trigger_stop is False

    # First miss (e.g. 110ms)
    is_ok, trigger_stop = watchdog.evaluate_cycle(110.0)
    assert is_ok is False
    assert trigger_stop is False

    # Second miss
    is_ok, trigger_stop = watchdog.evaluate_cycle(120.0)
    assert is_ok is False
    assert trigger_stop is False

    # Third consecutive miss triggers safe stop
    is_ok, trigger_stop = watchdog.evaluate_cycle(115.0)
    assert is_ok is False
    assert trigger_stop is True

    stats = watchdog.get_latency_percentiles()
    assert stats["miss_rate_pct"] > 50.0


def test_full_fdir_fault_injection():
    """Verifies systematic fault detection across sensor timeout, slip, and tracking divergence."""
    fdir = FDIRManager()
    assert fdir.has_critical_fault() is False

    # Ingest sensor timeout (0.5s > 0.3s)
    is_healthy, _ = fdir.check_sensor_health(last_sensor_time=1.0, current_time=1.6)
    assert is_healthy is False
    assert fdir.has_critical_fault() is True

    # Wheel slip hazard
    is_traction_ok = fdir.check_slip_hazard(slip_ratio=0.45, timestamp=1.6, max_slip=0.35)
    assert is_traction_ok is False

    # Path cross track divergence
    is_tracking_ok = fdir.check_cross_track_error(error_m=1.20, timestamp=1.6, max_error_m=0.80)
    assert is_tracking_ok is False


def test_flight_telemetry_logger(tmp_path):
    """Verifies flight recorder writes valid JSON and tabular CSV telemetry files."""
    log_dir = str(tmp_path)
    logger = FlightTelemetryLogger(log_dir=log_dir)

    for i in range(5):
        logger.log_telemetry_point(
            timestamp=i * 0.1,
            x=i * 0.2,
            y=0.0,
            z=0.1,
            yaw=0.05,
            velocity=0.35,
            state="EXECUTION",
            energy_wh=10.5 + i * 0.1,
        )

    json_f = logger.save_to_json("test_tlm.json")
    csv_f = logger.save_to_csv("test_tlm.csv")

    assert os.path.exists(json_f)
    assert os.path.exists(csv_f)
    assert os.path.getsize(json_f) > 100
    assert os.path.getsize(csv_f) > 50
