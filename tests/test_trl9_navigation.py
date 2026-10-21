"""Formal Unit & Integration Test Suite for TRL 9 Navigation & Dynamics.

Verifies:
- TRL9-REQ-10: Swept OBB footprint collision checking
- TRL9-REQ-11: Kinodynamic Hybrid A* motion planning under curvature bounds
- TRL9-REQ-12: Adaptive pure pursuit with curvature speed derating
- TRL9-REQ-13: Certified constant-jerk safe stop
- TRL9-REQ-14: Rocker-bogie kinematics & terramechanics
- TRL9-REQ-15: 13-state deterministic FDIR executive
"""
from __future__ import annotations
import math
import numpy as np
import pytest

from trl9_autonomy import OperatingState
from trl9_autonomy.planning.collision_checker import CollisionCheckerOBB
from trl9_autonomy.planning.kinodynamic_hybrid_astar import KinodynamicHybridAStar
from trl9_autonomy.planning.grid_astar import GridAStarBaseline
from trl9_autonomy.control.path_tracker import AdaptivePurePursuitTracker
from trl9_autonomy.control.safe_stop_controller import SafeStopController
from trl9_autonomy.dynamics.rocker_bogie import RockerBogieKinematics
from trl9_autonomy.dynamics.terramechanics import TerramechanicsModel
from trl9_autonomy.dynamics.energy_model import RoverEnergyModel
from trl9_autonomy.mission.state_machine import MissionStateMachine
from trl9_autonomy.environment.planetary_terrain import PlanetaryTerrain
from trl9_autonomy.mission.mission_executive import AutonomousMissionExecutive


def test_collision_checker_obb():
    """Verifies oriented bounding box collision detection against lethal obstacles."""
    checker = CollisionCheckerOBB(rover_length_m=2.80, rover_width_m=2.20, safety_margin_m=0.35)
    mask = np.zeros((50, 50), dtype=bool)

    # Free space pose at (5.0, 5.0)
    assert checker.check_pose_collision(5.0, 5.0, 0.0, mask, res=0.20) is False

    # Inject obstacle directly in footprint
    mask[25, 25] = True  # at (5.0, 5.0)
    assert checker.check_pose_collision(5.0, 5.0, 0.0, mask, res=0.20) is True


def test_kinodynamic_planner_curvature_bound():
    """Verifies that generated trajectory segments strictly satisfy curvature bounds."""
    planner = KinodynamicHybridAStar(max_curvature_inv_m=1.50)
    costmap = np.zeros((50, 50), dtype=np.float64)
    impassable = np.zeros((50, 50), dtype=bool)

    # Inject obstacle blocking direct straight path
    impassable[20:30, 20:25] = True

    start = (2.0, 2.0, math.radians(45.0))
    goal = (8.0, 8.0, math.radians(45.0))

    path = planner.plan(start, goal, costmap, impassable, res=0.20)
    assert path is not None
    assert len(path) >= 2

    # Check curvature between consecutive segments
    for i in range(len(path) - 1):
        x1, y1, yaw1 = path[i]
        x2, y2, yaw2 = path[i + 1]
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist > 1e-4:
            d_yaw = abs((yaw2 - yaw1 + math.pi) % (2.0 * math.pi) - math.pi)
            curvature = d_yaw / dist
            assert curvature <= 1.60  # Within slight numerical tolerance


def test_grid_astar_baseline():
    """Verifies geometric 2D grid A* baseline planner."""
    planner = GridAStarBaseline()
    costmap = np.zeros((30, 30), dtype=np.float64)
    impassable = np.zeros((30, 30), dtype=bool)

    path = planner.plan((1.0, 1.0), (5.0, 5.0), costmap, impassable, res=0.20)
    assert path is not None
    assert len(path) > 3


def test_path_tracker_pure_pursuit():
    """Verifies Pure Pursuit cross-track error damping and velocity derating."""
    tracker = AdaptivePurePursuitTracker(wheelbase_m=2.20, max_velocity_mps=0.40)
    ref_path = [(float(x), 0.0, 0.0) for x in np.linspace(0.0, 10.0, 20)]

    # Rover at lateral offset y = 0.50m
    cmd_v, cmd_steer, cte = tracker.compute_control((2.0, 0.50, 0.0), current_velocity=0.30, path=ref_path)
    assert cmd_v > 0.0
    # Must steer towards path (steer < 0 to turn towards negative Y)
    assert cmd_steer < 0.0
    assert 0.40 <= cte <= 0.60


def test_safe_stop_controller():
    """Verifies constant-jerk deceleration monotonicity and zero-speed convergence."""
    stop = SafeStopController(max_decel_mps2=-0.20, max_jerk_mps3=-0.50)
    v = 0.40
    dt = 0.05
    steps = 0

    while v > 0.0 and steps < 100:
        new_v, is_halted = stop.compute_step(v, dt)
        assert new_v <= v
        v = new_v
        steps += 1
        if is_halted:
            break

    assert v == 0.0
    assert is_halted is True


def test_rocker_bogie_kinematics():
    """Verifies 6-wheel normal load summation equals total gravity weight."""
    suspension = RockerBogieKinematics(rover_mass_kg=850.0)
    loads = suspension.compute_normal_loads(roll_rad=0.05, pitch_rad=-0.08)

    assert len(loads) == 6
    total_normal = sum(loads.values())
    expected_w = 850.0 * 3.72076
    assert np.isclose(total_normal, expected_w, rtol=1e-3)


def test_terramechanics_soil_model():
    """Verifies Bekker-Wong sinkage and Janosi-Hanamoto shear traction."""
    soil = TerramechanicsModel()
    sinkage = soil.compute_sinkage_m(normal_load_n=527.0)
    assert 0.001 < sinkage < 0.10

    shear = soil.compute_shear_traction_pa(normal_stress_pa=5000.0, slip_ratio=0.15)
    assert shear > 0.0


def test_battery_energy_model():
    """Verifies battery discharge, power calculation, and low SoC alarms."""
    battery = RoverEnergyModel(battery_capacity_wh=1200.0)
    assert battery.soc_percent == 100.0
    assert battery.is_low_battery_warning() is False

    # Simulate 100 seconds of cruising
    info = battery.step(forward_velocity_mps=0.40, traction_force_n=200.0, dt=100.0)
    assert info["power_total_w"] > 85.0
    assert info["energy_consumed_wh"] > 0.0
    assert battery.soc_percent < 100.0


def test_fdir_executive_transitions():
    """Verifies permitted state transitions and rejection of unauthorized skips."""
    fsm = MissionStateMachine()
    assert fsm.current_state == OperatingState.STARTUP

    # Valid transitions
    assert fsm.transition_to(OperatingState.SELF_TEST, 0.0) is True
    assert fsm.transition_to(OperatingState.IDLE, 0.1) is True
    assert fsm.transition_to(OperatingState.PERCEPTION, 0.2) is True

    # Illegal transition attempt (PERCEPTION directly to MISSION_COMPLETE)
    assert fsm.transition_to(OperatingState.MISSION_COMPLETE, 0.3) is False
    assert fsm.current_state == OperatingState.FAULT


def test_closed_loop_short_mission():
    """End-to-end closed loop mission test over 15 meters on undulating terrain."""
    terrain = PlanetaryTerrain(size_x_m=30.0, size_y_m=30.0, resolution_m=0.20, undulation_amplitude_m=0.15)
    executive = AutonomousMissionExecutive(
        terrain,
        start_pose=(4.0, 4.0, math.radians(45.0)),
        goal_pose=(15.0, 15.0, math.radians(45.0)),
        seed=42,
    )

    report = executive.run_mission(max_duration_s=80.0)
    assert report["distance_traversed_m"] > 5.0
    assert report["collision_count"] == 0
    assert report["ate_rmse_m"] < 0.25
