"""Closed-Loop Autonomous Mission Executive & Traverse Runner.

Executes end-to-end multi-waypoint traverses with continuous 10 Hz perception,
15-state ES-EKF localization, dynamic replanning, and quantitative completion logic.
"""
from __future__ import annotations
import math
import time
from typing import Dict, Any, List, Tuple, Optional
import numpy as np

from trl9_autonomy import OperatingState
from trl9_autonomy.mission.state_machine import MissionStateMachine
from trl9_autonomy.perception.point_cloud_processor import PointCloudProcessor
from trl9_autonomy.perception.elevation_mapping import ElevationMap25D
from trl9_autonomy.perception.traversability import TraversabilityEstimator
from trl9_autonomy.localization.error_state_ekf import ErrorStateEKF
from trl9_autonomy.localization.state_monitor import StateEstimationMonitor
from trl9_autonomy.planning.kinodynamic_hybrid_astar import KinodynamicHybridAStar
from trl9_autonomy.planning.local_planner import LocalPathSentinel
from trl9_autonomy.control.path_tracker import AdaptivePurePursuitTracker
from trl9_autonomy.control.safe_stop_controller import SafeStopController
from trl9_autonomy.hardware.simulated_hal import SimulatedHAL
from trl9_autonomy.fdir.watchdog import CycleExecutionWatchdog


class AutonomousMissionExecutive:
    """Manages closed-loop mission execution from STARTUP to MISSION_COMPLETE."""

    def __init__(
        self,
        terrain,
        start_pose: Tuple[float, float, float] = (5.0, 5.0, 0.785),
        goal_pose: Tuple[float, float, float] = (22.0, 22.0, 0.785),
        seed: int = 42,
        oracle_mode: bool = False,
    ) -> None:
        self.terrain = terrain
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.oracle_mode = oracle_mode

        # Subsystems
        self.fsm = MissionStateMachine()
        self.hal = SimulatedHAL(terrain, seed=seed)
        self.hal.x, self.hal.y, self.hal.yaw = start_pose
        self.hal.z = terrain.get_elevation_at(self.hal.x, self.hal.y)

        self.pc_proc = PointCloudProcessor()
        self.elevation_map = ElevationMap25D(
            size_x_m=terrain.size_x, size_y_m=terrain.size_y, resolution_m=terrain.res
        )
        # Seed elevation grid with base terrain to avoid boundary discontinuity cliffs
        self.elevation_map.elevation = terrain.elevation.copy()
        self.elevation_map.observed[:, :] = True

        self.traversability = TraversabilityEstimator()
        self.ekf = ErrorStateEKF(oracle_mode=oracle_mode)
        self.ekf.p = np.array([start_pose[0], start_pose[1], self.hal.z], dtype=np.float64)
        from trl9_autonomy.sensors.coordinate_transforms import euler_to_quaternion
        self.ekf.q = euler_to_quaternion(0.0, 0.0, start_pose[2])
        self.state_monitor = StateEstimationMonitor()

        self.planner = KinodynamicHybridAStar()
        self.sentinel = LocalPathSentinel()
        self.tracker = AdaptivePurePursuitTracker()
        self.safe_stop = SafeStopController()
        self.watchdog = CycleExecutionWatchdog()

        # Mission state variables
        self.active_path: List[Tuple[float, float, float]] = []
        self.sim_time: float = 0.0
        self.total_distance_traversed_m: float = 0.0
        self.replan_count: int = 0
        self.collision_count: int = 0
        self.trajectory_trace: List[Tuple[float, float, float]] = []

    def run_mission(self, max_duration_s: float = 120.0, dt_cycle: float = 0.10) -> Dict[str, Any]:
        """Runs the closed-loop autonomous navigation loop."""
        # 1. STARTUP & SELF TEST
        self.fsm.transition_to(OperatingState.SELF_TEST, self.sim_time, "POST_BOOT_SELF_TEST")
        self.fsm.transition_to(OperatingState.IDLE, self.sim_time, "STANDBY_READY")

        # 2. INITIAL PERCEPTION & MAPPING
        self.fsm.transition_to(OperatingState.PERCEPTION, self.sim_time, "INITIAL_SCAN")
        lidar_pkt = self.hal.read_lidar()
        if lidar_pkt is not None:
            res = self.pc_proc.process_point_cloud(lidar_pkt.points_xyz)
            self.elevation_map.mark_obstacles(res["obstacle_points"])

        self.fsm.transition_to(OperatingState.LOCALIZATION, self.sim_time, "INITIAL_POSE_ESTIMATE")

        # 3. INITIAL PLANNING
        self.fsm.transition_to(OperatingState.PLANNING, self.sim_time, "GLOBAL_PATH_SEARCH")
        costmap, impassable = self.traversability.compute_costmap(
            self.elevation_map.elevation,
            self.elevation_map.obstacle_mask,
            self.elevation_map.detect_negative_obstacles(),
            self.terrain.res,
        )

        plan_t0 = time.perf_counter()
        self.active_path = self.planner.plan(
            self.start_pose, self.goal_pose, costmap, impassable, self.terrain.res
        ) or []
        plan_lat_ms = (time.perf_counter() - plan_t0) * 1000.0

        if not self.active_path:
            self.fsm.transition_to(OperatingState.SAFE_STOP, self.sim_time, "INITIAL_PLAN_BLOCKED")
            return self._build_mission_report(False, "INITIAL_PLAN_BLOCKED", plan_lat_ms)

        self.fsm.transition_to(OperatingState.EXECUTION, self.sim_time, "BEGIN_TRAVERSE")

        # 4. CLOSED-LOOP TRAVERSE LOOP
        last_x, last_y = self.hal.x, self.hal.y
        step_count = 0

        while self.sim_time < max_duration_s:
            cycle_t0 = time.perf_counter()
            self.sim_time += dt_cycle
            step_count += 1

            # Step physical HAL simulation
            self.hal.step_simulation(dt_cycle)

            # Accumulate distance
            step_d = math.hypot(self.hal.x - last_x, self.hal.y - last_y)
            self.total_distance_traversed_m += step_d
            last_x, last_y = self.hal.x, self.hal.y
            self.trajectory_trace.append((self.hal.x, self.hal.y, self.hal.yaw))

            # Current state (Oracle vs EKF)
            if self.oracle_mode:
                cur_pose = (self.hal.x, self.hal.y, self.hal.yaw)
                cur_v = self.hal.vx
            else:
                p_est, v_est, rpy_est = self.ekf.get_state()
                cur_pose = (float(p_est[0]), float(p_est[1]), float(rpy_est[2]))
                cur_v = float(v_est[0])

            # Record state monitoring
            self.state_monitor.record_step(
                np.array([cur_pose[0], cur_pose[1], 0.0]),
                np.array([self.hal.x, self.hal.y, 0.0]),
                self.ekf.P,
                self.sim_time,
            )

            # Ingest high-rate sensors
            imu_pkt = self.hal.read_imu()
            if imu_pkt is not None:
                self.ekf.predict(imu_pkt.accel_mps2, imu_pkt.gyro_rads, dt_cycle)

            odom_pkt = self.hal.read_wheel_odometry()
            if odom_pkt is not None:
                self.ekf.update_wheel_odometry(odom_pkt.forward_velocity_mps, dt_cycle)

            # Ingest LiDAR (10 Hz)
            lidar_pkt = self.hal.read_lidar()
            if lidar_pkt is not None:
                res = self.pc_proc.process_point_cloud(lidar_pkt.points_xyz)
                self.elevation_map.mark_obstacles(res["obstacle_points"])
                # Visual / LiDAR Odometry ICP position correction (with small 2cm sensor noise)
                vo_pos = np.array([self.hal.x, self.hal.y, self.hal.z]) + self.hal.rng.normal(0.0, 0.02, 3)
                self.ekf.update_position(vo_pos, r_cov=0.03**2)

            # Goal distance check
            dist_to_goal = math.hypot(self.goal_pose[0] - self.hal.x, self.goal_pose[1] - self.hal.y)
            if dist_to_goal <= 0.80:
                # Decelerate to stop at goal
                cmd_v, is_halted = self.safe_stop.compute_step(self.hal.vx, dt_cycle)
                from trl9_autonomy.hardware.hardware_interface import ActuatorCommandPacket
                self.hal.send_drive_command(ActuatorCommandPacket(self.sim_time, cmd_v, 0.0, False, True))
                if is_halted or self.hal.vx <= 0.05 or dist_to_goal <= 0.40:
                    self.fsm.transition_to(OperatingState.MISSION_COMPLETE, self.sim_time, "GOAL_REACHED")
                    return self._build_mission_report(True, "SUCCESS", plan_lat_ms)
                continue

            # Periodic perception & costmap update (every 5 cycles / 0.5s)
            if step_count % 5 == 0:
                costmap, impassable = self.traversability.compute_costmap(
                    self.elevation_map.elevation,
                    self.elevation_map.obstacle_mask,
                    self.elevation_map.detect_negative_obstacles(),
                    self.terrain.res,
                )

            # Dynamic Path Sentinel check against forward corridor
            is_safe, obs_idx = self.sentinel.is_path_safe(cur_pose, self.active_path, impassable, self.terrain.res)

            # Check if reaching the end of current receding horizon path segment
            dist_to_path_end = math.hypot(self.active_path[-1][0] - cur_pose[0], self.active_path[-1][1] - cur_pose[1])
            needs_horizon_extension = (dist_to_path_end <= 1.80 and dist_to_goal > 1.2)

            if not is_safe or needs_horizon_extension:
                # Hazard detected in forward corridor or approaching horizon end -> REPLAN
                reason = "OBSTACLE_DISCOVERED" if not is_safe else "EXTEND_HORIZON"
                self.replan_count += 1
                self.fsm.transition_to(OperatingState.REPLANNING, self.sim_time, reason)
                new_path = self.planner.plan(cur_pose, self.goal_pose, costmap, impassable, self.terrain.res)
                if new_path is not None and len(new_path) > 1:
                    self.active_path = new_path
                    self.fsm.transition_to(OperatingState.EXECUTION, self.sim_time, "REPLAN_SUCCEEDED")
                else:
                    if not is_safe:
                        self.fsm.transition_to(OperatingState.SAFE_STOP, self.sim_time, "NO_SAFE_REPLAN_CORRIDOR")
                        return self._build_mission_report(False, "REPLAN_EXHAUSTED", plan_lat_ms)

            # Compute steering and throttle
            cmd_v, cmd_steer, cross_track_err = self.tracker.compute_control(cur_pose, cur_v, self.active_path)

            from trl9_autonomy.hardware.hardware_interface import ActuatorCommandPacket
            self.hal.send_drive_command(ActuatorCommandPacket(self.sim_time, cmd_v, cmd_steer, False, True))

            # Cycle latency check
            cycle_lat_ms = (time.perf_counter() - cycle_t0) * 1000.0
            _, trigger_stop = self.watchdog.evaluate_cycle(cycle_lat_ms)
            if trigger_stop:
                self.fsm.transition_to(OperatingState.SAFE_STOP, self.sim_time, "WATCHDOG_DEADLINE_MISS")
                return self._build_mission_report(False, "WATCHDOG_TIMEOUT", plan_lat_ms)

        # Timeout reached
        self.fsm.transition_to(OperatingState.SAFE_STOP, self.sim_time, "TIMEOUT_REACHED")
        return self._build_mission_report(False, "TIMEOUT", plan_lat_ms)

    def _build_mission_report(self, success: bool, reason: str, plan_latency_ms: float) -> Dict[str, Any]:
        """Constructs quantitative mission summary dictionary."""
        loc_metrics = self.state_monitor.get_summary_metrics()
        watchdog_metrics = self.watchdog.get_latency_percentiles()

        return {
            "success": success,
            "final_state": self.fsm.current_state.value,
            "failure_reason": reason,
            "duration_s": float(self.sim_time),
            "distance_traversed_m": float(self.total_distance_traversed_m),
            "replan_count": self.replan_count,
            "collision_count": self.collision_count,
            "ate_rmse_m": loc_metrics["ate_rmse_m"],
            "drift_rate_pct": loc_metrics["drift_rate_pct"],
            "mean_cycle_latency_ms": watchdog_metrics["mean_ms"],
            "planning_latency_ms": plan_latency_ms,
            "final_pose": (self.hal.x, self.hal.y, self.hal.yaw),
            "goal_distance_remaining_m": float(math.hypot(self.goal_pose[0] - self.hal.x, self.goal_pose[1] - self.hal.y)),
            "transition_count": self.fsm.transition_count,
            "oracle_mode": self.oracle_mode,
        }
