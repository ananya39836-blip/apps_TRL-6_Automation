# APPS System Requirements Specification (SRS) — TRL 9 Baseline

**Document ID**: `APPS-SRS-TRL9-001`  
**Revision**: 4.0.0  
**Classification**: NASA TRL 9 Operational-System Readiness Specification  
**Safety Invariant**: Research Guard Locks Active (`PROTOTYPE_LOCK = True`, `NAVIGATION_READY = False`)  

---

## 1. Scope & System Definition

This document establishes the functional, performance, environmental, interface, safety, and qualification requirements for the **Automated Planetary Perception System (APPS)** targeting **Technology Readiness Level 9 (TRL 9)**.

The system under evaluation is formally defined as:
> *"An integrated autonomous planetary rover perception, localization, mapping, terrain assessment, path planning, motion control, fault management, and mission-execution system demonstrated through an operationally representative synthetic and breadboard prototype."*

### Strict Boundary of Technology Scope & Non-Affiliation Invariant
1. **Third-Party Research Status**: This system is developed as an academic and industrial robotics autonomy prototype. NASA does not automatically certify, endorse, or qualify third-party software.
2. **Explicit Qualification Separation**:
   - **Software Algorithmic Qualification**: Achievable through formal V&V, synthetic Monte-Carlo operational campaigns, continuous regression testing, and deterministic replay.
   - **Subsystem & Integration Qualification**: Verified through discrete hardware abstraction layers (HAL), emulated space-grade telemetry (CCSDS 133.0-B-2), and timing determinism.
   - **Flight Hardware & Environmental Qualification (TRL 9 Incomplete)**: Requires physical flight-grade avionics (e.g., BAE RAD750 / RAD5545, COBB, space-grade FPGAs), thermal-vacuum chamber (TVAC) testing (-120°C to +25°C), vibrational launch profile qualification (GEVS vibration testing), and radiation tolerance (100 krad TID). **This physical hardware qualification cannot be achieved by software alone and remains an explicit qualification limitation.**

---

## 2. Requirement Taxonomy & Conventions

Requirements follow NASA Systems Engineering conventions:
- **`SHALL`**: Mandatory functional or performance constraint.
- **`SHOULD`**: Preferred architectural implementation.
- **Verification Methods**:
  - **`T` (Test)**: Quantitative measurement via automated automated execution in `pytest` or Monte-Carlo campaigns.
  - **`A` (Analysis)**: Mathematical proof, kinematic calculation, or statistical evaluation.
  - **`D` (Demonstration)**: Qualitative verification of observable system behavior (e.g., FDIR state transition).
  - **`I` (Inspection)**: Visual inspection of source code, configuration schema, or documentation.

---

## 3. Formal System Requirements Matrix

### 3.1 Safety, Invariants & Security Requirements

#### `TRL9-REQ-01`: Research Prototype Guard Latches
- **Purpose**: Prevent accidental deployment of experimental research code to physical mission hardware without authorized ground crew intervention.
- **Specification**: The system `SHALL` compile and execute with immutable global flags `PROTOTYPE_LOCK = True` and `NAVIGATION_READY = False`. Any attempt to toggle these flags in autonomous flight mode `SHALL` force immediate transition to `FAULT` or `SAFE_STOP`.
- **Verification Method**: `T` / `I`
- **Acceptance Criteria**: Both flags evaluate to their locked boolean values upon package import; mutation raises `AttributeError` or assertion failure.
- **Test ID**: `tests/test_trl9_perception.py::test_safety_invariants_latched`
- **Evidence Location**: `trl9_autonomy/__init__.py`
- **Remaining Limitations**: Purely software guard; physical flight avionics require hardware write-protect jumpers.

#### `TRL9-REQ-02`: Coordinate Frame Standardization
- **Purpose**: Prevent reference frame mismatch across perception, localization, and control modules.
- **Specification**: All spatial transformations `SHALL` strictly adhere to standardized frames: World $\mathcal{W}$ (Topocentric East-North-Up), Body $\mathcal{B}$ (X-Forward, Y-Left, Z-Up), LiDAR $\mathcal{L}$, Camera $\mathcal{C}$, and IMU $\mathcal{I}$. All units `SHALL` be standard SI ($m, m/s, m/s^2, rad, rad/s$).
- **Verification Method**: `T` / `A`
- **Acceptance Criteria**: Analytical round-trip transformation error $||\mathbf{T} \cdot \mathbf{T}^{-1} - \mathbf{I}||_F \le 10^{-10}$ across arbitrary 6-DOF rotations and translations.
- **Test ID**: `tests/test_trl9_perception.py::test_transform_se3_composition_and_inverse`
- **Evidence Location**: `trl9_autonomy/sensors/coordinate_transforms.py`
- **Remaining Limitations**: Relies on rigid body assumption; does not model chassis flexure or thermal expansion.

---

### 3.2 Sensor Ingestion & Telemetry Requirements

#### `TRL9-REQ-03`: Space Telemetry Framing & Checksums
- **Purpose**: Ensure flight telemetry complies with international space communications standards.
- **Specification**: All sensor data packets and actuator commands `SHALL` be framed in a 16-byte CCSDS-compliant header containing Packet Version Number, Type, APID, Sequence Flags, Packet Sequence Count, Packet Data Length, and a CCITT-16 CRC checksum.
- **Verification Method**: `T`
- **Acceptance Criteria**: Bit-flip injection in packet payload fails CRC-16 verification; correctly framed packets pass checksum check.
- **Test ID**: `tests/test_trl9_system_integration.py::test_ccsds_telemetry_packet_framing`
- **Evidence Location**: `trl9_autonomy/hardware/hardware_interface.py`
- **Remaining Limitations**: Checksum validation performed in CPU software rather than hardware FPGA/ASIC.

#### `TRL9-REQ-04`: Telemetry Ingestion Gatekeeper & Sanity Checks
- **Purpose**: Reject invalid, stale, corrupt, or out-of-order telemetry prior to algorithmic consumption.
- **Specification**: The sensor validator `SHALL` reject packets containing `NaN`, `+Inf`, `-Inf`, non-monotonic timestamps ($\Delta t \le 0$), or sensor values outside physical operational limits ($|a| > 50\text{ m/s}^2, |\omega| > 10\text{ rad/s}$).
- **Verification Method**: `T`
- **Acceptance Criteria**: 100% of packets with corrupted floats are dropped with error flags raised; valid packets pass with zero latency penalty.
- **Test ID**: `tests/test_trl9_perception.py::test_sensor_validator_rejects_nan_inf`
- **Evidence Location**: `trl9_autonomy/sensors/sensor_validator.py`
- **Remaining Limitations**: Rejection buffers assume finite ring-buffer memory; buffer overflow drops oldest frames.

---

### 3.3 Perception & Mapping Requirements

#### `TRL9-REQ-05`: Point Cloud Filtering & Ground Segmentation
- **Purpose**: Extract terrain geometry and isolate lethal obstacles from raw LiDAR point clouds.
- **Specification**: The perception pipeline `SHALL` downsample raw point clouds using a 3D voxel grid filter ($\Delta = 0.20\text{ m}$), filter statistical outliers, and segment points into ground and obstacle classes using Progressive Morphological Filtering (PMF) within $\le 20.0\text{ ms}$.
- **Verification Method**: `T`
- **Acceptance Criteria**: Ground extraction recall $\ge 90\%$, obstacle classification precision $\ge 90\%$ on synthetic calibration terrains.
- **Test ID**: `tests/test_trl9_perception.py::test_point_cloud_processor_voxel_and_ground`
- **Evidence Location**: `trl9_autonomy/perception/point_cloud_processor.py`
- **Remaining Limitations**: PMF assumes continuous ground connectivity; dense overhangs require full 3D voxel grids.

#### `TRL9-REQ-06`: Recursive 2.5D Elevation Grid Mapping
- **Purpose**: Build a persistent local elevation map with probabilistic uncertainty.
- **Specification**: The elevation map `SHALL` update cell elevations using a 1D recursive Kalman filter update for elevation and variance:
$$\hat{z}_{k} = \hat{z}_{k-1} + K_k (z_{meas} - \hat{z}_{k-1}), \quad \sigma^2_k = (1 - K_k) \sigma^2_{k-1}$$
- **Verification Method**: `T`
- **Acceptance Criteria**: Sequential observations of a static surface strictly decrease cell variance; spatial elevation RMSE $\le 0.05\text{ m}$ against ground truth DEM.
- **Test ID**: `tests/test_trl9_perception.py::test_elevation_mapping_kalman_update`
- **Evidence Location**: `trl9_autonomy/perception/elevation_mapping.py`
- **Remaining Limitations**: 2.5D grid cannot represent multiple elevation layers (e.g., caves or underpasses).

#### `TRL9-REQ-07`: Multi-Criteria Traversability Cost Assessment
- **Purpose**: Quantify terrain traversal risk based on physical rover limitations.
- **Specification**: Traversability cost $C(x, y) \in [0.0, 1.0]$ `SHALL` combine macroscopic surface gradient slope ($\theta_{max} = 25.0^\circ$), step obstacles ($h_{step} > 0.20\text{ m}$), surface roughness variance ($\sigma_z > 0.08\text{ m}$), and negative obstacle (crater) depressions. Cells exceeding tip-over limits ($\ge 30.0^\circ$) `SHALL` be classified as lethal ($C = 1.0$).
- **Verification Method**: `T`
- **Acceptance Criteria**: Smooth horizontal regolith has cost $C \le 0.15$; vertical step obstacles and steep crater rims have cost $C \ge 0.95$.
- **Test ID**: `tests/test_trl9_perception.py::test_traversability_evaluation_and_inflation`
- **Evidence Location**: `trl9_autonomy/perception/traversability.py`
- **Remaining Limitations**: Soil shear parameters are estimated globally rather than identified dynamically online per-wheel.

---

### 3.4 State Estimation & Localization Requirements

#### `TRL9-REQ-08`: 15-State Error-State Extended Kalman Filter (ES-EKF)
- **Purpose**: Provide high-rate, drift-bounded 6-DOF state estimates under Mars surface conditions.
- **Specification**: The state estimator `SHALL` maintain a 15-state error vector $\delta \mathbf{x} = [\delta \mathbf{p}, \delta \mathbf{v}, \delta \boldsymbol{\theta}, \delta \mathbf{b}_a, \delta \mathbf{b}_g]^T \in \mathbb{R}^{15}$. Propagation `SHALL` execute at 100 Hz using strapdown IMU mechanization. Measurement updates `SHALL` execute using Joseph-form covariance updates:
$$\mathbf{P}^+ = (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P}^- (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$$
- **Verification Method**: `T` / `A`
- **Acceptance Criteria**: Covariance matrix $\mathbf{P}$ remains strictly symmetric positive-definite; position error drift rate $\le 3.0\%$ of total distance traversed under nominal wheel odometry.
- **Test ID**: `tests/test_trl9_localization.py::test_es_ekf_joseph_update`
- **Evidence Location**: `trl9_autonomy/localization/error_state_ekf.py`
- **Remaining Limitations**: Visual odometry updates are emulated via synthetic pose measurements during GPS-denied traverses.

#### `TRL9-REQ-09`: Dual-Mode Ground Truth (Oracle) vs Estimated Telemetry
- **Purpose**: Enable rigorous verification and ablation studies without contaminating operational navigation logic.
- **Specification**: The state estimator `SHALL` provide an explicit dual-mode flag `oracle_mode: bool`. When `False`, navigation planners and controllers `SHALL` strictly consume estimated EKF states. Ground-truth states `SHALL` be isolated for ATE/RPE error metrics recording.
- **Verification Method**: `T`
- **Acceptance Criteria**: State monitor correctly computes Absolute Trajectory Error (ATE) RMSE between estimated and oracle trajectories.
- **Test ID**: `tests/test_trl9_localization.py::test_state_monitor_metrics`
- **Evidence Location**: `trl9_autonomy/localization/state_monitor.py`
- **Remaining Limitations**: In physical deployment, oracle ground truth is unavailable except via external optical tracking or total stations.

---

### 3.5 Planning & Collision Avoidance Requirements

#### `TRL9-REQ-10`: Continuous Swept Oriented Bounding Box (OBB) Collision Checking
- **Purpose**: Guarantee geometric clearance for the full rover vehicle footprint.
- **Specification**: The collision checker `SHALL` test the full rectangular footprint ($L = 2.80\text{ m}, W = 2.20\text{ m}$) plus an explicit safety buffer margin ($\Delta r = 0.35\text{ m}$) across all planned continuous poses $(x, y, \psi)$ using the Separating Axis Theorem (SAT).
- **Verification Method**: `T`
- **Acceptance Criteria**: Zero false negatives; poses intersecting lethal obstacle masks are rejected in $\le 0.10\text{ ms}$ per pose.
- **Test ID**: `tests/test_trl9_navigation.py::test_collision_checker_obb`
- **Evidence Location**: `trl9_autonomy/planning/collision_checker.py`
- **Remaining Limitations**: Assumes rigid 2D planar projection of rover footprint.

#### `TRL9-REQ-11`: Kinodynamic Hybrid A* Motion Planning
- **Purpose**: Generate dynamically executable, kinematically feasible trajectories satisfying vehicle curvature bounds.
- **Specification**: The global planner `SHALL` search continuous $SE(2)$ configuration space $(x, y, \psi)$ subject to maximum curvature $\kappa \le 1.50\text{ m}^{-1}$ (minimum turning radius $R_{min} \ge 0.67\text{ m}$). Paths `SHALL` include analytic expansions and progressive receding-horizon fallback.
- **Verification Method**: `T`
- **Acceptance Criteria**: All generated trajectory segments satisfy $|\kappa(s)| \le 1.50\text{ m}^{-1}$; search duration $\le 80.0\text{ ms}$ for a 20m traverse.
- **Test ID**: `tests/test_trl9_navigation.py::test_kinodynamic_planner_curvature_bound`
- **Evidence Location**: `trl9_autonomy/planning/kinodynamic_hybrid_astar.py`
- **Remaining Limitations**: 3D grade effects are accounted for via costmap penalty rather than full 3D kinodynamic branch search.

---

### 3.6 Motion Control & Vehicle Dynamics Requirements

#### `TRL9-REQ-12`: Adaptive Path Tracking with Curvature Speed Derating
- **Purpose**: Track planned trajectories accurately while mitigating lateral slippage and tip-over risks.
- **Specification**: The path tracker `SHALL` compute steering angle $\delta$ and linear velocity $v$ via adaptive Pure Pursuit. Command velocity `SHALL` automatically derate in curves such that lateral acceleration does not exceed $a_{lat,max} \le 0.30\text{ m/s}^2$:
$$v_{max} = \sqrt{\frac{a_{lat,max}}{|\kappa| + \epsilon}}$$
- **Verification Method**: `T`
- **Acceptance Criteria**: Cross-track error $e_{ct} \le 0.25\text{ m}$ under nominal regolith conditions; zero wheel velocity saturation.
- **Test ID**: `tests/test_trl9_navigation.py::test_path_tracker_pure_pursuit`
- **Evidence Location**: `trl9_autonomy/control/path_tracker.py`
- **Remaining Limitations**: Actuator response is modeled as first-order lag ($\tau = 0.10\text{ s}$) rather than non-linear motor inductance.

#### `TRL9-REQ-13`: Certified Constant-Jerk Safe Stop Controller
- **Purpose**: Execute an emergency or nominal halt without inducing vehicle tipping or wheel skidding.
- **Specification**: Upon receiving a `SAFE_STOP` trigger, the controller `SHALL` decelerate the vehicle to $v = 0.0\text{ m/s}$ using a certified constant-jerk profile ($a = -0.20\text{ m/s}^2, j = -0.50\text{ m/s}^3$).
- **Verification Method**: `T`
- **Acceptance Criteria**: Deceleration strictly monotonically decreases velocity to 0; stopping distance $d \le 0.60\text{ m}$ from $v = 0.40\text{ m/s}$ with zero overshoot.
- **Test ID**: `tests/test_trl9_navigation.py::test_safe_stop_controller`
- **Evidence Location**: `trl9_autonomy/control/safe_stop_controller.py`
- **Remaining Limitations**: Assumes regolith provides at least $\mu \ge 0.25$ traction.

#### `TRL9-REQ-14`: Rocker-Bogie Kinematics & Terramechanics Integration
- **Purpose**: Simulate realistic Mars rover wheel-ground interaction, sinkage, and drawbar pull.
- **Specification**: The dynamics engine `SHALL` compute the 6-wheel rocker-bogie pitch angles, differential tilt, normal wheel loads, Bekker-Wong sinkage ($z_0 = (p / (k_c/b + k_\phi))^{1/n}$), and Janosi-Hanamoto shear traction.
- **Verification Method**: `T`
- **Acceptance Criteria**: Passive articulation keeps all 6 wheels in ground contact across obstacles up to $0.25\text{ m}$ height; wheel normal force sum equals total vehicle weight ($W = m \cdot g_{mars} \approx 3162.6\text{ N}$).
- **Test ID**: `tests/test_trl9_navigation.py::test_rocker_bogie_kinematics`
- **Evidence Location**: `trl9_autonomy/dynamics/rocker_bogie.py`
- **Remaining Limitations**: Discrete rigid-body solver; does not simulate soil particle fluidization (DEM particle physics).

---

### 3.7 Mission Management, FDIR & Telemetry Requirements

#### `TRL9-REQ-15`: 13-State Deterministic FDIR Executive
- **Purpose**: Guarantee deterministic operational state transitions and rapid fault isolation.
- **Specification**: The executive `SHALL` implement exactly 13 mutually exclusive operational modes: `STARTUP`, `SELF_TEST`, `IDLE`, `PERCEPTION`, `LOCALIZATION`, `PLANNING`, `EXECUTION`, `REPLANNING`, `SAFE_STOP`, `RECOVERY`, `MISSION_COMPLETE`, `MISSION_ABORT`, `FAULT`. Unhandled events `SHALL` default to `FAULT` and trigger `SAFE_STOP`.
- **Verification Method**: `T` / `D`
- **Acceptance Criteria**: 100% of defined state transitions conform to formal state transition matrix; unauthorized transitions are rejected.
- **Test ID**: `tests/test_trl9_navigation.py::test_fdir_executive_transitions`
- **Evidence Location**: `trl9_autonomy/mission/state_machine.py`
- **Remaining Limitations**: Executive states are held in memory; non-volatile flash journaling required for power-loss recovery.

#### `TRL9-REQ-16`: Execution Cycle Time Watchdog & Real-Time Deadlines
- **Purpose**: Prevent timing overrun and ensure RTOS compatibility at 10 Hz navigation rates.
- **Specification**: The real-time watchdog `SHALL` monitor the total navigation loop execution time. If any cycle exceeds $100.0\text{ ms}$ (deadline miss), a fault counter `SHALL` increment; 3 consecutive misses `SHALL` force transition to `SAFE_STOP`.
- **Verification Method**: `T`
- **Acceptance Criteria**: Synthetic delay injection $>100\text{ ms}$ triggers fault escalation; nominal cycles execute in $<40\text{ ms}$.
- **Test ID**: `tests/test_trl9_system_integration.py::test_watchdog_deadline_monitor`
- **Evidence Location**: `trl9_autonomy/fdir/watchdog.py`
- **Remaining Limitations**: Python execution relies on OS process scheduling; hard real-time requires dedicated RTOS thread.

#### `TRL9-REQ-17`: Reproducible Monte-Carlo Qualification Framework
- **Purpose**: Produce scientifically defensible, statistically bounded evidence of system performance.
- **Specification**: The qualification suite `SHALL` execute 20 standardized operational scenarios (`SC-01` to `SC-20`) with deterministic random seeds, recording mission success, cross-track error, ATE RMSE, planning latency, and Wilson score 95% Confidence Intervals.
- **Verification Method**: `T` / `A`
- **Acceptance Criteria**: Monte-Carlo campaign script executes headlessly from CLI; outputs valid JSON summary and CSV tabular logs.
- **Test ID**: `experiments/run_trl9_campaign.py`
- **Evidence Location**: `results/trl9/qualification_summary.json`
- **Remaining Limitations**: Simulation testbed uses synthetic sensor models rather than physical flight hardware in an environmental chamber.
