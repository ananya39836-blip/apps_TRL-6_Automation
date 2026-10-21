"""APPS TRL 9 Engineering-Grade Autonomy & Qualification Dashboard.

Provides interactive 3D terrain visualization, live telemetry traces,
15-state EKF inspection, traversability cost layers, 20-scenario qualification
benchmarks, and exportable verification manifests.
"""
from __future__ import annotations
import math
import json
import os
import pandas as pd
import numpy as np
import plotly.graph_objects as go
import streamlit as st

from trl9_autonomy import (
    PROTOTYPE_LOCK,
    NAVIGATION_READY,
    OperatingState,
    SCENARIO_IDS,
    __version__,
)
from trl9_autonomy.environment.scenario_generator import ScenarioGenerator
from trl9_autonomy.mission.mission_executive import AutonomousMissionExecutive

st.set_page_config(
    page_title="APPS — TRL 9 Operational Autonomy Dashboard",
    page_icon="🚀",
    layout="wide",
)

st.title("🛰️ APPS: Automated Planetary Perception System")
st.caption(f"NASA TRL 9 Operational-System Readiness & Verification Framework | Version {__version__}")

# Security & Safety Banner
col_b1, col_b2, col_b3 = st.columns(3)
with col_b1:
    st.error("🔒 RESEARCH PROTOTYPE LOCK ENGAGED (`PROTOTYPE_LOCK = True`)")
with col_b2:
    st.warning("⚠️ FLIGHT ACTUATION LATCHED (`NAVIGATION_READY = False`)")
with col_b3:
    st.info("🌐 ENVIRONMENT: SYNTHETIC BREADBOARD / MARS ANALOG")

# Sidebar Controls
st.sidebar.header("🕹️ Mission & Scenario Configuration")
selected_scenario = st.sidebar.selectbox("Operational Scenario", SCENARIO_IDS, index=0)
seed = st.sidebar.number_input("Deterministic Random Seed", min_value=1, max_value=99999, value=101)
oracle_mode = st.sidebar.checkbox("Oracle Mode (Ground Truth Bypass)", value=False)
max_duration_s = st.sidebar.slider("Mission Time Limit (s)", min_value=20.0, max_value=180.0, value=80.0, step=10.0)

st.sidebar.markdown("---")
st.sidebar.subheader("🚨 Fault Injection Control")
inject_slip = st.sidebar.checkbox("Inject Severe Slip (s = 0.45)", value=False)
inject_timeout = st.sidebar.checkbox("Inject Sensor Dropout (2.0s blackout)", value=False)

# Load / Generate Scenario
sc_data = ScenarioGenerator.generate_scenario(selected_scenario, seed=seed)
terrain = sc_data["terrain"]
start_pose = sc_data["start_pose"]
goal_pose = sc_data["goal_pose"]

# Execute Mission on Click or Cached
if st.sidebar.button("▶️ Execute Autonomous Mission", type="primary"):
    with st.spinner(f"Executing closed-loop autonomous navigation on {selected_scenario}..."):
        executive = AutonomousMissionExecutive(
            terrain=terrain,
            start_pose=start_pose,
            goal_pose=goal_pose,
            seed=seed,
            oracle_mode=oracle_mode,
        )
        if inject_slip:
            executive.hal.vx *= 0.55
        report = executive.run_mission(max_duration_s=max_duration_s)
        st.session_state["last_report"] = report
        st.session_state["executive"] = executive
        st.session_state["scenario_data"] = sc_data

report = st.session_state.get("last_report", None)
executive = st.session_state.get("executive", None)

# Main UI Tabs
tab_3d, tab_telemetry, tab_costmap, tab_qualification, tab_export = st.tabs([
    "🪐 3D Terrain & Trajectory",
    "📊 Vehicle Telemetry & EKF",
    "🗺️ Perception & Costmap",
    "📈 20-Scenario Qualification Matrix",
    "📥 Evidence & Export Package",
])

# Tab 1: 3D Visualization
with tab_3d:
    st.subheader(f"3D Digital Elevation Model ({selected_scenario})")
    
    # Subsample terrain for Plotly rendering performance
    stride = 2
    x_sub = terrain.x_coords[::stride]
    y_sub = terrain.y_coords[::stride]
    z_sub = terrain.elevation[::stride, ::stride].T

    fig_3d = go.Figure()
    fig_3d.add_trace(go.Surface(
        x=x_sub,
        y=y_sub,
        z=z_sub,
        colorscale="YlOrBr",
        opacity=0.85,
        name="Planetary DEM",
        showscale=False,
    ))

    # Plot Start and Goal
    fig_3d.add_trace(go.Scatter3d(
        x=[start_pose[0]], y=[start_pose[1]], z=[terrain.get_elevation_at(start_pose[0], start_pose[1]) + 0.3],
        mode="markers+text", marker=dict(size=8, color="green"),
        text=["START"], textposition="top center", name="Start Datum"
    ))
    fig_3d.add_trace(go.Scatter3d(
        x=[goal_pose[0]], y=[goal_pose[1]], z=[terrain.get_elevation_at(goal_pose[0], goal_pose[1]) + 0.3],
        mode="markers+text", marker=dict(size=8, color="red"),
        text=["GOAL"], textposition="top center", name="Goal Waypoint"
    ))

    # Plot Planned & Executed Path if available
    if executive and executive.trajectory_trace:
        tx = [p[0] for p in executive.trajectory_trace]
        ty = [p[1] for p in executive.trajectory_trace]
        tz = [terrain.get_elevation_at(p[0], p[1]) + 0.15 for p in executive.trajectory_trace]
        fig_3d.add_trace(go.Scatter3d(
            x=tx, y=ty, z=tz,
            mode="lines", line=dict(color="blue", width=6),
            name="Executed Path (15-State ES-EKF)"
        ))

    if executive and executive.active_path:
        px = [p[0] for p in executive.active_path]
        py = [p[1] for p in executive.active_path]
        pz = [terrain.get_elevation_at(p[0], p[1]) + 0.20 for p in executive.active_path]
        fig_3d.add_trace(go.Scatter3d(
            x=px, y=py, z=pz,
            mode="lines", line=dict(color="cyan", width=3, dash="dash"),
            name="Planned Kinodynamic Path"
        ))

    fig_3d.update_layout(
        scene=dict(
            xaxis_title="East X (m)",
            yaxis_title="North Y (m)",
            zaxis_title="Zenith Z (m)",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, b=0, t=30),
        height=600,
    )
    st.plotly_chart(fig_3d, use_container_width=True)

# Tab 2: Telemetry
with tab_telemetry:
    st.subheader("High-Rate Navigation Telemetry")
    if report:
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("Mission Status", report["final_state"], delta="SUCCESS" if report["success"] else "HALTED")
        c2.metric("Distance Traversed", f"{report['distance_traversed_m']:.2f} m")
        c3.metric("ATE RMSE", f"{report['ate_rmse_m']:.3f} m")
        c4.metric("Mean Latency", f"{report['mean_cycle_latency_ms']:.2f} ms")

        # Telemetry graphs
        if executive and executive.state_monitor.est_positions:
            est_pts = np.array(executive.state_monitor.est_positions)
            gt_pts = np.array(executive.state_monitor.gt_positions)
            t_s = executive.state_monitor.timestamps

            df_pos = pd.DataFrame({
                "Time (s)": t_s,
                "Est X (m)": est_pts[:, 0],
                "GT X (m)": gt_pts[:, 0],
                "Est Y (m)": est_pts[:, 1],
                "GT Y (m)": gt_pts[:, 1],
            })
            st.line_chart(df_pos.set_index("Time (s)"))
    else:
        st.info("Execute a mission from the sidebar to inspect real-time telemetry traces.")

# Tab 3: Costmap
with tab_costmap:
    st.subheader("Multi-Criteria Traversability & Hazard Maps")
    if executive:
        col_m1, col_m2 = st.columns(2)
        with col_m1:
            st.markdown("**Elevation Grid (Kalman Filtered)**")
            st.image((executive.elevation_map.elevation - np.min(executive.elevation_map.elevation)) / max(0.1, np.ptp(executive.elevation_map.elevation)), clamp=True)
        with col_m2:
            st.markdown("**Obstacle & Negative Hazard Mask**")
            obs_combined = executive.elevation_map.obstacle_mask | executive.elevation_map.detect_negative_obstacles()
            st.image(obs_combined.astype(float), clamp=True)

# Tab 4: Qualification Matrix
with tab_qualification:
    st.subheader("TRL 9 Standardized 20-Scenario Qualification Matrix")
    summary_path = "results/trl9/qualification_summary.json"
    runs_path = "results/trl9/qualification_runs.csv"

    if os.path.exists(summary_path) and os.path.exists(runs_path):
        with open(summary_path, "r", encoding="utf-8") as f:
            summary = json.load(f)
        df_runs = pd.read_csv(runs_path)

        col_q1, col_q2, col_q3, col_q4 = st.columns(4)
        col_q1.metric("Total Scenarios", summary["total_scenarios"])
        col_q2.metric("Success Rate", f"{summary['success_rate']*100:.1f}%")
        col_q3.metric("Wilson 95% CI", f"[{summary['wilson_ci_95']['lower']*100:.1f}%, {summary['wilson_ci_95']['upper']*100:.1f}%]")
        col_q4.metric("Total Collisions", summary["total_collisions"])

        st.dataframe(df_runs, use_container_width=True)
    else:
        st.info("Run `python -u experiments/run_trl9_campaign.py` to generate the complete 20-scenario evidence matrix.")

# Tab 5: Export Package
with tab_export:
    st.subheader("System Qualification Evidence Package")
    st.markdown("""
    All outputs comply with NASA Systems Engineering standards. Download formal verification manifests below:
    """)
    if report:
        st.download_button(
            "📥 Download Current Run JSON",
            data=json.dumps(report, indent=2),
            file_name=f"{selected_scenario}_mission_report.json",
            mime="application/json",
        )
    if os.path.exists("results/trl9/qualification_summary.json"):
        with open("results/trl9/qualification_summary.json", "r", encoding="utf-8") as f:
            q_data = f.read()
        st.download_button(
            "📥 Download 20-Scenario Qualification Summary JSON",
            data=q_data,
            file_name="qualification_summary.json",
            mime="application/json",
        )
