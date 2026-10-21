# APPS Reproducibility & Verification Guide — TRL 9 Baseline

**Document ID**: `APPS-REP-TRL9-001`  
**Revision**: 4.0.0  
**Classification**: NASA TRL 9 Exact Reproduction Protocol  
**Safety Invariant**: Research Guard Locks Active (`PROTOTYPE_LOCK = True`, `NAVIGATION_READY = False`)  

---

## 1. System Requirements & Environment Setup

To reproduce the verification suite, empirical benchmarks, and interactive dashboard:
- **Operating System**: Windows 10/11, Ubuntu 22.04 LTS, or macOS 13+
- **Python Version**: Python 3.10 to 3.14 (x64)
- **Key Dependencies**:
  ```bash
  pip install numpy scipy pandas plotly streamlit pytest pytest-cov
  ```

---

## 2. Step-by-Step Execution Protocol

### Step 1: Execute Formal Automated Test Suite (27 Tests)
Run the complete unit and integration test suite across perception, localization, navigation, dynamics, and system integration:
```bash
python -m pytest tests/test_trl9_perception.py tests/test_trl9_localization.py tests/test_trl9_navigation.py tests/test_trl9_system_integration.py -v
```
**Expected Outcome**: `27 passed in ~37 seconds (100% pass rate)`.

---

### Step 2: Execute 20-Scenario Monte-Carlo Qualification Campaign
Run the headless qualification campaign across `SC-01` through `SC-20`:
```bash
python -u experiments/run_trl9_campaign.py
```
**Expected Outcome**:
- Outputs progress to stdout for all 20 scenarios.
- Generates `results/trl9/qualification_summary.json` and `results/trl9/qualification_runs.csv`.
- Confirms 0 collisions, ~0.079m mean ATE RMSE, and 60% completion rate with Wilson 95% CI $[38.7\%, 78.1\%]$.

---

### Step 3: Execute Comparative Baseline Benchmark
Run the baseline comparison comparing the proposed pipeline against Pure Odometry and Geometric 2D Grid A*:
```bash
python -u experiments/run_trl9_baselines.py
```
**Expected Outcome**:
- Generates `results/trl9/baseline_comparison.json`.
- Confirms 153.3x drift reduction of the proposed 15-state ES-EKF over unfiltered pure odometry.

---

### Step 4: Launch Interactive Engineering Dashboard
Launch the interactive Streamlit operational station:
```bash
streamlit run apps_trl9_dashboard.py
```
**Features Available**:
1. Interactive 3D planetary digital elevation model surface viewer.
2. Real-time telemetry inspector with 15-state EKF trajectory comparison.
3. Traversability costmaps and negative obstacle crater layers.
4. Live qualification test matrix displaying all 20 scenarios with Wilson 95% CIs.
5. JSON verification manifest export buttons.
