"""Streamlit application for APPS - Automated Planetary Perception System
Includes synthetic scene generator, image upload, uncertainty & unsafe maps,
parameterised planner interface, coverage-risk analysis, and structured observations.
"""
import json
import pickle
import numpy as np
import streamlit as st
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
import apps01 as A

st.set_page_config(page_title="APPS — Automated Planetary Perception System", layout="wide", page_icon="🪐")
st.title("🪐 APPS — Automated Planetary Perception System")
st.sidebar.markdown("### 🧭 Navigation & Subsystems")
st.sidebar.info("💡 **Select any TRL tier or Research Level from the Pages menu above!**")
st.warning("Research prototype trained on SYNTHETIC terrain only. No claims about real Mars performance, minerals, or safe driving.")


class APPSUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if (module in ("__main__", "apps01")) and name == "Ensemble":
            return A.Ensemble
        return super().find_class(module, name)


@st.cache_resource
def load():
    try:
        with open("apps01_model.pkl", "rb") as f:
            return APPSUnpickler(f).load()
    except Exception as e:
        st.error(f"Could not load apps01_model.pkl: {e}. Please run apps01.py to train and save the model.")
        return None


model = load()

mode = st.sidebar.radio("Input Mode", ["Generate synthetic scene", "Upload image (no depth)"])
if mode == "Generate synthetic scene":
    seed = st.sidebar.number_input("Generator Seed", 0, 99999, 7)
    ood = st.sidebar.checkbox("Add novel (never-trained) terrain", True)
    img, depth, lab = A.make_scene(np.random.default_rng(int(seed)), ood)
else:
    up = st.sidebar.file_uploader("Image", type=["png", "jpg", "jpeg"])
    if not up:
        st.info("Upload an image or switch to 'Generate synthetic scene'.")
        st.stop()
    img = np.asarray(Image.open(up).convert("RGB").resize((A.S, A.S)), dtype=np.float32) / 255.0
    depth = np.zeros((A.S, A.S), np.float32)
    lab = None
    st.caption("No depth supplied: slope detection is unreliable and the model has never seen real images. Output is demonstration only.")

st.sidebar.markdown("---")
st.sidebar.subheader("Perception Thresholds")
thr = st.sidebar.slider("Hazard probability threshold", 0.05, 0.60, 0.25, 0.01)
ut = st.sidebar.slider("Entropy threshold", 0.10, 0.80, 0.35, 0.01)
et = st.sidebar.slider("Disagreement threshold", 0.01, 0.20, 0.05, 0.01)

st.sidebar.markdown("---")
st.sidebar.subheader("Planner Parameters (Chapter 48)")
cell_size = st.sidebar.selectbox("Cell size (pixels)", [4, 6, 8, 12], index=1)
block_frac = st.sidebar.slider("Block fraction", 0.05, 0.50, 0.12, 0.01)
strict_truth = st.sidebar.checkbox("Strict truth rule (truth_bf = 0.0)", False)

if model is None:
    st.stop()

# Inference
P, ent, epi = model.predict(img, depth)
unsafe = A.unsafe_map(P, ent, epi, "C", thr=thr, ut=ut, et=et)

# Visualisation colormap
cm = np.array([[.55, .34, .24], [.9, .8, .4], [.2, .2, .2], [.8, .2, .2], [.2, .4, .9]])

panels = [
    ("Image", img, None),
    ("Prediction", cm[P.argmax(-1)], None),
    ("Uncertainty (Entropy)", ent, "magma"),
    ("Unsafe Map", unsafe, "gray")
]
if lab is not None:
    panels.insert(1, ("Truth (blue = novel)", cm[lab], None))

cols = st.columns(len(panels))
for c, (t, im, cmap) in zip(cols, panels):
    f, a = plt.subplots(figsize=(3.2, 3.2))
    a.imshow(im, cmap=cmap)
    a.axis("off")
    c.pyplot(f)
    c.caption(t)
    plt.close(f)

st.caption("Legend: Brown: Bedrock | Yellow: Sand | Dark Grey: Rock | Red: Steep Slope | Blue: Novel (OOD)")

# Navigation evaluation for this scene
n_c = A.S // cell_size
pred_blk = unsafe[:n_c * cell_size, :n_c * cell_size].reshape(n_c, cell_size, n_c, cell_size).mean((1, 3)) > block_frac

if lab is not None:
    h = np.isin(lab, list(A.HAZ))
    fs_rate = float((~unsafe & h).sum() / max(h.sum(), 1))
    tbf = 0.0 if strict_truth else block_frac
    truth_blk = h[:n_c * cell_size, :n_c * cell_size].reshape(n_c, cell_size, n_c, cell_size).mean((1, 3)) > tbf
    
    # Path planning
    path = A.astar(pred_blk, [(i, 0) for i in range(n_c)], n_c - 1)
    if path is None:
        nav_status = "No Path Found (Refused)"
    elif any(truth_blk[q] for q in path):
        nav_status = "Collision with Hazard"
    else:
        nav_status = f"Success (Path Length: {len(path)} cells)"
    
    m_col1, m_col2, m_col3 = st.columns(3)
    m_col1.metric("False-safe rate (hazard marked safe)", f"{fs_rate:.1%}")
    m_col2.metric("Planner Outcome", nav_status)
    m_col3.metric("Unsafe Area Share", f"{unsafe.mean():.1%}")
else:
    path = A.astar(pred_blk, [(i, 0) for i in range(n_c)], n_c - 1)
    nav_status = "Path found" if path else "No path found"
    st.info(f"Uploaded Image Demonstration: {nav_status} across coarse grid ({n_c}x{n_c} cells). Ground truth evaluation unavailable.")

# Chapter 37 / Task 10: Coverage and Risk Plot
try:
    with open("experiments_results.json", "r") as f:
        exp_data = json.load(f)
    if "E5" in exp_data:
        st.subheader("Coverage versus Risk (Experiment E5)")
        e5_data = exp_data["E5"]
        covs = [x["coverage"] for x in e5_data]
        errs = [x["error_on_accepted"] for x in e5_data]
        
        fig_e5, ax_e5 = plt.subplots(figsize=(6, 3))
        ax_e5.plot(covs, errs, marker='o', color='crimson', label="Accepted Error vs Coverage")
        ax_e5.set_xlabel("Coverage (Fraction of Pixels Accepted)")
        ax_e5.set_ylabel("Error on Accepted Pixels")
        ax_e5.set_title("Coverage vs Risk Curve (Synthetic Data)")
        ax_e5.grid(True, linestyle="--", alpha=0.5)
        ax_e5.legend()
        st.pyplot(fig_e5)
        plt.close(fig_e5)
except Exception:
    pass

st.subheader("Structured Observation Record")
obs = A.observation_record(
    scene_id=f"scene_{mode[:3]}_{seed if mode.startswith('Gen') else 'upload'}",
    P=P, ent=ent, epi=epi, unsafe=unsafe
)
st.json(obs)

st.markdown("---")
st.markdown("**APPS Metadata** | Model: `apps01_ensemble_5` | Feature Version: `v1.0` | Taxonomy: `v1.0` | Config: `cfg-0.1` | Environment: Python 3.14 / NumPy / SciPy / scikit-learn")
