#!/usr/bin/env python3
"""
Fig 12: System Architecture & Electronic Image Stabilization (EIS) Derotation Pipeline Diagram.

Conceptual block diagram visualizing the monocular VO pipeline with and without
EIS derotation (SLERP attitude interpolation + homography warping).

Outputs:
- figures/out/fig_12_system_diagram.png
- figures/out/fig_12_system_diagram.pdf
- figures/data/fig_12_system_diagram.csv
- figures/captions/fig_12.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_ALT, COLOR_GRAY

def main():
    setup_style()
    fig, ax = plt.subplots(figsize=(7.0, 3.2))
    ax.axis("off")

    # 1. Input Box
    ax.add_patch(patches.FancyBboxPatch((0.02, 0.55), 0.22, 0.38, boxstyle="round,pad=0.03", ec="#333333", fc="#f0f4f8", lw=1.5))
    ax.text(0.13, 0.86, "Camera & Telemetry", ha="center", va="center", weight="bold", size=10)
    ax.text(0.13, 0.74, "Monocular Images (30.3 Hz)\nIMU / GT Telemetry\nIntrinsic Matrix K", ha="center", va="center", size=8.5, color="#444444")

    # 2. RAW Pipeline Branch
    ax.add_patch(patches.FancyBboxPatch((0.35, 0.65), 0.28, 0.28, boxstyle="round,pad=0.03", ec=COLOR_RAW, fc="#fff5f0", lw=1.5))
    ax.text(0.49, 0.84, "RAW VO Pipeline", ha="center", va="center", weight="bold", color=COLOR_RAW, size=9.5)
    ax.text(0.49, 0.73, "1. KLT Feature Tracking\n2. 5-Pt Essential RANSAC\n3. recoverPose (SE3)", ha="center", va="center", size=8.5)

    # 3. EIS Derotation Block
    ax.add_patch(patches.FancyBboxPatch((0.35, 0.12), 0.28, 0.42, boxstyle="round,pad=0.03", ec=COLOR_GATED, fc="#f0fdf4", lw=1.5))
    ax.text(0.49, 0.46, "EIS Derotator Block", ha="center", va="center", weight="bold", color=COLOR_GATED, size=9.5)
    ax.text(0.49, 0.32, "• SLERP Attitude Interpolation\n• Relative Rotation R_rel\n• Homography H = K R_rel K^-1\n• Gating |yaw_rate| > thresh", ha="center", va="center", size=8.0)

    # 4. GATED VO Branch
    ax.add_patch(patches.FancyBboxPatch((0.70, 0.12), 0.26, 0.42, boxstyle="round,pad=0.03", ec=COLOR_GATED, fc="#f0fdf4", lw=1.5))
    ax.text(0.83, 0.46, "EIS-GATED VO", ha="center", va="center", weight="bold", color=COLOR_GATED, size=9.5)
    ax.text(0.83, 0.30, "Identical KLT & RANSAC\nEvaluated on Derotated\nVirtual Image Plane", ha="center", va="center", size=8.5)

    # Connectors
    arrow_props = dict(arrowstyle="->", lw=1.5, color="#333333")
    ax.annotate("", xy=(0.35, 0.79), xytext=(0.24, 0.79), arrowprops=arrow_props)
    ax.annotate("", xy=(0.35, 0.33), xytext=(0.24, 0.60), arrowprops=arrow_props)
    ax.annotate("", xy=(0.70, 0.33), xytext=(0.63, 0.33), arrowprops=arrow_props)

    ax.text(0.49, 0.02, "(a) Un-derotated Baseline", ha="center", va="center", size=9.0, weight="bold", color=COLOR_RAW)
    ax.text(0.83, 0.02, "(b) Synthetic Derotation", ha="center", va="center", size=9.0, weight="bold", color=COLOR_GATED)

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 0.95)

    df_sidecar = pd.DataFrame([{
        "camera_frequency_hz": 30.30,
        "intrinsic_fx": 539.936,
        "gating_threshold_degs": 15.0,
        "ransac_prob": 0.999,
        "recoverpose_distance_thresh_vo_units": 1000.0,
        "note": "Monocular scale is arbitrary; distanceThresh is specified in VO distance units.",
    }])

    caption_md = (
        "**Figure 12: System architecture of the monocular VO benchmark and synthetic derotation pipeline.** "
        "Monocular camera frames (measured 30.3 Hz from timestamps, $K_{f_x=539.9}$) and attitude telemetry are processed either via the un-derotated "
        "RAW VO pipeline or homography-derotated EIS-GATED pipeline ($H = K R_{\\text{rel}} K^{-1}$, gating threshold $15^\\circ/\\text{s}$, "
        "repaired `distanceThresh=1000.0` VO units, noting monocular scale is arbitrary)."
    )

    save_fig_and_sidecar(fig, "fig_12_system_diagram", df_sidecar, caption_md)

if __name__ == "__main__":
    main()

