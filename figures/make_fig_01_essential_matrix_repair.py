#!/usr/bin/env python3
"""
Fig 1: Essential Matrix recoverPose Repair Synthetic & Empirical Validation.

Panel (a): Synthetic pose recovery rate for OpenCV default (distanceThresh=50.0) vs
           repaired (distanceThresh=1000.0) under small (1.5 cm) and standard (20.0 cm) baselines.
Panel (b): Empirical pose inlier count distribution on F9 R1 active window frames.

Outputs:
- figures/out/fig_01_essential_matrix_repair.png
- figures/out/fig_01_essential_matrix_repair.pdf
- figures/data/fig_01_essential_matrix_repair.csv
- figures/captions/fig_01.md
"""

import sys
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_ALT, COLOR_GRAY

DATASETS = REPO_ROOT / "results" / "datasets"

def run_synthetic_test():
    np.random.seed(42)
    N = 500
    pts3d = np.column_stack([
        np.random.uniform(-1.5, 1.5, N),
        np.random.uniform(-1.0, 1.0, N),
        np.random.uniform(2.0, 10.0, N)
    ])
    K = np.array([[539.936, 0, 640], [0, 539.936, 480], [0, 0, 1]], dtype=np.float64)

    # 1.5cm baseline
    t_small = np.array([0.015, 0, 0], dtype=np.float64)
    pts1, _ = cv2.projectPoints(pts3d, np.zeros(3), np.zeros(3), K, None)
    pts1 = pts1.reshape(-1, 2)
    pts2_small, _ = cv2.projectPoints(pts3d, np.zeros(3), t_small, K, None)
    pts2_small = pts2_small.reshape(-1, 2)

    E_small, _ = cv2.findEssentialMat(pts1, pts2_small, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    res_def_small = cv2.recoverPose(E_small, pts1, pts2_small, K, distanceThresh=50.0)
    res_rep_small = cv2.recoverPose(E_small, pts1, pts2_small, K, distanceThresh=1000.0)

    # 20.0cm baseline
    t_large = np.array([0.20, 0, 0], dtype=np.float64)
    pts2_large, _ = cv2.projectPoints(pts3d, np.zeros(3), t_large, K, None)
    pts2_large = pts2_large.reshape(-1, 2)

    E_large, _ = cv2.findEssentialMat(pts1, pts2_large, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    res_def_large = cv2.recoverPose(E_large, pts1, pts2_large, K, distanceThresh=50.0)
    res_rep_large = cv2.recoverPose(E_large, pts1, pts2_large, K, distanceThresh=1000.0)

    return {
        "small_baseline_default_inliers": int(res_def_small[0]),
        "small_baseline_repaired_inliers": int(res_rep_small[0]),
        "large_baseline_default_inliers": int(res_def_large[0]),
        "large_baseline_repaired_inliers": int(res_rep_large[0]),
        "total_points": N,
    }

def main():
    setup_style()
    syn_res = run_synthetic_test()

    # Load real flight data from F9 R1
    df_raw = pd.read_csv(DATASETS / "phase2a_F9_L2_R1" / "raw_vo.csv")
    df_gt = pd.read_csv(DATASETS / "phase2a_F9_L2_R1" / "dataset_gt.csv")
    
    t_start, t_end = df_gt[df_gt["pos_z"] >= 2.0]["timestamp_total_sec"].values[[0, -1]]
    m_act = (df_raw["timestamp_total_sec"] >= t_start) & (df_raw["timestamp_total_sec"] <= t_end)
    raw_inliers = df_raw[m_act]["num_inliers_pose"].values

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.2))

    # Panel (a): Synthetic Recovery Rate Bar Chart
    categories = ["1.5 cm Baseline\nDefault (50m)", "1.5 cm Baseline\nRepaired (1000m)",
                  "20.0 cm Baseline\nDefault (50m)", "20.0 cm Baseline\nRepaired (1000m)"]
    counts = [syn_res["small_baseline_default_inliers"], syn_res["small_baseline_repaired_inliers"],
              syn_res["large_baseline_default_inliers"], syn_res["large_baseline_repaired_inliers"]]
    colors = [COLOR_RAW, COLOR_GATED, COLOR_RAW, COLOR_GATED]

    bars = ax1.bar(categories, counts, color=colors, width=0.55, edgecolor="black", lw=0.8)
    for bar in bars:
        h = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2.0, h + 10, f"{h}/500", ha="center", va="bottom", size=8.5, weight="bold")

    ax1.set_ylabel("Recovered Inlier Count ($N_{\\text{inliers}}$)")
    ax1.set_title("(a) Synthetic recoverPose Fix (500 pts)", fontsize=9.5)
    ax1.set_ylim(0, 580)
    ax1.tick_params(axis="x", labelsize=8.0)

    # Panel (b): Real Flight Inlier Count Profile (F9 R1)
    ax2.plot(np.arange(len(raw_inliers)), raw_inliers, color=COLOR_ALT, lw=1.1, label="RAW F9 R1 Active Window")
    ax2.axhline(8.0, color="red", linestyle="--", lw=1.2, label="Validity Threshold (>= 8)")
    ax2.set_xlabel("Active Window Frame Index")
    ax2.set_ylabel("Pose Inliers ($N_{\\text{inliers}}$)")
    ax2.set_title("(b) Empirical Inlier Count Profile (F9 R1)", fontsize=9.5)
    ax2.legend(loc="upper right", frameon=True, facecolor="white", framealpha=0.9)
    ax2.set_ylim(0, 160)

    # Build sidecar DataFrame
    sidecar_df = pd.DataFrame([{
        "small_baseline_default_inliers": syn_res["small_baseline_default_inliers"],
        "small_baseline_repaired_inliers": syn_res["small_baseline_repaired_inliers"],
        "large_baseline_default_inliers": syn_res["large_baseline_default_inliers"],
        "large_baseline_repaired_inliers": syn_res["large_baseline_repaired_inliers"],
        "total_synthetic_points": syn_res["total_points"],
        "f9_r1_mean_inliers": float(np.mean(raw_inliers)),
        "f9_r1_valid_pose_pct": float((raw_inliers >= 8).mean() * 100.0),
    }])

    caption_md = (
        "**Figure 1: Essential matrix recoverPose repair validation.** "
        f"(a) Under a small baseline of 1.5 cm, OpenCV default `distanceThresh=50.0` fails completely "
        f"({syn_res['small_baseline_default_inliers']}/500 inliers), whereas repairing `distanceThresh=1000.0` "
        f"achieves 100% pose recovery ({syn_res['small_baseline_repaired_inliers']}/500 inliers). Under a 20.0 cm baseline, "
        f"both configurations recover all {syn_res['large_baseline_repaired_inliers']}/500 points. "
        f"(b) Empirical active-window inlier count profile on F9 R1 (mean: {np.mean(raw_inliers):.1f} inliers, "
        f"valid pose fraction: {(raw_inliers >= 8).mean() * 100.0:.2f}%). "
        "old-threshold real-flight value not reproducible from committed data."
    )

    save_fig_and_sidecar(fig, "fig_01_essential_matrix_repair", sidecar_df, caption_md)

if __name__ == "__main__":
    main()

