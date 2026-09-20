#!/usr/bin/env python3
"""
Fig 11: Exploratory Benchmark Families Breakdown (HOVER_L0, F3_L2, F7_L2, F8_L2 ONLY).

Panel (a): ATE RMSE (m) for RAW vs EIS-GATED across exploratory trajectory conditions (n=3 runs).
Panel (b): Valid Pose Fraction (%) for RAW vs EIS-GATED across exploratory trajectory conditions (n=3 runs).

Outputs:
- figures/out/fig_11_exploratory_families.png
- figures/out/fig_11_exploratory_families.pdf
- figures/data/fig_11_exploratory_families.csv
- figures/captions/fig_11.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_GRAY

EXPLORATORY_CSV = REPO_ROOT / "results" / "analysis" / "per_run_metrics_exploratory.csv"

def main():
    setup_style()
    df_exp = pd.read_csv(EXPLORATORY_CSV)

    df_exp_filt = df_exp[df_exp["mechanism"].isin(["RAW", "EIS-GATED"])].copy()
    
    # Exact exploratory families from CSV: HOVER_L0, F3_L2, F7_L2, F8_L2 ONLY
    fam_order = ["HOVER_L0", "F3_L2", "F7_L2", "F8_L2"]
    df_exp_filt = df_exp_filt[df_exp_filt["family"].isin(fam_order)].copy()
    df_exp_filt["family"] = pd.Categorical(df_exp_filt["family"], categories=fam_order, ordered=True)
    df_exp_filt = df_exp_filt.sort_values("family")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.4))

    # Panel (a): ATE RMSE (m)
    sns.stripplot(data=df_exp_filt, x="family", y="ate_rmse", hue="mechanism",
                  palette={"RAW": COLOR_RAW, "EIS-GATED": COLOR_GATED},
                  dodge=True, alpha=0.85, jitter=0.15, size=5, ax=ax1)

    means = df_exp_filt.groupby(["family", "mechanism"])["ate_rmse"].mean().reset_index()
    for fam_idx, fam in enumerate(fam_order):
        raw_rows = means[(means["family"] == fam) & (means["mechanism"] == "RAW")]
        gated_rows = means[(means["family"] == fam) & (means["mechanism"] == "EIS-GATED")]
        if not raw_rows.empty and not gated_rows.empty:
            raw_m = raw_rows["ate_rmse"].values[0]
            gated_m = gated_rows["ate_rmse"].values[0]
            ax1.plot([fam_idx - 0.2, fam_idx + 0.2], [raw_m, gated_m], color=COLOR_GRAY, linestyle=":", lw=1.0, zorder=1)
            ax1.scatter(fam_idx - 0.2, raw_m, color=COLOR_RAW, marker="_", s=100, lw=2.5, zorder=3)
            ax1.scatter(fam_idx + 0.2, gated_m, color=COLOR_GATED, marker="_", s=100, lw=2.5, zorder=3)

    ax1.set_xlabel("Exploratory Trajectory Condition")
    ax1.set_ylabel("ATE RMSE (m)")
    ax1.set_title("(a) Trajectory Accuracy (ATE RMSE)", fontsize=9.5)
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles[:2], labels[:2], loc="upper left", frameon=True, facecolor="white", framealpha=0.9)

    # Panel (b): Valid Pose Fraction (%)
    sns.stripplot(data=df_exp_filt, x="family", y="valid_pose_pct", hue="mechanism",
                  palette={"RAW": COLOR_RAW, "EIS-GATED": COLOR_GATED},
                  dodge=True, alpha=0.85, jitter=0.15, size=5, ax=ax2)

    means_v = df_exp_filt.groupby(["family", "mechanism"])["valid_pose_pct"].mean().reset_index()
    for fam_idx, fam in enumerate(fam_order):
        raw_rows = means_v[(means_v["family"] == fam) & (means_v["mechanism"] == "RAW")]
        gated_rows = means_v[(means_v["family"] == fam) & (means_v["mechanism"] == "EIS-GATED")]
        if not raw_rows.empty and not gated_rows.empty:
            raw_v = raw_rows["valid_pose_pct"].values[0]
            gated_v = gated_rows["valid_pose_pct"].values[0]
            ax2.plot([fam_idx - 0.2, fam_idx + 0.2], [raw_v, gated_v], color=COLOR_GRAY, linestyle=":", lw=1.0, zorder=1)
            ax2.scatter(fam_idx - 0.2, raw_v, color=COLOR_RAW, marker="_", s=100, lw=2.5, zorder=3)
            ax2.scatter(fam_idx + 0.2, gated_v, color=COLOR_GATED, marker="_", s=100, lw=2.5, zorder=3)

    ax2.set_xlabel("Exploratory Trajectory Condition")
    ax2.set_ylabel("Valid Pose Fraction (%)")
    ax2.set_title("(b) Tracking Continuity (Valid Pose %)", fontsize=9.5)
    handles2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(handles2[:2], labels2[:2], loc="lower left", frameon=True, facecolor="white", framealpha=0.9)

    df_sidecar = df_exp_filt.copy()

    caption_md = (
        "**Figure 11: Exploratory benchmark families breakdown (HOVER_L0, F3_L2, F7_L2, F8_L2).** "
        "(a) Trajectory accuracy (ATE RMSE in meters) for RAW vs EIS-GATED across 4 exploratory families (n=3 runs). "
        "(b) Valid pose fraction (%) for RAW vs EIS-GATED across exploratory families. "
        "For low-parallax hover flights (HOVER_L0, n=3), validity remains high for both mechanisms and EIS-GATED does not improve tracking continuity compared to RAW."
    )

    save_fig_and_sidecar(fig, "fig_11_exploratory_families", df_sidecar, caption_md)

if __name__ == "__main__":
    main()

