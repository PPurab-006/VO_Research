#!/usr/bin/env python3
"""
Fig 8: Delayed Triangulation (DT) Artifact Analysis (F6_L2, F9_L2, F10_L3).

Panel (a): Full-Window Normalized RPE vs Valid-Intersection Normalized RPE for EIS-GATED vs DELAYED-TRI,
           demonstrating that DT's apparent low overall normRPE is an artifact of severe frame loss.
Panel (b): Valid Pose Fraction (%) for EIS-GATED vs DELAYED-TRI across active families.

Outputs:
- figures/out/fig_08_dt_artifact.png
- figures/out/fig_08_dt_artifact.pdf
- figures/data/fig_08_dt_artifact.csv
- figures/captions/fig_08.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar, COLOR_GATED, COLOR_ALT, COLOR_GRAY

DT_ARTIFACT_CSV = REPO_ROOT / "results" / "analysis" / "dt_artifact.csv"

def main():
    setup_style()
    df_dt = pd.read_csv(DT_ARTIFACT_CSV)

    # Filter active families: F6_L2, F9_L2, F10_L3
    df_dt["family_clean"] = df_dt["family"].replace({"F6_L2": "F6_L2", "F9_L2": "F9_L2", "F10_L3": "F10_L3"})
    fam_order = ["F6_L2", "F9_L2", "F10_L3"]
    df_dt = df_dt[df_dt["family_clean"].isin(fam_order)].copy()
    df_dt["family_clean"] = pd.Categorical(df_dt["family_clean"], categories=fam_order, ordered=True)
    df_dt = df_dt.sort_values("family_clean")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.4))

    # Panel (a): Full-Window vs Valid-Intersection Norm RPE
    # Group by family and mechanism
    means_all = df_dt.groupby(["family_clean", "mechanism"])[["rpe_t_norm", "rpe_t_norm_valid"]].mean().reset_index()

    x_positions = np.arange(len(fam_order))
    width = 0.35

    gated_full = means_all[means_all["mechanism"] == "EIS-GATED"]["rpe_t_norm"].values
    gated_valid = means_all[means_all["mechanism"] == "EIS-GATED"]["rpe_t_norm_valid"].values
    dt_full = means_all[means_all["mechanism"] == "DELAYED-TRI"]["rpe_t_norm"].values
    dt_valid = means_all[means_all["mechanism"] == "DELAYED-TRI"]["rpe_t_norm_valid"].values

    ax1.bar(x_positions - width/2, dt_full, width, label="DT Full-Window NormRPE", color=COLOR_ALT, alpha=0.5, edgecolor="black")
    ax1.bar(x_positions - width/2, dt_valid, width, label="DT Valid-Frames NormRPE", color=COLOR_ALT, hatch="//", alpha=0.9, edgecolor="black")

    ax1.bar(x_positions + width/2, gated_full, width, label="GATED Full-Window NormRPE", color=COLOR_GATED, alpha=0.5, edgecolor="black")
    ax1.bar(x_positions + width/2, gated_valid, width, label="GATED Valid-Frames NormRPE", color=COLOR_GATED, hatch="//", alpha=0.9, edgecolor="black")

    ax1.set_xticks(x_positions)
    ax1.set_xticklabels(fam_order, fontsize=9.0)
    ax1.set_xlabel("Active Benchmark Family")
    ax1.set_ylabel("Normalized RPE (unit step error)")
    ax1.set_title("(a) DT Artifact: Full vs Valid NormRPE", fontsize=9.5)
    ax1.legend(loc="upper right", fontsize=6.5, frameon=True, facecolor="white", framealpha=0.9)
    ax1.set_ylim(0, 1.4)

    # Panel (b): Valid Pose Fraction (%) Drop
    sns.stripplot(data=df_dt, x="family_clean", y="valid_pose_pct", hue="mechanism",
                  palette={"EIS-GATED": COLOR_GATED, "DELAYED-TRI": COLOR_ALT},
                  dodge=True, alpha=0.85, jitter=0.12, size=5, ax=ax2)

    means_v = df_dt.groupby(["family_clean", "mechanism"])["valid_pose_pct"].mean().reset_index()
    for fam_idx, fam in enumerate(fam_order):
        g_val = means_v[(means_v["family_clean"] == fam) & (means_v["mechanism"] == "EIS-GATED")]["valid_pose_pct"].values[0]
        dt_val = means_v[(means_v["family_clean"] == fam) & (means_v["mechanism"] == "DELAYED-TRI")]["valid_pose_pct"].values[0]
        ax2.plot([fam_idx - 0.2, fam_idx + 0.2], [g_val, dt_val], color=COLOR_GRAY, linestyle=":", lw=1.0, zorder=1)
        ax2.scatter(fam_idx - 0.2, g_val, color=COLOR_GATED, marker="_", s=100, lw=2.5, zorder=3)
        ax2.scatter(fam_idx + 0.2, dt_val, color=COLOR_ALT, marker="_", s=100, lw=2.5, zorder=3)

    ax2.set_xlabel("Active Benchmark Family")
    ax2.set_ylabel("Valid Pose Fraction (%)")
    ax2.set_title("(b) Tracking Loss Penalty (Active Cells)", fontsize=9.5)
    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles[:2], labels[:2], loc="lower left", frameon=True, facecolor="white", framealpha=0.9)
    ax2.set_ylim(30, 100)

    df_sidecar = df_dt.copy()

    caption_md = (
        "**Figure 8: Delayed Triangulation (DT) metric artifact analysis for F6_L2, F9_L2, and F10_L3.** "
        "(a) Shows that DT's apparent low overall full-window normalized RPE is an artifact of severe frame loss. "
        "When evaluated on valid frames alone (`rpe_t_norm_valid`), DT step error is comparable or worse than GATED. "
        "(b) Tracking validity drops precipitously under DT (e.g. F6_L2 validity falls from 92.3% to 43.0%)."
    )

    save_fig_and_sidecar(fig, "fig_08_dt_artifact", df_sidecar, caption_md)

if __name__ == "__main__":
    main()
