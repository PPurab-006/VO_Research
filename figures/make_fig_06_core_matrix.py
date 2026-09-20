#!/usr/bin/env python3
"""
Fig 6: Primary Core Matrix Comparison (8 Core Families, 24 Runs Total).

3 rows of subplots:
- Row 1: Tracking Continuity (Valid Pose Fraction %)
- Row 2: Trajectory Accuracy (ATE RMSE in meters)
- Row 3: Scale-Invariant Step Error (Normalized RPE)

Paired lines connecting RAW and GATED for each run (n=3 per family).
Annotated with k/3 (runs GATED is better) and Wilcoxon p-value from paired_stats.csv.
phase2a_ cells (F5_L2 and F9_L2) are explicitly marked with asterisks (*).

Outputs:
- figures/out/fig_06_core_matrix.png
- figures/out/fig_06_core_matrix.pdf
- figures/data/fig_06_core_matrix.csv
- figures/captions/fig_06.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_GRAY

PER_RUN_CSV = REPO_ROOT / "results" / "analysis" / "per_run_metrics.csv"
PAIRED_CSV = REPO_ROOT / "results" / "analysis" / "paired_stats.csv"

CORE_FAMILIES = ["F1_L2", "F2_L2", "F4_L2", "F5_L2*", "F6_L2", "F9_L2*", "F10_L3", "F11_L2"]
CORE_FAMILIES_RAW = ["F1_L2", "F2_L2", "F4_L2", "F5_L2", "F6_L2", "F9_L2", "F10_L3", "F11_L2"]

def main():
    setup_style()
    df_per_run = pd.read_csv(PER_RUN_CSV)
    df_paired = pd.read_csv(PAIRED_CSV)

    fig, axes = plt.subplots(3, 1, figsize=(7.0, 7.5), sharex=True)
    ax_valid, ax_ate, ax_rpe = axes

    df_core = df_per_run[df_per_run["mechanism"].isin(["RAW", "EIS-GATED"])].copy()

    x_positions = np.arange(len(CORE_FAMILIES))

    # Metric configurations: (axis, metric_column, y_label, title, is_higher_better)
    metrics_config = [
        (ax_valid, "valid_pose_pct", "Valid Pose Fraction (%)", "(a) Tracking Continuity (Valid Pose %)", True),
        (ax_ate, "ate_rmse", "ATE RMSE (m)", "(b) Trajectory Accuracy (ATE RMSE)", False),
        (ax_rpe, "rpe_t_norm", "Normalized RPE (unit step err)", "(c) Scale-Invariant Step Error (Normalized RPE)", False),
    ]

    for ax, metric_col, ylabel, title, higher_better in metrics_config:
        for x_idx, (fam_clean, fam_raw) in enumerate(zip(CORE_FAMILIES, CORE_FAMILIES_RAW)):
            df_fam = df_core[df_core["family"] == fam_raw]
            
            # Fetch paired stats
            p_sub = df_paired[(df_paired["family"] == fam_raw) & (df_paired["metric"] == metric_col)]
            if not p_sub.empty:
                k_better = p_sub["n_gated_better"].values[0]
                p_val = p_sub["paired_t_p"].values[0]
                stat_str = f"k={k_better}/3\np={p_val:.4f}"
            else:
                stat_str = ""

            # Plot paired lines for runs 1, 2, 3
            for r in [1, 2, 3]:
                r_raw = df_fam[(df_fam["run"] == r) & (df_fam["mechanism"] == "RAW")]
                r_gated = df_fam[(df_fam["run"] == r) & (df_fam["mechanism"] == "EIS-GATED")]
                
                if not r_raw.empty and not r_gated.empty:
                    val_raw = r_raw[metric_col].values[0]
                    val_gated = r_gated[metric_col].values[0]

                    # Offset positions for paired points
                    x_r = x_idx - 0.15
                    x_g = x_idx + 0.15
                    
                    line_color = COLOR_GATED if (val_gated > val_raw if higher_better else val_gated < val_raw) else COLOR_RAW
                    ax.plot([x_r, x_g], [val_raw, val_gated], color=line_color, alpha=0.5, lw=1.1)
                    ax.scatter(x_r, val_raw, color=COLOR_RAW, s=25, zorder=3)
                    ax.scatter(x_g, val_gated, color=COLOR_GATED, s=25, zorder=3)

            # Family mean bars / indicators
            m_raw = df_fam[df_fam["mechanism"] == "RAW"][metric_col].mean()
            m_gated = df_fam[df_fam["mechanism"] == "EIS-GATED"][metric_col].mean()
            ax.scatter(x_idx - 0.15, m_raw, color=COLOR_RAW, marker="_", s=140, lw=3.0, zorder=4)
            ax.scatter(x_idx + 0.15, m_gated, color=COLOR_GATED, marker="_", s=140, lw=3.0, zorder=4)

            # Annotate k/3 and p-value above family
            y_max_fam = max(df_fam[metric_col].max(), m_raw, m_gated)
            y_offset = (ax.get_ylim()[1] - ax.get_ylim()[0]) * 0.08 if ax.get_ylim()[1] > 0 else 0.1
            ax.text(x_idx, y_max_fam + y_offset * 0.15, stat_str, ha="center", va="bottom", fontsize=7.0, color="#333333", weight="bold")

        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=9.5)

    ax_rpe.set_xticks(x_positions)
    ax_rpe.set_xticklabels(CORE_FAMILIES, fontsize=9.0, weight="bold")
    ax_rpe.set_xlabel("Core Benchmark Families (* indicates phase2a_ provenance)", fontsize=9.5)

    # Global legend
    ax_valid.scatter([], [], color=COLOR_RAW, s=30, label="RAW Run (n=3)")
    ax_valid.scatter([], [], color=COLOR_GATED, s=30, label="EIS-GATED Run (n=3)")
    ax_valid.scatter([], [], color=COLOR_RAW, marker="_", s=100, lw=2.5, label="RAW Family Mean")
    ax_valid.scatter([], [], color=COLOR_GATED, marker="_", s=100, lw=2.5, label="EIS-GATED Family Mean")
    ax_valid.legend(loc="lower left", fontsize=7.5, ncol=2, frameon=True, facecolor="white", framealpha=0.9)

    plt.tight_layout()

    # Save sidecar CSV
    df_sidecar = df_paired[df_paired["family"].isin(CORE_FAMILIES_RAW)].copy()
    
    caption_md = (
        "**Figure 6: Primary 8-core matrix paired comparison.** "
        "Shows (a) valid pose fraction (%), (b) ATE RMSE (m), and (c) normalized RPE for RAW vs EIS-GATED "
        "across all 24 core runs (8 families $\\times$ 3 repeats). Each run is connected by a paired line. "
        "Each family is annotated with $k/3$ (number of runs GATED improves over RAW) and paired t-test $p$-value read from `paired_stats.csv` "
        "(e.g., F9 ATE $p=0.5304$, F10 ATE $p=0.1527$; discrete Wilcoxon rank-sum values $0.75$/$0.25$ arise from $n=3$ discrete rank combinations). "
        "Asterisks (*) mark `phase2a_` dataset provenance (F5_L2* and F9_L2*)."
    )

    save_fig_and_sidecar(fig, "fig_06_core_matrix", df_sidecar, caption_md)


if __name__ == "__main__":
    main()
