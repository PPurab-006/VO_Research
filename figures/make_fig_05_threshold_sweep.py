#!/usr/bin/env python3
"""
Fig 5: Gating Yaw-Rate Threshold Sensitivity & Threshold Sweep (F9).

Panel (a): RAW Pose-Loss Rate (%) vs Binned |Yaw Rate| (0 to >50 deg/s) with 95% bootstrap CIs,
           including an aligned frame-count strip below.
Panel (b): Full Gating Threshold Sweep (5, 10, 15, 20, 30, 45 deg/s) showing per-run points (n=3)
           and mean trend for Valid Pose % and ATE RMSE (m). RAW baseline mean is plotted
           as a horizontal reference line.

Outputs:
- figures/out/fig_05_threshold_sweep.png
- figures/out/fig_05_threshold_sweep.pdf
- figures/data/fig_05_threshold_sweep.csv
- figures/captions/fig_05.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_ALT, COLOR_GRAY

BINNED_CSV = REPO_ROOT / "results" / "analysis" / "yaw_rate_binned_poseloss.csv"
SWEEP_CSV = REPO_ROOT / "results" / "analysis" / "threshold_sweep.csv"

def main():
    setup_style()
    df_binned = pd.read_csv(BINNED_CSV)
    df_sweep = pd.read_csv(SWEEP_CSV)

    fig = plt.figure(figsize=(7.0, 3.8))
    
    # Subplot grid: left has main plot + frame count strip, right has threshold sweep
    gs = fig.add_gridspec(2, 2, height_ratios=[4, 1], hspace=0.35, wspace=0.3)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1_strip = fig.add_subplot(gs[1, 0], sharex=ax1)
    ax2 = fig.add_subplot(gs[:, 1])

    # ------------------------------------------------------------
    # Panel (a): Binned Pose-Loss Rate vs Yaw Rate + Frame Count Strip
    # ------------------------------------------------------------
    df_pooled = df_binned[df_binned["run"] == "pooled"].reset_index(drop=True)
    bins_labels = ["0-5", "5-10", "10-15", "15-20", "20-30", "30-50", ">50"]
    
    loss_rates = df_pooled["raw_pose_loss_pct"].values
    ci_lo = df_pooled["ci_95_lo"].values
    ci_hi = df_pooled["ci_95_hi"].values
    yerr = [loss_rates - ci_lo, ci_hi - loss_rates]

    x_indices = np.arange(len(bins_labels))
    ax1.errorbar(x_indices, loss_rates, yerr=yerr, fmt="o-", color=COLOR_RAW, ecolor=COLOR_RAW,
                 capsize=4, capthick=1.2, lw=1.5, ms=5, label="RAW Failure Rate (%)")

    ax1.axvline(2.5, color="red", linestyle="--", lw=1.2, alpha=0.8, label="Chosen Thresh (15 deg/s)")
    ax1.set_ylabel("RAW Pose-Loss Rate (%)")
    ax1.set_title("(a) Pose Loss vs Yaw Rate Bins", fontsize=9.5)
    ax1.legend(loc="upper left", fontsize=7.5, frameon=True, facecolor="white", framealpha=0.9)
    ax1.set_ylim(0, 15)
    plt.setp(ax1.get_xticklabels(), visible=False)

    # Frame count strip
    frame_counts = df_pooled["n_frames"].values
    ax1_strip.bar(x_indices, frame_counts, color=COLOR_GRAY, alpha=0.6, width=0.5)
    ax1_strip.set_xticks(x_indices)
    ax1_strip.set_xticklabels(bins_labels, rotation=0, fontsize=8.0)
    ax1_strip.set_xlabel("Abs. Yaw Rate Bin $|\\dot{\\psi}|$ (deg/s)", fontsize=9.0)
    ax1_strip.set_ylabel("Frames", fontsize=8.0)
    ax1_strip.set_ylim(0, 750)
    
    for x, cnt in zip(x_indices, frame_counts):
        ax1_strip.text(x, cnt + 20, str(cnt), ha="center", va="bottom", fontsize=7.0, color="#333333")

    # ------------------------------------------------------------
    # Panel (b): Full Threshold Sweep (Valid Pose % & ATE RMSE)
    # ------------------------------------------------------------
    thresh_unique = sorted(df_sweep["gate_thresh_deg"].unique())
    raw_valid_mean = 93.04
    raw_ate_mean = 3.3206

    ax2_left = ax2
    ax2_right = ax2.twinx()

    # Per-run points
    sns.stripplot(data=df_sweep, x="gate_thresh_deg", y="valid_pose_pct", ax=ax2_left,
                  color=COLOR_GATED, alpha=0.6, jitter=0.1, size=5, label="Valid Pose % (n=3)")
    
    mean_valid = df_sweep.groupby("gate_thresh_deg")["valid_pose_pct"].mean().values
    x_positions = np.arange(len(thresh_unique))
    ax2_left.plot(x_positions, mean_valid, color=COLOR_GATED, marker="o", lw=1.5, linestyle="-", label="Valid % Mean")
    ax2_left.axhline(raw_valid_mean, color=COLOR_RAW, linestyle=":", lw=1.3, label="RAW Valid % (93.04%)")

    sns.stripplot(data=df_sweep, x="gate_thresh_deg", y="ate_rmse", ax=ax2_right,
                  color=COLOR_ALT, alpha=0.6, jitter=0.1, marker="s", size=5, label="ATE RMSE (n=3)")
    
    mean_ate = df_sweep.groupby("gate_thresh_deg")["ate_rmse"].mean().values
    ax2_right.plot(x_positions, mean_ate, color=COLOR_ALT, marker="s", lw=1.5, linestyle="--", label="ATE RMSE Mean")
    ax2_right.axhline(raw_ate_mean, color=COLOR_GRAY, linestyle="--", lw=1.2, label="RAW ATE Mean (3.32m)")

    ax2_left.set_xlabel("Gating Yaw-Rate Threshold $\\tau$ (deg/s)")
    ax2_left.set_ylabel("Valid Pose Fraction (%)", color=COLOR_GATED)
    ax2_right.set_ylabel("ATE RMSE (m)", color=COLOR_ALT)
    ax2.set_title("(b) Gating Threshold Sweep (F9 R1..R3)", fontsize=9.5)
    ax2_left.set_ylim(85, 96)
    ax2_right.set_ylim(2.2, 4.0)

    lines_left, labels_left = ax2_left.get_legend_handles_labels()
    lines_right, labels_right = ax2_right.get_legend_handles_labels()
    unique_legend = {}
    for l, lab in zip(lines_left + lines_right, labels_left + labels_right):
        if lab not in unique_legend and "(n=3)" not in lab:
            unique_legend[lab] = l
            
    ax2_left.legend(unique_legend.values(), unique_legend.keys(), loc="lower left", fontsize=7.0, frameon=True, facecolor="white", framealpha=0.9)

    # Combine sidecar CSV data
    df_sidecar = df_sweep[["gate_thresh_deg", "repeat", "valid_pose_pct", "ate_rmse", "scale_factor", "meter_rpe", "norm_rpe"]].copy()

    caption_md = (
        "**Figure 5: Gating yaw-rate threshold sensitivity & 18-run sweep.** "
        "(a) RAW pose-loss rate (%) across binned yaw-rate ranges with 95% bootstrap CIs and frame-count strip. "
        "(b) Gating threshold sweep (5, 10, 15, 20, 30, 45 deg/s). No threshold level is statistically or practically "
        "distinguishable on ATE RMSE at n=3 (p > 0.05), whereas valid pose recovery fraction falls monotonically "
        "with increasing threshold (from 93.00% at 5 deg/s to 90.35% at 45 deg/s)."
    )

    save_fig_and_sidecar(fig, "fig_05_threshold_sweep", df_sidecar, caption_md)

if __name__ == "__main__":
    main()
