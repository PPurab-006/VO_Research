#!/usr/bin/env python3
"""
Fig 4: F9 EIS Derotation Progression Ladder (RAW, FIXED, INCREMENTAL, NULL, GATED).

Panel (a): Valid Pose Fraction (%) progression across derotation reference modes (n=3 runs).
Panel (b): ATE RMSE (m) progression across derotation reference modes (n=3 runs).

Outputs:
- figures/out/fig_04_f9_ladder.png
- figures/out/fig_04_f9_ladder.pdf
- figures/data/fig_04_f9_ladder.csv
- figures/captions/fig_04.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import (
    setup_style, save_fig_and_sidecar,
    COLOR_RAW, COLOR_GATED, COLOR_FIXED, COLOR_INCREMENTAL, COLOR_NULL, COLOR_GRAY
)

LADDER_CSV = REPO_ROOT / "results" / "analysis" / "f9_ladder.csv"

def main():
    setup_style()
    df_ladder = pd.read_csv(LADDER_CSV)

    modes_clean = ["RAW", "FIXED", "INCREMENTAL", "NULL", "GATED"]
    df_ladder["condition_clean"] = df_ladder["condition"].replace({
        "EIS_FIXED": "FIXED",
        "EIS_INCREMENTAL": "INCREMENTAL",
        "EIS_NULL": "NULL",
        "EIS_GATED": "GATED"
    })
    
    df_ladder["condition_clean"] = pd.Categorical(df_ladder["condition_clean"], categories=modes_clean, ordered=True)
    df_ladder = df_ladder.sort_values("condition_clean")

    palette = {
        "RAW": COLOR_RAW,
        "FIXED": COLOR_FIXED,
        "INCREMENTAL": COLOR_INCREMENTAL,
        "NULL": COLOR_NULL,
        "GATED": COLOR_GATED,
    }

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.4))

    # Panel (a): Valid Pose Fraction (%)
    sns.stripplot(data=df_ladder, x="condition_clean", y="valid_pose_pct", hue="condition_clean",
                  palette=palette, alpha=0.9, size=6, jitter=0.1, ax=ax1, legend=False)

    means_valid = df_ladder.groupby("condition_clean")["valid_pose_pct"].mean().values
    x_pos = np.arange(len(modes_clean))
    ax1.plot(x_pos, means_valid, color=COLOR_GRAY, linestyle="--", lw=1.2, zorder=1)
    
    for idx, (m, val) in enumerate(zip(modes_clean, means_valid)):
        ax1.scatter(idx, val, color=palette[m], marker="_", s=120, lw=2.5, zorder=3)

    ax1.set_xlabel("EIS Reference Mode Progression")
    ax1.set_ylabel("Valid Pose Fraction (%)")
    ax1.set_title("(a) Tracking Continuity Ladder", fontsize=9.5)
    ax1.set_ylim(55, 98)

    ax1.annotate("FIXED Failure\n(62.5% valid)", xy=(1, 62.5), xytext=(1.2, 70),
                 arrowprops=dict(arrowstyle="->", color=COLOR_FIXED, lw=1.2),
                 fontsize=8.0, color=COLOR_FIXED, weight="bold")

    # Panel (b): ATE RMSE (m)
    sns.stripplot(data=df_ladder, x="condition_clean", y="ate_rmse", hue="condition_clean",
                  palette=palette, alpha=0.9, size=6, jitter=0.1, ax=ax2, legend=False)

    means_ate = df_ladder.groupby("condition_clean")["ate_rmse"].mean().values
    ax2.plot(x_pos, means_ate, color=COLOR_GRAY, linestyle="--", lw=1.2, zorder=1)
    
    for idx, (m, val) in enumerate(zip(modes_clean, means_ate)):
        ax2.scatter(idx, val, color=palette[m], marker="_", s=120, lw=2.5, zorder=3)

    ax2.set_xlabel("EIS Reference Mode Progression")
    ax2.set_ylabel("ATE RMSE (m)")
    ax2.set_title("(b) Trajectory Accuracy Ladder", fontsize=9.5)
    ax2.set_ylim(2.2, 4.2)

    df_sidecar = df_ladder[["condition_clean", "run", "valid_pose_pct", "ate_rmse", "rpe_t_norm", "sim3_scale"]].copy()

    caption_md = (
        "**Figure 4: F9 EIS derotation progression ladder.** "
        "Shows valid pose fraction (%) and ATE RMSE (m) across 5 derotation modes: RAW, FIXED, INCREMENTAL, "
        "NULL, and GATED. FIXED mode causes severe tracking loss (62.51% valid vs 93.04% RAW), whereas "
        "INCREMENTAL (88.17%), NULL (93.28%), and GATED (92.14%) restore continuity."
    )

    save_fig_and_sidecar(fig, "fig_04_f9_ladder", df_sidecar, caption_md)

if __name__ == "__main__":
    main()
