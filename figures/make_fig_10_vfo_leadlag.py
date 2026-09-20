#!/usr/bin/env python3
"""
Fig 10: Visual Feature Odometry (VFO) Lead-Lag Correlation Matrices.

Panel (a): Heatmap of Pearson correlation r across 12 VFO predictors x Lags {0,1,2,3,5,10}
           against continuous RAW num_inliers_pose.
Panel (b): Heatmap of Pearson correlation r across 12 VFO predictors x Lags {0,1,2,3,5,10}
           against binary failure target (num_inliers_pose < 8).

Outputs:
- figures/out/fig_10_vfo_leadlag.png
- figures/out/fig_10_vfo_leadlag.pdf
- figures/data/fig_10_vfo_leadlag.csv
- figures/captions/fig_10.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar

VFO_CSV = REPO_ROOT / "results" / "analysis" / "vfo_leadlag_f9.csv"

def main():
    setup_style()
    df_vfo = pd.read_csv(VFO_CSV)

    piv_cont = df_vfo.pivot(index="predictor", columns="lag", values="r_continuous")
    piv_bin = df_vfo.pivot(index="predictor", columns="lag", values="r_binary")

    clean_labels = {
        "border_loss_pct": "Border Loss (%)",
        "estimated_rotational_flow_magnitude": "Est. Rotational Flow Mag",
        "estimated_translational_flow_magnitude": "Est. Translational Flow Mag",
        "feature_count": "Feature Count",
        "feature_survival_rate": "Feature Survival Rate",
        "feature_vel_mean": "Feature Vel. Mean",
        "flow_coherence": "Flow Coherence",
        "flow_direction_entropy": "Flow Direction Entropy",
        "mean_lk_err": "Mean LK Error",
        "rotational_flow_ratio": "Rotational Flow Ratio",
        "spatial_distribution_score": "Spatial Distribution Score",
        "texture_density": "Texture Density",
    }

    piv_cont.index = [clean_labels.get(i, i) for i in piv_cont.index]
    piv_bin.index = [clean_labels.get(i, i) for i in piv_bin.index]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 4.2), sharey=True)

    cmap = sns.diverging_palette(220, 20, as_cmap=True)

    sns.heatmap(piv_cont, annot=True, fmt=".3f", cmap=cmap, vmin=-0.2, vmax=0.2,
                cbar=False, annot_kws={"size": 7.5}, ax=ax1)
    ax1.set_title("(a) Continuous Target (Inlier Count)", fontsize=9.5)
    ax1.set_xlabel("Lag Frames $k$ ($t + k$)")
    ax1.set_ylabel("VFO Predictor ($t$)")

    sns.heatmap(piv_bin, annot=True, fmt=".3f", cmap=cmap, vmin=-0.2, vmax=0.2,
                cbar_kws={"label": "Pearson correlation $r$"}, annot_kws={"size": 7.5}, ax=ax2)
    ax2.set_title("(b) Binary Target (Failure: Inliers < 8)", fontsize=9.5)
    ax2.set_xlabel("Lag Frames $k$ ($t + k$)")
    ax2.set_ylabel("")

    df_sidecar = df_vfo.copy()

    caption_md = (
        "**Figure 10: VFO 12-predictor lead-lag correlation heatmaps (F9 pooled, N=2113).** "
        "(a) Pearson correlation $r$ against continuous inlier count. Est. rotational flow magnitude shows "
        "strongest negative correlation ($r = -0.1531$ at lag 0). (b) Pearson correlation $r$ against binary "
        "failure target (inliers < 8, target saturation 93.04%). Feature count shows strongest predictive signal "
        "($r = -0.1252$ at lag 1)."
    )

    save_fig_and_sidecar(fig, "fig_10_vfo_leadlag", df_sidecar, caption_md)

if __name__ == "__main__":
    main()
