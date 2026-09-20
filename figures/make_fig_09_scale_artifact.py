#!/usr/bin/env python3
"""
Fig 9: Scale Factor Expansion & Normalized RPE Analysis (F9 R1 Only).

Panel (a): Meter RPE (m/step) for RAW (0.0838 m), GATED (0.0980 m), and RAW-rescaled (0.0978 m),
           demonstrating that pre-alignment scale expansion by s_GATED / s_RAW reproduces GATED's meter RPE.
Panel (b): Normalized RPE (unit step error, scale-invariant) for RAW (0.9143) and GATED (0.8925) ONLY.

Outputs:
- figures/out/fig_09_scale_artifact.png
- figures/out/fig_09_scale_artifact.pdf
- figures/data/fig_09_scale_artifact.csv
- figures/captions/fig_09.md
Note: Restricted explicitly to F9 R1 only.
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_ALT

SCALE_CSV = REPO_ROOT / "results" / "analysis" / "scale_artifact_f9.csv"

def main():
    setup_style()
    df_scale = pd.read_csv(SCALE_CSV)
    row = df_scale.iloc[0]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.2))

    # Panel (a): Meter RPE Comparison (RAW vs GATED vs RAW-Rescaled)
    categories_a = ["RAW\n(s=0.0917)", "GATED\n(s=0.1099)", "RAW-Rescaled\n(by s_G/s_R)"]
    values_a = [row["meter_rpe_RAW"], row["meter_rpe_GATED"], row["meter_rpe_RAW_rescaled"]]
    colors_a = [COLOR_RAW, COLOR_GATED, COLOR_ALT]

    bars1 = ax1.bar(categories_a, values_a, color=colors_a, width=0.55, edgecolor="black", lw=0.8)
    for bar in bars1:
        yval = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2.0, yval + 0.001, f"{yval:.4f}m", ha="center", va="bottom", size=8.5, weight="bold")

    ax1.set_ylabel("Frame-to-Frame Meter RPE (m/step)")
    ax1.set_title("(a) Meter RPE Scale Artifact (F9 R1 Only)", fontsize=9.5)
    ax1.set_ylim(0, 0.12)

    # Panel (b): Normalized RPE (RAW and GATED ONLY)
    categories_b = ["RAW", "EIS-GATED"]
    values_b = [row["norm_rpe_RAW"], row["norm_rpe_GATED"]]
    colors_b = [COLOR_RAW, COLOR_GATED]

    bars2 = ax2.bar(categories_b, values_b, color=colors_b, width=0.45, edgecolor="black", lw=0.8)
    for bar in bars2:
        yval = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2.0, yval + 0.01, f"{yval:.4f}", ha="center", va="bottom", size=8.5, weight="bold")

    ax2.set_ylabel("Normalized RPE (unit step error)")
    ax2.set_title("(b) Scale-Invariant Normalized RPE (F9 R1 Only)", fontsize=9.5)
    ax2.set_ylim(0, 1.1)

    df_sidecar = pd.DataFrame([{
        "scope": "F9 R1 only (phase2a_F9_L2_R1)",
        "s_RAW": float(row["s_RAW"]),
        "s_GATED": float(row["s_GATED"]),
        "meter_rpe_RAW": float(row["meter_rpe_RAW"]),
        "meter_rpe_GATED": float(row["meter_rpe_GATED"]),
        "meter_rpe_RAW_rescaled": float(row["meter_rpe_RAW_rescaled"]),
        "norm_rpe_RAW": float(row["norm_rpe_RAW"]),
        "norm_rpe_GATED": float(row["norm_rpe_GATED"]),
        "ate_RAW": float(row["ate_RAW"]),
        "ate_GATED": float(row["ate_GATED"]),
        "ate_RAW_rescaled": float(row["ate_RAW_rescaled"]),
    }])

    caption_md = (
        "**Figure 9: Scale factor artifact & normalized RPE analysis (F9 R1 only).** "
        "(a) Shows meter RPE for RAW (0.0838 m/step), GATED (0.0980 m/step), and RAW-rescaled (0.0978 m/step). "
        "Pre-alignment scale expansion of RAW by $s_{\\text{GATED}}/s_{\\text{RAW}}$ reproduces GATED's meter RPE, "
        "proving that GATED's higher meter RPE is an artifact of its larger scale factor ($s=0.1099$ vs $0.0917$). "
        "(b) Scale-invariant normalized RPE for RAW (0.9143) vs GATED (0.8925) only. Data scope: F9 R1 only."
    )

    save_fig_and_sidecar(fig, "fig_09_scale_artifact", df_sidecar, caption_md)

if __name__ == "__main__":
    main()
