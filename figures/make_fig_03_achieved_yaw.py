#!/usr/bin/env python3
"""
Fig 3: Achieved Yaw Dynamics & Heading Profiles (F6_L2, F9_L2, F10_L3).

Panel (a): Unwrapped heading trajectory psi(t) showing true mean heading offsets (F6_L2: 78.78 deg,
           F9_L2: 83.02 deg, F10_L3: 89.49 deg) and peak-to-peak excursions (~118-129 deg).
Panel (b): Absolute yaw rate |yaw_rate| time series, highlighting high-dynamic bursts and
           flagging the extreme transient spike in p3_F6_L2_R1 (VO logged peak: 582.3 deg/s).

Outputs:
- figures/out/fig_03_achieved_yaw.png
- figures/out/fig_03_achieved_yaw.pdf
- figures/data/fig_03_achieved_yaw.csv
- figures/captions/fig_03.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_ALT, COLOR_GRAY
from evaluate_phase3_evo import get_canonical_active_window

DATASETS = REPO_ROOT / "results" / "datasets"
YAW_AUDIT_CSV = REPO_ROOT / "results" / "analysis" / "achieved_yaw_audit.csv"

def main():
    setup_style()
    df_audit = pd.read_csv(YAW_AUDIT_CSV)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 3.2))

    runs = {
        "F6_L2 (p3_F6_L2_R1)": ("p3_F6_L2_R1", COLOR_RAW),
        "F9_L2 (phase2a_F9_L2_R1)": ("phase2a_F9_L2_R1", COLOR_GATED),
        "F10_L3 (p3_F10_L3_R1)": ("p3_F10_L3_R1", COLOR_ALT),
    }

    for label, (r_name, col) in runs.items():
        ds_dir = DATASETS / r_name
        df_gt = pd.read_csv(ds_dir / "dataset_gt.csv")
        df_vo = pd.read_csv(ds_dir / ("eis_gated_vo.csv" if (ds_dir / "eis_gated_vo.csv").exists() else "raw_vo.csv"))

        t_start, t_end, _ = get_canonical_active_window(df_gt)
        mask_gt = (df_gt["timestamp_total_sec"] >= t_start) & (df_gt["timestamp_total_sec"] <= t_end)
        df_gt_act = df_gt[mask_gt].reset_index(drop=True)

        quats = df_gt_act[["rot_x", "rot_y", "rot_z", "rot_w"]].values
        rot_mat = R_scipy.from_quat(quats).as_matrix()
        body_x = rot_mat[:, :, 0]
        heading_rad = np.unwrap(np.arctan2(body_x[:, 1], body_x[:, 0]))
        heading_deg = np.degrees(heading_rad)
        rel_t_gt = df_gt_act["timestamp_total_sec"].values - df_gt_act["timestamp_total_sec"].values[0]

        ax1.plot(rel_t_gt, heading_deg, label=label, color=col, lw=1.3)

        mask_vo = (df_vo["timestamp_total_sec"] >= t_start) & (df_vo["timestamp_total_sec"] <= t_end)
        df_vo_act = df_vo[mask_vo].reset_index(drop=True)
        rel_t_vo = df_vo_act["timestamp_total_sec"].values - df_vo_act["timestamp_total_sec"].values[0]
        
        yaw_rate = df_vo_act["eis_yaw_rate_deg"].abs() if "eis_yaw_rate_deg" in df_vo_act.columns else pd.Series(np.zeros(len(df_vo_act)))
        ax2.plot(rel_t_vo, yaw_rate, label=label, color=col, lw=1.1, alpha=0.85)

    ax1.axhline(15.0, color=COLOR_GRAY, linestyle=":", lw=1.0, alpha=0.7)
    ax1.set_xlabel("Active Window Time (s)")
    ax1.set_ylabel("Unwrapped Heading $\\psi(t)$ (deg)")
    ax1.set_title("(a) Heading Profiles & Excursions", fontsize=9.5)
    ax1.legend(loc="upper left", frameon=True, facecolor="white", framealpha=0.9)

    ax2.axhline(15.0, color="red", linestyle="--", lw=1.2, label="Gating Threshold (15 deg/s)")
    ax2.annotate("p3_F6_L2_R1 Spike\n(Peak: 582.3 deg/s)", xy=(15.2, 582.3), xytext=(10.0, 450),
                 arrowprops=dict(facecolor=COLOR_RAW, shrink=0.08, width=1.0, headwidth=5.0),
                 fontsize=8.0, color=COLOR_RAW, weight="bold")

    ax2.set_xlabel("Active Window Time (s)")
    ax2.set_ylabel("Absolute Yaw Rate $|\\dot{\\psi}|$ (deg/s)")
    ax2.set_title("(b) Achieved Yaw-Rate Dynamics", fontsize=9.5)
    ax2.legend(loc="upper right", frameon=True, facecolor="white", framealpha=0.9)
    ax2.set_ylim(-10, 650)

    # Sidecar CSV
    df_sidecar = df_audit[df_audit["run_dir"].isin(["p3_F6_L2_R1", "phase2a_F9_L2_R1", "p3_F10_L3_R1"])].copy()

    caption_md = (
        "**Figure 3: Achieved yaw dynamics and heading profiles for F6_L2, F9_L2, and F10_L3.** "
        "(a) Unwrapped GT heading $\\psi(t)$ shows true mean heading offsets of 78.78 deg for F6_L2, "
        "83.02 deg for F9_L2, and 89.49 deg for F10_L3, with peak-to-peak excursions between 118.3 deg "
        "and 126.9 deg. (b) VO-logged absolute yaw rate $|\\dot{\\psi}|$, highlighting the transient burst "
        "spike in `p3_F6_L2_R1` (VO logged peak: 582.3 deg/s, GT peak: 974.9 deg/s)."
    )

    save_fig_and_sidecar(fig, "fig_03_achieved_yaw", df_sidecar, caption_md)

if __name__ == "__main__":
    main()
