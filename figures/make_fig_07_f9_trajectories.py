#!/usr/bin/env python3
"""
Fig 7: F9 Trajectory Overlay Across All Three Runs (R1, R2, R3).

3 subpanels:
- (a) F9 Run 1 (phase2a_F9_L2_R1): Sim(3)-aligned GT vs RAW vs GATED trajectories & ATEs
- (b) F9 Run 2 (phase2a_F9_L2_R2): Sim(3)-aligned GT vs RAW vs GATED trajectories & ATEs
- (c) F9 Run 3 (phase2a_F9_L2_R3): Sim(3)-aligned GT vs RAW vs GATED trajectories & ATEs

Outputs:
- figures/out/fig_07_f9_trajectories.png
- figures/out/fig_07_f9_trajectories.pdf
- figures/data/fig_07_f9_trajectories.csv
- figures/captions/fig_07.md
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from evo.core import trajectory, metrics
from scipy.spatial.transform import Rotation as R_scipy, Slerp

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "figures"))
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from style import setup_style, save_fig_and_sidecar, COLOR_RAW, COLOR_GATED, COLOR_GRAY
from evaluate_phase3_evo import get_canonical_active_window, gt_quats_to_wxyz

DATASETS = REPO_ROOT / "results" / "datasets"

def main():
    setup_style()
    fig, axes = plt.subplots(1, 3, figsize=(7.0, 2.8))

    sidecar_records = []

    runs = [
        ("phase2a_F9_L2_R1", 1, axes[0], "(a) F9 Run 1"),
        ("phase2a_F9_L2_R2", 2, axes[1], "(b) F9 Run 2"),
        ("phase2a_F9_L2_R3", 3, axes[2], "(c) F9 Run 3"),
    ]

    for ds_name, r_idx, ax, title in runs:
        ds_dir = DATASETS / ds_name
        df_raw = pd.read_csv(ds_dir / "raw_vo.csv")
        df_gated = pd.read_csv(ds_dir / "eis_gated_vo.csv")
        df_gt = pd.read_csv(ds_dir / "dataset_gt.csv")

        t_start, t_end, _ = get_canonical_active_window(df_gt)

        m_raw = (df_raw["timestamp_total_sec"] >= t_start) & (df_raw["timestamp_total_sec"] <= t_end)
        m_gated = (df_gated["timestamp_total_sec"] >= t_start) & (df_gated["timestamp_total_sec"] <= t_end)

        df_raw_act = df_raw[m_raw].reset_index(drop=True)
        df_gated_act = df_gated[m_gated].reset_index(drop=True)

        gt_t_raw = df_gt["timestamp_total_sec"].values.astype(float)
        gt_t_clean, u_idx = np.unique(gt_t_raw, return_index=True)
        gt_pos_clean = df_gt[["pos_x", "pos_y", "pos_z"]].values[u_idx]
        rot_clean = R_scipy.from_quat(df_gt[["rot_x", "rot_y", "rot_z", "rot_w"]].values[u_idx])
        slerp = Slerp(gt_t_clean, rot_clean)

        ts_raw = df_raw_act["timestamp_total_sec"].values.astype(float)
        gt_px = np.interp(ts_raw, gt_t_clean, gt_pos_clean[:, 0])
        gt_py = np.interp(ts_raw, gt_t_clean, gt_pos_clean[:, 1])
        gt_pz = np.interp(ts_raw, gt_t_clean, gt_pos_clean[:, 2])
        gt_pos_raw = np.column_stack([gt_px, gt_py, gt_pz])
        gt_q_raw = slerp(np.clip(ts_raw, gt_t_clean[0], gt_t_clean[-1])).as_quat()

        t_gt = trajectory.PoseTrajectory3D(positions_xyz=gt_pos_raw, orientations_quat_wxyz=gt_quats_to_wxyz(gt_q_raw), timestamps=ts_raw)
        t_vo_raw = trajectory.PoseTrajectory3D(positions_xyz=df_raw_act[["pos_x", "pos_y", "pos_z"]].values.astype(float), orientations_quat_wxyz=gt_quats_to_wxyz(df_raw_act[["rot_x", "rot_y", "rot_z", "rot_w"]].values.astype(float)), timestamps=ts_raw)

        ts_gated = df_gated_act["timestamp_total_sec"].values.astype(float)
        t_vo_gated = trajectory.PoseTrajectory3D(positions_xyz=df_gated_act[["pos_x", "pos_y", "pos_z"]].values.astype(float), orientations_quat_wxyz=gt_quats_to_wxyz(df_gated_act[["rot_x", "rot_y", "rot_z", "rot_w"]].values.astype(float)), timestamps=ts_gated)

        # Sim(3) align RAW
        t_raw_al = trajectory.PoseTrajectory3D(positions_xyz=np.copy(t_vo_raw.positions_xyz), orientations_quat_wxyz=np.copy(t_vo_raw.orientations_quat_wxyz), timestamps=ts_raw)
        _, _, s_raw = t_raw_al.align(t_gt, correct_scale=True)
        ape_raw = metrics.APE(metrics.PoseRelation.translation_part)
        ape_raw.process_data((t_gt, t_raw_al))
        ate_raw = ape_raw.get_statistic(metrics.StatisticsType.rmse)

        # Sim(3) align GATED
        t_gated_al = trajectory.PoseTrajectory3D(positions_xyz=np.copy(t_vo_gated.positions_xyz), orientations_quat_wxyz=np.copy(t_vo_gated.orientations_quat_wxyz), timestamps=ts_gated)
        _, _, s_gated = t_gated_al.align(t_gt, correct_scale=True)
        ape_gated = metrics.APE(metrics.PoseRelation.translation_part)
        ape_gated.process_data((t_gt, t_gated_al))
        ate_gated = ape_gated.get_statistic(metrics.StatisticsType.rmse)

        # Plot XY Plan View
        ax.plot(t_gt.positions_xyz[:, 0], t_gt.positions_xyz[:, 1], color="black", linestyle="-", lw=1.5, label="Ground Truth")
        ax.plot(t_raw_al.positions_xyz[:, 0], t_raw_al.positions_xyz[:, 1], color=COLOR_RAW, linestyle="--", lw=1.2, label=f"RAW ({ate_raw:.2f}m)")
        ax.plot(t_gated_al.positions_xyz[:, 0], t_gated_al.positions_xyz[:, 1], color=COLOR_GATED, linestyle="-.", lw=1.2, label=f"GATED ({ate_gated:.2f}m)")

        ax.set_xlabel("X Position (m)", fontsize=8.5)
        if r_idx == 1:
            ax.set_ylabel("Y Position (m)", fontsize=8.5)
        ax.set_title(title, fontsize=9.0)
        ax.legend(loc="lower right", fontsize=6.5, frameon=True, facecolor="white", framealpha=0.9)
        ax.set_aspect("equal", adjustable="datalim")

        sidecar_records.append({
            "run": f"F9 R{r_idx}",
            "dataset_dir": ds_name,
            "ate_raw_m": float(ate_raw),
            "ate_gated_m": float(ate_gated),
            "scale_raw": float(s_raw),
            "scale_gated": float(s_gated),
        })

    plt.tight_layout()
    df_sidecar = pd.DataFrame(sidecar_records)

    # ----------------------------------------------------
    # Verification & Printing Reconciliation Table
    # ----------------------------------------------------
    PER_RUN_CSV = REPO_ROOT / "results" / "analysis" / "per_run_metrics.csv"
    df_per = pd.read_csv(PER_RUN_CSV)
    df_f9_csv = df_per[df_per["family"] == "F9_L2"]

    print("\n" + "="*85)
    print("FIG 7 F9 ATE RECONCILIATION WITH per_run_metrics.csv")
    print("="*85)
    print(f"{'Run':<6} | {'Fig RAW ATE (m)':<16} | {'CSV RAW ATE (m)':<16} | {'Fig GATED ATE (m)':<17} | {'CSV GATED ATE (m)':<17} | {'Match?':<6}")
    print("-" * 85)

    for r_idx in [1, 2, 3]:
        raw_fig = df_sidecar.iloc[r_idx - 1]["ate_raw_m"]
        gated_fig = df_sidecar.iloc[r_idx - 1]["ate_gated_m"]

        raw_csv = df_f9_csv[(df_f9_csv["run"] == r_idx) & (df_f9_csv["mechanism"] == "RAW")]["ate_rmse"].values[0]
        gated_csv = df_f9_csv[(df_f9_csv["run"] == r_idx) & (df_f9_csv["mechanism"] == "EIS-GATED")]["ate_rmse"].values[0]

        match_raw = abs(raw_fig - raw_csv) < 1e-4
        match_gated = abs(gated_fig - gated_csv) < 1e-4
        is_match = match_raw and match_gated

        print(f"F9 R{r_idx} | {raw_fig:16.4f} | {raw_csv:16.4f} | {gated_fig:17.4f} | {gated_csv:17.4f} | {str(is_match):<6}")
        assert is_match, f"F9 R{r_idx} ATE mismatch! Fig: RAW={raw_fig}, GATED={gated_fig}; CSV: RAW={raw_csv}, GATED={gated_csv}"

    print("="*85)
    print("CONFIRMED: All Fig 7 F9 ATE values match per_run_metrics.csv exactly.")
    print("="*85 + "\n")

    caption_md = (
        "**Figure 7: F9 ground-truth vs RAW vs EIS-GATED trajectories across all three runs.** "
        f"Shows Sim(3)-aligned plan views (XY) over active windows for (a) Run 1 (RAW ATE: {df_sidecar.iloc[0]['ate_raw_m']:.2f}m, GATED ATE: {df_sidecar.iloc[0]['ate_gated_m']:.2f}m), "
        f"(b) Run 2 (RAW ATE: {df_sidecar.iloc[1]['ate_raw_m']:.2f}m, GATED ATE: {df_sidecar.iloc[1]['ate_gated_m']:.2f}m), and "
        f"(c) Run 3 (RAW ATE: {df_sidecar.iloc[2]['ate_raw_m']:.2f}m, GATED ATE: {df_sidecar.iloc[2]['ate_gated_m']:.2f}m). "
        "All printed ATE values match `per_run_metrics.csv` exactly."
    )

    save_fig_and_sidecar(fig, "fig_07_f9_trajectories", df_sidecar, caption_md)

if __name__ == "__main__":
    main()

