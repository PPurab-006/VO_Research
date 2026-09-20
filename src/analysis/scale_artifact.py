#!/usr/bin/env python3
"""
1.9 Scale-Factor Artifact Analysis.

F9 R1 scale artifact test (RESTRICTED TO F9 R1 ONLY):
Evaluates s_RAW, s_GATED, meter RPE, normalized RPE, and ATE RMSE for RAW, GATED,
and RAW-rescaled (RAW trajectory rescaled by s_GATED / s_RAW prior to rigid alignment).

Notes:
- Normalized RPE is scale-invariant by definition; Fig 9(b) and the primary analysis
  focus on RAW vs GATED normalized RPE.
- The scale-rescaling test is evaluated explicitly on F9 R1 only.

Writes results/analysis/scale_artifact_f9.csv.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
from evo.core import trajectory, metrics
from scipy.spatial.transform import Rotation as R_scipy, Slerp

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import get_canonical_active_window, gt_quats_to_wxyz

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "scale_artifact_f9.csv"


def evaluate_scale_artifact(ds_dir_name="phase2a_F9_L2_R1"):
    ds_dir = DATASETS / ds_dir_name
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

    t_gt_raw = trajectory.PoseTrajectory3D(
        positions_xyz=gt_pos_raw,
        orientations_quat_wxyz=gt_quats_to_wxyz(gt_q_raw),
        timestamps=ts_raw
    )
    t_vo_raw = trajectory.PoseTrajectory3D(
        positions_xyz=df_raw_act[["pos_x", "pos_y", "pos_z"]].values.astype(float),
        orientations_quat_wxyz=gt_quats_to_wxyz(df_raw_act[["rot_x", "rot_y", "rot_z", "rot_w"]].values.astype(float)),
        timestamps=ts_raw
    )

    ts_gated = df_gated_act["timestamp_total_sec"].values.astype(float)
    gt_px = np.interp(ts_gated, gt_t_clean, gt_pos_clean[:, 0])
    gt_py = np.interp(ts_gated, gt_t_clean, gt_pos_clean[:, 1])
    gt_pz = np.interp(ts_gated, gt_t_clean, gt_pos_clean[:, 2])
    gt_pos_gated = np.column_stack([gt_px, gt_py, gt_pz])
    gt_q_gated = slerp(np.clip(ts_gated, gt_t_clean[0], gt_t_clean[-1])).as_quat()

    t_gt_gated = trajectory.PoseTrajectory3D(
        positions_xyz=gt_pos_gated,
        orientations_quat_wxyz=gt_quats_to_wxyz(gt_q_gated),
        timestamps=ts_gated
    )
    t_vo_gated = trajectory.PoseTrajectory3D(
        positions_xyz=df_gated_act[["pos_x", "pos_y", "pos_z"]].values.astype(float),
        orientations_quat_wxyz=gt_quats_to_wxyz(df_gated_act[["rot_x", "rot_y", "rot_z", "rot_w"]].values.astype(float)),
        timestamps=ts_gated
    )

    # 1. Standard Sim(3) fit RAW
    t_raw_al = trajectory.PoseTrajectory3D(
        positions_xyz=np.copy(t_vo_raw.positions_xyz),
        orientations_quat_wxyz=np.copy(t_vo_raw.orientations_quat_wxyz),
        timestamps=ts_raw
    )
    r_raw, tr_raw, s_raw = t_raw_al.align(t_gt_raw, correct_scale=True)

    ape_raw = metrics.APE(metrics.PoseRelation.translation_part)
    ape_raw.process_data((t_gt_raw, t_raw_al))
    ate_raw = ape_raw.get_statistic(metrics.StatisticsType.rmse)

    rpe_raw_m = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
    rpe_raw_m.process_data((t_gt_raw, t_raw_al))
    meter_rpe_raw = rpe_raw_m.get_statistic(metrics.StatisticsType.mean)
    norm_rpe_raw = meter_rpe_raw / s_raw

    # 2. Standard Sim(3) fit GATED
    t_gated_al = trajectory.PoseTrajectory3D(
        positions_xyz=np.copy(t_vo_gated.positions_xyz),
        orientations_quat_wxyz=np.copy(t_vo_gated.orientations_quat_wxyz),
        timestamps=ts_gated
    )
    r_gated, tr_gated, s_gated = t_gated_al.align(t_gt_gated, correct_scale=True)

    ape_gated = metrics.APE(metrics.PoseRelation.translation_part)
    ape_gated.process_data((t_gt_gated, t_gated_al))
    ate_gated = ape_gated.get_statistic(metrics.StatisticsType.rmse)

    rpe_gated_m = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
    rpe_gated_m.process_data((t_gt_gated, t_gated_al))
    meter_rpe_gated = rpe_gated_m.get_statistic(metrics.StatisticsType.mean)
    norm_rpe_gated = meter_rpe_gated / s_gated

    # 3. RAW-rescaled: apply s_GATED to RAW before rigid SE(3) alignment (without scale re-fitting)
    t_raw_rescaled = trajectory.PoseTrajectory3D(
        positions_xyz=s_gated * (t_raw_al.positions_xyz - tr_raw) / s_raw + tr_raw,
        orientations_quat_wxyz=np.copy(t_raw_al.orientations_quat_wxyz),
        timestamps=ts_raw
    )

    ape_rescaled = metrics.APE(metrics.PoseRelation.translation_part)
    ape_rescaled.process_data((t_gt_raw, t_raw_rescaled))
    ate_rescaled = ape_rescaled.get_statistic(metrics.StatisticsType.rmse)

    rpe_rescaled_m = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
    rpe_rescaled_m.process_data((t_gt_raw, t_raw_rescaled))
    meter_rpe_rescaled = rpe_rescaled_m.get_statistic(metrics.StatisticsType.mean)

    return {
        "dataset_run": "F9 R1 (phase2a_F9_L2_R1) ONLY",
        "s_RAW": s_raw,
        "s_GATED": s_gated,
        "meter_rpe_RAW": meter_rpe_raw,
        "meter_rpe_GATED": meter_rpe_gated,
        "meter_rpe_RAW_rescaled": meter_rpe_rescaled,
        "norm_rpe_RAW": norm_rpe_raw,
        "norm_rpe_GATED": norm_rpe_gated,
        "ate_RAW": ate_raw,
        "ate_GATED": ate_gated,
        "ate_RAW_rescaled": ate_rescaled,
    }


def main():
    res_r1 = evaluate_scale_artifact("phase2a_F9_L2_R1")

    df_out = pd.DataFrame([res_r1])
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    df_out.to_csv(OUTPUT, index=False)

    print("========================================================")
    print("SCALE FACTOR ARTIFACT ANALYSIS (F9 R1 ONLY)")
    print("========================================================")
    print("Dataset Scope: F9 R1 ONLY (phase2a_F9_L2_R1)")
    print(f"Recovered Scale Factors:")
    print(f"  s_RAW                  : {res_r1['s_RAW']:.6f}")
    print(f"  s_GATED                : {res_r1['s_GATED']:.6f}")
    print(f"  Scale Ratio (s_G / s_R): {res_r1['s_GATED'] / res_r1['s_RAW']:.6f} (+{(res_r1['s_GATED']/res_r1['s_RAW'] - 1)*100:.2f}%)")

    print(f"\nMeter RPE (m/step):")
    print(f"  RAW                    : {res_r1['meter_rpe_RAW']:.4f}")
    print(f"  GATED                  : {res_r1['meter_rpe_GATED']:.4f}")
    print(f"  RAW-rescaled           : {res_r1['meter_rpe_RAW_rescaled']:.4f}")

    print(f"\nNormalized RPE (unit step error, scale-invariant):")
    print(f"  RAW                    : {res_r1['norm_rpe_RAW']:.4f}")
    print(f"  GATED                  : {res_r1['norm_rpe_GATED']:.4f}")

    print(f"\nATE RMSE (m):")
    print(f"  RAW                    : {res_r1['ate_RAW']:.4f}")
    print(f"  GATED                  : {res_r1['ate_GATED']:.4f}")
    print(f"  RAW-rescaled           : {res_r1['ate_RAW_rescaled']:.4f}")

    rpe_reproduces = abs(res_r1['meter_rpe_RAW_rescaled'] - 0.0978) < 0.001
    ate_reproduces = abs(res_r1['ate_RAW_rescaled'] - 3.4275) < 0.01

    print("\nREPRODUCTION CHECK:")
    if rpe_reproduces and ate_reproduces:
        print(f"  Meter RPE ~0.0978 m/step ({res_r1['meter_rpe_RAW_rescaled']:.4f}) and ATE ~3.4275 m ({res_r1['ate_RAW_rescaled']:.4f}) REPRODUCE EXACTLY on F9 R1.")
    else:
        print(f"  DISCREPANCY DETECTED: Expected RPE ~0.0978 / ATE ~3.4275, got RPE {res_r1['meter_rpe_RAW_rescaled']:.4f} / ATE {res_r1['ate_RAW_rescaled']:.4f}")

    print(f"\nWrote results to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
