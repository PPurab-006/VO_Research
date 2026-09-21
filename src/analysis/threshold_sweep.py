#!/usr/bin/env python3
"""
1.10 Gating Yaw-Rate Threshold Sweep Analysis (F9 R1-R3).

Executes the offline VO pipeline (run_offline_vo.py) across gating yaw-rate thresholds:
gate_thresh in {5, 10, 15, 20, 30, 45} deg/s for F9 R1, R2, R3 (18 runs total).

Computes per-run and mean +/- std:
- valid_pose_pct (%)
- ATE RMSE (m)
- Recovered scale factor s
- Meter RPE (m/step)
- Normalized RPE (unit step error)

Writes results/analysis/threshold_sweep.csv.
"""

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
from evo.core import trajectory, metrics
from scipy.spatial.transform import Rotation as R_scipy, Slerp

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import get_canonical_active_window, gt_quats_to_wxyz

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "threshold_sweep.csv"
REPLAYS_DIR = REPO_ROOT / "results" / "analysis" / "threshold_sweep_replays"
SCRATCH_DIR = REPO_ROOT / "scratch" / "threshold_sweep"

THRESHOLDS = [5.0, 10.0, 15.0, 20.0, 30.0, 45.0]
RUNS = [
    ("phase2a_F9_L2_R1", 1),
    ("phase2a_F9_L2_R2", 2),
    ("phase2a_F9_L2_R3", 3),
]


def evaluate_trajectory(vo_csv_path, gt_csv_path):
    df_vo = pd.read_csv(vo_csv_path)
    df_gt = pd.read_csv(gt_csv_path)

    t_start, t_end, _ = get_canonical_active_window(df_gt)

    m_vo = (df_vo["timestamp_total_sec"] >= t_start) & (df_vo["timestamp_total_sec"] <= t_end)
    df_vo_act = df_vo[m_vo].reset_index(drop=True)

    gt_t_raw = df_gt["timestamp_total_sec"].values.astype(float)
    gt_t_clean, u_idx = np.unique(gt_t_raw, return_index=True)
    gt_pos_clean = df_gt[["pos_x", "pos_y", "pos_z"]].values[u_idx]
    rot_clean = R_scipy.from_quat(df_gt[["rot_x", "rot_y", "rot_z", "rot_w"]].values[u_idx])
    slerp = Slerp(gt_t_clean, rot_clean)

    ts_vo = df_vo_act["timestamp_total_sec"].values.astype(float)
    gt_px = np.interp(ts_vo, gt_t_clean, gt_pos_clean[:, 0])
    gt_py = np.interp(ts_vo, gt_t_clean, gt_pos_clean[:, 1])
    gt_pz = np.interp(ts_vo, gt_t_clean, gt_pos_clean[:, 2])
    gt_pos = np.column_stack([gt_px, gt_py, gt_pz])
    gt_q = slerp(np.clip(ts_vo, gt_t_clean[0], gt_t_clean[-1])).as_quat()

    t_gt = trajectory.PoseTrajectory3D(
        positions_xyz=gt_pos,
        orientations_quat_wxyz=gt_quats_to_wxyz(gt_q),
        timestamps=ts_vo
    )
    t_vo = trajectory.PoseTrajectory3D(
        positions_xyz=df_vo_act[["pos_x", "pos_y", "pos_z"]].values.astype(float),
        orientations_quat_wxyz=gt_quats_to_wxyz(df_vo_act[["rot_x", "rot_y", "rot_z", "rot_w"]].values.astype(float)),
        timestamps=ts_vo
    )

    # Sim(3) align
    t_vo_al = trajectory.PoseTrajectory3D(
        positions_xyz=np.copy(t_vo.positions_xyz),
        orientations_quat_wxyz=np.copy(t_vo.orientations_quat_wxyz),
        timestamps=ts_vo
    )
    r_mat, tr_vec, s_factor = t_vo_al.align(t_gt, correct_scale=True)

    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((t_gt, t_vo_al))
    ate_rmse = ape_metric.get_statistic(metrics.StatisticsType.rmse)

    rpe_metric = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
    rpe_metric.process_data((t_gt, t_vo_al))
    meter_rpe = rpe_metric.get_statistic(metrics.StatisticsType.mean)

    valid_pose_pct = (df_vo_act["num_inliers_pose"] >= 8).mean() * 100.0
    norm_rpe = meter_rpe / s_factor if s_factor > 1e-6 else np.nan

    return {
        "valid_pose_pct": valid_pose_pct,
        "ate_rmse": float(ate_rmse),
        "scale_factor": float(s_factor),
        "meter_rpe": float(meter_rpe),
        "norm_rpe": float(norm_rpe),
        "active_frames": len(df_vo_act),
    }


def main():
    parser = argparse.ArgumentParser(description="Threshold sweep analysis")
    parser.add_argument("--rerun", action="store_true", help="Launch offline VO from raw camera frames")
    args = parser.parse_args()

    REPLAYS_DIR.mkdir(parents=True, exist_ok=True)
    records = []

    print("========================================================")
    if args.rerun:
        print("LAUNCHING THRESHOLD SWEEP (RERUN MODE - 18 RUNS TOTAL)")
    else:
        print("EVALUATING COMMITTED THRESHOLD SWEEP REPLAYS (DEFAULT MODE)")
    print("========================================================")

    t_start_sweep = time.time()

    # Pre-read existing runtime_sec if available in default mode
    prev_runtimes = {}
    if not args.rerun and OUTPUT.exists():
        try:
            df_prev = pd.read_csv(OUTPUT)
            if "runtime_sec" in df_prev.columns:
                for _, row in df_prev.iterrows():
                    prev_runtimes[(float(row["gate_thresh_deg"]), int(row["repeat"]))] = float(row["runtime_sec"])
        except Exception:
            pass

    for thresh in THRESHOLDS:
        for ds_name, repeat_idx in RUNS:
            ds_dir = DATASETS / ds_name
            gt_csv = ds_dir / "dataset_gt.csv"
            replay_filename = f"gated_thresh_{int(thresh)}_R{repeat_idx}.csv"
            replay_csv = REPLAYS_DIR / replay_filename
            if not replay_csv.exists() and (SCRATCH_DIR / replay_filename).exists():
                replay_csv = SCRATCH_DIR / replay_filename

            if args.rerun:
                t0 = time.time()
                cmd = (
                    f"python3 {REPO_ROOT}/src/pipelines/run_offline_vo.py "
                    f"--dataset-dir {ds_dir} "
                    f"--gt-csv {gt_csv} "
                    f"--output-csv {replay_csv} "
                    f"--eis --eis-mode gated "
                    f"--gate-thresh {thresh} > /dev/null 2>&1"
                )
                os.system(cmd)
                t_run = time.time() - t0
            else:
                if not replay_csv.exists():
                    raise FileNotFoundError(f"Replay file {replay_csv} not found. Run with --rerun or place replay CSVs in {REPLAYS_DIR}")
                t_run = prev_runtimes.get((thresh, repeat_idx), float("nan"))

            eval_res = evaluate_trajectory(replay_csv, gt_csv)
            runtime_str = f"{t_run:5.2f}s" if not np.isnan(t_run) else " NaN"
            print(f" thresh={thresh:4.1f} R{repeat_idx} done ({runtime_str}) | Valid={eval_res['valid_pose_pct']:5.2f}%, ATE={eval_res['ate_rmse']:6.4f}m, normRPE={eval_res['norm_rpe']:6.4f}")

            records.append({
                "gate_thresh_deg": thresh,
                "repeat": repeat_idx,
                "dataset_run": ds_name,
                "runtime_sec": t_run if np.isnan(t_run) else round(t_run, 2),
                "valid_pose_pct": float(eval_res["valid_pose_pct"]),
                "ate_rmse": round(eval_res["ate_rmse"], 4),
                "scale_factor": round(eval_res["scale_factor"], 6),
                "meter_rpe": round(eval_res["meter_rpe"], 4),
                "norm_rpe": round(eval_res["norm_rpe"], 4),
            })

    t_total = time.time() - t_start_sweep

    df = pd.DataFrame(records)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUTPUT, index=False)

    print("\n========================================================")
    print("THRESHOLD SWEEP PER-RUN RESULTS TABLE")
    print("========================================================")
    print(df[["gate_thresh_deg", "repeat", "valid_pose_pct", "ate_rmse", "scale_factor", "meter_rpe", "norm_rpe"]].to_string(index=False))

    print("\n========================================================")
    print("THRESHOLD SWEEP SUMMARY (MEAN +/- STD ACROSS F9 R1..R3)")
    print("========================================================")

    summary_rows = []
    for thresh in THRESHOLDS:
        sub = df[df["gate_thresh_deg"] == thresh]
        valid_mean, valid_std = sub["valid_pose_pct"].mean(), sub["valid_pose_pct"].std()
        ate_mean, ate_std = sub["ate_rmse"].mean(), sub["ate_rmse"].std()
        s_mean, s_std = sub["scale_factor"].mean(), sub["scale_factor"].std()
        mrpe_mean, mrpe_std = sub["meter_rpe"].mean(), sub["meter_rpe"].std()
        nrpe_mean, nrpe_std = sub["norm_rpe"].mean(), sub["norm_rpe"].std()

        summary_rows.append({
            "gate_thresh_deg": thresh,
            "valid_pose_pct_mean": round(valid_mean, 2),
            "valid_pose_pct_std": round(valid_std, 2),
            "ate_rmse_mean": round(ate_mean, 4),
            "ate_rmse_std": round(ate_std, 4),
            "scale_factor_mean": round(s_mean, 6),
            "scale_factor_std": round(s_std, 6),
            "meter_rpe_mean": round(mrpe_mean, 4),
            "meter_rpe_std": round(mrpe_std, 4),
            "norm_rpe_mean": round(nrpe_mean, 4),
            "norm_rpe_std": round(nrpe_std, 4),
        })

    df_sum = pd.DataFrame(summary_rows)
    print(df_sum.to_string(index=False))

    print(f"\nTotal sweep duration: {t_total / 60:.2f} minutes ({t_total:.1f} seconds)")
    print(f"Wrote per-run sweep results to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()

