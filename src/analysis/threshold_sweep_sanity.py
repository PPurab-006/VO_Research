#!/usr/bin/env python3
"""
Threshold Sweep Sanity Gate Script.

Replays run_offline_vo.py at gate_thresh=15.0 for RAW and EIS-GATED on F9 R1, R2, R3.
Reports per run: valid_pose_pct replayed vs stored, exact inliers match fraction,
and execution time. Times one run and estimates wall-clock duration for 18 runs.

Writes results/analysis/threshold_sweep_sanity.csv.
"""

import os
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "src" / "pipelines"))
from evaluate_phase3_evo import get_canonical_active_window

DATASETS = REPO_ROOT / "results" / "datasets"
OUTPUT = REPO_ROOT / "results" / "analysis" / "threshold_sweep_sanity.csv"
SCRATCH_DIR = REPO_ROOT / "scratch" / "sanity_replay"


def run_sanity_gate():
    SCRATCH_DIR.mkdir(parents=True, exist_ok=True)
    runs = ["phase2a_F9_L2_R1", "phase2a_F9_L2_R2", "phase2a_F9_L2_R3"]

    records = []
    runtimes = []

    for run_name in runs:
        ds_dir = DATASETS / run_name
        gt_csv = ds_dir / "dataset_gt.csv"
        df_gt = pd.read_csv(gt_csv)
        t_start, t_end, _ = get_canonical_active_window(df_gt)

        # 1. RAW Mode Replay
        raw_stored_csv = ds_dir / "raw_vo.csv"
        raw_out_csv = SCRATCH_DIR / f"{run_name}_raw_replayed.csv"

        t0 = time.time()
        cmd_raw = f"python3 {REPO_ROOT}/src/pipelines/run_offline_vo.py --dataset-dir {ds_dir} --gt-csv {gt_csv} --output-csv {raw_out_csv} > /dev/null 2>&1"
        os.system(cmd_raw)
        t_raw = time.time() - t0
        runtimes.append(t_raw)

        df_raw_stored = pd.read_csv(raw_stored_csv)
        df_raw_replayed = pd.read_csv(raw_out_csv)

        m_stored = (df_raw_stored["timestamp_total_sec"] >= t_start) & (df_raw_stored["timestamp_total_sec"] <= t_end)
        m_replayed = (df_raw_replayed["timestamp_total_sec"] >= t_start) & (df_raw_replayed["timestamp_total_sec"] <= t_end)

        act_stored = df_raw_stored[m_stored].reset_index(drop=True)
        act_replayed = df_raw_replayed[m_replayed].reset_index(drop=True)
        n = min(len(act_stored), len(act_replayed))
        act_stored, act_replayed = act_stored.iloc[:n], act_replayed.iloc[:n]

        valid_stored_raw = (act_stored["num_inliers_pose"] >= 8).mean() * 100.0
        valid_replayed_raw = (act_replayed["num_inliers_pose"] >= 8).mean() * 100.0
        exact_match_raw = (act_stored["num_inliers_pose"].values == act_replayed["num_inliers_pose"].values).mean() * 100.0

        records.append({
            "run": run_name,
            "mode": "RAW",
            "gate_thresh_deg": 15.0,
            "runtime_sec": round(t_raw, 2),
            "valid_pose_pct_stored": round(valid_stored_raw, 2),
            "valid_pose_pct_replayed": round(valid_replayed_raw, 2),
            "exact_match_frac": round(exact_match_raw, 2),
            "total_active_frames": n,
            "matching_frames": int((act_stored["num_inliers_pose"].values == act_replayed["num_inliers_pose"].values).sum()),
        })

        # 2. GATED Mode Replay (gate_thresh = 15.0)
        gated_stored_csv = ds_dir / "eis_gated_vo.csv"
        gated_out_csv = SCRATCH_DIR / f"{run_name}_gated_replayed.csv"

        t0 = time.time()
        cmd_gated = f"python3 {REPO_ROOT}/src/pipelines/run_offline_vo.py --dataset-dir {ds_dir} --gt-csv {gt_csv} --output-csv {gated_out_csv} --eis --eis-mode gated --gate-thresh 15.0 > /dev/null 2>&1"
        os.system(cmd_gated)
        t_gated = time.time() - t0
        runtimes.append(t_gated)

        df_gated_stored = pd.read_csv(gated_stored_csv)
        df_gated_replayed = pd.read_csv(gated_out_csv)

        m_gated_stored = (df_gated_stored["timestamp_total_sec"] >= t_start) & (df_gated_stored["timestamp_total_sec"] <= t_end)
        m_gated_replayed = (df_gated_replayed["timestamp_total_sec"] >= t_start) & (df_gated_replayed["timestamp_total_sec"] <= t_end)

        act_g_stored = df_gated_stored[m_gated_stored].reset_index(drop=True)
        act_g_replayed = df_gated_replayed[m_gated_replayed].reset_index(drop=True)
        ng = min(len(act_g_stored), len(act_g_replayed))
        act_g_stored, act_g_replayed = act_g_stored.iloc[:ng], act_g_replayed.iloc[:ng]

        valid_stored_gated = (act_g_stored["num_inliers_pose"] >= 8).mean() * 100.0
        valid_replayed_gated = (act_g_replayed["num_inliers_pose"] >= 8).mean() * 100.0
        exact_match_gated = (act_g_stored["num_inliers_pose"].values == act_g_replayed["num_inliers_pose"].values).mean() * 100.0

        records.append({
            "run": run_name,
            "mode": "EIS-GATED",
            "gate_thresh_deg": 15.0,
            "runtime_sec": round(t_gated, 2),
            "valid_pose_pct_stored": round(valid_stored_gated, 2),
            "valid_pose_pct_replayed": round(valid_replayed_gated, 2),
            "exact_match_frac": round(exact_match_gated, 2),
            "total_active_frames": ng,
            "matching_frames": int((act_g_stored["num_inliers_pose"].values == act_g_replayed["num_inliers_pose"].values).sum()),
        })

    df_out = pd.DataFrame(records)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    df_out.to_csv(OUTPUT, index=False)

    avg_time = np.mean(runtimes)
    est_18_runs = 18 * avg_time

    print("========================================================")
    print("THRESHOLD SWEEP SANITY GATE REPLAY RESULTS")
    print("========================================================")
    print(df_out[["run", "mode", "runtime_sec", "valid_pose_pct_stored", "valid_pose_pct_replayed", "exact_match_frac", "matching_frames", "total_active_frames"]].to_string(index=False))

    print(f"\nSingle-run average wall-clock time: {avg_time:.2f} seconds")
    print(f"Estimated wall-clock time for 18-run sweep: {est_18_runs / 60:.2f} minutes ({est_18_runs:.1f} seconds)")
    print(f"\nWrote results to {OUTPUT.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    run_sanity_gate()
