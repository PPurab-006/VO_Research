#!/usr/bin/env python3
"""
Phase 3 Dataset Recording & Processing Orchestrator

Sequentially records missing raw camera datasets in agriculture.world using PX4 SITL.
Enforces hard gates on active window altitude (Z >= 2.0m), flight duration (>= 18.0s), and image capture.

Separates dataset namespaces:
  - CORE Matrix: p3_{family}_{severity}_raw_{run} (F1, F2, F4, F6, F10, F11)
  - EXPLORATORY Matrix: p3x_{family}_{severity}_raw_{run} (F3, F7, F8, HOVER)

After each raw recording, runs offline VO for all 3 mechanisms:
  1. RAW: raw_vo.csv
  2. EIS-GATED: eis_gated_vo.csv (15.0 deg/s threshold)
  3. DELAYED-TRI: gated_dt_def_a_vo.csv (min_non_r_obs = 3)
"""

import argparse
import os
import sys
import time
import subprocess
import pandas as pd
import numpy as np

from record_single_phase2a_dataset import record_dataset, kill_all_sim_processes


def verify_dataset_hard_gates(dataset_dir, family=None, min_duration=18.0, min_alt=2.0):
    gt_csv = os.path.join(dataset_dir, "dataset_gt.csv")
    cam_csv = os.path.join(dataset_dir, "camera_frames.csv")
    img_dir = os.path.join(dataset_dir, "images")

    if not os.path.exists(gt_csv) or not os.path.exists(cam_csv) or not os.path.exists(img_dir):
        print(f"[FAIL] Missing files in dataset directory {dataset_dir}")
        return False, "Missing files"

    df_gt = pd.read_csv(gt_csv)
    df_cam = pd.read_csv(cam_csv)

    if len(df_gt) < 100 or len(df_cam) < 100:
        print(f"[FAIL] Dataset {dataset_dir} has insufficient records (GT: {len(df_gt)}, Cam: {len(df_cam)})")
        return False, "Insufficient frame count"

    gt_t = df_gt['timestamp_total_sec'].values.astype(float)
    z_gt = df_gt['pos_z'].values.astype(float) if 'pos_z' in df_gt.columns else df_gt['z'].values.astype(float)

    idx_act = np.where(z_gt >= min_alt)[0]
    if len(idx_act) < 10:
        print(f"[FAIL] Dataset {dataset_dir} failed altitude hard gate (Z >= {min_alt}m never achieved)")
        return False, "Altitude gate failed"

    t_act_start = gt_t[idx_act[0]]
    t_act_end = gt_t[idx_act[-1]]
    act_dur = t_act_end - t_act_start

    if act_dur < min_duration:
        print(f"[FAIL] Dataset {dataset_dir} failed active duration gate ({act_dur:.1f}s < {min_duration}s)")
        return False, "Duration gate failed"

    # Check images exist
    n_imgs = len([f for f in os.listdir(img_dir) if f.endswith('.png')])
    if n_imgs < len(df_cam) - 5:
        print(f"[FAIL] Dataset {dataset_dir} image count mismatch ({n_imgs} PNGs vs {len(df_cam)} CSV rows)")
        return False, "Image count mismatch"

    # Hard Achieved-Severity Gates for F6 and F10 (SET_ATTITUDE_TARGET)
    if family == 'F6':
        from scipy.spatial.transform import Rotation as R
        quats = df_gt[['rot_x', 'rot_y', 'rot_z', 'rot_w']].values[idx_act]
        euler = R.from_quat(quats).as_euler('xyz', degrees=True)
        yaws = euler[:, 2]

        idx_motion = (gt_t[idx_act] >= t_act_start + 1.0) & (gt_t[idx_act] <= t_act_end - 2.0)
        if np.sum(idx_motion) > 10:
            yaws_m = yaws[idx_motion]
            dt_act = np.diff(gt_t[idx_act][idx_motion])
        else:
            yaws_m = yaws
            dt_act = np.diff(gt_t[idx_act])

        p2p_yaw = float(np.max(yaws_m) - np.min(yaws_m))
        half_len = len(yaws_m) // 2
        h1 = float(np.mean(yaws_m[:half_len]))
        h2 = float(np.mean(yaws_m[half_len:]))
        mean_heading_drift = abs(h1 - h2)
        dyaw = np.diff(yaws_m)
        dyaw = (dyaw + 180) % 360 - 180
        dt_act = np.maximum(dt_act, 1e-4)
        yaw_rates = dyaw / dt_act
        sign_changes = int(np.sum(np.diff(np.sign(yaw_rates[np.abs(yaw_rates) > 5.0])) != 0))

        if not (40.0 <= p2p_yaw <= 125.0):
            print(f"[FAIL] F6 achieved p2p yaw amplitude gate failed ({p2p_yaw:.1f} deg not in 40-125 deg range)")
            return False, f"F6 p2p yaw amplitude gate failed ({p2p_yaw:.1f} deg)"
        if mean_heading_drift > 20.0:
            print(f"[FAIL] F6 mean heading drift gate failed ({mean_heading_drift:.1f} deg > 20 deg)")
            return False, f"F6 mean heading drift gate failed ({mean_heading_drift:.1f} deg)"
        if sign_changes < 4:
            print(f"[FAIL] F6 sign reversals gate failed ({sign_changes} sign changes < 4)")
            return False, f"F6 oscillation gate failed ({sign_changes} sign changes)"
        print(f"[PASS] F6 Achieved Severity Verified (p2p yaw: {p2p_yaw:.1f} deg, mean heading drift: {mean_heading_drift:.1f} deg, sign reversals: {sign_changes})")

    elif family == 'F10':
        from scipy.spatial.transform import Rotation as R
        quats = df_gt[['rot_x', 'rot_y', 'rot_z', 'rot_w']].values[idx_act]
        euler = R.from_quat(quats).as_euler('xyz', degrees=True)
        yaws = euler[:, 2]

        idx_motion = (gt_t[idx_act] >= t_act_start + 1.0) & (gt_t[idx_act] <= t_act_end - 2.0)
        if np.sum(idx_motion) > 10:
            yaws_m = yaws[idx_motion]
            dt_act = np.diff(gt_t[idx_act][idx_motion])
        else:
            yaws_m = yaws
            dt_act = np.diff(gt_t[idx_act])

        p2p_yaw = float(np.max(yaws_m) - np.min(yaws_m))
        dyaw = np.diff(yaws_m)
        dyaw = (dyaw + 180) % 360 - 180
        dt_act = np.maximum(dt_act, 1e-4)
        yaw_rates = np.abs(dyaw / dt_act)
        max_yaw_rate = float(np.max(yaw_rates))

        if p2p_yaw < 30.0 or max_yaw_rate < 30.0:
            print(f"[FAIL] F10 achieved severity gate failed (p2p: {p2p_yaw:.1f} deg, max_rate: {max_yaw_rate:.1f} deg/s)")
            return False, f"F10 severity gate failed (p2p: {p2p_yaw:.1f} deg, max_rate: {max_yaw_rate:.1f} deg/s)"
        print(f"[PASS] F10 Achieved Severity Verified (p2p yaw: {p2p_yaw:.1f} deg, max yaw rate: {max_yaw_rate:.1f} deg/s)")

    print(f"[PASS] Hard gates verified for {dataset_dir} (Duration: {act_dur:.1f}s, Max Alt: {np.max(z_gt):.2f}m, Frames: {n_imgs})")
    return True, "Passed"


def run_offline_vo_all_mechs(dataset_dir):
    gt_csv = os.path.join(dataset_dir, "dataset_gt.csv")
    raw_vo_csv = os.path.join(dataset_dir, "raw_vo.csv")
    gated_vo_csv = os.path.join(dataset_dir, "eis_gated_vo.csv")
    dt_vo_csv = os.path.join(dataset_dir, "gated_dt_def_a_vo.csv")

    cmd_base = [sys.executable, "src/phase2/run_offline_vo.py", "--dataset-dir", dataset_dir, "--gt-csv", gt_csv]

    # 1. RAW VO
    print(f"  Running Offline VO: RAW -> {raw_vo_csv}...")
    cmd_raw = cmd_base + ["--output-csv", raw_vo_csv]
    subprocess.run(cmd_raw, check=True)

    # 2. EIS-GATED VO
    print(f"  Running Offline VO: EIS-GATED -> {gated_vo_csv}...")
    cmd_gated = cmd_base + ["--output-csv", gated_vo_csv, "--eis", "--eis-mode", "gated", "--gate-thresh", "15.0"]
    subprocess.run(cmd_gated, check=True)

    # 3. DELAYED-TRI VO
    print(f"  Running Offline VO: DELAYED-TRI -> {dt_vo_csv}...")
    cmd_dt = cmd_base + ["--output-csv", dt_vo_csv, "--eis", "--eis-mode", "gated", "--gate-thresh", "15.0",
                         "--delayed-triangulation", "--r-frame-def", "yaw_rate", "--min-non-r-obs", "3"]
    subprocess.run(cmd_dt, check=True)

    print(f"  [SUCCESS] All 3 offline VO mechanisms processed for {dataset_dir}")


def record_and_process_batch(target_list, overwrite=False):
    for item in target_list:
        fam = item['family']
        sev = item['severity']
        run_idx = item['run']
        prefix = item['prefix'] # 'p3' for Core, 'p3x' for Exploratory

        run_id = f"{prefix}_{fam}_L{sev}_R{run_idx}"
        dataset_dir = f"results/datasets/{run_id}"

        print(f"\n==========================================================================")
        print(f"RECORDING & PROCESSING CELL: {run_id} ({item['type'].upper()} TRACK)")
        print(f"==========================================================================")

        if not overwrite and os.path.exists(os.path.join(dataset_dir, "gated_dt_def_a_vo.csv")):
            print(f"Dataset {run_id} already fully processed. Skipping re-recording.")
            continue

        if overwrite or not os.path.exists(os.path.join(dataset_dir, "dataset_gt.csv")):
            # Record raw flight dataset
            if os.path.exists(dataset_dir):
                import shutil
                shutil.rmtree(dataset_dir)
            record_dataset(family=fam, severity=sev, duration=20.0, run_id=run_id)

        # Verify hard gates
        passed, msg = verify_dataset_hard_gates(dataset_dir, family=fam)
        if not passed:
            print(f"[CRITICAL ERROR] Dataset {run_id} failed hard gate check: {msg}")
            print("Stopping batch run for manual inspection per project protocol.")
            sys.exit(1)

        # Process offline VO
        run_offline_vo_all_mechs(dataset_dir)


def main():
    parser = argparse.ArgumentParser(description="Phase 3 Batch Recording & Processing Orchestrator")
    parser.add_argument('--track', choices=['core', 'exploratory', 'f6_f10', 'all'], default='all', help="Track to record")
    parser.add_argument('--overwrite', action='store_true', help="Force re-recording and re-processing")
    args = parser.parse_args()

    import numpy as np
    globals()['np'] = np

    core_targets = [
        {'family': 'F1', 'severity': 2, 'run': 1, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F1', 'severity': 2, 'run': 2, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F1', 'severity': 2, 'run': 3, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F2', 'severity': 2, 'run': 1, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F2', 'severity': 2, 'run': 2, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F2', 'severity': 2, 'run': 3, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F4', 'severity': 2, 'run': 1, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F4', 'severity': 2, 'run': 2, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F4', 'severity': 2, 'run': 3, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F6', 'severity': 2, 'run': 1, 'prefix': 'p3', 'type': 'core'}, # Fresh per Amendment 1
        {'family': 'F6', 'severity': 2, 'run': 2, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F6', 'severity': 2, 'run': 3, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F10', 'severity': 3, 'run': 1, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F10', 'severity': 3, 'run': 2, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F10', 'severity': 3, 'run': 3, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F11', 'severity': 2, 'run': 1, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F11', 'severity': 2, 'run': 2, 'prefix': 'p3', 'type': 'core'},
        {'family': 'F11', 'severity': 2, 'run': 3, 'prefix': 'p3', 'type': 'core'},
    ]

    exploratory_targets = [
        {'family': 'HOVER', 'severity': 0, 'run': 1, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'HOVER', 'severity': 0, 'run': 2, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'HOVER', 'severity': 0, 'run': 3, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F3', 'severity': 2, 'run': 1, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F3', 'severity': 2, 'run': 2, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F3', 'severity': 2, 'run': 3, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F7', 'severity': 2, 'run': 1, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F7', 'severity': 2, 'run': 2, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F7', 'severity': 2, 'run': 3, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F8', 'severity': 2, 'run': 1, 'prefix': 'p3x', 'type': 'exploratory'},
        {'family': 'F8', 'severity': 2, 'run': 8, 'prefix': 'p3x', 'type': 'exploratory'}, # R2
        {'family': 'F8', 'severity': 2, 'run': 3, 'prefix': 'p3x', 'type': 'exploratory'},
    ]
    # Correct run index for F8 R2
    exploratory_targets[10]['run'] = 2

    targets = []
    if args.track == 'f6_f10':
        targets = [t for t in core_targets if t['family'] in ['F6', 'F10']]
    else:
        if args.track in ['core', 'all']:
            targets.extend(core_targets)
        if args.track in ['exploratory', 'all']:
            targets.extend(exploratory_targets)

    record_and_process_batch(targets, overwrite=args.overwrite)


if __name__ == '__main__':
    main()
