#!/usr/bin/env python3
"""
Phase 0D Evaluation Script: Sim(3)/Umeyama Alignment, ATE, and RPE for v3-rerun.

Evaluates monocular VO trajectory against ground truth for circular_flight_v3_rerun:
- Restricts analysis to active flight frames (pos_z > 0.3m).
- Uses relative timestamp matching to synchronize streams.
- Performs Sim(3) Umeyama alignment (rotation, translation, scale estimation) via `evo`.
- Computes Absolute Trajectory Error (ATE) and Relative Pose Error (RPE) for translation and rotation.
- Generates publication-ready plots saved to `plots/v3_rerun_aligned_trajectory.png` and `plots/v3_rerun_ate_over_time.png`.
"""

import os
import csv
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import evo.core.geometry as geometry
from evo.core.trajectory import PoseTrajectory3D
import evo.core.metrics as metrics

def load_csv(filepath):
    records = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)
    return records

def main():
    vo_path = 'results/circular_flight_v3_rerun_vo.csv'
    gt_path = 'results/circular_flight_v3_rerun_gt.csv'
    output_dir = 'plots'
    os.makedirs(output_dir, exist_ok=True)

    print("==========================================================================================")
    print("PHASE 0D CORE DELIVERABLE: SIM(3) / UMEYAMA ALIGNMENT & TRAJECTORY EVALUATION (V3-RERUN)")
    print("==========================================================================================")

    vo_records = load_csv(vo_path)
    gt_records = load_csv(gt_path)

    print(f"Loaded VO Records : {len(vo_records)} frames ({vo_path})")
    print(f"Loaded GT Records : {len(gt_records)} frames ({gt_path})")

    # 1. Filter to Active Flight (pos_z > 0.3m)
    active_vo = [r for r in vo_records if float(r['pos_z']) > 0.3]
    print(f"Active Flight VO Frames (pos_z > 0.3m): {len(active_vo)}")

    # 2. Time-Synchronize using Relative Timestamps (t - t0)
    vo_ts = np.array([float(r['timestamp_total_sec']) for r in active_vo])
    gt_ts = np.array([float(r['timestamp_total_sec']) for r in gt_records])

    vo_rel_ts = vo_ts - vo_ts[0]
    gt_rel_ts = gt_ts - gt_ts[0]

    matched_vo = []
    matched_gt = []

    for idx, r_vo in enumerate(active_vo):
        t_vo = vo_rel_ts[idx]
        diffs = np.abs(gt_rel_ts - t_vo)
        best_idx = np.argmin(diffs)
        if diffs[best_idx] < 0.1:  # 100ms tolerance
            matched_vo.append(r_vo)
            matched_gt.append(gt_records[best_idx])

    print(f"Synchronized Matched Pairs: {len(matched_vo)} / {len(active_vo)} active frames")

    # 3. Build evo PoseTrajectory3D Objects
    vo_xyz = np.array([[float(r['pos_x']), float(r['pos_y']), float(r['pos_z'])] for r in matched_vo], dtype=np.float64)
    gt_xyz = np.array([[float(r['pos_x']), float(r['pos_y']), float(r['pos_z'])] for r in matched_gt], dtype=np.float64)

    vo_quat_xyzw = np.array([[float(r['rot_x']), float(r['rot_y']), float(r['rot_z']), float(r['rot_w'])] for r in matched_vo], dtype=np.float64)
    gt_quat_xyzw = np.array([[float(r['rot_x']), float(r['rot_y']), float(r['rot_z']), float(r['rot_w'])] for r in matched_gt], dtype=np.float64)

    # Convert (x, y, z, w) -> (w, x, y, z) for evo
    vo_quat_wxyz = vo_quat_xyzw[:, [3, 0, 1, 2]]
    gt_quat_wxyz = gt_quat_xyzw[:, [3, 0, 1, 2]]

    rel_timestamps = vo_rel_ts

    traj_vo_raw = PoseTrajectory3D(positions_xyz=vo_xyz, orientations_quat_wxyz=vo_quat_wxyz, timestamps=rel_timestamps)
    traj_gt = PoseTrajectory3D(positions_xyz=gt_xyz, orientations_quat_wxyz=gt_quat_wxyz, timestamps=rel_timestamps)

    # 4. Compute Sim(3) Umeyama Alignment
    R, t, scale = geometry.umeyama_alignment(traj_vo_raw.positions_xyz.T, traj_gt.positions_xyz.T, with_scale=True)

    print("\n--- RECOVERED SIM(3) / UMEYAMA ALIGNMENT PARAMETERS ---")
    print(f"  Recovered Scale Factor (s) : {scale:.6f}")
    print(f"  Translation Vector (t)     : [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m")
    print(f"  Rotation Matrix (R):\n{R}")

    # Scale Factor Sanity Check
    if 0.1 <= scale <= 10.0:
        print(f"  Scale Factor Sanity Check  : PASS (Physical scale s = {scale:.6f} is well-behaved)")
    else:
        print(f"  Scale Factor Sanity Check  : WARNING (Scale s = {scale:.6f} is unusual)")

    # Perform alignment in evo
    traj_vo_aligned = PoseTrajectory3D(positions_xyz=vo_xyz, orientations_quat_wxyz=vo_quat_wxyz, timestamps=rel_timestamps)
    traj_vo_aligned.align(traj_gt, correct_scale=True)

    # 5. Compute Absolute Trajectory Error (ATE) - Translation
    ape_trans = metrics.APE(metrics.PoseRelation.translation_part)
    ape_trans.process_data((traj_gt, traj_vo_aligned))
    ate_stats = ape_trans.get_all_statistics()
    ate_series = ape_trans.error

    print("\n--- ABSOLUTE TRAJECTORY ERROR (ATE) STATISTICS (TRANSLATION, METERS) ---")
    print(f"  RMSE   : {ate_stats['rmse']:.4f} m")
    print(f"  Mean   : {ate_stats['mean']:.4f} m")
    print(f"  Median : {ate_stats['median']:.4f} m")
    print(f"  StdDev : {ate_stats['std']:.4f} m")
    print(f"  Min    : {ate_stats['min']:.4f} m")
    print(f"  Max    : {ate_stats['max']:.4f} m")

    # 6. Compute Relative Pose Error (RPE) - Translation
    rpe_trans_1 = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
    rpe_trans_1.process_data((traj_gt, traj_vo_aligned))
    rpe_1_stats = rpe_trans_1.get_all_statistics()

    rpe_trans_10 = metrics.RPE(metrics.PoseRelation.translation_part, delta=10, delta_unit=metrics.Unit.frames)
    rpe_trans_10.process_data((traj_gt, traj_vo_aligned))
    rpe_10_stats = rpe_trans_10.get_all_statistics()

    print("\n--- RELATIVE POSE ERROR (RPE) STATISTICS (TRANSLATION, METERS) ---")
    print("  [Delta = 1 Frame]:")
    print(f"    RMSE   : {rpe_1_stats['rmse']:.4f} m")
    print(f"    Mean   : {rpe_1_stats['mean']:.4f} m")
    print(f"    Median : {rpe_1_stats['median']:.4f} m")
    print(f"    StdDev : {rpe_1_stats['std']:.4f} m")
    print(f"    Min    : {rpe_1_stats['min']:.4f} m")
    print(f"    Max    : {rpe_1_stats['max']:.4f} m")
    print("  [Delta = 10 Frames]:")
    print(f"    RMSE   : {rpe_10_stats['rmse']:.4f} m")
    print(f"    Mean   : {rpe_10_stats['mean']:.4f} m")
    print(f"    Median : {rpe_10_stats['median']:.4f} m")
    print(f"    StdDev : {rpe_10_stats['std']:.4f} m")
    print(f"    Min    : {rpe_10_stats['min']:.4f} m")
    print(f"    Max    : {rpe_10_stats['max']:.4f} m")

    # 7. Compute Relative Pose Error (RPE) - Rotation (Angle Deg)
    rpe_rot_1 = metrics.RPE(metrics.PoseRelation.rotation_angle_deg, delta=1, delta_unit=metrics.Unit.frames)
    rpe_rot_1.process_data((traj_gt, traj_vo_aligned))
    rot_1_stats = rpe_rot_1.get_all_statistics()

    rpe_rot_10 = metrics.RPE(metrics.PoseRelation.rotation_angle_deg, delta=10, delta_unit=metrics.Unit.frames)
    rpe_rot_10.process_data((traj_gt, traj_vo_aligned))
    rot_10_stats = rpe_rot_10.get_all_statistics()

    print("\n--- RELATIVE POSE ERROR (RPE) STATISTICS (ROTATION, DEGREES) ---")
    print("  [Delta = 1 Frame]:")
    print(f"    RMSE   : {rot_1_stats['rmse']:.4f} deg")
    print(f"    Mean   : {rot_1_stats['mean']:.4f} deg")
    print(f"    Median : {rot_1_stats['median']:.4f} deg")
    print(f"    StdDev : {rot_1_stats['std']:.4f} deg")
    print(f"    Min    : {rot_1_stats['min']:.4f} deg")
    print(f"    Max    : {rot_1_stats['max']:.4f} deg")
    print("  [Delta = 10 Frames]:")
    print(f"    RMSE   : {rot_10_stats['rmse']:.4f} deg")
    print(f"    Mean   : {rot_10_stats['mean']:.4f} deg")
    print(f"    Median : {rot_10_stats['median']:.4f} deg")
    print(f"    StdDev : {rot_10_stats['std']:.4f} deg")
    print(f"    Min    : {rot_10_stats['min']:.4f} deg")
    print(f"    Max    : {rot_10_stats['max']:.4f} deg")

    # 8. Generate Plot 1: Trajectory Overlay Plot
    fig, ax = plt.subplots(figsize=(9, 7))
    gt_xy = traj_gt.positions_xyz
    vo_xy = traj_vo_aligned.positions_xyz

    ax.plot(gt_xy[:, 0], gt_xy[:, 1], 'g-', linewidth=2.5, label='Ground Truth Trajectory')
    ax.plot(vo_xy[:, 0], vo_xy[:, 1], 'r--', linewidth=2.0, label=f'Aligned VO Trajectory (Sim3, s={scale:.2f})')
    ax.scatter(gt_xy[0, 0], gt_xy[0, 1], c='green', marker='o', s=90, zorder=5, label='Start Point (GT)')
    ax.scatter(vo_xy[0, 0], vo_xy[0, 1], c='red', marker='x', s=90, zorder=5, label='Start Point (VO)')

    ax.set_title('Phase 0D Deliverable: v3-Rerun Monocular VO vs. Ground Truth (Sim3 Aligned)', fontsize=12, fontweight='bold')
    ax.set_xlabel('World X (East) [m]', fontsize=11)
    ax.set_ylabel('World Y (North) [m]', fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10, loc='best')
    ax.axis('equal')

    plot1_path = os.path.join(output_dir, 'v3_rerun_aligned_trajectory.png')
    plt.savefig(plot1_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\nSaved Trajectory Overlay Plot -> {os.path.abspath(plot1_path)}")

    # 9. Generate Plot 2: ATE Error Over Frame Index Plot
    fig, ax = plt.subplots(figsize=(9, 5))
    frames = np.arange(len(ate_series))
    ax.plot(frames, ate_series, 'b-', linewidth=1.8, label='Per-Frame ATE (m)')
    ax.axhline(ate_stats['rmse'], color='r', linestyle='--', linewidth=1.5, label=f"RMSE ATE ({ate_stats['rmse']:.3f} m)")
    ax.axhline(ate_stats['mean'], color='orange', linestyle=':', linewidth=1.5, label=f"Mean ATE ({ate_stats['mean']:.3f} m)")

    ax.set_title('Phase 0D Deliverable: Absolute Trajectory Error (ATE) Over Frame Index (v3-Rerun)', fontsize=12, fontweight='bold')
    ax.set_xlabel('Active Frame Index', fontsize=11)
    ax.set_ylabel('ATE Translation Error [m]', fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10, loc='upper left')

    plot2_path = os.path.join(output_dir, 'v3_rerun_ate_over_time.png')
    plt.savefig(plot2_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved ATE Error Over Time Plot -> {os.path.abspath(plot2_path)}")

    print("==========================================================================================")

if __name__ == '__main__':
    main()
