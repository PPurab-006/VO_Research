#!/usr/bin/env python3
"""
Trajectory Alignment & Evaluation Pipeline for Monocular VO (Phase 0D)

Loads raw unit-scale VO trajectory and Ground-Truth trajectory CSVs,
performs timestamp synchronization (20ms tolerance), executes Sim(3)/Umeyama
alignment (rotation + translation + scale estimation) via `evo`, computes
Absolute Trajectory Error (ATE) and Relative Pose Error (RPE), and exports
overlaid trajectory and error-over-time plots to `plots/`.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import evo.core.trajectory as trajectory
import evo.core.metrics as metrics
import evo.core.geometry as geometry
from evo.core.trajectory import PoseTrajectory3D


def associate_timestamps(vo_records, gt_records, tolerance_ms=20.0):
    """
    Nearest-neighbor timestamp matching between VO frames and Ground-Truth poses.
    Reuses the 20ms sync tolerance matching logic from verify_sync.py (Phase 0B).
    """
    gt_ts = np.array([float(g['timestamp_total_sec']) for g in gt_records])
    matched_vo = []
    matched_gt = []
    gaps_ms = []

    for vo in vo_records:
        t_vo = float(vo['timestamp_total_sec'])
        diffs = np.abs(gt_ts - t_vo)
        best_idx = np.argmin(diffs)
        min_gap_sec = diffs[best_idx]
        min_gap_ms = min_gap_sec * 1000.0

        if min_gap_ms <= tolerance_ms:
            matched_vo.append(vo)
            matched_gt.append(gt_records[best_idx])
            gaps_ms.append(min_gap_ms)

    return matched_vo, matched_gt, gaps_ms


def load_csv(filepath):
    records = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)
    return records


def records_to_pose_trajectory(records):
    """
    Converts CSV records into an evo PoseTrajectory3D object.
    ROS/CSV quaternion order is (x, y, z, w).
    evo expects orientations_quat_wxyz in (w, x, y, z) order.
    """
    xyz = np.array([[float(r['pos_x']), float(r['pos_y']), float(r['pos_z'])] for r in records], dtype=np.float64)
    quats_xyzw = np.array([[float(r['rot_x']), float(r['rot_y']), float(r['rot_z']), float(r['rot_w'])] for r in records], dtype=np.float64)
    # Convert (x, y, z, w) -> (w, x, y, z)
    quats_wxyz = quats_xyzw[:, [3, 0, 1, 2]]
    timestamps = np.array([float(r['timestamp_total_sec']) for r in records], dtype=np.float64)

    return PoseTrajectory3D(positions_xyz=xyz, orientations_quat_wxyz=quats_wxyz, timestamps=timestamps)


def evaluate(vo_csv, gt_csv, output_dir, tolerance_ms=20.0):
    print("=" * 70)
    print("PHASE 0D — TRAJECTORY ALIGNMENT & EVALUATION PIPELINE")
    print("=" * 70)
    print(f"VO Trajectory CSV   : {os.path.abspath(vo_csv)}")
    print(f"Ground Truth CSV   : {os.path.abspath(gt_csv)}")
    print(f"Plots Output Dir   : {os.path.abspath(output_dir)}")
    print(f"Sync Tolerance     : {tolerance_ms:.1f} ms")
    print("-" * 70)

    os.makedirs(output_dir, exist_ok=True)

    vo_records = load_csv(vo_csv)
    gt_records = load_csv(gt_csv)

    print(f"Loaded {len(vo_records)} VO records and {len(gt_records)} GT records.")

    matched_vo, matched_gt, gaps_ms = associate_timestamps(vo_records, gt_records, tolerance_ms=tolerance_ms)
    print(f"Synchronized Matched Pairs: {len(matched_vo)} / {len(vo_records)} frames (Mean Gap: {np.mean(gaps_ms):.2f} ms)")

    if len(matched_vo) < 10:
        raise ValueError("Insufficient synchronized frames for evaluation (< 10 pairs).")

    # Build evo trajectory objects
    traj_vo_raw = records_to_pose_trajectory(matched_vo)
    traj_gt = records_to_pose_trajectory(matched_gt)

    # 1. Sim(3) / Umeyama Alignment (Scale + Rotation + Translation)
    R, t, scale = geometry.umeyama_alignment(traj_vo_raw.positions_xyz.T, traj_gt.positions_xyz.T, with_scale=True)

    print("\n" + "-" * 70)
    print("ESTIMATED SIM(3) ALIGNMENT PARAMETERS (Umeyama Scale + Rotation + Translation):")
    print(f"  Estimated Scale Factor (s) : {scale:.6f}")
    print(f"  Estimated Translation (t)  : [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}] m")
    print(f"  Estimated Rotation Matrix R:\n{R}")

    # Sanity Check Scale Factor
    if scale <= 0 or math.isnan(scale) or math.isinf(scale):
        print("\n[WARNING/CRITICAL]: Invalid scale factor estimated! Scale must be positive.")
    else:
        print(f"  Scale Factor Sanity Check  : PASS (Valid positive scale factor: s = {scale:.6f})")

    # Create aligned copy of VO trajectory
    traj_vo_aligned = records_to_pose_trajectory(matched_vo)
    traj_vo_aligned.align(traj_gt, correct_scale=True)

    # 2. Compute Absolute Trajectory Error (ATE)
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data((traj_gt, traj_vo_aligned))
    ate_stats = ape_metric.get_all_statistics()
    ate_error_series = ape_metric.error

    print("\n" + "-" * 70)
    print("ABSOLUTE TRAJECTORY ERROR (ATE) STATISTICS (Translation Part, Meters):")
    print(f"  RMSE   : {ate_stats['rmse']:.4f} m")
    print(f"  Mean   : {ate_stats['mean']:.4f} m")
    print(f"  Median : {ate_stats['median']:.4f} m")
    print(f"  StdDev : {ate_stats['std']:.4f} m")
    print(f"  Min    : {ate_stats['min']:.4f} m")
    print(f"  Max    : {ate_stats['max']:.4f} m")

    # 3. Compute Relative Pose Error (RPE) for Delta = 1 frame and Delta = 10 frames
    rpe_1_metric = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
    rpe_1_metric.process_data((traj_gt, traj_vo_aligned))
    rpe_1_stats = rpe_1_metric.get_all_statistics()

    rpe_10_metric = metrics.RPE(metrics.PoseRelation.translation_part, delta=10, delta_unit=metrics.Unit.frames)
    rpe_10_metric.process_data((traj_gt, traj_vo_aligned))
    rpe_10_stats = rpe_10_metric.get_all_statistics()

    print("\n" + "-" * 70)
    print("RELATIVE POSE ERROR (RPE) STATISTICS (Translation Part, Meters):")
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

    # 4. Generate Plot 1: Trajectory Overlay (X-Y Top-Down View + 3D View)
    fig, ax = plt.subplots(figsize=(9, 7))
    gt_xyz = traj_gt.positions_xyz
    vo_aligned_xyz = traj_vo_aligned.positions_xyz

    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], 'g-', linewidth=2.5, label='Ground Truth Trajectory')
    ax.plot(vo_aligned_xyz[:, 0], vo_aligned_xyz[:, 1], 'r--', linewidth=2.0, label='Aligned VO Trajectory (Sim3)')
    ax.scatter(gt_xyz[0, 0], gt_xyz[0, 1], c='green', marker='o', s=80, label='Start Point (GT)')
    ax.scatter(vo_aligned_xyz[0, 0], vo_aligned_xyz[0, 1], c='red', marker='x', s=80, label='Start Point (VO)')

    ax.set_title('Phase 0D: Monocular VO vs. Ground-Truth Trajectory (Sim3 Aligned)', fontsize=13, fontweight='bold')
    ax.set_xlabel('World X (East) [m]', fontsize=11)
    ax.set_ylabel('World Y (North) [m]', fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10, loc='best')
    ax.axis('equal')

    plot1_path = os.path.join(output_dir, 'trajectory_overlay.png')
    plt.savefig(plot1_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\nSaved Trajectory Overlay Plot -> {os.path.abspath(plot1_path)}")

    # 5. Generate Plot 2: ATE Error Over Time / Frame Index
    fig, ax = plt.subplots(figsize=(9, 5))
    frames = np.arange(len(ate_error_series))
    ax.plot(frames, ate_error_series, 'b-', linewidth=1.8, label='Per-Frame ATE (m)')
    ax.axhline(ate_stats['rmse'], color='r', linestyle='--', linewidth=1.5, label=f"RMSE ATE ({ate_stats['rmse']:.3f} m)")
    ax.axhline(ate_stats['mean'], color='orange', linestyle=':', linewidth=1.5, label=f"Mean ATE ({ate_stats['mean']:.3f} m)")

    ax.set_title('Phase 0D: Absolute Trajectory Error (ATE) Over Frame Index', fontsize=13, fontweight='bold')
    ax.set_xlabel('Frame Index', fontsize=11)
    ax.set_ylabel('ATE Translation Error [m]', fontsize=11)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10, loc='upper left')

    plot2_path = os.path.join(output_dir, 'ate_error_over_time.png')
    plt.savefig(plot2_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved ATE Error Over Time Plot -> {os.path.abspath(plot2_path)}")

    print("=" * 70 + "\n")

    return {
        'scale': scale,
        'R': R,
        't': t,
        'ate_stats': ate_stats,
        'rpe_1_stats': rpe_1_stats,
        'rpe_10_stats': rpe_10_stats
    }


def main():
    parser = argparse.ArgumentParser(description="Evaluate Monocular VO Trajectory against Ground-Truth (Phase 0D)")
    parser.add_argument(
        '--vo-csv',
        default='results/vo_trajectory_textured.csv',
        help="Path to raw VO trajectory CSV"
    )
    parser.add_argument(
        '--gt-csv',
        default='results/ground_truth_textured.csv',
        help="Path to Ground-Truth pose CSV"
    )
    parser.add_argument(
        '--output-dir',
        default='plots',
        help="Directory to save evaluation plots"
    )
    parser.add_argument(
        '--tolerance-ms',
        type=float,
        default=20.0,
        help="Timestamp sync tolerance in ms"
    )

    args = parser.parse_args()
    evaluate(args.vo_csv, args.gt_csv, args.output_dir, tolerance_ms=args.tolerance_ms)


if __name__ == '__main__':
    main()
