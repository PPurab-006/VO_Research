#!/usr/bin/env python3
"""
Phase 0E Sweep A Analysis & Visualization Script

Processes the 6 dataset pairs (10, 20, 40, 80, 120, 180 deg/s), computes summary metrics:
- Mean Inlier Count
- Mean Feature Survival Rate (%)
- Mean & Max Image-Space Feature Velocity (px/frame)
- Operational Failure Detection (num_inliers < 5 for 10+ consecutive frames)

Prints summary table and generates plots/yaw_sweep_characterization.png.
"""

import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

YAW_RATES = [10, 20, 40, 80, 120, 180]

def load_csv(filepath):
    records = []
    if not os.path.exists(filepath):
        return records
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)
    return records

def process_yaw_level(rate):
    vo_path = f"results/yaw_sweep_v2_{rate}dps_vo.csv"
    gt_path = f"results/yaw_sweep_v2_{rate}dps_gt.csv"

    vo_records = load_csv(vo_path)
    gt_records = load_csv(gt_path)

    if not vo_records or not gt_records:
        return None

    # Compute total VO duration & achieved camera FPS
    vo_ts = np.array([float(r['timestamp_total_sec']) for r in vo_records])
    vo_dur = vo_ts[-1] - vo_ts[0] if len(vo_ts) > 1 else 0.0
    achieved_fps = float(len(vo_records) / vo_dur) if vo_dur > 0 else 0.0

    # Filter GT records to active flight (pos_z > 0.3m)
    act_gt = [r for r in gt_records if float(r['pos_z']) > 0.3]
    if not act_gt:
        act_gt = gt_records

    # Match VO frames to active GT flight using relative timestamps
    gt_ts = np.array([float(r['timestamp_total_sec']) for r in gt_records])
    vo_rel_ts = vo_ts - vo_ts[0]
    gt_rel_ts = gt_ts - gt_ts[0]

    active_vo = []
    for idx, r_vo in enumerate(vo_records):
        t_vo = vo_rel_ts[idx]
        best_idx = np.argmin(np.abs(gt_rel_ts - t_vo))
        if float(gt_records[best_idx]['pos_z']) > 0.3:
            active_vo.append(r_vo)

    if not active_vo:
        active_vo = vo_records

    total_active = len(active_vo)
    inlier_counts = [int(r['num_inliers']) for r in active_vo]
    inlier_ratios = [float(r['inlier_ratio']) * 100.0 for r in active_vo]
    survival_rates = [float(r.get('feature_survival_rate', 0.0)) * 100.0 for r in active_vo]
    vel_means = [float(r.get('feature_vel_mean', 0.0)) for r in active_vo]
    vel_maxs = [float(r.get('feature_vel_max', 0.0)) for r in active_vo]

    # Evaluate 1: Streak Failure (num_inliers < 5 for 10+ consecutive active frames)
    consec_low = 0
    fail_streak = False
    fail_streak_frame = None

    for r in active_vo:
        n_inl = int(r['num_inliers'])
        if n_inl < 5:
            consec_low += 1
        else:
            consec_low = 0
        if consec_low >= 10 and not fail_streak:
            fail_streak = True
            fail_streak_frame = int(r['frame_idx'])

    # Evaluate 2: Robust 2.0s Sliding Window Failure (>70% of frames in prior 2.0s have num_inliers < 5)
    fail_window = False
    fail_window_frame = None
    fail_window_time = None
    max_win_low_pct = 0.0

    active_history = []
    for i, r in enumerate(active_vo):
        t_total = float(r['timestamp_total_sec'])
        n_inl = int(r['num_inliers'])
        active_history.append((t_total, n_inl))

        win_frames = [item for item in active_history if (t_total - item[0]) <= 2.0]
        low_cnt = sum(1 for item in win_frames if item[1] < 5)
        win_pct = float(low_cnt / len(win_frames)) * 100.0 if win_frames else 0.0
        if win_pct > max_win_low_pct:
            max_win_low_pct = win_pct

        if win_pct > 70.0 and not fail_window:
            fail_window = True
            fail_window_frame = int(r['frame_idx'])
            fail_window_time = t_total - float(active_vo[0]['timestamp_total_sec'])

    return {
        'rate': rate,
        'achieved_fps': achieved_fps,
        'total_vo_frames': len(vo_records),
        'total_active': total_active,
        'mean_inliers': np.mean(inlier_counts),
        'mean_survival_rate': np.mean(survival_rates),
        'mean_vel_mean': np.mean(vel_means),
        'max_vel_max': np.max(vel_maxs),
        'fail_streak': fail_streak,
        'fail_streak_frame': fail_streak_frame,
        'fail_window': fail_window,
        'fail_window_frame': fail_window_frame,
        'fail_window_time': fail_window_time,
        'max_win_low_pct': max_win_low_pct
    }

def main():
    print("==========================================================================================")
    print("PHASE 0E SWEEP A V2 ANALYSIS: RE-RUN WITH FPS & SLIDING-WINDOW FAILURE FIXES")
    print("==========================================================================================")

    results = []
    for rate in YAW_RATES:
        res = process_yaw_level(rate)
        if res:
            results.append(res)
        else:
            print(f"[WARNING] Missing v2 dataset for {rate} deg/s!")

    if not results:
        print("[ERROR] No valid v2 dataset files found in results/!")
        return

    # Print Summary Table
    print("\n===============================================================================================================================================")
    print("SWEEP A V2 SUMMARY TABLE: YAW RATE VS. ACHIEVED FPS, PRECURSOR SIGNALS & SLIDING-WINDOW FAILURE STATUS")
    print("===============================================================================================================================================")
    hdr = f"{'Yaw Rate':<10} | {'Camera FPS':<11} | {'Total VO Fr':<12} | {'Active Fr':<10} | {'Mean Inliers':<12} | {'Surv Rate (%)':<14} | {'Mean Vel (px)':<14} | {'Window Fail?':<12} | {'Window Failure Detail':<22}"
    print(hdr)
    print("-" * len(hdr))

    for r in results:
        fail_str = "YES" if r['fail_window'] else "NO"
        detail_str = f"Frame #{r['fail_window_frame']} ({r['fail_window_time']:.2f}s)" if r['fail_window'] else f"Stable (Peak {r['max_win_low_pct']:.1f}%)"
        print(f"{r['rate']:<10.1f} | {r['achieved_fps']:<11.2f} | {r['total_vo_frames']:<12} | {r['total_active']:<10} | {r['mean_inliers']:<12.2f} | {r['mean_survival_rate']:<14.2f}% | {r['mean_vel_mean']:<14.2f} | {fail_str:<12} | {detail_str:<22}")

    print("===============================================================================================================================================\n")

    # Generate Characterization Plot for v2
    os.makedirs('plots', exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    rates = [r['rate'] for r in results]
    fps_vals = [r['achieved_fps'] for r in results]
    surv_rates = [r['mean_survival_rate'] for r in results]
    vel_means = [r['mean_vel_mean'] for r in results]
    inliers = [r['mean_inliers'] for r in results]

    # Panel 1: Achieved Camera FPS & Feature Survival Rate
    color1 = 'tab:blue'
    ax1.set_ylabel('Feature Survival Rate (%)', color=color1, fontsize=11, fontweight='bold')
    line1 = ax1.plot(rates, surv_rates, 'o-', color=color1, linewidth=2.2, markersize=8, label='Feature Survival Rate (%)')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.set_ylim(0, 105)
    ax1.grid(True, linestyle='--', alpha=0.6)

    ax1_twin = ax1.twinx()
    color2 = 'tab:purple'
    ax1_twin.set_ylabel('Achieved Camera FPS (Hz)', color=color2, fontsize=11, fontweight='bold')
    line2 = ax1_twin.plot(rates, fps_vals, 's--', color=color2, linewidth=2.0, markersize=7, label='Achieved Camera FPS')
    ax1_twin.tick_params(axis='y', labelcolor=color2)
    ax1_twin.set_ylim(0, 35)

    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='lower left', fontsize=10)
    ax1.set_title('Phase 0E Sweep A v2: Achieved Camera FPS & Feature Survival Rate vs. Yaw Rate', fontsize=12, fontweight='bold')

    # Panel 2: Image-Space Feature Velocity & Mean Inliers
    ax2.plot(rates, vel_means, 'o-', color='tab:orange', linewidth=2.2, markersize=8, label='Mean Feature Velocity (px/frame)')
    ax2.axhline(10.5, color='darkred', linestyle=':', linewidth=1.8, label='KLT Search Window Bound (21x21 win -> 10.5px)')

    ax2.set_xlabel('Commanded Yaw Rate [deg/s]', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Feature Velocity [px/frame]', fontsize=11, fontweight='bold')
    ax2.set_xticks(YAW_RATES)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper left', fontsize=10)
    ax2.set_title('Image-Space Feature Velocity vs. Yaw Rate (v2 Corrected)', fontsize=12, fontweight='bold')

    plt.tight_layout()
    plot_path = os.path.join('plots', 'yaw_sweep_v2_characterization.png')
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close()

if __name__ == '__main__':
    main()
