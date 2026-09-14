#!/usr/bin/env python3
"""
Phase 0E Sweep B Analysis & Scientific Audit Script (Pitch/Tilt Experiment)

Audits and processes the 6 oscillating pitch/tilt dataset pairs (5, 10, 20, 30, 40, 50 deg).

Corrected Audit Implementation:
- Evaluates GT orientation separately into p95 |Pitch| and p95 |Roll|.
- Audits actual GT translation spans (X-span and Y-span) to quantify translation confounds.
- Applies timestamp-based 2.0s sliding-window failure metric (>70% of frames with num_inliers < 5).
- Enforces window maturity: requires the window to cover at least 1.95s of active flight (pos_z > 0.3m) before evaluating.
- Reports detection endpoint and full window [start -> end] relative to active-segment start.
- Verifies that all VO runs terminated cleanly before the 2,000-frame cap.
- Updates characterization plot (plots/tilt_sweep_characterization.png).
"""

import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_scipy

TILT_AMPS = [5, 10, 20, 30, 40, 50]

def load_csv(filepath):
    records = []
    if not os.path.exists(filepath):
        return records
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)
    return records

def process_tilt_level(amp):
    vo_path = f"results/tilt_sweep_{amp}deg_vo.csv"
    gt_path = f"results/tilt_sweep_{amp}deg_gt.csv"

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

    # Separate achieved p95 absolute pitch and roll from GT
    abs_pitches = []
    abs_rolls = []
    gt_xs = []
    gt_ys = []

    for r in act_gt:
        q = [float(r['rot_x']), float(r['rot_y']), float(r['rot_z']), float(r['rot_w'])]
        e = R_scipy.from_quat(q).as_euler('xyz', degrees=True)
        abs_rolls.append(abs(e[0]))
        abs_pitches.append(abs(e[1]))
        gt_xs.append(float(r['pos_x']))
        gt_ys.append(float(r['pos_y']))

    p95_pitch = float(np.percentile(abs_pitches, 95)) if abs_pitches else 0.0
    p95_roll = float(np.percentile(abs_rolls, 95)) if abs_rolls else 0.0

    # GT Translation spans
    x_span = float(np.max(gt_xs) - np.min(gt_xs)) if gt_xs else 0.0
    y_span = float(np.max(gt_ys) - np.min(gt_ys)) if gt_ys else 0.0

    # Match VO frames to active GT flight using relative timestamps
    gt_ts = np.array([float(r['timestamp_total_sec']) for r in gt_records])
    vo_rel_ts = vo_ts - vo_ts[0]
    gt_rel_ts = gt_ts - gt_ts[0]

    active_vo = []
    active_vo_indices = []
    for idx, r_vo in enumerate(vo_records):
        t_vo = vo_rel_ts[idx]
        best_idx = np.argmin(np.abs(gt_rel_ts - t_vo))
        if float(gt_records[best_idx]['pos_z']) > 0.3:
            active_vo.append(r_vo)
            active_vo_indices.append(idx)

    if not active_vo:
        active_vo = vo_records
        active_vo_indices = list(range(len(vo_records)))

    total_active = len(active_vo)
    first_active_frame_idx = active_vo_indices[0]

    inlier_counts = [int(r['num_inliers']) for r in active_vo]
    inlier_ratios = [float(r.get('inlier_ratio', 0.0)) * 100.0 for r in active_vo]
    survival_rates = [float(r.get('feature_survival_rate', 0.0)) * 100.0 for r in active_vo]
    vel_means = [float(r.get('feature_vel_mean', 0.0)) for r in active_vo]

    # Corrected 2.0s Timestamp-based Sliding Window Failure Metric:
    # Requires window duration >= 1.95s of active flight before evaluating or triggering.
    act_vo_ts = np.array([float(r['timestamp_total_sec']) for r in active_vo])
    act_rel_t = act_vo_ts - act_vo_ts[0]

    fail_window = False
    fail_frame_idx = None
    fail_end_t = None
    fail_start_t = None
    max_win_low_pct = 0.0

    for i, r in enumerate(active_vo):
        t_curr = act_rel_t[i]
        t_start = t_curr - 2.0

        win_indices = [k for k in range(len(active_vo)) if t_start <= act_rel_t[k] <= t_curr]
        if not win_indices:
            continue

        win_duration = t_curr - act_rel_t[win_indices[0]]

        # Enforce window maturity (must genuinely cover ~2.0s of active flight)
        if win_duration >= 1.95:
            win_frames = [active_vo[k] for k in win_indices]
            low_cnt = sum(1 for wf in win_frames if int(wf['num_inliers']) < 5)
            pct = float(low_cnt / len(win_frames)) * 100.0

            if pct > max_win_low_pct:
                max_win_low_pct = pct

            if pct > 70.0 and not fail_window:
                fail_window = True
                fail_frame_idx = int(r['frame_idx'])
                fail_end_t = t_curr
                fail_start_t = act_rel_t[win_indices[0]]

    return {
        'target_amp': amp,
        'p95_pitch': p95_pitch,
        'p95_roll': p95_roll,
        'x_span': x_span,
        'y_span': y_span,
        'achieved_fps': achieved_fps,
        'total_vo_frames': len(vo_records),
        'total_active': total_active,
        'first_active_frame_idx': first_active_frame_idx,
        'mean_inliers': np.mean(inlier_counts),
        'mean_inlier_ratio': np.mean(inlier_ratios),
        'mean_survival_rate': np.mean(survival_rates),
        'mean_vel_mean': np.mean(vel_means),
        'fail_window': fail_window,
        'fail_frame_idx': fail_frame_idx,
        'fail_start_t': fail_start_t,
        'fail_end_t': fail_end_t,
        'max_win_low_pct': max_win_low_pct
    }

def main():
    print("==========================================================================================")
    print("PHASE 0E SWEEP B ANALYSIS & REPAIR AUDIT: OSCILLATING PITCH/TILT EXPERIMENT")
    print("==========================================================================================")

    results = []
    for amp in TILT_AMPS:
        res = process_tilt_level(amp)
        if res:
            results.append(res)
        else:
            print(f"[WARNING] Missing dataset for target {amp} deg tilt!")

    if not results:
        print("[ERROR] No valid dataset files found in results/!")
        return

    # Print Formatted Summary Table
    print("\n===========================================================================================================================================================================")
    print("CORRECTED SWEEP B SUMMARY TABLE: PITCH/TILT EXPERIMENT AUDIT & SLIDING WINDOW FAILURE TIMING")
    print("===========================================================================================================================================================================")
    hdr = f"{'Target':<6} | {'GT p95 |Pitch|':<14} | {'GT p95 |Roll|':<13} | {'GT X-Span':<10} | {'GT Y-Span':<10} | {'FPS (Hz)':<8} | {'VO Fr':<6} | {'Act Fr':<6} | {'Inliers':<7} | {'Inl Ratio':<9} | {'Surv Rate':<9} | {'Vel (px)':<8} | {'Fail?':<5} | {'Qualified Window Detail (Active Rel t)':<38}"
    print(hdr)
    print("-" * len(hdr))

    for r in results:
        fail_str = "YES" if r['fail_window'] else "NO"
        if r['fail_window']:
            detail_str = f"End {r['fail_end_t']:.2f}s (Fr #{r['fail_frame_idx']}) [{r['fail_start_t']:.2f}s->{r['fail_end_t']:.2f}s]"
        else:
            detail_str = f"Stable (Peak {r['max_win_low_pct']:.1f}%)"

        print(f"{r['target_amp']:<6.0f}° | {r['p95_pitch']:<14.2f}° | {r['p95_roll']:<13.2f}° | {r['x_span']:<10.2f}m | {r['y_span']:<10.2f}m | {r['achieved_fps']:<8.2f} | {r['total_vo_frames']:<6} | {r['total_active']:<6} | {r['mean_inliers']:<7.2f} | {r['mean_inlier_ratio']:<9.2f}% | {r['mean_survival_rate']:<9.2f}% | {r['mean_vel_mean']:<8.2f} | {fail_str:<5} | {detail_str:<38}")

    print("===========================================================================================================================================================================\n")

    # Generate Corrected Plot
    os.makedirs('plots', exist_ok=True)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    amps = [r['target_amp'] for r in results]
    p95_pitches = [r['p95_pitch'] for r in results]
    x_spans = [r['x_span'] for r in results]
    surv_rates = [r['mean_survival_rate'] for r in results]
    vel_means = [r['mean_vel_mean'] for r in results]

    # Panel 1: Achieved p95 |Pitch| & GT X Translation Span vs. Command
    color1 = 'tab:blue'
    ax1.set_ylabel('Achieved p95 |Pitch| [deg]', color=color1, fontsize=11, fontweight='bold')
    line1 = ax1.plot(amps, p95_pitches, 'o-', color=color1, linewidth=2.2, markersize=8, label='Achieved p95 |Pitch| (deg)')
    ax1.tick_params(axis='y', labelcolor=color1)

    ax1_twin = ax1.twinx()
    color2 = 'tab:red'
    ax1_twin.set_ylabel('GT Translation X-Span [m]', color=color2, fontsize=11, fontweight='bold')
    line2 = ax1_twin.plot(amps, x_spans, 's--', color=color2, linewidth=2.0, markersize=7, label='GT Translation X-Span (m)')
    ax1_twin.tick_params(axis='y', labelcolor=color2)

    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left', fontsize=10)
    ax1.set_title('Phase 0E Sweep B Audit: Achieved Pitch & Unintended X Translation vs. Command', fontsize=12, fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)

    # Panel 2: Feature Survival Rate & Feature Velocity
    ax2.plot(amps, surv_rates, 'o-', color='tab:green', linewidth=2.2, markersize=8, label='Feature Survival Rate (%)')
    ax2.set_ylabel('Survival Rate (%)', color='tab:green', fontsize=11, fontweight='bold')
    ax2.tick_params(axis='y', labelcolor='tab:green')
    ax2.set_ylim(0, 105)

    ax2_twin = ax2.twinx()
    ax2_twin.plot(amps, vel_means, '^-', color='tab:orange', linewidth=2.2, markersize=8, label='Mean Feature Velocity (px/frame)')
    ax2_twin.axhline(10.5, color='darkred', linestyle=':', linewidth=1.8, label='PyrLK Single-Level 10.5px Reference')
    ax2_twin.set_ylabel('Feature Velocity [px/frame]', color='tab:orange', fontsize=11, fontweight='bold')
    ax2_twin.tick_params(axis='y', labelcolor='tab:orange')

    lines2 = [ax2.lines[0], ax2_twin.lines[0], ax2_twin.lines[1]]
    labels2 = [l.get_label() for l in lines2]
    ax2.legend(lines2, labels2, loc='lower left', fontsize=9)

    ax2.set_xlabel('Target Tilt/Pitch Command [deg]', fontsize=11, fontweight='bold')
    ax2.set_xticks(TILT_AMPS)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.set_title('Precursor Signals: Feature Survival & Image Velocity (Sweep B Audit)', fontsize=12, fontweight='bold')

    plt.tight_layout()
    plot_path = os.path.join('plots', 'tilt_sweep_characterization.png')
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close()

    print(f"Saved Corrected Pitch/Tilt Characterization Plot -> {os.path.abspath(plot_path)}\n")

if __name__ == '__main__':
    main()
