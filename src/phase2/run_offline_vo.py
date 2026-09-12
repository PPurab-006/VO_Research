#!/usr/bin/env python3
"""
Offline Monocular Visual Odometry Runner for Phase 2A (RAW vs EIS Comparison)

Processes saved camera datasets offline using the EXACT SAME monocular VO pipeline
as `minimal_vo.py` (KLT tracking, 5-point Essential matrix RANSAC, recoverPose with distanceThresh=1000.0).

Supports:
- RAW mode: processes un-derotated raw image frames.
- EIS mode: derotates images using `EISDerotator` (SLERP attitude interpolation + homography warp)
  prior to feeding into the identical VO pipeline.
  Supports 'fixed', 'incremental', 'null', 'gated', and 'scaled' modes.
"""

import argparse
import csv
import math
import os
import sys
import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy

from eis_derotation import EISDerotator


class OfflineVOProcessor:
    def __init__(self, output_csv_path, mode='klt', eis_derotator=None, gate_thresh_deg=15.0, gate_max_deg=45.0,
                 delayed_triangulation=False, r_frame_def='yaw_rate', baseline_depth_thresh=0.005, min_non_r_obs=3):
        self.output_csv_path = output_csv_path
        self.mode = mode.lower()
        self.eis_derotator = eis_derotator
        self.gate_thresh_deg = gate_thresh_deg
        self.gate_max_deg = gate_max_deg
        self.delayed_triangulation = delayed_triangulation
        self.r_frame_def = r_frame_def.lower()
        self.baseline_depth_thresh = baseline_depth_thresh
        self.min_non_r_obs = min_non_r_obs

        # Telemetry & GT positioning for R-frame detection
        self.telemetry_derotator = None
        self.gt_times = None
        self.gt_pos_x = None
        self.gt_pos_y = None
        self.gt_pos_z = None
        self.prev_gt_pos = None

        # Camera Intrinsics Matrix K (identical to minimal_vo.py)
        self.fx = 539.9363327026367
        self.fy = 539.9363708496094
        self.cx = 640.0
        self.cy = 480.0
        self.K = np.array([
            [self.fx, 0.0,     self.cx],
            [0.0,     self.fy, self.cy],
            [0.0,     0.0,     1.0]
        ], dtype=np.float64)

        # Transformation matrix: CV Optical Frame -> Gazebo ENU World Frame
        self.R_opt2gaz = np.array([
            [ 0.0,  0.0,  1.0],
            [-1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0]
        ], dtype=np.float64)

        self.orb = cv2.ORB_create(nfeatures=2000, fastThreshold=5)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.prev_img = None
        self.prev_pts = None
        self.prev_statuses = None      # 1 for ACTIVE, 0 for PENDING
        self.prev_non_r_counts = None  # count of non-R frames observed
        self.prev_init_frames = None   # frame index when feature was detected
        self.promotion_latencies = []

        self.curr_pos = np.zeros((3, 1), dtype=np.float64)
        self.curr_rot = np.eye(3, dtype=np.float64)

        self.frame_idx = 0
        self.records = []
        self.frame_history = []

        self.consecutive_low_inliers = 0
        self.failure_triggered_streak = False
        self.failure_first_frame_streak = None

        self.failure_triggered_window = False
        self.failure_first_frame_window = None

    def load_telemetry(self, gt_csv_path):
        if self.eis_derotator is None:
            self.telemetry_derotator = EISDerotator(reference_mode='null')
            self.telemetry_derotator.load_attitude_telemetry(gt_csv_path)

        df_gt = pd.read_csv(gt_csv_path)
        t_col = 'timestamp_total_sec' if 'timestamp_total_sec' in df_gt.columns else 'timestamp'
        gt_t = df_gt[t_col].values.astype(np.float64)
        if gt_t[0] > 1e12:
            gt_t = gt_t / 1e9
        gt_t_clean, unique_idx = np.unique(gt_t, return_index=True)
        self.gt_times = gt_t_clean
        self.gt_pos_x = df_gt['pos_x'].values[unique_idx]
        self.gt_pos_y = df_gt['pos_y'].values[unique_idx]
        self.gt_pos_z = df_gt['pos_z'].values[unique_idx]

    def process_frame(self, cv_img, sec, nanosec, total_sec):
        # Apply EIS derotation if enabled
        cv_img_raw = cv_img
        dt_ms = 0.0
        crop_pct = 0.0
        warp_deg = 0.0
        cum_warp_deg = 0.0
        gate_scale = 1.0
        yaw_rate_deg = 0.0
        H_cv = None

        if self.eis_derotator is not None:
            cv_img, H_cv, dt_ms, gate_scale, yaw_rate_deg = self.eis_derotator.derotate_image(
                cv_img_raw, total_sec,
                gate_thresh_deg=self.gate_thresh_deg, gate_max_deg=self.gate_max_deg
            )
            crop_pct = self.eis_derotator.compute_crop_percentage(H_cv)
            warp_deg = self.eis_derotator.compute_warp_angle(H_cv)

            # Cumulative warp relative to t_0
            R_body_k = self.eis_derotator.get_attitude_at_timestamp(total_sec)
            H_cum, _, _ = self.eis_derotator.compute_homography(R_body_k, reference_mode='fixed')
            cum_warp_deg = self.eis_derotator.compute_warp_angle(H_cum)
        elif self.telemetry_derotator is not None:
            R_body_k = self.telemetry_derotator.get_attitude_at_timestamp(total_sec)
            _, _, yaw_rate_deg = self.telemetry_derotator.compute_homography(
                R_body_k, reference_mode='gated', t_sec=total_sec,
                prev_t_sec=self.telemetry_derotator.prev_t_sec,
                gate_thresh_deg=self.gate_thresh_deg
            )
            self.telemetry_derotator.prev_R_body = R_body_k
            self.telemetry_derotator.prev_t_sec = total_sec

        # Baseline / Depth ratio computation (for Definition b)
        bd_ratio = 0.0
        if self.gt_times is not None:
            px = float(np.interp(total_sec, self.gt_times, self.gt_pos_x))
            py = float(np.interp(total_sec, self.gt_times, self.gt_pos_y))
            pz = float(np.interp(total_sec, self.gt_times, self.gt_pos_z))
            curr_gt_pos = np.array([px, py, pz], dtype=float)
            if self.prev_gt_pos is not None:
                baseline = float(np.linalg.norm(curr_gt_pos - self.prev_gt_pos))
                depth = max(1.0, pz)
                bd_ratio = float(baseline / depth)
            self.prev_gt_pos = curr_gt_pos

        # R-frame detection
        is_r_frame = False
        if self.r_frame_def == 'yaw_rate':
            is_r_frame = (yaw_rate_deg > self.gate_thresh_deg)
        elif self.r_frame_def == 'baseline_depth':
            is_r_frame = (bd_ratio < self.baseline_depth_thresh)

        num_detected = 0
        num_matched = 0
        num_inliers = 0
        num_inliers_E = 0
        num_inliers_pose = 0
        inlier_ratio = 0.0
        survival_rate = 0.0
        vel_mean = 0.0
        vel_max = 0.0
        mean_lk_err = 0.0
        num_inliers_H = 0
        rel_tx, rel_ty, rel_tz = 0.0, 0.0, 0.0
        rel_rot_deg = 0.0
        is_r_frame_val = 0
        num_active = 0
        num_pending = 0
        promotions_this_frame = 0

        if self.mode == 'klt':
            (num_detected, num_matched, num_inliers, num_inliers_E, num_inliers_pose,
             inlier_ratio, survival_rate, vel_mean, vel_max, mean_lk_err, num_inliers_H,
             rel_tx, rel_ty, rel_tz, rel_rot_deg, is_r_frame_val, num_active, num_pending,
             promotions_this_frame) = self._process_klt(cv_img, cv_img_raw=cv_img_raw, H_cv=H_cv, is_r_frame=is_r_frame)
        else:
            raise NotImplementedError("Only KLT mode is used for Phase 2A monocular VO")

        if num_inliers < 5:
            self.consecutive_low_inliers += 1
        else:
            self.consecutive_low_inliers = 0

        if self.consecutive_low_inliers >= 10:
            self.failure_triggered_streak = True
            if self.failure_first_frame_streak is None:
                self.failure_first_frame_streak = self.frame_idx

        self.frame_history.append((total_sec, num_inliers))
        win_frames = [item for item in self.frame_history if (total_sec - item[0]) <= 2.0]
        low_inl_count = sum(1 for item in win_frames if item[1] < 5)
        window_low_inlier_pct = float(low_inl_count / len(win_frames)) * 100.0 if len(win_frames) > 0 else 0.0

        if window_low_inlier_pct > 70.0:
            self.failure_triggered_window = True
            if self.failure_first_frame_window is None:
                self.failure_first_frame_window = self.frame_idx

        r = R_scipy.from_matrix(self.curr_rot)
        quat_xyzw = r.as_quat()

        record = {
            'frame_idx': self.frame_idx,
            'timestamp_sec': sec,
            'timestamp_nanosec': nanosec,
            'timestamp_total_sec': f"{total_sec:.9f}",
            'pos_x': self.curr_pos[0, 0],
            'pos_y': self.curr_pos[1, 0],
            'pos_z': self.curr_pos[2, 0],
            'rot_x': quat_xyzw[0],
            'rot_y': quat_xyzw[1],
            'rot_z': quat_xyzw[2],
            'rot_w': quat_xyzw[3],
            'num_detected': num_detected,
            'num_matched': num_matched,
            'num_inliers': num_inliers,
            'num_inliers_E': num_inliers_E,
            'num_inliers_pose': num_inliers_pose,
            'num_inliers_H': num_inliers_H,
            'inlier_ratio': f"{inlier_ratio:.4f}",
            'feature_survival_rate': f"{survival_rate:.4f}",
            'feature_vel_mean': f"{vel_mean:.4f}",
            'feature_vel_max': f"{vel_max:.4f}",
            'mean_lk_err': f"{mean_lk_err:.4f}",
            'consecutive_low_inliers': self.consecutive_low_inliers,
            'failure_triggered_streak': self.failure_triggered_streak,
            'window_low_inlier_pct': f"{window_low_inlier_pct:.2f}",
            'failure_triggered_window': self.failure_triggered_window,
            'rel_tx': rel_tx,
            'rel_ty': rel_ty,
            'rel_tz': rel_tz,
            'rel_rot_deg': f"{rel_rot_deg:.2f}",
            'eis_dt_ms': f"{dt_ms:.4f}",
            'eis_crop_pct': f"{crop_pct:.2f}",
            'eis_warp_deg': f"{warp_deg:.4f}",
            'eis_cum_warp_deg': f"{cum_warp_deg:.4f}",
            'eis_gate_scale': f"{gate_scale:.4f}",
            'eis_yaw_rate_deg': f"{yaw_rate_deg:.4f}",
            'is_r_frame': is_r_frame_val,
            'num_active': num_active,
            'num_pending': num_pending,
            'promotions_this_frame': promotions_this_frame,
            'bd_ratio': f"{bd_ratio:.6f}"
        }
        self.records.append(record)
        self.frame_idx += 1

    def _process_klt(self, cv_img, cv_img_raw=None, H_cv=None, is_r_frame=False):
        num_detected = 0
        num_matched = 0
        num_inliers_E = 0
        num_inliers_pose = 0
        num_inliers = 0
        inlier_ratio = 0.0
        survival_rate = 0.0
        vel_mean = 0.0
        vel_max = 0.0
        mean_lk_err = 0.0
        num_inliers_H = 0
        rel_tx, rel_ty, rel_tz = 0.0, 0.0, 0.0
        rel_rot_deg = 0.0
        promotions_this_frame = 0

        is_inc_mode = self.eis_derotator is not None and self.eis_derotator.reference_mode in ['incremental', 'null', 'gated', 'scaled']
        target_detect_img = cv_img_raw if (cv_img_raw is not None and is_inc_mode) else cv_img

        if self.prev_img is None or self.prev_pts is None or len(self.prev_pts) < 100:
            pts = cv2.goodFeaturesToTrack(target_detect_img, maxCorners=2000, qualityLevel=0.001, minDistance=5)
            if pts is not None:
                self.prev_pts = pts
                n = len(pts)
                if self.delayed_triangulation and is_r_frame:
                    self.prev_statuses = np.zeros(n, dtype=int)
                else:
                    self.prev_statuses = np.ones(n, dtype=int)
                self.prev_non_r_counts = np.zeros(n, dtype=int)
                self.prev_init_frames = np.full(n, self.frame_idx, dtype=int)
            else:
                self.prev_pts = None
                self.prev_statuses = None
                self.prev_non_r_counts = None
                self.prev_init_frames = None

            self.prev_img = target_detect_img
            num_detected = len(pts) if pts is not None else 0
            n_pend = num_detected if (self.delayed_triangulation and is_r_frame) else 0
            n_act = num_detected - n_pend
            return (num_detected, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0,
                    int(is_r_frame), n_act, n_pend, 0)

        num_detected = len(self.prev_pts)

        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_img, cv_img, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        if status is not None and len(status) > 0:
            status_flat = status.ravel() == 1
            pts1 = self.prev_pts[status_flat].reshape(-1, 2)
            pts2 = curr_pts[status_flat].reshape(-1, 2)
            statuses = self.prev_statuses[status_flat]
            non_r_counts = self.prev_non_r_counts[status_flat]
            init_frames = self.prev_init_frames[status_flat]

            num_matched = len(pts2)
            survival_rate = float(num_matched / num_detected) if num_detected > 0 else 0.0

            if len(err) > 0 and np.sum(status_flat) > 0:
                mean_lk_err = float(np.mean(err[status_flat]))

            # Promotion logic: if delayed triangulation enabled and frame is NON-R
            if self.delayed_triangulation:
                if not is_r_frame:
                    for i in range(len(pts2)):
                        if statuses[i] == 0:
                            non_r_counts[i] += 1
                            if non_r_counts[i] >= self.min_non_r_obs:
                                statuses[i] = 1
                                promotions_this_frame += 1
                                latency = self.frame_idx - init_frames[i]
                                self.promotion_latencies.append(latency)

            active_mask = (statuses == 1) if self.delayed_triangulation else np.ones(len(pts2), dtype=bool)
            pending_mask = ~active_mask

            num_active = int(np.sum(active_mask))
            num_pending = int(np.sum(pending_mask))

            next_pts = None
            next_statuses = None
            next_non_r_counts = None
            next_init_frames = None

            if num_active >= 5:
                pts1_act = pts1[active_mask]
                pts2_act = pts2[active_mask]

                vels = np.linalg.norm(pts2_act - pts1_act, axis=1)
                vel_mean = float(np.mean(vels)) if len(vels) > 0 else 0.0
                vel_max = float(np.max(vels)) if len(vels) > 0 else 0.0

                E, mask_E = cv2.findEssentialMat(
                    pts1_act, pts2_act, self.K,
                    method=cv2.RANSAC, prob=0.999, threshold=1.0
                )
                if E is not None and E.shape == (3, 3):
                    num_inliers_E = int(np.sum(mask_E))
                    res_pose = cv2.recoverPose(
                        E,
                        pts1_act,
                        pts2_act,
                        self.K,
                        distanceThresh=1000.0,
                        mask=mask_E
                    )
                    inliers_count, R_opt, t_opt, mask_pose = res_pose[0], res_pose[1], res_pose[2], res_pose[3]
                    num_inliers_pose = int(inliers_count)

                    if num_inliers_pose >= 8:
                        R_gaz = self.R_opt2gaz @ R_opt @ self.R_opt2gaz.T
                        t_gaz = - (self.R_opt2gaz @ t_opt)

                        rel_tx = float(t_gaz[0, 0])
                        rel_ty = float(t_gaz[1, 0])
                        rel_tz = float(t_gaz[2, 0])

                        rot_angle_rad = math.acos(max(-1.0, min(1.0, (np.trace(R_gaz) - 1.0) / 2.0)))
                        rel_rot_deg = math.degrees(rot_angle_rad)

                        self.curr_pos += self.curr_rot @ t_gaz
                        self.curr_rot = self.curr_rot @ R_gaz

                        inlier_mask_act = (mask_pose > 0).reshape(-1)

                        # Retain active inliers + all pending features
                        keep_mask = np.zeros(len(pts2), dtype=bool)
                        active_indices = np.where(active_mask)[0]
                        inlier_active_indices = active_indices[inlier_mask_act]
                        keep_mask[inlier_active_indices] = True
                        keep_mask[pending_mask] = True

                        next_pts = pts2[keep_mask].reshape(-1, 1, 2)
                        next_statuses = statuses[keep_mask]
                        next_non_r_counts = non_r_counts[keep_mask]
                        next_init_frames = init_frames[keep_mask]
                    else:
                        next_pts = pts2.reshape(-1, 1, 2)
                        next_statuses = statuses
                        next_non_r_counts = non_r_counts
                        next_init_frames = init_frames
                else:
                    next_pts = pts2.reshape(-1, 1, 2)
                    next_statuses = statuses
                    next_non_r_counts = non_r_counts
                    next_init_frames = init_frames
            else:
                next_pts = pts2.reshape(-1, 1, 2)
                next_statuses = statuses
                next_non_r_counts = non_r_counts
                next_init_frames = init_frames

            if is_inc_mode:
                if next_pts is not None and len(next_pts) > 0:
                    pts_raw = self.eis_derotator.unwarp_points(next_pts, H_cv)
                    u = pts_raw[:, 0, 0]
                    v = pts_raw[:, 0, 1]
                    valid_b = (u >= 0) & (u < self.cx * 2) & (v >= 0) & (v < self.cy * 2)
                    if np.sum(valid_b) >= 10:
                        self.prev_pts = pts_raw[valid_b].reshape(-1, 1, 2).astype(np.float32)
                        self.prev_statuses = next_statuses[valid_b]
                        self.prev_non_r_counts = next_non_r_counts[valid_b]
                        self.prev_init_frames = next_init_frames[valid_b]
                    else:
                        pts = cv2.goodFeaturesToTrack(cv_img_raw, maxCorners=2000, qualityLevel=0.001, minDistance=5)
                        if pts is not None:
                            self.prev_pts = pts.astype(np.float32)
                            n = len(pts)
                            if self.delayed_triangulation and is_r_frame:
                                self.prev_statuses = np.zeros(n, dtype=int)
                            else:
                                self.prev_statuses = np.ones(n, dtype=int)
                            self.prev_non_r_counts = np.zeros(n, dtype=int)
                            self.prev_init_frames = np.full(n, self.frame_idx, dtype=int)
                        else:
                            self.prev_pts = None
                            self.prev_statuses = None
                            self.prev_non_r_counts = None
                            self.prev_init_frames = None
                else:
                    self.prev_pts = None
                    self.prev_statuses = None
                    self.prev_non_r_counts = None
                    self.prev_init_frames = None
                self.prev_img = cv_img_raw
            else:
                if next_pts is not None and len(next_pts) > 0:
                    self.prev_pts = next_pts.astype(np.float32)
                    self.prev_statuses = next_statuses
                    self.prev_non_r_counts = next_non_r_counts
                    self.prev_init_frames = next_init_frames
                else:
                    self.prev_pts = None
                    self.prev_statuses = None
                    self.prev_non_r_counts = None
                    self.prev_init_frames = None
                self.prev_img = cv_img

        num_inliers = num_inliers_pose
        inlier_ratio = float(num_inliers / num_matched) if num_matched > 0 else 0.0

        return (num_detected, num_matched, num_inliers, num_inliers_E, num_inliers_pose, inlier_ratio, survival_rate, vel_mean, vel_max, mean_lk_err, num_inliers_H, rel_tx, rel_ty, rel_tz, rel_rot_deg,
                int(is_r_frame), num_active, num_pending, promotions_this_frame)

    def save_csv(self):
        os.makedirs(os.path.dirname(self.output_csv_path), exist_ok=True)
        fieldnames = list(self.records[0].keys()) if self.records else []
        with open(self.output_csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)
        print(f"[VO COMPLETE] Saved VO trajectory log: '{os.path.abspath(self.output_csv_path)}' ({len(self.records)} frames)")


def main():
    parser = argparse.ArgumentParser(description="Offline Monocular VO Processor (RAW vs EIS vs Delayed Triangulation)")
    parser.add_argument('--dataset-dir', required=True, help="Directory containing dataset images/ and camera_frames.csv")
    parser.add_argument('--gt-csv', required=True, help="GT attitude/position CSV")
    parser.add_argument('--output-csv', required=True, help="Output CSV path for VO trajectory")
    parser.add_argument('--eis', action='store_true', help="Enable EIS derotation before VO")
    parser.add_argument('--eis-mode', choices=['fixed', 'incremental', 'null', 'gated', 'scaled'], default='fixed', help="EIS reference mode")
    parser.add_argument('--gate-thresh', type=float, default=15.0, help="Gating yaw rate threshold in deg/s")
    parser.add_argument('--gate-max', type=float, default=45.0, help="Scaling max yaw rate threshold in deg/s")
    parser.add_argument('--delayed-triangulation', action='store_true', help="Enable RD-VIO delayed landmark triangulation")
    parser.add_argument('--r-frame-def', choices=['yaw_rate', 'baseline_depth'], default='yaw_rate', help="Definition for R-frame detection")
    parser.add_argument('--baseline-depth-thresh', type=float, default=0.005, help="Threshold for baseline/depth ratio in def (b)")
    parser.add_argument('--min-non-r-obs', type=int, default=3, help="Minimum non-R frame observations before promotion")
    args = parser.parse_args()

    cam_csv_path = os.path.join(args.dataset_dir, 'camera_frames.csv')
    df_cam = pd.read_csv(cam_csv_path)

    eis_derotator = None
    if args.eis:
        eis_derotator = EISDerotator(reference_mode=args.eis_mode)
        eis_derotator.load_attitude_telemetry(args.gt_csv)
        print(f"[EIS ENABLED] Loaded GT attitude telemetry from '{args.gt_csv}' (Mode: {args.eis_mode.upper()}, Thresh: {args.gate_thresh} deg/s)")

    processor = OfflineVOProcessor(
        output_csv_path=args.output_csv, mode='klt',
        eis_derotator=eis_derotator,
        gate_thresh_deg=args.gate_thresh, gate_max_deg=args.gate_max,
        delayed_triangulation=args.delayed_triangulation,
        r_frame_def=args.r_frame_def,
        baseline_depth_thresh=args.baseline_depth_thresh,
        min_non_r_obs=args.min_non_r_obs
    )
    processor.load_telemetry(args.gt_csv)

    for idx, row in df_cam.iterrows():
        img_filename = row['filename']
        img_path = os.path.join(args.dataset_dir, 'images', img_filename)
        cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

        sec = int(row['timestamp_sec'])
        nanosec = int(row['timestamp_nanosec'])
        total_sec = float(row['timestamp_total_sec'])

        processor.process_frame(cv_img, sec, nanosec, total_sec)

    processor.save_csv()


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
