#!/usr/bin/env python3
"""
Minimal Monocular Visual Odometry (VO) Node for ROS 2 (Phase 0C)

Pipeline Options:
  1. KLT Optical Flow Mode (Default): GFTT Corners -> Pyramidal KLT Optical Flow Tracking
  2. ORB Feature Mode: ORB Keypoints -> Brute-Force KNN Matcher with Lowe's Ratio Test

Core Steps (Common to both modes):
  Matched Points -> Essential Matrix (5-point RANSAC algorithm) -> Relative Pose Recovery (cv2.recoverPose)
  -> Frame Transformation (CV Optical -> World ENU) -> Trajectory Integration

Monocular Scale Ambiguity Handling:
  cv2.recoverPose returns a normalized translation unit vector ||t|| = 1.0.
  We explicitly do NOT invent or hardcode a fake metric scale. The raw trajectory is accumulated
  at unit scale (1.0 step magnitude per relative motion estimate), to be scale-aligned via
  Sim(3) / Umeyama alignment against ground-truth in Phase 0D.

Frame Transformations:
  - CV Optical Frame (C_opt): +Z Forward (Optical Axis), +X Right, +Y Down
  - Gazebo ENU World (W_enu): +X Forward/East, +Y Left/North, +Z Up
  - Rotation transformation matrix: R_opt2gaz = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
"""

import argparse
import csv
import math
import os
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R_scipy

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class MinimalVO(Node):
    def __init__(self, camera_topic, output_dir, max_frames=500, mode='klt', output_filename='vo_trajectory.csv'):
        super().__init__('minimal_vo')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.camera_topic = camera_topic
        self.output_dir = output_dir
        self.max_frames = max_frames
        self.mode = mode.lower()

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, output_filename)

        # Camera Intrinsics Matrix K (from Phase 0A verification)
        # width: 1280, height: 960, fx: 539.9363, fy: 539.9364, cx: 640.0, cy: 480.0
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

        # Feature Detectors
        self.orb = cv2.ORB_create(nfeatures=2000, fastThreshold=5)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # State Variables
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.prev_img = None
        self.prev_pts = None   # For KLT mode
        self.prev_kp = None    # For ORB mode
        self.prev_des = None   # For ORB mode

        # Cumulative World Pose (ENU Frame): Position 3x1, Orientation 3x3 Matrix
        self.curr_pos = np.zeros((3, 1), dtype=np.float64)
        self.curr_rot = np.eye(3, dtype=np.float64)

        self.frame_idx = 0
        self.records = []

        # State Variables
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        self.prev_img = None
        self.prev_pts = None   # For KLT mode
        self.prev_kp = None    # For ORB mode
        self.prev_des = None   # For ORB mode

        # Cumulative World Pose (ENU Frame): Position 3x1, Orientation 3x3 Matrix
        self.curr_pos = np.zeros((3, 1), dtype=np.float64)
        self.curr_rot = np.eye(3, dtype=np.float64)

        self.frame_idx = 0
        self.records = []
        self.frame_history = []  # History buffer of (timestamp_total_sec, num_inliers)

        # Failure Characterization Variables
        self.consecutive_low_inliers = 0
        self.failure_triggered_streak = False
        self.failure_first_frame_streak = None

        self.failure_triggered_window = False
        self.failure_first_frame_window = None

        # ROS 2 Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/vo/pose', 10)
        self.path_pub = self.create_publisher(Path, '/vo/path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

        self.get_logger().info(f"Minimal VO Node Initialized in '{self.mode.upper()}' mode on topic '{self.camera_topic}'")
        self.get_logger().info(f"Intrinsics K:\n{self.K}")
        self.get_logger().info(f"Output directory: '{os.path.abspath(self.output_dir)}'")

        self.sub_cam = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)

    def image_callback(self, msg: Image):
        if self.frame_idx >= self.max_frames:
            return

        # Convert Image msg to OpenCV Grayscale
        if HAS_CV_BRIDGE:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            except Exception:
                cv_img = self._convert_numpy(msg)
        else:
            cv_img = self._convert_numpy(msg)

        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        total_sec = sec + nanosec * 1e-9

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

        if self.mode == 'klt':
            num_detected, num_matched, num_inliers, num_inliers_E, num_inliers_pose, inlier_ratio, survival_rate, vel_mean, vel_max, mean_lk_err, num_inliers_H, rel_tx, rel_ty, rel_tz, rel_rot_deg = self._process_klt(cv_img)
        else:
            num_detected, num_matched, num_inliers, num_inliers_E, num_inliers_pose, inlier_ratio, mean_lk_err, num_inliers_H, rel_tx, rel_ty, rel_tz, rel_rot_deg = self._process_orb(cv_img)
            survival_rate = float(num_matched / num_detected) if num_detected > 0 else 0.0

        # Update failure condition 1: Consecutive streak (inlier count < 5 for 10+ consecutive frames)
        if num_inliers < 5:
            self.consecutive_low_inliers += 1
        else:
            self.consecutive_low_inliers = 0

        if self.consecutive_low_inliers >= 10:
            self.failure_triggered_streak = True
            if self.failure_first_frame_streak is None:
                self.failure_first_frame_streak = self.frame_idx
                self.get_logger().warning(
                    f"[STREAK FAILURE TRIGGERED] Frame #{self.frame_idx}: num_inliers < 5 for {self.consecutive_low_inliers} consecutive frames!"
                )

        # Update failure condition 2: Robust 2.0s Sliding Window (>70% of frames in prior 2.0s have num_inliers < 5)
        self.frame_history.append((total_sec, num_inliers))
        win_frames = [item for item in self.frame_history if (total_sec - item[0]) <= 2.0]
        low_inl_count = sum(1 for item in win_frames if item[1] < 5)
        window_low_inlier_pct = float(low_inl_count / len(win_frames)) * 100.0 if len(win_frames) > 0 else 0.0

        if window_low_inlier_pct > 70.0:
            self.failure_triggered_window = True
            if self.failure_first_frame_window is None:
                self.failure_first_frame_window = self.frame_idx
                self.get_logger().warning(
                    f"[WINDOW FAILURE TRIGGERED] Frame #{self.frame_idx}: {window_low_inlier_pct:.1f}% low-inlier frames in 2.0s window!"
                )

        # Convert rotation matrix to quaternion (x, y, z, w)
        r = R_scipy.from_matrix(self.curr_rot)
        quat_xyzw = r.as_quat()

        # Save frame record for CSV (explicitly separating num_inliers_E and num_inliers_pose)
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
            'num_inliers': num_inliers,              # Backward compatibility: num_inliers == num_inliers_E
            'num_inliers_E': num_inliers_E,          # Essential Matrix RANSAC inliers count
            'num_inliers_pose': num_inliers_pose,    # recoverPose cheirality/depth inliers count
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
            'rel_rot_deg': f"{rel_rot_deg:.2f}"
        }
        self.records.append(record)

        # Publish ROS 2 PoseStamped & Path
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(self.curr_pos[0, 0])
        pose_msg.pose.position.y = float(self.curr_pos[1, 0])
        pose_msg.pose.position.z = float(self.curr_pos[2, 0])
        pose_msg.pose.orientation.x = float(quat_xyzw[0])
        pose_msg.pose.orientation.y = float(quat_xyzw[1])
        pose_msg.pose.orientation.z = float(quat_xyzw[2])
        pose_msg.pose.orientation.w = float(quat_xyzw[3])

        self.pose_pub.publish(pose_msg)
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose_msg)
        self.path_pub.publish(self.path_msg)

        if self.frame_idx % 25 == 0 or self.frame_idx == 0:
            self.save_csv_and_report()
            self.get_logger().info(
                f"[{self.frame_idx}] Pos: ({self.curr_pos[0,0]:.2f}, {self.curr_pos[1,0]:.2f}, {self.curr_pos[2,0]:.2f}) | "
                f"Matched: {num_matched}, InliersE: {num_inliers_E}, InliersPose: {num_inliers_pose}, InliersH: {num_inliers_H} | LKErr: {mean_lk_err:.2f}px"
            )

        self.frame_idx += 1
        if self.frame_idx >= self.max_frames:
            self.save_csv_and_report()
            rclpy.shutdown()

    def _process_klt(self, cv_img):
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

        if self.prev_img is None or self.prev_pts is None or len(self.prev_pts) < 100:
            # Re-detect Good Features to Track
            pts = cv2.goodFeaturesToTrack(cv_img, maxCorners=2000, qualityLevel=0.001, minDistance=5)
            self.prev_pts = pts
            self.prev_img = cv_img
            num_detected = len(pts) if pts is not None else 0
            return num_detected, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0, 0.0

        num_detected = len(self.prev_pts)

        # Track features via Pyramidal LK Optical Flow
        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_img, cv_img, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        if curr_pts is not None and status is not None:
            valid_mask = (status == 1).reshape(-1)
            pts1 = self.prev_pts[valid_mask]
            pts2 = curr_pts[valid_mask]
            num_matched = len(pts2)

            survival_rate = float(num_matched / num_detected) if num_detected > 0 else 0.0

            if num_matched > 0:
                displacements = np.linalg.norm(pts2[:, 0, :] - pts1[:, 0, :], axis=1)
                vel_mean = float(np.mean(displacements))
                vel_max = float(np.max(displacements))

                if err is not None:
                    valid_err = err[valid_mask]
                    if len(valid_err) > 0:
                        mean_lk_err = float(np.mean(valid_err))

                if num_matched >= 4:
                    try:
                        H, mask_H = cv2.findHomography(pts1, pts2, cv2.RANSAC, 3.0)
                        if H is not None and mask_H is not None:
                            num_inliers_H = int(np.sum(mask_H == 1))
                    except Exception:
                        num_inliers_H = 0

            if num_matched >= 8:
                # Estimate Essential Matrix with 5-point RANSAC algorithm
                E, mask_E = cv2.findEssentialMat(
                    pts1, pts2, self.K,
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=1.0
                )

                if mask_E is not None:
                    num_inliers_E = int(np.sum(mask_E == 1))
                else:
                    num_inliers_E = 0

                num_inliers = num_inliers_E  # Backward compatibility: num_inliers == num_inliers_E
                inlier_ratio = float(num_inliers_E / num_matched) if num_matched > 0 else 0.0

                if E is not None and E.shape == (3, 3):
                    res_pose = cv2.recoverPose(
                        E,
                        pts1,
                        pts2,
                        self.K,
                        distanceThresh=1000.0,
                        mask=mask_E
                    )
                    inliers_count, R_opt, t_opt, mask_pose = res_pose[0], res_pose[1], res_pose[2], res_pose[3]
                    num_inliers_pose = int(inliers_count)

                    # Pose update condition strictly uses recoverPose inliers count as originally specified
                    if num_inliers_pose >= 8:
                        # Frame Transform: CV Optical -> Gazebo ENU World (negating t_opt to get camera motion vector)
                        R_gaz = self.R_opt2gaz @ R_opt @ self.R_opt2gaz.T
                        t_gaz = - (self.R_opt2gaz @ t_opt)

                        rel_tx = float(t_gaz[0, 0])
                        rel_ty = float(t_gaz[1, 0])
                        rel_tz = float(t_gaz[2, 0])

                        rot_angle_rad = math.acos(max(-1.0, min(1.0, (np.trace(R_gaz) - 1.0) / 2.0)))
                        rel_rot_deg = math.degrees(rot_angle_rad)

                        # Accumulate pose (Unit Scale)
                        self.curr_pos += self.curr_rot @ t_gaz
                        self.curr_rot = self.curr_rot @ R_gaz

                        # Update tracked keypoints for next frame
                        inlier_mask = (mask_pose > 0).reshape(-1)
                        self.prev_pts = pts2[inlier_mask].reshape(-1, 1, 2)
                    else:
                        self.prev_pts = pts2.reshape(-1, 1, 2)
                else:
                    self.prev_pts = pts2.reshape(-1, 1, 2)
            else:
                self.prev_pts = None

        self.prev_img = cv_img
        return num_detected, num_matched, num_inliers, num_inliers_E, num_inliers_pose, inlier_ratio, survival_rate, vel_mean, vel_max, mean_lk_err, num_inliers_H, rel_tx, rel_ty, rel_tz, rel_rot_deg

    def _process_orb(self, cv_img):
        kp, des = self.orb.detectAndCompute(cv_img, None)
        num_detected = len(kp) if kp is not None else 0

        num_matched = 0
        num_inliers_E = 0
        num_inliers_pose = 0
        num_inliers = 0
        inlier_ratio = 0.0
        mean_lk_err = 0.0
        num_inliers_H = 0
        rel_tx, rel_ty, rel_tz = 0.0, 0.0, 0.0
        rel_rot_deg = 0.0

        if self.prev_img is not None and self.prev_des is not None and des is not None and len(des) > 8 and len(self.prev_des) > 8:
            matches_knn = self.bf.knnMatch(self.prev_des, des, k=2)
            good_matches = []
            for match_pair in matches_knn:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.85 * n.distance:
                        good_matches.append(m)

            num_matched = len(good_matches)

            if num_matched >= 4:
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in good_matches])

                try:
                    H, mask_H = cv2.findHomography(pts1, pts2, cv2.RANSAC, 3.0)
                    if H is not None and mask_H is not None:
                        num_inliers_H = int(np.sum(mask_H == 1))
                except Exception:
                    num_inliers_H = 0

            if num_matched >= 8:
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in good_matches])

                E, mask_E = cv2.findEssentialMat(pts1, pts2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                if mask_E is not None:
                    num_inliers_E = int(np.sum(mask_E == 1))
                else:
                    num_inliers_E = 0

                num_inliers = num_inliers_E  # Backward compatibility: num_inliers == num_inliers_E
                inlier_ratio = float(num_inliers_E / num_matched) if num_matched > 0 else 0.0

                if E is not None and E.shape == (3, 3):
                    inliers_count, R_opt, t_opt, mask_pose = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask_E)
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

        self.prev_img = cv_img
        self.prev_kp = kp
        self.prev_des = des

        return num_detected, num_matched, num_inliers, num_inliers_E, num_inliers_pose, inlier_ratio, mean_lk_err, num_inliers_H, rel_tx, rel_ty, rel_tz, rel_rot_deg

    def _convert_numpy(self, msg: Image) -> np.ndarray:
        if msg.encoding in ['rgb8', 'bgr8']:
            img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
        elif msg.encoding in ['mono8']:
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        else:
            raise ValueError(f"Unsupported encoding for numpy fallback: {msg.encoding}")

    def save_csv_and_report(self):
        if not self.records:
            return

        fieldnames = [
            'frame_idx', 'timestamp_sec', 'timestamp_nanosec', 'timestamp_total_sec',
            'pos_x', 'pos_y', 'pos_z', 'rot_x', 'rot_y', 'rot_z', 'rot_w',
            'num_detected', 'num_matched', 'num_inliers', 'num_inliers_E', 'num_inliers_pose', 'num_inliers_H', 'inlier_ratio',
            'feature_survival_rate', 'feature_vel_mean', 'feature_vel_max', 'mean_lk_err',
            'consecutive_low_inliers', 'failure_triggered_streak',
            'window_low_inlier_pct', 'failure_triggered_window',
            'rel_tx', 'rel_ty', 'rel_tz', 'rel_rot_deg'
        ]
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        det_list = [r['num_detected'] for r in self.records]
        match_list = [r['num_matched'] for r in self.records]
        inlier_list = [r['num_inliers'] for r in self.records]
        inlier_e_list = [r.get('num_inliers_E', r['num_inliers']) for r in self.records]
        inlier_pose_list = [r.get('num_inliers_pose', 0) for r in self.records]
        inlier_h_list = [r['num_inliers_H'] for r in self.records]
        ratio_list = [float(r['inlier_ratio']) for r in self.records]
        surv_list = [float(r['feature_survival_rate']) for r in self.records]
        vel_list = [float(r['feature_vel_mean']) for r in self.records]
        lk_err_list = [float(r['mean_lk_err']) for r in self.records]

        ts_list = [float(r['timestamp_total_sec']) for r in self.records]
        duration_sec = ts_list[-1] - ts_list[0] if len(ts_list) > 1 else 0.0
        achieved_fps = float(len(self.records) / duration_sec) if duration_sec > 0 else 0.0

        print("\n" + "=" * 70)
        print("PHASE 0E — MONOCULAR VO EXECUTION REPORT")
        print("=" * 70)
        print(f"Tracking Pipeline      : {self.mode.upper()}")
        print(f"Total Processed Frames : {len(self.records)} (Achieved FPS: {achieved_fps:.2f} Hz over {duration_sec:.2f}s)")
        print(f"Output CSV Path        : {os.path.abspath(self.csv_path)}")
        print(f"Streak Failure (Old)   : {self.failure_triggered_streak} (First at Frame #{self.failure_first_frame_streak})")
        print(f"Window Failure (New)   : {self.failure_triggered_window} (First at Frame #{self.failure_first_frame_window})")
        print("-" * 70)
        print("INTERMEDIATE METRICS SUMMARY:")
        print(f"  Detected Features/Frame : Mean = {np.mean(det_list):.1f}, Min = {np.min(det_list)}, Max = {np.max(det_list)}")
        print(f"  Matched Features/Frame  : Mean = {np.mean(match_list):.1f}, Min = {np.min(match_list)}, Max = {np.max(match_list)}")
        print(f"  Essential Inliers (E)   : Mean = {np.mean(inlier_e_list):.1f}, Min = {np.min(inlier_e_list)}, Max = {np.max(inlier_e_list)}")
        print(f"  Pose Inliers (recover)  : Mean = {np.mean(inlier_pose_list):.1f}, Min = {np.min(inlier_pose_list)}, Max = {np.max(inlier_pose_list)}")
        print(f"  Homography Inliers/Frame: Mean = {np.mean(inlier_h_list):.1f}, Min = {np.min(inlier_h_list)}, Max = {np.max(inlier_h_list)}")
        print(f"  Inlier Ratio            : Mean = {np.mean(ratio_list)*100:.2f}%, Min = {np.min(ratio_list)*100:.2f}%, Max = {np.max(ratio_list)*100:.2f}%")
        print(f"  Feature Survival Rate   : Mean = {np.mean(surv_list)*100:.2f}%, Min = {np.min(surv_list)*100:.2f}%, Max = {np.max(surv_list)*100:.2f}%")
        print(f"  Feature Velocity (Mean) : Mean = {np.mean(vel_list):.2f} px/fr, Max = {np.max(vel_list):.2f} px/fr")
        print(f"  LK Tracking Error (Mean): Mean = {np.mean(lk_err_list):.4f} px, Max = {np.max(lk_err_list):.4f} px")
        print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Minimal Monocular VO (Phase 0C)")
    parser.add_argument(
        '--camera-topic',
        default='/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
        help="ROS 2 camera image topic"
    )
    parser.add_argument(
        '--output-dir',
        default='results',
        help="Output directory for CSV trajectory"
    )
    parser.add_argument(
        '--output-filename',
        default='vo_trajectory.csv',
        help="Output CSV file name"
    )
    parser.add_argument(
        '--max-frames',
        type=int,
        default=300,
        help="Maximum frames to process"
    )
    parser.add_argument(
        '--mode',
        default='klt',
        choices=['klt', 'orb'],
        help="Feature tracking pipeline mode (klt or orb)"
    )

    args = parser.parse_args()

    rclpy.init()
    node = MinimalVO(args.camera_topic, args.output_dir, args.max_frames, mode=args.mode, output_filename=args.output_filename)

    def sig_handler(sig, frame):
        print("\n[INFO] Signal received in MinimalVO. Saving CSV before exiting...")
        node.save_csv_and_report()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    import signal
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("VO interrupted by user.")
    finally:
        node.save_csv_and_report()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
