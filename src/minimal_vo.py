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
        inlier_ratio = 0.0
        rel_tx, rel_ty, rel_tz = 0.0, 0.0, 0.0
        rel_rot_deg = 0.0

        if self.mode == 'klt':
            num_detected, num_matched, num_inliers, inlier_ratio, rel_tx, rel_ty, rel_tz, rel_rot_deg = self._process_klt(cv_img)
        else:
            num_detected, num_matched, num_inliers, inlier_ratio, rel_tx, rel_ty, rel_tz, rel_rot_deg = self._process_orb(cv_img)

        # Convert rotation matrix to quaternion (x, y, z, w)
        r = R_scipy.from_matrix(self.curr_rot)
        quat_xyzw = r.as_quat()

        # Save frame record for CSV
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
            'inlier_ratio': f"{inlier_ratio:.4f}",
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
            self.get_logger().info(
                f"[{self.frame_idx}] Pos: ({self.curr_pos[0,0]:.2f}, {self.curr_pos[1,0]:.2f}, {self.curr_pos[2,0]:.2f}) | "
                f"Detected: {num_detected}, Matched: {num_matched}, Inliers: {num_inliers} ({inlier_ratio*100:.1f}%)"
            )

        self.frame_idx += 1
        if self.frame_idx >= self.max_frames:
            self.save_csv_and_report()
            rclpy.shutdown()

    def _process_klt(self, cv_img):
        num_detected = 0
        num_matched = 0
        num_inliers = 0
        inlier_ratio = 0.0
        rel_tx, rel_ty, rel_tz = 0.0, 0.0, 0.0
        rel_rot_deg = 0.0

        if self.prev_img is None or self.prev_pts is None or len(self.prev_pts) < 100:
            # Re-detect Good Features to Track
            pts = cv2.goodFeaturesToTrack(cv_img, maxCorners=2000, qualityLevel=0.001, minDistance=5)
            self.prev_pts = pts
            self.prev_img = cv_img
            num_detected = len(pts) if pts is not None else 0
            return num_detected, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0

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

            if num_matched >= 8:
                # Estimate Essential Matrix with 5-point RANSAC algorithm
                E, mask_E = cv2.findEssentialMat(
                    pts1, pts2, self.K,
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=1.0
                )

                if E is not None and E.shape == (3, 3):
                    inliers_count, R_opt, t_opt, mask_pose = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask_E)
                    num_inliers = int(inliers_count)
                    inlier_ratio = float(num_inliers / num_matched) if num_matched > 0 else 0.0

                    if num_inliers >= 8:
                        # Frame Transform: CV Optical -> Gazebo ENU World
                        R_gaz = self.R_opt2gaz @ R_opt @ self.R_opt2gaz.T
                        t_gaz = self.R_opt2gaz @ t_opt

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
        return num_detected, num_matched, num_inliers, inlier_ratio, rel_tx, rel_ty, rel_tz, rel_rot_deg

    def _process_orb(self, cv_img):
        kp, des = self.orb.detectAndCompute(cv_img, None)
        num_detected = len(kp) if kp is not None else 0

        num_matched = 0
        num_inliers = 0
        inlier_ratio = 0.0
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

            if num_matched >= 8:
                pts1 = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
                pts2 = np.float32([kp[m.trainIdx].pt for m in good_matches])

                E, mask_E = cv2.findEssentialMat(pts1, pts2, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                if E is not None and E.shape == (3, 3):
                    inliers_count, R_opt, t_opt, mask_pose = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask_E)
                    num_inliers = int(inliers_count)
                    inlier_ratio = float(num_inliers / num_matched) if num_matched > 0 else 0.0

                    if num_inliers >= 8:
                        R_gaz = self.R_opt2gaz @ R_opt @ self.R_opt2gaz.T
                        t_gaz = self.R_opt2gaz @ t_opt

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

        return num_detected, num_matched, num_inliers, inlier_ratio, rel_tx, rel_ty, rel_tz, rel_rot_deg

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
            'num_detected', 'num_matched', 'num_inliers', 'inlier_ratio',
            'rel_tx', 'rel_ty', 'rel_tz', 'rel_rot_deg'
        ]
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        det_list = [r['num_detected'] for r in self.records]
        match_list = [r['num_matched'] for r in self.records]
        inlier_list = [r['num_inliers'] for r in self.records]
        ratio_list = [float(r['inlier_ratio']) for r in self.records]

        print("\n" + "=" * 70)
        print("PHASE 0C — MINIMAL MONOCULAR VO EXECUTION REPORT")
        print("=" * 70)
        print(f"Tracking Pipeline      : {self.mode.upper()}")
        print(f"Total Processed Frames : {len(self.records)}")
        print(f"Output CSV Path        : {os.path.abspath(self.csv_path)}")
        print(f"Final Integrated Pos   : ({self.curr_pos[0,0]:.4f}, {self.curr_pos[1,0]:.4f}, {self.curr_pos[2,0]:.4f}) [Unit Scale]")
        print("-" * 70)
        print("INTERMEDIATE METRICS SUMMARY:")
        print(f"  Detected Features/Frame : Mean = {np.mean(det_list):.1f}, Min = {np.min(det_list)}, Max = {np.max(det_list)}")
        print(f"  Matched Features/Frame  : Mean = {np.mean(match_list):.1f}, Min = {np.min(match_list)}, Max = {np.max(match_list)}")
        print(f"  RANSAC Inliers/Frame    : Mean = {np.mean(inlier_list):.1f}, Min = {np.min(inlier_list)}, Max = {np.max(inlier_list)}")
        print(f"  Inlier Ratio            : Mean = {np.mean(ratio_list)*100:.2f}%, Min = {np.min(ratio_list)*100:.2f}%, Max = {np.max(ratio_list)*100:.2f}%")
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
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("VO interrupted by user.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
