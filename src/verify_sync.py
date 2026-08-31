#!/usr/bin/env python3
"""
Synchronization Verification Script for ROS 2 + Gazebo SITL (Phase 0B)

Subscribes to camera image topic (~30 Hz) and ground-truth pose topic (~50 Hz) simultaneously,
records timestamped streams during a motion run, performs nearest-neighbor timestamp matching,
computes gap statistics (min, max, mean, median, std_dev), evaluates tolerance, and exports logs to CSV.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage


class SyncVerifier(Node):
    def __init__(self, cam_topic, pose_topic, duration, output_dir, model_name, tolerance_ms):
        super().__init__('sync_verifier')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.cam_topic = cam_topic
        self.pose_topic = pose_topic
        self.duration = duration
        self.output_dir = output_dir
        self.model_name = model_name
        self.tolerance_ms = tolerance_ms

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_summary_path = os.path.join(self.output_dir, 'sync_summary.csv')
        self.csv_matches_path = os.path.join(self.output_dir, 'sync_matches.csv')

        self.cam_records = []   # list of (seq, stamp_sec, total_sec)
        self.pose_records = []  # list of (seq, stamp_sec, total_sec, pos_x, pos_y, pos_z, rot_w)

        self.start_wall_time = None
        self.recording_complete = False

        self.get_logger().info(f"Subscribing to Camera Topic : '{self.cam_topic}'")
        self.get_logger().info(f"Subscribing to Pose Topic   : '{self.pose_topic}'")
        self.get_logger().info(f"Logging duration: {self.duration} s | Sync Tolerance: {self.tolerance_ms} ms")

        self.sub_cam = self.create_subscription(Image, self.cam_topic, self.cam_callback, 50)
        self.sub_pose = self.create_subscription(TFMessage, self.pose_topic, self.pose_callback, 100)

    def cam_callback(self, msg: Image):
        if self.recording_complete:
            return

        if self.start_wall_time is None:
            self.start_wall_time = time.time()

        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        total_sec = sec + nanosec * 1e-9

        self.cam_records.append({
            'seq': len(self.cam_records),
            'sec': sec,
            'nanosec': nanosec,
            'total_sec': total_sec,
            'width': msg.width,
            'height': msg.height
        })

        if time.time() - self.start_wall_time >= self.duration and not self.recording_complete:
            self.recording_complete = True
            self.process_sync_and_report()
            rclpy.shutdown()

    def pose_callback(self, msg: TFMessage):
        if self.recording_complete or not msg.transforms:
            return

        target_tf = None
        for tf in msg.transforms:
            if tf.child_frame_id == self.model_name or self.model_name in tf.child_frame_id:
                target_tf = tf
                break
        if target_tf is None:
            target_tf = msg.transforms[0]

        # Use node sim clock if tf header stamp is 0
        stamp = target_tf.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        sec = stamp.sec
        nanosec = stamp.nanosec
        total_sec = sec + nanosec * 1e-9

        pos = target_tf.transform.translation
        rot = target_tf.transform.rotation

        self.pose_records.append({
            'seq': len(self.pose_records),
            'sec': sec,
            'nanosec': nanosec,
            'total_sec': total_sec,
            'pos_x': pos.x,
            'pos_y': pos.y,
            'pos_z': pos.z,
            'rot_x': rot.x,
            'rot_y': rot.y,
            'rot_z': rot.z,
            'rot_w': rot.w
        })

    def process_sync_and_report(self):
        n_cam = len(self.cam_records)
        n_pose = len(self.pose_records)

        self.get_logger().info(f"Recording complete. Camera frames: {n_cam}, Ground-truth poses: {n_pose}")

        if n_cam == 0 or n_pose == 0:
            self.get_logger().error("Error: Insufficient data recorded for synchronization check.")
            return

        pose_ts = np.array([p['total_sec'] for p in self.pose_records])

        matches = []
        gaps_ms = []

        for cam in self.cam_records:
            t_cam = cam['total_sec']
            # Find index of nearest ground-truth pose by timestamp difference
            diffs = np.abs(pose_ts - t_cam)
            best_idx = np.argmin(diffs)
            min_gap_sec = diffs[best_idx]
            min_gap_ms = min_gap_sec * 1000.0

            best_pose = self.pose_records[best_idx]
            signed_dt_ms = (t_cam - best_pose['total_sec']) * 1000.0
            is_within_tol = min_gap_ms <= self.tolerance_ms

            matches.append({
                'cam_frame_idx': cam['seq'],
                'cam_timestamp_sec': f"{t_cam:.9f}",
                'pose_idx': best_pose['seq'],
                'pose_timestamp_sec': f"{best_pose['total_sec']:.9f}",
                'gap_abs_ms': f"{min_gap_ms:.4f}",
                'gap_signed_ms': f"{signed_dt_ms:.4f}",
                'within_tolerance': is_within_tol,
                'pos_x': best_pose['pos_x'],
                'pos_y': best_pose['pos_y'],
                'pos_z': best_pose['pos_z']
            })
            gaps_ms.append(min_gap_ms)

        gaps_ms = np.array(gaps_ms)
        matched_count = len(matches)
        within_tol_count = np.sum(gaps_ms <= self.tolerance_ms)
        tol_pass_rate = (within_tol_count / matched_count) * 100.0

        min_gap = np.min(gaps_ms)
        max_gap = np.max(gaps_ms)
        mean_gap = np.mean(gaps_ms)
        median_gap = np.median(gaps_ms)
        std_gap = np.std(gaps_ms)

        # Save matches CSV
        fieldnames = [
            'cam_frame_idx', 'cam_timestamp_sec', 'pose_idx', 'pose_timestamp_sec',
            'gap_abs_ms', 'gap_signed_ms', 'within_tolerance', 'pos_x', 'pos_y', 'pos_z'
        ]
        with open(self.csv_matches_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(matches)

        # Print detailed report
        print("\n" + "=" * 70)
        print("PHASE 0B — SYNCHRONIZATION VERIFICATION REPORT")
        print("=" * 70)
        print(f"Total Camera Frames Logged  : {n_cam} (~30 Hz)")
        print(f"Total Ground-Truth Poses    : {n_pose} (~50 Hz)")
        print(f"Total Matched Pairs         : {matched_count}")
        print(f"Tolerance Threshold         : {self.tolerance_ms:.2f} ms")
        print(f"Matches Within Tolerance    : {within_tol_count} / {matched_count} ({tol_pass_rate:.2f}%)")
        print("-" * 70)
        print("TIMESTAMP MATCHING GAP DISTRIBUTION (Nearest-Neighbor |t_cam - t_pose|):")
        print(f"  Min Gap                   : {min_gap:.4f} ms")
        print(f"  Max Gap                   : {max_gap:.4f} ms")
        print(f"  Mean Gap                  : {mean_gap:.4f} ms")
        print(f"  Median Gap                : {median_gap:.4f} ms")
        print(f"  Std Deviation             : {std_gap:.4f} ms")
        print("-" * 70)
        print("HISTOGRAM OF TIMESTAMP GAPS (1 ms bins):")
        counts, bin_edges = np.histogram(gaps_ms, bins=10, range=(0, max(10.0, max_gap)))
        for i in range(len(counts)):
            bar = '#' * int(counts[i] * 40 / max(counts) if max(counts) > 0 else 0)
            print(f"  [{bin_edges[i]:5.2f} - {bin_edges[i+1]:5.2f} ms] : {counts[i]:4d} | {bar}")
        print("-" * 70)
        print(f"Detailed Matches CSV        : {os.path.abspath(self.csv_matches_path)}")
        print(f"Sync Status                 : {'PASS' if tol_pass_rate >= 99.0 else 'FAIL (High timestamp jitter detected)'}")
        print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Verify Camera & Ground-Truth Topic Synchronization")
    parser.add_argument(
        '--cam-topic',
        default='/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
        help="ROS 2 camera image topic"
    )
    parser.add_argument(
        '--pose-topic',
        default='/world/default/dynamic_pose/info',
        help="ROS 2 ground-truth pose topic"
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=15.0,
        help="Recording duration in seconds (default: 15.0)"
    )
    parser.add_argument(
        '--output-dir',
        default='results/sync_verification',
        help="Directory to save CSV logs"
    )
    parser.add_argument(
        '--model-name',
        default='x500_mono_cam_0',
        help="Target model name in pose TF"
    )
    parser.add_argument(
        '--tolerance-ms',
        type=float,
        default=10.0,
        help="Acceptable sync tolerance threshold in ms (default: 10.0)"
    )

    args = parser.parse_args()

    rclpy.init()
    node = SyncVerifier(
        args.cam_topic,
        args.pose_topic,
        args.duration,
        args.output_dir,
        args.model_name,
        args.tolerance_ms
    )
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("Synchronization check interrupted by user.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
