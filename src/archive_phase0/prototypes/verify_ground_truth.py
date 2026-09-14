#!/usr/bin/env python3
"""
Ground-Truth Pose Verification Script for ROS 2 + Gazebo SITL (Phase 0A)

Subscribes to the ROS 2 ground-truth pose topic (/world/default/dynamic_pose/info),
extracts the 6-DoF pose of the drone model, logs N seconds of pose data to CSV,
and reports stability, rate, and coordinate metrics.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class GroundTruthVerifier(Node):
    def __init__(self, topic_name, duration_sec, output_dir, model_name):
        super().__init__('ground_truth_verifier')
        self.topic_name = topic_name
        self.target_duration = duration_sec
        self.output_dir = output_dir
        self.model_name = model_name

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'ground_truth_poses.csv')

        self.records = []
        self.start_wall_time = None
        self.sample_count = 0

        self.get_logger().info(f"Subscribing to ground-truth topic: '{self.topic_name}'")
        self.get_logger().info(f"Target model: '{self.model_name}' | Logging duration: {self.target_duration} s")
        self.get_logger().info(f"Saving output to: '{self.csv_path}'")

        self.sub = self.create_subscription(
            TFMessage,
            self.topic_name,
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: TFMessage):
        if not msg.transforms:
            return

        if self.start_wall_time is None:
            self.start_wall_time = time.time()

        elapsed = time.time() - self.start_wall_time
        if elapsed > self.target_duration:
            self.save_csv_and_report()
            rclpy.shutdown()
            return

        # Locate transform for target model or default to first transform
        target_tf = None
        if self.model_name:
            for tf in msg.transforms:
                if tf.child_frame_id == self.model_name or self.model_name in tf.child_frame_id:
                    target_tf = tf
                    break

        if target_tf is None:
            target_tf = msg.transforms[0]

        stamp = target_tf.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        sec = stamp.sec
        nanosec = stamp.nanosec
        total_sec = sec + nanosec * 1e-9


        pos = target_tf.transform.translation
        rot = target_tf.transform.rotation

        record = {
            'sample_idx': self.sample_count,
            'timestamp_sec': sec,
            'timestamp_nanosec': nanosec,
            'timestamp_total_sec': f"{total_sec:.9f}",
            'pos_x': pos.x,
            'pos_y': pos.y,
            'pos_z': pos.z,
            'rot_x': rot.x,
            'rot_y': rot.y,
            'rot_z': rot.z,
            'rot_w': rot.w
        }
        self.records.append(record)
        self.sample_count += 1

        if self.sample_count % 25 == 0 or self.sample_count == 1:
            self.get_logger().info(
                f"[{self.sample_count}] Stamp: {total_sec:.3f} s | "
                f"Pos: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}) m | "
                f"Quat(xyzw): ({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})"
            )

    def save_csv_and_report(self):
        if not self.records:
            self.get_logger().error("No pose records captured!")
            return

        fieldnames = [
            'sample_idx', 'timestamp_sec', 'timestamp_nanosec',
            'timestamp_total_sec', 'pos_x', 'pos_y', 'pos_z',
            'rot_x', 'rot_y', 'rot_z', 'rot_w'
        ]
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        # Quantitative Analysis
        ts_list = [float(r['timestamp_total_sec']) for r in self.records]
        diffs = [ts_list[i] - ts_list[i-1] for i in range(1, len(ts_list))]
        is_monotonic = all(d >= 0 for d in diffs) if diffs else True
        avg_dt = np.mean(diffs) if diffs else 0.0
        fps = 1.0 / avg_dt if avg_dt > 0 else 0.0

        pos_x = np.array([r['pos_x'] for r in self.records])
        pos_y = np.array([r['pos_y'] for r in self.records])
        pos_z = np.array([r['pos_z'] for r in self.records])

        rot_x = np.array([r['rot_x'] for r in self.records])
        rot_y = np.array([r['rot_y'] for r in self.records])
        rot_z = np.array([r['rot_z'] for r in self.records])
        rot_w = np.array([r['rot_w'] for r in self.records])

        print("\n" + "=" * 65)
        print("GROUND-TRUTH POSE VERIFICATION SUMMARY REPORT")
        print("=" * 65)
        print(f"Total Samples Logged : {self.sample_count}")
        print(f"CSV Output Path      : {os.path.abspath(self.csv_path)}")
        print(f"Monotonic Timestamps : {'PASS (Strictly Non-decreasing)' if is_monotonic else 'FAIL'}")
        print(f"Average Sampling Rate: {fps:.2f} Hz (dt = {avg_dt * 1000.0:.2f} ms)")
        print("-" * 65)
        print("POSITION METRICS (Meters, World ENU Frame):")
        print(f"  X-axis : Mean = {np.mean(pos_x):.6f}, StdDev = {np.std(pos_x):.6f}, Min = {np.min(pos_x):.6f}, Max = {np.max(pos_x):.6f}")
        print(f"  Y-axis : Mean = {np.mean(pos_y):.6f}, StdDev = {np.std(pos_y):.6f}, Min = {np.min(pos_y):.6f}, Max = {np.max(pos_y):.6f}")
        print(f"  Z-axis : Mean = {np.mean(pos_z):.6f}, StdDev = {np.std(pos_z):.6f}, Min = {np.min(pos_z):.6f}, Max = {np.max(pos_z):.6f}")
        print("-" * 65)
        print("ORIENTATION QUATERNION (x, y, z, w):")
        print(f"  Quat X : Mean = {np.mean(rot_x):.6f}, StdDev = {np.std(rot_x):.6f}")
        print(f"  Quat Y : Mean = {np.mean(rot_y):.6f}, StdDev = {np.std(rot_y):.6f}")
        print(f"  Quat Z : Mean = {np.mean(rot_z):.6f}, StdDev = {np.std(rot_z):.6f}")
        print(f"  Quat W : Mean = {np.mean(rot_w):.6f}, StdDev = {np.std(rot_w):.6f}")
        print("=" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Verify ROS 2 Ground-Truth Pose Topic")
    parser.add_argument(
        '--topic',
        default='/world/default/dynamic_pose/info',
        help="ROS 2 ground-truth pose topic"
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=10.0,
        help="Logging duration in seconds (default: 10.0)"
    )
    parser.add_argument(
        '--output-dir',
        default='results/ground_truth_verification',
        help="Directory to save CSV log"
    )
    parser.add_argument(
        '--model-name',
        default='x500_mono_cam_0',
        help="Target model name in TF message"
    )

    args = parser.parse_args()

    rclpy.init()
    node = GroundTruthVerifier(args.topic, args.duration, args.output_dir, args.model_name)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("Verification interrupted by user.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
