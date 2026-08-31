#!/usr/bin/env python3
"""
Live Gazebo Ground-Truth Pose Recorder (ROS 2)

Subscribes directly to Gazebo /world/<world>/dynamic_pose/info (tf2_msgs/msg/TFMessage)
and logs position & orientation at high resolution (~50 Hz) to a CSV file.

Includes immediate file open & flush for robust streaming.
"""

import os
import sys
import csv
import argparse
import signal
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class LiveGroundTruthRecorder(Node):
    def __init__(self, topic_name: str, output_dir: str, filename: str, model_name: str, max_samples: int):
        super().__init__('live_ground_truth_recorder')
        self.topic_name = topic_name
        self.output_dir = output_dir
        self.filename = filename
        self.model_name = model_name
        self.max_samples = max_samples
        self.sample_count = 0

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, self.filename)

        self.get_logger().info(f"Subscribing to LIVE ground-truth pose topic: '{self.topic_name}'")
        self.get_logger().info(f"Target model: '{self.model_name}' | Max samples: {self.max_samples}")
        self.get_logger().info(f"Saving output to: '{os.path.abspath(self.csv_path)}'")

        # Open CSV file immediately and write header
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.fieldnames = [
            'sample_idx', 'timestamp_sec', 'timestamp_nanosec',
            'timestamp_total_sec', 'pos_x', 'pos_y', 'pos_z',
            'rot_x', 'rot_y', 'rot_z', 'rot_w'
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()
        self.csv_file.flush()

        self.sub = self.create_subscription(
            TFMessage,
            self.topic_name,
            self.pose_callback,
            100
        )

    def pose_callback(self, msg: TFMessage):
        if not msg.transforms:
            return

        if self.sample_count >= self.max_samples:
            self.close_csv()
            rclpy.shutdown()
            return

        target_tf = None
        for tf in msg.transforms:
            if (tf.child_frame_id and (self.model_name in tf.child_frame_id or tf.child_frame_id == self.model_name)) or \
               (tf.header.frame_id and (self.model_name in tf.header.frame_id or tf.header.frame_id == self.model_name)):
                target_tf = tf
                break

        if target_tf is None:
            target_tf = msg.transforms[0]

        stamp = target_tf.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            if msg.transforms[0].header.stamp.sec != 0 or msg.transforms[0].header.stamp.nanosec != 0:
                stamp = msg.transforms[0].header.stamp
            else:
                stamp = self.get_clock().now().to_msg()

        if stamp.sec == 0 and stamp.nanosec == 0:
            import time
            now_f = time.time()
            sec = int(now_f)
            nanosec = int((now_f - sec) * 1e9)
        else:
            sec = stamp.sec
            nanosec = stamp.nanosec
        total_sec = sec + nanosec * 1e-9

        pos = target_tf.transform.translation
        rot = target_tf.transform.rotation

        row = {
            'sample_idx': str(self.sample_count),
            'timestamp_sec': str(sec),
            'timestamp_nanosec': str(nanosec),
            'timestamp_total_sec': f"{total_sec:.9f}",
            'pos_x': f"{pos.x:.16f}",
            'pos_y': f"{pos.y:.16f}",
            'pos_z': f"{pos.z:.16f}",
            'rot_x': f"{rot.x:.16f}",
            'rot_y': f"{rot.y:.16f}",
            'rot_z': f"{rot.z:.16f}",
            'rot_w': f"{rot.w:.16f}"
        }
        self.writer.writerow(row)
        self.csv_file.flush()

        self.sample_count += 1

        if self.sample_count % 100 == 0 or self.sample_count == 1:
            self.get_logger().info(
                f"[{self.sample_count}] LIVE GT Stamp: {total_sec:.3f} s | "
                f"Pos: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}) m | "
                f"Quat(xyzw): ({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})"
            )

        if self.sample_count >= self.max_samples:
            self.close_csv()
            rclpy.shutdown()

    def close_csv(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()
            print("\n" + "=" * 70)
            print("LIVE GROUND-TRUTH RECORDING COMPLETE")
            print("=" * 70)
            print(f"Total Live Samples Logged : {self.sample_count}")
            print(f"Output CSV Path           : {os.path.abspath(self.csv_path)}")
            print("=" * 70 + "\n")

    def save_csv(self):
        self.close_csv()


def main():
    parser = argparse.ArgumentParser(description="Record Live Gazebo Ground-Truth Pose Stream")
    parser.add_argument(
        '--topic',
        default='/world/textured/dynamic_pose/info',
        help="ROS 2 ground-truth pose topic"
    )
    parser.add_argument(
        '--output-dir',
        default='results',
        help="Directory to save CSV log"
    )
    parser.add_argument(
        '--filename',
        default='ground_truth_textured_REAL.csv',
        help="Output CSV filename"
    )
    parser.add_argument(
        '--model-name',
        default='x500_mono_cam_0',
        help="Target model name in TF message"
    )
    parser.add_argument(
        '--max-samples',
        type=int,
        default=5000,
        help="Max samples to record"
    )

    args = parser.parse_args()

    rclpy.init()
    node = LiveGroundTruthRecorder(args.topic, args.output_dir, args.filename, args.model_name, args.max_samples)

    def sig_handler(sig, frame):
        print("\n[INFO] Signal received. Flushing and closing CSV before exiting...")
        node.close_csv()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except BaseException:
        pass
    finally:
        node.close_csv()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
