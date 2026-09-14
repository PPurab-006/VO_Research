#!/usr/bin/env python3
"""
Camera Verification Script for ROS 2 + Gazebo SITL (Phase 0A)

Subscribes to the specified ROS 2 camera image topic, captures N frames,
saves the images to disk, and logs message metadata and timestamps into a CSV file.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class CameraVerifier(Node):
    def __init__(self, topic_name, num_frames, output_dir):
        super().__init__('camera_verifier')
        self.topic_name = topic_name
        self.target_frames = num_frames
        self.output_dir = output_dir
        self.img_dir = os.path.join(output_dir, 'images')
        os.makedirs(self.img_dir, exist_ok=True)

        self.csv_path = os.path.join(output_dir, 'frame_timestamps.csv')
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        
        self.captured_count = 0
        self.records = []

        self.get_logger().info(f"Subscribing to camera topic: '{self.topic_name}'")
        self.get_logger().info(f"Target frame count: {self.target_frames}")
        self.get_logger().info(f"Saving artifacts to: '{self.output_dir}'")

        self.sub = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        if self.captured_count >= self.target_frames:
            return

        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        total_sec = sec + nanosec * 1e-9

        filename = f"frame_{self.captured_count:04d}.png"
        filepath = os.path.join(self.img_dir, filename)

        # Convert Image message to OpenCV BGR image
        if HAS_CV_BRIDGE:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                cv_img = self._convert_numpy(msg)
        else:
            cv_img = self._convert_numpy(msg)

        # Save frame to disk
        cv2.imwrite(filepath, cv_img)

        record = {
            'frame_idx': self.captured_count,
            'timestamp_sec': sec,
            'timestamp_nanosec': nanosec,
            'timestamp_total_sec': f"{total_sec:.9f}",
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'filename': filename
        }
        self.records.append(record)
        self.captured_count += 1

        self.get_logger().info(
            f"[{self.captured_count}/{self.target_frames}] Saved {filename} "
            f"(Stamp: {sec}.{nanosec:09d} s, Res: {msg.width}x{msg.height}, Enc: {msg.encoding})"
        )

        if self.captured_count >= self.target_frames:
            self.save_csv_and_report()
            rclpy.shutdown()

    def _convert_numpy(self, msg: Image) -> np.ndarray:
        if msg.encoding in ['rgb8', 'bgr8']:
            img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding == 'rgb8':
                return cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            return img_np
        elif msg.encoding in ['mono8']:
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        else:
            raise ValueError(f"Unsupported encoding for numpy fallback: {msg.encoding}")

    def save_csv_and_report(self):
        # Save records to CSV
        fieldnames = [
            'frame_idx', 'timestamp_sec', 'timestamp_nanosec',
            'timestamp_total_sec', 'width', 'height', 'encoding', 'filename'
        ]
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        # Check monotonic timestamps
        ts_list = [float(r['timestamp_total_sec']) for r in self.records]
        diffs = [ts_list[i] - ts_list[i-1] for i in range(1, len(ts_list))]
        is_monotonic = all(d > 0 for d in diffs) if diffs else True
        avg_dt = np.mean(diffs) if diffs else 0.0
        fps = 1.0 / avg_dt if avg_dt > 0 else 0.0

        print("\n" + "=" * 60)
        print("CAMERA VERIFICATION SUMMARY REPORT")
        print("=" * 60)
        print(f"Total Frames Saved   : {self.captured_count}")
        print(f"Output Directory     : {os.path.abspath(self.output_dir)}")
        print(f"CSV Metadata File    : {os.path.abspath(self.csv_path)}")
        print(f"Resolution           : {self.records[0]['width']} x {self.records[0]['height']}")
        print(f"Encoding             : {self.records[0]['encoding']}")
        print(f"Monotonic Timestamps : {'PASS (Strictly Increasing)' if is_monotonic else 'FAIL (Non-monotonic detected)'}")
        print(f"Average Frame Delta  : {avg_dt * 1000.0:.3f} ms")
        print(f"Estimated Rate (FPS) : {fps:.2f} Hz")
        print("=" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Verify ROS 2 Camera Topic")
    parser.add_argument(
        '--topic',
        default='/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image',
        help="ROS 2 camera image topic name"
    )
    parser.add_argument(
        '--num-frames',
        type=int,
        default=20,
        help="Number of frames to capture (default: 20)"
    )
    parser.add_argument(
        '--output-dir',
        default='results/camera_verification',
        help="Directory to save image files and CSV log"
    )

    args = parser.parse_args()

    rclpy.init()
    node = CameraVerifier(args.topic, args.num_frames, args.output_dir)
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
