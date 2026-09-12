#!/usr/bin/env python3
"""
Single Trajectory Dataset Recorder for Phase 2A (Camera Images + GT Attitude)

Subscribes to camera image topic (~30 Hz) and dynamic pose topic (~50 Hz),
enforces use_sim_time=True via Gazebo /clock, saves raw PNG frames to disk,
and writes dataset_gt.csv for offline RAW vs EIS Visual Odometry evaluation.
"""

import argparse
import csv
import os
import sys
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class CameraDatasetRecorder(Node):
    def __init__(self, cam_topic, pose_topic, output_dir, model_name):
        super().__init__('camera_dataset_recorder')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.cam_topic = cam_topic
        self.pose_topic = pose_topic
        self.output_dir = output_dir
        self.model_name = model_name

        self.img_dir = os.path.join(output_dir, 'images')
        os.makedirs(self.img_dir, exist_ok=True)

        self.cam_csv_path = os.path.join(output_dir, 'camera_frames.csv')
        self.gt_csv_path = os.path.join(output_dir, 'dataset_gt.csv')

        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        self.cam_records = []
        self.gt_file = open(self.gt_csv_path, 'w', newline='')
        self.gt_fieldnames = [
            'sample_idx', 'timestamp_sec', 'timestamp_nanosec',
            'timestamp_total_sec', 'pos_x', 'pos_y', 'pos_z',
            'rot_x', 'rot_y', 'rot_z', 'rot_w'
        ]
        self.gt_writer = csv.DictWriter(self.gt_file, fieldnames=self.gt_fieldnames)
        self.gt_writer.writeheader()
        self.gt_sample_count = 0

        self.get_logger().info(f"Subscribing to Camera Topic : '{self.cam_topic}'")
        self.get_logger().info(f"Subscribing to Pose Topic   : '{self.pose_topic}'")
        self.get_logger().info(f"Output Directory            : '{os.path.abspath(self.output_dir)}'")

        self.sub_cam = self.create_subscription(Image, self.cam_topic, self.cam_callback, 50)
        self.sub_pose = self.create_subscription(TFMessage, self.pose_topic, self.pose_callback, 100)

    def cam_callback(self, msg: Image):
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        total_sec = sec + nanosec * 1e-9

        frame_idx = len(self.cam_records)
        filename = f"frame_{frame_idx:04d}.png"
        filepath = os.path.join(self.img_dir, filename)

        if HAS_CV_BRIDGE:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            except Exception:
                cv_img = self._convert_numpy(msg)
        else:
            cv_img = self._convert_numpy(msg)

        cv2.imwrite(filepath, cv_img)

        self.cam_records.append({
            'frame_idx': frame_idx,
            'timestamp_sec': sec,
            'timestamp_nanosec': nanosec,
            'timestamp_total_sec': f"{total_sec:.9f}",
            'filename': filename,
            'width': msg.width,
            'height': msg.height
        })

        if frame_idx % 30 == 0:
            self.get_logger().info(f"Recorded frame #{frame_idx} (Sim Stamp: {total_sec:.3f} s)")

    def pose_callback(self, msg: TFMessage):
        if not msg.transforms:
            return

        target_tf = None
        for tf in msg.transforms:
            if self.model_name in tf.child_frame_id or tf.child_frame_id == self.model_name:
                target_tf = tf
                break

        if target_tf is None and len(msg.transforms) > 0:
            target_tf = msg.transforms[0]

        if target_tf is None:
            return

        stamp = target_tf.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        sec = stamp.sec
        nanosec = stamp.nanosec
        total_sec = sec + nanosec * 1e-9

        pos = target_tf.transform.translation
        rot = target_tf.transform.rotation

        row = {
            'sample_idx': str(self.gt_sample_count),
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
        self.gt_writer.writerow(row)
        self.gt_file.flush()
        self.gt_sample_count += 1

    def _convert_numpy(self, msg: Image) -> np.ndarray:
        if msg.encoding in ['rgb8', 'bgr8']:
            img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
        elif msg.encoding in ['mono8']:
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

    def close_and_save(self):
        if not self.gt_file.closed:
            self.gt_file.flush()
            self.gt_file.close()

        # Write camera frames CSV
        cam_fieldnames = ['frame_idx', 'timestamp_sec', 'timestamp_nanosec', 'timestamp_total_sec', 'filename', 'width', 'height']
        with open(self.cam_csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=cam_fieldnames)
            writer.writeheader()
            writer.writerows(self.cam_records)

        print("\n" + "=" * 70)
        print("DATASET RECORDING COMPLETE")
        print("=" * 70)
        print(f"Total Camera Frames Saved : {len(self.cam_records)}")
        print(f"Total GT Samples Saved    : {self.gt_sample_count}")
        print(f"Dataset Output Directory  : {os.path.abspath(self.output_dir)}")
        print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Record Single Trajectory Dataset for Phase 2A")
    parser.add_argument('--cam-topic', default='/world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image')
    parser.add_argument('--pose-topic', default='/world/default/dynamic_pose/info')
    parser.add_argument('--output-dir', required=True, help="Directory to save dataset")
    parser.add_argument('--model-name', default='x500_mono_cam_0')
    args = parser.parse_args()

    rclpy.init()
    node = CameraDatasetRecorder(args.cam_topic, args.pose_topic, args.output_dir, args.model_name)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.close_and_save()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
