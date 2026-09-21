import sys
import time
import os
import subprocess
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ClusterFeatureTester(Node):
    def __init__(self, camera_topic, artifact_dir):
        super().__init__('cluster_feature_tester')
        self.bridge = CvBridge()
        self.artifact_dir = artifact_dir
        self.camera_topic = camera_topic
        self.current_frame = None
        self.sub = self.create_subscription(Image, camera_topic, self.image_cb, 10)

    def image_cb(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def get_latest_frame(self, timeout=3.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_frame is not None:
                frame = self.current_frame.copy()
                self.current_frame = None
                return frame
        return None

def teleport_drone(x, y, z, roll=0, pitch=0, yaw=0):
    # Convert Euler yaw to quaternion orientation
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    cmd = (
        f"gz service -s /world/boxworld_obstacles_tight/set_pose --reqtype gz.msgs.Pose "
        f"--reptype gz.msgs.Boolean --timeout 2000 --req "
        f"'name: \"x500_mono_cam_0\", position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'"
    )
    res = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    return "true" in res.stdout

def analyze_frame_features(img, frame_name, artifact_dir):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    # 1. GFTT (Good Features to Track - Shi-Tomasi) used in minimal_vo.py KLT mode
    gftt_pts = cv2.goodFeaturesToTrack(gray, maxCorners=2000, qualityLevel=0.001, minDistance=5)
    gftt_count = len(gftt_pts) if gftt_pts is not None else 0

    # 2. ORB Feature Detector used in minimal_vo.py ORB mode
    orb = cv2.ORB_create(nfeatures=2000, fastThreshold=5)
    orb_kps = orb.detect(gray, None)
    orb_count = len(orb_kps)

    # Define central ROI (center 50% box of frame where target cluster appears)
    roi_ymin, roi_ymax = int(h * 0.25), int(h * 0.75)
    roi_xmin, roi_xmax = int(w * 0.25), int(w * 0.75)

    gftt_roi_count = 0
    if gftt_pts is not None:
        for pt in gftt_pts:
            x, y = pt[0]
            if roi_xmin <= x <= roi_xmax and roi_ymin <= y <= roi_ymax:
                gftt_roi_count += 1

    orb_roi_count = 0
    for kp in orb_kps:
        x, y = kp.pt
        if roi_xmin <= x <= roi_xmax and roi_ymin <= y <= roi_ymax:
            orb_roi_count += 1

    # Draw visualization
    vis_img = img.copy()
    cv2.rectangle(vis_img, (roi_xmin, roi_ymin), (roi_xmax, roi_ymax), (0, 255, 255), 2)
    
    # Draw GFTT green dots
    if gftt_pts is not None:
        for pt in gftt_pts:
            x, y = int(pt[0][0]), int(pt[0][1])
            cv2.circle(vis_img, (x, y), 3, (0, 255, 0), -1)

    cv2.putText(vis_img, f"GFTT Total: {gftt_count} (ROI: {gftt_roi_count})", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(vis_img, f"ORB Total: {orb_count} (ROI: {orb_roi_count})", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

    raw_path = os.path.join(artifact_dir, f"{frame_name}.png")
    vis_path = os.path.join(artifact_dir, f"{frame_name}_features.png")
    cv2.imwrite(raw_path, img)
    cv2.imwrite(vis_path, vis_img)

    return {
        'name': frame_name,
        'raw_path': raw_path,
        'vis_path': vis_path,
        'gftt_total': gftt_count,
        'gftt_roi': gftt_roi_count,
        'orb_total': orb_count,
        'orb_roi': orb_roi_count,
        'mean_brightness': float(np.mean(gray)),
        'std_brightness': float(np.std(gray))
    }

def run_test():
    print("==========================================================================")
    print("PRE-FLIGHT CHECK: OBSTACLE CLUSTER FEATURE DENSITY & TRACKABILITY")
    print("==========================================================================")

    artifact_dir = None
    camera_topic = '/world/boxworld_obstacles_tight/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'

    rclpy.init()
    tester = ClusterFeatureTester(camera_topic, artifact_dir)

    # 4 Targeted Vantage Points
    test_points = [
        ("cluster_close_1.5m", 4.5, 0.0, 0.5, 0, 0, 0, "Cluster (6,0) at Close Range (1.5m away)"),
        ("cluster_sweet_3m",   3.0, 0.0, 1.0, 0, 0, 0, "Cluster (6,0) at Sweet-Spot Range (3.0m away)"),
        ("cluster_mid_5m",     1.0, 0.0, 1.5, 0, 0, 0, "Cluster (6,0) at Mid Range (5.0m away)"),
        ("plain_grass_baseline", -3.0, -3.0, 1.0, 0, 0, 0, "Plain Grass Ground Baseline (No Obstacles)")
    ]

    results = []

    for name, tx, ty, tz, r, p, y, desc in test_points:
        print(f"\n--- Testing Vantage Point: {desc} ---")
        print(f"    Teleporting drone to ({tx}, {ty}, {tz})...")
        ok = teleport_drone(tx, ty, tz, r, p, y)
        print(f"    Teleport status: {'SUCCESS' if ok else 'FAILED'}")
        
        time.sleep(1.0) # Wait for Gazebo rendering update

        frame = tester.get_latest_frame(timeout=3.0)
        if frame is None:
            print(f"    [WARNING] No frame received for {name}")
            continue

        res = analyze_frame_features(frame, name, artifact_dir)
        results.append(res)
        print(f"    Frame Captured & Analyzed!")
        print(f"    GFTT Features : Total = {res['gftt_total']}, ROI = {res['gftt_roi']}")
        print(f"    ORB Features  : Total = {res['orb_total']}, ROI = {res['orb_roi']}")
        print(f"    Brightness    : Mean = {res['mean_brightness']:.2f}, Std = {res['std_brightness']:.2f}")

    tester.destroy_node()
    rclpy.shutdown()

    # Print Summary Evaluation
    print("\n==========================================================================")
    print("FEATURE DENSITY & TRACKABILITY EVALUATION SUMMARY")
    print("==========================================================================")
    print(f"{'Vantage Point / Target':<30} | {'GFTT Total':<10} | {'GFTT ROI':<10} | {'ORB Total':<10} | {'ORB ROI':<10}")
    print("-" * 80)
    for r in results:
        print(f"{r['name']:<30} | {r['gftt_total']:<10} | {r['gftt_roi']:<10} | {r['orb_total']:<10} | {r['orb_roi']:<10}")
    print("==========================================================================")

if __name__ == '__main__':
    run_test()

