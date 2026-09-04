#!/usr/bin/env python3
"""
Environment Sanity Check Script for exploration_complex/in_and_out_vis.world
Tests vehicle spawn, hovering at X=15.0, Y=15.0, Z=2.5m, camera feed, and VO pipeline.
Does NOT alter controller tuning, PX4 parameters, or world geometry.
"""

import os
import sys
import time
import subprocess
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pymavlink import mavutil

WORLD_DIR = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps/exploration_complex"
WORLD_NAME = "in_and_out_vis"
PX4_DIR = "/home/purab/PX4-Autopilot"
MODEL = "gz_x500_mono_cam"
SPAWN_POSE = "15.0,15.0,0.2,0,0,0"

class CameraDumper(Node):
    def __init__(self, camera_topic, output_dir):
        super().__init__('sanity_camera_dumper')
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.count = 0
        self.sub = self.create_subscription(Image, camera_topic, self.image_cb, 10)
        self.get_logger().info(f"Subscribed to {camera_topic}")

    def image_cb(self, msg):
        if self.count < 5:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                fn = os.path.join(self.output_dir, f"sanity_frame_{self.count+1:02d}.png")
                cv2.imwrite(fn, cv_img)
                self.count += 1
                self.get_logger().info(f"Saved camera frame {self.count} -> {fn}")
            except Exception as e:
                self.get_logger().error(f"Error saving image: {e}")

def kill_sim():
    procs = ["gz-sim-main", "gz-sim-gui-client", "parameter_bridge", "px4", "ruby", "gz"]
    for p in procs:
        subprocess.run(["pkill", "-9", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.5)

def main():
    print("============================================================")
    print("ENVIRONMENT SANITY CHECK — exploration_complex/in_and_out_vis")
    print("============================================================")

    os.makedirs("results", exist_ok=True)
    kill_sim()

    maps_base = "/home/purab/Purab/Projects/ROS/configs/gazebo_maps"
    common_models = f"{maps_base}/common_models"
    
    env = os.environ.copy()
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    env["GZ_SIM_RENDER_ENGINE"] = "ogre2"
    env["PX4_GZ_WORLDS"] = WORLD_DIR
    env["PX4_GZ_WORLD"] = WORLD_NAME
    env["PX4_GZ_MODEL_POSE"] = SPAWN_POSE
    env["GZ_SIM_RESOURCE_PATH"] = f"{common_models}:{WORLD_DIR}:{PX4_DIR}/Tools/simulation/gz/models"
    env["GAZEBO_MODEL_PATH"] = f"{common_models}:{WORLD_DIR}"
    env["PYTHONUNBUFFERED"] = "1"

    print(f"[1/6] Launching PX4 SITL with world {WORLD_NAME} at spawn pose {SPAWN_POSE}...")
    px4_cmd = ["make", "-C", PX4_DIR, "px4_sitl", MODEL]
    px4_proc = subprocess.Popen(px4_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("[2/6] Waiting for Gazebo camera topic...")
    cam_gz_topic = None
    active_world_name = "default"
    for _ in range(40):
        try:
            out = subprocess.check_output(["gz", "topic", "-l"], stderr=subprocess.DEVNULL).decode('utf-8')
            for line in out.splitlines():
                if "camera/image" in line:
                    cam_gz_topic = line.strip()
                    parts = cam_gz_topic.split('/')
                    if len(parts) > 2 and parts[1] == 'world':
                        active_world_name = parts[2]
                    print(f"      Camera topic detected: {cam_gz_topic} (Active world: {active_world_name})")
                    break
            if cam_gz_topic:
                break
        except Exception:
            pass
        time.sleep(1.0)

    if not cam_gz_topic:
        print("[ERROR] Camera topic timed out!")
        kill_sim()
        return

    print(f"[3/6] Launching ros_gz_bridge for world '{active_world_name}'...")
    bridge_cmd = [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        f"/world/{active_world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
    ]
    bridge_proc = subprocess.Popen(bridge_cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)

    print("[4/6] Starting GT recorder and VO node...")
    gt_cmd = [
        "python3", "src/record_ground_truth.py",
        "--topic", f"/world/{active_world_name}/dynamic_pose/info",
        "--output-dir", "results",
        "--filename", "sanity_gt.csv",
        "--max-samples", "1500"
    ]
    gt_proc = subprocess.Popen(gt_cmd, env=env)

    vo_cmd = [
        "python3", "src/minimal_vo.py",
        "--camera-topic", f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image",
        "--output-dir", "results",
        "--output-filename", "sanity_vo.csv",
        "--max-frames", "1000",
        "--mode", "klt"
    ]
    vo_proc = subprocess.Popen(vo_cmd, env=env)

    # ROS 2 node for saving camera frames
    rclpy.init()
    ros_cam_topic = f"/world/{active_world_name}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image"
    dumper = CameraDumper(ros_cam_topic, "results")

    print("[5/6] Connecting to MAVLink and issuing Takeoff to 2.5m...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
    master.wait_heartbeat(timeout=15)
    print(f"      Heartbeat received! System {master.target_system}, Component {master.target_component}")

    # Spin dumper to capture initial frame
    for _ in range(10):
        rclpy.spin_once(dumper, timeout_sec=0.1)

    print("      Arming vehicle...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    print("      Sending Takeoff to 2.5m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 2.5
    )

    print("[6/6] Hovering for 12 seconds...")
    start_t = time.time()
    while (time.time() - start_t) < 12.0:
        rclpy.spin_once(dumper, timeout_sec=0.1)
        time.sleep(0.05)

    print("      Landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3.0)

    dumper.destroy_node()
    rclpy.shutdown()

    for p in [gt_proc, vo_proc, bridge_proc, px4_proc]:
        try:
            p.terminate()
        except Exception:
            pass

    kill_sim()
    print("[COMPLETE] Sanity check flight finished.")

if __name__ == '__main__':
    main()
