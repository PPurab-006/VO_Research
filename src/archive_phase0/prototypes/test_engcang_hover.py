import sys
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pymavlink import mavutil

class FrameDumper(Node):
    def __init__(self, camera_topic, output_dir):
        super().__init__('engcang_frame_dumper')
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.saved_count = 0
        self.get_logger().info(f"Frame dumper subscribed to {camera_topic}")

    def image_callback(self, msg):
        if self.saved_count < 3:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                filename = f"{self.output_dir}/engcang_frame_{self.saved_count+1:02d}.png"
                cv2.imwrite(filename, cv_img)
                self.saved_count += 1
                self.get_logger().info(f"Saved sample camera frame #{self.saved_count} to {filename}")
            except Exception as e:
                self.get_logger().error(f"Error saving image: {e}")

def run_validation():
    print("============================================================")
    print("STEP 2: MINIMAL VALIDATION FLIGHT (ENGCANG BOUNDED WORLD)")
    print("============================================================")
    
    # 1. ROS Node for dumping frames
    rclpy.init()
    output_dir = None
    camera_topic = '/world/boxworld_obstacles_tight/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'
    dumper = FrameDumper(camera_topic, output_dir)
    
    # 2. Connect to MAVLink
    print("1. Connecting to PX4 SITL MAVLink (127.0.0.1:14540)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
    master.wait_heartbeat()
    print(f"   [HEARTBEAT RECEIVED] System {master.target_system}, Component {master.target_component}")

    # Spin dumper to capture initial frame
    for _ in range(10):
        rclpy.spin_once(dumper, timeout_sec=0.1)

    # 3. Arming
    print("2. Arming drone...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    # 4. Takeoff
    print("3. Issuing Takeoff command to 2.0m altitude...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 2.0
    )
    
    # Spin ROS node during 6-second hover to capture camera frames
    print("4. Hovering at 2.0m altitude and dumping sample frames...")
    start_time = time.time()
    while time.sleep(0.1) is None and (time.time() - start_time) < 6.0:
        rclpy.spin_once(dumper, timeout_sec=0.1)

    # 5. Land
    print("5. Issuing LAND command...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3.0)

    dumper.destroy_node()
    rclpy.shutdown()
    print("============================================================")
    print("ENGCANG MINIMAL VALIDATION FLIGHT COMPLETE")
    print("============================================================")

if __name__ == '__main__':
    run_validation()

