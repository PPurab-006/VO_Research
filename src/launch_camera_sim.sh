#!/usr/bin/env bash
# Launch script for PX4 SITL + Gazebo with Mono Camera & ROS 2 Bridge (Phase 0A)

set -e

PX4_DIR="/home/purab/PX4-Autopilot"
MODEL="gz_x500_mono_cam"

echo "============================================================"
echo "Starting PX4 SITL + Gazebo Simulation (Model: ${MODEL})"
echo "============================================================"

# Source ROS 2 setup if available
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source /opt/ros/lyrical/setup.bash
fi

# 1. Start PX4 SITL in background
WORLD_NAME="${PX4_GZ_WORLD:-default}"
echo "Launching PX4 SITL in background (World: ${WORLD_NAME})..."
PX4_GZ_WORLD="${WORLD_NAME}" make -C "${PX4_DIR}" px4_sitl "${MODEL}" &
PX4_PID=$!

# Cleanup handler
cleanup() {
    echo "Shutting down PX4 SITL and ROS bridge..."
    kill "${PX4_PID}" 2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "px4" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 2. Wait for Gazebo transport topic to appear
echo "Waiting for Gazebo camera topic..."
until gz topic -l 2>/dev/null | grep -q "camera/image"; do
    sleep 1
done
echo "Gazebo camera topic detected!"

# 3. Start ROS 2 GZ Bridge
echo "Starting ros_gz_bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /world/${WORLD_NAME}/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/${WORLD_NAME}/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /world/${WORLD_NAME}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V

