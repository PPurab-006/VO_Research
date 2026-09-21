# Environment Specification

## System Environment
- **Operating System**: Ubuntu 26.04 LTS (*resolute*)
- **ROS 2 Distribution**: `lyrical` (`/opt/ros/lyrical`)
- **Gazebo Simulator**: Gazebo Sim / Tools `10.4.0`
- **PX4 Autopilot**: `~/PX4-Autopilot` (Git commit: `v1.18.0-beta1-209-g8aba32c862`)
- **Python**: `3.14.4`
- **Build Tools**: `git 2.53.0`, `cmake 4.2.3`, `colcon-common-extensions`

## Key Python Packages
- `opencv-python` (`cv2`): `4.10.0`
- `numpy`: `2.3.5`
- `matplotlib`: `3.10.7+dfsg1`
- `rclpy`: `ROS 2 Lyrical` system package
- `rosbag2_py`: `ROS 2 Lyrical` system package
- `evo`: `1.37.0`
- `scipy`: `1.18.0`
- `rosbags`: `0.11.5`
- `seaborn`: `0.13.2`

## Reproducibility & Build Rules
- All scripts rely strictly on the pinned environment specifications above.
- No manual environment mutations or unscripted build steps.
