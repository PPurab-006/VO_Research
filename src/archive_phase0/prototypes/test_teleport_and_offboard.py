import time
import subprocess
import math
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from pymavlink import mavutil

print("1. Teleporting drone to (5.0, 5.0, 2.5m) in Gazebo...")
cmd = (
    'gz service -s /world/boxworld_obstacles_tight/set_pose --reqtype gz.msgs.Pose '
    '--reptype gz.msgs.Boolean --timeout 2000 --req '
    '\'name: "x500_mono_cam_0", position: {x: 5.0, y: 5.0, z: 2.5}\''
)
res = subprocess.run(cmd, shell=True, capture_output=True, text=True)
print(f"   Teleport result: {res.stdout.strip()}")

time.sleep(1.0)

print("2. Connecting MAVLink...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print("   [HEARTBEAT OK]")

def set_p(name, val):
    master.mav.param_set_send(master.target_system, master.target_component, name.encode('utf-8'), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.05)

set_p('NAV_DLL_ACT', 0)
set_p('NAV_RCL_ACT', 0)
set_p('COM_RCL_EXCEPT', 4)
set_p('COM_ARM_WO_GPS', 1)
set_p('CBRK_SUPPLY_CHK', 894565)
set_p('CBRK_IO_SAFETY', 22027)

print("3. Arming...")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21968, 0, 0, 0, 0, 0)
time.sleep(0.5)

print("4. Pre-streaming OFFBOARD setpoints...")
MASK_POS_Z_VEL_XY = 1507
for _ in range(30):
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        MASK_POS_Z_VEL_XY,
        0.0, 0.0, -2.5,
        1.2, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.20
    )
    time.sleep(0.05)

print("5. Switching OFFBOARD...")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
time.sleep(1.0)

print("6. Verification flight (5 seconds)...")
for i in range(50):
    t = i * 0.1
    vx = 1.2 * math.cos(0.20 * t)
    vy = 1.2 * math.sin(0.20 * t)
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        MASK_POS_Z_VEL_XY,
        0.0, 0.0, -2.5,
        vx, vy, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.20
    )
    time.sleep(0.1)

print("Flight test complete.")

