#!/usr/bin/env python3
"""
Test script for Offboard Roll Oscillation Control in PX4 SITL via pymavlink
"""

import time
import math
import numpy as np
from pymavlink import mavutil
from scipy.spatial.transform import Rotation as R_scipy

def euler_to_quat_wxyz(roll_rad, pitch_rad, yaw_rad):
    """Returns quaternion [w, x, y, z] from euler angles (xyz, rad)."""
    r = R_scipy.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
    q_xyzw = r.as_quat()
    # Scipy returns [x, y, z, w], pymavlink set_attitude_target expects [w, x, y, z]
    return [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]

def main():
    mav_addr = "udpin:0.0.0.0:14540"
    print(f"Connecting to PX4 SITL at {mav_addr}...")
    master = mavutil.mavlink_connection(mav_addr)
    master.wait_heartbeat()
    print("Heartbeat received!")

    target_sys = master.target_system
    target_comp = master.target_component

    def send_att_target(roll_deg, pitch_deg=0.0, yaw_deg=0.0, thrust=0.55):
        q = euler_to_quat_wxyz(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
        # type_mask = 7 (ignore roll/pitch/yaw body rates, use quaternion + thrust)
        master.mav.set_attitude_target_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            7,  # type_mask
            q,
            0.0, 0.0, 0.0, # body rates (ignored by mask=7)
            thrust
        )

    print("Pre-streaming attitude setpoints...")
    start = time.time()
    while time.time() - start < 1.5:
        send_att_target(0.0)
        time.sleep(0.05)

    print("Arming & Setting OFFBOARD mode...")
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21968, 0, 0, 0, 0, 0)
    time.sleep(0.2)
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    print("Testing 10 deg roll oscillation at 0.5 Hz for 5 seconds...")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        elapsed = time.time() - t0
        roll = 10.0 * math.sin(2.0 * math.pi * 0.5 * elapsed)
        send_att_target(roll)
        time.sleep(0.02)

    print("Landing...")
    master.mav.command_long_send(target_sys, target_comp, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    print("Done!")

if __name__ == '__main__':
    main()
