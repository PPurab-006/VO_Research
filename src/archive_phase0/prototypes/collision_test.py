#!/usr/bin/env python3
"""
Dedicated Collision Test Flight Script
Flies straight along the Gazebo +X axis (East) at v = 1.2 m/s towards the wall at (10.0, 0.0, 2.5).

PX4 MAVLink NED mapping:
  vx_ned = North velocity (dY_gz / dt) = 0.0 m/s
  vy_ned = East velocity  (dX_gz / dt) = 1.2 m/s
  z_ned  = Down position  (-Z_gz)      = -2.5 m
"""

import math
import time
import sys
import threading
from pymavlink import mavutil

def main():
    print("Connecting to PX4 SITL on udp:127.0.0.1:14540...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14540')

    stop_hb = False
    def heartbeat_loop():
        while not stop_hb:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(0.5)

    hb_thread = threading.Thread(target=heartbeat_loop, daemon=True)
    hb_thread.start()

    print("Waiting for heartbeat from PX4...")
    master.wait_heartbeat(timeout=30)
    target_sys = master.target_system if master.target_system != 0 else 1
    target_comp = master.target_component if master.target_component != 0 else 1
    print(f"Heartbeat received from system {target_sys}, component {target_comp}")

    # Set SITL failsafe bypass parameters
    def set_p(name, val):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), val, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.05)

    set_p('NAV_DLL_ACT', 0)
    set_p('NAV_RCL_ACT', 0)
    set_p('COM_RCL_EXCEPT', 4)
    set_p('COM_ARM_WO_GPS', 1)
    set_p('CBRK_SUPPLY_CHK', 894565)
    set_p('MIS_TAKEOFF_ALT', 3)
    time.sleep(0.5)

    # 1. Arm vehicle
    print("1. Arming drone...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    # 2. Takeoff to 2.5m altitude
    print("2. Issuing Takeoff command to 2.5m...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 2.5
    )
    time.sleep(6.0)

    # MASK_POS_Z_VEL_XY = 1507 (0x05E3) -> USE Position Z (z = -2.5m), Velocity X (vx), Velocity Y (vy), Yaw Rate
    MASK_POS_Z_VEL_XY = 1507

    def send_setpoint(x=0.0, y=0.0, z=-2.5, vx=0.0, vy=1.2, vz=0.0, yaw=0.0, yaw_rate=0.0, mask=MASK_POS_Z_VEL_XY):
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            x, y, z,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            yaw, yaw_rate
        )

    # Target wall: Gazebo ENU X = 10.0m, Y = 0.0m, Z = 2.5m
    speed = 1.2 # m/s
    vx_ned = 0.0  # North
    vy_ned = speed # East (maps to +X in Gazebo ENU)

    # 3. Pre-stream setpoints BEFORE requesting OFFBOARD mode
    print(f"3. Pre-streaming OFFBOARD setpoints towards Gazebo ENU (+X wall) at vy_ned={vy_ned:.2f} m/s for 1.5s...")
    pre_start = time.time()
    while time.sleep(0.05) or (time.time() - pre_start < 1.5):
        send_setpoint(z=-2.5, vx=vx_ned, vy=vy_ned)

    # 4. Explicitly request OFFBOARD mode via MAV_CMD_DO_SET_MODE
    print("4. Requesting OFFBOARD mode via MAV_CMD_DO_SET_MODE (param1=1, param2=6)...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 6, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

    # 5. Verify OFFBOARD mode transition
    print("5. Verifying vehicle mode transition...")
    mode_verified = False
    for _ in range(25):
        send_setpoint(z=-2.5, vx=vx_ned, vy=vy_ned)
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
        if hb and hb.get_srcSystem() == target_sys and hb.get_srcComponent() == target_comp:
            main_m = (hb.custom_mode >> 16) & 0xFF
            is_offboard = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) or (main_m == 6)
            if is_offboard:
                mode_verified = True
                print("   [MODE VERIFIED] Drone in OFFBOARD mode!")
                break

    if not mode_verified:
        print("   [RETRYING MODE] Re-sending MAV_CMD_DO_SET_MODE...")
        master.mav.command_long_send(
            target_sys, target_comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, 1, 6, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)

    # 6. Stream HEAD-ON wall collision trajectory for 20 seconds
    flight_duration = 20.0
    print(f"6. Streaming HEAD-ON wall collision trajectory (speed={speed}m/s, duration={flight_duration}s)...")
    flight_start = time.time()

    while time.time() - flight_start < flight_duration:
        send_setpoint(z=-2.5, vx=vx_ned, vy=vy_ned, vz=0.0)
        time.sleep(0.05)

    # 7. Request LAND
    print("7. Collision test trajectory complete. Requesting LAND...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(3.0)

    stop_hb = True
    print("Dedicated Collision Test Execution Complete.")

if __name__ == '__main__':
    main()
