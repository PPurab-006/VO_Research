#!/usr/bin/env python3
"""
PX4 Native MAVLink Mission Execution Script for Gate Course.

Replaces open-loop / custom setpoint streaming with PX4's native mission executor:
1. Connects to PX4 SITL via MAVLink (UDP 14540).
2. Sets PX4 mission parameters:
   - MPC_XY_CRUISE = 0.60 m/s (horizontal cruise speed)
   - NAV_ACC_RAD = 0.50 m (waypoint acceptance radius)
   - MIS_YAWMODE = 1 (hold mission item yaw setpoint at each waypoint)
3. Converts existing gate course ENU waypoints to global (Lat, Lon, RelAlt, Yaw) items.
4. Uploads mission to PX4 via MAVLink MISSION_ITEM_INT protocol.
5. Switches mode to AUTO.MISSION, arms vehicle, and lets PX4's native mission executor navigate the course.
6. Monitors MISSION_ITEM_REACHED MAVLink messages until completion.
"""

import time
import math
import sys
import threading
import numpy as np
from pymavlink import mavutil

def enu_offset_to_lat_lon(x_enu, y_enu, lat0, lon0):
    """Converts ENU (x, y) offsets in meters to (Lat, Lon) relative to reference (lat0, lon0)."""
    r_earth = 6378137.0
    d_lat = (y_enu / r_earth) * (180.0 / math.pi)
    d_lon = (x_enu / (r_earth * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
    return lat0 + d_lat, lon0 + d_lon

def main():
    print("======================================================================")
    print("STARTING PX4 NATIVE MISSION EXECUTION FOR GATE COURSE (Gates 1 -> 7)")
    print("======================================================================")

    mav_addr = "udpin:0.0.0.0:14540"
    print(f"Connecting to PX4 SITL via MAVLink at {mav_addr}...")
    master = mavutil.mavlink_connection(mav_addr)

    # Start background GCS heartbeat thread
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
    print(f"Heartbeat received! System ID: {target_sys}, Component ID: {target_comp}")

    # Set PX4 parameters for SITL mission execution
    def set_param_float(name, val):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.05)

    def set_param_int(name, val):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), int(val), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.05)

    print("Configuring PX4 mission & SITL parameters...")
    set_param_int('NAV_DLL_ACT', 0)
    set_param_int('NAV_RCL_ACT', 0)
    set_param_int('COM_RCL_EXCEPT', 4)
    set_param_int('COM_ARM_WO_GPS', 1)
    set_param_int('CBRK_SUPPLY_CHK', 894565)
    set_param_int('CBRK_IO_SAFETY', 22027)
    set_param_int('EKF2_MAG_TYPE', 5)

    # Core Mission Parameters
    set_param_float('MPC_XY_CRUISE', 0.60)   # Cruise speed: 0.60 m/s
    set_param_float('NAV_ACC_RAD', 0.50)     # Waypoint acceptance radius: 0.50 m
    set_param_int('MIS_YAWMODE', 1)          # Hold mission item yaw heading (param4)
    set_param_float('MIS_TAKEOFF_ALT', 1.55) # Initial takeoff altitude
    time.sleep(0.5)

    # Obtain Home position
    print("Requesting Home position from PX4 EKF...")
    home_lat, home_lon, home_alt = 47.397742, 8.545594, 488.0
    for _ in range(5):
        master.mav.command_long_send(
            target_sys, target_comp,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        msg = master.recv_match(type='HOME_POSITION', blocking=True, timeout=1.0)
        if msg:
            home_lat = msg.latitude / 1e7
            home_lon = msg.longitude / 1e7
            home_alt = msg.altitude / 1000.0
            print(f"Home Position Received: Lat={home_lat:.7f}, Lon={home_lon:.7f}, Alt={home_alt:.2f}m")
            break
        time.sleep(0.2)

    # Gate waypoints in Gazebo ENU: (x, y, z, yaw_deg, desc)
    waypoints_enu = [
        (0.00,  0.00, 1.55,  90.0, "Pre-Gate 1 Takeoff & Hover"),
        (0.00,  3.50, 1.55,  90.0, "Gate 1 Center"),
        (0.00,  7.70, 1.55,  90.0, "Gate 2 Center"),
        (2.75, 10.45, 2.05,  45.0, "Gate 3 Center (Turn East-North)"),
        (4.90,  7.40, 1.55, -90.0, "Gate 4 Center (Turn South)"),
        (4.90,  2.60, 1.55, -90.0, "Gate 5 Center (South)"),
        (1.10,  2.00, 2.95, 180.0, "Gate 6 Center (Elevated Climb-Turn West)"),
        (2.00,  5.30, 1.45,  90.0, "Gate 7 Center (Turn North)"),
        (2.00,  7.30, 1.45,  90.0, "Post-Gate 7 Stabilization Hover")
    ]

    # Build MAVLink mission items list
    mission_items = []
    for idx, (x_enu, y_enu, z_enu, yaw_deg, desc) in enumerate(waypoints_enu):
        lat, lon = enu_offset_to_lat_lon(x_enu, y_enu, home_lat, home_lon)
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        rel_alt = float(z_enu)
        
        if idx == 0:
            # Item 0: MAV_CMD_NAV_TAKEOFF
            cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            p1 = 0.0     # Minimum pitch
            p2 = 0.0
            p3 = 0.0
            p4 = float(yaw_deg)
        else:
            # Items 1..N-1: MAV_CMD_NAV_WAYPOINT
            cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            p1 = 0.5     # Hold time at waypoint (seconds)
            p2 = 0.50    # Acceptance radius (meters)
            p3 = 0.0     # Pass radius
            p4 = float(yaw_deg)

        mission_items.append({
            'seq': idx,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': cmd,
            'current': 1 if idx == 0 else 0,
            'autocontinue': 1,
            'param1': p1,
            'param2': p2,
            'param3': p3,
            'param4': p4,
            'x': lat_int,
            'y': lon_int,
            'z': rel_alt,
            'desc': desc
        })

    # Step 1: Clear existing mission
    print("1. Clearing existing mission on PX4...")
    master.mav.mission_clear_all_send(target_sys, target_comp, mission_type=0)
    master.recv_match(type='MISSION_ACK', blocking=True, timeout=5.0)

    # Step 2: Send Mission Count
    num_items = len(mission_items)
    print(f"2. Uploading MAVLink mission ({num_items} items) to PX4...")
    master.mav.mission_count_send(target_sys, target_comp, num_items, mission_type=0)

    # Step 3: Upload each mission item as requested by PX4
    for _ in range(num_items):
        req = master.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=5.0)
        if not req:
            print("[ERROR] Timeout waiting for MISSION_REQUEST from PX4!")
            sys.exit(1)
        seq = req.seq
        item = mission_items[seq]
        print(f"   Uploading item [{seq+1}/{num_items}]: {item['desc']} -> Cmd={item['command']}, Yaw={item['param4']}°")

        master.mav.mission_item_int_send(
            target_sys, target_comp,
            item['seq'],
            item['frame'],
            item['command'],
            item['current'],
            item['autocontinue'],
            item['param1'],
            item['param2'],
            item['param3'],
            item['param4'],
            item['x'],
            item['y'],
            item['z'],
            0 # mission_type = 0 (MAV_MISSION_TYPE_MISSION)
        )

    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5.0)
    if ack and ack.type == 0:
        print("   Mission upload ACCEPTED by PX4 (MISSION_ACK type=0)!")
    else:
        print(f"   WARNING: Mission upload ACK status = {ack.type if ack else 'TIMEOUT'}")

    # Set current mission item to 0
    master.mav.mission_set_current_send(target_sys, target_comp, 0)
    time.sleep(0.5)

    # Step 4: Switch mode to AUTO.MISSION
    print("3. Switching mode to AUTO.MISSION via MAV_CMD_DO_SET_MODE...")
    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    # PX4 Main Mode 4 = AUTO, Sub Mode 4 = AUTO_MISSION
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        4,  # PX4 Main Mode AUTO = 4
        4,  # PX4 Sub Mode AUTO_MISSION = 4
        0, 0, 0, 0
    )
    time.sleep(0.5)

    # Step 5: Arm Vehicle
    print("4. Arming vehicle for native mission execution...")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21968, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    # Step 6: Monitor PX4 Native Mission Execution Progress
    print("======================================================================")
    print("EXECUTING PX4 NATIVE MISSION (Monitored via MISSION_ITEM_REACHED)")
    print("======================================================================")

    last_reached_seq = -1
    mission_start_time = time.time()
    
    while time.time() - mission_start_time < 120.0:
        msg = master.recv_match(type=['MISSION_ITEM_REACHED', 'HEARTBEAT'], blocking=True, timeout=1.0)
        if msg:
            mtype = msg.get_type()
            if mtype == 'MISSION_ITEM_REACHED':
                seq = msg.seq
                if seq != last_reached_seq and seq < num_items:
                    desc = waypoints_enu[seq][4]
                    elapsed = time.time() - mission_start_time
                    print(f"   [MISSION PROGRESS] Reached Item [{seq+1}/{num_items}]: {desc} (at t={elapsed:.1f}s)")
                    last_reached_seq = seq
                    if seq == num_items - 1:
                        print("   All mission items REACHED successfully!")
                        break

    print("======================================================================")
    print("PX4 NATIVE MISSION EXECUTION COMPLETE")
    print("======================================================================")
    
    stop_hb = True

if __name__ == '__main__':
    main()
