#!/usr/bin/env python3
"""
Targeted PX4 SITL ARM/OFFBOARD diagnostic for gate course world.
Captures detailed MAVLink state: heartbeat, COMMAND_ACK, SYS_STATUS, EKF_STATUS_REPORT.
Reports exactly WHY arming fails.
"""

import time
import sys
import threading
from pymavlink import mavutil

def main():
    mav_addr = "udpin:0.0.0.0:14540"
    print(f"Connecting to PX4 SITL at {mav_addr}...")
    master = mavutil.mavlink_connection(mav_addr)

    # GCS heartbeat thread
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

    print("Waiting for PX4 heartbeat...")
    master.wait_heartbeat(timeout=30)
    target_sys = master.target_system if master.target_system != 0 else 1
    target_comp = master.target_component if master.target_component != 0 else 1
    print(f"Heartbeat received! SysID={target_sys}, CompID={target_comp}")

    # Set failsafe bypass parameters
    def set_p(name, val, ptype=mavutil.mavlink.MAV_PARAM_TYPE_INT32):
        master.mav.param_set_send(target_sys, target_comp, name.encode('utf-8'), val, ptype)
        time.sleep(0.1)

    print("\n=== Setting failsafe bypass parameters ===")
    set_p('NAV_DLL_ACT', 0)
    set_p('NAV_RCL_ACT', 0)
    set_p('COM_RCL_EXCEPT', 4)
    set_p('COM_ARM_WO_GPS', 1)
    set_p('CBRK_SUPPLY_CHK', 894565)
    time.sleep(0.5)

    # Drain any buffered messages
    while master.recv_match(blocking=False):
        pass

    # === Phase 1: Monitor pre-arm state for 3 seconds ===
    print("\n=== Phase 1: Pre-arm state monitoring (3s) ===")
    phase1_end = time.time() + 3.0
    while time.time() < phase1_end:
        msg = master.recv_match(
            type=['HEARTBEAT', 'SYS_STATUS', 'STATUSTEXT', 'ESTIMATOR_STATUS', 
                  'EXTENDED_SYS_STATE', 'LOCAL_POSITION_NED'],
            blocking=True, timeout=0.5
        )
        if msg:
            mtype = msg.get_type()
            if mtype == 'HEARTBEAT' and msg.get_srcSystem() == target_sys:
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                custom = msg.custom_mode
                sys_status = msg.system_status
                print(f"  [HB] armed={armed}, custom_mode={custom} (main={custom>>16}), "
                      f"sys_status={sys_status}, base_mode=0x{msg.base_mode:02X}")
            elif mtype == 'SYS_STATUS':
                print(f"  [SYS] present=0x{msg.onboard_control_sensors_present:08X}, "
                      f"enabled=0x{msg.onboard_control_sensors_enabled:08X}, "
                      f"health=0x{msg.onboard_control_sensors_health:08X}")
            elif mtype == 'STATUSTEXT':
                print(f"  [STATUSTEXT] sev={msg.severity}: {msg.text}")
            elif mtype == 'ESTIMATOR_STATUS':
                print(f"  [EKF] flags=0x{msg.flags:04X}, vel_ratio={msg.vel_ratio:.3f}, "
                      f"pos_horiz_ratio={msg.pos_horiz_ratio:.3f}, "
                      f"pos_vert_ratio={msg.pos_vert_ratio:.3f}")
            elif mtype == 'LOCAL_POSITION_NED':
                print(f"  [POS] x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}, "
                      f"vx={msg.vx:.4f}, vy={msg.vy:.4f}, vz={msg.vz:.4f}")

    # === Phase 2: Pre-stream setpoints ===
    print("\n=== Phase 2: Pre-streaming setpoints (1.5s) ===")
    MASK_POS_YAW = 3576
    def send_sp(x, y, z, yaw=0.0):
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            target_sys, target_comp,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MASK_POS_YAW,
            x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0
        )

    pre_start = time.time()
    while time.time() - pre_start < 1.5:
        send_sp(0.0, 0.0, -1.55, 0.0)
        time.sleep(0.05)

    # === Phase 3: Attempt ARM with force-arm ===
    print("\n=== Phase 3: ARM attempt (force-arm param2=21968) ===")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21968, 0, 0, 0, 0, 0
    )

    # Wait for COMMAND_ACK and monitor state for 5s
    arm_acked = False
    arm_result = None
    phase3_end = time.time() + 5.0
    while time.time() < phase3_end:
        send_sp(0.0, 0.0, -1.55, 0.0)
        msg = master.recv_match(
            type=['HEARTBEAT', 'COMMAND_ACK', 'STATUSTEXT', 'SYS_STATUS'],
            blocking=True, timeout=0.2
        )
        if msg:
            mtype = msg.get_type()
            if mtype == 'COMMAND_ACK':
                cmd_id = msg.command
                result = msg.result
                result_names = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
                               3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS'}
                rname = result_names.get(result, f'UNKNOWN({result})')
                print(f"  [ACK] command={cmd_id}, result={rname} ({result})")
                if cmd_id == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    arm_acked = True
                    arm_result = result
            elif mtype == 'HEARTBEAT' and msg.get_srcSystem() == target_sys:
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                custom = msg.custom_mode
                print(f"  [HB] armed={armed}, custom_mode={custom} (main={custom>>16})")
                if armed:
                    print("  >>> VEHICLE IS ARMED <<<")
            elif mtype == 'STATUSTEXT':
                print(f"  [STATUSTEXT] sev={msg.severity}: {msg.text}")
            elif mtype == 'SYS_STATUS':
                pass  # Skip verbose sys_status during arm phase

    if not arm_acked:
        print("  WARNING: No COMMAND_ACK received for ARM command!")

    # === Phase 4: Attempt OFFBOARD mode ===
    print("\n=== Phase 4: OFFBOARD mode request ===")
    master.mav.command_long_send(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 6, 0, 0, 0, 0, 0
    )

    phase4_end = time.time() + 3.0
    while time.time() < phase4_end:
        send_sp(0.0, 0.0, -1.55, 0.0)
        msg = master.recv_match(
            type=['HEARTBEAT', 'COMMAND_ACK', 'STATUSTEXT'],
            blocking=True, timeout=0.2
        )
        if msg:
            mtype = msg.get_type()
            if mtype == 'COMMAND_ACK':
                cmd_id = msg.command
                result = msg.result
                result_names = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
                               3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS'}
                rname = result_names.get(result, f'UNKNOWN({result})')
                print(f"  [ACK] command={cmd_id}, result={rname} ({result})")
            elif mtype == 'HEARTBEAT' and msg.get_srcSystem() == target_sys:
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                custom = msg.custom_mode
                main_mode = custom >> 16
                print(f"  [HB] armed={armed}, custom_mode={custom} (main={main_mode}), "
                      f"{'OFFBOARD' if main_mode == 6 else 'NOT OFFBOARD'}")
            elif mtype == 'STATUSTEXT':
                print(f"  [STATUSTEXT] sev={msg.severity}: {msg.text}")

    # === Phase 5: Check local position for 3s ===
    print("\n=== Phase 5: Position check (3s) ===")
    phase5_end = time.time() + 3.0
    while time.time() < phase5_end:
        send_sp(0.0, 0.0, -1.55, 0.0)
        msg = master.recv_match(type=['LOCAL_POSITION_NED'], blocking=True, timeout=0.5)
        if msg:
            print(f"  [POS] x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}")

    stop_hb = True
    print("\n=== DIAGNOSTIC COMPLETE ===")

if __name__ == '__main__':
    main()
