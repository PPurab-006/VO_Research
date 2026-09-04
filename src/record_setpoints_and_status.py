#!/usr/bin/env python3
"""
Diagnostic Setpoint & Vehicle State Logger for PX4 SITL (Phase 0 Diagnostic)

Connects to PX4 MAVLink telemetry stream (udp:127.0.0.1:14550 or udp:127.0.0.1:14540)
and logs:
  - timestamp_total_sec
  - setpoint_z (position target if available, or target altitude)
  - setpoint_vx, setpoint_vy, setpoint_vz
  - type_mask
  - px4_custom_mode, px4_main_mode, px4_sub_mode
  - publish_gap_ms

Saves output to results/setpoint_log_diagnostic.csv.
"""

import os
import sys
import time
import csv
import argparse
import signal
from pymavlink import mavutil

def main():
    parser = argparse.ArgumentParser(description="Log MAVLink Setpoints and Vehicle Mode")
    parser.add_argument('--output-dir', default='results', help="Directory for output CSV")
    parser.add_argument('--filename', default='setpoint_log_diagnostic.csv', help="Output CSV filename")
    parser.add_argument('--connection', default='udp:127.0.0.1:14550', help="MAVLink connection string")
    parser.add_argument('--duration', type=float, default=45.0, help="Max logging duration in seconds")

    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    csv_path = os.path.join(args.output_dir, args.filename)

    print(f"Connecting to MAVLink telemetry on {args.connection}...")
    try:
        master = mavutil.mavlink_connection(args.connection)
        master.wait_heartbeat(timeout=10)
        print(f"Heartbeat received! System ID: {master.target_system}, Component ID: {master.target_component}")
    except Exception as e:
        print(f"Error connecting to MAVLink on {args.connection}: {e}")
        # Fallback to 14540 if 14550 isn't outputting
        print("Retrying on udp:127.0.0.1:14540...")
        master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
        master.wait_heartbeat(timeout=10)
        print(f"Heartbeat received on 14540! System ID: {master.target_system}")

    fieldnames = [
        'sample_idx', 'timestamp_total_sec', 'msg_type',
        'setpoint_x', 'setpoint_y', 'setpoint_z',
        'setpoint_vx', 'setpoint_vy', 'setpoint_vz',
        'type_mask', 'base_mode', 'custom_mode', 'nav_state', 'publish_gap_ms'
    ]

    csv_file = open(csv_path, 'w', newline='')
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    csv_file.flush()

    print(f"Logging setpoints and vehicle state to: {os.path.abspath(csv_path)}")

    running = True
    def stop_handler(sig, frame):
        nonlocal running
        print("\nStopping setpoint logger...")
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    sample_idx = 0
    last_setpoint_time = None
    start_time = time.time()

    curr_base_mode = 0
    curr_custom_mode = 0
    curr_nav_state = 0

    while running and (time.time() - start_time < args.duration):
        msg = master.recv_match(blocking=True, timeout=0.5)
        if not msg:
            continue

        now = time.time()
        mtype = msg.get_type()

        if mtype == 'HEARTBEAT':
            curr_base_mode = msg.base_mode
            curr_custom_mode = msg.custom_mode
        elif mtype == 'EXTENDED_SYS_STATE':
            curr_nav_state = getattr(msg, 'landed_state', 0)
        elif mtype in ['POSITION_TARGET_LOCAL_NED', 'SET_POSITION_TARGET_LOCAL_NED']:
            gap_ms = (now - last_setpoint_time) * 1000.0 if last_setpoint_time else 0.0
            last_setpoint_time = now

            row = {
                'sample_idx': sample_idx,
                'timestamp_total_sec': f"{now:.6f}",
                'msg_type': mtype,
                'setpoint_x': f"{msg.x:.4f}",
                'setpoint_y': f"{msg.y:.4f}",
                'setpoint_z': f"{msg.z:.4f}",
                'setpoint_vx': f"{msg.vx:.4f}",
                'setpoint_vy': f"{msg.vy:.4f}",
                'setpoint_vz': f"{msg.vz:.4f}",
                'type_mask': getattr(msg, 'type_mask', 0),
                'base_mode': curr_base_mode,
                'custom_mode': curr_custom_mode,
                'nav_state': curr_nav_state,
                'publish_gap_ms': f"{gap_ms:.2f}"
            }
            writer.writerow(row)
            csv_file.flush()
            sample_idx += 1

            if sample_idx % 20 == 0 or sample_idx == 1:
                print(f"[{sample_idx}] Time: {now:.2f} | Target Z: {msg.z:.2f} | Vz: {msg.vz:.2f} | Mode(base/custom): {curr_base_mode}/{curr_custom_mode} | Gap: {gap_ms:.1f}ms")

    csv_file.close()
    print("Setpoint logging complete. Total records:", sample_idx)

if __name__ == '__main__':
    main()
