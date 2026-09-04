import time
from pymavlink import mavutil

print("Connecting to PX4 SITL...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
target_sys = master.target_system if master.target_system != 0 else 1
target_comp = master.target_component if master.target_component != 0 else 1
print(f"Heartbeat OK (Sys {target_sys}, Comp {target_comp})")

print("Waiting 10s for PX4 EKF2 alignment...")
time.sleep(10.0)

print("1. Arming vehicle...")
master.mav.command_long_send(
    target_sys, target_comp,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
time.sleep(1.0)

print("2. Issuing Takeoff command to 2.5m...")
master.mav.command_long_send(
    target_sys, target_comp,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 2.5
)

print("3. Monitoring climb (10s)...")
for i in range(10):
    time.sleep(1.0)
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        print(f"[{i+1}s] Local Pos Z: {-msg.z:.3f} m | Vz: {-msg.vz:.3f} m/s")
    else:
        print(f"[{i+1}s] Waiting for pose message...")

