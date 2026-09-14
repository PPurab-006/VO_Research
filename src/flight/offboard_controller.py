import time
from pymavlink import mavutil

print("Connecting to PX4 MAVLink...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print(f"Connected to Sys {master.target_system}, Comp {master.target_component}")

def set_p(name, val):
    master.mav.param_set_send(master.target_system, master.target_component, name.encode('utf-8'), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.05)

print("Setting PX4 SITL bypass parameters...")
set_p('NAV_DLL_ACT', 0)
set_p('NAV_RCL_ACT', 0)
set_p('COM_RCL_EXCEPT', 4)
set_p('COM_ARM_WO_GPS', 1)
set_p('COM_ARM_MAG_STR', -1)
set_p('CBRK_SUPPLY_CHK', 894565)
set_p('CBRK_IO_SAFETY', 22027)
set_p('CBRK_USB_CHK', 197848)
set_p('MIS_TAKEOFF_ALT', 2.5)

time.sleep(0.5)

print("Arming drone...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
time.sleep(1.0)

print("Commanding TAKEOFF to 2.5m...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 2.5
)

print("Waiting 5s for takeoff...")
for i in range(5):
    time.sleep(1)
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        print(f"[{i+1}s] Local Pos Z: {-msg.z:.3f} m | Vz: {-msg.vz:.3f} m/s")
    else:
        print(f"[{i+1}s] Waiting for pose msg...")

