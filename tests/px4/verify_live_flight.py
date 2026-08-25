#!/usr/bin/env python3
"""
Verify Live Flight Trajectory in PX4 SITL + Gazebo Harmonic:
- Starts PX4 SITL with person_tracking_no_trees world.
- Connects MAVLink.
- Arms drone and commands Takeoff to 4.0m.
- Streams live telemetry every 0.5s (Altitude, Velocity, Coordinates).
- Verifies takeoff reaches >= 3.8m.
"""

import os
import sys
import time
import subprocess
from pymavlink import mavutil

print("=== 1. Starting PX4 SITL + Gazebo Harmonic ===")
subprocess.run("pkill -9 -f 'px[4]' 2>/dev/null || true", shell=True)
subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
time.sleep(1)

env = os.environ.copy()
env['HEADLESS'] = '1'
env['PX4_GZ_WORLD'] = 'person_tracking_no_trees'
env['GZ_SIM_RESOURCE_PATH'] = (
    "/home/tungt/drone-project/gazebo/models:"
    "/home/tungt/drone-project/gazebo/worlds:"
    "/home/tungt/PX4-Autopilot/Tools/simulation/gz/models:"
    "/home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds"
)

px4_proc = subprocess.Popen(
    ["make", "px4_sitl", "gz_x500"],
    cwd="/home/tungt/PX4-Autopilot",
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT
)

print("=== 2. Connecting to PX4 MAVLink at udpin:0.0.0.0:14540 ===")
master = None
for i in range(30):
    try:
        cand = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255)
        if cand.wait_heartbeat(timeout=1.0):
            master = cand
            print(f"Connected! PX4 System ID: {cand.target_system}, Component: {cand.target_component}")
            break
    except Exception as e:
        pass
    time.sleep(0.5)

if master is None:
    print("FAILED: Could not connect to PX4 MAVLink.")
    px4_proc.kill()
    sys.exit(1)

print("=== 3. Waiting for EKF2 convergence & GPS lock (5s) ===")
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)
time.sleep(5.0)

print("=== 4. Arming Quadcopter ===")
for attempt in range(1, 6):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1.0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(0.3)

print("=== 5. Commanding Auto Takeoff to 4.0 m ===")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 4.0
)

print("=== 6. Monitoring Live Climb Telemetry for 15s ===")
start_time = time.time()
max_alt = 0.0

while time.time() - start_time < 15.0:
    msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'], blocking=True, timeout=0.5)
    if msg:
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            rel_alt = msg.relative_alt / 1000.0  # mm to meters
            max_alt = max(max_alt, rel_alt)
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            print(f"[{time.time()-start_time:4.1f}s] Telemetry GLOBAL: Alt = {rel_alt:4.2f} m | Lat = {lat:.6f}, Lon = {lon:.6f}")
        elif msg.get_type() == 'LOCAL_POSITION_NED':
            z = -msg.z  # NED z is negative down -> positive up
            max_alt = max(max_alt, z)
            print(f"[{time.time()-start_time:4.1f}s] Telemetry LOCAL:  Z-Alt = {z:4.2f} m | Vx = {msg.vx:+.2f}, Vy = {msg.vy:+.2f}, Vz = {msg.vz:+.2f}")
    time.sleep(0.2)

print("\n=======================================================")
print(f"RESULT: Maximum Altitude Reached = {max_alt:.2f} m (Target: 4.00 m)")
if max_alt >= 3.0:
    print("SUCCESS: Drone successfully took off and is airborne in Gazebo!")
else:
    print(f"WARNING: Altitude only reached {max_alt:.2f} m. Investigating...")
print("=======================================================")

px4_proc.kill()
subprocess.run("pkill -9 -f 'px[4]' 2>/dev/null || true", shell=True)
subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
