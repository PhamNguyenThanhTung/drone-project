import subprocess, time, os, sys
from pymavlink import mavutil

subprocess.run("pkill -9 -f px4 2>/dev/null || true", shell=True)
subprocess.run("pkill -9 -f \"gz si[m]\" 2>/dev/null || true", shell=True)
time.sleep(1)

install_world = "install -m 644 /home/tungt/drone-project/gazebo/worlds/person_tracking_no_trees.sdf /home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds/person_tracking_no_trees.sdf"
install_model = "install -m 644 /home/tungt/drone-project/gazebo/models/x500/model.sdf /home/tungt/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf"
subprocess.run(install_world, shell=True)
subprocess.run(install_model, shell=True)

env = os.environ.copy()
env["HEADLESS"] = "1"
env["WORLD_NAME"] = "person_tracking_no_trees"
env["PX4_GZ_WORLD"] = "person_tracking_no_trees"
env["PX4_SIM_MODEL"] = "x500"
env["GZ_SIM_RESOURCE_PATH"] = "/home/tungt/drone-project/gazebo/models:/home/tungt/drone-project/gazebo/worlds:/home/tungt/PX4-Autopilot/Tools/simulation/gz/models:/home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds"
env["LD_LIBRARY_PATH"] = "/usr/lib/wsl/lib:" + env.get("LD_LIBRARY_PATH", "")

print(">>> 1. Starting PX4 SITL...", flush=True)
px4_proc = subprocess.Popen(
    ["make", "px4_sitl", "gz_x500"],
    cwd="/home/tungt/PX4-Autopilot",
    env=env,
    stdout=open("/tmp/sitl_test_live.log", "w"),
    stderr=subprocess.STDOUT
)

print(">>> 2. Connecting MAVLink at udpin:0.0.0.0:14540...", flush=True)
m = None
t0 = time.time()
while time.time() - t0 < 30:
    try:
        cand = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255)
        if cand.wait_heartbeat(timeout=1.0):
            m = cand
            print(f">>> CONNECTED! PX4 SysID={cand.target_system}", flush=True)
            break
    except Exception:
        pass
    time.sleep(0.5)

if not m:
    print("FAILED to connect!", flush=True)
    px4_proc.kill()
    sys.exit(1)

# Enable telemetry
m.mav.request_data_stream_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
)

print(">>> 3. Waiting for Preflight Checks / EKF2 to settle...", flush=True)
armed = False
t_arm_start = time.time()
while time.time() - t_arm_start < 25.0:
    # Try sending ARM
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1.0, 0, 0, 0, 0, 0, 0
    )

    # Check responses
    t_check = time.time()
    while time.time() - t_check < 1.0:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg:
            mtype = msg.get_type()
            if mtype == "COMMAND_ACK" and msg.command == 400:
                print(f"[{time.time()-t_arm_start:4.1f}s] ARM Command ACK result={msg.result}", flush=True)
                if msg.result == 0:
                    armed = True
                    break
            elif mtype == "HEARTBEAT":
                # Check base_mode & MAV_MODE_FLAG_SAFETY_ARMED (128)
                if msg.base_mode & 128:
                    armed = True
                    print(f"[{time.time()-t_arm_start:4.1f}s] HEARTBEAT indicates ARMED! base_mode={msg.base_mode}", flush=True)
                    break
            elif mtype == "STATUSTEXT":
                print(f"[{time.time()-t_arm_start:4.1f}s] STATUSTEXT: {msg.text}", flush=True)
    if armed:
        break
    time.sleep(0.5)

if not armed:
    print(">>> Arming failed after 25s!", flush=True)
    px4_proc.kill()
    sys.exit(1)

print("\n>>> 4. ARMED SUCCESSFULLY! Sending TAKEOFF to 4.0m...", flush=True)
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 4.0
)

print(">>> 5. Monitoring Altitude Telemetry for 15s...", flush=True)
t_climb = time.time()
max_alt = 0.0
while time.time() - t_climb < 15.0:
    msg = m.recv_match(blocking=True, timeout=0.2)
    if msg:
        mtype = msg.get_type()
        if mtype == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
            max_alt = max(max_alt, alt)
            print(f"[{time.time()-t_climb:4.1f}s] GLOBAL_POS: RelAlt = {alt:4.2f} m, Lat = {msg.lat/1e7:.6f}, Lon = {msg.lon/1e7:.6f}", flush=True)
        elif mtype == "LOCAL_POSITION_NED":
            z = -msg.z
            max_alt = max(max_alt, z)
            print(f"[{time.time()-t_climb:4.1f}s] LOCAL_NED:  Z-Alt = {z:4.2f} m, Vx = {msg.vx:+.2f}, Vy = {msg.vy:+.2f}, Vz = {msg.vz:+.2f}", flush=True)
        elif mtype == "ALTITUDE":
            alt = getattr(msg, "altitude_relative", 0.0)
            if not (alt != alt): # check nan
                max_alt = max(max_alt, alt)
                print(f"[{time.time()-t_climb:4.1f}s] ALTITUDE:   RelAlt = {alt:4.2f} m", flush=True)
    time.sleep(0.1)

print(f"\n=======================================================", flush=True)
print(f"TEST RESULT: Max Altitude = {max_alt:.2f} m", flush=True)
if max_alt >= 3.0:
    print("SUCCESS: Drone successfully armed and flew up to target altitude!", flush=True)
else:
    print("Takeoff did not complete.", flush=True)
print(f"=======================================================", flush=True)

px4_proc.kill()
subprocess.run("pkill -9 -f px4 2>/dev/null || true", shell=True)
subprocess.run("pkill -9 -f \"gz si[m]\" 2>/dev/null || true", shell=True)
