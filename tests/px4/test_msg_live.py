import subprocess, time, os
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

px4_proc = subprocess.Popen(
    ["make", "px4_sitl", "gz_x500"],
    cwd="/home/tungt/PX4-Autopilot",
    env=env,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)

m = None
t0 = time.time()
while time.time() - t0 < 30:
    try:
        cand = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255)
        if cand.wait_heartbeat(timeout=1.0):
            m = cand
            print("Connected!", flush=True)
            break
    except Exception:
        pass
    time.sleep(0.5)

m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
time.sleep(5)

print("Arming...", flush=True)
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1.0, 0, 0, 0, 0, 0, 0
)
time.sleep(1)

print("Taking off...", flush=True)
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 3.8
)

t1 = time.time()
counts = {}
while time.time() - t1 < 10.0:
    msg = m.recv_match(blocking=True, timeout=0.1)
    if msg:
        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1
        if t in ("COMMAND_ACK", "STATUSTEXT", "GPS_RAW_INT", "ALTITUDE", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "HIGHRES_IMU"):
            if t == "GPS_RAW_INT":
                print(f"[{time.time()-t1:4.1f}s] GPS_RAW_INT: alt={msg.alt/1000.0:.2f}m, lat={msg.lat/1e7:.6f}, lon={msg.lon/1e7:.6f}, vel={msg.vel/100.0:.2f}m/s", flush=True)
            elif t == "ALTITUDE":
                print(f"[{time.time()-t1:4.1f}s] ALTITUDE: monotonic={msg.altitude_monotonic:.2f}m, rel={msg.altitude_relative:.2f}m", flush=True)
            elif t == "COMMAND_ACK":
                print(f"[{time.time()-t1:4.1f}s] COMMAND_ACK: cmd={msg.command}, result={msg.result}", flush=True)
            elif t == "STATUSTEXT":
                print(f"[{time.time()-t1:4.1f}s] STATUSTEXT: {msg.text}", flush=True)

print("Total message counts:", counts, flush=True)

px4_proc.kill()
subprocess.run("pkill -9 -f px4 2>/dev/null || true", shell=True)
subprocess.run("pkill -9 -f \"gz si[m]\" 2>/dev/null || true", shell=True)
