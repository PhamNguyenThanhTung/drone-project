import subprocess, time, os
from pymavlink import mavutil

subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
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

print("1. Connecting MAVLink...", flush=True)
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

print("2. Waiting for EKF2...", flush=True)
t_wait = time.time()
while time.time() - t_wait < 25.0:
    msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
    if msg and msg.lat != 0:
        print(f"GPS Fix: {msg.lat/1e7:.6f}, {msg.lon/1e7:.6f}", flush=True)
        break
    time.sleep(0.5)

time.sleep(2.0)

print("3. Arming Quadcopter...", flush=True)
for _ in range(5):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1.0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(0.3)

print("4. Streaming Warm-up OFFBOARD setpoints (vz = -1.0 m/s)...", flush=True)
for _ in range(20):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0x05C7,
        0, 0, 0,
        0.0, 0.0, -1.0,
        0, 0, 0,
        0.0, 0.0
    )
    time.sleep(0.05)

print("5. Switching to OFFBOARD mode...", flush=True)
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    0, 0, 0, 0, 0
)

print("6. Climbing in OFFBOARD mode for 8 seconds...", flush=True)
t_climb = time.time()
ground_alt = None
while time.time() - t_climb < 8.0:
    # Send climb velocity setpoint at 20 Hz
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0x05C7,
        0, 0, 0,
        0.0, 0.0, -1.0,
        0, 0, 0,
        0.0, 0.0
    )
    msg = m.recv_match(type=["GPS_RAW_INT", "LOCAL_POSITION_NED"], blocking=True, timeout=0.05)
    if msg:
        if msg.get_type() == "GPS_RAW_INT":
            alt = msg.alt / 1000.0
            if ground_alt is None:
                ground_alt = alt
            rel_alt = alt - ground_alt
            print(f"[{time.time()-t_climb:4.1f}s] CLIMBING: RelAlt = {rel_alt:4.2f} m (AMSL = {alt:.2f} m)", flush=True)
        elif msg.get_type() == "LOCAL_POSITION_NED":
            print(f"[{time.time()-t_climb:4.1f}s] LOCAL_NED:  Z-Alt = {-msg.z:4.2f} m, Vz = {msg.vz:+.2f} m/s", flush=True)

print("7. Hovering at altitude...", flush=True)
t_hover = time.time()
while time.time() - t_hover < 5.0:
    # Send hover setpoint (vx=0, vy=0, vz=0)
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0x05C7,
        0, 0, 0,
        0.0, 0.0, 0.0,
        0, 0, 0,
        0.0, 0.0
    )
    msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=0.05)
    if msg:
        alt = msg.alt / 1000.0
        rel_alt = alt - ground_alt if ground_alt else 0
        print(f"[{time.time()-t_hover:4.1f}s] HOVERING: RelAlt = {rel_alt:4.2f} m", flush=True)

px4_proc.kill()
subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
subprocess.run("pkill -9 -f \"gz si[m]\" 2>/dev/null || true", shell=True)
print(">>> Finished climb test!", flush=True)
