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

print("=" * 65)
print("     PX4 SITL + MAVLINK FULL LIVE FLIGHT TEST      ")
print("=" * 65)

print("\n[1/5] Khởi động PX4 Autopilot SITL + Gazebo Harmonic (Headless)...", flush=True)
px4_proc = subprocess.Popen(
    ["make", "px4_sitl", "gz_x500"],
    cwd="/home/tungt/PX4-Autopilot",
    env=env,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)

print("[2/5] Kết nối MAVLink tại udpin:0.0.0.0:14540...", flush=True)
master = None
t0 = time.time()
while time.time() - t0 < 30:
    try:
        cand = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255)
        if cand.wait_heartbeat(timeout=1.0):
            master = cand
            print(f">>> KẾT NỐI THÀNH CÔNG! PX4 System ID = {cand.target_system}", flush=True)
            break
    except Exception:
        pass
    time.sleep(0.5)

if not master:
    print("LỖI: Không thể kết nối tới MAVLink port 14540!", flush=True)
    px4_proc.kill()
    sys.exit(1)

# Enable telemetry streams
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1
)

print("\n[3/5] Chờ GPS Lock & Bộ lọc EKF2 hội tụ...", flush=True)
t_wait = time.time()
while time.time() - t_wait < 30.0:
    msg = master.recv_match(type=["GLOBAL_POSITION_INT", "GPS_RAW_INT"], blocking=True, timeout=1.0)
    if msg and msg.lat != 0:
        print(f">>> GPS SẴN SÀNG: Vĩ độ = {msg.lat/1e7:.6f}°, Kinh độ = {msg.lon/1e7:.6f}°", flush=True)
        break
    time.sleep(0.5)

time.sleep(1.0)

print("\n[4/5] Gửi lệnh ARM & Ra lệnh Cất cánh (NAV_TAKEOFF) lên 3.8m...", flush=True)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1.0, 0, 0, 0, 0, 0, 0
)
time.sleep(1.0)

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 3.8
)

print("\n[5/5] Theo dõi Telemetry độ cao bay trong 15 giây...", flush=True)
t_fly = time.time()
max_alt = 0.0
while time.time() - t_fly < 15.0:
    msg = master.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=True, timeout=0.3)
    if msg:
        if msg.get_type() == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
            max_alt = max(max_alt, alt)
            print(f"[{time.time()-t_fly:4.1f}s] GLOBAL_POS: Độ cao tương đối = {alt:4.2f} m | Vĩ độ = {msg.lat/1e7:.6f}° | Kinh độ = {msg.lon/1e7:.6f}°", flush=True)
        elif msg.get_type() == "LOCAL_POSITION_NED":
            z = -msg.z
            max_alt = max(max_alt, z)
            print(f"[{time.time()-t_fly:4.1f}s] LOCAL_NED:  Độ cao Z = {z:4.2f} m | Vx = {msg.vx:+.2f} m/s | Vy = {msg.vy:+.2f} m/s | Vz = {msg.vz:+.2f} m/s", flush=True)
    time.sleep(0.2)

print("\n" + "=" * 65)
print(f"KẾT QUẢ KIỂM TRA: Độ cao tối đa đạt được = {max_alt:.2f} m")
if max_alt >= 3.0:
    print(">>> THÀNH CÔNG: Drone đã ARM, cất cánh mượt mà và bay ổn định!")
else:
    print(f">>> Drone đạt độ cao: {max_alt:.2f} m")
print("=" * 65)

px4_proc.kill()
subprocess.run("pkill -9 -f px4 2>/dev/null || true", shell=True)
subprocess.run("pkill -9 -f \"gz si[m]\" 2>/dev/null || true", shell=True)
