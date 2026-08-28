#!/usr/bin/env python3
"""
Test Long-Distance Multi-Turn Autonomous Tracking.

Evaluates:
- Straight line pacing (>10m)
- 90° Left Turns
- 90° Right Turns
- 180° Turnarounds
- Continuous visual servoing & heading memory
- Real-time telemetry logging of Altitude, Yaw, Position, Tracking Substates
"""

import os
import sys
import time
import json
import subprocess
import threading
from pymavlink import mavutil

PROJECT = '/home/tungt/drone-project'
LOG_DIR = f'{PROJECT}/logs'
LOG_CSV = f'{LOG_DIR}/long_path_tracking_telemetry.csv'
WORLD_SDF = f'{PROJECT}/gazebo/worlds/person_tracking_long_path.sdf'

def cleanup():
    subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f yolo_detector_no[d] 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f motion_arbite[r] 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'parameter_bridge /camera' 2>/dev/null || true", shell=True)
    time.sleep(1.0)

def main():
    print("=" * 85)
    print("   LONG-DISTANCE MULTI-TURN AUTONOMOUS VISION TRACKING TEST (70M PATH)   ")
    print("=" * 85)

    os.makedirs(LOG_DIR, exist_ok=True)
    cleanup()

    env = os.environ.copy()
    env['HEADLESS'] = '1'
    env['PX4_GZ_WORLD'] = 'person_tracking_long_path'
    env['GZ_SIM_RESOURCE_PATH'] = (
        f"{PROJECT}/gazebo/models:{PROJECT}/gazebo/worlds:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/models:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds"
    )
    env['LD_LIBRARY_PATH'] = f"/usr/lib/wsl/lib:{env.get('LD_LIBRARY_PATH', '')}"

    procs = []

    try:
        # 1. Start Gazebo
        print("\n[1/5] Khởi động Gazebo Harmonic (world: person_tracking_long_path)...")
        gz_log = open('/tmp/test_long_path_gz.log', 'w')
        procs.append(subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '--headless-rendering', WORLD_SDF],
            env=env, stdout=gz_log, stderr=subprocess.STDOUT
        ))

        for _ in range(30):
            res = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, env=env)
            if 'clock' in res.stdout:
                print("  Gazebo world clock is UP.")
                break
            time.sleep(1.0)

        # 2. Start PX4 SITL
        print("\n[2/5] Khởi động PX4 Autopilot SITL...")
        px4_log = open('/tmp/test_long_path_px4.log', 'w')
        procs.append(subprocess.Popen(
            ['make', 'px4_sitl', 'gz_x500'],
            cwd='/home/tungt/PX4-Autopilot', env=env,
            stdout=px4_log, stderr=subprocess.STDOUT
        ))

        # 3. Start ROS-GZ bridge
        print("\n[3/5] Khởi động ROS 2 Parameter Bridge cho Camera...")
        bridge_log = open('/tmp/test_long_path_bridge.log', 'w')
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            stdout=bridge_log, stderr=subprocess.STDOUT
        ))

        # 4. Start YOLO
        print("\n[4/5] Khởi động YOLOv8 Detector Node...")
        yolo_env = env.copy()
        yolo_env['PYTHONPATH'] = f"{PROJECT}/ros2_ws/src/vision_tracking:{env.get('PYTHONPATH', '')}"
        yolo_log = open('/tmp/test_long_path_yolo.log', 'w')
        procs.append(subprocess.Popen(
            ['python3', f'{PROJECT}/ros2_ws/src/vision_tracking/vision_tracking/yolo_detector_node.py',
             '--ros-args', '-p', 'image_topic:=/camera/image_raw',
             '-p', f'model_path:={PROJECT}/yolov8n.pt',
             '-p', 'device:=cuda:0',
             '-p', 'conf:=0.30',
             '-p', 'show_debug_image:=false'],
            cwd=PROJECT, env=yolo_env,
            stdout=yolo_log, stderr=subprocess.STDOUT
        ))

        # 5. Connect GCS Channel
        print("\n[5/5] Kết nối GCS Telemetry Channel...")
        mav_gcs = None
        t_conn = time.time()
        while time.time() - t_conn < 30.0:
            try:
                cand = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=254)
                if cand.wait_heartbeat(timeout=1.0):
                    mav_gcs = cand
                    break
            except Exception:
                pass
            time.sleep(0.5)

        def gcs_heartbeat_worker():
            while True:
                if mav_gcs is not None:
                    try:
                        mav_gcs.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            0, mavutil.mavlink.MAV_STATE_ACTIVE
                        )
                    except Exception:
                        pass
                time.sleep(1.0)
        threading.Thread(target=gcs_heartbeat_worker, daemon=True).start()

        # 6. Start MotionArbiter
        print("\n[6/6] Khởi động MotionArbiter...")
        arbiter_env = env.copy()
        arbiter_env['PYTHONPATH'] = f"{PROJECT}:{env.get('PYTHONPATH', '')}"
        arbiter_log = open('/tmp/test_long_path_arbiter.log', 'w')
        procs.append(subprocess.Popen(
            ['python3', f'{PROJECT}/motion_arbiter.py',
             '--ros-args', '-p', 'takeoff_alt:=3.8', '-p', 'auto_takeoff:=true',
             '-p', 'mavlink:=udpin:0.0.0.0:14540'],
            cwd=PROJECT, env=arbiter_env,
            stdout=arbiter_log, stderr=subprocess.STDOUT
        ))

        # Wait for drone to reach 3.8m
        print("\n>>> Chờ Drone cất cánh và đạt độ cao 3.8m...")
        t_climb = time.time()
        while time.time() - t_climb < 50.0:
            msg = mav_gcs.recv_match(type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'], blocking=True, timeout=0.5)
            if msg:
                alt = (float(msg.relative_alt) / 1000.0) if msg.get_type() == 'GLOBAL_POSITION_INT' else -float(msg.z)
                if alt >= 3.6:
                    print(f"  Drone đã đạt độ cao {alt:.2f}m! Bắt đầu ghi log đường bay dài...")
                    break
            time.sleep(0.2)

        # Telemetry logging loop for 60s
        print("\n" + "=" * 95)
        print(">>> BẮT ĐẦU GHI LOG BÁM ĐƯỜNG DÀI (ĐI THẲNG -> RẼ TRÁI -> RẼ PHẢI -> QUAY ĐẦU)...")
        print(f"{'Time(s)':<8} | {'Pos X':<7} | {'Pos Y':<7} | {'Alt (m)':<8} | {'Yaw (°)':<8} | {'YawRate':<9} | {'Cmd Vx':<7} | {'Err X':<7} | {'Err Y':<7} | {'Substate':<18}")
        print("-" * 95)

        csv_file = open(LOG_CSV, 'w')
        csv_file.write("timestamp,elapsed_s,pos_x,pos_y,alt_m,yaw_deg,yaw_rate_deg_s,cmd_vx,cmd_vy,cmd_yaw_rate,err_x,err_y,substate,target_detected\n")

        t_start = time.time()
        test_duration = 55.0
        records = []
        last_print = 0.0

        pos_x, pos_y = 0.0, 0.0

        while time.time() - t_start < test_duration:
            now = time.time()
            elapsed = now - t_start

            alt_m = 3.8
            yaw_deg = 0.0
            yaw_rate_deg_s = 0.0

            if mav_gcs:
                msg_att = mav_gcs.recv_match(type='ATTITUDE', blocking=False)
                if msg_att:
                    yaw_deg = float(msg_att.yaw) * 57.2958
                    yaw_rate_deg_s = float(msg_att.yawspeed) * 57.2958

                msg_pos = mav_gcs.recv_match(type=['LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT'], blocking=False)
                if msg_pos:
                    if msg_pos.get_type() == 'LOCAL_POSITION_NED':
                        pos_x = float(msg_pos.x)
                        pos_y = float(msg_pos.y)
                        alt_m = -float(msg_pos.z)
                    elif msg_pos.get_type() == 'GLOBAL_POSITION_INT':
                        alt_m = float(msg_pos.relative_alt) / 1000.0

            # Read latest diagnostics line from JSONL
            diag_entry = {}
            if os.path.exists(f'{PROJECT}/logs/tracking_diagnostics.jsonl'):
                try:
                    with open(f'{PROJECT}/logs/tracking_diagnostics.jsonl', 'r') as df:
                        lines = df.readlines()
                        if lines:
                            diag_entry = json.loads(lines[-1])
                except Exception:
                    pass

            substate = diag_entry.get('substate', 'UNKNOWN')
            alt_diag = diag_entry.get('current_alt', alt_m)
            cmd_vx = diag_entry.get('cmd_vx', 0.0)
            cmd_vy = diag_entry.get('cmd_vy', 0.0)
            cmd_yaw_rate = diag_entry.get('cmd_yaw_rate', 0.0) * 57.2958
            err_x = diag_entry.get('error_x', None)
            err_y = diag_entry.get('error_y', None)
            target_detected = err_x is not None and (diag_entry.get('age', 99.0) <= 0.5)

            effective_alt = alt_diag if alt_diag > 1.5 else alt_m

            row = f"{now:.3f},{elapsed:.2f},{pos_x:.2f},{pos_y:.2f},{effective_alt:.3f},{yaw_deg:.1f},{yaw_rate_deg_s:.2f},{cmd_vx:.3f},{cmd_vy:.3f},{cmd_yaw_rate:.2f},{err_x if err_x is not None else ''},{err_y if err_y is not None else ''},{substate},{target_detected}\n"
            csv_file.write(row)
            csv_file.flush()

            rec = {
                'elapsed': elapsed,
                'pos_x': pos_x,
                'pos_y': pos_y,
                'alt': effective_alt,
                'yaw_deg': yaw_deg,
                'yaw_rate': yaw_rate_deg_s,
                'cmd_vx': cmd_vx,
                'cmd_vy': cmd_vy,
                'cmd_yaw_rate': cmd_yaw_rate,
                'err_x': err_x,
                'err_y': err_y,
                'substate': substate,
                'target_detected': target_detected
            }
            records.append(rec)

            if now - last_print >= 1.5:
                last_print = now
                ex_str = f"{err_x:+.1f}" if err_x is not None else "---"
                ey_str = f"{err_y:+.1f}" if err_y is not None else "---"
                print(f"  {elapsed:6.1f}s  | {pos_x:+6.1f}  | {pos_y:+6.1f}  | {effective_alt:6.2f}m  | {yaw_deg:+6.1f}°  | {yaw_rate_deg_s:+6.1f}°/s  | {cmd_vx:+6.2f}  | {ex_str:<7} | {ey_str:<7} | {substate:<18}")

            time.sleep(0.05)

        csv_file.close()
        print("\n" + "=" * 95)
        print(">>> HOÀN THÀNH THỬ NGHIỆM ĐƯỜNG DÀI!")
        print("=" * 95)

        # Statistics
        valid = [r for r in records if r['alt'] >= 2.0]
        if valid:
            min_alt = min(r['alt'] for r in valid)
            max_alt = max(r['alt'] for r in valid)
            mean_alt = sum(r['alt'] for r in valid) / len(valid)
            total_dist = ((pos_x**2 + pos_y**2)**0.5)
            print(f"\n--- TỔNG KẾT HÀNH TRÌNH BÁM ĐƯỜNG DÀI ---")
            print(f"  Tọa độ Drone đạt được : X={pos_x:.1f} m, Y={pos_y:.1f} m (Khoảng cách từ gốc: {total_dist:.1f} m)")
            print(f"  Độ cao trung bình      : {mean_alt:.2f} m (Min: {min_alt:.2f} m, Max: {max_alt:.2f} m)")
            print(f"  Góc quay Heading (Yaw): biến thiên từ {min(r['yaw_deg'] for r in valid):.1f}° đến {max(r['yaw_deg'] for r in valid):.1f}°")
            print(f"  File log CSV chi tiết  : {LOG_CSV}")

        return 0

    except Exception as exc:
        print(f"LỖI: {exc}")
        return 1
    finally:
        for p in procs:
            try:
                p.terminate()
                p.kill()
            except Exception:
                pass
        cleanup()

if __name__ == '__main__':
    sys.exit(main())
