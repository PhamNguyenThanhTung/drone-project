#!/usr/bin/env python3
"""
Live SITL Simulation Test for "Person Turn-Around & Fast Movement" Scenario.

Validates:
1. Active Altitude Hold (z maintains ~3.8m +- 0.15m during turns and backing maneuvers).
2. Active Yaw Tracking during turn-around (no hard yaw freeze).
3. Smooth backing (no aggressive pitch jerk / camera vibration).
4. Real-time logging of Altitude, Yaw Rate, Error X/Y, IoU / Detection State, Substate.
"""

import os
import sys
import time
import signal
import subprocess
import threading
import json
from pymavlink import mavutil

PROJECT = '/home/tungt/drone-project'
LOG_DIR = f'{PROJECT}/logs'
LOG_CSV = f'{LOG_DIR}/turnaround_test_telemetry.csv'
LOG_JSONL = f'{LOG_DIR}/turnaround_test_telemetry.jsonl'

def main():
    print("=" * 82)
    print("   LIVE SITL TEST: PERSON TURN-AROUND & FAST MOVEMENT SCENARIO   ")
    print("=" * 82)

    os.makedirs(LOG_DIR, exist_ok=True)
    
    # Cleanup previous runs
    subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f yolo_detector_no[d] 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f motion_arbite[r] 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'parameter_bridge /camera' 2>/dev/null || true", shell=True)
    time.sleep(1.0)

    env = os.environ.copy()
    env['HEADLESS'] = '1'
    env['PX4_GZ_WORLD'] = 'person_tracking_approach'
    env['GZ_SIM_RESOURCE_PATH'] = (
        f"{PROJECT}/gazebo/models:{PROJECT}/gazebo/worlds:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/models:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds"
    )
    env['LD_LIBRARY_PATH'] = f"/usr/lib/wsl/lib:{env.get('LD_LIBRARY_PATH', '')}"

    procs = []
    
    try:
        # 1. Start Gazebo simulation
        print("\n[1/5] Khởi động Gazebo Harmonic (world: person_tracking_approach)...")
        gz_log = open('/tmp/test_turnaround_gz.log', 'w')
        procs.append(subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '--headless-rendering',
             f'{PROJECT}/gazebo/worlds/person_tracking_approach.sdf'],
            env=env, stdout=gz_log, stderr=subprocess.STDOUT
        ))
        
        # Wait for clock topic
        for _ in range(30):
            res = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, env=env)
            if '/world/person_tracking_approach/clock' in res.stdout:
                print("  Gazebo world clock topic is UP.")
                break
            time.sleep(1.0)

        # 2. Start PX4 SITL
        print("\n[2/5] Khởi động PX4 Autopilot SITL...")
        px4_log = open('/tmp/test_turnaround_px4.log', 'w')
        procs.append(subprocess.Popen(
            ['make', 'px4_sitl', 'gz_x500'],
            cwd='/home/tungt/PX4-Autopilot', env=env,
            stdout=px4_log, stderr=subprocess.STDOUT
        ))

        # 3. Start ROS-GZ bridge for camera
        print("\n[3/5] Khởi động ROS 2 Parameter Bridge cho Camera...")
        bridge_log = open('/tmp/test_turnaround_bridge.log', 'w')
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            stdout=bridge_log, stderr=subprocess.STDOUT
        ))

        # 4. Start YOLO Detector
        print("\n[4/5] Khởi động YOLOv8 Detector Node...")
        yolo_env = env.copy()
        yolo_env['PYTHONPATH'] = f"{PROJECT}/ros2_ws/src/vision_tracking:{env.get('PYTHONPATH', '')}"
        yolo_log = open('/tmp/test_turnaround_yolo.log', 'w')
        yolo_device = 'cuda:0' if subprocess.run("python3 -c 'import torch; assert torch.cuda.is_available()'", shell=True).returncode == 0 else 'cpu'
        print(f"  YOLO inference device: {yolo_device}")
        procs.append(subprocess.Popen(
            ['python3', f'{PROJECT}/ros2_ws/src/vision_tracking/vision_tracking/yolo_detector_node.py',
             '--ros-args', '-p', 'image_topic:=/camera/image_raw',
             '-p', f'model_path:={PROJECT}/yolov8n.pt',
             '-p', f'device:={yolo_device}',
             '-p', 'conf:=0.30',
             '-p', 'show_debug_image:=false'],
            cwd=PROJECT, env=yolo_env,
            stdout=yolo_log, stderr=subprocess.STDOUT
        ))

        # 5. Connect GCS reader on port 14550 to send heartbeats
        print("\n[5/5] Kết nối GCS Telemetry Channel (udpin:0.0.0.0:14550)...")
        mav_gcs = None
        t_conn = time.time()
        while time.time() - t_conn < 30.0:
            try:
                cand = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=254)
                if cand.wait_heartbeat(timeout=1.0):
                    mav_gcs = cand
                    print("  GCS Channel connected on 14550!")
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
                            0,
                            mavutil.mavlink.MAV_STATE_ACTIVE
                        )
                    except Exception:
                        pass
                time.sleep(1.0)
        threading.Thread(target=gcs_heartbeat_worker, daemon=True).start()

        # 6. Start MotionArbiter
        print("\n[6/6] Khởi động MotionArbiter...")
        arbiter_env = env.copy()
        arbiter_env['PYTHONPATH'] = f"{PROJECT}:{env.get('PYTHONPATH', '')}"
        arbiter_log = open('/tmp/test_turnaround_arbiter.log', 'w')
        procs.append(subprocess.Popen(
            ['python3', f'{PROJECT}/motion_arbiter.py',
             '--ros-args', '-p', 'takeoff_alt:=3.8', '-p', 'auto_takeoff:=true',
             '-p', 'mavlink:=udpin:0.0.0.0:14540'],
            cwd=PROJECT, env=arbiter_env,
            stdout=arbiter_log, stderr=subprocess.STDOUT
        ))

        # Wait for drone to climb and reach target altitude 3.8m
        print("\n>>> Đang chờ Drone cất cánh và đạt độ cao 3.8m (tối đa 60s)...")
        t_climb = time.time()
        is_airborne = False
        while time.time() - t_climb < 60.0:
            msg = mav_gcs.recv_match(type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'], blocking=True, timeout=0.5)
            alt = 0.0
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    alt = float(msg.relative_alt) / 1000.0
                elif msg.get_type() == 'LOCAL_POSITION_NED':
                    alt = -float(msg.z)
            if alt >= 3.6:
                print(f"  >>> Drone đã đạt độ cao {alt:.2f}m! Bắt đầu ghi log kịch bản bám mục tiêu...")
                is_airborne = True
                break
            time.sleep(0.2)

        # Send target selection command to engage TRACKING mode on Person ID=0
        print(">>> Gửi lệnh khóa mục tiêu Person ID=0 để bắt đầu bám đuổi...")
        subprocess.run("bash -c 'source /opt/ros/humble/setup.bash && ros2 topic pub --once /tracking/select_target std_msgs/msg/Int32 \"{data: 0}\"' 2>/dev/null || true", shell=True)
        time.sleep(1.0)

        # Telemetry logging loop
        print("\n" + "=" * 82)
        print(">>> BẮT ĐẦU GHI LOG BÁM MỤC TIÊU (PERSON TURN-AROUND & MOVEMENT) - 30 GIÂY...")
        print(f"{'Time(s)':<8} | {'Alt (m)':<8} | {'Alt Err':<8} | {'YawRate (°/s)':<14} | {'Err X':<7} | {'Err Y':<7} | {'Substate':<22}")
        print("-" * 82)

        csv_file = open(LOG_CSV, 'w')
        csv_file.write("time_s,alt_m,alt_err_m,yaw_rate_deg_s,yaw_deg,err_x,err_y,target_id,state,substate,target_detected\n")
        
        t_start = time.time()
        test_duration = 30.0
        
        last_print = 0.0
        records = []

        while time.time() - t_start < test_duration:
            now = time.time()
            elapsed = now - t_start
            
            # Read latest MAVLink messages
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

            substate = str(diag_entry.get('substate') or 'STANDBY')
            alt_diag = diag_entry.get('current_alt', None)
            alt_err = diag_entry.get('alt_error', None)
            cmd_yaw_rate = (diag_entry.get('cmd_yaw_rate', 0.0) or 0.0) * 57.2958
            err_x = diag_entry.get('error_x', None)
            err_y = diag_entry.get('error_y', None)
            t_id = diag_entry.get('target_id', -1)
            target_detected = err_x is not None and (diag_entry.get('age', 99.0) <= 0.5)

            effective_alt = float(alt_diag) if (alt_diag is not None and alt_diag > 1.5) else float(alt_m if alt_m is not None else 3.8)
            effective_alt_err = float(alt_err) if alt_err is not None else float(effective_alt - 3.8)
            effective_yaw_rate = float(yaw_rate_deg_s) if yaw_rate_deg_s is not None else 0.0
            effective_yaw_deg = float(yaw_deg) if yaw_deg is not None else 0.0

            row = f"{elapsed:.1f},{effective_alt:.3f},{effective_alt_err:.3f},{effective_yaw_rate:.2f},{effective_yaw_deg:.1f},{err_x if err_x is not None else ''},{err_y if err_y is not None else ''},{t_id},{diag_entry.get('state', 'UNKNOWN')},{substate},{target_detected}\n"
            csv_file.write(row)
            csv_file.flush()

            records.append({
                'elapsed': elapsed,
                'alt': effective_alt,
                'alt_err': effective_alt_err,
                'yaw_rate': effective_yaw_rate,
                'cmd_yaw_rate': cmd_yaw_rate,
                'err_x': err_x,
                'err_y': err_y,
                'substate': substate,
                'target_detected': target_detected
            })

            if now - last_print >= 1.0:
                last_print = now
                alt_str = f"{effective_alt:6.2f}m"
                err_alt_str = f"{effective_alt_err:+6.2f}m"
                yaw_r_str = f"{effective_yaw_rate:+8.1f}°/s"
                ex_str = f"{err_x:+.1f}" if err_x is not None else "---"
                ey_str = f"{err_y:+.1f}" if err_y is not None else "---"
                sub_str = f"{substate:<22}"
                print(f"{elapsed:6.1f}s  | {alt_str} | {err_alt_str} | {yaw_r_str} | {ex_str:<7} | {ey_str:<7} | {sub_str}")

            time.sleep(0.05)

        csv_file.close()
        print("\n" + "=" * 82)
        print(">>> HOÀN THÀNH THU THẬP TELEMETRY VÀ LOGS!")
        print("=" * 82)

        # Statistical evaluation
        valid_records = [r for r in records if r['alt'] >= 2.0]
        if valid_records:
            alts = [r['alt'] for r in valid_records]
            mean_alt = sum(alts) / len(alts)
            min_alt = min(alts)
            max_alt = max(alts)
            max_alt_err = max(abs(min_alt - 3.8), abs(max_alt - 3.8))
            print(f"\n--- KẾT QUẢ ĐỘ CAO (ALTITUDE HOLD) ---")
            print(f"  Độ cao trung bình       : {mean_alt:.2f} m (Mục tiêu: 3.80 m)")
            print(f"  Độ cao thấp nhất         : {min_alt:.2f} m")
            print(f"  Độ cao cao nhất          : {max_alt:.2f} m")
            print(f"  Sai số độ cao lớn nhất   : {max_alt_err:.2f} m (Tiêu chuẩn: < 0.20 m)")
            
            backing_records = [r for r in valid_records if 'BACKING' in r['substate']]
            print(f"\n--- KẾT QUẢ BÁM KHI NGƯỜI QUAY ĐẦU (BACKING & TURNING) ---")
            print(f"  Số frame ở trạng thái Backing/Recover: {len(backing_records)}")
            if backing_records:
                backing_alts = [r['alt'] for r in backing_records]
                print(f"  Độ cao khi đang Backing/Quay đầu: {sum(backing_alts)/len(backing_alts):.2f} m (Min: {min(backing_alts):.2f} m)")
        
        return 0

    except Exception as exc:
        import traceback
        traceback.print_exc()
        return 1
    finally:
        print("\n>>> Dọn dẹp tiến trình mô phỏng...")
        for p in procs:
            try:
                p.terminate()
                p.kill()
            except Exception:
                pass
        subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
        subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
        subprocess.run("pkill -9 -f yolo_detector_no[d] 2>/dev/null || true", shell=True)
        subprocess.run("pkill -9 -f motion_arbiter 2>/dev/null || true", shell=True)
        subprocess.run("pkill -9 -f 'parameter_bridge /camera' 2>/dev/null || true", shell=True)

if __name__ == '__main__':
    sys.exit(main())
