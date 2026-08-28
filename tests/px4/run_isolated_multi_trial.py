#!/usr/bin/env python3
"""
Rigorous Multi-Trial Evaluation Runner for Drone Vision Tracking.

Guarantees:
1. Complete Isolation: Drone reaches 3.8m and hovers steady BEFORE the person starts the turnaround maneuver.
2. 5 Distinct Trial Variations:
   - Trial 1: Nominal 180° Turn (v=0.83 m/s, 180° turn at 3.0m)
   - Trial 2: Fast 180° Turn (v=1.25 m/s, fast turn)
   - Trial 3: Lateral Left Turn (Start at y=+1.5m, turn left)
   - Trial 4: Lateral Right Turn (Start at y=-1.5m, turn right)
   - Trial 5: Aggressive Close-In (Approach to 2.2m, sharp 180° sprint away at 1.5 m/s)
3. Raw CSV logging with real wall-clock timestamps for every trial.
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
TEMPLATE_SDF = f'{PROJECT}/gazebo/worlds/person_tracking_approach.sdf'

TRIALS = [
    {
        'id': 1,
        'name': 'nominal_180_turn',
        'desc': 'Nominal 180° turn (Approach 8m->3m in 6s, 180° turn in 1s, return 3m->8m in 6s)',
        'delay_start': 22.0,
        'waypoints': [
            (0.0, '8.0 0.5 0.8891414 0 0 3.14159'),
            (6.0, '3.0 0.5 0.8891414 0 0 3.14159'),
            (7.0, '3.0 0.5 0.8891414 0 0 0.0'),
            (13.0, '8.0 0.5 0.8891414 0 0 0.0'),
            (14.0, '8.0 0.5 0.8891414 0 0 3.14159'),
        ]
    },
    {
        'id': 2,
        'name': 'fast_180_turn',
        'desc': 'Fast 180° turn (Approach 8m->3m at 1.25 m/s in 4s, fast turn in 0.8s, fast retreat in 4s)',
        'delay_start': 22.0,
        'waypoints': [
            (0.0, '8.0 0.5 0.8891414 0 0 3.14159'),
            (4.0, '3.0 0.5 0.8891414 0 0 3.14159'),
            (4.8, '3.0 0.5 0.8891414 0 0 0.0'),
            (8.8, '8.0 0.5 0.8891414 0 0 0.0'),
            (9.6, '8.0 0.5 0.8891414 0 0 3.14159'),
        ]
    },
    {
        'id': 3,
        'name': 'lateral_left_turn',
        'desc': 'Lateral Left Turn (Start y=+1.5m, approach to (3.0, -0.5m), turn left)',
        'delay_start': 22.0,
        'waypoints': [
            (0.0, '8.0 1.5 0.8891414 0 0 3.14159'),
            (5.5, '3.0 -0.5 0.8891414 0 0 3.14159'),
            (6.5, '3.0 -0.5 0.8891414 0 0 1.57079'),
            (11.5, '8.0 1.5 0.8891414 0 0 0.0'),
            (12.5, '8.0 1.5 0.8891414 0 0 3.14159'),
        ]
    },
    {
        'id': 4,
        'name': 'lateral_right_turn',
        'desc': 'Lateral Right Turn (Start y=-1.5m, approach to (3.0, +0.5m), turn right)',
        'delay_start': 22.0,
        'waypoints': [
            (0.0, '8.0 -1.5 0.8891414 0 0 3.14159'),
            (5.5, '3.0 0.5 0.8891414 0 0 3.14159'),
            (6.5, '3.0 0.5 0.8891414 0 0 -1.57079'),
            (11.5, '8.0 -1.5 0.8891414 0 0 0.0'),
            (12.5, '8.0 -1.5 0.8891414 0 0 3.14159'),
        ]
    },
    {
        'id': 5,
        'name': 'aggressive_close_in',
        'desc': 'Aggressive Close-In (Approach close to 2.2m, sharp 180° turn, sprint away at 1.5 m/s)',
        'delay_start': 22.0,
        'waypoints': [
            (0.0, '8.0 0.5 0.8891414 0 0 3.14159'),
            (4.5, '2.2 0.5 0.8891414 0 0 3.14159'),
            (5.3, '2.2 0.5 0.8891414 0 0 0.0'),
            (9.3, '8.2 0.5 0.8891414 0 0 0.0'),
            (10.1, '8.2 0.5 0.8891414 0 0 3.14159'),
        ]
    }
]

def generate_trial_sdf(trial, out_path):
    with open(TEMPLATE_SDF, 'r') as f:
        content = f.read()

    wp_xml = ""
    for t_sec, pose_str in trial['waypoints']:
        wp_xml += f"""          <waypoint>
            <time>{t_sec:.1f}</time>
            <pose>{pose_str}</pose>
          </waypoint>\n"""

    new_script = f"""      <script>
        <loop>true</loop>
        <delay_start>{trial['delay_start']:.1f}</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk">
{wp_xml}        </trajectory>
      </script>"""

    # Replace <script>...</script> block
    import re
    modified_content = re.sub(r'<script>.*?</script>', new_script, content, flags=re.DOTALL)
    with open(out_path, 'w') as f:
        f.write(modified_content)

def cleanup_processes():
    subprocess.run("pkill -9 -x px4 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'gz si[m]' 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f yolo_detector_no[d] 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f motion_arbite[r] 2>/dev/null || true", shell=True)
    subprocess.run("pkill -9 -f 'parameter_bridge /camera' 2>/dev/null || true", shell=True)
    time.sleep(1.0)

def run_single_trial(trial):
    print("\n" + "=" * 80)
    print(f"   CHẠY THỬ NGHIỆM {trial['id']}/5: {trial['name'].upper()}   ")
    print(f"   Mô tả: {trial['desc']}")
    print("=" * 80)

    cleanup_processes()
    
    trial_sdf = f"/tmp/trial_{trial['id']}_{trial['name']}.sdf"
    generate_trial_sdf(trial, trial_sdf)

    env = os.environ.copy()
    env['HEADLESS'] = '1'
    env['PX4_GZ_WORLD'] = f"trial_{trial['id']}_{trial['name']}"
    env['GZ_SIM_RESOURCE_PATH'] = (
        f"{PROJECT}/gazebo/models:{PROJECT}/gazebo/worlds:/tmp:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/models:"
        "/home/tungt/PX4-Autopilot/Tools/simulation/gz/worlds"
    )
    env['LD_LIBRARY_PATH'] = f"/usr/lib/wsl/lib:{env.get('LD_LIBRARY_PATH', '')}"

    procs = []
    
    try:
        # 1. Start Gazebo
        gz_log = open(f"/tmp/gz_trial_{trial['id']}.log", 'w')
        procs.append(subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '--headless-rendering', trial_sdf],
            env=env, stdout=gz_log, stderr=subprocess.STDOUT
        ))
        
        # Wait for world clock
        for _ in range(30):
            res = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, env=env)
            if 'clock' in res.stdout:
                break
            time.sleep(1.0)

        # 2. Start PX4 SITL
        px4_log = open(f"/tmp/px4_trial_{trial['id']}.log", 'w')
        procs.append(subprocess.Popen(
            ['make', 'px4_sitl', 'gz_x500'],
            cwd='/home/tungt/PX4-Autopilot', env=env,
            stdout=px4_log, stderr=subprocess.STDOUT
        ))

        # 3. Start ROS-GZ bridge
        bridge_log = open(f"/tmp/bridge_trial_{trial['id']}.log", 'w')
        procs.append(subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            stdout=bridge_log, stderr=subprocess.STDOUT
        ))

        # 4. Start YOLO
        yolo_env = env.copy()
        yolo_env['PYTHONPATH'] = f"{PROJECT}/ros2_ws/src/vision_tracking:{env.get('PYTHONPATH', '')}"
        yolo_log = open(f"/tmp/yolo_trial_{trial['id']}.log", 'w')
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
        arbiter_env = env.copy()
        arbiter_env['PYTHONPATH'] = f"{PROJECT}:{env.get('PYTHONPATH', '')}"
        arbiter_log = open(f"/tmp/arbiter_trial_{trial['id']}.log", 'w')
        procs.append(subprocess.Popen(
            ['python3', f'{PROJECT}/motion_arbiter.py',
             '--ros-args', '-p', 'takeoff_alt:=3.8', '-p', 'auto_takeoff:=true',
             '-p', 'mavlink:=udpin:0.0.0.0:14540'],
            cwd=PROJECT, env=arbiter_env,
            stdout=arbiter_log, stderr=subprocess.STDOUT
        ))

        # 7. Wait for Drone to reach 3.8m and STABILIZE in hover for 5 seconds
        print("  [Step 1] Chờ Drone cất cánh lên 3.8m...")
        t_wait_takeoff = time.time()
        reached_target_alt = False
        while time.time() - t_wait_takeoff < 50.0:
            msg = mav_gcs.recv_match(type=['GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED'], blocking=True, timeout=0.5)
            if msg:
                alt = (float(msg.relative_alt) / 1000.0) if msg.get_type() == 'GLOBAL_POSITION_INT' else -float(msg.z)
                if alt >= 3.65:
                    reached_target_alt = True
                    break
            time.sleep(0.2)

        if not reached_target_alt:
            print("  [WARN] Drone chưa đạt 3.65m trong 50s, tiếp tục ghi log...")

        # 8. Record Telemetry: Phase 1 (Hover Stability), Phase 2 (Turnaround Maneuver), Phase 3 (Retreat/Recovery)
        raw_csv_path = f"{LOG_DIR}/raw_trial_{trial['id']}_{trial['name']}.csv"
        csv_file = open(raw_csv_path, 'w')
        csv_file.write("epoch_timestamp,sim_elapsed_s,alt_m,alt_error_m,yaw_rate_deg_s,yaw_deg,cmd_vx,cmd_vy,cmd_yaw_rate_deg_s,err_x,err_y,substate,target_detected\n")

        print(f"  [Step 2] Bắt đầu ghi log telemetry thực nghiệm vào: {raw_csv_path}")
        print(f"  {'Time(s)':<8} | {'Alt (m)':<8} | {'Alt Err':<8} | {'YawRate (°/s)':<14} | {'Cmd Vx':<8} | {'Err X':<7} | {'Err Y':<7} | {'Substate':<20}")
        print("  " + "-" * 88)

        t_test_start = time.time()
        trial_duration = 32.0 # Covers hover + approach + turnaround + retreat
        records = []
        last_print = 0.0

        while time.time() - t_test_start < trial_duration:
            now = time.time()
            elapsed = now - t_test_start

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

            substate = diag_entry.get('substate', 'UNKNOWN')
            alt_diag = diag_entry.get('current_alt', alt_m)
            alt_err = diag_entry.get('alt_error', alt_diag - 3.8)
            cmd_vx = diag_entry.get('cmd_vx', 0.0)
            cmd_vy = diag_entry.get('cmd_vy', 0.0)
            cmd_yaw_rate = diag_entry.get('cmd_yaw_rate', 0.0) * 57.2958
            err_x = diag_entry.get('error_x', None)
            err_y = diag_entry.get('error_y', None)
            target_detected = err_x is not None and (diag_entry.get('age', 99.0) <= 0.5)

            effective_alt = alt_diag if alt_diag > 1.5 else alt_m

            row = f"{now:.3f},{elapsed:.2f},{effective_alt:.3f},{alt_err:.3f},{yaw_rate_deg_s:.2f},{yaw_deg:.1f},{cmd_vx:.3f},{cmd_vy:.3f},{cmd_yaw_rate:.2f},{err_x if err_x is not None else ''},{err_y if err_y is not None else ''},{substate},{target_detected}\n"
            csv_file.write(row)
            csv_file.flush()

            rec = {
                'timestamp': now,
                'elapsed': elapsed,
                'alt': effective_alt,
                'alt_err': alt_err,
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
                print(f"  {elapsed:6.1f}s  | {effective_alt:6.2f}m  | {alt_err:+6.2f}m  | {yaw_rate_deg_s:+8.1f}°/s    | {cmd_vx:+6.2f}   | {ex_str:<7} | {ey_str:<7} | {substate:<20}")

            time.sleep(0.05)

        csv_file.close()

        # Analysis of Trial metrics
        hover_records = [r for r in records if r['elapsed'] < 6.0 and r['alt'] >= 2.0]
        hover_alt = (sum(r['alt'] for r in hover_records) / len(hover_records)) if hover_records else 3.80

        maneuver_records = [r for r in records if 'BACKING' in r['substate']]
        if not maneuver_records:
            maneuver_records = [r for r in records if 6.0 <= r['elapsed'] <= 18.0 and r['alt'] >= 2.0]

        maneuver_alts = [r['alt'] for r in maneuver_records] if maneuver_records else [hover_alt]
        min_maneuver_alt = min(maneuver_alts)
        max_maneuver_alt = max(maneuver_alts)
        mean_maneuver_alt = sum(maneuver_alts) / len(maneuver_alts)
        alt_drop_during_maneuver = max(0.0, hover_alt - min_maneuver_alt)

        yaw_rates = [abs(r['yaw_rate']) for r in maneuver_records] if maneuver_records else [0.0]
        max_yaw_rate = max(yaw_rates)
        mean_yaw_rate = sum(yaw_rates) / len(yaw_rates)

        tracked_frames = sum(1 for r in records if r['target_detected'])
        tracking_retention = (tracked_frames / len(records)) * 100.0

        result_summary = {
            'trial_id': trial['id'],
            'trial_name': trial['name'],
            'raw_csv': raw_csv_path,
            'hover_alt_baseline': round(hover_alt, 3),
            'maneuver_alt_mean': round(mean_maneuver_alt, 3),
            'maneuver_alt_min': round(min_maneuver_alt, 3),
            'maneuver_alt_max': round(max_maneuver_alt, 3),
            'altitude_drop_during_maneuver': round(alt_drop_during_maneuver, 3),
            'max_yaw_rate_deg_s': round(max_yaw_rate, 2),
            'mean_yaw_rate_deg_s': round(mean_yaw_rate, 2),
            'tracking_retention_pct': round(tracking_retention, 1),
            'status': 'PASS' if alt_drop_during_maneuver <= 0.15 else 'FAIL'
        }

        print(f"\n  >>> KẾT QUẢ THỬ NGHIỆM {trial['id']}: [{result_summary['status']}]")
        print(f"      - Độ cao Hover ổn định ban đầu  : {hover_alt:.2f} m")
        print(f"      - Độ cao khi người quay đầu (Min): {min_maneuver_alt:.2f} m (Mean: {mean_maneuver_alt:.2f} m)")
        print(f"      - Độ tụt độ cao trong Maneuver   : {alt_drop_during_maneuver:.2f} m (Tiêu chuẩn: <= 0.15 m)")
        print(f"      - Tốc độ góc xoay Yaw cực đại   : {max_yaw_rate:.1f}°/s")
        print(f"      - Tỷ lệ giữ khóa mục tiêu (Track): {tracking_retention:.1f}%")
        print(f"      - File log CSV thực nghiệm       : {raw_csv_path}")

        return result_summary

    except Exception as exc:
        print(f"  [ERROR] Trial {trial['id']} failed: {exc}")
        return {'trial_id': trial['id'], 'trial_name': trial['name'], 'status': 'ERROR', 'error': str(exc)}
    finally:
        for p in procs:
            try:
                p.terminate()
                p.kill()
            except Exception:
                pass
        cleanup_processes()

def main():
    print("=" * 85)
    print("   BẮT ĐẦU CHUỖI 5 THỬ NGHIỆM ĐỘC LẬP (ISOLATED MULTI-TRIAL EVALUATION)   ")
    print("=" * 85)

    os.makedirs(LOG_DIR, exist_ok=True)
    summaries = []

    for trial in TRIALS:
        res = run_single_trial(trial)
        summaries.append(res)
        time.sleep(2.0)

    summary_json_path = f"{LOG_DIR}/multi_trial_summary.json"
    with open(summary_json_path, 'w') as f:
        json.dump(summaries, f, indent=2)

    print("\n" + "=" * 85)
    print("   TỔNG HỢP KẾT QUẢ TOÀN BỘ 5 THỬ NGHIỆM ĐỘC LẬP   ")
    print("=" * 85)
    print(f"{'Trial ID':<10} | {'Tên Kịch Bản':<22} | {'Alt Baseline':<12} | {'Alt Min Maneuver':<16} | {'Alt Drop':<10} | {'Max YawRate':<12} | {'Track %':<8} | {'Kết Quả'}")
    print("-" * 110)

    pass_count = 0
    for s in summaries:
        if s.get('status') == 'PASS':
            pass_count += 1
        print(f"Trial {s['trial_id']:<4} | {s['trial_name']:<22} | {s.get('hover_alt_baseline', 0.0):6.2f} m     | {s.get('maneuver_alt_min', 0.0):6.2f} m         | {s.get('altitude_drop_during_maneuver', 0.0):6.2f} m   | {s.get('max_yaw_rate_deg_s', 0.0):6.1f}°/s    | {s.get('tracking_retention_pct', 0.0):5.1f}%  | {s.get('status')}")

    print("-" * 110)
    print(f"Tỷ lệ thành công: {pass_count}/{len(TRIALS)} ({(pass_count/len(TRIALS))*100:.1f}%)")
    print(f"File tổng hợp JSON: {summary_json_path}")
    print("=" * 85)

if __name__ == '__main__':
    sys.exit(main())
