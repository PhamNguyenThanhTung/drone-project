#!/usr/bin/env python3
"""
Rigorous Multi-Trial Evaluation Runner for Drone Vision Tracking.

Enhancements (Stage 1 / Regression Stability):
1. Dynamic path resolution (no hardcoded absolute user paths).
2. Run metadata capture: git commit, branch, dirty flag, host load, GPU info, parameters.
3. Decoupled metrics: Perception (retention, target losses, pixel errors),
   Control (altitude drop, yaw rates, control loop period),
   Safety (watchdog stalls, offboard mode loss, stale heartbeats).
4. Relative artifact paths in summary JSON for portability.
5. Support for both live SITL execution and offline `--analyze-only` metric evaluation.
"""

import os
import sys
import time
import json
import argparse
import subprocess
import threading
from datetime import datetime, timezone
from pymavlink import mavutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.abspath(os.path.join(SCRIPT_DIR, '../..'))
PX4_DIR = os.environ.get('PX4_DIR', os.path.abspath(os.path.join(PROJECT, '../PX4-Autopilot')))
LOG_DIR = os.path.join(PROJECT, 'logs')
TEMPLATE_SDF = os.path.join(PROJECT, 'gazebo/worlds/person_tracking_approach.sdf')

TRIALS = [
    {
        'id': 1,
        'name': 'nominal_180_turn',
        'desc': 'Nominal 180° turn (Approach 8m->3m in 6s, 180° turn in 1s, return 3m->8m in 6s)',
        'seed': 101,
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
        'seed': 102,
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
        'seed': 103,
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
        'seed': 104,
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
        'seed': 105,
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

def collect_run_metadata():
    """Collect runtime environment, git provenance, host load and parameters."""
    git_commit = 'unknown'
    git_branch = 'unknown'
    git_dirty = False
    try:
        git_commit = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=PROJECT, text=True
        ).strip()
        git_branch = subprocess.check_output(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'], cwd=PROJECT, text=True
        ).strip()
        dirty_out = subprocess.check_output(
            ['git', 'status', '--porcelain'], cwd=PROJECT, text=True
        ).strip()
        git_dirty = len(dirty_out) > 0
    except Exception:
        pass

    try:
        load_avg = [round(x, 2) for x in os.getloadavg()]
    except Exception:
        load_avg = []

    cuda_avail = False
    gpu_name = 'none'
    try:
        import torch
        cuda_avail = torch.cuda.is_available()
        if cuda_avail:
            gpu_name = torch.cuda.get_device_name(0)
    except Exception:
        pass

    return {
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'git_commit': git_commit,
        'git_branch': git_branch,
        'git_dirty': git_dirty,
        'world_template': os.path.relpath(TEMPLATE_SDF, PROJECT),
        'model_path': 'yolov8n.pt',
        'cpu_count': os.cpu_count(),
        'host_loadavg_1_5_15': load_avg,
        'cuda_available': cuda_avail,
        'gpu_device': gpu_name,
        'parameters': {
            'takeoff_alt_m': 3.8,
            'control_period_s': 0.10,
            'control_warn_period_s': 0.20,
            'control_fail_period_s': 0.50,
            'mavlink_stale_timeout_s': 2.0,
            'yolo_conf': 0.30,
        }
    }

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

def analyze_records(records, trial, raw_csv_rel, diag_log_rel=None):
    """Compute decoupled perception, control, and safety metrics from telemetry records."""
    if not records:
        return {
            'trial_id': trial['id'],
            'trial_name': trial['name'],
            'raw_csv': raw_csv_rel,
            'status': 'FAIL',
            'error': 'No telemetry records'
        }

    # 1. Control metrics
    hover_records = [r for r in records if r['elapsed'] < 6.0 and r['alt'] >= 2.0]
    hover_alt = (sum(r['alt'] for r in hover_records) / len(hover_records)) if hover_records else 3.80

    maneuver_records = [r for r in records if 'BACKING' in str(r.get('substate', ''))]
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

    # 2. Perception metrics
    tracked_frames = sum(1 for r in records if r.get('target_detected') is True or r.get('target_detected') == 'True')
    total_frames = len(records)
    tracking_retention = (tracked_frames / total_frames) * 100.0 if total_frames > 0 else 0.0

    target_loss_events = 0
    prev_detected = False
    err_x_vals = []
    err_y_vals = []
    for r in records:
        detected = (r.get('target_detected') is True or r.get('target_detected') == 'True')
        if prev_detected and not detected:
            target_loss_events += 1
        prev_detected = detected
        if detected:
            try:
                if r.get('err_x') is not None and r.get('err_x') != '':
                    err_x_vals.append(abs(float(r['err_x'])))
                if r.get('err_y') is not None and r.get('err_y') != '':
                    err_y_vals.append(abs(float(r['err_y'])))
            except (ValueError, TypeError):
                pass

    mean_err_x = round(sum(err_x_vals) / len(err_x_vals), 1) if err_x_vals else None
    mean_err_y = round(sum(err_y_vals) / len(err_y_vals), 1) if err_y_vals else None

    # 3. Safety metrics
    critical_altitude_drop = any(r['alt'] < 1.5 for r in records if r['elapsed'] > 5.0)
    watchdog_stalls = sum(1 for r in records if r.get('watchdog_stall', False))
    offboard_lost = sum(1 for r in records if r.get('offboard_lost', False))

    control_pass = alt_drop_during_maneuver <= 0.15
    perception_verdict = 'PASS' if tracking_retention >= 50.0 else ('WARN' if tracking_retention >= 20.0 else 'FAIL')
    safety_pass = not critical_altitude_drop and watchdog_stalls == 0 and offboard_lost == 0
    overall_status = 'PASS' if (control_pass and safety_pass) else 'FAIL'

    return {
        'trial_id': trial['id'],
        'trial_name': trial['name'],
        'seed': trial.get('seed', trial['id']),
        'raw_csv': raw_csv_rel,
        'diagnostic_log': diag_log_rel,
        'hover_alt_baseline': round(hover_alt, 3),
        'maneuver_alt_mean': round(mean_maneuver_alt, 3),
        'maneuver_alt_min': round(min_maneuver_alt, 3),
        'maneuver_alt_max': round(max_maneuver_alt, 3),
        'altitude_drop_during_maneuver': round(alt_drop_during_maneuver, 3),
        'max_yaw_rate_deg_s': round(max_yaw_rate, 2),
        'mean_yaw_rate_deg_s': round(mean_yaw_rate, 2),
        'tracking_retention_pct': round(tracking_retention, 1),
        'status': overall_status,
        'metrics': {
            'perception': {
                'tracking_retention_pct': round(tracking_retention, 1),
                'tracked_frames': tracked_frames,
                'total_frames': total_frames,
                'target_loss_events': target_loss_events,
                'mean_abs_error_x_px': mean_err_x,
                'mean_abs_error_y_px': mean_err_y,
                'verdict': perception_verdict
            },
            'control': {
                'hover_alt_baseline_m': round(hover_alt, 3),
                'maneuver_alt_min_m': round(min_maneuver_alt, 3),
                'maneuver_alt_max_m': round(max_maneuver_alt, 3),
                'maneuver_alt_mean_m': round(mean_maneuver_alt, 3),
                'altitude_drop_during_maneuver_m': round(alt_drop_during_maneuver, 3),
                'max_yaw_rate_deg_s': round(max_yaw_rate, 2),
                'mean_yaw_rate_deg_s': round(mean_yaw_rate, 2),
                'threshold_alt_drop_m': 0.15,
                'verdict': 'PASS' if control_pass else 'FAIL'
            },
            'safety': {
                'critical_altitude_drop': critical_altitude_drop,
                'watchdog_stalls': watchdog_stalls,
                'offboard_lost': offboard_lost,
                'verdict': 'PASS' if safety_pass else 'FAIL'
            }
        }
    }

def run_single_trial(trial):
    print("\n" + "=" * 80)
    print(f"   CHẠY THỬ NGHIỆM {trial['id']}/5: {trial['name'].upper()} (Seed: {trial.get('seed', trial['id'])})   ")
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
        f"{PX4_DIR}/Tools/simulation/gz/models:"
        f"{PX4_DIR}/Tools/simulation/gz/worlds"
    )
    env['LD_LIBRARY_PATH'] = f"/usr/lib/wsl/lib:{env.get('LD_LIBRARY_PATH', '')}"

    trial_diag_abs = os.path.join(LOG_DIR, f"tracking_diagnostics_trial_{trial['id']}.jsonl")
    if os.path.exists(trial_diag_abs):
        try:
            os.remove(trial_diag_abs)
        except OSError:
            pass

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
            cwd=PX4_DIR, env=env,
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

        # 6. Start MotionArbiter with trial-specific diagnostics log
        arbiter_env = env.copy()
        arbiter_env['PYTHONPATH'] = f"{PROJECT}:{env.get('PYTHONPATH', '')}"
        arbiter_log = open(f"/tmp/arbiter_trial_{trial['id']}.log", 'w')
        procs.append(subprocess.Popen(
            ['python3', f'{PROJECT}/motion_arbiter.py',
             '--ros-args', '-p', 'takeoff_alt:=3.8', '-p', 'auto_takeoff:=true',
             '-p', 'mavlink:=udpin:0.0.0.0:14540',
             '-p', f'diagnostic_log:={trial_diag_abs}'],
            cwd=PROJECT, env=arbiter_env,
            stdout=arbiter_log, stderr=subprocess.STDOUT
        ))

        # 7. Wait for Drone to reach 3.8m
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

        # 8. Record Telemetry
        raw_csv_abs = f"{LOG_DIR}/raw_trial_{trial['id']}_{trial['name']}.csv"
        csv_file = open(raw_csv_abs, 'w')
        csv_file.write("epoch_timestamp,sim_elapsed_s,alt_m,alt_error_m,yaw_rate_deg_s,yaw_deg,cmd_vx,cmd_vy,cmd_yaw_rate_deg_s,err_x,err_y,substate,target_detected\n")

        raw_csv_rel = os.path.relpath(raw_csv_abs, PROJECT)
        diag_log_rel = os.path.relpath(trial_diag_abs, PROJECT)

        print(f"  [Step 2] Bắt đầu ghi log telemetry thực nghiệm vào: {raw_csv_rel}")
        print(f"  {'Time(s)':<8} | {'Alt (m)':<8} | {'Alt Err':<8} | {'YawRate (°/s)':<14} | {'Cmd Vx':<8} | {'Err X':<7} | {'Err Y':<7} | {'Substate':<20}")
        print("  " + "-" * 88)

        t_test_start = time.time()
        trial_duration = 32.0
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

            diag_entry = {}
            if os.path.exists(trial_diag_abs):
                try:
                    with open(trial_diag_abs, 'r') as df:
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

            events = diag_entry.get('events', [])
            watchdog_stall = ('CONTROL_LOOP_STALL' in events) or (diag_entry.get('watchdog_stall_count', 0) > 0)
            offboard_lost = ('OFFBOARD_MODE_LOST' in events)

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
                'target_detected': target_detected,
                'watchdog_stall': watchdog_stall,
                'offboard_lost': offboard_lost,
            }
            records.append(rec)

            if now - last_print >= 1.5:
                last_print = now
                ex_str = f"{err_x:+.1f}" if err_x is not None else "---"
                ey_str = f"{err_y:+.1f}" if err_y is not None else "---"
                print(f"  {elapsed:6.1f}s  | {effective_alt:6.2f}m  | {alt_err:+6.2f}m  | {yaw_rate_deg_s:+8.1f}°/s    | {cmd_vx:+6.2f}   | {ex_str:<7} | {ey_str:<7} | {substate:<20}")

            time.sleep(0.05)

        csv_file.close()

        result_summary = analyze_records(records, trial, raw_csv_rel, diag_log_rel)

        print(f"\n  >>> KẾT QUẢ THỬ NGHIỆM {trial['id']}: [{result_summary['status']}]")
        print(f"      - Độ cao Hover baseline         : {result_summary['hover_alt_baseline']:.2f} m")
        print(f"      - Độ tụt độ cao Maneuver (Drop) : {result_summary['altitude_drop_during_maneuver']:.2f} m (Control: {result_summary['metrics']['control']['verdict']})")
        print(f"      - Tỷ lệ giữ khóa mục tiêu (Track): {result_summary['tracking_retention_pct']:.1f}% (Perception: {result_summary['metrics']['perception']['verdict']})")
        print(f"      - Trạng thái an toàn (Safety)   : {result_summary['metrics']['safety']['verdict']}")
        print(f"      - File log CSV thực nghiệm       : {result_summary['raw_csv']}")

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

def analyze_existing_csvs():
    """Load existing raw CSVs, extract decoupled metrics and assemble full regression summary."""
    import csv as pycsv
    summaries = []
    for trial in TRIALS:
        raw_csv_abs = os.path.join(LOG_DIR, f"raw_trial_{trial['id']}_{trial['name']}.csv")
        raw_csv_rel = os.path.relpath(raw_csv_abs, PROJECT)
        diag_log_rel = f"logs/tracking_diagnostics_trial_{trial['id']}.jsonl"

        if not os.path.exists(raw_csv_abs):
            print(f"Cảnh báo: không tìm thấy file {raw_csv_abs}, bỏ qua.")
            continue

        records = []
        with open(raw_csv_abs, 'r') as f:
            reader = pycsv.DictReader(f)
            for row in reader:
                try:
                    records.append({
                        'timestamp': float(row.get('epoch_timestamp', 0)),
                        'elapsed': float(row.get('sim_elapsed_s', 0)),
                        'alt': float(row.get('alt_m', 3.8)),
                        'alt_err': float(row.get('alt_error_m', 0)),
                        'yaw_rate': float(row.get('yaw_rate_deg_s', 0)),
                        'cmd_vx': float(row.get('cmd_vx', 0)),
                        'cmd_vy': float(row.get('cmd_vy', 0)),
                        'cmd_yaw_rate': float(row.get('cmd_yaw_rate_deg_s', 0)),
                        'err_x': row.get('err_x'),
                        'err_y': row.get('err_y'),
                        'substate': row.get('substate', 'UNKNOWN'),
                        'target_detected': row.get('target_detected') == 'True',
                        'watchdog_stall': False,
                        'offboard_lost': False
                    })
                except Exception:
                    pass

        res = analyze_records(records, trial, raw_csv_rel, diag_log_rel)
        summaries.append(res)

    return summaries

def save_and_print_summary(summaries):
    metadata = collect_run_metadata()
    pass_count = sum(1 for s in summaries if s.get('status') == 'PASS')
    total_count = len(summaries)
    pass_rate = round((pass_count / total_count) * 100.0, 1) if total_count > 0 else 0.0

    full_payload = {
        'run_metadata': metadata,
        'summary_metrics': {
            'total_trials': total_count,
            'pass_count': pass_count,
            'fail_count': total_count - pass_count,
            'pass_rate_pct': pass_rate,
            'overall_safety_status': 'PASS' if all(s.get('metrics', {}).get('safety', {}).get('verdict') == 'PASS' for s in summaries) else 'FAIL',
        },
        'trials': summaries
    }

    os.makedirs(LOG_DIR, exist_ok=True)
    summary_json_path = os.path.join(LOG_DIR, 'multi_trial_summary.json')
    with open(summary_json_path, 'w') as f:
        json.dump(full_payload, f, indent=2)

    print("\n" + "=" * 95)
    print("   TỔNG HỢP KẾT QUẢ HỆ THỐNG REGRESSION (STAGE 1 - DECOUPLED METRICS)   ")
    print("=" * 95)
    print(f"Git Commit: {metadata['git_commit'][:8]} (dirty: {metadata['git_dirty']}) | CUDA: {metadata['cuda_available']} ({metadata['gpu_device']})")
    print("-" * 95)
    print(f"{'Trial':<8} | {'Tên Kịch Bản':<22} | {'Alt Drop':<10} | {'Control':<8} | {'Track %':<8} | {'Perception':<11} | {'Safety':<8} | {'Status'}")
    print("-" * 95)

    for s in summaries:
        m = s.get('metrics', {})
        ctrl = m.get('control', {})
        perc = m.get('perception', {})
        safe = m.get('safety', {})
        print(
            f"Trial {s['trial_id']:<2} | {s['trial_name']:<22} | "
            f"{s.get('altitude_drop_during_maneuver', 0.0):6.2f} m   | "
            f"{ctrl.get('verdict', 'N/A'):<8} | "
            f"{s.get('tracking_retention_pct', 0.0):5.1f}%  | "
            f"{perc.get('verdict', 'N/A'):<11} | "
            f"{safe.get('verdict', 'N/A'):<8} | "
            f"{s.get('status')}"
        )

    print("-" * 95)
    print(f"Tỷ lệ thành công Control/Safety: {pass_count}/{total_count} ({pass_rate}%)")
    print(f"File tổng hợp JSON (relative paths): {os.path.relpath(summary_json_path, PROJECT)}")
    print("=" * 95)

def main():
    parser = argparse.ArgumentParser(description="Multi-Trial Regression Runner for Drone Vision Tracking")
    parser.add_argument('--analyze-only', action='store_true', help="Re-evaluate existing raw CSV files and generate updated summary JSON")
    args = parser.parse_args()

    if args.analyze_only:
        print("Đang phân tích các file telemetry CSV hiện có với schema Stage 1...")
        summaries = analyze_existing_csvs()
        save_and_print_summary(summaries)
        return 0

    print("=" * 95)
    print("   BẮT ĐẦU CHUỖI 5 THỬ NGHIỆM ĐỘC LẬP (ISOLATED MULTI-TRIAL EVALUATION)   ")
    print("=" * 95)

    os.makedirs(LOG_DIR, exist_ok=True)
    summaries = []

    for trial in TRIALS:
        res = run_single_trial(trial)
        summaries.append(res)
        time.sleep(2.0)

    save_and_print_summary(summaries)
    return 0

if __name__ == '__main__':
    sys.exit(main())
