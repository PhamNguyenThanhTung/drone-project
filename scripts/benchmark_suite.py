#!/usr/bin/env python3
import json
import math
import os
import signal
import subprocess
import sys
import time
import numpy as np

PROJECT_DIR = "/home/tungt/drone-project"
DIAG_LOG = os.path.join(PROJECT_DIR, "logs", "tracking_diagnostics.jsonl")

def clean_system():
    patterns = [
        "tracking_stack.launch",
        "live_camera_hud",
        "motion_arbiter",
        "yolo_detector",
        "sim_realism",
        "parameter_bridge",
        "gz sim",
        "gz-sim",
        "ruby.*person_tracking",
        "px4",
        "QGroundControl"
    ]
    for p in patterns:
        subprocess.run(["pkill", "-9", "-f", p], stderr=subprocess.DEVNULL)
    subprocess.run("rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps*", shell=True, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def run_trial(name: str, duration_sec: int = 45, teleop_at: float = 25.0, teleop_duration: float = 4.0):
    print(f"\n==========================================")
    print(f" STARTING TRIAL: {name}")
    print(f"==========================================")
    clean_system()

    if os.path.exists(DIAG_LOG):
        os.remove(DIAG_LOG)
    stack_log_path = f"/tmp/trial_{name}_stack.log"
    if os.path.exists(stack_log_path):
        os.remove(stack_log_path)

    env = os.environ.copy()
    env["HEADLESS"] = "1"
    env["SIM_REALISM"] = "0"
    env["SHOW_HUD"] = "0"
    env["USE_SOFTWARE_RENDERING"] = "1"

    t_start = time.time()
    with open(stack_log_path, "w") as out_f:
        proc = subprocess.Popen(
            ["./start_stack.sh"],
            cwd=PROJECT_DIR,
            env=env,
            stdout=out_f,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )

    print(f"[{name}] Waiting for stack READY...")
    ready = False
    for _ in range(60):
        if os.path.exists(stack_log_path):
            with open(stack_log_path, "r", errors="ignore") as f:
                content = f.read()
                if "SẴN SÀNG!" in content:
                    ready = True
                    break
        time.sleep(1.0)

    if not ready:
        print(f"[{name}] ERROR: Stack did not reach READY in 60s!")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        clean_system()
        return None

    ready_time = time.time() - t_start
    print(f"[{name}] Stack READY in {ready_time:.2f}s! Running test for {duration_sec}s...")

    teleop_sent = False
    target_selected = False
    run_start = time.time()
    while time.time() - run_start < duration_sec:
        elapsed = time.time() - run_start
        if elapsed >= 3.0 and not target_selected:
            print(f"[{name}] Selecting target 0 at {elapsed:.2f}s...")
            subprocess.run("ros2 topic pub --once /tracking/select_target std_msgs/msg/Int32 '{data: 0}'", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            target_selected = True

        if teleop_at > 0 and elapsed >= teleop_at and not teleop_sent:
            print(f"[{name}] Injecting teleop cmd_vel at {elapsed:.2f}s for {teleop_duration}s...")
            teleop_cmd = "ros2 topic pub --rate 10 /teleop/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}'"
            teleop_p = subprocess.Popen(teleop_cmd, shell=True, preexec_fn=os.setsid)
            time.sleep(teleop_duration)
            try:
                os.killpg(os.getpgid(teleop_p.pid), signal.SIGTERM)
            except Exception:
                pass
            teleop_sent = True
            print(f"[{name}] Teleop injection complete.")
        time.sleep(0.5)

    print(f"[{name}] Trial finished. Shutting down gracefully...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=10)
    except Exception:
        pass

    clean_system()
    print(f"[{name}] Post-trial cleanup done. Analyzing diagnostics...")

    metrics = analyze_diagnostics(DIAG_LOG, stack_log_path, ready_time)
    metrics["name"] = name
    return metrics

def analyze_diagnostics(diag_path, stack_log_path, ready_time):
    records = []
    if os.path.exists(diag_path):
        with open(diag_path, "r") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        records.append(json.loads(line))
                    except Exception:
                        pass

    if not records:
        return {"error": "No diagnostics records found"}

    timestamps = [r["timestamp"] for r in records if "timestamp" in r]
    dt_arr = []
    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i-1]
        dt_arr.append(dt)

    control_period_mean = float(np.mean(dt_arr)) if dt_arr else 0.0
    control_jitter_std = float(np.std(dt_arr)) if dt_arr else 0.0
    max_setpoint_gap = float(np.max(dt_arr)) if dt_arr else 0.0

    vision_ages = [r["age"] for r in records if r.get("age") is not None and r["age"] >= 0]
    mean_vision_age = float(np.mean(vision_ages)) if vision_ages else 0.0
    p95_vision_age = float(np.percentile(vision_ages, 95)) if vision_ages else 0.0
    max_vision_age = float(np.max(vision_ages)) if vision_ages else 0.0

    states = [r.get("state") for r in records]
    tracking_ticks = sum(1 for s in states if s == "TRACKING")
    manual_ticks = sum(1 for s in states if s == "MANUAL")
    standby_ticks = sum(1 for s in states if s == "STANDBY")
    total_ticks = len(states)
    tracking_retention_pct = (tracking_ticks / total_ticks * 100.0) if total_ticks else 0.0

    state_flips = 0
    manual_to_tracking = 0
    tracking_to_manual = 0
    for i in range(1, len(states)):
        if states[i] != states[i-1]:
            state_flips += 1
            if states[i-1] == "MANUAL" and states[i] == "TRACKING":
                manual_to_tracking += 1
            elif states[i-1] == "TRACKING" and states[i] == "MANUAL":
                tracking_to_manual += 1

    tracking_errors = []
    for r in records:
        if r.get("state") == "TRACKING":
            ex = r.get("error_x")
            ey = r.get("error_y")
            if ex is not None and ey is not None:
                tracking_errors.append(math.sqrt(ex**2 + ey**2))
    mean_error = float(np.mean(tracking_errors)) if tracking_errors else 0.0

    offboard_losses = 0
    offboard_actives = [r.get("offboard_active", False) for r in records]
    for i in range(1, len(offboard_actives)):
        if offboard_actives[i-1] is True and offboard_actives[i] is False:
            offboard_losses += 1

    alts = [r.get("current_alt", 0.0) for r in records if r.get("current_alt") is not None]
    min_alt = float(np.min(alts)) if alts else 0.0
    max_alt = float(np.max(alts)) if alts else 0.0
    mean_alt = float(np.mean(alts)) if alts else 0.0

    watchdog_stalls = max([r.get("watchdog_stall_count", 0) for r in records] + [0])

    return {
        "ready_time_s": ready_time,
        "total_ticks": total_ticks,
        "control_period_mean_s": round(control_period_mean, 4),
        "control_jitter_std_s": round(control_jitter_std, 4),
        "max_setpoint_gap_s": round(max_setpoint_gap, 4),
        "mean_vision_age_s": round(mean_vision_age, 3),
        "p95_vision_age_s": round(p95_vision_age, 3),
        "max_vision_age_s": round(max_vision_age, 3),
        "tracking_retention_pct": round(tracking_retention_pct, 1),
        "manual_ticks": manual_ticks,
        "state_flips": state_flips,
        "manual_to_tracking": manual_to_tracking,
        "tracking_to_manual": tracking_to_manual,
        "mean_pixel_error": round(mean_error, 2),
        "offboard_loss_count": offboard_losses,
        "min_alt_m": round(min_alt, 2),
        "max_alt_m": round(max_alt, 2),
        "mean_alt_m": round(mean_alt, 2),
        "watchdog_stall_count": watchdog_stalls,
    }

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "A"
    res = run_trial(mode, duration_sec=45, teleop_at=25.0, teleop_duration=4.0)
    print("\n--- RESULTS JSON ---")
    print(json.dumps(res, indent=2))
