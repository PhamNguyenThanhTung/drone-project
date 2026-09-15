# Real-Flight Readiness & Sim-to-Real Gap Audit

**Status:** Pre-Flight Engineering Audit & Safety Review  
**Current Flight Readiness Verdict:** **NOT FLIGHT READY (0% Real Flight Testing)**  
**Platform Stage:** Simulation & Pure Software-in-the-Loop (SITL) Only

---

## 1. Executive Summary & Sim-to-Real Reality

This project operates entirely inside **Gazebo Harmonic simulation on Ubuntu 22.04 / WSL2**. Zero real-world flight tests, tethered tests, or hardware-in-the-loop (HIL) bench tests have occurred.

Simulation obscures critical real-world failure modes:
1. **Zero Camera Distortion:** Gazebo uses an ideal pinhole camera model without lens barrel distortion, chromatic aberration, or motion blur.
2. **Perfect Network Loopback:** UDP loopback has 0% packet drop and $<0.2\text{ ms}$ latency; real wireless or serial telemetry experiences RF interference, packet loss, and buffer stalls.
3. **Rigid Airframe Physics:** Simulation does not model structural frame resonance, propeller blade wash on the downward camera, or motor thermal throttling.
4. **Dangerous SITL Failsafe Compromise:** The codebase sets `COM_OF_LOSS_T = 5.0 s` to absorb desktop CPU load spikes. In a physical outdoor UAV, waiting 5 seconds before reacting to an Offboard link failure will cause a high-speed flyaway or fatal crash.

---

## 2. Comprehensive Sim-to-Real Readiness Checklist

| Subsystem / Requirement | SITL Reality | Real-Flight Physical Requirement | Current Status |
|---|---|---|---|
| **Camera Hardware & Driver** | Gazebo virtual camera topic | UVC/MIPI-CSI camera (e.g. Sony IMX477 / Global Shutter), calibrated ROS 2 v4l2 driver, fixed exposure. | **MISSING** |
| **Camera Optical Calibration** | Assumed perfect pinhole ($HFOV=2.0$, $fx=133.55$) | Full OpenCV 8-parameter polynomial distortion calibration ($K, D$ matrix); `sensor_msgs/CameraInfo`. | **MISSING** |
| **Companion Computer** | Workstation Intel/AMD CPU + RTX 4060 Laptop GPU | Onboard embedded edge AI computer (e.g. Jetson Orin Nano 8GB / Orin NX); TensorRT model quantization. | **MISSING** |
| **Flight Controller (FCU)** | PX4 SITL process | Physical Pixhawk 6C / Cube Orange running PX4 v1.14+ connected via high-speed UART (`/dev/ttyTHS1`, 921600 baud). | **MISSING** |
| **Hardware Power & Thermal** | Wall outlet power | Dedicated 5V/12V step-down BEC; active heatsink cooling on companion computer to prevent thermal throttling. | **MISSING** |
| **Physical RC Link & Kill Switch**| Simulated teleop keyboard | Dedicated 2.4 GHz / 900 MHz RC transmitter (ELRS/Crossfire) with hardware emergency motor KILL switch assigned to Switch SF. | **CRITICAL MISSING** |
| **Manual Pilot Takeover Priority** | State transitions to `STATE_MANUAL` via HUD keypress | Flight controller firmware-level RC takeover: flipping RC flight mode switch to `POSCTL` or `STABILIZED` instantly overrides Offboard. | **NOT VERIFIED ON HW** |
| **PX4 Offboard Failsafe** | `COM_OF_LOSS_T = 5.0 s` (Relaxed for desktop load) | `COM_OF_LOSS_T <= 1.0 s`, `COM_OBL_ACT = 2` (Auto-Land or Return-to-Launch on stream failure). | **UNSAFE FOR FLIGHT** |
| **Battery Voltage Failsafe** | Virtual battery infinite capacity | Real 4S/6S LiPo power module; calibrated low-voltage RTL ($14.2\text{V}$) and critical autoland ($13.8\text{V}$). | **MISSING** |
| **Downward Rangefinder** | Simulated Gazebo `LW20` model | Physical LiDAR/optical rangefinder (e.g. Benewake TF-Mini / Lightware LW20) for reliable low-altitude tracking. | **NOT BENCH TESTED** |
| **Collision Prevention Sensor** | None | Forward/lateral depth sensor or 360° laser scanner; MAVLink `OBSTACLE_DISTANCE` feed. | **MISSING** |
| **Vibration & EMI Isolation** | Ideal simulation | Anti-vibration silicone dampeners for FCU and camera; magnetic shielding for GPS compass. | **MISSING** |

---

## 3. The 6-Tier Progressive Testing Path

Under no circumstances should the software be flashed directly onto a free-flying quadcopter. The engineering progression must follow six strict validation gates:

```
[ Tier 1: SITL Simulation ]
  - Gazebo Harmonic multi-scenario regression
  - Decoupled tracking retention and safety scoring
  - Latency baseline profiling
            │  (PASS: 100% state machine, 0 collisions)
            ▼
[ Tier 2: Software Replay (Rosbag) ]
  - Replay recorded outdoor flight video bags through YOLO + Tracker
  - Verify detection stability under natural lighting and wind gusts
  - Benchmark HOTA, IDF1, and ID switches on recorded datasets
            │  (PASS: High target retention, 0 false swaps)
            ▼
[ Tier 3: Benchtop Hardware-in-the-Loop (HIL) ]
  - Companion computer (Jetson) connected to physical Pixhawk over UART (921600 baud)
  - Synthetic MAVLink setpoint streaming; verify 10 Hz rate without packet loss
  - Test RC takeover switch: flip to POSCTL; confirm Offboard disconnection in < 50 ms
            │  (PASS: Zero MAVLink drops, instant RC override)
            ▼
[ Tier 4: Tethered Ground / Hover Test ]
  - Drone physically tethered to ground with 2-meter elastic safety harness
  - Test motor arming, auto-takeoff, and stationary hovering
  - Verify companion computer thermal stability under full YOLO inference load (>10 mins)
            │  (PASS: Thermal < 75°C, no vibration spikes)
            ▼
[ Tier 5: Low-Altitude / Low-Speed Field Trial ]
  - Open field (50m x 50m clear grass, zero trees, zero bystanders)
  - Safety pilot fingers resting on RC manual override switch at all times
  - Max speed clamped to $0.6\text{ m/s}$, altitude locked at $2.5\text{ m}$
  - Single cooperative target wearing distinct high-contrast clothing
            │  (PASS: Stable following, smooth braking)
            ▼
[ Tier 6: Operational Envelope Expansion ]
  - Multi-person crossing trials, pedestrian cornering, variable speeds
  - Activation of obstacle avoidance and turn-point recovery
```

---

## 4. Go / No-Go Flight Authorization Criteria

A real flight trial may ONLY receive "GO" authorization when all of the following conditions are formally signed off:

1. **Hardware Kill Switch Confirmed:** Flipping the physical RC kill switch disarms motors in $< 20\text{ ms}$.
2. **Firmware Offboard Failsafe Configured:** `COM_OF_LOSS_T` set to $\le 1.0\text{ s}$; vehicle automatically switches to `ALTCTL` or `LAND` if ROS freezes.
3. **No Wrong-Target Hijack:** Target identity lock architecture (P0 fix) implemented and verified in software replay.
4. **Camera Calibration Loaded:** Real intrinsic parameters loaded via ROS `camera_info`.
5. **Clearance Envelope:** Test area designated with a minimum $15\text{ m}$ safety perimeter free of non-participating personnel.
