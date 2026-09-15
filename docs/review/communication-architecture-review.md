# Communication Architecture & End-to-End Latency Review

**Status:** Technical Architecture Audit & Latency Measurement Design  
**Current System:** Gazebo Harmonic + PX4 SITL + ROS 2 Humble in Ubuntu 22.04 / WSL2  
**Physical Hardware Deployment:** **UNKNOWN / NOT IMPLEMENTED**

---

## 1. Trace of the Current Communication Pipeline

Tracing the exact data path through `start_stack.sh`, `tracking_stack.launch.py`, and the active nodes:

```
[Gazebo Harmonic Simulation Engine]
   │
   │ Gazebo Transport (Shared memory / loopback protobuf: /camera/image_raw)
   ▼
[ros_gz_bridge parameter_bridge]
   │
   │ ROS 2 Inter-Process Topic (/camera/image_raw, sensor_msgs/msg/Image)
   ▼
[yolo_detector_node (Python/C++ in ROS 2)]
   │ 1. PyTorch / CUDA Inference (YOLOv8n @ imgsz=640)
   │ 2. ByteTrack Association (Kalman update on detections)
   │ 3. Pixel error calculation
   │
   │ ROS 2 Topic (/tracking/error, geometry_msgs/msg/Point)
   ▼
[motion_arbiter_node (Python in ROS 2)]
   │ 1. State machine evaluation (MANUAL / TRACKING / STANDBY)
   │ 2. Visual servoing control law & slew limiters
   │ 3. Pinhole 3D projection & turn-point vector
   │ 4. Coordinate frame conversion (Body to LOCAL_NED)
   │
   │ MAVLink UDP Socket (UDP Loopback to 127.0.0.1:14540)
   ▼
[PX4 Autopilot SITL]
   │ 1. MAVLink receiver thread in PX4 posix-configs
   │ 2. Commander mode transition to OFFBOARD
   │ 3. Position & Rate Controller (EKF2 local navigation fusion)
   │ 4. Actuator motor mixer output
   │
   │ Gazebo-PX4 Bridge (Multicopter motor speed commands)
   ▼
[Gazebo Quadcopter Model (gz_x500_flow)]
```

### 1.1 Architectural Classification: Simulation Loopback
- **Camera Source:** Simulated sensor link in Gazebo SDF model.
- **Video Transport:** `gz-transport` bridged to ROS 2 Humble `sensor_msgs/Image`.
- **Processing Location:** Single host workstation (Ubuntu inside WSL2; local NVIDIA GeForce RTX 4060 GPU).
- **ROS 2 Transport:** Local intra-host shared memory / DDS loopback (FastDDS / CycloneDDS).
- **MAVLink Transport:** Local UDP loopback (`udpin:0.0.0.0:14540`).
- **Control Endpoint:** Local PX4 SITL process.

### 1.2 Physical Hardware Topology: **UNKNOWN**
The repository contains **zero configuration files or network topologies for physical aircraft**.
- It is UNKNOWN whether real deployment intends to use an onboard companion computer (e.g., Nvidia Jetson Orin Nano / Raspberry Pi 5) or an offboard ground station server with a wireless RTSP/Wi-Fi video stream.
- **Critical Architectural Warning:** If offboard server processing over Wi-Fi is attempted, wireless latency spikes (100–300 ms jitter and packet loss) will severely compromise PX4 Offboard mode, which requires consistent setpoint streaming $\ge 2\text{ Hz}$ (default failsafe timeout $1.0\text{ s}$).

---

## 2. End-to-End Latency Measurement Framework

### 2.1 Current Latency Gap & Blind Spots
Currently, `scripts/command_latency_micro_test.py` only measures:
$$\text{Latency}_{teleop} = T_{\text{arbiter\_receive}} - T_{\text{hud\_publish}}$$
This completely misses the **perception pipeline**, which accounts for $>80\%$ of total end-to-end latency:
- Sensor exposure and image transport: **Unmeasured**.
- YOLOv8 forward pass: Logged internally, but dropped from output message.
- Tracker association: **Unmeasured**.
- Camera-to-PX4 total delay: **Unmeasured**.

### 2.2 Complete Measurement Design
To capture true glass-to-propeller latency, a timestamp chain must be embedded across the pipeline:

```
Frame Capture ($T_0$)
       │
       ▼
ROS Bridge Publish ($T_1$)
       │  [Δt_bridge = T_1 - T_0]
       ▼
YOLO Ingest ($T_2$)
       │  [Δt_ros_transport = T_2 - T_1]
       ▼
YOLO + ByteTrack Complete ($T_3$)
       │  [Δt_inference = T_3 - T_2]
       ▼
/tracking/error Ingest in Arbiter ($T_4$)
       │  [Δt_inter_node = T_4 - T_3]
       ▼
Arbiter Control Law Complete ($T_5$)
       │  [Δt_control = T_5 - T_4]
       ▼
MAVLink Packet Sent ($T_6$)
       │  [Δt_mavlink_send = T_6 - T_5]
       ▼
PX4 EKF2 / Controller Receive ($T_7$)
          [Δt_mavlink_net = T_7 - T_6]
```

### 2.3 Required Metrics to Record

| Latency Segment | Target (SITL) | Target (Physical Onboard Jetson) | Failure Threshold | Impact of Excess Latency |
|---|---|---|---|---|
| **Camera $\rightarrow$ YOLO Ingest** ($T_2 - T_0$) | $< 15\text{ ms}$ | $< 25\text{ ms}$ | $> 50\text{ ms}$ | Stale frames; motion blur mismatch. |
| **Inference + Tracking** ($T_3 - T_2$) | $< 18\text{ ms}$ | $< 35\text{ ms}$ | $> 60\text{ ms}$ | Target moves outside predicted bounding box. |
| **Inter-node + Arbiter** ($T_5 - T_3$) | $< 10\text{ ms}$ | $< 15\text{ ms}$ | $> 30\text{ ms}$ | Sets offboard loop jitter. |
| **MAVLink Transport** ($T_7 - T_5$) | $< 2\text{ ms}$ | $< 5\text{ ms}$ (UART/USB) | $> 20\text{ ms}$ | Offboard stream stalls; triggers PX4 failsafe. |
| **Total End-to-End ($T_7 - T_0$)** | **$< 45\text{ ms}$** | **$< 80\text{ ms}$** | **$> 120\text{ ms}$** | Phase lag induces aggressive pitch/roll oscillations. |

---

## 3. Impact Analysis on Critical Flight Regimes

### 3.1 Fast Walking / Running Target ($V > 1.5\text{ m/s}$)
- At $V = 1.5\text{ m/s}$, a $100\text{ ms}$ end-to-end latency causes the drone to act on a target position that is $15\text{ cm}$ in the past.
- With ByteTrack's 2D Kalman filter, if camera-to-decision latency jitter is high ($\pm 40\text{ ms}$), the estimated target velocity oscillates, generating erratic forward/backward thrust spikes.

### 3.2 Sharp 90° / 180° Cornering
- When the human makes a sharp 90° turn around a corner, the angular velocity in the image frame peaks at $> 80^\circ/\text{s}$.
- If latency exceeds $80\text{ ms}$, the visual feedback lag causes the UAV to continue flying straight for almost $1.0\text{ m}$ past the corner before beginning yaw rotation. This directly accounts for the 0.0% retention in Trial 1 (Nominal 180° Turn).

### 3.3 Obstacle Avoidance & Emergency Braking
- Stopping distance formula: $D_{stop} = V_0 \cdot t_{latency} + \frac{V_0^2}{2 a_{max}}$.
- At $V_0 = 1.8\text{ m/s}$ and braking deceleration $a_{max} = 2.4\text{ m/s}^2$:
  - If latency is $40\text{ ms}$: Reaction distance is $0.07\text{ m}$; Total stop = $0.75\text{ m}$.
  - If latency is $250\text{ ms}$ (e.g. Wi-Fi lag): Reaction distance is $0.45\text{ m}$; Total stop = $1.13\text{ m}$.
  - A $200\text{ ms}$ latency increase consumes nearly half the safety clearance margin!

### 3.4 Lost-Target Recovery
- In turn-point navigation (`ADVANCING_TO_TURN_POINT`), the drone computes the advance vector from the last known 3D pinhole coordinates.
- If the last frame was delayed by $150\text{ ms}$, the estimated turn point is offset by the human's travel distance during that lag, causing the drone to turn too late and miss the pedestrian's path.
