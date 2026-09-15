# Obstacle, Collision, and Stuck Detection Architecture Review

**Status:** Technical Review & Safety Architecture Specification  
**Current Implementation Status:** **COMPLETELY ABSENT (0% Implemented)**  
**Target Subsystem:** Safety, Collision Prevention, and Flight Envelope Protection

---

## 1. Current State Audit: Sensors, Topics, and PX4 Capabilities

A thorough audit of the simulation models, ROS 2 topics, and autopilot parameters reveals that **the current drone has zero real-time obstacle perception**:

| Subsystem Component | Reality in Current Codebase | Risk & Capability Assessment |
|---|---|---|
| **Forward Sensors** | **None.** `gazebo/models/x500/model.sdf` contains only an RGB camera (pitch 0.65 rad) and a downward-pointing 1D rangefinder (`LW20` oriented at 1.57 rad). | The drone is blind to trees, buildings, and overhead hazards in its direction of travel. |
| **360° / Depth Sensing** | **None.** No stereo depth camera, no 2D/3D LiDAR, no ultrasonic/mmWave radar. | Inability to map surroundings or detect obstacles during lateral or yaw maneuvers. |
| **ROS 2 Obstacle Topics** | **Zero.** No `/obstacle/distance`, `/scan`, or `/camera/depth/image_rect_raw` topics exist in `tracking_stack.launch.py`. | ROS perception stack has no collision awareness. |
| **PX4 Collision Prevention** | Disabled in default SITL (`CP_DIST` default is disabled/not mapped). No `OBSTACLE_DISTANCE` MAVLink stream is ingested. | PX4 executes commanded velocity vectors even if an obstacle is 10 cm ahead. |
| **Tree Clearance Mechanism** | Open-loop heuristic only (`target_dx = dx + 1.8m` in `pinhole_geometry.py`). | Assumes a tree exists at a fixed geometric offset; does not actually sense physical geometry. |
| **Stuck Detection** | **Zero.** No logic compares commanded velocity against actual GPS/EKF2 displacement. | If the drone hits a tree trunk or obstacle, it will continue spinning motors and demanding forward velocity until EKF2 diverges or motor burnout occurs. |

---

## 2. Decoupling the Safety Triad

Obstacle handling must not be treated as a single monolithic block. It comprises three distinct functional layers operating at different timescales:

```
+---------------------------------------------------------------------------------+
| 1. COLLISION PREVENTION (Timescale: 10–50 ms)                                   |
|    - Goal: Hard emergency braking to prevent physical impact.                   |
|    - Logic: Reactive distance thresholding (Stop margin: d < d_stop).          |
|    - Output: Velocity command clamping / emergency zero-velocity override.     |
+---------------------------------------------------------------------------------+
                                      |
+---------------------------------------------------------------------------------+
| 2. OBSTACLE AVOIDANCE (Timescale: 100–500 ms)                                   |
|    - Goal: Active local path planning to detour around obstacles.               |
|    - Logic: Vector Field Histogram (VFH+), Artificial Potential Fields (APF),   |
|      or 3D occupancy grid path search.                                          |
|    - Output: Modified lateral/vertical deviation vector.                        |
+---------------------------------------------------------------------------------+
                                      |
+---------------------------------------------------------------------------------+
| 3. STUCK DETECTION (Timescale: 1.0–3.0 s)                                       |
|    - Goal: Detecting that the drone is trapped/blocked despite thrust commands. |
|    - Logic: Discrepancy between commanded velocity and physical EKF2 motion.    |
|    - Output: Motor safety cut-off, hover hold, and high-priority operator alert.|
+---------------------------------------------------------------------------------+
```

---

## 3. Strict Priority Hierarchy: Obstacle Safety vs. Target Controller

### 3.1 Architectural Rule
**The Target Controller NEVER overrides the Collision Safety Layer.**

The flow of motion authority is strictly unidirectional:
```
Target Tracking Controller (Visual Servoing)
      ↓
[Desired Velocity Vector: V_desired]
      ↓
Obstacle Avoidance Layer (Path Detour Planning)
      ↓
[Modified Velocity Vector: V_avoid]
      ↓
Collision Prevention Layer (Hard Braking Gate)
      ↓
[Safe Velocity Vector: V_safe]
      ↓
PX4 MAVLink Offboard Dispatcher (motion_arbiter)
```

### 3.2 Concrete Scenario: Target Ahead with Tree Directly in Path

```
     [Human Target]
           ↑
        [Tree] (Obstacle at 2.5m)
           ↑
       [UAV Drone]
```

1. **Target Controller Observation:** The person is walking forward, centered in the camera frame. The visual servo law outputs:
   $$V_{desired} = [V_x = +1.2\text{ m/s}, V_y = 0.0\text{ m/s}, V_z = 0.0\text{ m/s}]$$
2. **Safety Layer Intervention:**
   - Obstacle sensor detects the tree trunk at $d = 2.5\text{ m}$ in the forward sector ($[-15^\circ, +15^\circ]$).
   - If Obstacle Avoidance is active: It computes a lateral detour vector $V_{avoid} = [V_x = +0.6\text{ m/s}, V_y = +0.8\text{ m/s}]$.
   - If Avoidance is blocked or unavailable, Collision Prevention triggers:
     $$d_{measured} \le d_{brake\_threshold} \implies V_{safe} = [V_x = 0.0\text{ m/s}, V_y = 0.0\text{ m/s}, V_z = 0.0\text{ m/s}]$$
3. **Result:** The drone **halts and hovers** $2.0\text{ m}$ from the tree. The target controller's forward pursuit request is overridden. The system issues an `OBSTACLE_BRAKING` alarm to the operator. Under no circumstances may tracking override safety to ram the tree.

---

## 4. Stuck Detection Algorithm Specification

### 4.1 Physical Failure Pattern
When a multicopter is blocked by an obstacle (e.g. tree canopy, wire, wall), the flight controller demands positive thrust and velocity, but physical displacement remains near zero. This leads to motor overheating, attitude jitter, and eventual violent EKF2 divergence.

### 4.2 Mathematical Formulation
Every control tick ($10\text{ Hz}$), the detector samples:
1. Commanded forward velocity: $V_{cmd} = \sqrt{V_{x,cmd}^2 + V_{y,cmd}^2}$
2. Estimated actual velocity (from PX4 `LOCAL_POSITION_NED`): $V_{act} = \sqrt{v_x^2 + v_y^2}$
3. Integrated physical displacement over sliding window $\Delta T = 2.0\text{ s}$:
   $$\Delta P_{measured} = \|\mathbf{p}(t) - \mathbf{p}(t - \Delta T)\|$$

**Stuck Condition Evaluator:**
$$\text{Condition A:} \quad V_{cmd} > 0.60\text{ m/s} \quad (\text{Drone intends to move})$$
$$\text{Condition B:} \quad V_{act} < 0.15\text{ m/s} \quad (\text{Vehicle is stationary})$$
$$\text{Condition C:} \quad \Delta P_{measured} < 0.30\text{ m} \quad (\text{Displacement is negligible})$$
$$\text{Condition D:} \quad \Delta t_{persists} \ge 2.5\text{ s} \quad (\text{Condition holds continuously})$$

If all conditions hold simultaneously:
$$\mathbf{State} \leftarrow \mathbf{STUCK\_SUSPECTED}$$

### 4.3 Action upon `STUCK_SUSPECTED`:
1. Immediately command $V_x = 0, V_y = 0$ (neutralize forward thrust).
2. If altitude permits ($h > 2.0\text{ m}$), command a gentle backward pulse ($V_x = -0.4\text{ m/s}$ for $0.8\text{ s}$) to disengage from foliage.
3. Switch arbiter state to `STANDBY` hover.
4. Broadcast high-priority MAVLink `STATUSTEXT` error: `"[EMERGENCY] DRONE STUCK DETECTED - FORWARD MOTION HALTED"`.

---

## 5. Comprehensive Safety State Machine

```
                        +----------------------+
                        |        NORMAL        |
                        | (Free flight / track)|
                        +----------------------+
                                   |
                     Obstacle detected (d < 4.0m)
                                   v
                        +----------------------+
                        |   OBSTACLE_WARNING   |
                        | (Speed capped at 50%)|
                        +----------------------+
                                   |
                     Obstacle close (d < 2.5m)
                                   v
                        +----------------------+
                        |       BRAKING        |
                        | (Decelerate to zero) |
                        +----------------------+
                               /        \
              Avoidance path open?     Dead-end / Blocked
                             /            \
                            v              v
            +--------------------+    +----------------------+
            |      AVOIDING      |    |         HOLD         |
            | (Detour around obs)|    | (Hover at safe dist) |
            +--------------------+    +----------------------+
                                           |
                              Commanded > 0 but Movement == 0 (>2.5s)
                                           v
                              +--------------------------+
                              |     STUCK_SUSPECTED      |
                              | (Disengage & Alert Pilot)|
                              +--------------------------+
```

---

## 6. Operator Awareness & Telemetry Contract

### 6.1 Current Gaps in Operator Feedback
- Currently, `/tracking/motion_state` only outputs: `"<STATE>:<target_id>:<ARMED|DISARMED>"`.
- The operator in QGroundControl or looking at the HUD cannot tell whether:
  - The drone stopped because the target stopped, or because an obstacle is blocking the path.
  - Tracking was lost due to occlusion, or because the camera dropped frames.
  - The vehicle is experiencing an Offboard latency stall.

### 6.2 Proposed Telemetry Contract

#### Channel A: ROS 2 Topic `/tracking/control_health` (JSON payload at 2 Hz)
```json
{
  "timestamp": 1789374050.25,
  "flight_state": "TRACKING",
  "safety_state": "OBSTACLE_WARNING",
  "target_handle": "TARGET_001",
  "target_status": "TRACKING",
  "obstacle_distance_m": 3.42,
  "obstacle_sector": "FRONT_CENTER",
  "stuck_detected": false,
  "vision_fresh": true,
  "offboard_healthy": true,
  "control_latency_ms": 14.2
}
```

#### Channel B: MAVLink `STATUSTEXT` to QGroundControl
The flight controller should emit standardized MAVLink text messages on state transitions:
- `[TRACKING] Target locked (TARGET_001)`
- `[WARN] Target uncertain - extrapolating (1.2s)`
- `[WARN] Obstacle ahead at 2.8m - braking applied`
- `[ALERT] Path blocked by obstacle - entering hover hold`
- `[EMERGENCY] Vehicle stuck detected - pilot override recommended`
- `[INFO] Returning to STANDBY hover`
