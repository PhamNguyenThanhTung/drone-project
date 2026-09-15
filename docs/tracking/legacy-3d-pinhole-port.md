# Porting Legacy 3D Pinhole Distance Estimation & Control to PX4 + ROS 2

## 1. Executive Overview & Architecture Decoupling

In reference commit `cb6b1ae` (ArduPilot SITL / `vehicle_yaw_search.py`), the drone demonstrated highly stable person following and robust corner recovery around dense trees. However, that implementation coupled MAVLink communication, geometric projection, state arbitration, and control laws into a monolithic script.

During the migration to PX4 and ROS 2, an area-based heuristic ($A = w \times h$) was temporarily substituted as a proxy for distance. While area-based control provides basic proximity feedback, it suffers from severe depth ambiguity: crouching, turning sideways, or changing posture triggers false forward/backward lunges. Furthermore, turning recovery was degraded by spinning 360° or losing track of the corner turn-point.

This refactor restores the verified **3D Pinhole Camera Geometry** and **safe-zone flight control laws** from `cb6b1ae`, architected cleanly into four decoupled layers:

```
+-------------------------------------------------------------------------+
| Layer 1: Perception & Geometry (pinhole_geometry.py)                    |
| - 3D Pinhole ray projection from pixel error (dx, dy)                   |
| - Altitude & camera downward tilt (0.65 rad) trigonometric compensation |
| - Turn-point navigation vector calculation with tree clearance (+1.8m)  |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
| Layer 2: Target Tracking Management (yolo_detector_node.py)             |
| - YOLOv8n object detection & ByteTrack ID persistence                   |
| - Target ID assignment, locking, and click-to-track arbitration        |
| - Publishes pixel error & bbox telemetry on /tracking/error             |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
| Layer 3: Control Law & State Arbitration (motion_arbiter_node.py)       |
| - 50% Safe-Zone deadband (hover when target is centered: |dx|<20, |dy|<25)
| - Constant base walking pursuit (0.85 m/s) + vertical excess boost      |
| - Lateral-yaw coupled slowdown during sharp turns                       |
| - Bottom-loss straight reverse recovery (no blind 360° spin)           |
| - 2-Stage Turn-point corner recovery (Advance straight -> Rotate yaw)  |
+-------------------------------------------------------------------------+
                                    |
                                    v
+-------------------------------------------------------------------------+
| Layer 4: Autopilot Transport & Safety Layer                             |
| - MAV_FRAME_BODY_NED velocity & yaw-rate streaming at 10 Hz            |
| - High-rate EKF2 altitude & NED position caching                        |
| - Slew-rate acceleration/deceleration limiting (prevents pitch bobbing) |
| - Watchdog timers & SITL failsafe tolerances (COM_OF_LOSS_T)            |
+-------------------------------------------------------------------------+
```

---

## 2. Mathematical Derivation of 3D Pinhole Camera Geometry

### 2.1 Coordinate Frames & Calibration Constants

The camera mounted on the PX4 `x500` quadrotor in Gazebo Harmonic (`gazebo/models/x500/model.sdf`) has the following physical characteristics:
- **Native Resolution:** $W_{native} = 640\text{ px}$, $H_{native} = 480\text{ px}$
- **Horizontal Field of View:** $HFOV = 2.0\text{ rad} \approx 114.59^\circ$
- **Camera Mount Downward Pitch:** $\alpha_0 = 0.65\text{ rad} \approx 37.24^\circ$
- **Tracking / Inference Resolution:** $W = 416\text{ px}$, $H = 416\text{ px}$
- **Image Principal Point:** $c_x = 208.0\text{ px}$, $c_y = 208.0\text{ px}$

Using pinhole projection, the native focal lengths are:
$$f_{x, native} = \frac{W_{native} / 2}{\tan(HFOV / 2)} = \frac{320}{\tan(1.0\text{ rad})} = \frac{320}{1.5574077} \approx 205.47\text{ px}$$

Scaling to the $416 \times 416$ normalized YOLO tracking frame:
$$f_x = f_{x, native} \times \frac{416}{640} = 205.47 \times 0.65 = 133.55\text{ px}$$
$$f_y = f_{y, native} \times \frac{416}{480} = 205.47 \times 0.8667 = 178.07\text{ px}$$

These parameters match the exact calibration constants established in `cb6b1ae`.

### 2.2 Forward & Lateral Ground Distance Estimation

Given the pixel offsets $(\Delta x, \Delta y)$ from the image center $(c_x, c_y)$ published on `/tracking/error`:
- $\Delta x = x_{pixel} - c_x$
- $\Delta y = y_{pixel} - c_y$

The angular elevation offset $\theta_y$ of the target ray relative to the camera optical axis is:
$$\theta_y = \arctan\left(\frac{\Delta y}{f_y}\right)$$

Total angle of depression below the horizontal horizon:
$$\theta = \alpha_0 + \theta_{pitch} + \theta_y$$
where $\alpha_0 = 0.65\text{ rad}$ is nominal camera pitch and $\theta_{pitch}$ is the real-time vehicle pitch angle from EKF2 attitude telemetry.

The relative vertical separation between drone and target is:
$$h_{rel} = \max(h_{drone} - h_{person}, h_{min})$$
where $h_{person} = 0.90\text{ m}$ (center of torso) and $h_{min} = 1.50\text{ m}$ protects against division-by-zero singularities during low-altitude flight.

The forward ground distance $dx$ and lateral ground distance $dy$ are computed as:
$$dx = \frac{h_{rel}}{\tan(\theta)}$$
$$dy = dx \cdot \left(\frac{\Delta x}{f_x}\right) \cdot \cos(\theta)$$
$$d_{ground} = \sqrt{dx^2 + dy^2}$$

### 2.3 Turn-Point Trajectory Vector with Tree Clearance

When the target turns a corner around dense trees or obstacles and disappears laterally, tracking switches to turn-point navigation. To prevent clipping foliage at the corner apex, a physical tree clearance margin ($m = 1.80\text{ m}$) is added along the line of sight:
$$target\_dx = dx + m$$
$$target\_dy = dy$$
$$target\_dist = \sqrt{target\_dx^2 + target\_dy^2}$$

---

## 3. Restored Flight Control Laws

### 3.1 Safe-Zone Deadband (`SAFE_ZONE_HOVER`)

To eliminate continuous micro-oscillations, a 50% central safe-zone deadband is enforced:
- Horizontal deadband: $|\Delta x| \le 20.0\text{ px}$
- Vertical deadband: $|\Delta y| \le 25.0\text{ px}$

When $(\Delta x, \Delta y)$ falls within this window, the vehicle executes `SAFE_ZONE_HOVER`:
- $V_x = 0.0\text{ m/s}$
- $V_y = 0.0\text{ m/s}$
- $\omega_z = 0.0\text{ rad/s}$

### 3.2 Proportional Pursuit & Backing

When outside the deadband:
1. **Target Ahead ($\Delta y < -25\text{ px}$):**
   $$V_x = V_{walk} + K_{p\_y\_boost} \cdot (-\Delta y - 25.0)$$
   where $V_{walk} = 0.85\text{ m/s}$ and $K_{p\_y\_boost} = 0.0050$.
   If the target is making a sharp turn ($|\Delta x| > 60\text{ px}$), forward speed is smoothly tapered:
   $$V_x \leftarrow V_x \cdot \max\left(0.20, 1.0 - \frac{|\Delta x| - 60.0}{60.0}\right)$$

2. **Target Too Close ($\Delta y > +25\text{ px}$):**
   $$V_x = -\left(V_{backup} + K_{p\_y\_boost} \cdot (\Delta y - 25.0)\right)$$
   where $V_{backup} = 0.75\text{ m/s}$.

3. **Yaw Tracking:**
   $$\omega_z = K_p \cdot \Delta x \quad (K_p = 0.0070, |\omega_z| \le 1.50\text{ rad/s})$$

### 3.3 Lost Target Recovery Laws

1. **Bottom Exit Recovery (`BACKING_UP_TO_RECOVER`):**
   If the target moves rapidly under the drone and exits out the bottom of the camera frame ($\Delta y_{last} > 25.0\text{ px}$), the drone executes a straight reverse:
   $$V_x = -0.85\text{ m/s}, \quad V_y = 0.0\text{ m/s}, \quad \omega_z = 0.0\text{ rad/s}$$
   for up to $t_{bottom} = 4.0\text{ s}$. Crucially, yaw rate is clamped to zero to prevent blind spinning.

2. **Turn-Point Traversal (`ADVANCING_TO_TURN_POINT` & `ROTATING_AT_TURN_POINT`):**
   If the target disappears laterally ($\Delta y_{last} \le 25.0\text{ px}$):
   - **Stage 1:** Fly along the 3D turn-point vector $(target\_dx, target\_dy)$ at $V = 1.35\text{ m/s}$ with $\omega_z = 0.0\text{ rad/s}$ until $d_{advanced} \ge target\_dist$ or timeout ($8.0\text{ s}$).
   - **Stage 2:** At the turn point, hover in place ($V_x = 0, V_y = 0$) and rotate with heading angular rate $\omega_z = \text{dir} \times 0.50\text{ rad/s}$ towards the target's exit direction for up to $4.0\text{ s}$ to reacquire the target.

---

## 4. Parameter Mapping & Configuration

All geometry and control constants are fully configurable via ROS 2 parameters with backward-compatible aliases:

| Parameter | Alias | Default | Unit | Description |
|---|---|---|---|---|
| `camera_pitch_rad` | - | `0.65` | rad | Downward pitch angle of camera |
| `fx` | - | `133.55` | px | Horizontal focal length (416x416 frame) |
| `fy` | - | `178.07` | px | Vertical focal length (416x416 frame) |
| `cx`, `cy` | - | `208.0` | px | Principal center point |
| `person_height_m` | - | `0.90` | m | Assumed center of torso height |
| `min_relative_height_m` | - | `1.50` | m | Altitude division safety threshold |
| `target_distance_m` | - | `4.50` | m | Nominal tracking standoff distance |
| `tree_clearance_margin_m`| `tree_clearance_margin` | `1.80` | m | Clearance buffer for turn point navigation |
| `base_forward_speed_mps` | `default_walk_speed` | `0.85` | m/s | Constant forward walking speed |
| `backward_speed_mps` | `default_backup_speed` | `0.75` | m/s | Backing velocity when target too close |
| `deadband_x_px` | `deadband_x` | `20.0` | px | Horizontal safe-zone deadband half-width |
| `deadband_y_px` | `deadband_y` | `25.0` | px | Vertical safe-zone deadband half-height |
| `turn_point_speed_mps` | `turn_point_speed` | `1.35` | m/s | Stage 1 advance speed to turn point |
| `turn_point_rotate_speed_rad` | `turn_point_rotate_speed` | `0.50` | rad/s | Stage 2 yaw rotation rate at turn point |
| `turn_point_timeout_s` | `turn_point_timeout` | `8.0` | s | Max duration for Stage 1 turn-point flight |
| `turn_point_rotate_timeout_s` | - | `4.0` | s | Max duration for Stage 2 yaw search |
| `bottom_backup_timeout_s` | `bottom_backup_timeout` | `4.0` | s | Max duration for bottom reverse recovery |
| `bottom_backup_speed_mps` | `bottom_backup_speed` | `0.85` | m/s | Speed for bottom reverse recovery |
