# Post-Audit Architecture Review & Analysis

**Audit Date:** 2026-09-14  
**Repository:** `/home/tungt/drone-project`  
**Branch:** `refactor/restore-3d-pinhole-tracking`  
**Classification:** Read-Only Review & Blueprint Proposal (Zero Production Code Changes)

---

## 1. Codebase Verification of the 10 Audit Findings

Every key finding from the initial audit was re-verified directly against the active source code in `ros2_ws/src/vision_tracking`:

| # | Audit Finding | Source Code Evidence | Verification Verdict | Severity / Priority |
|---|---|---|---|---|
| **1** | **Actual tracker is Ultralytics ByteTrack** | `yolo_detector_node.py` (L202–212): `self.model.track(..., persist=True, tracker=self.tracker, ...)`. In `yolo_detector.yaml`: `tracker: "bytetrack.yaml"`. No BoT-SORT, DeepSORT, or OC-SORT code exists. | **CONFIRMED** | Baseline Fact |
| **2** | **No production Re-ID implementation** | Git commit `389689f` permanently removed partial Re-ID scripts. Zero feature extraction models (OSNet, FastReID, MobileNet-ReID) or feature galleries exist in `ros2_ws/`. Only documentation mentions remain. | **CONFIRMED** | Architecture Reality |
| **3** | **Target identity is tied directly to ByteTrack `track_id`** | `yolo_detector_node.py` (L245, L398, L412, L472): `self.target_id = tid`. The mission target is represented solely by an ephemeral integer issued by ByteTrack. | **CONFIRMED** | **P0 (Design Defect)** |
| **4** | **Auto mode selects candidates via `area * confidence`** | `yolo_detector_node.py` (L415): `return max(cands, key=lambda c: c[6] * c[5])` where `c[6]` is box area ($w \times h$) and `c[5]` is detection confidence. | **CONFIRMED** | **P0 (Critical Safety Risk)** |
| **5** | **Current 3D pinhole geometry exists** | `pinhole_geometry.py` provides `PinholeCameraConfig`, `TargetGeometry`, and `estimate_pinhole_geometry()`. Integrated in `motion_arbiter_node.py` (L1131–1195). | **CONFIRMED** | Active Capability |
| **6** | **`motion_arbiter` is the sole normal MAVLink writer** | `motion_arbiter_node.py` (L1351): `self._send_offboard_velocity(...)` via `udpin:0.0.0.0:14540`. Only node with an open MAVLink command socket to PX4. | **CONFIRMED** | Architectural Boundary (Must Keep) |
| **7** | **Safety status can PASS while target retention fails completely** | `logs/multi_trial_summary.json` (Commit `4b156b9`): Trial 1 (Nominal 180° Turn) recorded **0.0% retention**, Trial 4 (Lateral Right) recorded **3.0% retention**. Both received overall `Status: PASS` because the drone did not crash or drop altitude. | **CONFIRMED** | **P1 (Metric Decoupling Debt)** |
| **8** | **Package test suite fails on linters** | Running `pytest -q ros2_ws/src/vision_tracking/test` yielded `2 failed, 1 skipped`. Style tests fail due to broad scans over build artifacts and test scripts. | **CONFIRMED** | **P2 (Tooling Debt)** |
| **9** | **Live ROS rate measurement is unavailable offline** | Running `ros2 topic hz` fails with `PermissionError: [Errno 1] Operation not permitted` in current sandbox/container when the stack is not running. | **CONFIRMED** | Operational Constraint |
| **10**| **Severe flake8 and pep257 debt exists** | Pytest reports 433 flake8 errors and 72 pep257 errors because linters scan `ros2_ws/install/`, root test shims, and simulation scripts. | **CONFIRMED** | **P2 (Linter Scope Debt)** |

---

## 2. P0: Wrong-Target Hijack Risk (Root Cause & Failure Trace)

### 2.1 The Vulnerability Trace
The most dangerous failure mode in the current system is an **uncommanded target switch**, where the drone quietly transfers mission tracking to an unintended person without human intervention or alarm.

```
Camera Frame (t)
      ↓
YOLOv8 Detection (Persons A & B detected)
      ↓
ByteTrack Association (A: track_id=1, B: track_id=2)
      ↓
yolo_detector_node.py :: select_target()
      ↓
[CRITICAL DEFECT]: Target A occluded for 1 frame -> cands has only B
      ↓
Auto-fallback: max(cands, key=area * confidence) -> Selects Person B (track_id=2)
      ↓
Target Identity Remap: self.target_id = 2
      ↓
/tracking/error Point(x, y, area) computed from Person B
      ↓
motion_arbiter_node.py: Consumes Point without Target ID verification
      ↓
MAVLink SET_POSITION_TARGET_LOCAL_NED sent to PX4
      ↓
Drone pursues Person B indefinitely! (Target A permanently abandoned)
```

### 2.2 Concrete Code Proof

#### Vulnerability Path A: Auto-Mode Silent Hijack
In `ros2_ws/src/vision_tracking/vision_tracking/yolo_detector_node.py` (lines 411–415):
```python
if self.target_id is not None:
    same = [c for c in cands if c[0] == self.target_id]
    if same:
        return same[0]
return max(cands, key=lambda c: c[6] * c[5])  # <--- CRITICAL BUG
```
- **Trigger:** Target A (currently tracked) is occluded by a tree, passes behind another person, or experiences a 1-frame detector dropout.
- **Condition:** `same` is empty because Target A's `track_id` is missing in this frame, but Person B (a bystander or crossing pedestrian) is visible.
- **Execution:** Line 415 executes unconditionally, selecting Person B because Person B maximizes `area * confidence`.
- **Consequence:** `self.target_id` is permanently assigned to Person B's ID. When Person A emerges from occlusion, Person A is ignored forever. The UAV tracks the wrong human.

#### Vulnerability Path B: Manual Lock Hijack via Proximity
In `ros2_ws/src/vision_tracking/vision_tracking/yolo_detector_node.py` (lines 462–477):
```python
is_valid = (iou >= self.reacquire_min_iou) or (dist_norm < 1.2 and proximity_score >= 0.45)
if is_valid and hybrid_score > best_score:
    best = cand
...
self.manual_target_id = int(best[0])
```
- **Trigger:** Target A is manually locked by the operator. Target A and Person B cross paths. Target A is momentarily occluded by Person B.
- **Condition:** ByteTrack loses Target A. Target A's last smoothed box overlaps Person B (`dist_norm < 1.2`).
- **Execution:** `_reacquire_lock` evaluates Person B. Since Person B is within 1.2 box dimensions of Target A's last position, `is_valid` evaluates to `True`.
- **Consequence:** `self.manual_target_id` is remapped to Person B. The operator's explicit lock is hijacked by a bystander.

#### Vulnerability Path C: Blind Controller Consumption
In `ros2_ws/src/vision_tracking/vision_tracking/motion_arbiter_node.py` (lines 1131–1145):
The subscriber receives `geometry_msgs/msg/Point`:
```python
def on_tracking_error(self, msg: Point):
    self.error_x = msg.x
    self.error_y = msg.y
    self.area = msg.z
```
- **Finding:** The message contains no `target_id`, no `target_handle`, no timestamp, and no source verification. The flight controller cannot detect if the perception node switched people. It blindly servos whatever coordinates arrive.

### 2.3 Proposed Fix Architecture (Fail-Closed Target Lock)
1. **Never Fall Back to `max(area * confidence)` during an active mission:** If the locked target is lost, the system MUST transition to `TARGET_UNCERTAIN` or `RECOVERY_HOLD`. It must **never** pick another candidate automatically.
2. **Require Target Handle Continuity:** Target identity must be tracked by an explicit `target_handle` data structure, not an ephemeral tracker integer.
3. **Multi-Stage Recovery Gates:** Re-acquisition must pass 2D motion gating, 3D pinhole position gating, and velocity vector consistency before re-binding. If ambiguity exists (e.g. crossing), the system must request operator confirmation or hold position.

---

## 3. Multi-Object Tracker Evaluation: ByteTrack vs. Alternatives

### 3.1 State of the Art in Literature vs. Project Evidence

| Tracker | Primary Mechanism | UAV Literature Evidence (VisDrone / UAVDT) | Project Empirical Evidence | Suitability for This Project |
|---|---|---|---|---|
| **ByteTrack** *(Current)* | Kalman Filter on 2D bbox coordinates; two-stage association using both high- and low-confidence detections. | Fast (100+ FPS on GPU). Highly effective when camera is static or smoothly moving. Fails under sudden camera ego-motion or sharp yaw rotations. | Baseline: 92.2% retention in fast turn, but **0.0% in nominal 180°** and **3.0% in lateral right**. Loses track when drone yaws rapidly. | **Keep as baseline benchmark**; insufficient alone for high-yaw UAV maneuvers. |
| **BoT-SORT + GMC** | Adds **Camera Motion Compensation (GMC)** via sparse optical flow (ORB/ECC) + Kalman Filter with width/height state estimation + optional appearance. | Consistently outperforms ByteTrack on moving-camera UAV benchmarks (HOTA +3–6%) by subtracting camera ego-rotation from bounding box velocity. | Not yet integrated in codebase. GMC computational cost is ~3–6 ms/frame on 640x480 images. | **Preferred Candidate for Benchmark.** Directly addresses the high-yaw retention failures seen in Trials 1 & 4. |
| **OC-SORT** | **Observation-Centric SORT:** Computes virtual trajectories across occlusion gaps to prevent Kalman filter covariance blowup during sharp direction changes. | Strong performance in non-linear pedestrian maneuvers (sudden reversals, sharp corners). Lower CPU overhead than GMC. | Not yet integrated in codebase. Highly relevant for tree cornering. | **Strong Alternative Candidate** for benchmark alongside BoT-SORT. |

### 3.2 Key Recommendation
Do NOT immediately replace ByteTrack in production code. Instead, implement a **controlled benchmark matrix** comparing:
1. `ByteTrack` (current baseline)
2. `ByteTrack + Target Handle Lock` (fixes wrong-target hijack)
3. `BoT-SORT with GMC (Appearance OFF)` (tests camera-motion compensation benefits)
4. `OC-SORT` (tests non-linear momentum benefits)

Measure: HOTA, IDF1, ID Switches (IDSW), Frame Processing Latency (ms), and Target Retention (%) under identical Gazebo scenarios.

---

## 4. 3D Pinhole Geometry: Trace & Ground-Truth Verification

### 4.1 Dataflow Trace
- **Calculation:** Performed in `pinhole_geometry.py` via `estimate_pinhole_geometry(error_x, error_y, altitude, pitch)`.
- **Publisher:** Published by `motion_arbiter_node.py` on `/tracking/target_geometry` (`std_msgs/String` containing JSON) and `/tracking/ground_distance` (`geometry_msgs/Point`).
- **Consumer:** Currently, **zero ROS nodes subscribe** to `/tracking/target_geometry` or `/tracking/ground_distance`.
- **Control Usage:** The geometry is used **internally** within `motion_arbiter_node.py` to calculate the turn-point vector $(target\_dx, target\_dy)$ for `ADVANCING_TO_TURN_POINT` and to log telemetry. Regular forward/backward speed control still uses pixel error `error_y`.

### 4.2 Calibration Constants vs. Gazebo Physical Model

| Parameter | Legacy `cb6b1ae` | Current Software | Gazebo SDF (`gazebo/models/x500/model.sdf`) | Match Assessment |
|---|---|---|---|---|
| Camera Mount Pitch ($\alpha_0$) | $0.65\text{ rad} \approx 37.24^\circ$ | `0.65 rad` | `<pose relative_to="base_link">0.12 0 -0.05 0 0.65 0</pose>` | **Exact Match** |
| Horizontal FOV | Not specified | $2.00\text{ rad} \approx 114.59^\circ$ | `<horizontal_fov>2.0</horizontal_fov>` | **Exact Match** |
| Native Resolution | $640 \times 480$ | $640 \times 480$ | `<width>640</width><height>480</height>` | **Exact Match** |
| Tracking Coordinate Frame | $416 \times 416$ | $416 \times 416$ | Scaled by `yolo_detector_node` | Verified |
| Focal Length $f_x$ | $133.55\text{ px}$ | `133.55 px` | $\frac{320}{\tan(1.0)} \times \frac{416}{640} = 205.47 \times 0.65 = 133.55\text{ px}$ | **Mathematically Exact** |
| Focal Length $f_y$ | $178.07\text{ px}$ | `178.07 px` | $205.47 \times \frac{416}{480} = 205.47 \times 0.8667 = 178.07\text{ px}$ | **Mathematically Exact** |
| Person Height Assumption ($h_{person}$) | $0.90\text{ m}$ | `0.90 m` | Pedestrian height is 1.75m; center of torso $\approx 0.90\text{ m}$ | **Good Assumption** |
| Nominal Standoff Distance | $4.50\text{ m}$ | `4.50 m` | Setpoint in mission planner | Configurable |
| Tree Clearance Margin | $+1.80\text{ m}$ | `1.80 m` | Safe distance from foliage during turn | Configurable |

### 4.3 Calibration Discrepancy & Real-World Reality
While the constants match the simulated Gazebo SDF camera perfectly, **they will not match a physical camera**. A physical camera on a real drone has:
1. Lens distortion (radial $k_1, k_2$, tangential $p_1, p_2$).
2. Assembly tolerance on gimbal/mount pitch ($\pm 2^\circ$ error produces up to $1.5\text{m}$ distance error at $4.5\text{m}$ range).
3. Rolling shutter distortions during high-rate yaw.

**Validation Requirement:** Before deploying on physical hardware, calibrate with a standard OpenCV chessboard (`camera_calibration` package) and publish standard `sensor_msgs/CameraInfo`.

---

## 5. Control & Legacy Behavior Comparison (`cb6b1ae` vs. Current)

| Flight Behavior | Legacy `cb6b1ae` (ArduPilot) | Current `refactor` (PX4) | Evaluation | Recommendation |
|---|---|---|---|---|
| **Safe-Zone Deadband** | $|\Delta x| \le 20\text{px}, |\Delta y| \le 25\text{px}$: Hover. | $|\Delta x| \le 20\text{px}, |\Delta y| \le 25\text{px}$: Hover. | Eliminates jitter and limit cycles when target is steady. | **RESTORED & KEEP** |
| **Forward Pursuit Speed** | Base $0.85\text{ m/s} + K_p \times \Delta y$ boost. | Base $0.85\text{ m/s} + K_p \times \Delta y$ boost. | Matches human walking pace ($1.1\text{ m/s}$) smoothly. | **RESTORED & KEEP** |
| **Backing Speed** | Fixed $-0.75\text{ m/s}$ proportional reverse. | Fixed $-0.75\text{ m/s}$ proportional reverse. | Safe separation control without aggressive heave. | **RESTORED & KEEP** |
| **Yaw Tracking Law** | Proportional yaw rate, max $1.5\text{ rad/s}$. | Proportional yaw rate, max $1.5\text{ rad/s}$. | Clean angular tracking. | **KEEP** |
| **Sharp Turn Slowdown** | Linear taper when $|\Delta x| > 60\text{ px}$. | Linear taper when $|\Delta x| > 60\text{ px}$. | Prevents overshooting corners when target turns. | **RESTORED & KEEP** |
| **Bottom-Exit Recovery** | Straight reverse ($V_x = -0.85\text{ m/s}, \omega_z = 0$). | Straight reverse ($V_x = -0.85\text{ m/s}, \omega_z = 0$). | Avoids 360° blind spin when person walks under drone. | **RESTORED & KEEP** |
| **Turn-Point Traversal** | Inline 2-stage vector advance + yaw scan. | 2-stage vector advance + yaw scan with timeout. | Navigates around tree apex to reacquire target. | **RESTORED & KEEP** |
| **Slew Rate Limiters** | Handled by ArduPilot copter limits. | Explicit acceleration limiters ($1.5\text{ m/s}^2$). | Prevents pitch oscillations and altitude drops in PX4. | **KEEP CURRENT (Superior)** |
| **Watchdog & Telemetry** | Monolithic script loop. | Multi-threaded watchdog, monotonic latency logs. | Much better diagnostics and failsafe handling. | **KEEP CURRENT (Superior)** |
| **Area Proxy Control ($z = w \cdot h$)** | Bbox area used as distance proxy. | Area proxy replaced by 3D pinhole geometry. | Area proxy caused erratic lunges on crouching/turning. | **DEPRECATE PERMANENTLY** |

---

## 6. Lost-Target Recovery: Temporal Envelopes & State Machine

When visual contact is lost, the system must distinguish between a momentary glitch and a complete loss of target.

```
       +-------------------------------------------------------+
       |                       TRACKING                        |
       |  (Target handle confirmed, high confidence, in-frame)  |
       +-------------------------------------------------------+
                                   |
                          Target unseen > 0.40s
                                   v
       +-------------------------------------------------------+
       |                      UNCERTAIN                        |
       |  (0.4s < t <= 1.5s: Bounded Kalman extrapolation,     |
       |   hold velocity, evaluate candidate velocity vectors) |
       +-------------------------------------------------------+
                                   |
                          Target unseen > 1.5s
                                   v
       +-------------------------------------------------------+
       |                      RECOVERY                         |
       |  Case 1: Bottom Loss -> BACKING_UP_TO_RECOVER         |
       |  Case 2: Lateral Loss -> ADVANCING_TO_TURN_POINT      |
       +-------------------------------------------------------+
                                   |
                    Candidate detected in gate?
                    /                         \
                 YES                           NO (t > 5.0s)
                  v                             v
       +-----------------------+     +-------------------------+
       |      REID_VERIFY      |     |      TARGET_LOST        |
       |  (Spatial gate check, |     |  (Zero velocity setpoint|
       |   Optional Re-ID)     |     |   Enter STANDBY hover)  |
       +-----------------------+     +-------------------------+
            |              |
         MATCH           AMBIGUOUS / MISMATCH
            v              v
       REACQUIRED    STAY IN RECOVERY / STANDBY
```

### 6.1 Temporal Recovery Action Matrix

| Loss Envelope | Duration | Likely Cause | Prescribed UAV Action | Identity Action |
|---|---|---|---|---|
| **Micro Loss** | $\Delta t \le 0.40\text{ s}$ (1–4 frames) | Detection false negative, motion blur, partial occlusion. | Maintain current velocity setpoints via slew limiters. Do not switch state. | Retain `target_handle`; continue Kalman state prediction. |
| **Short Occlusion** | $0.40\text{ s} < \Delta t \le 1.50\text{ s}$ | Tree trunk, lamp post, single pedestrian crossing. | Transition to `UNCERTAIN`. Decelerate smoothly to hover. Extrapolate position using last known velocity vector. | Match candidates using spatial-velocity gate. Fail-closed: do not switch to crossing pedestrian. |
| **Medium Corner Loss** | $1.50\text{ s} < \Delta t \le 4.00\text{ s}$ | Target turned 90° corner behind dense foliage. | Transition to `RECOVERY`: Stage 1 advance along pinhole 3D vector $+1.8\text{m}$ clearance; Stage 2 yaw rotation toward turn direction. | Require candidate to appear within predicted corner search sector. If multiple candidates appear, invoke Re-ID or hold. |
| **Under-Flight Loss** | $0.40\text{ s} < \Delta t \le 4.00\text{ s}$ (Bottom Exit) | Person accelerated underneath the camera FOV. | Transition to `BACKING_UP_TO_RECOVER`: Straight backward flight ($V_x = -0.85\text{ m/s}$, $\omega_z = 0$). | Candidate re-emerging at bottom of frame with forward relative velocity is bound to `target_handle`. |
| **Long Loss / Desertion**| $\Delta t > 5.00\text{ s}$ | Target left the area or entered a building. | Transition to `TARGET_LOST` -> `STANDBY`: Complete velocity zeroing, hover at current GPS coordinate, signal operator via HUD and QGC. | Deactivate `target_handle`. **Absolute prohibition on re-acquiring any candidate automatically.** Requires operator click. |

---

## 7. Re-ID Decision: Tracking-First + On-Demand Re-ID

### 7.1 Key Decision
**Do NOT run deep Re-ID models on every video frame.**

Running an embedding neural network (e.g. OSNet, FastReID, MobileNet) at 30 FPS consumes significant GPU resources (adding 15–35 ms latency), causing frame drops in YOLO and destabilizing the 10 Hz flight control loop.

### 7.2 The Recommended "On-Demand" Architecture
1. **Normal Flight (`TRACKING`):** Re-ID is **COMPLETELY OFF**. Tracking is performed purely by geometry, Kalman motion prediction, and high-frequency ByteTrack association.
2. **Triggering Conditions for Re-ID:**
   - Candidate ambiguity after crossing (two pedestrians emerge from an occlusion zone).
   - Recovery after medium loss ($1.5\text{ s} < \Delta t \le 5.0\text{ s}$).
   - Target re-entry after exiting the camera field of view.
3. **Execution:** Single-shot crop extraction passed to a lightweight embedding extractor. Compare cosine distance against the target's initial gallery embedding.
4. **Safety Gate:** If cosine similarity $> 0.75$, re-acquire `target_handle`. If between $0.45$ and $0.75$ (ambiguous), do NOT track; alert operator on HUD. If $< 0.45$, reject candidate.
