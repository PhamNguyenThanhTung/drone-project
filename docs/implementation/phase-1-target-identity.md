# Implementation Report: Phase 1 — Target Identity, Target Lock, and Fail-Closed Behavior

**Repository:** `~/drone-project`  
**Branch:** `refactor/restore-3d-pinhole-tracking`  
**Date:** September 2026  
**Status:** Completed & Validated

---

## 1. Executive Summary

Phase 1 resolves the highest-priority architectural flaw identified during the system audit:
> **The mission target identity was previously coupled directly to ByteTrack's ephemeral `track_id`, and automatic reacquisition fallback paths silently reassigned the mission to nearby pedestrians or distractors with larger bounding boxes.**

To eliminate this critical safety vulnerability, Phase 1 establishes two non-negotiable safety invariants:
1. `TRACK_ID CHANGE != TARGET_IDENTITY CHANGE`: Ephemeral tracker ID turnover (lineage shifts) no longer creates a new mission target or breaks lock. The logical mission identity (`TargetHandle`) survives tracker ID reassignment.
2. `TARGET LOST != SELECT ANOTHER PERSON`: An occluded, out-of-frame, or missing target immediately forces the system into a fail-closed posture (`UNCERTAIN` -> `TARGET_LOST`). The system **never** automatically retargets another person based on heuristic score, bounding box area, or detector confidence.

---

## 2. Current Problem Analysis

### 2.1 Ephemeral Tracker Coupling
In `yolo_detector_node.py` (prior to this fix), the detector maintained target state as a plain integer:
```python
# PRE-FIX (FLAWED)
self.target_id = tid
self.manual_target_id = req_id
```
Whenever ByteTrack lost tracking for a few frames (routine on CPU inference at 8–15 FPS) and assigned a fresh track ID (e.g., track 7 became track 15), the node struggled to disambiguate whether this was the same person or an arbitrary newcomer.

### 2.2 The P0 Auto-Retarget Vulnerability
In `select_target(cands)` (line 415 of pre-fix `yolo_detector_node.py`), when an active target was temporarily unobserved, the fallback code executed:
```python
# PRE-FIX (CRITICAL VULNERABILITY)
return max(cands, key=lambda c: c[6] * c[5]) # c[6] is area, c[5] is confidence
```
If the drone was pursuing a person in the distance and a bystander walked closer to the camera, the bystander's bounding box area was dramatically larger. Line 415 immediately and silently hijacked the mission, locking onto the bystander and driving the drone toward them.

### 2.3 Insufficient Spatial Gating in Heuristic Reacquisition
In `_reacquire_lock(cands)` (lines 433–484 of pre-fix `yolo_detector_node.py`), reacquisition relied almost entirely on normalized box-center Euclidean distance:
```python
# PRE-FIX (INSUFFICIENT GATING)
is_valid = (iou >= self.reacquire_min_iou) or (dist_norm < 1.2 and proximity_score >= 0.45)
```
During a pedestrian crossing event where Person B crossed paths with Target A, Person B satisfied `dist_norm < 1.2` during occlusion. Consequently, the drone transferred lock onto Person B and followed them away from the intended target.

---

## 3. The Logical Target Handle Identity Model

Phase 1 introduces the `TargetHandle` abstraction inside a dedicated target governance module:
`ros2_ws/src/vision_tracking/vision_tracking/target_identity.py`.

```text
                  +----------------------------------------------+
                  |                 TargetHandle                 |
                  +----------------------------------------------+
                  | handle_id: "TARGET_001" (Persistent)        |
                  | state: TRACKING / UNCERTAIN / TARGET_LOST    |
                  | lock_mode: MANUAL / AUTO                     |
                  | current_track_id: 15 (Active ByteTrack ID)   |
                  | previous_track_ids: [7]                      |
                  | last_confirmed_bbox: (x1, y1, x2, y2)        |
                  | smoothed_bbox: (sx1, sy1, sx2, sy2)          |
                  | velocity_2d: (vx_px_s, vy_px_s)              |
                  | consecutive_detections / consecutive_misses  |
                  | reacquire_count: 1                           |
                  +----------------------------------------------+
```

### Key Principles:
- **Separation of Concerns:** ByteTrack provides transient inter-frame trajectory associations (`track_id`). `TargetStateManager` owns the mission-level logical identity (`TargetHandle`).
- **Lineage History:** When a target's tracker ID changes from 7 to 15 under verified spatio-temporal continuity, `previous_track_ids` records `[7]`, `current_track_id` updates to `15`, while `handle_id` strictly remains `TARGET_001`.
- **Identity Invalidation:** A `TargetHandle` is only invalidated when:
  1. The operator explicitly clears the lock (`req_id = -1` via HUD or GCS),
  2. The target remains unobserved past the fail-closed timeout (`target_lost_timeout_s = 4.0s`),
  3. The operator explicitly commands a new lock on a different person.

---

## 4. Target State Machine

The target lifecycle is governed by a 6-state finite state machine implemented in `TargetStateManager`:

```text
   [ NO_TARGET ]
         |
         | select_target(track_id)
         v
   [ TARGET_SELECTED ]
         |
         | candidate confirmed in frame
         v
   [ TARGET_LOCKED ]
         |
         | first tracking cycle
         v
 +-> [ TRACKING ] <-----------------------------------------+
 |       |                                                  |
 |       | target observation missing                       | gated reacquisition
 |       v                                                  | passes all 5 gates
 |   [ UNCERTAIN ] -----------------------------------------+
 |       |
 |       | age > target_lost_timeout_s (4.0s)
 |       v
 |   [ TARGET_LOST ] (FAIL-CLOSED: Drone holds, zero retargeting)
 |       |
 +-------+-- operator re-selects target / clears to NO_TARGET
```

### State Definitions and Behaviors:

| State | Condition | Error Publishing (`/tracking/error`) | Motion Arbiter Action |
|---|---|---|---|
| `NO_TARGET` | No target selected or target cleared | Suppressed (None) | Standby / Hover |
| `TARGET_SELECTED` | Target designated by operator, awaiting first detection | Suppressed (None) | Standby / Hover |
| `TARGET_LOCKED` | Candidate matched and bounding box initialized | Published with smoothed bbox | Engages pursuit law |
| `TRACKING` | Candidate actively observed with positive tracking | Published with smoothed bbox | Follows pursuit / distance law |
| `UNCERTAIN` | Target temporarily occluded or missed (< 3.5s) | **Suppressed (Fail-Closed)** | Velocity coasting / backup grace |
| `TARGET_LOST` | Target missing > 4.0s without verified recovery | **Suppressed (Fail-Closed)** | Safe position hold; no retarget |

---

## 5. Multi-Gate Fail-Closed Reacquisition Policy

When the active tracker ID vanishes, `TargetStateManager._attempt_gated_reacquisition` evaluates all available candidates in the frame through 5 strict sequential validation gates:

### Gate 1: Scale Invariance Gate
Human bounding box area cannot abruptly change beyond physiological and optical bounds across frames:
```text
0.35 * Area_ref <= Area_cand <= 2.80 * Area_ref
```
*Blocks distractors walking close to the camera whose bounding box area is 5x to 10x larger.*

### Gate 2: Aspect Ratio Consistency Gate
Human vertical/horizontal aspect ratio (h/w) cannot distort arbitrarily:
```text
|AR_cand - AR_ref| / AR_ref <= 0.60
```
*Blocks false detections, limbs, or background clutter.*

### Gate 3: Physical Displacement & Implied Velocity Gate
Calculates the implied velocity required to reach the candidate position from the last confirmed position given elapsed time dt:
```text
v_implied = sqrt((cx - ref_cx)^2 + (cy - ref_cy)^2) / max(dt, 0.05) <= 450.0 px/s
```
*Blocks persons detected on the opposite side of the screen.*

### Gate 4: Spatial & IoU Proximity Gate
Evaluates bounding box overlap against the motion-predicted target centroid:
```text
IoU(Box_cand, Box_ref) >= 0.15 OR (Dist_norm <= 1.25 and v_implied <= 350.0 px/s)
```

### Gate 5: Ambiguity Separation Margin Gate (Anti-Crossing Protection)
If multiple candidates satisfy Gates 1–4 (classic crossing pedestrians scenario), candidates are scored by composite affinity:
```text
Score = 0.50 * IoU + 0.30 * (1 - Dist_norm/2) + 0.20 * Conf
```
The margin between the top two candidates is evaluated:
```text
delta_Score = Score_top1 - Score_top2 >= 0.20
```
**If delta_Score < 0.20, the reacquisition is classified as AMBIGUOUS and immediately FAILS CLOSED (`None`, state `UNCERTAIN`).** Reacquisition only completes once the pedestrians separate and ambiguity clears.

---

## 6. Topic and API Compatibility

All existing topic interfaces and message contracts were strictly preserved:

1. `/tracking/select_target` (`std_msgs/Int32`):
   - Preserved identical type and semantics.
   - Values >= 0: designate target ID.
   - Value -1: explicitly clears target and transitions state machine to `NO_TARGET`.
   - Echo prevention implemented to prevent recursive selection loops when detector forwards track ID remapping.
2. `/tracking/error` (`geometry_msgs/Point`):
   - Preserved identical coordinate convention (416x416 frame center error and calibrated area).
   - Only published when state is `TRACKING`. When `UNCERTAIN` or `TARGET_LOST`, publishing is withheld.
3. `/tracking/motion_state` (`std_msgs/String`):
   - Format `STATE:TARGET_ID:FLIGHT_STATUS` preserved without disruption.
4. `/tracking/control_health` (`std_msgs/String`):
   - Heartbeat and watchdog monitors unchanged.
5. `/tracking/target_handle` (`std_msgs/String` — Additive Telemetry):
   - Additive JSON telemetry publisher providing real-time observability of the logical handle:
   ```json
   {
     "handle_id": "TARGET_001",
     "state": "TRACKING",
     "current_track_id": 15,
     "previous_track_ids": [7],
     "lock_mode": "MANUAL",
     "last_seen": 1789380.12,
     "confidence": 0.88,
     "velocity_2d": [12.4, -3.2],
     "reacquire_count": 1,
     "consecutive_detections": 14,
     "consecutive_misses": 0
   }
   ```

---

## 7. Verification and Test Results

### 7.1 Mandatory Unit Test Suite (`tests/test_target_identity.py`)
All 7 unit and regression test cases pass with 100% success rate:

```text
tests/test_target_identity.py::TestTargetIdentityPhase1::test_01_normal_lock PASSED [ 14%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_02_track_id_changes PASSED [ 28%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_03_distractor_with_larger_bounding_box PASSED [ 42%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_04_crossing_person PASSED [ 57%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_05_manual_clear PASSED [ 71%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_06_no_target PASSED [ 85%]
tests/test_target_identity.py::TestTargetIdentityPhase1::test_07_telemetry_serialization PASSED [100%]
============================== 7 passed in 0.02s ===============================
```

### 7.2 Node-Level Integration Lifecycle Test
Direct ROS 2 execution verified:
- Node startup: `TargetStateManager` initialized, `auto_track: false` declared and honored.
- Target selection: `Int32(7)` created `TARGET_001` with `current_track_id = 7`.
- Echo suppression: Repeated `Int32(7)` messages safely ignored.
- Target clear: `Int32(-1)` invalidated `target_handle`, reset state to `NO_TARGET`, published updated telemetry.

---

## 8. Known Limitations & Phase 2 Roadmap

1. **2D Kinematic & Geometric Gating Only:**
   Reacquisition currently relies on bounding box kinematics, scale, and aspect ratio. If two people of identical height, build, and apparel cross and move in parallel, 2D gating cannot distinguish them visually.
   *(Slated for Phase 2: Deep Re-ID feature embeddings via OSNet / FastReID).*
2. **Short-Term Occlusion Horizon:**
   The fail-closed reacquisition timeout is bounded to 3.5s to prevent stale trajectory extrapolation in dynamic environments. Long-term occlusions (> 4s) transition to `TARGET_LOST`, requiring operator re-selection or global search.
