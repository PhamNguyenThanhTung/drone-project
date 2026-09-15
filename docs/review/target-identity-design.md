# Target Identity Model & Selection Architecture Design

**Status:** Proposed Architecture Blueprint (No Implementation in This Phase)  
**Target Module:** `yolo_detector_node` / Target Manager Subsystem  
**Scope:** Decoupling Detection, Short-Term Tracking, and Logical Mission Identity

---

## 1. Architectural Distinction: Detection vs. Track vs. Mission Handle

A fundamental flaw in the existing implementation is treating ByteTrack's ephemeral integer `track_id` as the human being's persistent mission identity.

| Identifier Layer | Scope | Lifespan | Source | Properties & Responsibilities |
|---|---|---|---|---|
| **`detection_index`** | Frame-local | Single video frame | YOLO detector output tensor | Raw list index `[0, 1, ..., N-1]` in a single inference pass. Completely unordered between frames. Must never be exposed outside the detector. |
| **`track_id`** | Temporal local | Seconds to minutes (continuous visual tracking) | ByteTrack / Multi-Object Tracker | Integer assigned by Hungarian association on Kalman filter predictions. Resets or increments whenever visual tracking is broken by occlusion or sharp rotation. |
| **`target_handle`** | Mission persistent | Entire flight / mission duration | **Target State Manager (Proposed)** | Logical identity assigned to the human being that the drone is instructed to follow (e.g. `TARGET_ALPHA` or `HANDLE_001`). Persists across track losses, occlusions, and `track_id` changes. |

---

## 2. Specification of `target_handle`

```python
@dataclass
class TargetHandle:
    handle_id: str                      # Unique mission identity (e.g. "TARGET_001")
    state: str                          # State: INIT, TRACKING, UNCERTAIN, RECOVERY, REID_VERIFY, LOST
    created_at_wall: float              # Creation timestamp (seconds)
    last_seen_wall: float               # Last verified observation timestamp (seconds)
    
    # Association Lineage
    current_track_id: Optional[int]     # Active ByteTrack track_id in current frame
    previous_track_ids: List[int]       # Historical track_ids associated with this person
    
    # 2D Perception State
    bbox_xyxy: Tuple[float, float, float, float]  # Smoothed bounding box in pixel space
    confidence: float                   # Detection confidence [0.0, 1.0]
    velocity_2d_px_s: Tuple[float, float]          # Bbox center velocity in pixels/sec
    covariance_2d: np.ndarray           # 4x4 or 6x6 Kalman covariance matrix
    
    # 3D Ground & Metric State
    ground_pos_m: Tuple[float, float]   # Estimated (dx, dy) relative to drone body
    ground_dist_m: float                # Euclidean ground standoff distance
    altitude_agl_m: float               # Vehicle altitude at time of estimation
    
    # Appearance & Quality Metrics
    tracking_quality: float             # Composite confidence score [0.0, 1.0]
    appearance_embedding: Optional[np.ndarray] # 512-d normalized L2 feature vector (if Re-ID enabled)
    embedding_timestamp: Optional[float]
    
    # Gating & Safety Bounds
    gate_radius_m: float                # Dynamic 3D search radius based on time lost
```

---

## 3. Seven Core Identity Governance Rules

### Rule 1: When is `target_handle` created?
- **Manual Mode (Default):** Created immediately upon receipt of an explicit operator selection (HUD mouse click on bounding box, or numeric key `1`–`9` matching candidate list).
- **Auto Mode:** Created ONLY if no active mission target exists, and a candidate satisfies the **Initial Acquisition Gate** (candidate confidence $\ge 0.65$, duration persistence $\ge 0.5\text{ s}$ across 5 consecutive frames, aspect ratio $1.2 \le h/w \le 3.5$, distance within $2.5\text{m} \le d \le 8.0\text{m}$).
- **Invariance:** Only **ONE active mission `target_handle`** can exist at any time.

### Rule 2: Who owns the `target_handle`?
- **Perception Node (`TargetStateManager` inside `yolo_detector_node`):** Owns candidate association, state transitions, trajectory extrapolation, and identity verification.
- **Flight Controller (`motion_arbiter_node`):** Subscribes to the active `target_handle` state and errors. The flight controller CANNOT reassign the identity; it merely executes flight commands matching the handle's state.

### Rule 3: How is a `track_id` switch handled?
When ByteTrack drops `track_id=7` and discovers `track_id=12`:
1. `TargetStateManager` checks if `track_id=12` falls within the **Gated Spatio-Temporal Envelope** of `target_handle`:
   - 2D Center Distance: $\Delta x_{center} < 1.5 \times w_{last}$ and $\Delta y_{center} < 1.5 \times h_{last}$.
   - Velocity Alignment: Angle between predicted velocity vector and candidate displacement vector $\le 45^\circ$.
   - Scale Invariance: Box area difference $|A_{new} - A_{last}| / A_{last} \le 0.40$.
2. If gates pass with no competing candidates: `target_handle.previous_track_ids.append(7)`, `target_handle.current_track_id = 12`.
3. If multiple candidates pass (crossing ambiguity): Transition to `UNCERTAIN` and trigger `REID_VERIFY` or hold position.

### Rule 4: How long after target loss before state transitions occur?
- **$t \le 0.40\text{ s}$:** State remains `TRACKING`. Kalman filter predicts position.
- **$0.40\text{ s} < t \le 1.50\text{ s}$:** State transitions to `UNCERTAIN`. Slew rate limiters smooth deceleration; UAV stays in hover/slow drift.
- **$1.50\text{ s} < t \le 4.00\text{ s}$:** State transitions to `RECOVERY` (`BACKING_UP_TO_RECOVER` or `ADVANCING_TO_TURN_POINT`).
- **$t > 5.00\text{ s}$:** State transitions to `LOST`. Arbiter enters `STANDBY` hover.

### Rule 5: When can the target be reacquired?
- When a candidate detection satisfies the **Composite Reacquisition Gate**:
  $$\text{Gate Score} = 0.40 \cdot \text{SpatialGate} + 0.30 \cdot \text{VelocityGate} + 0.30 \cdot \text{AppearanceGate} \ge 0.75$$
- The candidate must have an unambiguous margin over the next-best candidate ($\text{Score}_1 - \text{Score}_2 \ge 0.25$).
- Reacquisition is strictly forbidden if the candidate appears in a location that implies physically impossible human speed ($V_{human} > 4.5\text{ m/s}$).

### Rule 6: When is it absolutely forbidden to switch targets?
1. **During Active Tracking:** As long as `state == TRACKING`, the system must NEVER switch to another candidate, regardless of candidate box size or confidence.
2. **During Crossing / Multi-Person Congestion:** When another person is within 2.0m of the target trajectory, automatic re-binding is blocked until the cluster separates.
3. **During Active Manual Flight Override (`STATE_MANUAL`):** Automatic reacquisition is suppressed.
4. **On High Uncertainty:** If candidate similarity is marginal ($0.45 \le \text{Score} < 0.75$), switching is prohibited; UAV must hover and request operator input.

### Rule 7: How does manual clear work?
- Operator presses `0` or `SPACE` on HUD (or publishes `-1` on `/tracking/select_target`).
- `TargetStateManager` immediately sets `target_handle = None`, resets historical buffers, and sends `STANDBY` state.
- Perception enters idle standby. The drone halts motion and hovers in place.

---

## 4. Target Selection Architecture & Candidate Scoring

### 4.1 Manual Selection (Highest Authority)
- Click resolution considers all raw detections (`all_persons`), not just shape-filtered boxes.
- Click coordinates $(x_{click}, y_{click})$ must land within the candidate bounding box (or within an expanded 15-pixel margin).
- Upon click, the selected candidate instantly creates a new `TargetHandle`.

### 4.2 Automated Selection Policy (Replaces Instantaneous Area)
Instantaneous bounding box area ($w \times h$) must **NEVER** autonomously dictate mission identity. When running in autonomous initial search mode, the candidate selection score is formulated as:

$$\mathcal{S}_{cand} = w_c \cdot c_{det} + w_p \cdot P_{pers} + w_a \cdot A_{norm} + w_d \cdot D_{standoff} + w_s \cdot S_{shape}$$

Where:
- $c_{det}$: YOLO detection confidence (Gate: $c_{det} \ge 0.50$).
- $P_{pers}$: Temporal persistence ($N_{consecutive} / N_{window}$, Gate: $\ge 0.60$ over 10 frames).
- $A_{norm}$: Normalized area relative to nominal 4.5m standoff ($A_{cand} / A_{ideal}$).
- $D_{standoff}$: Closeness of estimated pinhole distance to nominal $4.5\text{m}$.
- $S_{shape}$: Aspect ratio penalty function (peaks at human aspect ratio $h/w \approx 2.5$).

**Selection Margin Gate:**
A candidate is accepted if:
$$\mathcal{S}_1 \ge 0.70 \quad \text{AND} \quad \mathcal{S}_1 - \mathcal{S}_2 \ge 0.20$$
If the top two candidates have close scores ($\Delta \mathcal{S} < 0.20$), automatic selection is deferred to prevent chattering between targets.

### 4.3 Future Reference-Guided Selection (One-Shot Search)
For mission search-and-rescue:
1. Operator loads a reference photo of the target individual before flight.
2. Feature extractor generates reference embedding $\mathbf{f}_{ref}$.
3. When the drone flies an autonomous sweep pattern, detections are compared against $\mathbf{f}_{ref}$.
4. A target handle is instantiated only when $\cos(\mathbf{f}_{cand}, \mathbf{f}_{ref}) \ge 0.80$ across multiple viewpoints.

---

## 5. Proposed ROS 2 Interface (Backward Compatible)

To ensure zero disruption to current topics during Phase 0 and Phase 1, the new target handle information will be exposed via non-breaking side topics, with `/tracking/error` preserved for flight control:

```
Existing Topics (Preserved Unchanged):
/camera/image_raw           [sensor_msgs/Image]
/tracking/error             [geometry_msgs/Point: x, y, area]  <-- Controller continues reading this
/tracking/select_target     [std_msgs/Int32: ID / -1]
/tracking/motion_state      [std_msgs/String]
/tracking/debug_image       [sensor_msgs/Image]

Proposed Observation & Identity Topics (Add in Phase 2):
/tracking/target_handle     [std_msgs/String (JSON schema)]
    {
      "handle_id": "TARGET_001",
      "track_id": 12,
      "state": "TRACKING",
      "confidence": 0.88,
      "ground_dist_m": 4.52,
      "dx_m": 4.15,
      "dy_m": 1.78,
      "last_seen_s": 1789374000.12
    }
/tracking/candidates        [std_msgs/String (JSON schema: list of active candidates & scores)]
```
