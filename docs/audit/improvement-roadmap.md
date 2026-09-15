# Improvement Roadmap

This is a proposal only. No implementation is included in this audit.

| Problem | Priority | Evidence | Proposed fix | Risk | Dependencies | Test required |
|---|---|---|---|---|---|---|
| Automatic fallback can switch to another person after current track loss | P0 | Detector `select_target()` falls back to `max(area*confidence)`; no logical target handle | Make target switching fail-closed; retain mission target handle and require explicit operator/recovery confirmation | Wrong-person flight | Identity contract, target state policy | Multi-person crossing, disappearance, false-switch rate |
| `/tracking/error` drops ID, timestamp, confidence, and frame context | P0 | `Point(x,y,z)` only; arbiter cannot verify source person | Define a versioned observation contract in a later implementation task | Topic/message regression | Topic compatibility plan, consumers inventory | Contract tests and replay |
| Pinhole geometry is hard-coded and not validated at runtime | P0 | SDF-derived constants match numerically, but no `camera_info` or ground-truth check | Validate calibration against known actor positions; add acceptance gates before using distance for control | Incorrect range/turn vector | Camera pose, altitude, actor ground truth | 3-D error RMSE, bias by image row/pitch |
| SITL Offboard-loss tuning is unsafe if reused outdoors | P0 | Arbiter writes `COM_OF_LOSS_T=5.0` and marks it SITL-only | Separate simulation parameters from flight profile; require explicit real-vehicle configuration | Failsafe delay | PX4 safety review, field test plan | Parameter audit and loss-of-stream test |
| Severe retention under some camera motions | P1 | 0.0% nominal 180 and 3.0% right-lateral retention despite safety PASS | Benchmark GMC/tracker settings and camera-motion compensation before selecting a new tracker | Compute/latency increase | Reproducible scenarios, clean metrics | HOTA, IDF1, ID switches, retention by motion |
| Short-term IoU/proximity rebind can select a nearby person | P1 | `_reacquire_lock()` accepts IoU >= 0.15 or normalized proximity for 5 s, no appearance | Add ambiguity margin and reject state; use geometry/velocity gating first | Temporary loss/rejection | Target handle, candidate telemetry | False recovery and target-switch tests |
| Recovery motion continues while target is unseen | P1 | Backing and turn-point substates run before hold | Bound recovery by risk envelope and require geometric confidence/vehicle state gates | Collision or drift | Calibrated geometry, obstacle policy | Tree/occlusion/exit-FOV scenarios |
| Auto mode and arbiter can disagree on target ID | P1 | Arbiter sets `active_target_id=0` on first auto vision message; detector may use another ByteTrack ID | Make selection acknowledgement and target handle explicit | Misleading state/HUD and recovery | Observation contract | ID consistency replay |
| Area-distance parameters appear stale/unreferenced | P1 | `target_area_min`, `target_area_max`, `kp_area`, `small_box_max_speed` are declared but absent from active law | Decide whether area is retained, validated, or removed in a scoped change | Behavior change if assumed active | Baseline logs and controller owner | Distance hold and approach tests |
| Configuration has multiple sources and aliases | P2 | `tracking_stack.yaml` duplicates per-node YAML; source has alias parameters | Establish one generated/validated source of truth without changing values first | Parameter drift | Parameter inventory | Startup resolved-parameter snapshot |
| `/tracking/select_target` has two publishers and feedback semantics | P2 | HUD and detector both publish; YOLO subscribes to its own output | Separate command from acknowledgement in a future interface revision | Operator lock regression | Topic compatibility decision | ROS graph and command-loop test |
| Root shims/build/install pollute linter results | P2 | 433 flake8 and 72 pep257 failures include generated files/shims | Narrow test scope and add focused source linting; preserve shims | CI behavior changes | Package/test ownership | Clean package-only lint run |
| Geometry topics have no production consumer | P2 | `/tracking/target_geometry` and `/tracking/ground_distance` only published | Either document telemetry ownership or consume them in a measured recovery module | Dead interface | Consumer decision | Topic usage audit |
| No clean stage-level performance baseline | P3 | Existing logs have conflicting control gaps and no current YOLO/tracker FPS/VRAM | Add deterministic replay/profiling harness | Benchmark overhead | Stable scenario and resource sampler | FPS, latency, CPU, GPU, VRAM, drops |
| No UAV-specific identity benchmark in repository | P3 | Re-ID absent; conventional metrics not connected to drone scenarios | Build a dataset/replay matrix using VisDrone/UAVDT-like viewpoint changes and project scenes | Dataset mismatch | Annotation/replay plan | Rank-1/mAP plus end-to-end target metrics |

## Recommended Phases

### Phase 0: Baseline and audit closure

Freeze current topics, messages, tracker, controller, and safety parameters. Capture clean replayable logs with camera rate, detector latency, tracker latency, control period, CPU/GPU/VRAM, target retention, and target-switch events. Separate safety PASS from perception PASS.

### Phase 1: Contract and source-of-truth validation

Document the resolved launch parameters and topic contracts. Confirm that `/tracking/error` area semantics, pinhole calibration, frame conventions, and target IDs are understood by every consumer. Do not change values yet.

### Phase 2: Target handle and lock policy

Introduce a logical mission target handle in a future implementation task, with explicit `UNKNOWN`, `RECOVERY`, `REJECTED`, and `STANDBY` behavior. Prevent automatic replacement after timeout. Keep manual override priority.

### Phase 3: Geometry and recovery validation

Use known actor ground truth to validate pinhole range and turn vectors. Add 2-D position, velocity, 3-D plausibility, and trajectory-consistency gates to recovery evaluation. Keep geometry out of association until its error bounds are measured.

### Phase 4: Tracker/camera-motion benchmark

Compare current ByteTrack against ByteTrack with motion prediction/GMC and candidates such as BoT-SORT, OC-SORT, and StrongSORT on the same UAV scenarios. Select by IDF1/HOTA, ID switches, retention, latency, and safety, not by detector FPS alone.

### Phase 5: No-Re-ID end-to-end benchmark

Evaluate tracker-only, tracker+motion, tracker+lock, and tracker+lock+motion+3-D gating. Include 0.2-0.5 s occlusion, 1-2 s occlusion, tree obstruction, crossing, FOV exit/re-entry, sharp drone turns, similar clothing, and sudden target turns.

### Phase 6: On-demand Re-ID only if required

If long-gap/re-entry and crowded-scene false recovery remain above the safety threshold, add appearance embedding inference only in `RECOVERY/REID_VERIFY`, with a confidence margin and no automatic switch on ambiguous scores. Benchmark Rank-1/mAP separately from end-to-end target recovery.

### Phase 7: Real-camera validation

Repeat the selected architecture with calibrated camera intrinsics, real motion blur/exposure, safety pilot, short Offboard-loss timeout, and explicit no-target behavior. SITL results are not a substitute for this gate.

## Do Not Change During Audit Follow-Up

Keep the sole `motion_arbiter` MAVLink ownership, 10 Hz Offboard stream, manual override priority, existing topic/message names, compatibility shims, and scenario fixtures until the baseline and regression tests are complete.
