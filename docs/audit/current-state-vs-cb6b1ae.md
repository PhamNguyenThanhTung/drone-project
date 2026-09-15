# Current Main Versus `cb6b1ae`

Comparison is based on read-only `git show`, `git diff`, and current working-tree source. `cb6b1ae` itself changed the detector and default world; the broader original 3-D/control implementation was introduced in `a1efcf3` and then refactored. The table therefore identifies both direct and architectural differences.

| Capability | `cb6b1ae` / legacy behavior | Current main / working tree | Assessment | Recommendation |
|---|---|---|---|---|
| Runtime architecture | ROS detector plus earlier direct control evolution; defaulted demo world to no-trees | ROS 2 launch, Gazebo Harmonic, PX4 SITL, packaged nodes | More integrated and testable | Keep architecture; document ownership |
| Tracker | YOLO tracking with single-person auto-lock changes | Ultralytics `bytetrack.yaml`, `persist=True`, manual/auto policy | Current has explicit short-term association | Benchmark before changing tracker |
| Forward tracking | `default_walk_speed` and visual servo with forward enable | Same control family, aliases restored; current source uses `default_walk_speed` | Largely retained, parameter duplication exists | Validate against target speed and logs |
| Walking speed | Approx. 0.85 m/s in old controller | 0.85 m/s source/config in current dirty tree; other historical YAML used 1.15 | Multiple values existed across revisions | Establish one calibrated source |
| Deadband | `deadband_x=20`, `deadband_y=25` | Current aliases resolve to 20/25 in unified config; older YAML had 25/30 | Behavior depends on selected YAML | Consolidate after regression baseline |
| Bbox-area distance proxy | Area was used as a practical stand-off signal in detector/arbiter contracts | `/tracking/error.z` remains area, but `target_area_*`/`kp_area` are not used in current velocity law | Contract survives; active regulation is unclear | Measure and either validate or retire explicitly |
| 3-D pinhole | Inline math in `vehicle_yaw_search.py`: hard-coded `fx=133.55`, `fy=178.07`, pitch 0.65, assumed height, clearance | `pinhole_geometry.py` module and arbiter integration in dirty tree; telemetry plus turn-point vector | Cleaner separation, but no camera-info contract and no association use | Validate calibration against ground truth |
| Tree clearance | Inline `tree_clearance_margin=1.8` added to forward vector before turn | Configured margin and pinhole output retained | Concept retained | Scenario-test before real flight |
| Lost target timeout | Legacy `lost_timeout=1.2 s`; direct search/yaw behavior | Arbiter default 5.0 s, vision freshness 0.4 s; recovery then `SEARCHING_HOLD` | Safer against stale commands, slower to abandon lock | Tune by scenario; do not silently shorten |
| Bottom-frame recovery | Reverse recovery and no-spin behavior existed in later legacy control | `BACKING_UP_TO_RECOVER`, bounded timeout/speed retained | Retained in refactored state machine | Benchmark occlusion/near-field cases |
| Turn point | Advance toward estimated vector, then rotate/search | `ADVANCING_TO_TURN_POINT` and `ROTATING_AT_TURN_POINT` use pinhole vector | Retained but now inside arbiter | Validate camera-motion failure cases |
| Yaw behavior | Legacy direct node sent BODY_NED setpoints and searched on loss | Current converts body commands to LOCAL_NED and sends yaw rate; final loss is hold, not indefinite spin | Frame/transport changed; search semantics intentionally reduced | Keep single-writer PX4 path; verify frame with logs |
| Lost-target recovery identity | No independent logical identity | Manual lock can remap by IoU/proximity for 5 s; auto mode can fall back to max area*confidence | Current is still not long-term identity | Add target handle/explicit rejection before Re-ID |
| Target selection | `cb6b1ae` focused on one visible person auto-lock | Normal config waits for HUD click/key; optional auto mode keeps ID or selects max area*confidence | Better operator control, but auto fallback can switch people | Make switching policy explicit and fail-closed |
| Target identity | Detector/track ID only | `target_id`, `manual_target_id`, `active_target_id` all carry ByteTrack IDs | No logical mission identity in either | Add contract in a later task |
| PX4 transport | Legacy default `tcp:127.0.0.1:5760`, ArduPilot GUIDED/BODY_NED | PX4 SITL UDP 14540, Offboard LOCAL_NED, global goto for HUD | Necessary platform migration | Keep unchanged during perception work |
| Setpoint rate | 0.1 s timer, with direct MAVLink sends | Steady-clock 10 Hz dispatch plus watchdog and one RX reader | More observable/robust under WSL timing | Preserve and benchmark |
| Manual override | Direct control path in old node | `MANUAL` state has priority; YOLO suppresses re-acquisition while manual | Current is safer and explicitly tested | Keep unchanged |
| Re-ID | Unfinished experiments were later removed | No executable Re-ID; `389689f` records removal | No regression-compatible implementation exists | Do not restore before baseline benchmark |

## What Was Lost Versus the Old Inline Controller

- The old direct `vehicle_yaw_search.py` is not the current runtime and used ArduPilot-specific GUIDED/takeoff and `tcp:127.0.0.1:5760`.
- Its inline pinhole/search code is no longer the source of truth; current geometry is factored into `pinhole_geometry.py` in the dirty working tree.
- The old loss behavior could rotate/search after timeout. Current code deliberately bounds recovery and ends in `SEARCHING_HOLD`/`STANDBY`, which reduces uncontrolled motion but can reduce reacquisition success.
- The old area and geometry concepts remain in contracts/config, but active use must be confirmed rather than inferred from parameter names.

## Evidence-Based Conclusion

Current main is a safer, more instrumented PX4/ROS system, but its target identity model is not stronger than a tracker ID plus short-term geometric rebinding. The major regression risk is not the absence of a particular tracker; it is that the controller receives no identity-bearing observation and automatic fallback can select a different person. Preserve PX4/safety behavior while validating geometry, target-handle semantics, and camera-motion retention in isolated benchmarks.
