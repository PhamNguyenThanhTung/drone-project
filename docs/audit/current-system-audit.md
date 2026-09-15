# Current System Audit

Audit date: 2026-09-14  
Repository: `/home/tungt/drone-project`  
Scope: read-only audit. No production source, configuration, topic, message, tracker, controller, or architecture files were changed by this audit.

## 1. Repository State

| Item | Observed value |
|---|---|
| Branch | `refactor/restore-3d-pinhole-tracking` |
| HEAD | `4b156b99823cfd4f88ddc79472e69fd16a1cb1b9` (`fix: stabilize tracking control and runtime timing`) |
| `main` / `origin/main` | Point to the same commit as HEAD |
| Remote | `https://github.com/PhamNguyenThanhTung/drone-project.git` |
| Worktree | Dirty before audit: three modified production files, one untracked pinhole module, and pre-existing `docs/` changes |

Relevant history inspected with `git show`, `git diff`, and `git log`:

- `a1efcf3`: original ROS 2 YOLO tracking, inline 3-D pinhole math, tree-clearance behavior, and `vehicle_yaw_search.py`.
- `cb6b1ae`: single-person auto-lock and cleaned HUD.
- `2f360be`: workspace rebuild on stack start and vendored `yolov8n.pt`.
- `389689f`: PX4 test organization and removal of unfinished Re-ID modules.
- Current sequence: `3d0e1fd` stabilized detector locking/ID-free HUD labels; `429b847` and `4b156b9` stabilized control, timing, and safety instrumentation.

## 2. Components Currently Implemented

| File/group | Role | Runtime classification | Status |
|---|---|---|---|
| `start_stack.sh` | Builds workspace, starts Gazebo/PX4/ROS launch, readiness checks, cleanup | Production entry point | Active |
| `ros2_ws/src/vision_tracking/vision_tracking/yolo_detector_node.py` | YOLOv8 detection, Ultralytics ByteTrack, target policy, error/debug output | Production ROS node | Active; target identity is track-ID based |
| `.../motion_arbiter_node.py` | PX4 MAVLink connection, takeoff, Offboard setpoints, teleop arbitration, tracking control, watchdog | Production ROS node and sole normal MAVLink sender | Active |
| `.../pinhole_geometry.py` | Calibrated 2-D-to-ground geometry and turn-point vector | Current uncommitted production-side module | Present but not an identity association mechanism |
| `.../live_camera_hud_node.py` | OpenCV HUD, click-to-select, keyboard lock/standby, teleop, GPS minimap | Production ROS node | Active when `SHOW_HUD=1` |
| `.../sim_realism_node.py` | Optional image delay/drop/blur and sensor noise/dropout | Optional production ROS node | Packaged; disabled by default |
| `tracking_stack.launch.py` | Launch orchestration and parameter overrides | Production launch | Active |
| `tracking_stack.yaml` | Unified parameters for all four nodes | Production config | Active; duplicates `motion_arbiter.yaml` and other YAMLs |
| `yolo_detector.yaml`, `motion_arbiter.yaml`, `live_camera_hud.yaml`, `sim_realism.yaml` | Per-node configs | Installed/config compatibility | Potential alternate sources of truth |
| root `motion_arbiter.py`, `live_camera_hud.py` | Import-and-run compatibility shims | Test/legacy compatibility | Not the normal launch implementation |
| `tests/px4/*.py` | Live SITL, MAVLink, camera, turn, long-path, and regression scripts | Integration/live tests | Require external PX4/Gazebo; not ordinary unit tests |
| `ros2_ws/src/vision_tracking/test/*.py` | ament copyright/flake8/pep257 checks | Package tests | Present; style tests currently fail |
| `scripts/*.py`, `scripts/*.sh` | Benchmarks, latency, process sampling, setup, regression helpers | Tooling | Active/diagnostic, not runtime nodes |
| `gazebo/models/*.sdf`, `gazebo/worlds/*.sdf` | Camera, vehicle, pedestrian, path, and obstacle simulation | Simulation assets | Active through `start_stack.sh` |
| `yolov8n.pt` | Vendored detector weights | Runtime asset | Active |
| Re-ID modules | OSNet/FastReID/gallery/embedding implementation | None | Removed/missing; only documentation claims remain |

The broad inventory also includes generated `ros2_ws/build`, `ros2_ws/install`, caches, and bytecode. They are build artifacts, not independent production implementations.

### File-Level Inventory

| File | Role / entry point | Used by | I/O summary | Classification |
|---|---|---|---|---|
| `start_stack.sh` | Shell entry point | Operator | Starts Gazebo, PX4, ROS launch; readiness/cleanup | Production |
| `motion_arbiter.py` | Compatibility `main()` shim | Legacy PX4 tests/imports | Delegates to packaged arbiter | Compatibility |
| `live_camera_hud.py` | Compatibility `main()` shim | Legacy tests/imports | Delegates to packaged HUD | Compatibility |
| `yolov8n.pt` | YOLO weights | Detector | Image -> person boxes | Runtime asset |
| `ros2_ws/src/vision_tracking/package.xml` | ROS package manifest | Colcon | Declares dependencies | Build |
| `ros2_ws/src/vision_tracking/setup.py` | ament-python packaging/entry points | Colcon/ROS launch | Installs nodes/config/launch | Build |
| `ros2_ws/src/vision_tracking/setup.cfg` | Setuptools install paths | Colcon | Script destination | Build |
| `ros2_ws/src/vision_tracking/resource/vision_tracking` | Ament resource marker | ROS package index | Package discovery | Build |
| `.../launch/tracking_stack.launch.py` | Launch description | `start_stack.sh` | Starts bridge, realism, detector, arbiter, HUD | Production |
| `.../vision_tracking/yolo_detector_node.py` | Detector/tracker/selection node | Launch | Image + commands -> error/debug/select | Production |
| `.../vision_tracking/motion_arbiter_node.py` | Flight controller/state machine | Launch | ROS commands -> MAVLink + telemetry | Production |
| `.../vision_tracking/pinhole_geometry.py` | Pinhole estimator | Arbiter | Pixel/altitude/pitch -> ground geometry | Current working-tree module |
| `.../vision_tracking/live_camera_hud_node.py` | OpenCV HUD/operator interface | Launch | Debug/GPS/state -> commands | Production |
| `.../vision_tracking/sim_realism_node.py` | Delay/drop/noise/blur injection | Optional launch | Sensor input -> degraded topics | Optional production |
| `config/tracking_stack.yaml` | Unified ROS parameters | Launch | Parameters for all nodes | Config; source-of-truth debt |
| `config/yolo_detector.yaml` | Detector parameters | Manual/alternate launch | Detector settings | Config/duplicate |
| `config/motion_arbiter.yaml` | Arbiter parameters | Manual/alternate launch | Controller settings | Config/duplicate |
| `config/live_camera_hud.yaml` | HUD parameters | Manual/alternate launch | UI settings | Config/duplicate |
| `config/sim_realism.yaml` | Realism parameters | Manual/alternate launch | Fault model settings | Config/duplicate |
| `gazebo/models/x500/model.sdf` | Vehicle/camera model | Gazebo world | Camera/vehicle sensors | Simulation |
| `gazebo/models/pedestrian_blue/model.sdf` | Pedestrian model | Gazebo worlds | Actor geometry/appearance | Simulation |
| `gazebo/worlds/person_tracking_path.sdf` | Default path world | `start_stack.sh` | Repeating actor/path/obstacles | Simulation |
| `gazebo/worlds/person_tracking_approach.sdf` | Approach scenario | Tests | Close-in scenario | Simulation |
| `gazebo/worlds/person_tracking_long_path.sdf` | Long path scenario | Tests | Multi-turn path | Simulation |
| `gazebo/worlds/person_tracking_no_trees.sdf` | No-tree scenario | Historical/demo use | Simplified path | Simulation/legacy |
| `scripts/apply_px4_patch.sh` | PX4 SITL patch helper | `start_stack.sh` | Applies project-local patch best effort | Tooling |
| `scripts/benchmark_suite.py` | Multi-trial benchmark driver | Operator/CI | Publishes commands and records trials | Test/tooling |
| `scripts/command_latency_micro_test.py` | Command latency probe | Operator | HUD -> callback -> MAVLink timing | Test/tooling |
| `scripts/phase3_process_sampler.py` | CPU/process sampler | Operator | Process/resource samples | Test/tooling |
| `scripts/run_live_regression.sh` | Live regression wrapper | Operator | Launches and samples stack | Test/tooling |
| `scripts/run_postfix_trials.py` | Post-fix trial runner | Operator | Scenario execution and CSV output | Test/tooling |
| `scripts/setup_environment.sh` | Environment setup | Operator | Dependency/environment preparation | Tooling |
| `simulation/inject_failure.py` | Failure injection helper | Simulation tests | Fault scenario control | Test/tooling |
| `simulation/realism.yaml` | Simulation fault profile | Realism tooling | Delay/drop/noise values | Config/tooling |
| `simulation/vehicle_profile.yaml` | Vehicle simulation profile | Simulation tooling | Vehicle settings | Config/tooling |
| `test_repro_autoland_and_takeoff.py` | Takeoff/autoland reproduction | Operator | Live PX4 process/network | Live test |
| `tests/px4/*.py` | PX4, MAVLink, camera, state, long-path, live flight scripts | Operator | SITL/ROS/MAVLink integration | Live/integration tests |
| `tests/px4/README.md` | Test scope and safety guidance | Developers | Commands and interpretation | Documentation |
| `ros2_ws/src/vision_tracking/test/*.py` | ament lint tests | Pytest/colcon | Copyright, flake8, pep257 | Package tests |
| `docs/research/*.md` | Tracking/selection/Re-ID research | Developers | Architecture and benchmark proposals | Documentation only |
| `logs/*` | Trial CSV/JSONL summaries | Benchmark scripts | Performance/safety evidence | Runtime artifacts |

Files under `ros2_ws/build/`, `ros2_ws/install/`, `__pycache__/`, and `.pytest_cache/` are generated artifacts and are not separate implementations.

## 3. Runtime Pipeline

`start_stack.sh` performs: source ROS 2 Humble -> best-effort PX4 patch -> `colcon build --symlink-install` -> source `ros2_ws/install/setup.bash` -> set Gazebo/PX4 resource and model variables -> kill old stack processes -> start Gazebo server -> start PX4 SITL (`make px4_sitl gz_x500_flow`) -> optional QGC UDP 14550 endpoint -> launch `tracking_stack.launch.py` -> wait for `/clock`, `/camera/image_raw`, `/tracking/error`, and `/tracking/control_health`.

Actual data path:

```text
Gazebo camera (640x480, 30 Hz, HFOV 2.0 rad, pitch 0.65 rad)
  -> ros_gz_bridge: /camera/image_raw (sensor_msgs/msg/Image)
  -> optional sim_realism: /simulation/camera/image
  -> yolo_detector_node: YOLOv8n + Ultralytics ByteTrack
  -> /tracking/error (geometry_msgs/msg/Point: x error, y error, z box area)
  -> motion_arbiter (visual servo, pinhole telemetry, state machine)
  -> MAVLink LOCAL_NED velocity/yaw-rate setpoint at nominal 10 Hz
  -> PX4 Offboard -> vehicle motion -> next camera frame
```

The HUD is a side channel: `/tracking/debug_image` feeds the viewer; click/key commands feed `/tracking/click_point`, `/tracking/select_target`, `/teleop/cmd_vel`, `/teleop/flight_action`, and `/tracking/goto_gps`.

### Runtime Recording Table

| Step | Actual command/process | Topic/message | Nominal rate | Frame/time behavior | Key parameters |
|---|---|---|---:|---|---|
| Gazebo | `gz sim -r -s gazebo/worlds/${WORLD_NAME}.sdf` | Gazebo transport camera, `/clock` | Camera 30 Hz | SDF camera frame; simulation clock | `WORLD_NAME=person_tracking_path`, `GZ_VERSION=harmonic` |
| PX4 | `(sleep infinity | (cd $PX4_DIR && make px4_sitl $PX4_GZ_TARGET))` | MAVLink UDP 14540; optional GCS UDP 14550 | Telemetry requested 10 Hz | PX4 timestamps; vehicle local/global frames | `PX4_SIM_MODEL=x500_flow`, `PX4_GZ_TARGET=gz_x500_flow` |
| Bridge | `ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image` | `/camera/image_raw` `sensor_msgs/Image` | Up to 30 Hz | Gazebo image header; no added target timestamp | `use_bridge=true` |
| Realism (optional) | ROS executable `sim_realism_node` | `/simulation/camera/image` | Input-rate dependent | Queued delay/drop/blur; sensor headers copied | `use_realism`, `camera_delay_ms`, drop/blur params |
| Detector | ROS entry point `yolo_detector_node` | `/tracking/error`, `/tracking/debug_image` | One per processed image | Native 640x480 inference, output scaled to 416x416; no frame ID in Point | `yolov8n.pt`, `bytetrack.yaml`, `conf=.45`, `iou=.45`, device override |
| Target command | HUD and detector callbacks | `/tracking/select_target` `Int32` | Event-driven | ByteTrack integer ID only | click/key or auto fallback |
| Arbiter | ROS entry point `motion_arbiter_node` (`motion_arbiter`) | `/tracking/error` in; MAVLink out | Control timer 10 Hz | Steady-clock dispatch; LOCAL_NED after yaw conversion | `control_period_s=.10`, limits/deadbands |
| PX4 setpoint | `set_position_target_local_ned_send` | MAVLink `SET_POSITION_TARGET_LOCAL_NED` | Nominal 10 Hz | Position Z + velocity XYZ + yaw rate; type mask `0x05C3` | `MAV_FRAME_LOCAL_NED`, UDP 14540 |
| Telemetry/UI | Arbiter publishers and HUD subscriptions | `/tracking/motion_state`, `/tracking/gps`, `/tracking/control_health` | State events, GPS per tick, health ~2 Hz | State string embeds target ID/armed status; health is JSON | watchdog and stale-timeout parameters |

## 4. ROS 2 Nodes and Topics

All ordinary ROS publishers/subscribers use depth 10, reliable default QoS unless noted. The camera subscription deliberately uses KEEP_LAST depth 1 and BEST_EFFORT. The debug image publisher uses depth 2.

| Topic | Publisher(s) | Subscriber(s) | Type and semantics | Rate/status |
|---|---|---|---|---|
| `/camera/image_raw` | `ros_gz_bridge` | YOLO, and realism when enabled | `sensor_msgs/Image`, Gazebo camera | Up to 30 Hz; actual detector rate is compute-limited |
| `/simulation/camera/image` | `sim_realism_node` | YOLO when `use_realism=true` | Delayed/dropped/blurred image | Optional |
| `/tracking/error` | YOLO | `motion_arbiter` | `Point.x/y` = signed 416x416 pixel offset; `.z` = scaled bbox area | One per processed frame |
| `/tracking/debug_image` | YOLO | HUD | Annotated BGR image; labels intentionally omit numeric IDs | One per processed frame when enabled |
| `/tracking/click_point` | HUD | YOLO | `Point.x/y` click coordinates in detector image space | Event-driven |
| `/tracking/select_target` | HUD and YOLO re-acquisition | YOLO and arbiter | `Int32`: nonnegative ByteTrack ID locks; negative clears/standby | Event-driven; two publishers share one command topic |
| `/tracking/motion_state` | Arbiter | YOLO and HUD | `String` `STATE:target_id:ARMED|DISARMED` | On state/target transitions |
| `/tracking/goto_gps` | HUD | Arbiter | `Point` x=latitude, y=longitude, z=altitude | Event-driven |
| `/teleop/cmd_vel` | HUD | Arbiter | `Twist` manual velocity/yaw override | Event-driven/keyboard repeat |
| `/teleop/flight_action` | HUD | Arbiter | `String` TAKEOFF/LAND | Event-driven |
| `/tracking/gps` | Arbiter | HUD | `NavSatFix` from cached PX4 global position | Normally each control tick while valid |
| `/tracking/control_health` | Arbiter | Startup checks/diagnostic consumers | JSON `String`: heartbeat/position age, control period, Offboard, watchdog, state | About 2 Hz |
| `/tracking/target_geometry` | Arbiter | No production subscriber found | JSON `String` from pinhole geometry | One per vision message; telemetry-only |
| `/tracking/ground_distance` | Arbiter | No production subscriber found | `Point`: x forward, y lateral, z Euclidean ground distance | One per vision message; telemetry-only |
| `/simulation/imu`, `/simulation/gps`, `/simulation/baro` | Realism only if input parameters are configured | No current runtime consumer | Optional noisy sensor outputs | Disabled by empty input defaults |

Findings: there is no published detection list, per-person geometry, appearance embedding, or logical target handle. `/tracking/select_target` has intentionally coupled command/feedback semantics, and the four YAML files plus unified YAML duplicate parameter declarations. No topic type mismatch was found in source; unused geometry topics are architecture debt rather than broken connections.

## 5. Detection

- Ultralytics YOLOv8 with vendored `yolov8n.pt`; class filter `[0]` (COCO person).
- Defaults: confidence `0.45`, IoU `0.45`; `start_stack.sh` overrides device to `cuda:0` by default after checking `torch.cuda.is_available()`; YAML/source default is CPU.
- Native camera inference is enabled. For a 640x480 frame, `imgsz` becomes 640; boxes are independently scaled to the 416x416 error coordinate space (`sx=0.65`, `sy=0.8667`).
- Automatic candidate filters: area ratio 0.0003 to 0.85, aspect ratio 0.70 to 4.80, no bottom margin. The currently locked person is exempt from area/aspect gates, but not image-bottom exclusion.
- Output contract is exactly `Point(error_x, error_y, scaled_bbox_area)`. It does not contain confidence, width, height, depth, timestamp, frame ID, or track ID.
- Detection latency is logged internally, but no clean current live benchmark was possible because the stack was not running and the ROS 2 CLI failed with `PermissionError: [Errno 1] Operation not permitted`.

## 6. Tracking

The actual tracker is Ultralytics built-in **ByteTrack** selected by `tracker: bytetrack.yaml`, called through `YOLO.track(..., persist=True)`. There is no DeepSORT, BoT-SORT, StrongSORT, OC-SORT, camera-motion compensation, or appearance/Re-ID association in the current project.

ByteTrack supplies a frame-to-frame `track_id`. The detector keeps an internal `target_id`, smooths the selected box with EMA (`alpha=0.60`), and counts switches. ByteTrack persistence is therefore short-term association, not guaranteed long-term identity. The project does not expose ByteTrack buffer/track activation/matching thresholds in its own YAML; they live in the Ultralytics tracker preset.

Current behavior is appropriate for low-density, short-gap tracking but vulnerable to camera turns, crossings, similar clothing, and target re-entry. Existing logs demonstrate the limitation: target retention was 92.2% in one fast-turn trial, 87.3% left-lateral, 88.6% close-in, but only 3.0% right-lateral and 0.0% nominal 180-turn. Those trials were safety passes, not universal tracking passes.

## 7. Target Selection

Initial selection is separate from identity recovery:

- Normal configuration has `auto_track=false`; the operator clicks a box in the HUD or uses number keys. The HUD publishes an ID, and the detector locks `manual_target_id` to that current ByteTrack ID.
- Detector click resolution considers every detected person (`all_persons`), including boxes rejected by automatic shape filters.
- If automatic mode is used, an already visible `target_id` is retained; otherwise the candidate maximizing `area * confidence` is selected.
- `motion_arbiter` can also receive a positive `/tracking/select_target` and enter `TRACKING`, or a negative value and enter `STANDBY`.
- When a manual lock's ByteTrack ID disappears, `_reacquire_lock()` can bind to another current box using IoU/proximity for up to five seconds. This is geometric re-binding, not Re-ID.
- There is no explicit UNKNOWN/REJECT state, candidate score publication, or operator confirmation step after an ambiguous recovery.

Consequently, "select initial target" means choosing a visible candidate according to click/key or `area*confidence`; "re-identify after loss" means proving that a later candidate is the same person. The current system implements the former and only a weak short-term geometric approximation of the latter.

## 8. Target Identity

Identity fields found:

| Field | Meaning | Assessment |
|---|---|---|
| `detection_index` | Implicit list position in a YOLO result | Not stable and not published |
| `track_id` | ByteTrack per-track integer | Short-term tracker ID only |
| `target_id` | Detector variable carrying selected `track_id` | Not an independent identity |
| `manual_target_id` | Detector lock carrying selected `track_id` | Can be remapped by IoU/proximity |
| `active_target_id` | Arbiter copy used in state/telemetry | Can be stale or absent in auto mode |
| `mission_target_id` / `target_handle` | Logical mission identity | **MISSING** |

Risk: after a track disappears, auto mode can select a different candidate using `area*confidence`; the controller does not receive a person-specific observation ID in `/tracking/error`. In tests with `auto_track=true`, the arbiter sets `active_target_id=0` on first vision input regardless of the detector's actual ByteTrack ID. This creates a label/identity mismatch even though motion control still uses the latest aggregate error.

## 9. Distance / 3-D Geometry

The current working tree contains `pinhole_geometry.py` and arbiter integration (pre-existing user changes). It estimates forward/lateral ground displacement and distance from pixel error, altitude, vehicle pitch, camera pitch, focal lengths, and assumed person height; publishes JSON geometry and `ground_distance`, and supplies a turn-point vector for recovery.

Calibration is internally consistent with `x500/model.sdf`: 640x480, HFOV 2.0 rad gives native `fx~205.47`; after nonuniform scaling to 416x416, `fx~133.55`, `fy~178.07`, matching config. There is no `camera_info` topic or runtime calibration validation, so this is a hard-coded model contract.

Important limitation: normal tracking still uses `/tracking/error` pixel error and area; `target_area_min`, `target_area_max`, and `kp_area` are declared/configured but not referenced by the current velocity law. Pinhole output is therefore primarily telemetry and lost-target turn-point guidance, not a multi-person association gate or closed-loop metric-distance controller.

## 10. Controllers

The sole normal controller is `motion_arbiter_node.py`. Top-level states are `MANUAL`, `TRACKING`, `STANDBY`, and `MANUAL_GOTO`. Tracking substates include `SAFE_ZONE_HOVER`, `ADVANCING`, `BACKING_SMOOTH`, `LATERAL_YAW_ONLY`, `WAITING_FOR_TARGET`, `BACKING_UP_TO_RECOVER`, `ADVANCING_TO_TURN_POINT`, `ROTATING_AT_TURN_POINT`, and `SEARCHING_HOLD`.

Good behavior: 20/25 px deadbands, EMA error smoothing, lateral velocity/yaw caps, forward/backward limits, slew limiters, altitude hold, explicit manual authority, and no automatic new-person selection after the arbiter's five-second lost timeout. Temporary loss can trigger straight backing or calibrated turn-point advance/rotation before settling into hold.

Bad or unverified behavior: recovery motion can move while the target is unseen; the nominal 180-turn and right-lateral logs show severe retention failures; `small_box_max_speed` and area-control parameters appear stale/unreferenced; exact recovery timing differs between YAML aliases and source defaults; and target identity is not carried into the controller observation.

## 11. PX4 Interface

- Arbiter connects to `udpin:0.0.0.0:14540`, requests MAVLink streams at 10 Hz, and uses one RX monitor/cache thread.
- Normal dispatch is one `set_position_target_local_ned_send` per control tick, nominal 10 Hz (`control_period_s=0.10`). The function converts body-relative controller velocities using current yaw, then sends `MAV_FRAME_LOCAL_NED`, active position Z, velocity X/Y/Z, and yaw rate (`type_mask=0x05C3`). The module docstring still says `MAV_FRAME_BODY_NED`; the implementation is LOCAL_NED after conversion.
- `MANUAL_GOTO` sends `set_position_target_global_int_send` with `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`.
- Auto-takeoff waits for GPS/EKF and fresh vision; Offboard/armed state, heartbeat age, local-position age, watchdog, velocity/yaw caps, and slew limiters are monitored.

## 12. Safety

Present protections: process cleanup, GPS/EKF readiness, vision freshness (`0.40 s`), MAVLink stale timeout (`2 s`), control-loop warning/fail thresholds (`0.20/0.50 s`), heartbeat/Offboard checks, velocity/yaw/altitude limits, slew limiters, manual override, and disarm synchronization. Lost-target timeout returns to `STANDBY` hover/altitude hold and does not automatically pick another person.

The arbiter applies `COM_OF_LOSS_T=5.0` and related SITL-only parameter changes. The source explicitly warns this is a desktop-load compromise and must not be carried to real flight without safety review. Existing SITL "PASS" results cannot establish physical-flight safety.

## 13. Re-ID

No executable OSNet, FastReID, DeepSORT embedding, BoT-SORT appearance branch, gallery, or person-search module exists in this repository. `389689f` removed unfinished Re-ID modules. Remaining mentions are documentation/report text and audit/research material. Re-ID is therefore **MISSING/removed**, not dormant runtime functionality.

## 14. Tests

Executed: `python3 -m pytest -q ros2_ws/src/vision_tracking/test`

- `2 failed, 1 skipped`.
- `test_flake8.py`: 433 style errors/warnings, including generated build/install copies, root shims, long lines, imports, and formatting.
- `test_pep257.py`: 72 docstring errors.
- `test_copyright.py`: skipped by design.

PX4 scripts are live/integration harnesses, not safe blanket pytest tests. `tests/px4/README.md` documents camera, MAVLink, Offboard, state-machine, turnaround, long-path, and multi-trial scopes. Existing logs show 5/5 safety-status passes, but perception verdicts failed for at least nominal 180 and right-lateral scenarios. No active stack was available for ROS topic-rate measurement; `ros2 topic` failed with an operation-not-permitted error.

## 15. Performance Baseline

The recorded `logs/multi_trial_summary.json` is tagged commit `4b156b9`, dirty worktree, 4 CPUs, NVIDIA RTX 4060 Laptop GPU, 0.1 s control period, and reports 5/5 safety passes. It is not a clean end-to-end benchmark:

| Scenario | Tracking retention | Safety/control interpretation |
|---|---:|---|
| nominal 180 turn | 0.0% | Safety PASS; perception FAIL |
| fast 180 turn | 92.2% | Safety/control PASS; 17 target-loss events |
| lateral left | 87.3% | Safety/control PASS; 15 target-loss events |
| lateral right | 3.0% | Safety PASS; perception FAIL |
| aggressive close-in | 88.6% | Safety/control PASS; 16 target-loss events |

`FORENSIC_REGRESSION_AUDIT_FINAL.md` reports one nominal trial near 0.104 s control period, vision age mean 0.112 s/p95 0.185 s, and zero Offboard loss in its fix evidence. Conversely, `POST_FIX_VALIDATION.csv` records 0.207-0.320 s control means, roughly 5.1 s maximum gaps, watchdog stalls in most trials, and FAIL results. These datasets have different provenance and must not be merged into one number. No reliable current YOLO/tracker FPS, VRAM, or per-stage latency baseline was captured in this audit.

## 16. Technical Debt

- Four overlapping YAML configurations and duplicated parameter aliases (`deadband_x_px`/`deadband_x`, speed and turn-point aliases).
- `/tracking/select_target` is both operator command and detector feedback, with two publishers.
- Root compatibility shims are required by legacy tests but are included in linter scope.
- Generated build/install artifacts are included in broad lint scans.
- `/tracking/error` is an unversioned overloaded `Point` contract with no timestamp, frame ID, confidence, or identity.
- Geometry telemetry has no production consumer; camera calibration has no `camera_info` contract.
- Apparently unused area-control parameters (`target_area_*`, `kp_area`, `small_box_max_speed`) need confirmation before any cleanup.
- No logical mission target handle, explicit ambiguity rejection, or structured target state machine.

### Source of Truth Report

| Capability | Current source of truth | Conflicts/debt |
|---|---|---|
| Detection/tracking | `vision_tracking/yolo_detector_node.py` plus Ultralytics `bytetrack.yaml` | Tracker thresholds are inherited from Ultralytics; no project-level benchmark config |
| Target selection/lock | Detector `select_target()`, `_reacquire_lock()`, and HUD command publishers | `/tracking/select_target` has two publishers; arbiter also mirrors an ID |
| Distance/geometry | `pinhole_geometry.py` called from arbiter (working-tree change) | Hard-coded calibration; legacy inline implementation existed; no `camera_info` |
| Motion/control | `vision_tracking/motion_arbiter_node.py` | Root shim is used by legacy tests; YAML aliases/duplicates |
| PX4 interface | Arbiter `_send_offboard_velocity()` and `_send_goto_position_setpoint()` | Module docstring still names BODY_NED while implementation sends LOCAL_NED after conversion |
| Runtime orchestration | `start_stack.sh` + `tracking_stack.launch.py` | Launch overrides device/takeoff; optional realism changes image topic |
| Configuration | Installed `tracking_stack.yaml` in normal launch | Per-node YAMLs and source defaults duplicate values |

## 17. Risks

| Priority | Risk | Evidence |
|---|---|---|
| P0 | Wrong-person continuation in automatic mode | Fallback `max(area*confidence)` after current track loss; no logical identity |
| P0 | Geometry/area contract mistaken for metric distance | `/tracking/error.z` is only scaled bbox area; area controller parameters are stale |
| P0 | SITL failsafe tuning copied to real flight | Arbiter writes `COM_OF_LOSS_T=5.0` and labels it SITL-only |
| P1 | Severe camera-motion/crossing loss | 0.0% and 3.0% retention scenarios |
| P1 | Ambiguous short-term ID remap | IoU/proximity remap can bind to a nearby person without appearance confirmation |
| P1 | Recovery motion while unseen | Backing/turn-point substates operate before final hold |
| P2 | Multiple configuration/source paths | Unified YAML, per-node YAML, source defaults, launch overrides |
| P2 | Test signal polluted by generated artifacts and live scripts | 433 flake8 and 72 pep257 errors; root shims/build/install scanned |
| P3 | Unmeasured compute budget | No clean stage-level FPS/VRAM/CPU benchmark |

## Direct Answers

1. **Working modules:** stack launcher, Gazebo/PX4 integration, packaged YOLOv8+ByteTrack detector, packaged motion arbiter, HUD, optional realism node, MAVLink Offboard path, pinhole telemetry module, and simulation assets. "Working" means present and exercised; perception quality is scenario-dependent.
2. **Source of truth:** normal runtime source is the packaged ROS nodes launched by `tracking_stack.launch.py`; the arbiter is the sole normal PX4 command source. Configuration has no single clean source because YAMLs and source defaults overlap.
3. **Tracking:** YOLOv8 `track(... persist=True, tracker='bytetrack.yaml')`; selected box is EMA-smoothed and published as aggregate pixel error/area.
4. **Target selection:** operator click/key in normal mode; optional auto mode keeps current track or chooses maximum `area*confidence`.
5. **Logical identity:** No. `target_id` and `active_target_id` are ByteTrack IDs; `target_handle`/`mission_target_id` is missing.
6. **Distance:** current working tree computes calibrated pinhole ground geometry from pixel offset, altitude, pitch, and assumed person height, but ordinary speed control still uses 2-D error and area parameters are not active.
7. **Controller good/bad:** good deadbands, smoothing, slew limits, altitude hold, manual priority, and standby on timeout; bad retention under some camera motions, recovery movement without visual confirmation, stale area parameters, and no identity in the observation contract.
8. **Lost versus `cb6b1ae`:** the original inline pinhole/search implementation and direct ArduPilot BODY_NED path were refactored into PX4/ROS; current tree has restored pinhole support in the working tree, but old direct search semantics and one source-of-truth control path are gone.
9. **Re-ID remaining:** no executable code; removed at `389689f`; documentation-only references remain.
10. **Immediate fixes:** prevent automatic wrong-target switches, define an identity/observation contract, validate pinhole calibration and distance use, and separate SITL failsafe tuning from flight configuration.
11. **Do not change now:** ROS topic/message contracts, sole arbiter ownership of MAVLink, manual override priority, root compatibility shims, or tracker/controller until benchmark evidence exists.
12. **Safest order:** baseline instrumentation -> contract/source-of-truth cleanup -> target handle/lock policy -> geometry validation -> camera-motion/tracker benchmark -> recovery benchmark -> optional on-demand Re-ID.
13. **Benchmark before Re-ID:** IDF1/HOTA/MOTA, ID switches, target switches/loss/recovery/false recovery, retention under occlusion/crossing/turns, end-to-end latency/FPS/CPU/GPU/VRAM, and separate Rank-1/mAP Re-ID tests.
14. **Keep unchanged to avoid regression:** ByteTrack invocation and camera QoS while measuring, MAVLink single-writer arbiter, 10 Hz Offboard stream, manual authority gating, safety watchdogs, and existing scenario fixtures.

### References

Bewley et al., "Simple Online and Realtime Tracking," ICIP 2016, doi:10.1109/ICIP.2016.7533003. Wojke et al., "Simple Online and Realtime Tracking with a Deep Association Metric," ICIP 2017. Zhang et al., "ByteTrack: Multi-Object Tracking by Associating Every Detection Box," ECCV 2022. Aharon et al., "BoT-SORT," arXiv:2206.14651. Cao et al., "Observation-Centric SORT," CVPR 2023. Du et al., "StrongSORT," IEEE TMM 2023. Zhou et al., "Omni-Scale Feature Learning for Person Re-Identification," ICCV 2019. Liu et al., "A Deep Model for Person Re-identification in a Joint Detection and Identification Framework," ICCV 2015. UAVDT (Du et al., ECCV 2018) and VisDrone (Zhu et al., ECCV 2018) are relevant UAV benchmarks. Official runtime evidence is from this repository's source, launch files, logs, and test guides.
