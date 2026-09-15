# Gazebo and Live HUD Performance Investigation

Investigation date: 2026-09-14  
Scope: measurement and diagnosis only. No production code, GPU/WSL configuration, world, camera, model, tracker, QoS, or safety setting was changed.

## Executive Summary

The observed "~10 FPS HUD" is not explained by the Gazebo camera sensor itself. A temporary host-level Gazebo probe of the existing `person_tracking_path.sdf` measured `/camera/image_raw` at approximately 28-30 Hz and `/camera/camera_info` at approximately 29-30 Hz. World statistics showed a 4 ms physics step and a real-time factor varying from about 0.84 to 1.00 during the short probe.

The full stack is CPU-contended. Existing process samples show Gazebo at 83-86% CPU (occasionally above one core), YOLO at 79-96% mean CPU with peaks above 200%, PX4 around 6-7% mean, bridge around 4%, and MotionArbiter around 6%. Existing forensic logs report that under combined load the effective camera/vision stream was about 15 FPS, with YOLO CUDA inference about 18-25 ms once warm. This means the live HUD usually displays the detector's `/tracking/debug_image`, not the raw camera, so its visible rate is bounded by detector output and CPU scheduling.

There is also a separate latency/backpressure risk: `live_camera_hud_node.image_callback()` performs conversion, all drawing, `cv2.imshow`, `cv2.getWindowImageRect`, and `cv2.waitKey(1)` synchronously in the ROS callback. The HUD subscription uses a queue depth of 1, while the detector debug publisher uses depth 2. This is not a historical-frame backlog of arbitrary length, but the callback can still block ROS image delivery and display the newest completed debug frame only when the callback returns. The present data does not prove whether the user's exact 10 FPS is detector-limited, UI-limited, or both.

The strongest measured bottlenecks are therefore:

1. Combined CPU contention between Gazebo simulation/render/sensors and Python YOLO execution.
2. Detector output rate and per-frame inference work, not the ByteTrack algorithm itself.
3. Synchronous HUD display work and lack of a decoupled latest-frame render path.
4. WSLg/D3D12 GPU behavior remains unresolved: host `nvidia-smi` sees the RTX 4060, but no active Gazebo process was attributed GPU utilization in the probe and OpenGL/EGL tools are unavailable.

## Current Environment

| Item | Measurement |
|---|---|
| Kernel | WSL2 `6.18.33.2-microsoft-standard-WSL2` |
| Distribution | Ubuntu 22.04.5 LTS (Jammy) |
| Python | 3.10.12 |
| ROS | ROS 2 Humble environment (`ROS_VERSION=2`, `ROS_DISTRO=humble`); `ros2 --version` is not a valid CLI form |
| Gazebo | Gazebo Sim 8.15.0, project selects `GZ_VERSION=harmonic` |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU, driver 581.86, CUDA 13.0, 8188 MiB |
| GPU idle probe | 0 MiB / 0% utilization when no host process was active |
| Display | `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0` |
| Project camera | 640x480, 30 Hz, HFOV 2.0 rad, OGRE2 sensor renderer |
| Project inference | YOLOv8n, native 640x480 inference, ByteTrack, CUDA selected by `start_stack.sh` when available |

The audit shell initially contained `LIBGL_ALWAYS_SOFTWARE=1`, but `start_stack.sh` explicitly unsets that variable unless `USE_SOFTWARE_RENDERING=1`. Therefore the shell value is not evidence that the normal launcher uses software rendering.

## Measured Baseline

### Temporary Gazebo-only probe

The existing world was run headless with the project model/resource paths and no production file changes.

| Signal | Observed |
|---|---:|
| `/camera/image_raw` | 28-30 Hz over repeated windows; approximately 29.1 Hz in one stable window |
| `/camera/camera_info` | approximately 29-30 Hz |
| `/world/person_tracking_path/stats` | 4 ms `step_size`; `iterations` consistent with 250 Hz physics; sample RTF 0.838 then approximately 1.000 |
| Gazebo server CPU | approximately 53-82% in short headless probes, depending on startup/measurement interval |
| Gazebo GUI CPU | approximately 60% in a temporary GUI probe; server approximately 41% at the same instant |
| Host NVIDIA utilization during GUI probe | 0%; no process attributed by `nvidia-smi` |

The GUI probe was intentionally short and not a visual FPS counter. It identifies process CPU cost, but not a definitive WSLg render backend.

### Existing full-stack process samples

`logs/phase3_process_samples.csv` and `_after.csv` contain 60 one-second samples per role (180 PX4 rows because multiple PX4 processes were matched).

| Process | Mean CPU | Maximum observed CPU | Interpretation |
|---|---:|---:|---|
| Gazebo | 85.7% (`after`: 83.0%) | 101.6% (`after`: 106.2%) | Main simulation/render/sensor CPU consumer |
| YOLO detector | 79.2% (`after`: 95.5%) | 211.4% (`after`: 233.3%) | Major bursty Python/inference consumer |
| PX4 | 6.6% | 23.9% | Material but not primary |
| `ros_gz_bridge` | 4.1% | 6.0% | Low relative CPU cost in this profile |
| MotionArbiter | 6.0% | 8.0% | Not the FPS bottleneck |
| HUD | No row captured by the sampler | Unknown | Sampler pattern did not observe a distinct HUD process in this dataset |

`FORENSIC_REGRESSION_AUDIT_FINAL.md` reports a tested combined-load camera/vision rate near 15 FPS, CUDA inference 18-25 ms after initialization, and DDS IPC below 2 ms when the host is not starved. The same report documents historical Gazebo CPU peaks near 101-106% and warns that host contention creates scheduler gaps.

### Existing latency evidence

The detector's own logs show warm mean inference latency settling around 20-70 ms in one captured run, with early negative/invalid values caused by startup timestamp mixing. The forensic report's controlled warm CUDA figure of 18-25 ms is the more reliable reference. `logs/COMMAND_LATENCY_VALIDATION.csv` measures command path latency, not image/HUD FPS; it must not be used as a frame-rate measurement.

## Gazebo Investigation

The world requests ODE physics at `max_step_size=0.004` and `real_time_update_rate=250`, plus contact, physics, sensors (OGRE2), scene broadcaster, IMU, magnetometer, air pressure, NavSat, and a custom optical-flow plugin. The x500 model includes a 30 Hz camera and a 50 Hz GPU lidar. The scene includes a sky/cloud system, a shadow-casting directional sun, static axes, multiple walkway visual/collision segments, a vehicle, and an animated pedestrian actor.

The headless probe demonstrates that the camera sensor is capable of its configured 30 Hz and that the physics loop can reach its configured 250 Hz update rate when isolated. Therefore the configured sensor rate is not the same as the full-stack delivered rate. Under full load, Gazebo is close to one CPU core and can starve Python callbacks and the bridge.

The world log also reports a missing `libOpticalFlowSystem.so` in one direct probe. That is a configuration/runtime warning and should be resolved or explicitly excluded in a separate task; this audit does not assume it is the cause of low camera FPS. The project world can still enumerate camera and stats topics in the host probe.

## Camera Pipeline Investigation

```text
Gazebo camera sensor (configured 30 Hz)
  -> Gazebo transport /camera/image_raw
  -> ros_gz_bridge parameter_bridge
  -> ROS /camera/image_raw
  -> yolo_detector_node (native 640x480 YOLO + ByteTrack)
  -> ROS /tracking/debug_image
  -> live_camera_hud_node callback
  -> cv_bridge + OpenCV overlays + imshow/waitKey
```

The HUD subscribes to `/tracking/debug_image`, not the raw camera. Consequently, a HUD counter around 10 FPS can mean:

- detector/debug-image publication is around 10 FPS;
- HUD callback/display is slower than debug-image publication;
- both are operating under CPU scheduling delays;
- or the display is presenting completed frames at a lower rate while the source is faster.

There is no timestamp-bearing image-processing metric in the HUD. The image message header is converted by `cv_bridge`, but the HUD does not record source stamp, callback start/end, display time, or queue age. Exact end-to-end image latency therefore cannot currently be derived from the HUD itself.

## YOLO Investigation

The detector calls `YOLO.track()` on every accepted image, with `persist=True`, `bytetrack.yaml`, class 0, confidence 0.45, IoU 0.45, and native 640x480 inference (`imgsz=640` when native). It then filters boxes, performs selection/reacquisition, creates an annotated image, and publishes `/tracking/debug_image`.

The available evidence points to YOLO as a major full-stack cost but not a fixed 10 FPS limiter in isolation: 18-25 ms warm CUDA inference would permit more than 30 FPS in a perfectly scheduled pipeline, while recorded process CPU and vision rates degrade substantially under Gazebo/PX4 contention. Startup CUDA initialization also took about 3.8-4.9 seconds in the forensic report; that is a startup transient, not steady-state FPS.

There is no clean current run that separately records preprocessing, model execution, postprocessing, ByteTrack association, drawing, and publish time. The detector's aggregate latency log is wall-time based and was contaminated at startup by clock initialization, so it should be treated as indicative only.

## Tracker Investigation

ByteTrack is executed inside the same detector callback after YOLO inference. No independent tracker process or tracker FPS counter exists. ByteTrack association is unlikely to be the primary computational bottleneck at this scene density; the expensive operation is model inference plus image handling. However, tracker persistence and target loss can alter how many detections are drawn and how often the HUD appears to update, so tracking quality and display cadence should be measured separately in replay.

## HUD Investigation

`live_camera_hud_node.image_callback()` performs all of the following synchronously for every debug image:

1. `CvBridge.imgmsg_to_cv2(..., desired_encoding='bgr8')` conversion.
2. FPS counter and text rendering.
3. Full-frame minimap/GPS drawing, multiple rectangles/lines/circles/text overlays, and state/help banners.
4. `cv2.imshow()`.
5. `cv2.getWindowImageRect()`.
6. `cv2.waitKey(1)` and key-command publication.

The callback is the only render loop. `rclpy.spin()` cannot dispatch another HUD image callback until this work returns. The subscription queue depth is `1`, which favors dropping queued images over accumulating an unbounded backlog, but it does not prevent a frame from becoming stale while the callback is blocked. The debug publisher queue depth is `2`; both use default reliable ROS QoS because no explicit QoS profile is supplied on those endpoints.

The HUD's displayed FPS counter measures callback arrivals over one-second windows, not actual monitor refresh, source camera FPS, or frame age. No separate `imshow`/display timing is recorded.

## ROS/DDS Investigation

Camera input to YOLO uses KEEP_LAST depth 1 and BEST_EFFORT. This is an intentional perishable-frame policy. The HUD debug-image subscription uses depth 1 but default reliable QoS, and the detector debug publisher uses depth 2/default reliable QoS. The raw image bridge is a separate ROS/Gazebo transport process, so serialization/copying occurs across the bridge and into Python `cv_bridge` buffers.

Existing forensic measurements found localhost DDS IPC below 2 ms when CPU is available, which rejects DDS transport as the primary cause in that tested configuration. DDS still consumes CPU and memory bandwidth, and reliable debug-image delivery can add scheduling pressure, but there is no evidence here that ROS transport alone creates the 10 FPS result.

## WSL2 / GPU Investigation

Host-level `nvidia-smi` works and identifies the RTX 4060, driver 581.86, with 0% utilization while idle. Inside the audit sandbox, Linux NVML and `/dev/dxg` access were blocked; this is an execution-environment limitation. The host-level GUI probe showed Gazebo server/gui processes but `nvidia-smi` reported 0% GPU and no attributed process. `glxinfo` and `eglinfo` are not installed, so the actual WSLg OpenGL/EGL adapter cannot be named from this environment.

The evidence supports the following bounded conclusion:

- NVIDIA CUDA access for YOLO has been observed in prior project logs (`CUDA device available: cuda:0`, RTX 4060).
- Gazebo's actual render adapter/backend is **not proven** by this audit.
- Gazebo and GUI CPU usage is high enough that CPU-bound or software/translated rendering remains plausible.
- WSLg/D3D12 contention is a credible contributing factor from prior forensic logs, but it is not isolated sufficiently to call the root cause.

## Root-Cause Classification

| Finding | Classification | Confidence | Evidence |
|---|---|---:|---|
| Full stack is CPU-contended | CPU bottleneck; synchronization/backpressure | High | Gazebo 83-86% mean CPU, YOLO 79-96% mean, peaks >200%; historical scheduler gaps |
| Isolated camera sensor is not inherently 10 FPS | Not a camera configuration bottleneck | High | Host probe `/camera/image_raw` approximately 29-30 Hz |
| Detector output is below raw camera under load | CUDA/CPU inference bottleneck; synchronization | Medium-high | Prior full-stack camera/vision rate near 15 FPS; warm CUDA 18-25 ms; process CPU peaks |
| HUD callback can limit display cadence | Python/UI bottleneck; synchronization/backpressure | High (code path), unknown (share of observed FPS) | Synchronous conversion/drawing/`imshow`/`waitKey`, no render thread |
| DDS is primary cause | Rejected in tested configuration | High | Prior measured IPC <2 ms; bridge CPU ~4% |
| Gazebo physics alone is primary cause | Unproven | Medium-low | Isolated RTF 0.84-1.0 and 250 Hz stats, but full stack Gazebo CPU ~1 core |
| Gazebo GPU rendering is primary cause | Unknown | Low | No WSLg renderer query available; host `nvidia-smi` did not attribute GUI usage |
| World complexity contributes | Plausible secondary | Medium | OGRE2 camera, 50 Hz lidar, shadows, sky/clouds, animation, multiple sensors/models |
| Historical orphaned Gazebo processes | Confirmed historical risk, not present in current probe | High historically | Forensic report identified orphan Ruby/Gazebo CPU consumers; current probe cleaned its temporary process |

## Evidence and Unknowns

Known:

- The raw camera can publish close to 30 Hz in isolation.
- The full stack has high Gazebo and YOLO CPU utilization.
- Warm CUDA inference is materially faster than the historical CPU fallback (90-150 ms/frame), but still competes for resources.
- The HUD is synchronous and has no source/display timestamps.

Unknown:

- Exact current `/tracking/debug_image` rate during the user's session.
- Exact HUD callback rate versus `imshow` presentation rate.
- Per-stage detector timing for preprocessing, inference, tracking, drawing, and publish.
- Actual WSLg renderer adapter and whether Gazebo GUI uses D3D12 hardware acceleration, llvmpipe, or another path.
- Current GPU utilization while the full stack is running; the stack was not safely started as part of this audit.
- Whether the missing optical-flow library warning occurs in the user's normal `start_stack.sh` path and whether it affects sensor scheduling.

## Recommended Fixes (Not Applied)

### Minimum-risk fixes

- Add read-only profiling and timestamps around raw camera receipt, detector start/end, debug-image publish, HUD callback start/end, `imshow`, and `waitKey`.
- Record `ros2 topic hz` for `/camera/image_raw` and `/tracking/debug_image` in the same run, plus `/world/.../stats` RTF and process CPU.
- Add a process sampler row for the HUD and capture one host `nvidia-smi dmon` trace during the run.
- Verify and document the actual WSLg renderer with host-side OpenGL/EGL tools before changing graphics settings.
- Preserve queue depth and safety behavior while measuring; do not infer latency from FPS.

### Medium fixes

- Decouple HUD display from ROS callbacks with a latest-frame-wins buffer and a dedicated display loop, preserving command handling and queue semantics.
- Reduce repeated HUD overlay work only after profiling identifies it as material.
- Separate detector profiling into preprocessing, YOLO, ByteTrack/postprocess, drawing, and publication.
- Use CPU/process affinity or scheduling isolation only as a measured experiment, not a permanent default.

### Architecture-level fixes

- Separate raw-camera visualization from detector debug visualization when operator responsiveness matters.
- Add timestamped image/observation contracts and a replay profiler that reports frame age, drops, and stage latency.
- Evaluate a native Linux GPU execution path if WSLg measurements prove graphics translation is the limiting factor.
- Consider zero-copy/intra-process transport only if profiling demonstrates image-copy/DDS cost after CPU contention is controlled.

## Explicit Answers

### Why is the HUD around 10 FPS?

The evidence does **not** support "the camera is configured for 10 FPS." The isolated camera is about 29-30 Hz. In the full stack, the most likely chain is CPU contention reducing detector/debug-image production toward the recorded ~15 FPS (and potentially lower during stalls), combined with a synchronous HUD callback that performs image conversion, full-frame OpenCV drawing, and GUI calls. The exact split between detector rate and HUD render rate is not measured yet because no live `/tracking/debug_image` sample or HUD callback/display timestamps were captured. A stale-frame backlog is bounded by depth-1 HUD subscription, but callback blocking can still make displayed frames old relative to capture time.

### Why is Gazebo slow?

Gazebo is not camera-sensor-rate bound in isolation and is not proven GPU-bound. It is a high-CPU process in the combined profile, often near one full core, while running physics, OGRE2 camera rendering, a 50 Hz lidar, optical-flow/sensor plugins, animated actor, shadows, sky/clouds, and scene geometry. This CPU contention plausibly lowers effective RTF and starves YOLO/HUD callbacks. WSLg/D3D12 translation may contribute, but the actual adapter and GPU utilization during a normal stack run remain unknown. The safe conclusion is **CPU contention is confirmed; GPU/WSLg rendering is unresolved**.

## What Should Not Change in This Investigation

Do not lower resolution, replace YOLO, replace ByteTrack, disable the HUD or safety systems, change ROS message/QoS contracts, simplify the world, or alter GPU/WSL configuration until the missing stage-level measurements are captured.
