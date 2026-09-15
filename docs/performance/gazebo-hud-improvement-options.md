# Gazebo and HUD Improvement Options

These are proposals only. No option was applied during the investigation.

## Evidence-Driven Order

1. Instrument and capture a clean baseline.
2. Prove detector rate versus HUD render rate.
3. Prove Gazebo render backend and GPU use.
4. Isolate image-copy/DDS cost.
5. Apply the smallest fix matching the measured bottleneck.

## Options by Root Cause

| Root cause to prove | Minimum-risk fix | Medium fix | Architecture-level fix | Do not do yet |
|---|---|---|---|---|
| HUD callback/display backpressure | Add callback/render timing and source-stamp age metrics | Latest-frame-wins buffer with a dedicated display loop; retain command handling | Separate visualization process or raw-camera viewer from detector debug stream | Do not rewrite HUD before measuring callback and display time |
| Detector compute/scheduling | Profile preprocessing, inference, postprocess, drawing separately; warm up before trials | Scheduling/affinity experiment; avoid duplicate work only after evidence | Dedicated perception process/accelerator pipeline with timestamped outputs | Do not change model, resolution, tracker, or precision in this task |
| Gazebo CPU contention | Capture RTF, `/stats`, camera rate, and process CPU in the same run | Measured renderer/sensor configuration tuning while preserving scenario | Move simulation to a native Linux GPU host or isolate simulation/perception resources | Do not simplify the world or disable safety sensors blindly |
| WSLg/D3D12 translation | Install/use existing host-side renderer diagnostics; capture `nvidia-smi dmon` concurrently | Compare hardware-rendered and software-rendered temporary runs with identical world and record RTF/CPU/GPU | Native Linux Gazebo/ROS execution if A/B proves WSLg cost | Do not set `LIBGL_ALWAYS_SOFTWARE=1` or change `.wslconfig` as a guess |
| ROS/DDS/image copies | Measure topic rates, message size, callback age, and bridge CPU | Revisit QoS only with measured queue/drop evidence | Intra-process/zero-copy or compressed transport for non-control visualization | Do not change topic/message contracts during diagnosis |
| Stale frames | Add frame sequence/timestamp and drop counters | Latest-frame-wins for HUD and diagnostic consumers | Timestamped observation pipeline with explicit age budget | Do not infer backlog from FPS watermark alone |
| World/sensor complexity | Attribute CPU/RTF to existing sensor/plugin processes | Temporarily toggle one non-control visual sensor per A/B experiment and revert | Separate high-fidelity flight physics from operator visualization scene | Do not remove models, shadows, lidar, or optical flow in production |

## Required Measurements Before Implementation

Capture one reproducible run with:

- `/camera/image_raw` frequency and message header stamps;
- `/tracking/debug_image` frequency and stamps;
- detector callback count, preprocess/inference/postprocess/draw/publish durations;
- HUD callback start/end, `imshow`/`waitKey` duration, displayed frame stamp, and drop count;
- `/world/person_tracking_path/stats` RTF, iterations, and step size;
- CPU samples for Gazebo server, Gazebo GUI, PX4, bridge, YOLO, arbiter, and HUD;
- host `nvidia-smi dmon` utilization, memory, power, and temperature;
- ROS topic QoS and bandwidth for raw/debug images;
- a controlled sequence: Gazebo-only, camera-only, bridge, YOLO, full stack.

The acceptance report should distinguish:

```text
source FPS -> bridge FPS -> detector FPS -> debug-image FPS -> HUD callback FPS -> display FPS
```

and separately:

```text
capture stamp -> callback receipt -> inference end -> debug publish -> HUD receipt -> display time
```

## Decision Rules

- If raw camera is near 30 Hz but debug image is near 10 Hz, optimize or schedule the detector path first.
- If debug image is above 20 Hz but HUD callback/display is near 10 Hz, decouple rendering and use latest-frame-wins.
- If Gazebo-only RTF is low and Gazebo CPU is saturated, investigate physics/sensor/render cost before changing perception.
- If Gazebo-only RTF is healthy but full-stack RTF collapses, treat it as cross-process CPU/GPU contention.
- If GPU utilization is low while Gazebo and YOLO CPU are high, prioritize CPU scheduling, Python/image copies, or software/translated rendering investigation.
- If GPU utilization is near saturation and YOLO latency dominates, only then evaluate model/inference optimization in a separate approved task.
- If DDS bandwidth or callback age is high while compute is idle, evaluate transport/QoS or zero-copy architecture.

## Changes Explicitly Deferred

The following remain unchanged until evidence supports them: camera resolution, YOLOv8n weights, ByteTrack configuration, ROS topic names and messages, safety timeouts, PX4 interface, Gazebo world contents, and WSL2/GPU configuration.

## Current Recommendation

The first implementation should be instrumentation, not optimization: timestamp each image stage, add a HUD process sample, collect simultaneous Gazebo stats and host GPU data, and run the four isolation levels. Based on current evidence, the likely minimum-risk improvement after that measurement is a latest-frame-wins HUD display path, while the likely system-level issue is CPU contention between Gazebo and YOLO. A WSLg/native-Linux decision should wait for an actual renderer/backend A/B result.
