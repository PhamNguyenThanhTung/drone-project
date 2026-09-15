# GPU and Gazebo Rendering Verification

Date: 2026-09-14

Scope: forensic verification only. No production source, WSL setting, CUDA setting, Gazebo world, camera setting, model, tracker, QoS, or ROS interface was changed. The probes used temporary processes and were terminated afterward.

## Final classification

| Item | Result | Evidence strength |
|---|---|---|
| YOLO GPU | **CONFIRMED** | The real detector selected `cuda:0` and named the RTX 4060; direct-process probing showed a VRAM increase and non-zero RTX SM activity while it was running. |
| Gazebo GPU | **CONFIRMED OTHER GPU** | Gazebo's OGRE2 log reports `GL_RENDERER = D3D12 (AMD Radeon(TM) 780M)`, not the RTX 4060. |
| Gazebo software rendering | **NOT CONFIRMED** | The EGL device is labelled `EGL_MESA_device_software`, but the active GL renderer is explicitly D3D12 on the AMD 780M. This is WSLg/Mesa D3D12 translation, not evidence of llvmpipe. |
| Primary bottleneck | **CPU / synchronization, with Gazebo on the AMD WSLg path** | Isolated Gazebo stayed near real time and published the camera near 30 Hz; combined historical samples show Gazebo and Python vision competing for CPU. RTX utilization was low for Gazebo and increased during YOLO. |

## Environment

Read-only command results:

```text
Linux tun 6.18.33.2-microsoft-standard-WSL2 x86_64
Ubuntu 22.04.5 LTS
Python 3.10.12
Gazebo Sim 8.15.0
DISPLAY=:0
WAYLAND_DISPLAY=wayland-0
MESA_D3D12_DEFAULT_ADAPTER_NAME=<empty>
LIBGL_ALWAYS_SOFTWARE=1 (present in the audit shell)
```

The normal project launcher explicitly unsets `LIBGL_ALWAYS_SOFTWARE` unless `USE_SOFTWARE_RENDERING=1`; therefore the shell value above is not treated as the launcher's renderer setting.

Host-visible NVIDIA query:

```text
NVIDIA GeForce RTX 4060 Laptop GPU
Driver 581.86
CUDA reported by driver 13.0
VRAM 8188 MiB
```

`glxinfo`, `eglinfo`, and `vulkaninfo` are not installed. The authoritative Gazebo renderer result below came from Gazebo/OGRE's own log, not from those absent utilities.

## YOLO / PyTorch verification

### Real detector initialization

The packaged detector was run with the existing project model and `device:=cuda:0`. Its log (`/tmp/gpu_verify_direct_yolo.log` and `/tmp/gpu_verify_probe2_yolo.log`) contains:

```text
CUDA device available: cuda:0 (NVIDIA GeForce RTX 4060 Laptop GPU)
Loading YOLO model [.../yolov8n.pt] on device [cuda:0]
yolo_detector_node ready ... device=cuda:0
Original image: 640x480 | Inference image: 640x480 (native)
```

This is runtime evidence from the actual ROS node, not a README or a requested parameter alone. It establishes that the node's PyTorch/Ultralytics path initialized CUDA device 0 and identified it as the RTX 4060.

An independent shell query in the restricted audit sandbox reported `torch 2.13.0+cu130`, `built_cuda 13.0`, but `torch.cuda.is_available() = False`. That result is an execution-sandbox limitation: the same sandbox reports NVML access blocked and has no `/dev/dxg`. It does not contradict the host-level detector run, which initialized CUDA successfully.

### Direct-process probe

The detector was launched as the installed executable, avoiding the `ros2 run` wrapper:

```text
PID 21432 /usr/bin/python3 .../install/vision_tracking/lib/vision_tracking/yolo_detector_node ... -p device:=cuda:0 ...
```

The process tree showed the detector itself at about 620 MiB RSS shortly after startup and about 1.43 GiB RSS after active image processing. During the same probe:

```text
RTX VRAM before detector activity: 147 MiB
RTX VRAM after CUDA model/activity: 286-294 MiB
Observed increase: approximately 139-147 MiB
RTX SM utilization: 20%, 10%, 6% before warm-up; later samples up to 47%
RTX memory-controller utilization: up to 28%
GPU clocks: 8200 MHz during active samples
```

The sampled query file was `/tmp/gpu_verify_probe2_query.csv`; the corresponding `dmon` output was `/tmp/gpu_verify_probe2_dmon.log`.

The detector log reported warm-window mean latencies of approximately 117-153 ms in this short probe. Existing project profiling has a more stable warm CUDA figure of approximately 18-25 ms; the short probe was CPU-contended and had no usable detections for much of its run, so it is not used as a model benchmark.

### Process attribution limitation

`nvidia-smi --query-compute-apps` did report the direct detector PID (`21432`) but returned `[Not Found]` for the process name and `[N/A]` for used memory. The same limitation appeared for an older process entry (`20166`). `nvidia-smi pmon` likewise returned no process rows. WSL's NVML interface therefore cannot provide clean per-process names/memory accounting here.

This does not remove the CUDA conclusion: the PID was the direct detector executable, its own log identified the RTX 4060, VRAM rose exactly when it started processing, and RTX SM activity was observed during the same interval. It does mean the report does not claim an exact per-process VRAM allocation.

## Gazebo renderer verification

### Gazebo configuration and runtime

The project world requests OGRE2 for sensors:

```xml
<plugin filename="gz-sim-sensors-system" ...>
  <render_engine>ogre2</render_engine>
</plugin>
```

Gazebo's server log also shows `Loading plugin [gz-rendering-ogre2]` and `SensorsPrivate::RenderThread started`. The configuration identifies the rendering engine, but not the adapter; the adapter comes from OGRE's runtime log.

### Authoritative OGRE runtime output

`/home/tungt/.gz/rendering/ogre2.log`, generated during the GUI probe, contains:

```text
Loading library .../OGRE/RenderSystem_GL3Plus.so
Found Num EGL Devices: 1
EGL Device: EGL_MESA_device_software EGL_EXT_device_drm_render_node #0
Created GL 4.2 context for device EGL_MESA_device_software EGL_EXT_device_drm_render_node #0
GL_VERSION = 4.2 (Core Profile) Mesa 23.2.1
GL_VENDOR = Microsoft Corporation
GL_RENDERER = D3D12 (AMD Radeon(TM) 780M)
```

The decisive field is `GL_RENDERER`: Gazebo is using the WSLg/Mesa D3D12 path backed by the integrated AMD Radeon 780M. It is not using the NVIDIA RTX 4060. The `EGL_MESA_device_software` label is a Mesa EGL device label in this environment; it should not be rewritten as “llvmpipe” because the same context reports a D3D12 AMD renderer.

### NVIDIA telemetry during Gazebo-only probes

Headless server with `--headless-rendering`, complete project resource paths, and the existing world:

```text
Gazebo process: about 58% CPU at the sampled instant
RTF: approximately 0.999-1.000 in /world/person_tracking_path/stats
Physics: 4 ms step, approximately 250 Hz iterations
Camera: 640x480 RGB frames, approximately 28-30 Hz in prior topic measurements
RTX VRAM: 147 MiB throughout the Gazebo-only sample
RTX SM: 0% in 9/10 dmon samples (one transient memory-utilization sample)
RTX power: approximately 1.3 W
```

GUI run (`gz sim -r`):

```text
Processes: gz sim server and gz sim gui were both present
RTX VRAM: 147 MiB throughout the 12-second sample
RTX SM: 0% in every dmon sample
RTX power: approximately 1.3 W
```

The GUI probe did not expose a numerical GUI render-FPS counter. It does establish that Gazebo GUI activity did not create measurable RTX load, while OGRE independently identifies the AMD D3D12 renderer.

## Controlled comparison

| Probe | RTX observation | Gazebo / camera observation |
|---|---|---|
| Gazebo server + headless rendering only | 147 MiB, 0% SM in essentially all samples | RTF near 1.0; camera capable of about 30 Hz |
| Gazebo GUI (`gz sim -r`) | 147 MiB, 0% SM throughout sample | GUI and server processes present; renderer log says AMD 780M D3D12 |
| Gazebo + bridge + direct YOLO | VRAM rose to 286-294 MiB; SM reached 47% | Direct detector PID active; Gazebo remained a high-CPU process |

The incremental VRAM/SM change occurs with YOLO startup and image processing, not with Gazebo alone. This independently separates the two GPU paths.

## What is confirmed vs unresolved

### Confirmed

- The RTX 4060 is visible to host `nvidia-smi` and is used by the real YOLO node through CUDA.
- YOLO startup selects `cuda:0`, names the RTX 4060, increases reported RTX VRAM by roughly 140 MiB, and produces non-zero RTX SM activity.
- Gazebo uses OGRE2 and its active GL context reports D3D12 on `AMD Radeon(TM) 780M`.
- Gazebo GUI and Gazebo-only headless probes produce no sustained measurable RTX utilization.
- Isolated Gazebo physics/camera is close to configured timing; the camera is not intrinsically a 10 FPS source.

### Not measured or not attributable exactly

- Exact GUI monitor/render FPS: no in-app counter or renderer timing endpoint was available in the probe.
- Per-process GPU names and exact per-process VRAM: WSL NVML returned `[Not Found]`/`[N/A]` from `query-compute-apps` and empty `pmon` rows.
- The fraction of full-stack lag caused by Gazebo's AMD D3D12 path versus Python/ROS scheduling: stage timestamps are not currently emitted by the project.
- The shell's `LIBGL_ALWAYS_SOFTWARE=1` value is not representative of normal `start_stack.sh` behavior because the launcher unsets it by default.

## Bottleneck interpretation

The evidence does not support “the RTX 4060 is rendering Gazebo.” Gazebo is on the integrated AMD WSLg/Mesa D3D12 path, and its isolated camera/physics timing is near target. Under the full stack, historical process samples show Gazebo around 83-86% mean CPU with peaks above one core, YOLO around 79-96% mean CPU with peaks above 200%, and the bridge/controller adding smaller loads. That CPU contention is the primary demonstrated system bottleneck and can reduce detector/debug-image cadence and make the HUD feel delayed.

The RTX is not idle overall: it is used by YOLO. However, Gazebo's rendering path is separate and currently does not consume the RTX according to both OGRE's renderer string and NVIDIA telemetry.

## Recommended next investigations (not applied)

1. Preserve the current evidence and add timestamp-only instrumentation at camera receipt, detector start/end, debug-image publish, and HUD callback/display boundaries. This is needed to split true low FPS from stale-frame latency.
2. Capture a normal full-stack run with the same process and topic measurements, keeping Gazebo's current renderer unchanged, to quantify CPU scheduling and bridge cost.
3. If changing graphics configuration is later considered, benchmark the current AMD D3D12 WSLg path against the proposed path with identical world, camera, and ROS workload. Do not assume that moving Gazebo to the RTX will improve the end-to-end HUD until that comparison exists.

No optimization, configuration change, or production-code change was made as part of this verification.
