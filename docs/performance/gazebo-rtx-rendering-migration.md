# Gazebo RTX Rendering Migration

**Date:** 2026-09-15  
**Repository:** `/home/tungt/drone-project`  
**Branch:** `refactor/restore-3d-pinhole-tracking`

## Executive Verdict

The RTX override is **not integrated**. The checked-out `start_stack.sh` is restored to the known baseline. Selecting the RTX adapter works for isolated Gazebo server rendering, but the combined server + GUI path is not reliable in this WSL2 session: sustained runs ended with a Gazebo Ruby SIGSEGV, and the GUI path is therefore not accepted as a migration.

These are separate criteria:

1. **RTX adapter selected:** demonstrated by OGRE2 and NVIDIA telemetry.
2. **Gazebo functional:** demonstrated for server-only runs, but not reliably for sustained server + GUI runs.
3. **Gazebo performance improved:** not demonstrated; no integration is justified.

## Phase 1: Baseline Restoration

`git diff -- start_stack.sh` was empty before and after the investigation. The exact migration block found in the prior change was:

```bash
if [[ "${USE_SOFTWARE_RENDERING:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  unset MESA_D3D12_DEFAULT_ADAPTER_NAME
else
  unset LIBGL_ALWAYS_SOFTWARE
  if [[ "${USE_NVIDIA_GPU:-1}" == "1" ]]; then
    export GAZEBO_GPU_ADAPTER="${GAZEBO_GPU_ADAPTER:-NVIDIA GeForce RTX 4060 Laptop GPU}"
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${GAZEBO_GPU_ADAPTER}"
  else
    unset MESA_D3D12_DEFAULT_ADAPTER_NAME
  fi
fi
```

That block is absent. The current rendering environment is exactly:

```bash
if [[ "${USE_SOFTWARE_RENDERING:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  unset LIBGL_ALWAYS_SOFTWARE
fi
```

The launcher starts the project world with `gz sim -r -s "${PROJECT_DIR}/gazebo/worlds/${WORLD_NAME}.sdf"` and, unless `HEADLESS=1`, starts the GUI with `gz sim -g`. No ROS 2 node, YOLO, PX4, SDF, camera, QoS, or control file was changed.

## Phase 2: Fresh Baseline Measurement

World: `gazebo/worlds/person_tracking_path.sdf`  
Environment: `MESA_D3D12_DEFAULT_ADAPTER_NAME` unset and `LIBGL_ALWAYS_SOFTWARE` unset, with the same `GZ_SIM_RESOURCE_PATH`, server config, plugin path, and library path as `start_stack.sh`.

Direct server command:

```bash
gz sim -r -s /home/tungt/drone-project/gazebo/worlds/person_tracking_path.sdf
```

Observed server-only baseline:

- `/camera/image_raw` exists and publishes at **28.8-29.1 Hz** (initial samples 30.45 Hz).
- `/stats` `real_time_factor`: **0.9985-1.0014**.
- Gazebo server Ruby process: **about 86.6% CPU**.
- OGRE2: `GL_RENDERER = D3D12 (AMD Radeon(TM) 780M)`.
- RTX telemetry: **0% GPU, 0 MiB VRAM, no Gazebo process**.

The GUI-only default-adapter process stayed alive during the probe. A short manual server + GUI run also stayed alive, but sustained combined runs later crashed a Gazebo Ruby process; this is recorded as a current environment stability issue rather than evidence of a successful migration.

## Phase 3: Isolated RTX Experiment

The only experimental change was:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA GeForce RTX 4060 Laptop GPU"
```

### A. Server only / actual camera world

- Startup: **PASS** for the measured run.
- World and camera: **PASS**; `/camera/image_raw` exists.
- Camera frequency: **28.5-29.1 Hz**.
- RTF: **0.9992-1.0013**.
- Gazebo Ruby CPU: **about 90.3%**.
- OGRE2: `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`.
- NVIDIA telemetry while running: **35% GPU, 223 MiB VRAM**; the active process was the Gazebo Ruby renderer.

This proves adapter selection and attributable server-side GPU activity. It does not prove GUI stability or an end-to-end FPS improvement.

### B. GUI only

`gz sim -g <world>` stayed alive for the 15-second probes with both default and RTX environments. GUI-only startup has no camera or world-clock publisher to measure. No renderer string is emitted by the GUI process in the available OGRE log.

### C. Server + GUI

The normal two-process arrangement was tested with identical world and environment setup.

- A short default-adapter run produced **29.7-30.6 Hz** camera samples and RTF near 1.0.
- A short RTX run produced **28.0-30.0 Hz** camera samples and RTF near 1.0, with NVIDIA activity present.
- Sustained combined runs were **not reliable**: both default and RTX runs eventually produced a Gazebo Ruby **SIGSEGV**. Kernel diagnostics identified the fault in `libgcc_s.so.1`; the RTX run's crashed process was the Gazebo server Ruby process. This prevents accepting the GUI path as stable.

### D. Explicit headless sensor rendering

With `gz sim -r -s --headless-rendering`, the default adapter reached **30.99 Hz** and RTF **0.9990-1.0013**. The corresponding RTX probe selected the NVIDIA renderer but crashed before producing a valid camera/RTF sample. This is additional evidence that the RTX path is not reproducibly stable under repeated rendering experiments, even though a non-explicit-headless server run was successful.

## Phase 4: Diagnosis of the Reported ~11 FPS

The ~11 FPS value was **not reproduced by the isolated Gazebo camera server**: both adapters delivered approximately 29-30 Hz with the project camera and RTF near 1.0. Therefore the 11 FPS loss cannot be attributed to the OGRE2 sensor alone from these measurements.

The controlled split localizes the risk to the display/combined path:

- Server-only: near 30 Hz on AMD and RTX.
- Server + GUI: near 28-30 Hz during short healthy windows, but sustained Ruby crashes occur.
- The project HUD/YOLO/PX4 stack was intentionally not started in this phase, so no claim is made about image transport, YOLO, HUD, or CPU contention in the reported 11 FPS observation.

`MESA_D3D12_DEFAULT_ADAPTER_NAME` is process-inherited, so exporting it before `start_stack.sh` would affect both `gz sim -s` (OGRE2 sensor rendering) and `gz sim -g` (GUI/WSLg display path). Selecting the RTX D3D12 adapter for the render server is therefore not equivalent to making the complete visual pipeline faster. On this hybrid AMD-display/NVIDIA-compute WSLg system, cross-adapter presentation and driver synchronization remain plausible mechanisms, but the observed crashes are the directly measured blocker.

## Verdict

| Criterion | Result |
|---|---|
| RTX adapter visible | **PASS** - OGRE2 reported NVIDIA RTX 4060; NVIDIA showed 35% / 223 MiB during server-only run |
| Gazebo server starts | **PASS (isolated run)** - server-only RTX run started and published topics |
| Gazebo GUI opens | **FAIL migration acceptance** - the GUI process stayed alive in short probes, but sustained combined runs crashed a Gazebo Ruby process and no visual window confirmation is claimed |
| World loads | **PASS server-only; FAIL migration acceptance** because combined GUI stability is not reliable |
| Camera works | **PASS server-only** - `/camera/image_raw` present |
| Camera FPS | **Baseline AMD: 28.8-29.1 Hz; RTX server: 28.5-29.1 Hz; short combined: 28.0-30.6 Hz** |
| RTF | **Baseline: 0.9985-1.0014; RTX server: 0.9992-1.0013** |
| RTX utilization | **0% baseline; 35% RTX server-only sample** |
| RTX VRAM | **0 MiB baseline; 223 MiB RTX server-only sample** |
| Performance regression | **YES for migration acceptance** - sustained combined rendering is unstable; no improvement measured |
| Final migration status | **FAIL** - RTX override remains disabled |

## Revert / Current State

No revert command is needed because `start_stack.sh` already contains the baseline block and has no RTX override. Verify with:

```bash
git diff --check
git diff --stat
git diff -- start_stack.sh
```

The only remaining work before any future integration would be a separately controlled WSLg/driver investigation that can demonstrate stable server + GUI operation and at least baseline camera/RTF performance. Until then, keep the default adapter path.


