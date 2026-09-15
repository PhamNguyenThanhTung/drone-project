# Gazebo and HUD Bottleneck Table

Values marked `unknown` were not measured in a clean live full-stack run. Existing logs and temporary probes are labeled by provenance.

| Component | Measured FPS / rate | Latency | CPU | GPU / VRAM | Observed bottleneck | Confidence | Evidence |
|---|---:|---:|---:|---|---|---:|---|
| Gazebo physics/world | 250 Hz configured; `/world/.../stats` iterations consistent with 250 Hz | 4 ms step; RTF 0.84-1.00 in isolated probe | 53-82% headless probe; 83-86% full-stack mean; peaks 101-106% | Not attributed by `nvidia-smi`; VRAM unknown | CPU contention from physics + sensors + rendering | High for CPU, low for GPU | Host Gazebo probe; `phase3_process_samples*.csv` |
| Gazebo GUI/render | No direct FPS counter captured | Unknown | Server ~41%, GUI ~60% at one GUI probe instant | `nvidia-smi`: 0%, no process attribution | Render backend/WSLg path unresolved; high GUI CPU | Medium | Temporary GUI probe; missing `glxinfo`/`eglinfo` |
| Gazebo camera sensor | 28-30 Hz isolated | Frame timestamp latency not captured | Included in Gazebo CPU | Unknown | Not intrinsically limited to 10 FPS | High | `/camera/image_raw` frequency probe |
| `ros_gz_bridge` | Raw topic rate not captured during full stack; isolated Gazebo source ~29-30 Hz | Prior DDS IPC <2 ms when host available | 4.1-4.2% mean, max 6% | Unknown | Low relative cost; serialization/copy remains | Medium-high | Process samples and forensic report |
| YOLO preprocessing | Unknown | Not separately instrumented | Included in detector process | CUDA selected in prior run; VRAM unknown | Image conversion/model input work unmeasured | Low | Source inspection; no stage timer |
| YOLO inference | Full-stack output often near 15 FPS in prior forensic run | 18-25 ms warm CUDA; startup 3.8-4.9 s | Detector 79-96% mean, peaks 211-233% | RTX 4060 available in prior logs; live utilization unknown | Major bursty compute cost and scheduler contention | High for cost, medium for exact rate | Forensic report and process samples |
| ByteTrack/postprocess | No independent rate | Unknown | Included in YOLO process | Unknown | Not expected to dominate at current person count | Medium-low | Same callback as YOLO; no separate timer |
| `/tracking/debug_image` | Unknown in current live run; bounded by detector output | Source stamp not used by HUD | Publisher cost included in detector | Unknown | Debug rendering/publish inherits detector cadence | Medium | Source and missing live topic measurement |
| HUD input callback | Unknown; user reports ~10 FPS | No callback duration/age metrics | HUD not captured as a separate sampler role | Unknown | Synchronous callback and GUI calls can throttle | High for mechanism, low for share | `live_camera_hud_node.py` source |
| HUD OpenCV render | Unknown display FPS | `imshow`/`waitKey` timing not recorded | Not isolated | WSLg path unknown | UI work is serialized with ROS callback | High for mechanism | Source inspection |
| MotionArbiter | Control 10 Hz nominal | Prior control near 0.104 s in one trial; other CSVs show multi-second gaps during startup/stalls | 6.0-6.1% mean, max 8% | Unknown | Not primary visual FPS bottleneck | High | Process samples and diagnostics |
| PX4 SITL | Telemetry/setpoint nominal 10 Hz | Startup/stall gaps documented | 6.5-6.6% mean, peaks 24.8% | Unknown | Secondary host load; not visual bottleneck | Medium-high | Process samples |
| End-to-end camera -> HUD | Unknown | Cannot derive precisely without timestamps | Combined CPU contention confirmed | GPU path unresolved | Detector cadence plus synchronous HUD; stale-frame age unknown | Medium | Raw-camera probe, prior 15 FPS forensic rate, source inspection |

## Interpretation

- `Gazebo camera ~=30 Hz` and `HUD ~=10 FPS` are compatible: the HUD consumes `/tracking/debug_image`, which is produced only after YOLO and drawing.
- High Gazebo and YOLO CPU percentages are additive contention, not proof that either is exactly 10 FPS.
- GPU utilization cannot be inferred from CUDA availability or from an idle `nvidia-smi` sample. A live host-side trace is still required.
- FPS and latency are different metrics. The current HUD has no source timestamp or queue-age telemetry, so visual lag cannot be quantified from its watermark alone.
