# Current runtime failure fix

## Root causes

The camera transport was functional. A live sample from `/camera/image_raw` was
`640x480`, `rgb8`, with increasing Gazebo timestamps and non-zero pixels
(sample min 30, max 255, mean about 155.8). A saved diagnostic frame showed the
project world and the walking actor, so the black HUD was not a Gazebo sensor or
bridge failure.

The HUD failure was in the GUI lifecycle. OpenCV's Qt window was created by the
ROS thread but `imshow`/`waitKey` were called from a daemon display thread.
That is unsupported by the Qt backend used in this WSLg environment and could
leave a live black window. The HUD now keeps the bounded latest-frame buffer,
but performs GUI work from a ROS timer on the node's executor thread and logs
throttled callback/display/exception counters.

The no-takeoff failure was a readiness deadlock. The configured mode is
`auto_track=false` (manual target selection), so YOLO intentionally publishes
no `/tracking/error` while the detector state is `NO_TARGET`. MotionArbiter was
waiting for `/tracking/error` before auto-takeoff and therefore timed out with
`msgs=0`. In manual-selection mode it now waits for the detector's fresh
`/tracking/target_handle` heartbeat instead. Automatic-tracking mode still
requires a fresh tracking error, preserving its safety gate.

## Evidence and verification

Before the fix, the running stack logged:

```text
[AUTO-TAKEOFF] [WAITING_FOR_VISION] ... /tracking/error
[AUTO-TAKEOFF] [DO NOT TAKEOFF] ... (msgs=0, ready=False)
```

The same run showed valid camera frames and CUDA YOLO initialization, but
`state=NO_TARGET` because no target had been selected. An explicit existing
`/teleop/flight_action: TAKEOFF` command independently proved the PX4 path was
healthy: OFFBOARD was accepted, PX4 armed, altitude progressed through 1, 2,
and 3 m, and the arbiter reported `Drone Airborne at 3.8m`.

After the fix, the actual `./start_stack.sh` run logged:

```text
[AUTO-TAKEOFF] [WAITING_FOR_VISION] ... /tracking/target_handle
[AUTO-TAKEOFF] [VISION_READY] ... source=/tracking/target_handle
[TAKEOFF] OFFBOARD climb altitude: 1.01 m
[TAKEOFF] OFFBOARD climb altitude: 2.05 m
[TAKEOFF] OFFBOARD climb altitude: 3.01 m
[TAKEOFF] Drone Airborne at 3.8m. State: STANDBY.
```

The HUD reported sustained delivery with no display exceptions, for example
`callbacks=721 displayed=710 ... exceptions=0`, and the WSLg window was present
(`Live Drone Camera POV (Direct Stream)`, 960x720). Diagnostic debug frames
were non-black (min 0, max 255, mean about 166.3).

Observed post-fix rates in the full CUDA stack were approximately:

| Signal | Measured |
|---|---:|
| `/camera/image_raw` | 13–15.6 Hz during this loaded run |
| `/tracking/debug_image` | 8.7–11.1 Hz |
| HUD display callbacks | about 29–30 Hz while frames were available |
| MotionArbiter control period | about 0.10 s |
| RTF / Gazebo isolated baseline | unchanged, about 1.0 / 29 Hz |

The reduced full-stack topic rates are a separate CPU/YOLO workload issue; this
fix does not claim to solve the previously measured performance bottleneck.
RTX adapter selection remains disabled in production and no WSL or GPU
configuration was changed.

## Files changed

- `vision_tracking/live_camera_hud_node.py`: ROS-thread GUI scheduling,
  latest-frame rendering, and throttled exception diagnostics.
- `vision_tracking/motion_arbiter_node.py`: mode-aware perception readiness;
  manual-selection mode uses the detector heartbeat, auto-track mode retains
  the tracking-error requirement.

No SDF/world, camera geometry, YOLO/ByteTrack algorithm, PX4 parameters,
control law, QoS bridge, or RTX override was changed by this fix.

## Reproduction and revert

Run with RTX disabled:

```bash
HEADLESS=1 SHOW_HUD=1 LAUNCH_QGC=0 SIM_REALISM=0 ./start_stack.sh
```

To revert only this runtime fix, restore the two files above from the prior
revision, rebuild `ros2_ws` with `colcon build --symlink-install`, and rerun the
same command. The known-good Gazebo rendering environment remains intact.

## Remaining issues

The repository's phase integration validator still reports pre-existing target
identity regression cases (tracker churn/crossing/loss horizon). Those are
outside this runtime failure fix and were not modified here. Full-stack image
rates remain below the isolated Gazebo ~29 Hz baseline and should be profiled
separately after functional acceptance.
