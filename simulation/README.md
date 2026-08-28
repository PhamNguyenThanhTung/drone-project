# Realism and safety test ladder

`realism.yaml` is a deterministic fault-injection profile. It adds camera
latency/frame loss/blur and, when raw topics are supplied, IMU bias + random
noise + drift, GPS noise/dropouts, and barometer noise. Start it alongside the
stack with:

```bash
ros2 run vision_tracking sim_realism_node --ros-args --params-file simulation/realism.yaml
```

Point the detector at the degraded stream with
`YOLO_IMAGE_TOPIC=/simulation/camera/image`. Keep `seed` fixed for repeatable
regression tests. Wind/turbulence and ground-effect settings belong in the
Gazebo/PX4 airframe profile; `vehicle_profile.yaml` records the parameters and
clearly separates the x500 SITL baseline from measurements that must come from
the real drone (mass, inertia, thrust curve, and battery discharge curve).

Required validation gates, in order:

1. SITL: nominal and realism profile; verify estimator innovation and failsafes.
2. HIL: real flight controller, props removed, current-limited bench supply.
3. Propeller-less test: arm, mode changes, link/sensor/target-loss injection.
4. Safety harness: tethered low hover in a netted area with an independent kill switch.
5. Geofence flight: conservative polygon/radius, RTL and loss-of-GPS/MAVLink checks.

Exercise MAVLink loss by stopping the bridge process, target loss by dropping all
camera frames, and GPS loss by setting the GPS dropout probability to `1.0`.
Enable PX4 geofence parameters in the test vehicle configuration and record
the parameter dump with each run. Never skip a gate or fly with unmeasured
airframe constants.

PX4-native failure injection is available after SITL starts:

```bash
python3 simulation/inject_failure.py gps off
python3 simulation/inject_failure.py gps ok
python3 simulation/inject_failure.py mavlink_signal off
```

For companion-computer parity, launch with a restricted CPU set and inference
rate, for example `COMPANION_CPUSET=0,1 YOLO_MAX_FPS=15 YOLO_IMGSZ=416`.
Use the target computer's real thermal/power mode or vendor tooling for GPU
power and memory limits; those controls are hardware-specific.

## PX4 checkout patch

The project requires simulated GPS enabled for the `x500_flow` airframe. Apply
the repository patch to a matching PX4 checkout before building SITL:

```bash
cd /path/to/PX4-Autopilot
git apply /path/to/drone-project/patches/4021_gz_x500_flow_gps.patch
make px4_sitl gz_x500_flow
```

The patch targets the stock PX4 `v1.16.2` airframe file. Use
`git apply --check` first when applying it to another PX4 revision.
