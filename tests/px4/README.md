# PX4 Tests

PX4 and MAVLink integration checks live here, separated from application code.

Run a test from the repository root so shared modules and paths resolve:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=.:$PYTHONPATH python3 tests/px4/<script>.py
```

Preserve the ROS-provided `PYTHONPATH`: replacing it with only `.` hides
generated message packages such as `geometry_msgs`.

The scripts cover baseline MAVLink/MAVSDK flights, offboard climb and axis
checks, live-message verification, the motion-arbiter state machine, and the
camera-driven approach check (`test_approach_camera.py`, uses the
`person_tracking_approach` world whose actor paces 3–8 m ahead of the drone).

These are live SITL tests. They may start PX4/Gazebo processes and require the
PX4-Autopilot checkout at `/home/tungt/PX4-Autopilot`.

## Gotchas

- **MAV_CMD_NAV_TAKEOFF via pymavlink must use all-NaN params** (plus a
  `MIS_TAKEOFF_ALT` PARAM_SET). PX4 1.14 ACKs the command with a finite
  `param7` but the takeoff task then never generates setpoints — thrust stays
  at 0 and `COM_DISARM_PRFLT` auto-disarms 10 s after arming. MAVSDK works
  because it sends NaN params. Verified by traffic capture on 2026-08-25.
- **PX4 ends the takeoff phase ~1 acceptance radius below the commanded
  altitude** (`NAV_MC_ALT_RAD`, 0.8 m default) and loiters there: commanding
  4.0 m only ever holds ~3.2 m. Command 5.0 m when the test needs ≥ 3.8 m
  of actual altitude.
- **PX4 only resolves `PX4_GZ_WORLD` against its own worlds dir**
  (`Tools/simulation/gz/worlds`); repo worlds must be launched separately
  first — PX4 then detects the running world via `gz topic -l` and only
  starts the bridge (see `test_approach_camera.py`).
- **Run tests on an idle machine.** A leftover `px4`/`gz sim` process halves
  the CPU, collapses the real-time factor, and makes wall-clock thresholds
  (displacement per 3 s, climb per 15 s) fail spuriously. The scripts pkill
  stale sims on startup and kill their own sim in a `finally` block — keep
  that guarantee when editing them.
- Wall-clock test windows measure wall time while physics runs on sim time;
  on a loaded machine expect proportionally less simulated motion.
