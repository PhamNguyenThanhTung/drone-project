# PX4 Tests

PX4 and MAVLink integration checks live here, separated from application code.

Run a test from the repository root so shared modules and paths resolve:

```bash
PYTHONPATH=. python3 tests/px4/<script>.py
```

The scripts cover baseline MAVLink/MAVSDK flights, offboard climb and axis
checks, live-message verification, and the motion-arbiter state machine.

These are live SITL tests. They may start PX4/Gazebo processes and require the
PX4-Autopilot checkout at `/home/tungt/PX4-Autopilot`.
