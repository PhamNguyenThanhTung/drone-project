# Current Architecture

## Required Layer Diagram

```text
CAMERA
  |
  v
YOLO DETECTION
  |
  v
TRACKER (Ultralytics ByteTrack, persist=True)
  |
  v
TARGET SELECTION (HUD click/key, or area*confidence auto policy)
  |
  v
TARGET LOCK (ByteTrack ID plus short IoU/proximity remap)
  |
  v
GEOMETRY (pixel error/area; pinhole ground estimate for telemetry/recovery)
  |
  v
MOTION (MotionArbiter state machine and visual servo)
  |
  v
PX4 (MAVLink Offboard LOCAL_NED velocity/yaw-rate)
```

## Actual Runtime Graph

```text
Gazebo Harmonic
  x500 camera: 640x480, 30 Hz, HFOV 2.0 rad, pitch 0.65 rad
      |
      | Gazebo transport Image
      v
ros_gz_bridge parameter_bridge
      |
      +--> /camera/image_raw [sensor_msgs/Image, bridge]
              |
              +--> sim_realism_node (optional) --> /simulation/camera/image
              |                                      |
              +--------------------------------------+ 
                                                     v
                                     yolo_detector_node
                                     YOLOv8n + ByteTrack
                                     persist=True, class 0 person
                                       |             |
                         /tracking/debug_image       | /tracking/select_target
                         [Image]                     | [Int32]
                                       v             v
                                 live_camera_hud  <---+---> yolo_detector_node
                                       |                  |
                click_point/select/cmd/action/goto       |
                                       v                  |
                               motion_arbiter <-----------+
                               /tracking/error [Point]
                               /tracking/select_target [Int32]
                               /teleop/cmd_vel [Twist]
                               /teleop/flight_action [String]
                               /tracking/goto_gps [Point]
                                  |
                                  +--> /tracking/motion_state [String]
                                  +--> /tracking/gps [NavSatFix]
                                  +--> /tracking/control_health [String JSON]
                                  +--> /tracking/target_geometry [String JSON]
                                  +--> /tracking/ground_distance [Point]
                                  |
                                  v
                         pymavlink UDP 14540
                                  |
                                  v
                         PX4 SITL Offboard
                                  |
                                  v
                         simulated drone / next camera frame
```

## Ownership and Boundaries

| Capability | Current owner | Boundary |
|---|---|---|
| Person detection | `yolo_detector_node` | YOLO result boxes, class/confidence |
| Short-term association | Ultralytics ByteTrack inside detector | Per-frame `track_id` |
| Initial target choice | Detector plus HUD command | Manual ID or `area*confidence` |
| Lock/rebind | Detector | `manual_target_id`, EMA box, IoU/proximity for 5 s |
| Metric geometry | Arbiter plus `pinhole_geometry.py` | Consumes aggregate pixel error; no candidate list |
| Flight state/control | `motion_arbiter_node.py` | Sole normal MAVLink writer |
| Visualization/operator | `live_camera_hud_node.py` | Publishes commands, receives state/GPS/image |
| Fault injection | `sim_realism_node.py` | Optional camera/sensor perturbation |
| Vehicle simulation | Gazebo + PX4 SITL | Camera, physics, telemetry |

## Important Architectural Gaps

1. Detection and tracking identity are internal to the detector; `/tracking/error` drops the ID and timestamp, so the controller cannot independently verify which person generated the command.
2. `target_id` is a tracker ID, not a mission identity. No `target_handle`, reference embedding, candidate list, or explicit rejection state exists.
3. `/tracking/select_target` is both an input command and detector-generated feedback, creating a two-publisher control loop.
4. Pinhole geometry is calculated after target selection and does not participate in multi-candidate association. It can reduce recovery ambiguity only after a candidate has already been chosen.
5. Configuration is duplicated across source defaults, `tracking_stack.yaml`, and per-node YAML files. `start_stack.sh` also injects device and takeoff overrides.
6. Root scripts are compatibility shims for tests. They are not separate production implementations, but broad linting treats them as source.

## Control and Safety Sequence

```text
Fresh /tracking/error
  -> update EMA error, area, last-seen time
  -> estimate pinhole geometry and publish telemetry
  -> 10 Hz steady-clock tick reads state under lock
  -> choose MANUAL / TRACKING / MANUAL_GOTO / STANDBY output
  -> apply velocity/yaw/altitude limits and slew limiters
  -> send exactly one normal LOCAL_NED setpoint

Vision age > 0.40 s:
  -> recovery substates may back up or advance to turn point

Target age > 5.0 s:
  -> STANDBY hover/altitude hold; no automatic new-person selection

Manual Twist fresh:
  -> MANUAL authority; detector suppresses lock re-acquisition
```

The state machine is operationally safer than automatic switching, but it is not the requested richer `NO_TARGET -> CANDIDATES -> ... -> REID_VERIFY` model. Any future change should preserve the single MAVLink-writer boundary and explicit manual priority.
