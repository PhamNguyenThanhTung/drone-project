# Gazebo FPS Fix

## Symptom

The project was observed with a user-visible Gazebo/HUD cadence near 11 FPS even though the Gazebo camera sensor itself could produce about 29 Hz. The important distinction is between Gazebo sensor output, ROS image delivery, debug-image generation, and display refresh.

## Confirmed Bottleneck

The original HUD subscribed to /tracking/debug_image and did all of the following inside the ROS image callback:

- cv_bridge conversion
- full-frame OpenCV overlays
- cv2.imshow
- cv2.getWindowImageRect
- cv2.waitKey

The debug-image publisher and HUD subscriber also used the default reliable DDS profile. A slow WSLg/OpenCV consumer could therefore block the YOLO callback and add backpressure to the visualization stream.

## Fix Implemented

Only visualization code and visualization-topic QoS were changed:

1. live_camera_hud_node.py now stores the newest converted frame under a lock and returns immediately from the ROS callback.
2. A daemon display thread renders at a bounded display_fps (default 30 Hz), dropping stale frames.
3. cv2.getWindowImageRect is no longer called per frame; it is queried only on mouse clicks.
4. /tracking/debug_image uses best-effort, depth-1 QoS on both the YOLO publisher and HUD subscriber.
5. display_fps: 30.0 is explicit in tracking_stack.yaml.

Detection, ByteTrack, target identity, control, PX4, camera geometry, SDF, and ROS control topics were not changed.

## Measurements

All stack measurements used the actual person_tracking_path.sdf, HEADLESS=1, SIM_REALISM=0, CUDA YOLO, and a direct rclpy best-effort image counter.

| Configuration | Camera Hz | Debug image Hz | Gazebo CPU | YOLO CPU | HUD CPU | RTX |
|---|---:|---:|---:|---:|---:|---|
| Full stack, HUD disabled | 16.56 | 9.51 | 87.6% | 41.5% | N/A | 0%, 149 MiB |
| HUD enabled, before final QoS/display fix | 11.89 | 3.52 | 87.8% | 69.0% | 13-40% | 0-6%, 147 MiB |
| HUD enabled, async display + throttled geometry | 12.85-13.58 | 4.33-5.53 | 85.6-89.9% | 39-54% | 6-8% | 19-32%, 147-149 MiB |
| HUD enabled, async display + best-effort depth-1 debug QoS | **16.86** | **7.64** | **87.2%** | 62.7% | 5.0% | 17%, 149 MiB |

The final fix approximately doubled debug-image delivery versus the pre-fix HUD run (3.52 to 7.64 Hz) and restored camera delivery to the no-HUD full-stack level (16.86 versus 16.56 Hz). It removes the HUD-induced backpressure and materially improves the visual stream, but the full stack remains CPU/YOLO limited below the isolated 29 Hz camera capability.

The isolated Gazebo server remains near 29 Hz and RTF near 1.0. The RTX adapter is not integrated into start_stack.sh; server-only RTX rendering was previously measured near 29 Hz, but the combined RTX GUI path remains unstable.

## Stability

The final HUD run reached the normal launcher READY state, kept Gazebo, YOLO, MotionArbiter, bridge, and HUD alive for the measurement window, and exited through the normal cleanup path. No HUD exception or crash was logged.

The WSLg RTX server+GUI SIGSEGV remains unresolved and is independent of this HUD backpressure fix. The production RTX override therefore remains disabled.

## Revert

To revert the performance fix, remove the HUD changes in live_camera_hud_node.py, restore the original reliable depth-1 subscription, remove the best-effort depth-1 debug publisher in yolo_detector_node.py, and remove display_fps from tracking_stack.yaml. No change is needed in start_stack.sh.

Verification commands: git diff --check and bash -n start_stack.sh.

