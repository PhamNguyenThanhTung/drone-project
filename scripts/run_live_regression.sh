#!/bin/bash
PROJECT_DIR="/home/tungt/drone-project"
cd "${PROJECT_DIR}"

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"

MODE="${1:-normal}"  # normal or realism
LOG_OUT="/tmp/start_stack_live_test_${MODE}.log"
rm -f "${LOG_OUT}" /tmp/ros_tracking_stack.log /dev/shm/fastrtps* /dev/shm/sem.fastrtps*

if [[ "${MODE}" == "realism" ]]; then
  echo "=== LAUNCHING ./start_stack.sh (HEADLESS=1, SIM_REALISM=1) ==="
  SIM_REALISM=1 HEADLESS=1 ./start_stack.sh > "${LOG_OUT}" 2>&1 &
else
  echo "=== LAUNCHING ./start_stack.sh (HEADLESS=1, SIM_REALISM=0) ==="
  SIM_REALISM=0 HEADLESS=1 ./start_stack.sh > "${LOG_OUT}" 2>&1 &
fi
STACK_SH_PID=$!
echo "start_stack.sh PID: ${STACK_SH_PID}"

READY=0
for i in $(seq 1 60); do
  if grep -q 'SẴN SÀNG!' "${LOG_OUT}" 2>/dev/null; then
    echo "Full stack reached READY state after ${i} seconds!"
    READY=1
    break
  fi
  sleep 1
done

if [[ "${READY}" -ne 1 ]]; then
  echo "LỖI: start_stack.sh không đạt trạng thái READY sau 60s!"
  tail -40 "${LOG_OUT}"
  kill -TERM "${STACK_SH_PID}" 2>/dev/null || true
  exit 1
fi

sleep 3

echo ""
echo "======================================================="
echo "   1. ROS 2 NODE LIST VERIFICATION (${MODE} mode)      "
echo "======================================================="
ros2 node list

echo ""
echo "======================================================="
echo "   2. ROS 2 TOPIC LIST VERIFICATION (${MODE} mode)     "
echo "======================================================="
ros2 topic list

echo ""
echo "======================================================="
echo "   3. ROS 2 TOPIC INFO (12 BASELINE TOPICS)           "
echo "======================================================="
declare -a check_topics=(
  "/camera/image_raw"
  "/simulation/camera/image"
  "/tracking/error"
  "/tracking/debug_image"
  "/tracking/select_target"
  "/tracking/click_point"
  "/tracking/goto_gps"
  "/teleop/cmd_vel"
  "/teleop/flight_action"
  "/tracking/motion_state"
  "/tracking/gps"
  "/tracking/control_health"
)

for tp in "${check_topics[@]}"; do
  echo "--- Topic: ${tp} ---"
  ros2 topic info "${tp}" 2>&1 || true
done

echo ""
echo "======================================================="
echo "   4. RUNTIME PARAMETER VERIFICATION                   "
echo "======================================================="
echo -n "motion_arbiter kp: "
ros2 param get /motion_arbiter kp 2>&1 || true
echo -n "motion_arbiter kp_area: "
ros2 param get /motion_arbiter kp_area 2>&1 || true
echo -n "motion_arbiter takeoff_alt: "
ros2 param get /motion_arbiter takeoff_alt 2>&1 || true
echo -n "motion_arbiter control_period_s: "
ros2 param get /motion_arbiter control_period_s 2>&1 || true
echo -n "motion_arbiter mavlink: "
ros2 param get /motion_arbiter mavlink 2>&1 || true
echo -n "motion_arbiter small_box_max_speed: "
ros2 param get /motion_arbiter small_box_max_speed 2>&1 || true
echo -n "motion_arbiter wait_for_vision_before_takeoff: "
ros2 param get /motion_arbiter wait_for_vision_before_takeoff 2>&1 || true

echo -n "yolo_detector_node model_path: "
ros2 param get /yolo_detector_node model_path 2>&1 || true
echo -n "yolo_detector_node conf: "
ros2 param get /yolo_detector_node conf 2>&1 || true
echo -n "yolo_detector_node iou: "
ros2 param get /yolo_detector_node iou 2>&1 || true
echo -n "yolo_detector_node device: "
ros2 param get /yolo_detector_node device 2>&1 || true
echo -n "yolo_detector_node classes: "
ros2 param get /yolo_detector_node classes 2>&1 || true
echo -n "yolo_detector_node image_topic: "
ros2 param get /yolo_detector_node image_topic 2>&1 || true

if [[ "${MODE}" == "realism" ]]; then
  echo -n "simulation_realism camera_delay_ms: "
  ros2 param get /simulation_realism camera_delay_ms 2>&1 || true
  echo -n "simulation_realism camera_drop_probability: "
  ros2 param get /simulation_realism camera_drop_probability 2>&1 || true
  echo -n "simulation_realism seed: "
  ros2 param get /simulation_realism seed 2>&1 || true
fi

echo -n "live_camera_hud topic: "
ros2 param get /live_camera_hud topic 2>&1 || true
echo -n "live_camera_hud teleop_speed: "
ros2 param get /live_camera_hud teleop_speed 2>&1 || true
echo -n "live_camera_hud map_radius_m: "
ros2 param get /live_camera_hud map_radius_m 2>&1 || true

echo ""
echo "======================================================="
echo "   5. LOGS & TELEMETRY VERIFICATION                    "
echo "======================================================="
ls -lh /tmp/ros_tracking_stack.log /tmp/motion_arbiter.log /tmp/yolo.log /tmp/ros_bridge.log /tmp/camera_hud.log
ls -lh /home/tungt/drone-project/logs/tracking_diagnostics.jsonl

echo ""
echo "======================================================="
echo "   6. CLEAN SHUTDOWN TRIGGER                           "
echo "======================================================="
echo "Stopping start_stack.sh (PID ${STACK_SH_PID})..."
kill -TERM "${STACK_SH_PID}" 2>/dev/null || true
wait "${STACK_SH_PID}" 2>/dev/null || true
sleep 5

echo ""
echo "======================================================="
echo "   7. POST-SHUTDOWN PROCESS CLEANLINESS CHECK          "
echo "======================================================="
stale_found=0
for pat in "px4" "gz sim" "tracking_stack.launch" "motion_arbiter" "yolo_detector" "parameter_bridge"; do
  matches=$(pgrep -f "$pat" || true)
  if [[ -n "$matches" ]]; then
    echo "WARNING: Remaining process matching '$pat': $matches"
    stale_found=1
  else
    echo "  Clean: '$pat' -> NONE"
  fi
done

if [[ "$stale_found" -eq 0 ]]; then
  echo "SUCCESS: All processes cleanly terminated, no orphans!"
fi
