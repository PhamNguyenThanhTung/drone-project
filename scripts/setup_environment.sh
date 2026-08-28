#!/bin/bash
# ==============================================================================
# Script: setup_environment.sh
# Purpose: Comprehensive environment and dependency setup for the Drone Project.
#          Sets up ROS 2 Humble workspace, Python packages, Gazebo paths, and
#          applies PX4 airframe GPS patches consistently across any workstation.
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}   DRONE TRACKING PROJECT: AUTOMATED ENVIRONMENT SETUP          ${NC}"
echo -e "${BLUE}================================================================${NC}"
echo -e "Project Root: ${YELLOW}${PROJECT_DIR}${NC}"
echo ""

# 1. Check ROS 2 Humble
echo -e "${BLUE}[1/5] Checking ROS 2 Installation...${NC}"
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source "/opt/ros/humble/setup.bash"
    echo -e "${GREEN}[OK] Found ROS 2 Humble at /opt/ros/humble${NC}"
else
    echo -e "${RED}[ERROR] ROS 2 Humble not found at /opt/ros/humble.${NC}"
    echo "Please install ROS 2 Humble before proceeding: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# 2. Check and install Python dependencies
echo ""
echo -e "${BLUE}[2/5] Checking Python Dependencies (requirements.txt)...${NC}"
if command -v pip3 >/dev/null 2>&1; then
    pip3 install -r "${PROJECT_DIR}/requirements.txt" --quiet
    echo -e "${GREEN}[OK] Python packages (ultralytics, pymavlink, torch, opencv) verified.${NC}"
else
    echo -e "${YELLOW}[WARN] pip3 not found. Skipping pip install. Ensure dependencies are satisfied manually.${NC}"
fi

# 3. Apply PX4 Airframe Patch
echo ""
echo -e "${BLUE}[3/5] Applying PX4 Airframe Patch (GPS for 4021_gz_x500_flow)...${NC}"
"${PROJECT_DIR}/scripts/apply_px4_patch.sh" --apply

# 4. Build ROS 2 Workspace
echo ""
echo -e "${BLUE}[4/5] Building ROS 2 Workspace (vision_tracking package)...${NC}"
cd "${PROJECT_DIR}/ros2_ws"
colcon build --symlink-install --packages-select vision_tracking
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
echo -e "${GREEN}[OK] ROS 2 workspace built and sourced successfully.${NC}"

# 5. Check YOLOv8 Model Weights
echo ""
echo -e "${BLUE}[5/5] Checking YOLOv8 Model Weights...${NC}"
if [[ -f "${PROJECT_DIR}/yolov8n.pt" ]]; then
    echo -e "${GREEN}[OK] Found model weights at ${PROJECT_DIR}/yolov8n.pt${NC}"
else
    echo -e "${YELLOW}[INFO] Downloading yolov8n.pt baseline weights...${NC}"
    python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
fi

echo ""
echo -e "${GREEN}================================================================${NC}"
echo -e "${GREEN}   SETUP COMPLETE! YOUR ENVIRONMENT IS READY TO RUN             ${NC}"
echo -e "${GREEN}================================================================${NC}"
echo -e "To start the full simulation stack (Gazebo + PX4 + Tracking + HUD):"
echo -e "  ${YELLOW}./start_stack.sh${NC}"
echo ""
echo -e "To run the automated turnaround regression tests:"
echo -e "  ${YELLOW}python3 tests/px4/run_isolated_multi_trial.py${NC}"
echo ""
