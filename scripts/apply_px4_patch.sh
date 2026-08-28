#!/bin/bash
# ==============================================================================
# Script: apply_px4_patch.sh
# Purpose: Automatically apply, check, or revert the PX4 GPS patch for airframe
#          4021_gz_x500_flow to enable simulated GPS and GCS failsafe bypass.
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PATCH_FILE="${PROJECT_DIR}/patches/4021_gz_x500_flow_gps.patch"

# Color helpers
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

usage() {
    echo -e "${BLUE}Usage:${NC} $0 [options] [path_to_PX4_Autopilot]"
    echo ""
    echo "Options:"
    echo "  -a, --apply     Apply the patch (default action)"
    echo "  -r, --revert    Revert the patch"
    echo "  -c, --check     Check patch status without modifying"
    echo "  -h, --help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0"
    echo "  $0 /path/to/PX4-Autopilot"
    echo "  $0 --revert ~/PX4-Autopilot"
    echo "  $0 --check"
    exit 1
}

ACTION="apply"
PX4_PATH_ARG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -a|--apply)
            ACTION="apply"
            shift
            ;;
        -r|--revert)
            ACTION="revert"
            shift
            ;;
        -c|--check)
            ACTION="check"
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            if [[ -z "$PX4_PATH_ARG" ]]; then
                PX4_PATH_ARG="$1"
            else
                echo -e "${RED}Unknown argument: $1${NC}"
                usage
            fi
            shift
            ;;
    esac
done

# Resolve PX4 directory
find_px4_dir() {
    if [[ -n "$PX4_PATH_ARG" && -d "$PX4_PATH_ARG" ]]; then
        echo "$(cd "$PX4_PATH_ARG" && pwd)"
        return 0
    fi
    if [[ -n "${PX4_DIR}" && -d "${PX4_DIR}" ]]; then
        echo "$(cd "${PX4_DIR}" && pwd)"
        return 0
    fi
    if [[ -d "${PROJECT_DIR}/../PX4-Autopilot" ]]; then
        echo "$(cd "${PROJECT_DIR}/../PX4-Autopilot" && pwd)"
        return 0
    fi
    if [[ -d "${HOME}/PX4-Autopilot" ]]; then
        echo "$(cd "${HOME}/PX4-Autopilot" && pwd)"
        return 0
    fi
    return 1
}

PX4_DIR_RESOLVED="$(find_px4_dir || true)"

if [[ -z "$PX4_DIR_RESOLVED" || ! -d "$PX4_DIR_RESOLVED" ]]; then
    echo -e "${RED}[ERROR] Could not find PX4-Autopilot directory.${NC}"
    echo "Please specify the PX4 directory path as an argument or set the PX4_DIR environment variable."
    echo "Example: $0 /path/to/PX4-Autopilot"
    exit 1
fi

if [[ ! -f "$PATCH_FILE" ]]; then
    echo -e "${RED}[ERROR] Patch file not found at: ${PATCH_FILE}${NC}"
    exit 1
fi

AIRFRAME_FILE="${PX4_DIR_RESOLVED}/ROMFS/px4fmu_common/init.d-posix/airframes/4021_gz_x500_flow"

if [[ ! -f "$AIRFRAME_FILE" ]]; then
    echo -e "${RED}[ERROR] PX4 airframe file not found:${NC} ${AIRFRAME_FILE}"
    echo "Ensure that the specified path is a valid PX4-Autopilot repository checkout."
    exit 1
fi

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}   PX4 Airframe 4021_gz_x500_flow Patch Utility       ${NC}"
echo -e "${BLUE}======================================================${NC}"
echo -e "PX4 Directory: ${YELLOW}${PX4_DIR_RESOLVED}${NC}"
echo -e "Patch File   : ${YELLOW}${PATCH_FILE}${NC}"
echo -e "Action       : ${YELLOW}${ACTION}${NC}"
echo ""

# Check current patch status
cd "${PX4_DIR_RESOLVED}"

is_already_applied=false
if git apply -R --check "${PATCH_FILE}" >/dev/null 2>&1; then
    is_already_applied=true
fi

case "$ACTION" in
    check)
        if [[ "$is_already_applied" == true ]]; then
            echo -e "${GREEN}[STATUS] Patch is currently APPLIED to PX4-Autopilot.${NC}"
            echo "Simulated GPS is enabled (SYS_HAS_GPS=1, EKF2_GPS_CTRL=7, NAV_DLL_ACT=0)."
            exit 0
        else
            if git apply --check "${PATCH_FILE}" >/dev/null 2>&1; then
                echo -e "${YELLOW}[STATUS] Patch is NOT applied, but can be cleanly applied.${NC}"
                exit 0
            else
                echo -e "${RED}[STATUS] Patch is NOT applied and cannot be applied cleanly (conflicts or modified).${NC}"
                exit 1
            fi
        fi
        ;;

    apply)
        if [[ "$is_already_applied" == true ]]; then
            echo -e "${GREEN}[OK] Patch is ALREADY applied to ${PX4_DIR_RESOLVED}.${NC}"
            echo "Airframe 4021_gz_x500_flow already has simulated GPS and NAV_DLL_ACT=0 configured."
            exit 0
        fi

        echo -e "Checking if patch applies cleanly..."
        if ! git apply --check "${PATCH_FILE}"; then
            echo -e "${RED}[ERROR] Patch cannot be applied cleanly to ${PX4_DIR_RESOLVED}.${NC}"
            echo "Please check if ${AIRFRAME_FILE} has local conflicts."
            exit 1
        fi

        echo -e "Applying patch ${YELLOW}4021_gz_x500_flow_gps.patch${NC}..."
        git apply "${PATCH_FILE}"
        echo -e "${GREEN}[SUCCESS] Patch applied successfully!${NC}"
        echo -e "Simulated GPS and GCS failsafe bypass are now active for airframe 4021."
        ;;

    revert)
        if [[ "$is_already_applied" == false ]]; then
            echo -e "${YELLOW}[INFO] Patch is not currently applied to ${PX4_DIR_RESOLVED}. Nothing to revert.${NC}"
            exit 0
        fi

        echo -e "Reverting patch ${YELLOW}4021_gz_x500_flow_gps.patch${NC}..."
        git apply -R "${PATCH_FILE}"
        echo -e "${GREEN}[SUCCESS] Patch reverted successfully.${NC}"
        echo "Airframe 4021_gz_x500_flow restored to stock PX4 configuration."
        ;;
esac
