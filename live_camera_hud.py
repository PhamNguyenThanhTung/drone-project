#!/usr/bin/env python3
"""
Backward-compatibility shim for LiveCameraHUD.
Re-exports everything from vision_tracking.live_camera_hud_node so tests
and scripts importing or invoking this file continue to work seamlessly.
All core GUI, ROS, and tracking logic lives in vision_tracking.live_camera_hud_node.
"""
import os
import sys

_src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ros2_ws', 'src', 'vision_tracking')
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

from vision_tracking.live_camera_hud_node import (
    LiveCameraHUD,
    main,
)
from vision_tracking.live_camera_hud_node import *

if __name__ == '__main__':
    main()
