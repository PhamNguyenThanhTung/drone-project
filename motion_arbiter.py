#!/usr/bin/env python3
"""
Backward-compatibility shim for MotionArbiter.
Re-exports everything from vision_tracking.motion_arbiter_node so tests
and scripts importing or invoking this file continue to work seamlessly.
"""
import sys
import os

_src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ros2_ws', 'src', 'vision_tracking')
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

from vision_tracking.motion_arbiter_node import *
from vision_tracking.motion_arbiter_node import main

if __name__ == '__main__':
    main()
