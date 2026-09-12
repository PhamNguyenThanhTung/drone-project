#!/usr/bin/env python3
"""
Standard ROS 2 Launch File for Vision Tracking Stack.
Orchestrates:
  1. ros_gz_bridge (camera transport bridge, optional via use_bridge)
  2. simulation_realism (sensor/camera fault injection, optional via use_realism)
  3. yolo_detector_node (YOLOv8 + ByteTrack perception)
  4. motion_arbiter_node (PX4 Offboard flight controller, named 'motion_arbiter')
  5. live_camera_hud_node (OpenCV HUD visualization, optional via show_hud, named 'live_camera_hud')

All nodes strictly adhere to the parameter mapping in tracking_stack.yaml.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    use_realism_str = LaunchConfiguration('use_realism').perform(context).lower()
    use_realism = use_realism_str in ('true', '1')
    show_hud_str = LaunchConfiguration('show_hud').perform(context).lower()
    show_hud = show_hud_str in ('true', '1')
    use_bridge_str = LaunchConfiguration('use_bridge').perform(context).lower()
    use_bridge = use_bridge_str in ('true', '1')
    yolo_device = LaunchConfiguration('yolo_device').perform(context).strip()
    takeoff_alt = LaunchConfiguration('takeoff_alt').perform(context).strip()

    nodes_to_launch = []

    # 1. ros_gz_bridge: Bridge camera image from Gazebo transport to ROS 2 Image
    if use_bridge:
        nodes_to_launch.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            )
        )

    # 2. simulation_realism: Fault injection node (optional)
    if use_realism:
        nodes_to_launch.append(
            Node(
                package='vision_tracking',
                executable='sim_realism_node',
                name='simulation_realism',
                output='screen',
                parameters=[params_file],
            )
        )

    # 3. yolo_detector_node: Perception node
    yolo_params = [params_file]
    yolo_overrides = {}
    if use_realism:
        yolo_overrides['image_topic'] = '/simulation/camera/image'
    if yolo_device:
        yolo_overrides['device'] = yolo_device
    if yolo_overrides:
        yolo_params.append(yolo_overrides)

    nodes_to_launch.append(
        Node(
            package='vision_tracking',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen',
            parameters=yolo_params,
        )
    )

    # 4. motion_arbiter_node: Named 'motion_arbiter' to match tracking_stack.yaml
    arbiter_params = [params_file]
    arbiter_overrides = {}
    if takeoff_alt:
        try:
            arbiter_overrides['takeoff_alt'] = float(takeoff_alt)
        except ValueError:
            pass
    if arbiter_overrides:
        arbiter_params.append(arbiter_overrides)

    nodes_to_launch.append(
        Node(
            package='vision_tracking',
            executable='motion_arbiter_node',
            name='motion_arbiter',
            output='screen',
            parameters=arbiter_params,
        )
    )

    # 5. live_camera_hud_node: Named 'live_camera_hud' to match tracking_stack.yaml
    if show_hud:
        nodes_to_launch.append(
            Node(
                package='vision_tracking',
                executable='live_camera_hud_node',
                name='live_camera_hud',
                output='screen',
                parameters=[params_file],
            )
        )

    return nodes_to_launch


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_tracking')
    default_params_file = os.path.join(pkg_share, 'config', 'tracking_stack.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to ROS 2 parameters YAML file'
    )
    use_realism_arg = DeclareLaunchArgument(
        'use_realism',
        default_value='false',
        description='Enable sensor/camera fault injection via simulation_realism node'
    )
    show_hud_arg = DeclareLaunchArgument(
        'show_hud',
        default_value='true',
        description='Launch OpenCV HUD viewer'
    )
    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge',
        default_value='true',
        description='Launch ros_gz_bridge parameter_bridge for camera stream'
    )
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='',
        description='Override YOLO device (e.g. cuda:0 or cpu). Empty uses YAML default.'
    )
    takeoff_alt_arg = DeclareLaunchArgument(
        'takeoff_alt',
        default_value='',
        description='Override takeoff altitude in meters. Empty uses YAML default.'
    )

    return LaunchDescription([
        params_file_arg,
        use_realism_arg,
        show_hud_arg,
        use_bridge_arg,
        yolo_device_arg,
        takeoff_alt_arg,
        OpaqueFunction(function=launch_setup),
    ])
