#!/usr/bin/env python

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    tiagosim = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('tiagosim'), 'launch'),
         '/my_robot_launch.py'])
    )

    cv_basics = Node(
        package='cv_basics',
        namespace='follow_object',
        executable='img_subscriber',
        prefix=["lxterminal --geometry=120x30%s -e"],
        remappings=[('/color_camera', '/TIAGo_Iron/kinect_color')]
    )

    depth_finder = Node(
        package='tiago_nodes',
        namespace='follow_object',
        executable='depth_finder',
        prefix=["lxterminal --geometry=120x30%s -e"],
        remappings=[('/range_camera', '/TIAGo_Iron/kinect_range')]
    )

    pid_controller = Node(
        package='tiago_nodes',
        namespace='follow_object',
        executable='pid_controller',
        prefix=["lxterminal --geometry=120x30%s -e"],
    )

    obstacle_avoidance = Node(
        package='py_obstacle_avoidance',
        namespace='follow_object',
        executable='robot_controller',
        prefix=["lxterminal --geometry=120x30%s -e"],
    )

    return LaunchDescription([
        tiagosim,
        cv_basics,
        depth_finder,
        pid_controller,
        obstacle_avoidance,
    ])
