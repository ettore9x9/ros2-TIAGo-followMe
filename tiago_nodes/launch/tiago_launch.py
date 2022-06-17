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
        namespace='TIAGo_Iron',
        executable='img_subscriber',
    )

    return LaunchDescription([
        tiagosim,
        cv_basics,

        
        # Node(
        #     package='tiago_nodes',
        #     namespace='TIAGo_Iron',
        #     executable='depth_finder',
        # ),
    ])
