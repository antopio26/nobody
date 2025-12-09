# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the pointcloud filter nodes.
    """

    # Path to the pointcloud filter config file
    pointcloud_filter_config_file = os.path.join(
        get_package_share_directory('nobody_nodes'),
        'config',
        'pointcloud_filter.yaml'
    )
    
    # Pointcloud filter node
    pointcloud_unitree_filter_node = Node(
        package='nobody_nodes',
        executable='pointcloud_filter_node',
        name='pointcloud_unitree_filter_node',
        parameters=[pointcloud_filter_config_file],
        output='screen'
    )

    # Pointcloud filter node
    pointcloud_hesai_filter_node = Node(
        package='nobody_nodes',
        executable='pointcloud_filter_node',
        name='pointcloud_hesai_filter_node',
        parameters=[pointcloud_filter_config_file],
        output='screen'
    )

    # Create the LaunchDescription with all components
    return LaunchDescription([
        pointcloud_unitree_filter_node,
        pointcloud_hesai_filter_node
    ])