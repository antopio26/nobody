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
    Launches the target position estimation node.
    """

    # Path to the target estimation config file
    target_est_config_file = os.path.join(
        get_package_share_directory('nobody_nodes'),
        'config',
        'target_est.yaml'
    )

    # Target position estimation node
    target_pos_estimation_node = Node(
        package='nobody_nodes',
        executable='target_pos_estimation',
        name='target_pos_estimation_node',
        parameters=[target_est_config_file],
        output='screen'
    )

    # Create the LaunchDescription with all components
    return LaunchDescription([
        target_pos_estimation_node
    ])