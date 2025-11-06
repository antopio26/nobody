# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Generates the launch description to start the Go2 robot, navigation stack, and optionally the Foxglove bridge.
    """

    # Declare launch arguments
    launch_foxglove_arg = DeclareLaunchArgument(
        'foxglove',
        default_value='true',
        description='Whether to launch the Foxglove bridge'
    )

    launch_slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to launch SLAM Toolbox'
    )

    launch_nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Whether to launch Nav2'
    )

    # Create LaunchConfiguration variables
    launch_foxglove_config = LaunchConfiguration('foxglove')
    launch_slam_config = LaunchConfiguration('slam')
    launch_nav2_config = LaunchConfiguration('nav2')

    # Path to the go2_bringup launch file
    go2_bringup_launch_file = os.path.join(
        get_package_share_directory('go2_bringup'),
        'launch',
        'go2.launch.py'
    )

    # Path to the foxglove_bridge launch file
    foxglove_bridge_launch_file = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )

    # Path to the navigation_mapping launch file
    navigation_mapping_launch_file = os.path.join(
        get_package_share_directory('navigation_stack'),
        'launch',
        'navigation_mapping.launch.py'
    )

    # Include the go2_bringup launch file with specified arguments
    go2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_bringup_launch_file),
        launch_arguments={
            'lidar': 'False',
            'realsense': 'False',
            'rviz': 'False',
        }.items()
    )

    # Include the navigation_mapping launch file
    navigation_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_mapping_launch_file),
        launch_arguments={
            'slam': launch_slam_config,
            'nav2': launch_nav2_config,
            'use_sim_time': 'false',
        }.items()
    )

    # Include the foxglove_bridge launch file conditionally
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(foxglove_bridge_launch_file),
        condition=IfCondition(launch_foxglove_config)
    )

    # Create the LaunchDescription with all components
    return LaunchDescription([
        launch_foxglove_arg,
        launch_slam_arg,
        launch_nav2_arg,
        go2_bringup_launch,
        navigation_mapping_launch,
        foxglove_bridge_launch,
    ])