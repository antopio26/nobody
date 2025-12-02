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
    Generates the launch description to start the Go2 robot, navigation stack, and optionally the Foxglove bridge.
    """

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

    launch_rko_lio_arg = DeclareLaunchArgument(
        'rko_lio',
        default_value='false',
        description='Whether to launch RKO_LIO'
    )

    # Create LaunchConfiguration variables
    launch_slam_config = LaunchConfiguration('slam')
    launch_nav2_config = LaunchConfiguration('nav2')
    launch_rko_lio_config = LaunchConfiguration('rko_lio')

    # Path to the go2_bringup launch file
    go2_bringup_launch_file = os.path.join(
        get_package_share_directory('nobody_bringup'),
        'launch',
        'go2.launch.py'
    )

    # Path to the navigation_mapping launch file
    navigation_mapping_launch_file = os.path.join(
        get_package_share_directory('nobody_bringup'),
        'launch',
        'nav.launch.py'
    )

    # Path to the rko_lio launch file
    rko_lio_launch_file = os.path.join(
        get_package_share_directory('nobody_bringup'),
        'launch',
        'rko_lio.launch.py'
    )

    # Path to the pointcloud filter config file
    pointcloud_filter_config_file = os.path.join(
        get_package_share_directory('nobody_bringup'),
        'config',
        'pointcloud_filter.yaml'
    )

    # Include the go2_bringup launch file with specified arguments
    go2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(go2_bringup_launch_file),
        launch_arguments={
            'lidar': 'True',
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

    # Include the rko_lio launch file
    rko_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rko_lio_launch_file),
        condition=IfCondition(launch_rko_lio_config)
    )

    # Pointcloud filter node
    pointcloud_unitree_filter_node = Node(
        package='nobody_bringup',
        executable='pointcloud_filter_node',
        name='pointcloud_unitree_filter_node',
        parameters=[pointcloud_filter_config_file],
        output='screen'
    )

    # Pointcloud filter node
    pointcloud_hesai_filter_node = Node(
        package='nobody_bringup',
        executable='pointcloud_filter_node',
        name='pointcloud_hesai_filter_node',
        parameters=[pointcloud_filter_config_file],
        output='screen'
    )

    # Create the LaunchDescription with all components
    return LaunchDescription([
        launch_slam_arg,
        launch_nav2_arg,
        launch_rko_lio_arg,
        go2_bringup_launch,
        pointcloud_unitree_filter_node,
        pointcloud_hesai_filter_node,
        rko_lio_launch,
        navigation_mapping_launch,
    ])