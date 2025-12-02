# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for SLAM, Nav2, and teleoperation.
    """

    # Get the nobody_bringup package directory
    nobody_bringup_dir = get_package_share_directory('nobody_bringup')

    # Declare launch arguments
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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Create LaunchConfiguration variables
    launch_slam_config = LaunchConfiguration('slam')
    launch_nav2_config = LaunchConfiguration('nav2')
    use_sim_time_config = LaunchConfiguration('use_sim_time')

    # Configuration file paths
    slam_params_file = os.path.join(nobody_bringup_dir, 'config', 'mapper_params.yaml')
    nav2_params_file = os.path.join(nobody_bringup_dir, 'config', 'nav2_params.yaml')

    # SLAM Toolbox launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'),
                        'launch', 'online_async_launch.py')
        ]),
        condition=IfCondition(launch_slam_config),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time_config,
        }.items(),
    )

    # Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                        'launch', 'navigation_launch.py')
        ]),
        condition=IfCondition(launch_nav2_config),
        launch_arguments={
            'params_file': nav2_params_file,
            'slam': 'false',
            'use_sim_time': use_sim_time_config,
        }.items(),
    )

    # Base Footprint Publisher
    base_footprint_publisher = Node(
        package='nobody_bringup',
        executable='base_footprint_publisher.py',
        name='base_footprint_publisher',
        output='screen'
    )


    # Create the LaunchDescription with all components
    return LaunchDescription([
        launch_slam_arg,
        launch_nav2_arg,
        use_sim_time_arg,
        slam_toolbox_launch,
        nav2_launch,
        base_footprint_publisher,
    ])