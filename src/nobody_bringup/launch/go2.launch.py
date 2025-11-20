# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    realsense = LaunchConfiguration('realsense')

    nobody_bringup_dir = get_package_share_directory('nobody_bringup')
    robot_self_filter_config_file = os.path.join(nobody_bringup_dir, 'config', 'self_filter.yaml')

    declare_lidar_cmd = DeclareLaunchArgument(
        'lidar',
        default_value='False',
        description='Launch hesai lidar driver'
    )

    declare_realsense_cmd = DeclareLaunchArgument(
        'realsense',
        default_value='False',
        description='Launch realsense driver'
    )

    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_description'),
            'launch/'), 'robot.launch.py'])
    )

    lidar_cmd = Node(
        namespace='hesai_ros_driver',
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        condition=IfCondition(PythonExpression([lidar]))
    )

    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch/'), 'rs_launch.py']),
        condition=IfCondition(PythonExpression([realsense]))
    )

    composable_nodes = []
    
    composable_node = ComposableNode(
        package='go2_driver',
        plugin='go2_driver::Go2Driver',
        name='go2_driver',
        namespace='',

    )
    composable_nodes.append(composable_node)

    driver_container = ComposableNodeContainer(
        name='go2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    pointcloud_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace='',
        output='screen',
        remappings=[('/cloud_in', '/lidar_points')],
        parameters=[{
                'target_frame': 'hesai_lidar',
                'max_height': 0.7,
                # 'transform_tolerance': 0.01,
        }],
    )

    # ADD PointCloud to LaserScan for the internal lidar and remap the output topics

    static_tf_lidar_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['-0.12', '0', '0.1', '1.5708', '0', '0', 'Head_upper', 'hesai_lidar'],
        output='screen',
    )


    ld = LaunchDescription()
    ld.add_action(declare_lidar_cmd)
    ld.add_action(declare_realsense_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(realsense_cmd)
    ld.add_action(driver_container)
    ld.add_action(pointcloud_to_laserscan_cmd)
    ld.add_action(static_tf_lidar_cmd)

    return ld
