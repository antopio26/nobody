import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Launches both the filtering and vision components of the perception stack.
    """
    nobody_nodes_pkg_dir = get_package_share_directory('nobody_nodes')
    nobody_nodes_launch_dir = os.path.join(nobody_nodes_pkg_dir, 'launch')

    # Include filters.launch.py
    filters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nobody_nodes_launch_dir, 'filters.launch.py')
        )
    )

    # Include vision.launch.py
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nobody_nodes_launch_dir, 'vision.launch.py')
        )
    )

    return LaunchDescription([
        filters_launch,
        vision_launch
    ])
