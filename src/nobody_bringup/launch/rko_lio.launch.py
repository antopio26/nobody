import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nobody_bringup_dir = get_package_share_directory('nobody_bringup')
    config_file_path = os.path.join(nobody_bringup_dir, 'config', 'rko_lio.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_path,
        description='Path to RKO_LIO config file'
    )

    config_file = LaunchConfiguration('config_file')

    # RKO_LIO launch
    rko_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rko_lio'),
                        'launch', 'odometry.launch.py')
        ]),
        launch_arguments={
            'config_file': config_file,
        }.items(),
    )

    return LaunchDescription([
        config_file_arg,
        rko_lio_launch
    ])
