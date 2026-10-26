import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Run speed_governor alone (the maze bringup launches it itself when governor.enabled is true)."""
    params = os.path.join(get_package_share_directory('terrain_speed_governor'), 'config', 'governor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        Node(package='terrain_speed_governor', executable='speed_governor', name='speed_governor',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
