import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """
    Master bringup launch file for terrain navigation simulation.
    Launches: ROS-TCP-Endpoint + RViz2 (core simulation infrastructure).
    """
    bringup_dir = get_package_share_directory('terrain_nav_bringup')

    # Declare launch arguments
    ros_ip_arg = DeclareLaunchArgument(
        'ros_ip', default_value='0.0.0.0',
        description='IP address for ROS TCP Endpoint')
    ros_port_arg = DeclareLaunchArgument(
        'ros_port', default_value='10000',
        description='Port for ROS TCP Endpoint')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time from Unity /clock topic')

    ros_ip = LaunchConfiguration('ros_ip')
    ros_port = LaunchConfiguration('ros_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ROS-TCP-Endpoint node (Unity <-> ROS 2 bridge)
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        output='screen',
        parameters=[{
            'ROS_IP': ros_ip,
            'ROS_PORT': ros_port,
        }],
    )

    # RViz2 with terrain navigation config
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'terrain_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        ros_ip_arg,
        ros_port_arg,
        use_sim_time_arg,
        ros_tcp_endpoint,
        rviz2,
    ])
