import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """
    3D Proposed pipeline launch file.
    Launches: ROS-TCP-Endpoint + RViz2 + Nav2 (3D terrain costmap layer).
    
    Pipeline: Unity 3D LiDAR -> PointCloud2 -> Elevation & Traversability Map -> Nav2 -> /cmd_vel
    
    Note: SLAM Toolbox is NOT used in the 3D pipeline — we use ground-truth TF 
    from Unity to isolate the terrain perception variable (per Section 7 of arch plan).
    """
    bringup_dir = get_package_share_directory('terrain_nav_bringup')
    nav_dir = get_package_share_directory('terrain_navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time from Unity /clock topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include base simulation bringup (TCP endpoint + RViz)
    simulation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Terrain mapping node (PointCloud2 -> Elevation -> Traversability)
    # TODO: Implement in terrain_mapping package (Milestones 5-6)
    # terrain_mapping = Node(
    #     package='terrain_mapping',
    #     executable='elevation_mapper',
    #     name='elevation_mapper',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    # Nav2 stack with 3D terrain parameters
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav_dir, 'config', 'nav2_params_3d.yaml'),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        simulation_bringup,
        nav2_bringup,
    ])
