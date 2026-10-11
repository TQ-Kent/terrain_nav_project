import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    """
    2D Baseline pipeline launch file.
    Launches: ROS-TCP-Endpoint + RViz2 + SLAM Toolbox + Nav2 (2D obstacle layer).
    
    Pipeline: Unity 2D LiDAR -> LaserScan -> 2D Occupancy Grid -> Nav2 -> /cmd_vel
    """
    bringup_dir = get_package_share_directory('terrain_nav_bringup')
    nav_dir = get_package_share_directory('terrain_navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time from Unity /clock topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Algorithms are also switchable live: `ros2 run terrain_navigation set_algo ...`
    planner_arg = DeclareLaunchArgument(
        'planner', default_value='GridBased',
        description='Initial global planner: GridBased(NavFn) | Smac2D | SmacHybrid | ThetaStar')
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='FollowPath',
        description='Initial local controller: FollowPath(DWB) | MPPI | RPP | Graceful')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_dir, 'config', 'nav2_params_2d.yaml'),
        description='Nav2 parameter file')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav_dir, 'config', 'slam_toolbox_2d.yaml'),
        description='slam_toolbox parameter file')

    # Include base simulation bringup (TCP endpoint + RViz)
    simulation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # SLAM Toolbox (online async mode)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )

    # Nav2 stack with 2D parameters
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    # Apply the initial selection once Nav2 is up (BT listens on *_selector topics)
    initial_algo = TimerAction(period=15.0, actions=[ExecuteProcess(
        cmd=['ros2', 'run', 'terrain_navigation', 'set_algo',
             '--planner', LaunchConfiguration('planner'),
             '--controller', LaunchConfiguration('controller')],
        output='screen')])

    return LaunchDescription([
        use_sim_time_arg,
        planner_arg,
        controller_arg,
        params_file_arg,
        slam_params_arg,
        simulation_bringup,
        slam_toolbox,
        nav2_bringup,
        initial_algo,
    ])
