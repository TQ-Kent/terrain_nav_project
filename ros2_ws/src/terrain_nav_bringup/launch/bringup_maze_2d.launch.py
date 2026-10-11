import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, TimerAction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Maze test: baseline pipeline + goal-biased frontier exploration toward a known goal.

    The robot is told where the goal is (map frame = robot start pose) but not the map.
    Nav2 plans only through space SLAM has already seen; goal_explorer feeds it frontier
    goals until the goal itself is reachable, then sends the goal. Avoids the planner
    routing through unseen space and bouncing between dead ends.

    Default goal is the far corner cell of MazeScene (Unity (10, 10) -> ROS x=10, y=-10),
    assuming the robot starts at the origin facing Unity +z (= ROS +x).
    """
    bringup_dir = get_package_share_directory('terrain_nav_bringup')
    nav_dir = get_package_share_directory('terrain_navigation')

    args = [
        DeclareLaunchArgument('planner', default_value='GridBased'),
        DeclareLaunchArgument('controller', default_value='FollowPath'),
        DeclareLaunchArgument('goal_x', default_value='10.0', description='Goal x, map frame (m)'),
        DeclareLaunchArgument('goal_y', default_value='-10.0', description='Goal y, map frame (m)'),
        DeclareLaunchArgument('goal_weight', default_value='1.0',
                              description='Pull toward the goal when ranking frontiers'),
        DeclareLaunchArgument('stall_timeout', default_value='40.0',
                              description='Stop the whole test after this many s (sim time) '
                                          'without the robot moving; 0 disables'),
        DeclareLaunchArgument('localization', default_value='ground_truth',
                              description='ground_truth: static identity map->odom (Unity odom is '
                                          'the true pose), slam_toolbox only maps. '
                                          'slam: slam_toolbox localizes (pose can jump on re-optimisation)'),
    ]

    gt = IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'ground_truth'"]))

    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_baseline_2d.launch.py')),
        launch_arguments={
            'planner': LaunchConfiguration('planner'),
            'controller': LaunchConfiguration('controller'),
            'params_file': os.path.join(nav_dir, 'config', 'nav2_params_maze.yaml'),
            'slam_params_file': PythonExpression([
                "'", os.path.join(nav_dir, 'config', 'slam_toolbox_2d_gt.yaml'), "' if '",
                LaunchConfiguration('localization'), "' == 'ground_truth' else '",
                os.path.join(nav_dir, 'config', 'slam_toolbox_2d.yaml'), "'"]),
        }.items(),
    )

    explorer_node = Node(
        package='terrain_navigation',
        executable='goal_explorer',
        name='goal_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_x': ParameterValue(LaunchConfiguration('goal_x'), value_type=float),
            'goal_y': ParameterValue(LaunchConfiguration('goal_y'), value_type=float),
            'goal_weight': ParameterValue(LaunchConfiguration('goal_weight'), value_type=float),
            'stall_timeout': ParameterValue(LaunchConfiguration('stall_timeout'), value_type=float),
        }],
    )
    explorer = TimerAction(period=20.0, actions=[explorer_node])

    # goal_explorer exits when the robot stalls (stall_timeout s without moving): end the test.
    stop_on_stall = RegisterEventHandler(OnProcessExit(
        target_action=explorer_node, on_exit=[Shutdown(reason='robot stalled or test finished')]))

    # Ground truth: Unity publishes the true odom->base_footprint, so map == odom.
    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=gt)

    return LaunchDescription(args + [static_map_odom, baseline, explorer, stop_on_stall])
