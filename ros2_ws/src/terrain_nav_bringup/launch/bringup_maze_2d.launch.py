import os
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, TimerAction,
                            RegisterEventHandler, Shutdown, OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from terrain_nav_bringup import maze_config


def _setup(context):
    """Everything is read from config/maze_test.yaml, then optional launch-argument overrides."""
    bringup_dir = get_package_share_directory('terrain_nav_bringup')
    nav_dir = get_package_share_directory('terrain_navigation')
    arg = lambda n: context.launch_configurations.get(n, '')

    config_path = arg('config') or os.path.join(bringup_dir, 'config', 'maze_test.yaml')
    overrides = [o for o in arg('set').split(',') if o.strip()]
    for name, key in (('planner', 'algorithms.planner'), ('controller', 'algorithms.controller'),
                      ('goal_x', 'goal.x'), ('goal_y', 'goal.y'), ('localization', 'localization'),
                      ('simulator', 'simulator')):
        if arg(name):
            overrides.append(f'{key}={arg(name)}')
    cfg = maze_config.merged(config_path, overrides)
    print(f'[bringup_maze_2d] config {config_path} overrides {overrides}')

    ground_truth = cfg['localization'] == 'ground_truth'
    headless = cfg['simulator'] == 'headless'
    ex, sp = cfg['exploration'], cfg['speed']

    profile = {'maze': 'nav2_params_maze.yaml', 'baseline_2d': 'nav2_params_2d.yaml'}[cfg['nav2_profile']]
    nav2_params = maze_config.nav2_params(os.path.join(nav_dir, 'config', profile), sp,
                                         cfg.get('tolerance'))
    slam_params = os.path.join(
        nav_dir, 'config', 'slam_toolbox_2d_gt.yaml' if ground_truth else 'slam_toolbox_2d.yaml')

    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_baseline_2d.launch.py')),
        launch_arguments={
            'planner': cfg['algorithms']['planner'],
            'controller': cfg['algorithms']['controller'],
            'params_file': nav2_params,
            'slam_params_file': slam_params,
            'use_endpoint': 'false' if headless else 'true',
            'use_rviz': 'true' if cfg['rviz'] else 'false',
        }.items(),
    )

    explorer_node = Node(
        package='terrain_navigation', executable='goal_explorer', name='goal_explorer',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'goal_x': float(cfg['goal']['x']), 'goal_y': float(cfg['goal']['y']),
                     **{k: (float(v) if isinstance(v, (int, float)) else v)
                        for k, v in ex.items() if k not in ('start_delay', 'mode', 'result_file')},
                     'mode': str(ex['mode']), 'result_file': str(ex['result_file'] or ''),
                     'min_frontier_cells': int(ex['min_frontier_cells'])}],
    )
    explorer = TimerAction(period=float(ex['start_delay']), actions=[explorer_node])

    # goal_explorer exits on GOAL REACHED or STALLED: end the whole test.
    stop = RegisterEventHandler(OnProcessExit(
        target_action=explorer_node, on_exit=[Shutdown(reason='goal reached or robot stalled')]))

    actions = [baseline, explorer, stop]
    if ground_truth:
        actions.insert(0, Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom']))
    if headless:
        h = cfg['headless']
        actions.insert(0, Node(
            package='terrain_navigation', executable='headless_sim', name='headless_sim',
            output='screen', parameters=[{
                'realtime_factor': float(h['realtime_factor']),
                'robot_radius': float(h['robot_radius']),
                'max_linear': float(h['max_linear']), 'max_angular': float(h['max_angular']),
                'linear_accel': float(h['linear_accel']), 'angular_accel': float(h['angular_accel'])}]))
    return actions


def generate_launch_description():
    """
    Maze test: baseline pipeline + goal-biased frontier exploration toward a known goal.
    All settings live in terrain_nav_bringup/config/maze_test.yaml (goal, algorithms, speeds,
    exploration, localization, simulator). Launch arguments below only override that file.

    The robot is told where the goal is (map frame = robot start pose) but not the map.
    Nav2 plans only through space SLAM has already seen; goal_explorer feeds it frontier
    goals until the goal itself is reachable, then sends the goal.
    """
    helps = {
        'config': 'path to a config file (default: share/terrain_nav_bringup/config/maze_test.yaml)',
        'set': "comma-separated overrides, e.g. 'speed.max_linear=0.8,exploration.goal_weight=2'",
        'planner': 'override algorithms.planner', 'controller': 'override algorithms.controller',
        'goal_x': 'override goal.x', 'goal_y': 'override goal.y',
        'localization': 'override localization: ground_truth | slam',
        'simulator': 'override simulator: unity | headless',
    }
    args = [DeclareLaunchArgument(k, default_value='', description=v) for k, v in helps.items()]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
