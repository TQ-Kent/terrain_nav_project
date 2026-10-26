"""Several Unity robots, one Nav2 stack each, and CBBA task allocation between them.

Prerequisite: Unity scene built with TerrainNav > Build Maze Scene (N robots), in Play.
  ros2 launch terrain_multirobot bringup_multirobot.launch.py robots:=3 tasks:=tasks_demo.yaml profile:=reliable

Shared: ROS-TCP-Endpoint, /map from a map server (the maze is known in this version), map->odom identity,
task list. Per robot k (namespace robotK): TF relay, Nav2, planner/controller selection, CBBA agent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

from terrain_nav_bringup import maze_config
from terrain_multirobot.map_tools import write_map


def _setup(context):
    arg = lambda n: context.launch_configurations.get(n, '')
    n = int(arg('robots') or 3)
    mr_dir = get_package_share_directory('terrain_multirobot')
    bringup_dir = get_package_share_directory('terrain_nav_bringup')
    nav_dir = get_package_share_directory('terrain_navigation')

    tasks = arg('tasks') or 'tasks_demo.yaml'
    if not os.path.isabs(tasks):
        tasks = os.path.join(mr_dir, 'config', tasks)

    config_path = os.path.join(bringup_dir, 'config', 'maze_test.yaml')
    overrides = []
    if arg('profile'):
        prof = (maze_config.load(config_path).get('profiles') or {}).get(arg('profile'))
        if prof is None:
            raise RuntimeError(f"unknown profile '{arg('profile')}' in {config_path}")
        overrides = [f'{k}={str(v).lower() if isinstance(v, bool) else v}' for k, v in prof.items()]
    overrides += [o for o in arg('set').split(',') if o.strip()]
    cfg = maze_config.merged(config_path, overrides)
    print(f'[bringup_multirobot] {n} robots, tasks {tasks}, overrides {overrides}')

    map_yaml = write_map(os.path.join(nav_dir, 'maps', 'maze_seed42.txt'), '/tmp/terrain_multirobot_map/maze')
    base = maze_config.nav2_params(os.path.join(nav_dir, 'config', 'nav2_params_maze.yaml'), cfg['speed'],
                                  cfg.get('tolerance'), costmap=cfg.get('costmap'), rpp=cfg.get('rpp'),
                                  planner=cfg.get('planner'))

    actions = [
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint', name='ros_tcp_endpoint',
             output='screen', parameters=[{'ROS_IP': '0.0.0.0', 'ROS_PORT': 10000}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'], parameters=[{'use_sim_time': True}]),
        Node(package='terrain_multirobot', executable='task_publisher', name='task_publisher', output='screen',
             parameters=[{'tasks_file': tasks, 'use_sim_time': True}]),
    ]
    for k in range(n):
        ns = f'robot{k}'
        params = maze_config.namespace_params(base, ns)
        actions += [
            Node(package='terrain_multirobot', executable='tf_relay', name=f'tf_relay_{ns}', output='screen',
                 parameters=[{'namespace': ns, 'use_sim_time': True}]),
            # navigation_launch.py only uses `namespace` to rewrite the params file; the nodes need the push.
            GroupAction([
                PushRosNamespace(ns),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                    launch_arguments={'namespace': ns, 'use_sim_time': 'true', 'params_file': params,
                                      'autostart': 'true'}.items()),
            ]),
            TimerAction(period=15.0, actions=[ExecuteProcess(
                cmd=['ros2', 'run', 'terrain_navigation', 'set_algo', '--planner', cfg['algorithms']['planner'],
                     '--controller', cfg['algorithms']['controller'], '--ros-args', '-r', f'__ns:=/{ns}'],
                output='screen')]),
            TimerAction(period=20.0, actions=[Node(
                package='terrain_multirobot', executable='cbba_agent', name='cbba_agent', namespace=ns,
                output='screen', parameters=[{
                    'robot_id': k, 'num_robots': n, 'namespace': ns, 'base_frame': f'{ns}/base_link',
                    'speed': float(cfg['speed']['max_linear']), 'use_sim_time': True}])]),
        ]
    return actions


def generate_launch_description():
    helps = {'robots': 'number of robots (default 3)', 'tasks': 'tasks YAML (default tasks_demo.yaml)',
             'profile': "profile from maze_test.yaml, e.g. 'reliable'", 'set': 'comma-separated config overrides'}
    args = [DeclareLaunchArgument(k, default_value='', description=v) for k, v in helps.items()]
    return LaunchDescription([SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')] + args + [OpaqueFunction(function=_setup)])
