"""Switch the Nav2 global planner / local controller at runtime.

Nav2's default behavior tree listens on /planner_selector and
/controller_selector (std_msgs/String), so no restart is needed.

    ros2 run terrain_navigation set_algo --planner Smac2D --controller MPPI
    ros2 run terrain_navigation set_algo --list
"""
import argparse
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

PLANNERS = {
    'GridBased': 'NavFn (A*/Dijkstra) - default',
    'Smac2D': 'Smac 2D A*',
    'SmacHybrid': 'Smac Hybrid-A* (Dubins)',
    'ThetaStar': 'Theta* any-angle',
}
CONTROLLERS = {
    'FollowPath': 'DWB - default',
    'MPPI': 'MPPI',
    'RPP': 'Regulated Pure Pursuit',
    'Graceful': 'Graceful Controller',
}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--planner', choices=PLANNERS)
    parser.add_argument('--controller', choices=CONTROLLERS)
    parser.add_argument('--list', action='store_true', help='list available algorithms')
    opts, ros_args = parser.parse_known_args(args)

    if opts.list or not (opts.planner or opts.controller):
        print('Planners:')
        for k, v in PLANNERS.items():
            print(f'  {k:11s} {v}')
        print('Controllers:')
        for k, v in CONTROLLERS.items():
            print(f'  {k:11s} {v}')
        return

    rclpy.init(args=ros_args)
    node = rclpy.create_node('set_algo')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
              reliability=ReliabilityPolicy.RELIABLE)
    pubs = []
    if opts.planner:
        pubs.append((node.create_publisher(String, 'planner_selector', qos), opts.planner))
    if opts.controller:
        pubs.append((node.create_publisher(String, 'controller_selector', qos), opts.controller))
    time.sleep(0.5)  # let discovery match the BT subscriber
    for pub, value in pubs:
        pub.publish(String(data=value))
        node.get_logger().info(f'{pub.topic_name} -> {value}')
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
