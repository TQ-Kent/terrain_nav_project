"""Publishes the task list once (latched) from a YAML file so scenarios are repeatable.

tasks.yaml: {tasks: [{x: 1.0, y: -2.0, value: 100.0, discount: 0.98}, ...]}
"""
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from terrain_multirobot_msgs.msg import TaskList, TaskPoint


def load_tasks(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return [(float(t['x']), float(t['y']), float(t.get('value', 100.0)), float(t.get('discount', 0.98)))
            for t in data['tasks']]


class TaskPublisher(Node):
    def __init__(self):
        super().__init__('task_publisher')
        self.declare_parameter('tasks_file', '')
        tasks = load_tasks(self.get_parameter('tasks_file').value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.create_publisher(TaskList, '/cbba/tasks', qos)
        msg = TaskList()
        for i, (x, y, value, discount) in enumerate(tasks):
            msg.tasks.append(TaskPoint(id=i, x=x, y=y, value=value, discount=discount))
        pub.publish(msg)
        self.get_logger().info(f'published {len(tasks)} tasks')


def main(args=None):
    rclpy.init(args=args)
    node = TaskPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
