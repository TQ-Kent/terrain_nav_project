"""Copies the global /tf and /tf_static into <namespace>/tf and <namespace>/tf_static.

Unity publishes one global /tf for every robot, but Nav2's navigation_launch.py remaps /tf to the relative
name `tf`, so a Nav2 stack started under a namespace listens on /<namespace>/tf.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        self.declare_parameter('namespace', 'robot0')
        ns = self.get_parameter('namespace').value.strip('/')
        dyn = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        static = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dyn_pub = self.create_publisher(TFMessage, f'/{ns}/tf', dyn)
        self.static_pub = self.create_publisher(TFMessage, f'/{ns}/tf_static', static)
        self.create_subscription(TFMessage, '/tf', self.dyn_pub.publish, dyn)
        self.create_subscription(TFMessage, '/tf_static', self.static_pub.publish, static)


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
