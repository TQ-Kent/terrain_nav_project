"""Headless stand-in for the Unity maze scene (no Unity needed).

Provides what Unity provides: /clock, /scan (180 beams over 0..359 deg, 10 Hz, 0.12-3.5 m,
misses as +inf), TF odom->base_footprint->base_link->base_scan from the true pose, and it
drives from /cmd_vel with speed caps, acceleration limits and a 0.5 s command timeout.
Walls come from maps/maze_seed42.txt (same layout Unity builds). It also publishes the true
pose on /sim/ground_truth and counts wall contacts.

Not modelled: wheel slip, physics-engine contacts, lidar noise, Unity frame-time jitter.
"""
import math
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

CHAR = 0.5            # m per layout character
ORIGIN_SHIFT = 1.25   # cell (0,0) centre is the origin: char index c spans [c*CHAR-1.25, +CHAR]


def load_layout(path):
    rows = [r.rstrip('\n') for r in open(path) if not r.startswith('//') and r.strip()]
    size = len(rows)
    wall = np.zeros((size, size), dtype=bool)      # wall[cx, cz]
    for r, line in enumerate(rows):
        cz = size - 1 - r                          # top row = far z
        for cx, ch in enumerate(line):
            wall[cx, cz] = ch == '#'
    return wall


class HeadlessSim(Node):

    def __init__(self):
        super().__init__('headless_sim', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=False)])
        default_map = os.path.join(get_package_share_directory('terrain_navigation'),
                                   'maps', 'maze_seed42.txt')
        for name, default in (('layout', default_map), ('realtime_factor', 1.0),
                              ('robot_radius', 0.2), ('max_linear', 1.0), ('max_angular', 2.0),
                              ('linear_accel', 2.0), ('angular_accel', 6.0)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.wall = load_layout(g('layout'))
        self.rtf, self.radius = g('realtime_factor'), g('robot_radius')
        self.vmax, self.wmax = g('max_linear'), g('max_angular')
        self.amax, self.awmax = g('linear_accel'), g('angular_accel')

        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_time = -1e9
        self.sim_t = 0.0
        self.next_scan = 0.0
        self.contacts = 0
        self.in_contact = False
        self.distance = 0.0

        n = 180
        self.angle_min, self.angle_max = 0.0, math.radians(359.0)
        self.increment = (self.angle_max - self.angle_min) / n
        self.beam_angles = self.angle_min + self.increment * np.arange(n)
        self.steps = np.arange(0.12, 3.5, 0.01)

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.gt_pub = self.create_publisher(PoseStamped, '/sim/ground_truth', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static()
        self.wall_last = time.monotonic()
        self.create_timer(0.01, self.tick)
        self.get_logger().info(
            f'headless maze simulator: {self.wall.shape[0]}x{self.wall.shape[1]} chars, '
            f'rtf {self.rtf}, caps {self.vmax} m/s {self.wmax} rad/s')

    # -- world ----------------------------------------------------------------------------
    def is_wall(self, x, y):
        """Vectorised wall lookup in ROS coordinates (Unity x = -y_ros, z = x_ros)."""
        cx = np.floor((-np.asarray(y) + ORIGIN_SHIFT) / CHAR).astype(int)
        cz = np.floor((np.asarray(x) + ORIGIN_SHIFT) / CHAR).astype(int)
        size = self.wall.shape[0]
        inside = (cx >= 0) & (cx < size) & (cz >= 0) & (cz < size)
        out = np.ones(np.shape(cx), dtype=bool)
        out[inside] = self.wall[cx[inside], cz[inside]]
        return out

    def collides(self, x, y):
        a = np.linspace(0, 2 * math.pi, 12, endpoint=False)
        px = np.concatenate(([x], x + self.radius * np.cos(a)))
        py = np.concatenate(([y], y + self.radius * np.sin(a)))
        return bool(self.is_wall(px, py).any())

    # -- ROS ------------------------------------------------------------------------------
    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.sim_t

    def stamp(self):
        s = self.sim_t
        return rclpy.time.Time(seconds=int(s), nanoseconds=int((s % 1) * 1e9)).to_msg()

    def publish_static(self):
        out = []
        for parent, child, z in (('base_footprint', 'base_link', 0.0), ('base_link', 'base_scan', 0.1)):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            out.append(t)
        self.static_tf.sendTransform(out)

    def tick(self):
        wall_now = time.monotonic()
        dt = min((wall_now - self.wall_last) * self.rtf, 0.1)
        self.wall_last = wall_now
        if dt <= 0:
            return
        self.sim_t += dt
        self.step(dt)

        clock = Clock()
        clock.clock = self.stamp()
        self.clock_pub.publish(clock)

        t = TransformStamped()
        t.header.stamp = clock.clock
        t.header.frame_id, t.child_frame_id = 'odom', 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.tf.sendTransform(t)

        if self.sim_t >= self.next_scan:
            self.next_scan = self.sim_t + 0.1
            self.publish_scan(clock.clock)
            gt = PoseStamped()
            gt.header.stamp, gt.header.frame_id = clock.clock, 'odom'
            gt.pose.position.x, gt.pose.position.y = self.x, self.y
            gt.pose.position.z = self.distance          # travelled distance, for benchmarking
            gt.pose.orientation.z, gt.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            self.gt_pub.publish(gt)

    def step(self, dt):
        cv, cw = self.cmd if self.sim_t - self.cmd_time < 0.5 else (0.0, 0.0)
        cv = max(-self.vmax, min(self.vmax, cv))
        cw = max(-self.wmax, min(self.wmax, cw))
        self.v += max(-self.amax * dt, min(self.amax * dt, cv - self.v))
        self.w += max(-self.awmax * dt, min(self.awmax * dt, cw - self.w))
        self.yaw = (self.yaw + self.w * dt + math.pi) % (2 * math.pi) - math.pi
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        if self.collides(nx, ny):
            if not self.in_contact:
                self.contacts += 1
                self.get_logger().warn(f'wall contact #{self.contacts} at ({self.x:.2f}, {self.y:.2f})')
            self.in_contact = True
            self.v = 0.0
        else:
            self.in_contact = False
            self.distance += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny

    def publish_scan(self, stamp):
        ang = self.yaw + self.beam_angles
        px = self.x + np.outer(np.cos(ang), self.steps)
        py = self.y + np.outer(np.sin(ang), self.steps)
        hit = self.is_wall(px, py)
        first = hit.argmax(axis=1)
        any_hit = hit.any(axis=1)
        ranges = np.where(any_hit, self.steps[first], np.inf).astype(np.float32)
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp, 'base_scan'
        msg.angle_min, msg.angle_max = self.angle_min, self.angle_max
        msg.angle_increment = self.increment
        msg.scan_time, msg.range_min, msg.range_max = 0.1, 0.12, 3.5
        msg.ranges = ranges.tolist()
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadlessSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.get_logger().info(f'wall contacts: {node.contacts}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
