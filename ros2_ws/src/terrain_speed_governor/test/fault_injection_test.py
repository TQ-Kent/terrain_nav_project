#!/usr/bin/env python3
"""Integration test for the speed_governor node with synthetic inputs (no Nav2, no simulator).

    ROS_DOMAIN_ID=77 python3 test/fault_injection_test.py      # needs the built workspace sourced

Spawns the node (wall-clock time), feeds it a smoothed command, a straight plan, a free local
costmap, scans and TF, then injects faults and checks the governed output:
  healthy -> passes the command through (no more than requested)
  wall ahead in the costmap -> slows, and stops before reaching it
  stale scan / stale costmap / lost TF -> command driven to zero and held, then released after recovery
  invalid (all-NaN) scan -> zero
  command never exceeds the request
Exit code 0 = all checks passed.
"""
import json
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class Feeder(Node):
    def __init__(self):
        super().__init__('governor_test_feeder')
        self.scan_on = self.map_on = self.tf_on = True
        self.scan_nan = False
        self.wall_x = None      # wall at this x (odom frame), or None
        self.cmd_v = 0.5
        self.out = None
        self.status = {}
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'local_costmap/costmap', qos)
        self.plan_pub = self.create_publisher(Path, 'received_global_plan', 5)
        self.create_subscription(Twist, 'cmd_vel_governed', lambda m: setattr(self, 'out', m), 10)
        self.create_subscription(String, 'speed_governor/status',
                                 lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.rotation.w = 1.0
        self.static.sendTransform(t)
        self.create_timer(0.05, self.fast)
        self.create_timer(0.1, self.scan)
        self.create_timer(0.5, self.costmap)
        self.create_timer(1.0, self.plan)

    def fast(self):
        c = Twist()
        c.linear.x = self.cmd_v
        self.cmd_pub.publish(c)
        if self.tf_on:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
            t.transform.rotation.w = 1.0
            self.tf.sendTransform(t)

    def scan(self):
        if not self.scan_on:
            return
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_scan'
        m.angle_min, m.angle_max, m.angle_increment = 0.0, 2 * math.pi - 0.0175, 0.0175
        m.range_min, m.range_max = 0.12, 10.0
        m.ranges = [float('nan') if self.scan_nan else 3.0] * 360
        self.scan_pub.publish(m)

    def costmap(self):
        if not self.map_on:
            return
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom'
        g.info.resolution, g.info.width, g.info.height = 0.05, 120, 120
        g.info.origin.position.x = g.info.origin.position.y = -3.0
        g.info.origin.orientation.w = 1.0
        data = [0] * (120 * 120)
        if self.wall_x is not None:
            ix = int((self.wall_x + 3.0) / 0.05)
            for iy in range(120):
                data[iy * 120 + ix] = 100
        g.data = data
        self.map_pub.publish(g)

    def plan(self):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        for i in range(60):
            ps = PoseStamped()
            ps.pose.position.x = 0.1 * i
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.plan_pub.publish(p)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    node = subprocess.Popen(['ros2', 'run', 'terrain_speed_governor', 'speed_governor', '--ros-args',
                             '-p', 'use_sim_time:=false', '-p', 'v_max:=0.5'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    f = Feeder()
    failures = []

    def check(name, ok, detail=''):
        print(('PASS ' if ok else 'FAIL ') + name + (f'  [{detail}]' if detail else ''))
        if not ok:
            failures.append(name)

    out_v = lambda: f.out.linear.x if f.out else float('nan')
    try:
        f.spin_for(4.0)
        check('healthy: passes ~0.5 m/s', abs(out_v() - 0.5) < 0.02, f'out={out_v():.3f}')
        check('healthy: never above request', out_v() <= 0.5 + 1e-9)

        f.wall_x = 1.0
        f.spin_for(1.5)
        check('wall 1.0 m ahead: not needlessly slowed', out_v() > 0.45, f'out={out_v():.3f}')
        f.wall_x = 0.45   # gap 0.23 m < stopping distance at 0.5 m/s (0.28 m)
        f.spin_for(1.5)
        check('wall 0.45 m ahead: slowed below request', out_v() < 0.45, f'out={out_v():.3f} '
              f'd_free={f.status.get("d_free")} reason={f.status.get("reason")}')
        f.wall_x = 0.27   # gap 0.05 m = safety margin: no room left
        f.spin_for(1.5)
        check('wall at the safety margin: at most creep speed (min_speed 0.05)', out_v() <= 0.05 + 1e-6, f'out={out_v():.3f}')
        f.wall_x = None
        f.spin_for(2.5)
        check('wall removed: motion resumes', out_v() > 0.4, f'out={out_v():.3f}')

        f.scan_on = False
        f.spin_for(2.0)
        check('stale scan: zero command', out_v() == 0.0 and f.status.get('fault') == 'scan_stale',
              f'out={out_v():.3f} fault={f.status.get("fault")}')
        f.scan_on = True
        f.spin_for(0.3)
        check('scan back: still held (hysteresis)', out_v() == 0.0, f'out={out_v():.3f}')
        f.spin_for(2.5)
        check('scan recovered: motion resumes', out_v() > 0.4 and f.status.get('fault') == 'none',
              f'out={out_v():.3f}')

        f.scan_nan = True
        f.spin_for(1.5)
        check('invalid (NaN) scan: zero', out_v() == 0.0, f'fault={f.status.get("fault")}')
        f.scan_nan = False
        f.spin_for(3.0)

        f.tf_on = False
        f.spin_for(1.5)
        check('lost TF: zero', out_v() == 0.0, f'fault={f.status.get("fault")}')
        f.tf_on = True
        f.spin_for(3.0)

        f.map_on = False
        f.spin_for(5.0)
        check('stale costmap: zero', out_v() == 0.0 and f.status.get('fault') == 'costmap_stale',
              f'fault={f.status.get("fault")}')
        f.map_on = True
        f.spin_for(3.5)
        check('costmap recovered: motion resumes', out_v() > 0.4, f'out={out_v():.3f}')

        f.cmd_v = 0.2
        f.spin_for(1.0)
        check('lower request is respected', out_v() <= 0.2 + 1e-9, f'out={out_v():.3f}')
    finally:
        node.terminate()
        node.wait()
        rclpy.shutdown()
    print('ALL PASSED' if not failures else f'FAILED: {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
