#!/usr/bin/env python3
"""Measure reaction delay and effective deceleration of the simulated robot.

Needs a sourced workspace and an open-field layout (a text file with only a border wall):
    python3 tools/measure_braking.py --layout /tmp/open_field.txt [--speeds 0.3 0.5 0.8 1.0] [--out docs/results/braking.json]
(without --layout it expects a headless_sim to be running already)

For each speed v: command v for 3 s, then command 0 and record the true speed from
/sim/ground_truth. Reports
  delay_s   time from the stop command until speed first drops below 98 % of v
  decel     mean deceleration between 90 % and 10 % of v
  stop_dist distance travelled from the stop command to rest
Results are specific to the headless simulator; Unity must be measured separately.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class Meter(Node):
    def __init__(self):
        super().__init__('braking_meter')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/sim/ground_truth', self.on_gt, 50)
        self.samples = []  # (sim time, x, y): sim stamps avoid wall-clock jitter in the DDS callbacks

    def on_gt(self, m):
        self.samples.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.position.x, m.pose.position.y))

    def speed_series(self, t0):
        out = []
        for a, b in zip(self.samples, self.samples[1:]):
            if b[0] > a[0] and a[0] >= t0 - 0.05:
                out.append((b[0], math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0])))
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', default='', help='open-field layout file; spawns headless_sim for you')
    ap.add_argument('--speeds', type=float, nargs='+', default=[0.3, 0.5, 0.8, 1.0])
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    sim = None
    if args.layout:
        # new session so the whole process group (ros2 run wrapper + the real node) can be killed
        sim = subprocess.Popen(['ros2', 'run', 'terrain_navigation', 'headless_sim', '--ros-args',
                                '-p', f'layout:={args.layout}'], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, start_new_session=True)
        time.sleep(3.0)
    rclpy.init()
    n = Meter()

    def spin(sec, cmd):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            n.pub.publish(cmd)
            rclpy.spin_once(n, timeout_sec=0.02)

    results = []
    for v in args.speeds:
        spin(1.0, Twist())  # settle
        go = Twist()
        go.linear.x = v
        spin(3.0, go)
        n.samples.clear()
        spin(1.0, go)       # steady-state samples before the stop (/sim/ground_truth is 10 Hz)
        x0 = n.samples[-1][1:3]
        t_stop = n.samples[-1][0]
        spin(2.0, Twist())
        ser = [s for s in n.speed_series(t_stop - 0.6) if s[0] >= t_stop - 0.6]
        sm = ser  # 0.1 s position differences: time resolution of every result below is 0.1 s
        v0 = sum(s[1] for s in sm if s[0] < t_stop) / max(1, sum(1 for s in sm if s[0] < t_stop))
        after = [s for s in sm if s[0] >= t_stop]
        t98 = next((s[0] for s in after if s[1] < 0.98 * v0), None)  # first sample already slower
        t90 = next((s[0] for s in after if s[1] < 0.90 * v0), None)
        t10 = next((s[0] for s in after if s[1] < 0.10 * v0), None)
        rest = n.samples[-1][1:3]
        dec = (0.8 * v0) / (t10 - t90) if t90 and t10 and t10 > t90 else None
        results.append({'cmd_v': v, 'measured_v': round(v0, 3),
                        'delay_s': round(t98 - t_stop, 3) if t98 else None,
                        'decel_mps2': round(dec, 2) if dec else None,
                        'stop_dist_m': round(math.hypot(rest[0] - x0[0], rest[1] - x0[1]), 3)})
        print(results[-1])
        back = Twist()
        back.linear.x = -0.3
        spin(3.0, back)     # keep the robot inside the field for the next run
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(results, f, indent=1)
    rclpy.shutdown()
    if sim:
        os.killpg(sim.pid, signal.SIGTERM)


if __name__ == '__main__':
    main()
