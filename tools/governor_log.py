#!/usr/bin/env python3
"""Per-trial recorder for the speed-governor benchmark (headless simulator only).

    python3 tools/governor_log.py --prefix /tmp/run1 [--layout maze.txt] [--radius 0.2]

Samples at every /sim/ground_truth message (10 Hz sim time): true pose, actual speed, and the
latest requested (/cmd_vel_nav), smoothed (/cmd_vel_smoothed), governed (/cmd_vel_governed, only
present when the governor is enabled) and final (/cmd_vel) linear speeds, plus the governor's own
/speed_governor/status fields. On SIGINT/SIGTERM it writes <prefix>_samples.csv and
<prefix>_summary.json (speeds, true minimum wall clearance from the layout file, stalled time,
scan/costmap staleness, governor compute time).

True clearance is computed from the maze text file and the true pose, so it is independent of
the costmap the governor itself uses. Costmap/scan staleness = /clock minus message stamp at
the time the message is received.
"""
import argparse
import csv
import json
import math
import signal
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

CHAR = 0.5
SHIFT = 1.25


def wall_cells(path):
    rows = [r.rstrip('\n') for r in open(path) if not r.startswith('//') and r.strip()]
    n = len(rows)
    cells = []
    for r, line in enumerate(rows):
        cz = n - 1 - r
        for cx, ch in enumerate(line):
            if ch == '#':
                # ROS x = z_unity axis, y = -x_unity axis; cell spans below in ROS coordinates
                x0, x1 = cz * CHAR - SHIFT, cz * CHAR - SHIFT + CHAR
                y1, y0 = -(cx * CHAR - SHIFT), -(cx * CHAR - SHIFT + CHAR)
                cells.append((x0, x1, y0, y1))
    return np.array(cells)


def clearance_to_walls(cells, x, y, radius):
    """Distance from circle (x, y, radius) to the nearest wall rectangle (negative = overlap)."""
    dx = np.maximum(np.maximum(cells[:, 0] - x, x - cells[:, 1]), 0.0)
    dy = np.maximum(np.maximum(cells[:, 2] - y, y - cells[:, 3]), 0.0)
    return float(np.min(np.hypot(dx, dy)) - radius)


class Recorder(Node):
    def __init__(self, a):
        super().__init__('governor_log', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=False)])
        self.a = a
        self.cmd = {k: float('nan') for k in ('nav', 'smoothed', 'governed', 'final')}
        self.status = {}
        self.clock = 0.0
        self.rows = []
        self.scan_age, self.map_age, self.compute_us = [], [], []
        self.prev = None
        self.create_subscription(Clock, '/clock', lambda m: setattr(
            self, 'clock', m.clock.sec + m.clock.nanosec * 1e-9), 10)
        for key, topic in (('nav', '/cmd_vel_nav'), ('smoothed', '/cmd_vel_smoothed'),
                           ('governed', '/cmd_vel_governed'), ('final', '/cmd_vel')):
            self.create_subscription(Twist, topic, lambda m, k=key: self.cmd.__setitem__(
                k, m.linear.x), 10)
        self.create_subscription(PoseStamped, '/sim/ground_truth', self.on_gt, 50)
        self.create_subscription(String, '/speed_governor/status', self.on_status, 10)
        self.create_subscription(LaserScan, '/scan', lambda m: self.scan_age.append(
            self.clock - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), qos_profile_sensor_data)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: self.map_age.append(
            self.clock - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), qos)

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
            self.compute_us.append(self.status['compute_us'])
        except (ValueError, KeyError):
            pass

    def on_gt(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y = m.pose.position.x, m.pose.position.y
        v = 0.0
        if self.prev and t > self.prev[0]:
            v = math.hypot(x - self.prev[1], y - self.prev[2]) / (t - self.prev[0])
        self.prev = (t, x, y)
        s = self.status
        self.rows.append({
            't': round(t, 3), 'x': round(x, 3), 'y': round(y, 3), 'v_actual': round(v, 3),
            'v_req': self.cmd['nav'], 'v_smoothed': self.cmd['smoothed'],
            'v_governed': self.cmd['governed'], 'v_final': self.cmd['final'],
            'gov_reason': s.get('reason', ''), 'gov_fault': s.get('fault', ''),
            'gov_clearance': s.get('clearance', ''), 'gov_d_free': s.get('d_free', ''),
            'stall_hint': s.get('stall_hint', '')})

    def finish(self):
        rows = self.rows
        with open(self.a.prefix + '_samples.csv', 'w', newline='') as f:
            if rows:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
        v = np.array([r['v_actual'] for r in rows]) if rows else np.zeros(1)
        started = np.argmax(v > 0.05) if (v > 0.05).any() else 0
        vm = v[started:]
        dt = np.diff([r['t'] for r in rows]) if len(rows) > 1 else np.array([])
        req = np.array([r['v_req'] for r in rows], dtype=float)
        # Only forward time steps count: a repeated or backwards sim clock (e.g. two simulators
        # running by mistake) must not make the total negative.
        stalled = float(np.sum(np.clip(dt, 0.0, 1.0)[((req[1:] > 0.2) & (v[1:] < 0.02))])) if len(dt) else 0.0
        out = {'samples': len(rows),
               'v_mean': float(np.mean(vm)) if len(vm) else None,
               'v_p95': float(np.percentile(vm, 95)) if len(vm) else None,
               'v_max': float(np.max(v)),
               'stalled_s': round(stalled, 2)}
        if self.a.layout and rows:
            cells = wall_cells(self.a.layout)
            cl = [clearance_to_walls(cells, r['x'], r['y'], self.a.radius) for r in rows]
            out['min_clearance_true_m'] = round(min(cl), 3)
            out['frac_time_clearance_lt_0.1m'] = round(float(np.mean(np.array(cl) < 0.1)), 4)
        for name, arr in (('scan_age', self.scan_age), ('costmap_age', self.map_age)):
            if arr:
                out[f'{name}_mean_s'] = round(float(np.mean(arr)), 3)
                out[f'{name}_max_s'] = round(float(np.max(arr)), 3)
        if self.compute_us:
            out['governor_compute_us_mean'] = round(float(np.mean(self.compute_us)), 1)
            out['governor_compute_us_p95'] = round(float(np.percentile(self.compute_us, 95)), 1)
            out['governor_compute_us_max'] = round(float(np.max(self.compute_us)), 1)
        if 'gov_reason' in (rows[0] if rows else {}):
            reasons = [r['gov_reason'] for r in rows if r['gov_reason']]
            for k in set(reasons):
                out[f'frac_reason_{k}'] = round(reasons.count(k) / len(reasons), 4)
        with open(self.a.prefix + '_summary.json', 'w') as f:
            json.dump(out, f, indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prefix', required=True)
    ap.add_argument('--layout', default='')
    ap.add_argument('--radius', type=float, default=0.2, help='robot collision radius of the simulator')
    a = ap.parse_args()
    rclpy.init()
    n = Recorder(a)
    done = []
    signal.signal(signal.SIGINT, lambda *_: done.append(1))
    signal.signal(signal.SIGTERM, lambda *_: done.append(1))
    while rclpy.ok() and not done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.finish()
    return 0


if __name__ == '__main__':
    sys.exit(main())
