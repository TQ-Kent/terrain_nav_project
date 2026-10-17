#!/usr/bin/env python3
"""Find out where a maze run loses time: record it, then classify every halt.

    # terminal 1 (headless simulator; Unity must not be in Play mode)
    ros2 launch terrain_nav_bringup bringup_maze_2d.launch.py set:=rviz=false,exploration.result_file=/tmp/run.json
    # terminal 2, started before the launch ends
    python3 tools/halt_analysis.py record /tmp/run_trace
    # afterwards
    python3 tools/halt_analysis.py analyze /tmp/run_trace

record writes <prefix>_samples.csv (20 Hz: sim time, pose, speed, the three cmd_vel stages) and
<prefix>_events.csv (Nav2 NavigateToPose status changes and new global plans, in sim time).
analyze splits the run, from the first goal to the last sample, into
    moving   speed >= 80 % of the top speed seen
    slow     moving, but below that
    halted   speed < HALT_V for at least MIN_HALT s, split by cause (see classify)
"""
import csv
import math
import sys
import time

HALT_V = 0.05          # m/s measured speed below which the robot counts as stopped
MIN_HALT = 0.3         # s a stop must last to be reported
STATUS = {1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def record(prefix):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Path
    from action_msgs.msg import GoalStatusArray
    from rosgraph_msgs.msg import Clock

    class Rec(Node):
        def __init__(self):
            super().__init__('halt_recorder')
            self.t = None
            self.pose = None
            self.cmd = {'nav': 0.0, 'smooth': 0.0, 'final': 0.0}
            self.az = 0.0
            self.status = {}
            self.samples = open(f'{prefix}_samples.csv', 'w', newline='')
            self.events = open(f'{prefix}_events.csv', 'w', newline='')
            self.sw, self.ew = csv.writer(self.samples), csv.writer(self.events)
            self.sw.writerow(['t', 'x', 'y', 'cmd_nav', 'cmd_smooth', 'cmd_final', 'az_nav'])
            self.ew.writerow(['t', 'event', 'detail'])
            be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Clock, '/clock', self.on_clock, be)
            self.create_subscription(PoseStamped, '/sim/ground_truth', self.on_pose, 10)
            for k, topic in (('nav', '/cmd_vel_nav'), ('smooth', '/cmd_vel_smoothed'), ('final', '/cmd_vel')):
                self.create_subscription(Twist, topic, lambda m, k=k: self.on_cmd(k, m), 10)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, 10)
            self.create_subscription(Path, '/plan', lambda m: self.event('PLAN', len(m.poses)), 10)
            self.create_timer(0.05, self.sample)

        def on_cmd(self, k, m):
            self.cmd[k] = m.linear.x
            if k == 'nav':
                self.az = m.angular.z

        def on_clock(self, m):
            self.t = m.clock.sec + m.clock.nanosec * 1e-9

        def on_pose(self, m):
            self.pose = (m.pose.position.x, m.pose.position.y)

        def event(self, name, detail=''):
            if self.t is not None:
                self.ew.writerow([f'{self.t:.3f}', name, detail])

        def on_status(self, m):
            for s in m.status_list:
                uid = bytes(s.goal_info.goal_id.uuid).hex()[:6]
                if self.status.get(uid) != s.status:
                    self.status[uid] = s.status
                    self.event(STATUS.get(s.status, s.status), uid)

        def sample(self):
            if self.t is None or self.pose is None:
                return
            self.sw.writerow([f'{self.t:.3f}', f'{self.pose[0]:.4f}', f'{self.pose[1]:.4f}',
                              f'{self.cmd["nav"]:.3f}', f'{self.cmd["smooth"]:.3f}', f'{self.cmd["final"]:.3f}', f'{self.az:.3f}'])

    rclpy.init()
    node = Rec()
    print(f'recording to {prefix}_*.csv ; Ctrl-C (or the end of the run) stops it')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.t is not None and getattr(node, 'last_flush', 0) < time.time() - 2:
                node.samples.flush()
                node.events.flush()
                node.last_flush = time.time()
    except KeyboardInterrupt:
        pass
    node.samples.close()
    node.events.close()


def load(prefix):
    s = [[float(v) for v in r] for r in list(csv.reader(open(f'{prefix}_samples.csv')))[1:]]
    e = [(float(r[0]), r[1], r[2]) for r in list(csv.reader(open(f'{prefix}_events.csv')))[1:]]
    return s, e


def classify(t, cmd, goal_active, since_goal, since_end, az=0.0):
    """Cause of a halt at time t. cmd = (nav, smooth, final) forward commands at that time."""
    nav, smooth, final = cmd
    if not goal_active:
        return 'between goals (no active Nav2 goal)'
    if since_goal is not None and since_goal < 1.5:
        return 'goal just sent (accept / plan / controller start)'
    if abs(nav) < 0.02 and abs(az) > 0.05:
        return 'goal active, rotating in place (controller turns, no forward speed)'
    if abs(nav) < 0.02:
        return 'goal active, controller commands neither forward nor turning'
    if abs(smooth) < 0.02:
        return 'goal active, velocity smoother output zero'
    if abs(final) < 0.02:
        return 'goal active, collision monitor output zero'
    return 'goal active, commanded but not moving (accel lag / blocked)'


def analyze(prefix):
    samples, events = load(prefix)
    if len(samples) < 20:
        print('too few samples')
        return
    full = list(samples)                                  # whole recording, including a stalled tail
    # speed over a 0.5 s window; samples that share a timestamp (pose republished) are dropped first
    samples = [x for i, x in enumerate(samples) if i == 0 or x[0] > samples[i - 1][0]]
    n, w = len(samples), 10
    speed = [0.0] * n
    for i in range(w, n):
        dt = samples[i][0] - samples[i - w][0]
        if dt > 0:
            speed[i] = math.hypot(samples[i][1] - samples[i - w][1], samples[i][2] - samples[i - w][2]) / dt
    vmax = sorted(speed)[int(0.98 * n)]                  # 98th percentile: robust against a single spike
    # one interval per Nav2 goal id: first EXECUTING -> terminal status
    begin, intervals = {}, []
    for tt, name, uid in events:
        if name in ('ACCEPTED', 'EXECUTING') and uid not in begin:
            begin[uid] = tt
        elif name in ('SUCCEEDED', 'CANCELED', 'ABORTED') and uid in begin:
            intervals.append((begin.pop(uid), tt, name, uid))
    first = min(a for a, *_ in intervals) if intervals else samples[0][0]
    ends = [b for a, b, r, u in intervals if r == 'SUCCEEDED']
    t_end = max(ends) if ends else samples[-1][0]
    # an ABORTED goal that ends within 0.3 s of another goal starting was pre-empted by it, not a failure
    starts = sorted(a for a, *_ in intervals)
    def preempted(b):
        return any(0 <= st - b <= 0.3 or 0 <= b - st <= 0.3 for st in starts if st != b)
    intervals = [(a, b, 'PREEMPTED' if r == 'ABORTED' and preempted(b) else r, u) for a, b, r, u in intervals]
    samples = [x for x in samples if x[0] <= t_end]
    n = len(samples)
    speed = speed[:n]

    def active(tt):
        for a, b, *_ in intervals:
            if a <= tt <= b:
                return True, tt - a
        return False, None

    buckets = {'moving (>= 80 % of top speed)': 0.0, 'slow (moving below 80 %)': 0.0}
    halts = {}
    longest = []
    i0 = next(i for i, s in enumerate(samples) if s[0] >= first)
    run_start, run_len = None, 0.0
    for i in range(i0, n):
        tt = samples[i][0]
        dt = tt - samples[i - 1][0] if i > 0 else 0.0
        if speed[i] >= 0.8 * vmax:
            buckets['moving (>= 80 % of top speed)'] += dt
        elif speed[i] >= HALT_V:
            buckets['slow (moving below 80 %)'] += dt
        halted = speed[i] < HALT_V
        if halted:
            if run_start is None:
                run_start, run_len = i, 0.0
            run_len += dt
        if (not halted or i == n - 1) and run_start is not None:
            if run_len >= MIN_HALT:
                mid = samples[run_start][0]
                act, since = active(mid)
                cause = classify(mid, tuple(samples[run_start][3:6]), act, since, None, samples[run_start][6] if len(samples[run_start]) > 6 else 0.0)
                halts.setdefault(cause, [0, 0.0])
                halts[cause][0] += 1
                halts[cause][1] += run_len
                longest.append((run_len, mid, cause))
            run_start = None
    total = t_end - first
    cnt = lambda k: sum(1 for *_, r, u in intervals if r == k)
    print(f'run {prefix}: {total:.1f} s from the first goal to the last, typical top speed {vmax:.2f} m/s, '
          f'{len(intervals)} Nav2 goals ({cnt("SUCCEEDED")} reached with a stop, {cnt("PREEMPTED")} replaced '
          f'while moving, {cnt("ABORTED")} aborted, {cnt("CANCELED")} cancelled)')
    print(f'{"":58s}{"time s":>8s}{"share":>8s}{"count":>7s}')
    for k, v in buckets.items():
        print(f'{k:58s}{v:8.1f}{100 * v / total:7.1f}%')
    for k, (cnt, v) in sorted(halts.items(), key=lambda kv: -kv[1][1]):
        print(f'halted: {k:50s}{v:8.1f}{100 * v / total:7.1f}%{cnt:7d}')
    print('longest halts (s, sim time, cause):')
    for ln, mid, cause in sorted(longest, reverse=True)[:6]:
        print(f'  {ln:5.1f}  t={mid:7.1f}  {cause}')
    # why is the robot slow? label every slow sample by what the commands were doing
    succ = sorted(b for a, b, r, u in intervals if r == 'SUCCEEDED')
    halt_ends = []
    for i in range(i0 + 1, n):
        if speed[i - 1] < HALT_V <= speed[i]:
            halt_ends.append(samples[i][0])
    slow = {'braking toward a goal stop (2 s before it)': 0.0, 'speeding up after a stop (2 s after)': 0.0,
            'controller commanding below 80 % (curves, obstacles)': 0.0,
            'commanded full speed but not there yet': 0.0}
    for i in range(i0, n):
        if not (HALT_V <= speed[i] < 0.8 * vmax):
            continue
        tt, dt = samples[i][0], samples[i][0] - samples[i - 1][0]
        nav = samples[i][3]
        if any(0 <= b - tt <= 2.0 for b in succ):
            slow['braking toward a goal stop (2 s before it)'] += dt
        elif any(0 <= tt - h <= 2.0 for h in halt_ends):
            slow['speeding up after a stop (2 s after)'] += dt
        elif nav < 0.8 * vmax:
            slow['controller commanding below 80 % (curves, obstacles)'] += dt
        else:
            slow['commanded full speed but not there yet'] += dt
    print('slow time split:')
    for k, v in slow.items():
        print(f'  {k:58s}{v:7.1f} s{100 * v / total:6.1f}%')
    # command killed after the controller: nav output wants to move, the final /cmd_vel is zero. This is the
    # collision monitor holding a robot that is touching a wall (or, rarely, the smoother). The window above ends
    # at the last successful goal, so this is measured over the whole recording and catches a stalled tail.
    lock = sum(b[0] - a[0] for a, b in zip(full, full[1:]) if a[3] > 0.3 and a[5] < 0.02)
    tail = [x for x in full if x[0] >= full[-1][0] - 35]
    lock_tail = sum(b[0] - a[0] for a, b in zip(tail, tail[1:]) if a[3] > 0.3 and a[5] < 0.02)
    print(f'controller commanded >0.3 m/s but final /cmd_vel was 0: {lock:.1f} s in the whole recording, '
          f'{lock_tail:.1f} s of its last 35 s' + ('   <-- locked against a wall' if lock_tail > 10 else ''))
    # latency from a goal being accepted to the robot moving
    lat = []
    for a, b, res, u in intervals:
        for i in range(i0, n):
            if samples[i][0] >= a and speed[i] >= 0.1:
                lat.append(samples[i][0] - a)
                break
    if lat:
        print(f'time from goal accepted to first movement: mean {sum(lat) / len(lat):.2f} s, '
              f'max {max(lat):.2f} s over {len(lat)} goals')
    # time spent braking to a stop at each goal end (speed falling from 80 % to halt)
    dur = [b - a for a, b, r, u in intervals if r == 'SUCCEEDED']
    if dur:
        print(f'successful goals last {sum(dur) / len(dur):.1f} s on average (mean path to a frontier)')


if __name__ == '__main__':
    if len(sys.argv) != 3 or sys.argv[1] not in ('record', 'analyze'):
        sys.exit(__doc__)
    (record if sys.argv[1] == 'record' else analyze)(sys.argv[2])
