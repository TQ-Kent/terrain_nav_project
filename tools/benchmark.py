#!/usr/bin/env python3
"""Run maze trials without Unity (built-in headless simulator) and write a CSV.

    source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
    python3 tools/benchmark.py                 # full suite -> docs/results/benchmark.csv
    python3 tools/benchmark.py --only speed    # one group: version | speed | controller | planner
    python3 tools/benchmark.py --dry-run

Each trial is one `bringup_maze_2d.launch.py simulator:=headless` run, configured only through
`set:=` overrides of config/maze_test.yaml. Results come from goal_explorer's result file
(exploration.result_file) and the simulator's wall-contact count.
"""
import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OLD_SPEED = 'speed.max_linear=0.26,speed.max_angular=1.0,speed.linear_accel=2.5,speed.angular_accel=3.2'
COMMON = 'rviz=false,exploration.max_time=300'
# checkpoint-2/3 goal tolerances (yaw 0.25 rad, progress radius 0.5 m), for the version comparison
OLD_TOL = 'tolerance.yaw=0.25,tolerance.progress_radius=0.5'

# max_linear -> (max_angular, linear_accel)
SPEED_STEPS = {0.26: (1.0, 2.5), 0.5: (1.5, 1.5), 0.8: (2.0, 1.5), 1.0: (2.0, 2.0)}


def speed(v):
    w, a = SPEED_STEPS[v]
    return f'speed.max_linear={v},speed.max_angular={w},speed.linear_accel={a}'


def suite():
    runs = []
    # Exploration: checkpoint-2 behaviour vs the current one, at the same (old) speed.
    for name, extra in (
            ('old_direct_slam', 'nav2_profile=baseline_2d,exploration.mode=direct,localization=slam'),
            ('old_direct_gt', 'nav2_profile=baseline_2d,exploration.mode=direct,localization=ground_truth'),
            ('new_frontier_slam', 'localization=slam'),
            ('new_frontier_gt', 'localization=ground_truth')):
        runs.append(('version', name, 1, f'{COMMON},{OLD_SPEED},{OLD_TOL},{extra}'))
    runs.append(('version', 'new_frontier_gt_wide_yaw', 1, f'{COMMON},{OLD_SPEED},localization=ground_truth'))
    for v in SPEED_STEPS:
        runs.append(('speed', f'speed_{v}', 2, f'{COMMON},{speed(v)}'))
    for c in ('FollowPath', 'MPPI', 'RPP', 'Graceful'):
        runs.append(('controller', f'ctrl_{c}', 1, f'{COMMON},{speed(0.5)},algorithms.controller={c}'))
    for p in ('GridBased', 'Smac2D', 'SmacHybrid', 'ThetaStar'):
        runs.append(('planner', f'plan_{p}', 1, f'{COMMON},{speed(0.5)},algorithms.planner={p}'))
    # Every planner x controller pair at the Unity speed cap, 2 trials each.
    for p in ('GridBased', 'Smac2D', 'SmacHybrid', 'ThetaStar'):
        for c in ('FollowPath', 'MPPI', 'RPP', 'Graceful'):
            runs.append(('combo', f'{p}+{c}', 2,
                         f'{COMMON},{speed(1.0)},algorithms.planner={p},algorithms.controller={c}'))
    return runs


def run_trial(group, name, trial, overrides, logdir, wall_limit):
    result_file = os.path.join(logdir, f'{name}_{trial}.json')
    if os.path.exists(result_file):
        os.remove(result_file)
    log_path = os.path.join(logdir, f'{name}_{trial}.log')
    cmd = ['ros2', 'launch', 'terrain_nav_bringup', 'bringup_maze_2d.launch.py',
           'simulator:=headless', f'set:={overrides},exploration.result_file={result_file}']
    t0 = time.time()
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            proc.wait(timeout=wall_limit)
            status = 'exited'
        except subprocess.TimeoutExpired:
            status = 'wall_timeout'
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):   # clean up any leftovers
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            time.sleep(4)
    row = {'group': group, 'name': name, 'trial': trial, 'wall_s': round(time.time() - t0),
           'outcome': 'no_result', 'sim_time_s': '', 'path_m': '', 'avg_speed_mps': '',
           'frontier_goals': '', 'failed_targets': '', 'wall_contacts': '', 'overrides': overrides}
    if os.path.exists(result_file):
        row.update({k: v for k, v in json.load(open(result_file)).items() if k in row})
    elif status == 'wall_timeout':
        row['outcome'] = 'wall_timeout'
    m = re.findall(r'wall contacts: (\d+)', open(log_path, errors='ignore').read())
    row['wall_contacts'] = m[-1] if m else ''
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', choices=['version', 'speed', 'controller', 'planner', 'combo'])
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'results', 'benchmark.csv'))
    ap.add_argument('--logdir', default='/tmp/terrain_bench')
    ap.add_argument('--wall-limit', type=int, default=480)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.logdir, exist_ok=True)
    runs = [r for r in suite() if not args.only or r[0] == args.only]
    fields = ['group', 'name', 'trial', 'outcome', 'sim_time_s', 'path_m', 'avg_speed_mps',
              'frontier_goals', 'failed_targets', 'wall_contacts', 'wall_s', 'overrides']
    total = sum(r[2] for r in runs)
    print(f'{total} trials')
    if args.dry_run:
        for g, n, reps, o in runs:
            print(g, n, reps, o)
        return
    new_file = not os.path.exists(args.out)
    with open(args.out, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if new_file:
            w.writeheader()
        done = 0
        for group, name, reps, overrides in runs:
            for trial in range(1, reps + 1):
                row = run_trial(group, name, trial, overrides, args.logdir, args.wall_limit)
                w.writerow(row)
                f.flush()
                done += 1
                print(f"[{done}/{total}] {name}#{trial}: {row['outcome']} "
                      f"t={row['sim_time_s']} path={row['path_m']} v={row['avg_speed_mps']} "
                      f"contacts={row['wall_contacts']}", flush=True)


if __name__ == '__main__':
    sys.exit(main())
