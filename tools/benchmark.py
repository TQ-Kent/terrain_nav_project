#!/usr/bin/env python3
"""Run maze trials without Unity (built-in headless simulator) and write a CSV.

    source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
    python3 tools/benchmark.py                 # full suite -> docs/results/benchmark.csv
    python3 tools/benchmark.py --only speed    # one group: version | speed | controller | planner
    python3 tools/benchmark.py --dry-run

Speed-governor comparison (A Smac2D+RPP, B +governor, C Smac2D+MPPI, D +governor), headless only:
    python3 tools/benchmark.py --only governor --split dev --caps 0.5 --trials 1      # pilot
    python3 tools/benchmark.py --only governor --split test --caps 0.5 0.8 --trials 5  # held-out
  Seeds: dev = 1-5 (tune on these), test = 101-105 (report these, never tune on them); --seeds
  overrides. Mazes come from tools/gen_maze.py (seed 42 = the built-in maze). Output goes to
  docs/results/governor_benchmark.csv (+ .jsonl with the full per-trial summaries); existing
  result files are never overwritten, rows are appended.

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


GOV_CONFIGS = {  # name -> (controller, governor on)
    'A_RPP': ('RPP', False), 'B_RPP_gov': ('RPP', True),
    'C_MPPI': ('MPPI', False), 'D_MPPI_gov': ('MPPI', True)}
SEED_SPLITS = {'dev': [1, 2, 3, 4, 5], 'test': [101, 102, 103, 104, 105]}


def governor_suite(seeds, caps, trials, configs, extra=''):
    """(group, name, trials, overrides, seed, config, cap) for every config x cap x seed."""
    runs = []
    for cap in caps:
        for seed in seeds:
            for cfg in configs:
                ctrl, on = GOV_CONFIGS[cfg]
                ov = (f'{COMMON},{speed(cap)},algorithms.planner=Smac2D,algorithms.controller={ctrl},'
                      f'governor.enabled={str(on).lower()},localization=ground_truth')
                if extra:
                    ov += ',' + extra
                runs.append(('governor', f'gov_{cfg}_cap{cap}_seed{seed}', trials, ov, seed, cfg, cap))
    return runs


def make_layout(seed, logdir):
    """Layout file for a seed ('' = built-in seed-42 maze)."""
    if seed == 42:
        return ''
    path = os.path.join(logdir, f'maze_seed{seed}.txt')
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'gen_maze.py'), '--seed', str(seed),
                    '--out', path], check=True)
    return path


RECOVERY_RE = re.compile(r'Running (spin|backup|wait|drive_on_heading|assisted_teleop)')
PROGRESS_FAIL_RE = re.compile(r'Failed to make progress')
MISSED_RE = re.compile(r'Control loop missed its desired rate')


def run_trial(group, name, trial, overrides, logdir, wall_limit, layout=''):
    result_file = os.path.join(logdir, f'{name}_{trial}.json')
    if os.path.exists(result_file):
        os.remove(result_file)
    log_path = os.path.join(logdir, f'{name}_{trial}.log')
    if layout:
        overrides = f'{overrides},headless.layout={layout}'
    cmd = ['ros2', 'launch', 'terrain_nav_bringup', 'bringup_maze_2d.launch.py',
           'simulator:=headless', f'set:={overrides},exploration.result_file={result_file}']
    rec = None
    rec_prefix = os.path.join(logdir, f'{name}_{trial}')
    if group == 'governor':
        rec_cmd = [sys.executable, os.path.join(ROOT, 'tools', 'governor_log.py'), '--prefix', rec_prefix]
        rec_cmd += ['--layout', layout or os.path.join(ROOT, 'ros2_ws', 'src', 'terrain_navigation',
                                                      'maps', 'maze_seed42.txt')]
        rec = subprocess.Popen(rec_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               env={**os.environ, 'ROS_LOCALHOST_ONLY': '1'})
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
    summary = {}
    if rec:
        rec.send_signal(signal.SIGINT)
        try:
            rec.wait(timeout=30)
        except subprocess.TimeoutExpired:
            rec.kill()
        try:
            summary = json.load(open(rec_prefix + '_summary.json'))
        except (OSError, ValueError):
            summary = {}
    row = {'group': group, 'name': name, 'trial': trial, 'wall_s': round(time.time() - t0),
           'outcome': 'no_result', 'sim_time_s': '', 'path_m': '', 'avg_speed_mps': '',
           'frontier_goals': '', 'failed_targets': '', 'wall_contacts': '', 'overrides': overrides}
    if os.path.exists(result_file):
        row.update({k: v for k, v in json.load(open(result_file)).items() if k in row})
    elif status == 'wall_timeout':
        row['outcome'] = 'wall_timeout'
    text = open(log_path, errors='ignore').read()
    m = re.findall(r'wall contacts: (\d+)', text)
    row['wall_contacts'] = m[-1] if m else ''
    if group == 'governor':
        row.update(summary=summary, recoveries=len(RECOVERY_RE.findall(text)),
                   progress_failures=len(PROGRESS_FAIL_RE.findall(text)),
                   controller_missed_rate=len(MISSED_RE.findall(text)))
    return row


def main_governor(args):
    seeds = args.seeds or SEED_SPLITS[args.split]
    split = 'custom' if args.seeds else args.split
    runs = governor_suite(seeds, args.caps, args.trials, args.configs, args.set)
    total = sum(r[2] for r in runs)
    print(f'{total} trials, seeds {seeds} ({split}), caps {args.caps}')
    if args.dry_run:
        for r in runs:
            print(*r)
        return 0
    out = args.out if not args.out.endswith('benchmark.csv') else os.path.join(
        ROOT, 'docs', 'results', 'governor_benchmark.csv')
    fields = ['split', 'seed', 'config', 'cap', 'trial', 'outcome', 'sim_time_s', 'path_m',
              'avg_speed_mps', 'v_mean', 'v_p95', 'wall_contacts', 'min_clearance_true_m',
              'stalled_s', 'recoveries', 'progress_failures', 'controller_missed_rate',
              'scan_age_max_s', 'costmap_age_max_s', 'governor_compute_us_p95', 'wall_s', 'overrides']
    new_file = not os.path.exists(out)
    done = 0
    with open(out, 'a', newline='') as f, open(os.path.splitext(out)[0] + '.jsonl', 'a') as jl:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        if new_file:
            w.writeheader()
        for group, name, reps, overrides, seed, cfg, cap in runs:
            layout = make_layout(seed, args.logdir)
            for trial in range(1, reps + 1):
                row = run_trial(group, name, trial, overrides, args.logdir, args.wall_limit, layout)
                flat = {**row, **row.get('summary', {}), 'split': split, 'seed': seed,
                        'config': cfg, 'cap': cap}
                w.writerow(flat)
                f.flush()
                jl.write(json.dumps({**flat, 'summary': row.get('summary', {})}) + '\n')
                jl.flush()
                done += 1
                print(f"[{done}/{total}] {name}#{trial}: {row['outcome']} t={row['sim_time_s']} "
                      f"path={row['path_m']} contacts={row['wall_contacts']} "
                      f"stalled={flat.get('stalled_s')}", flush=True)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', choices=['version', 'speed', 'controller', 'planner', 'combo', 'governor'])
    ap.add_argument('--split', choices=list(SEED_SPLITS), default='dev', help='governor: seed set')
    ap.add_argument('--seeds', type=int, nargs='+', help='governor: explicit maze seeds')
    ap.add_argument('--caps', type=float, nargs='+', default=[0.5], help='governor: max_linear values')
    ap.add_argument('--trials', type=int, default=1, help='governor: trials per config and seed')
    ap.add_argument('--configs', nargs='+', choices=list(GOV_CONFIGS), default=list(GOV_CONFIGS))
    ap.add_argument('--set', default='', help='governor: extra comma-separated overrides, e.g. governor.a_lat_max=0.8')
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'results', 'benchmark.csv'))
    ap.add_argument('--logdir', default='/tmp/terrain_bench')
    ap.add_argument('--wall-limit', type=int, default=480)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.logdir, exist_ok=True)
    if args.only == 'governor':
        return main_governor(args)
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
