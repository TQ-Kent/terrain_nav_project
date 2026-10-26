#!/usr/bin/env python3
"""Summarise governor benchmark CSVs: success rate next to every time, per config and cap.

    python3 tools/summarize_governor.py docs/results/governor_test_cap0.5.csv docs/results/governor_test_cap0.8.csv

Times, speeds and path lengths are averaged over *reached* trials only and always shown next to the
number of reached/total trials; failures are listed by outcome. Also prints a paired comparison
(governor on vs off, same controller, cap and seed) of the per-seed mean time over seeds where
both sides reached the goal in every trial, with the number of such seeds.
"""
import csv
import statistics as st
import sys
from collections import defaultdict


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def mean(v):
    v = [x for x in v if x is not None]
    return st.mean(v) if v else None


def fmt(x, p=2):
    return '-' if x is None else f'{x:.{p}f}'


def main(paths):
    rows = []
    for p in paths:
        rows += list(csv.DictReader(open(p)))
    groups = defaultdict(list)
    for r in rows:
        groups[(r['cap'], r['config'])].append(r)
    print('| cap | config | reached/total | time s (reached) | path m | v mean | v p95 | min clear m (all) '
          '| wall contacts (sum) | stalled s (mean) | recoveries (sum) | missed rate (sum) | failures |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for (cap, cfg), rs in sorted(groups.items()):
        ok = [r for r in rs if r['outcome'] == 'reached']
        fails = defaultdict(int)
        for r in rs:
            if r['outcome'] != 'reached':
                fails[r['outcome']] += 1
        print(f"| {cap} | {cfg} | {len(ok)}/{len(rs)} | {fmt(mean([num(r['sim_time_s']) for r in ok]), 1)} "
              f"| {fmt(mean([num(r['path_m']) for r in ok]), 1)} "
              f"| {fmt(mean([num(r['v_mean']) for r in ok]))} | {fmt(mean([num(r['v_p95']) for r in ok]))} "
              f"| {fmt(min([num(r['min_clearance_true_m']) for r in rs if num(r['min_clearance_true_m']) is not None] or [None]), 3)} "
              f"| {sum(int(r['wall_contacts'] or 0) for r in rs)} | {fmt(mean([num(r['stalled_s']) for r in rs]), 1)} "
              f"| {sum(int(r['recoveries'] or 0) for r in rs)} | {sum(int(r['controller_missed_rate'] or 0) for r in rs)} "
              f"| {dict(fails) or '-'} |")
    print()
    pairs = (('A_RPP', 'B_RPP_gov'), ('C_MPPI', 'D_MPPI_gov'))
    for cap in sorted({r['cap'] for r in rows}):
        for off, on in pairs:
            per = defaultdict(lambda: defaultdict(list))
            for r in rows:
                if r['cap'] == cap and r['config'] in (off, on):
                    per[r['seed']][r['config']].append(r)
            diffs, n_seeds = [], 0
            for seed, d in per.items():
                n_seeds += 1
                if all(len(d[c]) and all(x['outcome'] == 'reached' for x in d[c]) for c in (off, on)):
                    diffs.append(mean([num(x['sim_time_s']) for x in d[on]]) -
                                 mean([num(x['sim_time_s']) for x in d[off]]))
            if diffs:
                sd = st.stdev(diffs) if len(diffs) > 1 else float('nan')
                print(f'cap {cap} {on} - {off}: mean time difference {st.mean(diffs):+.1f} s '
                      f'(sd {sd:.1f}) over {len(diffs)} of {n_seeds} seeds where every trial of both reached the goal')
            else:
                print(f'cap {cap} {on} - {off}: no seed with all-reached trials on both sides')


if __name__ == '__main__':
    main(sys.argv[1:])
