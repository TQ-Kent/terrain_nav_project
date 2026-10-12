#!/usr/bin/env python3
"""Print markdown tables (median over trials) from docs/results/benchmark.csv."""
import csv
import os
import statistics as st
import sys

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs', 'results', 'benchmark.csv')
rows = list(csv.DictReader(open(path)))
groups = {}
for r in rows:
    groups.setdefault((r['group'], r['name']), []).append(r)


def num(rs, k):
    v = [float(r[k]) for r in rs if r[k] not in ('', None)]
    return v


for group in ('version', 'speed', 'controller', 'planner'):
    items = [(n, rs) for (g, n), rs in groups.items() if g == group]
    if not items:
        continue
    print(f'\n### {group}\n')
    print('| config | trials | reached | sim time s (each) | path m (median) | avg speed m/s (median) | wall contacts |')
    print('| :-- | --: | --: | :-- | --: | --: | --: |')
    for name, rs in items:
        ok = [r for r in rs if r['outcome'] == 'reached']
        t = ', '.join(f"{float(r['sim_time_s']):.0f}" if r['outcome'] == 'reached' else r['outcome']
                      for r in rs)
        p, v = num(ok, 'path_m'), num(ok, 'avg_speed_mps')
        c = ','.join(r['wall_contacts'] or '?' for r in rs)
        print(f"| {name} | {len(rs)} | {len(ok)}/{len(rs)} | {t} | "
              f"{st.median(p):.1f} | {st.median(v):.3f} | {c} |" if ok else
              f"| {name} | {len(rs)} | 0/{len(rs)} | {t} | - | - | {c} |")
