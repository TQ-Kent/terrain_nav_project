#!/usr/bin/env python3
"""Generate a seeded 6x6 maze in the maze_seed42.txt layout format (headless simulator only).

    python3 tools/gen_maze.py --seed 7 --out /tmp/maze_seed7.txt [--loops 3]

Same geometry rules as the Unity scene: 25x25 characters of 0.5 m, cells of 3x3 interior
characters separated by 1-character walls, robot start in the cell centred on the origin
(bottom-left), default goal (10, -10) in the opposite (top-right) cell. A depth-first search
makes a perfect maze (one route to every cell); --loops removes that many extra wall segments
so planners have route choices. The same seed always gives the same maze. Unity does not read
these files: results on generated mazes are headless-only.
"""
import argparse
import random

CELLS = 6
SIZE = 4 * CELLS + 1


def generate(seed, loops=3, profile='none', obstacles=8):
    rng = random.Random(seed)
    grid = [['#'] * SIZE for _ in range(SIZE)]          # grid[row][col], row 0 = far z

    def centre(i, j):
        return 4 * j + 2, 4 * i + 2

    def clear_cell(i, j):
        for r in range(4 * i + 1, 4 * i + 4):
            for c in range(4 * j + 1, 4 * j + 4):
                grid[r][c] = ' '

    def open_between(a, b):
        (i, j), (k, l) = a, b
        if i == k:                                       # horizontal neighbours: wall column
            c = 4 * min(j, l) + 4
            for r in range(4 * i + 1, 4 * i + 4):
                grid[r][c] = ' '
        else:
            r = 4 * min(i, k) + 4
            for c in range(4 * j + 1, 4 * j + 4):
                grid[r][c] = ' '

    for i in range(CELLS):
        for j in range(CELLS):
            clear_cell(i, j)
    seen = {(0, 0)}
    stack = [(0, 0)]
    edges = set()
    while stack:
        i, j = stack[-1]
        nbrs = [(i + di, j + dj) for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= i + di < CELLS and 0 <= j + dj < CELLS and (i + di, j + dj) not in seen]
        if not nbrs:
            stack.pop()
            continue
        n = rng.choice(nbrs)
        seen.add(n)
        edges.add(frozenset(((i, j), n)))
        open_between((i, j), n)
        stack.append(n)
    closed = [frozenset(((i, j), (i + di, j + dj))) for i in range(CELLS) for j in range(CELLS)
              for di, dj in ((1, 0), (0, 1)) if i + di < CELLS and j + dj < CELLS
              and frozenset(((i, j), (i + di, j + dj))) not in edges]
    rng.shuffle(closed)
    for e in closed[:loops]:
        open_between(*sorted(e))
    # The DFS starts at cell (0, 0) only to carve a spanning tree; every cell ends up connected, so the
    # carving start does not matter. The robot start (bottom-left) and goal (top-right) are fixed by the layout.
    if profile != 'none':
        add_obstacles(grid, random.Random(seed * 7919 + 1), profile, obstacles)
    return [''.join(r) for r in grid]


PROFILES = ('none', 'pillars', 'baffles', 'narrow', 'mixed')


def _free(grid, r, c):
    return grid[r][c] == ' '


def _traversable(grid, start, goal):
    """BFS over 2x2 free windows: true if a 1.0 m wide body can get from start to goal."""
    n = len(grid)
    def ok(r, c):
        return all(0 <= r + a < n and 0 <= c + b < n and _free(grid, r + a, c + b)
                   for a in (0, 1) for b in (0, 1))
    seen = {start}
    todo = [start]
    while todo:
        r, c = todo.pop()
        if (r, c) == goal:
            return True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            q = (r + dr, c + dc)
            if q not in seen and ok(*q):
                seen.add(q)
                todo.append(q)
    return False


def add_obstacles(grid, rng, profile, count):
    """Add grid-aligned wall blocks that keep every corridor at least 2 characters (1.0 m) wide.

    pillars: one character in a cell corner. baffles: 1x2 block along a cell edge (a slalom
    when several line up). narrow: one character of an open doorway, leaving a 1.0 m gap.
    The start cell, goal cell and global connectivity (for 2x2 windows) are preserved.
    """
    kinds = ['pillars', 'baffles', 'narrow'] if profile == 'mixed' else [profile]
    keep = {(CELLS - 1, 0), (0, CELLS - 1)}                     # start (bottom-left), goal cells
    start = (4 * (CELLS - 1) + 1, 1)
    goal = (1, 4 * (CELLS - 1) + 1)
    added = 0
    for _ in range(count * 40):
        if added >= count:
            break
        kind = rng.choice(kinds)
        i, j = rng.randrange(CELLS), rng.randrange(CELLS)
        if (i, j) in keep:
            continue
        r0, c0 = 4 * i + 1, 4 * j + 1
        if kind == 'pillars':
            cells = [(r0 + rng.choice((0, 2)), c0 + rng.choice((0, 2)))]
        elif kind == 'baffles':
            side = rng.choice('NSEW')
            cells = {'N': [(r0, c0), (r0, c0 + 1)], 'S': [(r0 + 2, c0 + 1), (r0 + 2, c0 + 2)],
                     'W': [(r0, c0), (r0 + 1, c0)], 'E': [(r0 + 1, c0 + 2), (r0 + 2, c0 + 2)]}[side]
        else:
            if rng.random() < 0.5:                              # doorway on the east wall column
                if j + 1 >= CELLS:
                    continue
                col = 4 * j + 4
                if not all(_free(grid, r0 + k, col) for k in range(3)):
                    continue
                cells = [(r0 + rng.choice((0, 2)), col)]
            else:                                               # doorway on the south wall row
                if i + 1 >= CELLS:
                    continue
                row = 4 * i + 4
                if not all(_free(grid, row, c0 + k) for k in range(3)):
                    continue
                cells = [(row, c0 + rng.choice((0, 2)))]
        if not all(_free(grid, r, c) for r, c in cells):
            continue
        for r, c in cells:
            grid[r][c] = '#'
        if _traversable(grid, start, goal):
            added += 1
        else:
            for r, c in cells:
                grid[r][c] = ' '
    return added


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--loops', type=int, default=3)
    ap.add_argument('--profile', choices=PROFILES, default='none',
                    help='obstacle profile added on top of the maze walls')
    ap.add_argument('--obstacles', type=int, default=8, help='number of obstacle blocks')
    a = ap.parse_args()
    rows = generate(a.seed, a.loops, a.profile, a.obstacles)
    with open(a.out, 'w') as f:
        f.write(f'// Generated by tools/gen_maze.py seed={a.seed} loops={a.loops} profile={a.profile} obstacles={a.obstacles}. '
                '0.5 m per character, # = wall, top row = far z (ROS +x).\n')
        f.write('\n'.join(rows) + '\n')


if __name__ == '__main__':
    main()
