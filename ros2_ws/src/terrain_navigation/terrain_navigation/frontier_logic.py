"""Goal-biased frontier selection on an occupancy grid (pure numpy/scipy, no ROS).

Planning only through known free space means the robot can never "shortcut" through
unseen walls. Frontiers (known free cells next to unknown cells) are the only places
it can go to learn more, and every explored dead end loses its frontiers for good,
so the robot cannot cycle back into it. Among the frontiers, the one minimising

    path_length(robot -> frontier) + goal_weight * straight_distance(frontier -> goal)

is chosen, so exploration is pulled toward the goal without ever planning through
unknown space.
"""
import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path

OCC_THRESH = 50          # occupancy >= this is an obstacle
EIGHT = np.ones((3, 3), dtype=bool)


def disk(radius_cells):
    r = int(np.ceil(radius_cells))
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= radius_cells * radius_cells


def safe_cells(grid, clearance_cells, occ_thresh=OCC_THRESH):
    """Known-free cells at least `clearance_cells` away from any obstacle."""
    occ = grid >= occ_thresh
    free = (grid >= 0) & ~occ
    return free & ~ndimage.binary_dilation(occ, structure=disk(clearance_cells))


def path_distance(safe, start):
    """4-connected step count from `start` (row, col) over `safe`; -1 if unreachable.

    Unweighted shortest paths from scipy's C implementation (a pure-Python BFS took 0.15-0.4 s
    per call on a 12 m maze, which left the robot idle between goals)."""
    h, w = safe.shape
    dist = np.full((h, w), -1, dtype=np.int32)
    if not (0 <= start[0] < h and 0 <= start[1] < w):
        return dist
    idx = np.arange(h * w).reshape(h, w)
    s = safe.copy()
    s[start] = True
    right = s[:, :-1] & s[:, 1:]
    down = s[:-1, :] & s[1:, :]
    rows = np.concatenate([idx[:, :-1][right], idx[:-1, :][down]])
    cols = np.concatenate([idx[:, 1:][right], idx[1:, :][down]])
    graph = csr_matrix((np.ones(len(rows), dtype=np.float32), (rows, cols)), shape=(h * w, h * w))
    d = shortest_path(graph, method='D', directed=False, unweighted=True,
                      indices=int(idx[start]))
    reached = np.isfinite(d).reshape(h, w)
    dist[reached] = d.reshape(h, w)[reached].astype(np.int32)
    return dist


def find_frontiers(grid, safe, dist, min_size):
    """Clusters of reachable safe cells that touch unknown space.

    Returns a list of (cells ndarray [N,2], representative (row, col))."""
    unknown = grid < 0
    near_unknown = ndimage.binary_dilation(unknown)          # 4-connected
    mask = safe & (dist >= 0) & near_unknown
    labels, count = ndimage.label(mask, structure=EIGHT)
    clusters = []
    for i in range(1, count + 1):
        cells = np.argwhere(labels == i)
        if len(cells) < min_size:
            continue
        centroid = cells.mean(axis=0)
        rep = tuple(cells[np.argmin(((cells - centroid) ** 2).sum(axis=1))])
        clusters.append((cells, rep))
    return clusters


def score(dist_cells, cell, goal_cell, resolution, goal_weight):
    d_goal = np.hypot(cell[0] - goal_cell[0], cell[1] - goal_cell[1])
    return (dist_cells[cell] + goal_weight * d_goal) * resolution


def best_frontier(grid, robot_cell, goal_cell, resolution, clearance_m=0.30,
                  min_size=3, goal_weight=1.0, blacklist=(), min_clearance_m=0.24,
                  occ_thresh=OCC_THRESH):
    """Returns (rep_cell, score, goal_reachable, dist, safe). rep_cell is None if no frontier.

    If nothing is reachable at `clearance_m` (narrow or partly seen passages), retries with
    less clearance down to `min_clearance_m` (robot radius 0.22 m plus a small margin)."""
    clearance = clearance_m
    while True:
        result = _best_frontier(grid, robot_cell, goal_cell, resolution, clearance,
                                min_size, goal_weight, blacklist, occ_thresh)
        if result[0] is not None or result[2] or clearance <= min_clearance_m:
            return result
        clearance = max(min_clearance_m, clearance - 0.03)


def _best_frontier(grid, robot_cell, goal_cell, resolution, clearance_m, min_size,
                   goal_weight, blacklist, occ_thresh):
    safe = safe_cells(grid, clearance_m / resolution, occ_thresh)
    h, w = safe.shape
    if 0 <= robot_cell[0] < h and 0 <= robot_cell[1] < w:
        # Known-free cells right around the robot are usable even inside the clearance band.
        r = int(np.ceil(clearance_m / resolution))
        rows = slice(max(robot_cell[0] - r, 0), robot_cell[0] + r + 1)
        cols = slice(max(robot_cell[1] - r, 0), robot_cell[1] + r + 1)
        safe[rows, cols] |= (grid[rows, cols] >= 0) & (grid[rows, cols] < occ_thresh)
        safe[robot_cell] = True
    dist = path_distance(safe, robot_cell)
    g_ok = (0 <= goal_cell[0] < h and 0 <= goal_cell[1] < w and dist[goal_cell] >= 0)
    best, best_s = None, np.inf
    for _, rep in find_frontiers(grid, safe, dist, min_size):
        if any(np.hypot(rep[0] - b[0], rep[1] - b[1]) * resolution < 0.6 for b in blacklist):
            continue
        s = score(dist, rep, goal_cell, resolution, goal_weight)
        if s < best_s:
            best, best_s = rep, s
    return best, best_s, bool(g_ok), dist, safe


def escape_cell(grid, cell, clearance_cells, occ_thresh=OCC_THRESH, max_cells=40, avoid=()):
    """Nearest known-free cell with full clearance, for a robot sitting in the obstacle/inflation band.

    best_frontier() finds nothing when the robot's own cell is at lethal cost (it is wedged against a
    wall), so exploration would stop for good. Returns (row, col) or None if nothing is within
    `max_cells`. `avoid` lists (row, col) cells to skip (earlier failed escapes)."""
    safe = safe_cells(grid, clearance_cells, occ_thresh)
    h, w = safe.shape
    if not (0 <= cell[0] < h and 0 <= cell[1] < w) or safe[cell]:
        return None                                       # not stuck
    r0, r1 = max(cell[0] - max_cells, 0), min(cell[0] + max_cells + 1, h)
    c0, c1 = max(cell[1] - max_cells, 0), min(cell[1] + max_cells + 1, w)
    window = safe[r0:r1, c0:c1].copy()
    for a in avoid:
        window[max(a[0] - r0 - 5, 0):max(a[0] - r0 + 6, 0), max(a[1] - c0 - 5, 0):max(a[1] - c0 + 6, 0)] = False
    cells = np.argwhere(window)
    if len(cells) == 0:
        return None
    cells += (r0, c0)
    d = ((cells - np.array(cell)) ** 2).sum(axis=1)
    return tuple(int(v) for v in cells[np.argmin(d)])


def find_frontier_near(grid, safe, dist, cell, radius_cells=10):
    """True if any reachable frontier cell lies within `radius_cells` of `cell`."""
    mask = safe & (dist >= 0) & ndimage.binary_dilation(grid < 0)
    r, c = cell
    return bool(mask[max(r - radius_cells, 0):r + radius_cells + 1,
                     max(c - radius_cells, 0):c + radius_cells + 1].any())
